#include "wbc_io_probe.h"

#include "rknn_model.h"
#include "wbc_lora_assets.h"
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;

double elapsedMs(const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void writeYaml(const fs::path& path, const YAML::Node& value) {
  YAML::Emitter emitter;
  emitter << value;
  std::ofstream stream(path);
  stream << emitter.c_str() << '\n';
  stream.close();
  if (!emitter.good() || !stream) throw std::runtime_error("Cannot save " + path.string());
}

std::vector<float> readFloats(const fs::path& path, std::size_t count) {
  if (!fs::is_regular_file(path) || fs::file_size(path) != count * sizeof(float))
    throw std::runtime_error("Wrong input byte count: " + path.string());
  std::vector<float> values(count);
  std::ifstream stream(path, std::ios::binary);
  if (!stream.read(reinterpret_cast<char*>(values.data()), count * sizeof(float)) ||
      !std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); }))
    throw std::runtime_error("Invalid/non-finite input: " + path.string());
  return values;
}

void saveRuntimeMaps(const fs::path& output) {
  std::ifstream maps("/proc/self/maps");
  std::ofstream stream(output / "runtime_maps.txt");
  std::string line;
  while (std::getline(maps, line)) {
    if (line.find("rknn") != std::string::npos) stream << line << '\n';
  }
}
}  // namespace

int runWbcIoProbe(int argc, char** argv, const volatile sig_atomic_t& stop_requested) {
  if (argc != 5) {
    std::cerr << "Usage: rl_controller --wbc-io-probe MANIFEST CASE_DIRECTORY NEW_OUTPUT_DIRECTORY\n";
    return 2;
  }
  const fs::path output(argv[4]);
  YAML::Node report;
  report["format"] = "wbc_resident_io_probe_v1";
  report["status"] = "initializing";
  report["completed_samples"] = 0;
  bool owns_output = false;
  try {
    // Do not overwrite a previous run, including failed or interrupted evidence.
    if (!fs::create_directory(output)) throw std::runtime_error("Output directory already exists");
    owns_output = true;
    writeYaml(output / "runtime.yaml", report);
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "IEEE float32 required");
    const std::uint16_t endian = 1;
    if (*reinterpret_cast<const unsigned char*>(&endian) != 1)
      throw std::runtime_error("Probe binary format requires little-endian host");
    const fs::path cases_root(argv[3]);
    const auto request = YAML::LoadFile((cases_root / "cases.json").string());
    const auto cases = request["cases"];
    if (request["format"].as<std::string>() != "wbc_policy_io_cases_v1" ||
        !cases.IsSequence() || cases.size() == 0 || cases.size() > 8192)
      throw std::runtime_error("Invalid probe case format/count");
    const auto policy = readFloats(cases_root / "policy.f32", cases.size() * 399);
    const auto command = readFloats(cases_root / "command_short.f32", cases.size() * 240);
    std::vector<std::string> selected;
    for (const auto& item : cases) {
      const auto id = item["action_id"].as<std::string>();
      if (std::find(selected.begin(), selected.end(), id) == selected.end()) selected.push_back(id);
    }

    // These are the SAME loader, native tensor buffers and SDK calls used by RLWbc.
    const auto initialized = std::chrono::steady_clock::now();
    WbcLoraAssets assets(argv[2], selected);
    RknnModel model(assets.modelPath(), RknnModel::InputMode::ResidentWbc);
    assets.bindModel(model);
    std::array<WbcLoraAssets::Buffer, 2> buffers;
    for (auto& buffer : buffers) assets.allocate(buffer);
    report["initialization_ms"] = elapsedMs(initialized);
    report["model_path"] = assets.modelPath();
    report["base_checkpoint_sha256"] = assets.baseHash();
    report["sdk_version"] = model.runtimeVersion();
    report["execution_path"] = "WbcLoraAssets + RknnModel::ResidentWbc (same as RLWbc)";
    report["scope"] = "policy inference only; fixed policy/command_short; no reference queries, ROS or motor output";
    report["requested_samples"] = cases.size();
    report["status"] = "running";
    writeYaml(output / "runtime.yaml", report);
    saveRuntimeMaps(output);
    std::ofstream layout(output / "resident_layout.txt");
    layout << model.residentLayout() << '\n';
    layout.close();
    if (!layout) throw std::runtime_error("Cannot save resident layout");
    std::cout << "WBC IO probe: " << model.runtimeVersion() << "; samples=" << cases.size() << std::endl;
    std::ofstream values(output / "actions.f32", std::ios::binary);
    std::ofstream frames(output / "frames.csv");
    frames << "case_index,action_id,slot,generation,inference_ms,runs,parameter_prepares,parameter_binds,parameter_syncs,observation_syncs,output_syncs,observation_ms_total,execution_ms_total,output_ms_total\n";
    frames << std::setprecision(17);
    int active_slot = 0;
    std::uint64_t generation = 0;
    std::string active_action;
    for (std::size_t i = 0; i < cases.size(); ++i) {
      if (stop_requested) throw std::runtime_error("Probe interrupted; completed outputs retained");
      const auto id = cases[i]["action_id"].as<std::string>();
      if (id != active_action) {
        const auto load_started = std::chrono::steady_clock::now();
        const int slot = 1 - active_slot;
        assets.loadAdapter(assets.action(id), buffers[slot]);
        model.stageResidentParameters(slot, assets.parameterInputs(buffers[slot]));
        if (!model.commitResidentParameters(slot)) throw std::runtime_error(model.lastError());
        active_slot = slot;
        active_action = id;
        ++generation;
        YAML::Node change;
        change["case_index"] = i;
        change["action_id"] = id;
        change["generation"] = generation;
        change["slot"] = slot;
        change["load_pack_commit_ms"] = elapsedMs(load_started);
        change["adapter_manifest_sha256"] = assets.action(id).adapter_hash;
        report["switches"].push_back(change);
        std::cout << "prepare action=" << id << " generation=" << generation << " slot=" << slot << std::endl;
      }
      const auto started = std::chrono::steady_clock::now();
      if (!model.runResident(policy.data() + i * 399, 399, command.data() + i * 240, 240))
        throw std::runtime_error("case " + std::to_string(i) + ": " + model.lastError());
      const double inference_ms = elapsedMs(started);
      const auto& action = model.output(0);
      if (action.size() != 21 || !std::all_of(action.begin(), action.end(), [](float x) { return std::isfinite(x); }))
        throw std::runtime_error("Invalid action output");
      // Save raw network output BEFORE clipping/action scaling, exactly as returned.
      values.write(reinterpret_cast<const char*>(action.data()), 21 * sizeof(float));
      values.flush();
      const auto stats = model.residentStats();
      frames << i << ',' << id << ',' << active_slot << ',' << generation << ',' << inference_ms << ','
             << stats.runs << ',' << stats.parameter_prepares << ',' << stats.parameter_bind_calls << ','
             << stats.parameter_sync_calls << ',' << stats.observation_sync_calls << ',' << stats.output_sync_calls << ','
             << stats.observation_ms << ',' << stats.execution_ms << ',' << stats.output_ms << '\n';
      frames.flush();
      if (!values || !frames) throw std::runtime_error("Cannot save output; disk/write failure");
      report["completed_samples"] = i + 1;
    }
    const auto stats = model.residentStats();
    report["context_initializations"] = stats.context_initializations;
    report["parameter_prepares"] = stats.parameter_prepares;
    report["commits"] = stats.commits;
    report["rollbacks"] = stats.rollbacks;
    report["runs"] = stats.runs;
    report["allocated_bytes"] = stats.allocated_bytes;
    report["status"] = "completed";
    writeYaml(output / "runtime.yaml", report);
    std::cout << "Completed " << cases.size() << " samples using one resident context" << std::endl;
    return 0;
  } catch (const std::exception& error) {
    report["status"] = "failed";
    report["error"] = error.what();
    if (owns_output) {
      try {
        saveRuntimeMaps(output);
        writeYaml(output / "runtime.yaml", report);
      } catch (...) {}  // Preserve the original runtime failure in stderr.
    }
    std::cerr << "WBC IO probe failed: " << error.what() << std::endl;
    return 1;
  }
}
