#include "TextMotionPipeline.h"
#include "TextMotionInference.h"

#include <Eigen/Core>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <random>
#include <set>

namespace legged {
namespace motion {
namespace {
using Part = std::array<float, 3>;
using PartFrames = std::vector<Part>;
const std::array<std::string, 3> kParts{{"head", "left_hand", "right_hand"}};
const std::array<std::size_t, 3> kColumns{{6, 3, 0}};
const std::set<std::string> kHandRandom{"放后面", "前伸", "指向右侧", "指向左侧", "上下摆动", "前后摆动", "斜下举", "张开", "抬起"};
const std::set<std::string> kHandOne{"放后面", "前伸", "指向右侧", "指向左侧", "碰头", "抱胸", "斜下举", "张开", "抬起", "抱肚子"};
const std::set<std::string> kHandTwo{"拍肚子", "拍胸", "挠头", "摸肚子", "弯曲回收", "上下摆动", "前后摆动", "向下环绕"};
const std::set<std::string> kHeadOffset{"点头", "向左倾斜", "向右倾斜", "左右倾斜", "摇头"};
const std::set<std::string> kHeadOne{"抬头", "低头", "向左转头", "向右转头", "向左低头", "向右低头", "向左抬头", "向右抬头", "后仰", "前倾"};
const std::set<std::string> kHeadTwo{"画弧", "画圈", "左右转头"};
float clipped(float value, const std::pair<float, float>& limit) {
  return std::max(limit.first, std::min(value, limit.second));
}
bool validPart(const Part& value, const std::array<std::pair<float, float>, 3>& limits) {
  for (std::size_t j = 0; j < 3; ++j)
    if (!std::isfinite(value[j]) || value[j] < limits[j].first || value[j] > limits[j].second) return false;
  return true;
}
Part partOf(const Pose& pose, std::size_t part) {
  const auto offset = kColumns.at(part);
  return {{pose[offset], pose[offset + 1], pose[offset + 2]}};
}
std::uint32_t little32(const char* bytes) {
  const auto* p = reinterpret_cast<const unsigned char*>(bytes);
  return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
      (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}
Frames resample(const Frames& source) {
  require(!source.empty() && source.size() <= 5000, "Invalid resampling input");
  const float last = static_cast<float>(source.size() - 1);
  const auto count = static_cast<std::size_t>(std::ceil(last / 1.2f)) + 1;
  Frames result(count);
  for (std::size_t i = 0; i < count; ++i) {
    const float grid = static_cast<float>(i) / static_cast<float>(std::max<std::size_t>(count - 1, 1)) * last;
    const auto left = std::min(static_cast<std::size_t>(std::floor(grid)), source.size() - 1);
    const auto right = std::min(left + 1, source.size() - 1);
    const float alpha = grid - static_cast<float>(left);
    for (std::size_t j = 0; j < 9; ++j) result[i][j] = source[left][j] * (1 - alpha) + source[right][j] * alpha;
  }
  return result;
}
void expectedTensor(const Json& tensors, const std::string& name, const std::vector<int>& shape, const std::string& dtype) {
  unsigned count = 0;
  for (const auto& tensor : tensors) if (tensor.at("name") == name) {
    ++count;
    require(tensor.at("shape") == Json(shape) && tensor.at("dtype") == dtype, "Unknown text model ABI: " + name);
  }
  require(count == 1, "Missing/duplicate text model tensor: " + name);
}
}  // namespace

struct TextMotionPipeline::Impl {
  Json info, vocabulary;
  std::vector<std::string> names;
  Limits limits;
  std::array<std::pair<float, float>, 9> model_limits;
  std::array<std::array<std::pair<float, float>, 3>, 3> part_limits;
  std::array<std::vector<std::string>, 3> actions;
  std::array<std::vector<std::vector<std::size_t>>, 3> candidates;
  std::vector<float> atoms;
  std::vector<std::uint32_t> lengths;
  std::size_t padded{0};
  bool half_bridge{false};
  std::shared_ptr<TextInferenceApi> api;
  std::unique_ptr<TextInferenceSession> classifier, inbetweener;
  std::vector<std::int32_t> ids, padding;

  Impl(const std::string& root, const std::vector<std::string>& all_names, const Limits& all_limits) {
    require(all_names.size() == 21 && all_limits.size() == 21, "Text pipeline requires 21-joint mapping");
    names.assign(all_names.begin() + 12, all_names.end());
    limits.assign(all_limits.begin() + 12, all_limits.end());
    const auto manifest = assetPath(root, "manifest.json");
    const auto package = parentPath(manifest);
    info = readJson(manifest);
    require(info.at("format") == "rknn_neural_package_v1" && info.at("target_platform") == "rk3588" &&
            info.at("fps") == 50 && info.at("motion_metadata").at("joint_names") == Json(names) &&
            info.at("trajectory_columns") == Json({"right_hand[0:3]", "left_hand[3:6]", "head[6:9]"}),
            "Unknown text motion coordinates/FPS/package");
    const auto& models = info.at("models");
    require(models.is_object() && models.size() == 2, "Expected exactly two text models");
    const auto& qwen = models.at("qwen_tokenizer");
    const auto& between = models.at("inbetweener");
    require(qwen.at("inputs").size() == 2 && qwen.at("outputs").size() == 4 &&
            between.at("inputs").size() == 3 && between.at("outputs").size() == 2, "Unknown text neural ABI");
    expectedTensor(qwen.at("inputs"), "input_ids", {1, 129}, "int32");
    expectedTensor(qwen.at("inputs"), "padding_mask", {1, 129}, "int32");
    expectedTensor(qwen.at("outputs"), "qwen_embedding", {1, 1024}, "float32");
    expectedTensor(qwen.at("outputs"), "head_logits", {1, 47}, "float32");
    expectedTensor(qwen.at("outputs"), "left_hand_logits", {1, 43}, "float32");
    expectedTensor(qwen.at("outputs"), "right_hand_logits", {1, 43}, "float32");
    expectedTensor(between.at("inputs"), "initial_pose", {1, 9}, "float32");
    expectedTensor(between.at("inputs"), "target_pose", {1, 9}, "float32");
    expectedTensor(between.at("inputs"), "frame_count", {1}, "int32");
    expectedTensor(between.at("outputs"), "motion", {1, 90, 9}, "float32");
    expectedTensor(between.at("outputs"), "output_frame_count", {1}, "int32");
    const auto& transition = info.at("inbetweener");
    require(transition.at("padded_frames") == 90 && transition.at("auto_min_frames") == 1 &&
            transition.at("auto_max_frames") == 40, "Unknown transition frame contract");
    vocabulary = readJson(assetPath(package, info.at("vocabulary").at("file").get<std::string>()));
    require(vocabulary.at("vocabulary") == info.at("motion_metadata").at("vocabulary") &&
            vocabulary.at("max_length") == 128 && vocabulary.at("text_preprocessing") == "identity" &&
            vocabulary.at("token_order") == Json(kParts), "Text vocabulary differs from motion metadata");
    for (const auto& value : vocabulary.at("char_vocab")) integer(value, 0, std::numeric_limits<std::int32_t>::max());
    for (const auto* key : {"<cls>", "<pad>", "<unk>"})
      integer(vocabulary.at("char_vocab").at(key), 0, std::numeric_limits<std::int32_t>::max());
    const auto precision = info.at("motion_metadata").value("host_bridge_input_precision", "float32");
    require(precision == "float16" || precision == "float32", "Unknown bridge precision");
    half_bridge = precision == "float16";
    Pose lower, upper;
    for (std::size_t j = 0; j < 9; ++j) { lower[j] = limits[j].first; upper[j] = limits[j].second; }
    lower = mirrorArms(lower); upper = mirrorArms(upper);
    for (std::size_t j = 0; j < 9; ++j) model_limits[j] = {std::min(lower[j], upper[j]), std::max(lower[j], upper[j])};
    for (std::size_t p = 0; p < 3; ++p)
      for (std::size_t j = 0; j < 3; ++j) part_limits[p][j] = model_limits[kColumns[p] + j];
    loadMaterials(package);
#ifdef USE_RKNN
    const auto board = readFile("/proc/device-tree/compatible", 4096);
    require(board.find("rk3588") != std::string::npos, "RKNN text inference requires RK3588");
    const auto library = ament_index_cpp::get_package_prefix("legged_deploy") + "/lib/librknnrt.so";
    api = std::make_shared<TextInferenceApi>(library);
    constexpr const char* model_key = "file";
#else
    api = std::make_shared<TextInferenceApi>();
    constexpr const char* model_key = "onnx";
#endif
    classifier.reset(new TextInferenceSession(api, assetPath(package, qwen.at(model_key).get<std::string>()), qwen));
    inbetweener.reset(new TextInferenceSession(api, assetPath(package, between.at(model_key).get<std::string>()), between));
    // Keep only the small inference contracts after loading materials.
    auto runtime_models = std::move(info.at("models"));
    auto characters = std::move(vocabulary.at("char_vocab"));
    info = {{"models", std::move(runtime_models)}};
    vocabulary = {{"char_vocab", std::move(characters)}};
    // Initialization/one-time neural warmup completes before the generation service becomes visible.
    const auto warm_until = steadyNs() + 30000000000LL;
    classify("左手抬起", warm_until, [] {});
    Pose zero{};
    predict(zero, zero, warm_until, [] {});
  }

  void loadMaterials(const std::string& package) {
    const auto record_path = assetPath(package, "materials/assets.json");
    const auto record = readJson(record_path);
    require(record.at("format") == "txt2motion_cpp_assets_v1" &&
            record.at("a_pose") == info.at("a_pose"), "Native materials differ from package declaration");
    const auto data = readFile(assetPath(parentPath(record_path), record.at("data").at("file").get<std::string>()), 64 * 1024 * 1024);
    require(data.size() >= 16 && data.compare(0, 8, "T2MAT001") == 0, "Invalid native material header");
    const std::size_t count = little32(data.data() + 8);
    padded = little32(data.data() + 12);
    require(count > 0 && count <= 65536 && padded > 0 && padded <= 5000 &&
            record.at("data").at("materials") == count && record.at("data").at("padded_frames") == padded,
            "Invalid native material dimensions");
    const std::size_t values = count * padded * 3;
    require(values <= (64 * 1024 * 1024 - 16) / 4 && data.size() == 16 + values * 4 + count * 4,
            "Native material file length mismatch");
    atoms.resize(values);
    lengths.resize(count);
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "Materials require IEEE FP32");
    for (std::size_t i = 0; i < values; ++i) {
      const auto bits = little32(data.data() + 16 + i * 4);
      std::memcpy(&atoms[i], &bits, 4);
      require(std::isfinite(atoms[i]), "Non-finite material value");
    }
    for (std::size_t i = 0; i < count; ++i) {
      lengths[i] = little32(data.data() + 16 + values * 4 + i * 4);
      require(lengths[i] >= 1 && lengths[i] <= padded, "Invalid material length");
    }
    for (std::size_t p = 0; p < 3; ++p) {
      const auto& tokens = info.at("motion_metadata").at("vocabulary").at("parts").at(kParts[p]).at("tokens");
      require(tokens.is_array() && tokens.size() == (p == 0 ? 47 : 43), "Invalid class vocabulary");
      actions[p].resize(tokens.size());
      candidates[p].resize(tokens.size());
      candidates[p][0].push_back(0);
      for (std::size_t t = 0; t < tokens.size(); ++t) {
        require(integer(tokens[t].at("id"), 0, tokens.size() - 1) == static_cast<std::int64_t>(t), "Non-contiguous class IDs");
        if (t == 0) { require(tokens[t].at("fields").is_null(), "Token zero must mean NONE"); continue; }
        const auto action = tokens[t].at("fields").at("动作").get<std::string>();
        require(p == 0 ? (kHeadOffset.count(action) || kHeadOne.count(action) || kHeadTwo.count(action)) :
                (kHandOne.count(action) || kHandTwo.count(action)), "Unknown material action route");
        actions[p][t] = action;
      }
    }
    std::set<std::size_t> seen{0};
    const auto& materials = info.at("motion_metadata").at("source").at("materials");
    require(materials.is_array() && !materials.empty() && materials[0].at("id") == 0, "Invalid material catalog");
    for (std::size_t i = 1; i < materials.size(); ++i) {
      const auto& material = materials[i];
      const auto found = std::find(kParts.begin(), kParts.end(), material.at("part").get<std::string>());
      require(found != kParts.end(), "Unknown material body part");
      const auto p = static_cast<std::size_t>(found - kParts.begin());
      const auto token = static_cast<std::size_t>(integer(material.at("token_id"), 1, actions[p].size() - 1));
      const auto id = static_cast<std::size_t>(integer(material.at("id"), 1, count - 1));
      require(seen.insert(id).second && material.at("fields") ==
          info.at("motion_metadata").at("vocabulary").at("parts").at(kParts[p]).at("tokens")[token].at("fields"),
          "Duplicate material or mismatched label");
      candidates[p][token].push_back(id);
    }
  }

  std::array<int, 3> classify(const std::string& text, std::int64_t deadline, const std::function<void()>& checkpoint) {
    validateText(text);
    const auto& vocab = vocabulary.at("char_vocab");
    ids.assign(129, vocab.at("<pad>").get<std::int32_t>());
    padding.assign(129, 1);
    ids[0] = vocab.at("<cls>").get<std::int32_t>(); padding[0] = 0;
    const auto characters = textCharacters(text);
    for (std::size_t i = 0; i < characters.size(); ++i) {
      const auto found = vocab.find(characters[i]);
      ids[i + 1] = found == vocab.end() ? vocab.at("<unk>").get<std::int32_t>() : found->get<std::int32_t>();
      padding[i + 1] = 0;
    }
    // Manifest order is checked explicitly: the inference session maps these inputs by name.
    std::vector<TextInferenceSession::Input> inputs;
    for (const auto& spec : info.at("models").at("qwen_tokenizer").at("inputs"))
      inputs.push_back({spec.at("name") == "input_ids" ? ids.data() : padding.data(), 129, kTextInt32});
    classifier->run(inputs, deadline, checkpoint);
    std::array<int, 3> result;
    for (std::size_t p = 0; p < 3; ++p) {
      const auto& logits = classifier->output(kParts[p] + "_logits");
      require(logits.size() == actions[p].size(), "Classifier vocabulary shape mismatch");
      result[p] = static_cast<int>(std::max_element(logits.begin(), logits.end()) - logits.begin());
      require(!candidates[p][result[p]].empty(), "Classifier selected an unavailable action token");
    }
    return result;
  }

  Frames predict(const Pose& first, const Pose& last, std::int64_t deadline, const std::function<void()>& checkpoint) {
    const std::int32_t auto_frames = -1;
    std::vector<TextInferenceSession::Input> inputs;
    for (const auto& spec : info.at("models").at("inbetweener").at("inputs")) {
      const auto name = spec.at("name").get<std::string>();
      if (name == "frame_count") inputs.push_back({&auto_frames, 1, kTextInt32});
      else inputs.push_back({name == "initial_pose" ? first.data() : last.data(), 9, kTextFloat32});
    }
    inbetweener->run(inputs, deadline, checkpoint);
    const auto& predicted = inbetweener->output("output_frame_count");
    require(predicted.size() == 1 && predicted[0] == std::round(predicted[0]) && predicted[0] >= 1 && predicted[0] <= 40,
            "Invalid automatic inbetweener frame count");
    const auto count = static_cast<std::size_t>(predicted[0]);
    const auto& motion = inbetweener->output("motion");
    require(motion.size() == 90 * 9, "Invalid inbetweener trajectory dimensions");
    Frames result(count);
    for (std::size_t i = 0; i < count; ++i) std::copy_n(motion.begin() + i * 9, 9, result[i].begin());
    return result;
  }

  void augment(PartFrames& source, std::size_t p, const std::array<float, 2>& fraction) const {
    constexpr float maximum = 0.17453292519943295f;
    for (std::size_t j = 0; j < 2; ++j) {
      const float first = source[0][j];
      float low_delta = 0, high_delta = 0;
      for (const auto& frame : source) { low_delta = std::min(low_delta, frame[j] - first); high_delta = std::max(high_delta, frame[j] - first); }
      const float lower = std::max(part_limits[p][j].first + 0.02f - low_delta, first - maximum);
      const float upper = std::min(part_limits[p][j].second - 0.02f - high_delta, first + maximum);
      require(lower <= upper, "No feasible shoulder offset for selected material");
      const float offset = lower + fraction[j] * (upper - lower) - first;
      for (auto& frame : source) frame[j] += offset;
    }
  }

  PartFrames headMotion(PartFrames source, const Part& first, const std::string& action) const {
    const auto origin = source.front();
    if (action == "点头") {
      PartFrames kept;
      for (const auto& frame : source) {
        Part candidate;
        for (std::size_t j = 0; j < 3; ++j) candidate[j] = first[j] + (frame[j] - origin[j]);
        if (validPart(candidate, part_limits[0])) kept.push_back(frame);
      }
      require(validPart(first, part_limits[0]) && !kept.empty(), "Nod starts outside head limits");
      source = std::move(kept);
    } else if (action == "摇头") {
      bool overflow = false;
      for (const auto& frame : source) {
        Part candidate;
        for (std::size_t j = 0; j < 3; ++j) candidate[j] = first[j] + (frame[j] - origin[j]);
        overflow = overflow || !validPart(candidate, part_limits[0]);
      }
      if (overflow) {
        PartFrames candidate = source, kept;
        for (auto& frame : candidate) for (std::size_t j = 0; j < 3; ++j) frame[j] = first[j] - (frame[j] - origin[j]);
        require(validPart(candidate[0], part_limits[0]), "Shake starts outside head limits");
        kept.push_back(candidate[0]);
        for (std::size_t i = 1; i < candidate.size(); ++i) {
          if (validPart(candidate[i], part_limits[0])) kept.push_back(candidate[i]);
          else if (validPart(candidate[i - 1], part_limits[0])) {
            double fraction = 1.0;
            for (std::size_t j = 0; j < 3; ++j) {
              const double delta = static_cast<double>(candidate[i][j]) - candidate[i - 1][j];
              if (delta > 0) fraction = std::min(fraction, (static_cast<double>(part_limits[0][j].second) - candidate[i - 1][j]) / delta);
              if (delta < 0) fraction = std::min(fraction, (static_cast<double>(part_limits[0][j].first) - candidate[i - 1][j]) / delta);
            }
            Part boundary;
            bool different = false;
            for (std::size_t j = 0; j < 3; ++j) {
              boundary[j] = clipped(static_cast<float>(candidate[i - 1][j] + fraction *
                  (static_cast<double>(candidate[i][j]) - candidate[i - 1][j])), part_limits[0][j]);
              different = different || std::abs(boundary[j] - candidate[i - 1][j]) > 1e-7f;
            }
            if (different) kept.push_back(boundary);
          }
        }
        source = std::move(kept);
      }
    }
    Part accumulator{};
    PartFrames result(source.size());
    for (std::size_t i = 0; i < source.size(); ++i)
      for (std::size_t j = 0; j < 3; ++j) {
        if (i) accumulator[j] += source[i][j] - source[i - 1][j];
        result[i][j] = clipped(first[j] + accumulator[j], part_limits[0][j]);
      }
    return result;
  }

  PartFrames bridge(PartFrames source, Part first, std::size_t p, bool crop, int& transition_frames,
                    std::int64_t deadline, const std::function<void()>& checkpoint) {
    const Part original = first;
    if (half_bridge) {
      for (auto& frame : source) for (auto& value : frame) value = static_cast<float>(Eigen::half(value));
      for (auto& value : first) value = static_cast<float>(Eigen::half(value));
    }
    std::size_t cut = 0;
    if (crop) {
      float best = std::numeric_limits<float>::infinity();
      for (std::size_t i = 0; i < (source.size() + 1) / 2; ++i) {
        const float a = source[i][0] - first[0], b = source[i][1] - first[1], c = source[i][2] - first[2];
        const float distance = (a*a + b*b) + c*c;
        if (distance < best) { best = distance; cut = i; }
      }
    }
    const auto& target = crop ? source[cut] : source.back();
    Pose first9{}, last9{};
    const std::size_t column = p == 0 ? 6 : 3;
    for (std::size_t j = 0; j < 3; ++j) {
      const float sign = p == 2 && j != 0 ? -1.0f : 1.0f;
      first9[column + j] = first[j] * sign;
      last9[column + j] = target[j] * sign;
    }
    const auto predicted = predict(first9, last9, deadline, checkpoint);
    PartFrames result;
    result.reserve(predicted.size() + source.size() + 1);
    for (const auto& frame : predicted) {
      Part value;
      for (std::size_t j = 0; j < 3; ++j) value[j] = frame[column + j] * (p == 2 && j != 0 ? -1.0f : 1.0f);
      result.push_back(value);
    }
    transition_frames = predicted.size();
    if (crop) result.insert(result.end(), source.begin() + cut + 1, source.end());
    for (auto& frame : result) for (std::size_t j = 0; j < 3; ++j) frame[j] = clipped(frame[j], part_limits[p][j]);
    if (transition_frames == 1 && result[0] != original) { result.insert(result.begin(), original); ++transition_frames; }
    return result;
  }

  Json generate(const Json& request, const Json& snapshot, std::int64_t deadline, const std::function<void()>& checkpoint) {
    const auto started = steadyNs();
    const auto check = [&] { checkpoint(); checkDeadline(deadline); };
    check();
    const auto tokens = classify(request.at("text").get<std::string>(), deadline, check);
    if (tokens == std::array<int, 3>{{0, 0, 0}}) return nullptr;
    const auto seed = static_cast<std::uint32_t>(integer(request.at("seed"), 1, 4294967295LL));
    std::mt19937 rng(seed);
    std::array<std::uint32_t, 3> selector;
    std::array<std::array<float, 2>, 3> fractions;
    for (auto& value : selector) value = static_cast<std::uint32_t>(rng()) % 2147483647U;
    for (auto& pair : fractions) for (auto& value : pair) value = static_cast<float>(rng() >> 8) * (1.0f / 16777216.0f);
    Pose initial;
    for (std::size_t j = 0; j < 9; ++j) {
      const float value = finiteNumber(snapshot.at("joint_positions")[12 + j]);
      require(std::isfinite(value) && value >= limits[j].first - 1e-6 && value <= limits[j].second + 1e-6, "Invalid generation start pose");
      initial[j] = static_cast<float>(std::max(limits[j].first, std::min(static_cast<double>(value), limits[j].second)));
    }
    const auto model_initial = mirrorArms(initial);
    std::array<PartFrames, 3> parts;
    std::array<std::size_t, 3> material_ids;
    std::array<int, 3> transitions{{1, 1, 1}};
    for (std::size_t p = 0; p < 3; ++p) {
      check();
      const auto& choices = candidates[p].at(tokens[p]);
      const auto material = choices[selector[p] % choices.size()];
      material_ids[p] = material;
      const auto first = partOf(model_initial, p);
      if (tokens[p] == 0) { parts[p].push_back(first); continue; }
      PartFrames source(lengths.at(material));
      for (std::size_t i = 0; i < source.size(); ++i)
        std::copy_n(atoms.begin() + (material * padded + i) * 3, 3, source[i].begin());
      const auto& action = actions[p].at(tokens[p]);
      if (p > 0 && kHandRandom.count(action)) augment(source, p, fractions[p]);
      if (p == 0 && kHeadOffset.count(action)) parts[p] = headMotion(std::move(source), first, action);
      else parts[p] = bridge(std::move(source), first, p, p == 0 ? kHeadTwo.count(action) : kHandTwo.count(action),
                             transitions[p], deadline, check);
      require(!parts[p].empty(), "Empty body-part trajectory");
    }
    const auto action_count = std::max({parts[0].size(), parts[1].size(), parts[2].size()});
    Frames motion(action_count);
    for (std::size_t i = 0; i < action_count; ++i)
      for (std::size_t p = 0; p < 3; ++p) {
        const auto& value = parts[p][std::min(i, parts[p].size() - 1)];
        std::copy(value.begin(), value.end(), motion[i].begin() + kColumns[p]);
      }
    auto home = predict(motion.back(), model_initial, deadline, check);
    for (auto& frame : home) for (std::size_t j = 0; j < 9; ++j) {
      frame[j] = clipped(frame[j], model_limits[j]);
      if (tokens[j < 3 ? 2 : (j < 6 ? 1 : 0)] == 0) frame[j] = motion.back()[j];
    }
    if (home.size() == 1) home.insert(home.begin(), motion.back());
    // The return resampling starts at the actual final action pose, as in the source composer.
    home[0] = motion.back();
    const auto before_speed = motion.size() + home.size() - 1;
    auto action = resample(motion);
    auto returning = resample(home);
    const auto action_after_speed = action.size();
    action.reserve(action.size() + 25 + returning.size() - 1);
    const auto held = action.back();
    action.insert(action.end(), 25, held);
    const auto seam = action.size() - 1;
    action.insert(action.end(), returning.begin() + 1, returning.end());
    require(action.size() >= 2 && action.size() <= 5000, "Complete trajectory exceeds frame limits");
    Json adjustments = Json::object(), positions = Json::array(), generation_limits = Json::object();
    positions.get_ref<Json::array_t&>().reserve(action.size());
    std::array<int, 9> changes{};
    std::array<double, 9> maxima{};
    for (std::size_t i = 0; i < action.size(); ++i) {
      if (i % 64 == 0) check();
      auto pose = mirrorArms(action[i]);
      for (std::size_t j = 0; j < 9; ++j) {
        const double value = pose[j];
        require(std::isfinite(value), "Non-finite generated trajectory");
        const double bounded = std::max(limits[j].first, std::min(value, limits[j].second));
        const double adjustment = std::abs(bounded - value);
        require(adjustment <= 1e-6, "Generated trajectory exceeds robot limits: " + names[j]);
        changes[j] += adjustment > 1e-6;
        maxima[j] = std::max(maxima[j], adjustment);
        pose[j] = bounded;
        const bool active = tokens[j < 3 ? 1 : (j < 6 ? 2 : 0)] != 0;
        if (i == 0 || i + 1 == action.size() || !active)
          require(std::abs(pose[j] - initial[j]) <= 0.005f, "Generated endpoint/inactive joint differs from captured pose");
      }
      positions.push_back(pose);
    }
    for (std::size_t j = 0; j < 9; ++j) {
      adjustments[names[j]] = {{"count", changes[j]}, {"max_rad", maxima[j]}};
      generation_limits[names[j]] = {limits[j].first, limits[j].second};
    }
    Json active = Json::array();
    for (std::size_t j = 0; j < 9; ++j) active.push_back(tokens[j < 3 ? 1 : (j < 6 ? 2 : 0)] != 0);
    check();
    return {{"format", "l0_text_motion_v2"}, {"robot", "l0_v5"}, {"angle_unit", "rad"}, {"fps", 50},
        {"coordinate_contract", "robot_anatomical_v1"}, {"request_id", request.at("request_id")},
        {"joint_names", names}, {"active_joints", std::move(active)},
        {"positions", std::move(positions)}, {"frame_count", action.size()}, {"token_ids", tokens},
        {"material_ids", material_ids}, {"backend", kTextBackend}, {"inference_ms", (steadyNs() - started) * 1e-6},
        {"generation_request", request}, {"start_pose", initial}, {"end_pose", initial}, {"limit_policy", "reject"},
        {"limit_adjustments", std::move(adjustments)}, {"model_diagnostics", {
            {"generation_joint_limits_rad", std::move(generation_limits)}, {"part_transition_frames", transitions},
            {"action_frame_count_before_speed", {action_count}}, {"frame_count_before_speed", {before_speed}},
            {"action_frame_count", {action_after_speed}}, {"return_home_hold_frames", {25}},
            {"return_home_start_frame", {seam}}, {"selector", selector}}}};
  }
};

TextMotionPipeline::TextMotionPipeline(const std::string& root, const std::vector<std::string>& names, const Limits& limits)
    : impl_(new Impl(root, names, limits)) {}
TextMotionPipeline::~TextMotionPipeline() = default;
Json TextMotionPipeline::generate(const Json& request, const Json& snapshot, std::int64_t deadline,
                                  const std::function<void()>& checkpoint) {
  require(retired(), "Previous text inference has not retired");
  return impl_->generate(request, snapshot, deadline, checkpoint);
}
bool TextMotionPipeline::retired() const {
  return !impl_->api->uncertain.load() && impl_->classifier->retired() && impl_->inbetweener->retired();
}
}  // namespace motion
}  // namespace legged
