#include <gio/gio.h>

#include <chrono>
#include <cstdio>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "AppVersion.h"

namespace {
constexpr char kNetworkManager[] = "org.freedesktop.NetworkManager";
constexpr char kNetworkManagerPath[] = "/org/freedesktop/NetworkManager";

// Both states have a default route; broker reachability is checked by the cloud bridge.
bool hasNetworkRoute(guint32 state) {
  return state == 60 || state == 70;  // NM_STATE_CONNECTED_SITE / NM_STATE_CONNECTED_GLOBAL
}
}  // namespace

class NetworkReadyGate : public rclcpp::Node {
 public:
  NetworkReadyGate() : Node("luvbot_network_ready_gate") {
    result_sub_ = create_subscription<std_msgs::msg::String>(
        "/luvbot/provision_result", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          if (message->data == "ok") markReady("BLE provisioning succeeded");
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(250), [this]() {
      // Dispatch local D-Bus notifications; this does not probe DNS or the MQTT server.
      while (g_main_context_pending(nullptr)) g_main_context_iteration(nullptr, FALSE);
      if (ready_) rclcpp::shutdown();
    });
    watchNetwork();
    if (!ready_) {
      RCLCPP_INFO(get_logger(), "No usable network yet; waiting for NetworkManager or BLE success. Cloud bridge is not started.");
    }
  }

  ~NetworkReadyGate() override {
    if (connection_) {
      g_dbus_connection_signal_unsubscribe(connection_, signal_id_);
      g_object_unref(connection_);
    }
  }

  bool ready() const { return ready_; }

 private:
  void markReady(const char *reason) {
    if (ready_) return;
    ready_ = true;
    RCLCPP_INFO(get_logger(), "Network ready: %s; allowing cloud bridge to start", reason);
  }

  void watchNetwork() {
    GError *error = nullptr;
    connection_ = g_bus_get_sync(G_BUS_TYPE_SYSTEM, nullptr, &error);
    if (!connection_) {
      RCLCPP_WARN(get_logger(), "Cannot monitor NetworkManager: %s; waiting for BLE success", error->message);
      g_clear_error(&error);
      return;
    }
    // Subscribe before reading the initial state so autoconnect cannot be missed between them.
    signal_id_ = g_dbus_connection_signal_subscribe(
        connection_, kNetworkManager, kNetworkManager, "StateChanged", kNetworkManagerPath,
        nullptr, G_DBUS_SIGNAL_FLAGS_NONE,
        [](GDBusConnection *, const gchar *, const gchar *, const gchar *, const gchar *,
           GVariant *parameters, gpointer data) {
          if (!g_variant_is_of_type(parameters, G_VARIANT_TYPE("(u)"))) return;
          guint32 state = 0;
          g_variant_get(parameters, "(u)", &state);
          if (hasNetworkRoute(state)) {
            static_cast<NetworkReadyGate *>(data)->markReady("NetworkManager has a default route");
          }
        }, this, nullptr);
    GVariant *reply = g_dbus_connection_call_sync(
        connection_, kNetworkManager, kNetworkManagerPath, "org.freedesktop.DBus.Properties", "Get",
        g_variant_new("(ss)", kNetworkManager, "State"), G_VARIANT_TYPE("(v)"),
        G_DBUS_CALL_FLAGS_NO_AUTO_START, 2000, nullptr, &error);
    if (!reply) {
      RCLCPP_WARN(get_logger(), "Cannot read initial network state: %s; waiting for state changes or BLE success", error->message);
      g_clear_error(&error);
      return;
    }
    GVariant *state = nullptr;
    g_variant_get(reply, "(v)", &state);
    if (g_variant_is_of_type(state, G_VARIANT_TYPE_UINT32) && hasNetworkRoute(g_variant_get_uint32(state))) {
      markReady("network already connected at startup");
    }
    g_variant_unref(state);
    g_variant_unref(reply);
  }

  bool ready_{false};
  GDBusConnection *connection_{nullptr};
  guint signal_id_{0};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  l0::printAppVersion("network_ready_gate");
  rclcpp::init(argc, argv);
  int code = 1;
  try {
    auto node = std::make_shared<NetworkReadyGate>();
    rclcpp::spin(node);
    code = node->ready() ? 0 : 1;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "network_ready_gate: %s\n", error.what());
  }
  rclcpp::shutdown();
  return code;
}
