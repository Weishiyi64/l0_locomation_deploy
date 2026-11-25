// Planner input adapter. The original metric depth message remains untouched.
#include <algorithm>
#include <cmath>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>

class PlannerDepthPreprocessor : public rclcpp::Node {
 public:
  PlannerDepthPreprocessor() : Node("planner_depth_preprocessor") {
    const auto topic =
        declare_parameter<std::string>("depth_topic", "/manifold/ODIN2/device0/pointcloud2depth/depth_sru");
    max_depth_ = declare_parameter<double>("max_depth_m", 5.0);
    if (!std::isfinite(max_depth_) || max_depth_ <= 0)
      throw std::invalid_argument("max_depth_m must be finite and positive");
    publisher_ = create_publisher<sensor_msgs::msg::Image>(
        declare_parameter<std::string>("output_topic", "/planner/depth_mono8"), 1);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
        topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::Image::ConstSharedPtr image) { convert(*image); });
  }

 private:
  void convert(const sensor_msgs::msg::Image& input) {
    if (publisher_->get_subscription_count() == 0U) return;
    if (input.encoding != "32FC1" || input.width == 0 || input.height == 0 ||
        input.step < static_cast<size_t>(input.width) * 4 ||
        input.data.size() < static_cast<size_t>(input.step) * input.height)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid 32FC1 depth layout");
      return;
    }
    sensor_msgs::msg::Image output;
    output.header = input.header;
    output.width = input.width;
    output.height = input.height;
    output.encoding = "mono8";
    output.step = input.width;
    output.is_bigendian = false;
    output.data.resize(static_cast<size_t>(output.step) * output.height);
    const uint16_t endian_test = 1;
    const bool host_bigendian = *reinterpret_cast<const uint8_t*>(&endian_test) == 0;
    for (size_t y = 0; y < input.height; ++y)
    {
      for (size_t x = 0; x < input.width; ++x)
      {
        uint8_t bytes[4];
        std::memcpy(bytes, input.data.data() + y * input.step + x * 4, 4);
        if (bool(input.is_bigendian) != host_bigendian) std::reverse(bytes, bytes + 4);
        float depth;
        std::memcpy(&depth, bytes, 4);
        // Invalid=black. Valid ranges use 1..255, with max_depth_m as the white point.
        output.data[y * output.step + x] =
            std::isfinite(depth) && depth > 0
                ? static_cast<uint8_t>(1 + std::lround(254 * std::min(1.0, depth / max_depth_)))
                : 0;
      }
    }
    publisher_->publish(output);
  }
  double max_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerDepthPreprocessor>());
  rclcpp::shutdown();
  return 0;
}
