// Test-only interposer for a DDS publication failure and a worker exception.
#include <rcl/error_handling.h>
#include <rcl/publisher.h>
#include <std_msgs/msg/string.hpp>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <stdexcept>

extern "C" rcl_ret_t rcl_publish(
  const rcl_publisher_t * publisher, const void * message, rmw_publisher_allocation_t * allocation)
{
  const char * fault = std::getenv("HPM_DISPLAY_TEST_FAULT");
  const char * topic = rcl_publisher_get_topic_name(publisher);
  if (fault && topic && std::strcmp(topic, "/display/connection/state") == 0) {
    const auto & state = static_cast<const std_msgs::msg::String *>(message)->data;
    if (state == "online" && std::strcmp(fault, "worker") == 0) {
      throw std::runtime_error("injected display worker exception");
    }
    if (state == "offline" && std::strcmp(fault, "publish") == 0) {
      RCL_SET_ERROR_MSG("injected display ROS publication failure");
      return RCL_RET_ERROR;
    }
  }
  static const auto publish = reinterpret_cast<decltype(&rcl_publish)>(
    dlsym(RTLD_NEXT, "rcl_publish"));
  return publish(publisher, message, allocation);
}
