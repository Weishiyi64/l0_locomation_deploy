#include <serial/serial.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "analysis_data.h"
#include "yesense_imu/msg/yesense_imu_all_data.hpp"
#include "yesense_imu/msg/yesense_imu_cmd_resp.hpp"
#include "yesense_imu/msg/yesense_imu_gnss_data.hpp"
#include "yesense_imu/msg/yesense_imu_gps_data.hpp"
#include "yesense_imu/msg/yesense_imu_inertial_data.hpp"
#include "yesense_imu/msg/yesense_imu_nav_data.hpp"
#include "yesense_imu/msg/yesense_imu_sensor_data.hpp"
#include "yesense_imu/msg/yesense_imu_status.hpp"

#define ENBALE_DEBUG_OUTPUT 1
#define ENABLE_SERIAL_INPUT 1

#define DATA_BUF_SIZE 1024

namespace yesense {

const uint8_t MODE_HEADER1 = 0;
const uint8_t MODE_HEADER2 = 1;
const uint8_t MODE_TID_L = 2;
const uint8_t MODE_TID_H = 3;
const uint8_t MODE_LENGTH = 4;
const uint8_t MODE_MESSAGE = 5;
const uint8_t MODE_CHECKSUM_L = 6;
const uint8_t MODE_CHECKSUM_H = 7;
const uint8_t MODE_GPS_RAW = 10;

const uint8_t PORDUCTION_INFO = 0x00;
const uint8_t BAUDRATE = 0x02;
const uint8_t OUTPUT_FREEQUENCY = 0x03;
const uint8_t OUTPUT_CONTENT = 0x04;
const uint8_t STANDARD_PARAM = 0x05;
const uint8_t MODE_SETTING = 0x4D;

typedef enum _Id { QUERY_STATUS = 0x00, CONFIG_MEMERY = 0x01, CONFIG_FLASH = 0x02 } ID;

#pragma pack(1)
typedef struct {
  uint8_t type;
  uint8_t len;
} yis_tlv_header_t;
#pragma pack()

class YesenseDriver {
 public:
  explicit YesenseDriver(rclcpp::Node::SharedPtr node);
  ~YesenseDriver();

  void run();
  void setParam();

  void publish_imu(const protocol_info_t &imu_data);

 protected:
  void initSerial();
  void _spin();
  void spin();

  void update_position_by_gps(const protocol_info_t &imu_data, geometry_msgs::msg::PoseStamped &pose);

  void onProductionInformationQuery(const std_msgs::msg::Int8::ConstSharedPtr msg);
  void onResetAllParam(const std_msgs::msg::UInt8::ConstSharedPtr msg);
  void onBaudrateQuery(const std_msgs::msg::Empty::ConstSharedPtr msg);
  void onBaudrateSetting(const std_msgs::msg::UInt8::ConstSharedPtr msg);
  void onFrequencyQuery(const std_msgs::msg::Empty::ConstSharedPtr msg);
  void onFrequencySetting(const std_msgs::msg::UInt8::ConstSharedPtr msg);
  void onOutputContentQuery(const std_msgs::msg::Empty::ConstSharedPtr msg);
  void onOutputContentSetting(const std_msgs::msg::UInt8::ConstSharedPtr msg);
  void onStandardParamQuery(const std_msgs::msg::UInt8::ConstSharedPtr msg);
  void onStandardParamSetting(const std_msgs::msg::UInt8::ConstSharedPtr msg);
  void onModeSettingQuery(const std_msgs::msg::UInt8::ConstSharedPtr msg);
  void onModeSettingSetting(const std_msgs::msg::UInt8::ConstSharedPtr msg);
  void onNmeaQuery(const std_msgs::msg::Empty::ConstSharedPtr msg);
  void onNmeaSetting(const std_msgs::msg::UInt8::ConstSharedPtr msg);
  void on_gyro_bias_estimate(const std_msgs::msg::String::ConstSharedPtr msg);

  int proc_response(const yis_cmd_response_t &cmd, unsigned char *data);

 private:
  std::string port_;
  int baudrate_;
  serial::Serial serial_;
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_product_info_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_reset_param_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_baudrate_request_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_baudrate_setting_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_frequency_request_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_frequency_setting_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_output_content_request_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_output_content_setting_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_standard_request_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_standard_setting_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_mode_request_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_mode_setting_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_nmea_request_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_nmea_setting_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_gyro_bias_estimate_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr imu_marker_pub_;
  rclcpp::Publisher<yesense_imu::msg::YesenseImuInertialData>::SharedPtr imu_inertial_data_pub_;
  rclcpp::Publisher<yesense_imu::msg::YesenseImuGpsData>::SharedPtr imu_gps_data_pub_;
  rclcpp::Publisher<yesense_imu::msg::YesenseImuGnssData>::SharedPtr imu_gnss_data_pub_;
  rclcpp::Publisher<yesense_imu::msg::YesenseImuNavData>::SharedPtr imu_nav_pub_;
  rclcpp::Publisher<yesense_imu::msg::YesenseImuStatus>::SharedPtr imu_status_pub_;
  rclcpp::Publisher<yesense_imu::msg::YesenseImuSensorData>::SharedPtr imu_sensor_data_pub_;
  rclcpp::Publisher<yesense_imu::msg::YesenseImuCmdResp>::SharedPtr pub_cmd_exec_resp_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;

  sensor_msgs::msg::Imu g_imu_;
  std::string tf_parent_frame_id_;
  std::string tf_frame_id_;
  std::string frame_id_;
  double time_offset_in_seconds_;
  bool broadcast_tf_;
  bool enableRviz_;
  double linear_acceleration_stddev_;
  double angular_velocity_stddev_;
  double orientation_stddev_;

  std::string data_;
  boost::mutex m_mutex_;
  const int buffer_size_;
  boost::shared_ptr<boost::circular_buffer<char> > data_buffer_ptr_;

  protocol_info_t yesense_out;
  int bytes_;
  bool configured_;

  uint32_t last_sync_time;
  uint32_t last_sync_receive_time;
  uint32_t last_msg_timeout_time;

  uint8_t message_in_[DATA_BUF_SIZE];
  uint8_t gps_buf[DATA_BUF_SIZE];
  uint32_t gps_buf_index;
  std::map<uint32_t, std::string> gps_raw;

  boost::mutex m_response_mutex_;
  bool wait_response_flag_;
  bool check_respose_flag_;
  int error_respose_cnt_;

  uint8_t param_class_;
  uint8_t param_id_;
  yis_cmd_response_t response;

  std::string param_prev_topic_id_;
  std::string param_prev_topic_cmd_;

  boost::thread deseralize_thread_;
};

}  // namespace yesense
