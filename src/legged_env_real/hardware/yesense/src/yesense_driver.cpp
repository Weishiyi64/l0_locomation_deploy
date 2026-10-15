#include "yesense_driver.h"

#include <tf2/LinearMath/Quaternion.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/shared_array.hpp>
#include <chrono>
#include <map>
#include <thread>
#include <vector>

namespace yesense {

YesenseDriver::YesenseDriver(rclcpp::Node::SharedPtr node)
    : node_(node),
      port_("/dev/ttyUSB0"),
      baudrate_(460800),
      buffer_size_(4096),
      wait_response_flag_(false),
      check_respose_flag_(false),
      error_respose_cnt_(0),
      configured_(false) {
  node_->declare_parameter<std::string>("yesense_port", port_);
  node_->declare_parameter<int>("yesense_baudrate", baudrate_);
  node_->get_parameter("yesense_port", port_);
  node_->get_parameter("yesense_baudrate", baudrate_);

  node_->declare_parameter<std::string>("tf_parent_frame_id", "imu_base");
  node_->declare_parameter<std::string>("tf_frame_id", "imu_link");
  node_->declare_parameter<std::string>("frame_id", "imu_link");
  node_->declare_parameter<double>("time_offset_in_seconds", 0.0);
  node_->declare_parameter<bool>("broadcast_tf", true);
  node_->declare_parameter<bool>("enable_rviz", false);
  node_->declare_parameter<double>("linear_acceleration_stddev", 0.0);
  node_->declare_parameter<double>("angular_velocity_stddev", 0.0);
  node_->declare_parameter<double>("orientation_stddev", 0.0);
  node_->get_parameter("tf_parent_frame_id", tf_parent_frame_id_);
  node_->get_parameter("tf_frame_id", tf_frame_id_);
  node_->get_parameter("frame_id", frame_id_);
  node_->get_parameter("time_offset_in_seconds", time_offset_in_seconds_);
  node_->get_parameter("broadcast_tf", broadcast_tf_);
  node_->get_parameter("enable_rviz", enableRviz_);
  node_->get_parameter("linear_acceleration_stddev", linear_acceleration_stddev_);
  node_->get_parameter("angular_velocity_stddev", angular_velocity_stddev_);
  node_->get_parameter("orientation_stddev", orientation_stddev_);

  g_imu_.header.frame_id = frame_id_;

  g_imu_.linear_acceleration_covariance[0] = linear_acceleration_stddev_;
  g_imu_.linear_acceleration_covariance[4] = linear_acceleration_stddev_;
  g_imu_.linear_acceleration_covariance[8] = linear_acceleration_stddev_;

  g_imu_.angular_velocity_covariance[0] = angular_velocity_stddev_;
  g_imu_.angular_velocity_covariance[4] = angular_velocity_stddev_;
  g_imu_.angular_velocity_covariance[8] = angular_velocity_stddev_;

  g_imu_.orientation_covariance[0] = orientation_stddev_;
  g_imu_.orientation_covariance[4] = orientation_stddev_;
  g_imu_.orientation_covariance[8] = orientation_stddev_;

  data_buffer_ptr_ = boost::shared_ptr<boost::circular_buffer<char> >(new boost::circular_buffer<char>(buffer_size_));
  bytes_ = 0;

  sub_product_info_ = node_->create_subscription<std_msgs::msg::Int8>(
      "production_query", 1, std::bind(&YesenseDriver::onProductionInformationQuery, this, std::placeholders::_1));
  sub_baudrate_request_ = node_->create_subscription<std_msgs::msg::Empty>(
      "baudrate_query", 1, std::bind(&YesenseDriver::onBaudrateQuery, this, std::placeholders::_1));
  sub_baudrate_setting_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "baudrate_setting", 1, std::bind(&YesenseDriver::onBaudrateSetting, this, std::placeholders::_1));
  sub_frequency_request_ = node_->create_subscription<std_msgs::msg::Empty>(
      "freequency_query", 1, std::bind(&YesenseDriver::onFrequencyQuery, this, std::placeholders::_1));
  sub_frequency_setting_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "freequency_setting", 1, std::bind(&YesenseDriver::onFrequencySetting, this, std::placeholders::_1));
  sub_output_content_request_ = node_->create_subscription<std_msgs::msg::Empty>(
      "output_content_query", 1, std::bind(&YesenseDriver::onOutputContentQuery, this, std::placeholders::_1));
  sub_output_content_setting_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "output_content_setting", 1, std::bind(&YesenseDriver::onOutputContentSetting, this, std::placeholders::_1));
  sub_standard_request_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "standard_param_query", 1, std::bind(&YesenseDriver::onStandardParamQuery, this, std::placeholders::_1));
  sub_standard_setting_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "standard_param_setting", 1, std::bind(&YesenseDriver::onStandardParamSetting, this, std::placeholders::_1));
  sub_mode_request_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "mode_query", 1, std::bind(&YesenseDriver::onModeSettingQuery, this, std::placeholders::_1));
  sub_mode_setting_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "mode_setting", 1, std::bind(&YesenseDriver::onModeSettingSetting, this, std::placeholders::_1));
  sub_nmea_request_ = node_->create_subscription<std_msgs::msg::Empty>(
      "nmea_query", 1, std::bind(&YesenseDriver::onNmeaQuery, this, std::placeholders::_1));
  sub_nmea_setting_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "nmea_setting", 1, std::bind(&YesenseDriver::onNmeaSetting, this, std::placeholders::_1));
  sub_gyro_bias_estimate_ = node_->create_subscription<std_msgs::msg::String>(
      "yesense/gyro_bias_estimate", 1, std::bind(&YesenseDriver::on_gyro_bias_estimate, this, std::placeholders::_1));

  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data", 100);
  if (enableRviz_)
  {
    RCLCPP_INFO(node_->get_logger(), "RViz visualization enabled - creating publisher for /imu/marker");
    imu_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("imu/marker", 100);
  }
  imu_inertial_data_pub_ =
      node_->create_publisher<yesense_imu::msg::YesenseImuInertialData>("yesense/inertial_data", 100);
  imu_nav_pub_ = node_->create_publisher<yesense_imu::msg::YesenseImuNavData>("yesense/nav_data", 100);
  imu_gnss_data_pub_ = node_->create_publisher<yesense_imu::msg::YesenseImuGnssData>("yesense/gnss_data", 100);
  imu_gps_data_pub_ = node_->create_publisher<yesense_imu::msg::YesenseImuGpsData>("yesense/gps_data", 100);
  imu_status_pub_ = node_->create_publisher<yesense_imu::msg::YesenseImuStatus>("yesense/imu_status", 100);
  imu_sensor_data_pub_ = node_->create_publisher<yesense_imu::msg::YesenseImuSensorData>("yesense/sensor_data", 100);
  pub_cmd_exec_resp_ = node_->create_publisher<yesense_imu::msg::YesenseImuCmdResp>("yesense/command_resp", 100);

  response.response_need = false;
  response.response_found = false;
  response.response_recv_done = false;
  response.len = 0;
  memset((uint8_t *)&yesense_out, 0, sizeof(yesense_out));

  initSerial();

  deseralize_thread_ = boost::thread(boost::bind(&YesenseDriver::_spin, this));
}

YesenseDriver::~YesenseDriver() {
  RCLCPP_INFO(node_->get_logger(), "Close yesense device.");
  if (serial_.isOpen())
  {
    serial_.close();
  }
  data_buffer_ptr_.reset();

  configured_ = false;
  deseralize_thread_.join();
}

void YesenseDriver::run() {
  try
  {
    rclcpp::Rate rate(1000);
    int zero_count = 0;

    while (rclcpp::ok())
    {
      // read data from serial
      //  ("Check imu serial available: %ld, zero_count: %d",
      //  serial_.available(), zero_count);
      if (serial_.available())
      {
        zero_count = 0;
        data_ = serial_.read(serial_.available());
        // ("Read data size: %d",data_.length());

        {
          boost::mutex::scoped_lock lock(m_mutex_);

          for (int i = 0; i < data_.length(); i++)
          {
            data_buffer_ptr_->push_back(data_[i]);
          }
        }
      }
      else
      {
        zero_count++;
        if (zero_count >= 1000)  // 1s
        {
          zero_count = 0;
          // reset the serial connection
          if (serial_.isOpen())
          {
            serial_.close();
          }
          std::this_thread::sleep_for(std::chrono::seconds(1));
          initSerial();
        }
      }

      rclcpp::spin_some(node_);
      rate.sleep();
    }

    RCLCPP_WARN(node_->get_logger(), "ROS Exited !");
  } catch (std::exception &err)
  { RCLCPP_ERROR(node_->get_logger(), "error in 'run' function, msg: %s", err.what()); }
}

void YesenseDriver::initSerial() {
  while (serial_.isOpen() == false)
  {
    try
    {
      RCLCPP_INFO(node_->get_logger(), "port:%s, rate:%d", port_.c_str(), baudrate_);
      serial_.setPort(port_);
      serial_.setBaudrate(baudrate_);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      serial_.setTimeout(to);
      serial_.open();
    } catch (serial::IOException &e)
    {
      RCLCPP_INFO(node_->get_logger(), "Unable to open serial port: %s ,Trying again in 5 seconds.",
                  serial_.getPort().c_str());
      std::this_thread::sleep_for(std::chrono::seconds(5));
    }
  }

  if (serial_.isOpen())
  {
    RCLCPP_INFO(node_->get_logger(), "Serial port: %s initialized and opened.", serial_.getPort().c_str());

    // warning if the serial port is not available
    // ros::Duration(1.0).sleep(); //等待1秒钟，让串口稳定
    // if (serial_.available() == 0)
    // {
    //     ("Serial port %s is not available!",
    //     serial_.getPort().c_str());
    // } else
    // {
    //     ("Serial port %s is available.", serial_.getPort().c_str());
    // }

    configured_ = true;

    ready_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/hardware_status/imu_ready", 1);
    std_msgs::msg::Bool ready_msg;
    ready_msg.data = true;
    ready_pub_->publish(ready_msg);
    RCLCPP_INFO(node_->get_logger(),
                "\033[32m[IMU] Ready status published to "
                "/hardware_status/imu_ready\033[0m");
  }
}

void YesenseDriver::onProductionInformationQuery(const std_msgs::msg::Int8::ConstSharedPtr msg) {
  boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);
  param_prev_topic_id_ = "product info query";

  if (msg->data == 1)  // 查询软件版本信息
  {
    param_prev_topic_cmd_ = "softver";
    // header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    // class
    buffer[2] = PORDUCTION_INFO;

    // id & length
    unsigned short id_length = 0;

    // 设置id 为 0
    //  id_length |= 0 << 0;

    // 长度设置
    id_length |= (1 << 3);

    buffer[3] = id_length & 0xff;
    buffer[4] = id_length >> 8 & 0xff;

    // data
    buffer[5] = 0x02;

    // crc check
    uint8_t ck1 = 0, ck2 = 0;
    for (int i = 0; i < 4; i++)
    {
      ck1 += buffer[2 + i];
      ck2 += ck1;
    }

    buffer[6] = ck1;
    buffer[7] = ck2;
  }
  else if (msg->data == 2)
  {
    param_prev_topic_cmd_ = "product_info";
    // 查询产品信息
    // header
    buffer[0] = 0x59;
    buffer[1] = 0x53;

    // class
    buffer[2] = PORDUCTION_INFO;

    // id & length
    unsigned short id_length = 0;

    // 长度设置
    id_length |= (1 << 3);

    buffer[3] = id_length & 0xff;
    buffer[4] = id_length >> 8 & 0xff;

    // data
    buffer[5] = 0x04;

    // crc check
    uint8_t ck1 = 0, ck2 = 0;
    for (int i = 0; i < 4; i++)
    {
      ck1 += buffer[2 + i];
      ck2 += ck1;
    }

    buffer[6] = ck1;
    buffer[7] = ck2;
  }

#if (ENBALE_DEBUG_OUTPUT)
  std::cout << "Write data to yesense: " << std::endl;
  for (int i = 0; i < 8; i++) printf("%02X ", buffer[i]);
  std::cout << std::endl;
#endif  //

#if (ENABLE_SERIAL_INPUT)
  if (serial_.isOpen())
  {
    int size = serial_.write(buffer.get(), 8);

    // 暂时不考虑写入不成功的情况
    if (size == 8)
    {
      {
        boost::mutex::scoped_lock lock(m_response_mutex_);
        // wait_response_flag_ = true;
        response.response_need = true;
        response.type = buffer[2];          // param_class_ = buffer[2];
        response.op_id = buffer[3] & 0x07;  // param_id_    = (buffer[3] &
                                            // 0x07);
      }
    }
  }
#endif  //
}

/*
 *查询当前波特率的值
 */
void YesenseDriver::onBaudrateQuery(const std_msgs::msg::Empty::ConstSharedPtr msg) {
  boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

  param_prev_topic_id_ = "baudrate command";
  param_prev_topic_cmd_ = "query";

  // header
  buffer[0] = 0x59;
  buffer[1] = 0x53;

  // class
  buffer[2] = BAUDRATE;

  // id & length
  unsigned short id_length = 0;

  // 设置id 为 0（查询）
  //  id_length |= 0 << 0;

  // 长度设置为0
  //  id_length |= (1 << 3);

  buffer[3] = id_length & 0xff;
  buffer[4] = id_length >> 8 & 0xff;

  // crc check
  uint8_t ck1 = 0, ck2 = 0;
  for (int i = 0; i < 3; i++)
  {
    ck1 += buffer[2 + i];
    ck2 += ck1;
  }

  buffer[5] = ck1;
  buffer[6] = ck2;

#if (ENBALE_DEBUG_OUTPUT)
  std::cout << "Write data to yesense: " << std::endl;
  for (int i = 0; i < 7; i++) printf("%02X ", buffer[i]);
  std::cout << std::endl;
#endif  //

#if (ENABLE_SERIAL_INPUT)
  if (serial_.isOpen())
  {
    int size = serial_.write(buffer.get(), 7);

    if (size == 7)
    {
      {
        boost::mutex::scoped_lock lock(m_response_mutex_);
        // wait_response_flag_ = true;
        response.response_need = true;
        response.type = buffer[2];          // param_class_ = buffer[2];
        response.op_id = buffer[3] & 0x07;  // param_id_    = (buffer[3] &
                                            // 0x07);
      }
    }
  }
#endif  //
}

/*
 *设置波特率
 */
void YesenseDriver::onBaudrateSetting(const std_msgs::msg::UInt8::ConstSharedPtr msg) {
  boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);
  param_prev_topic_id_ = "baudrate command";
  param_prev_topic_cmd_ = "setting";

  // header
  buffer[0] = 0x59;
  buffer[1] = 0x53;

  // class
  buffer[2] = BAUDRATE;

  // id & length
  unsigned short id_length = 0;

  // 设置id  (最高位表示写入到内存或者flash)
  if (msg->data >> 7 & 0x01)
    id_length |= 1 << 0;  // 设置到memery
  else
    id_length |= 1 << 1;  // 设置到flash

  // 长度设置为1
  id_length |= (1 << 3);

  buffer[3] = id_length & 0xff;
  buffer[4] = id_length >> 8 & 0xff;

  // data
  buffer[5] = (msg->data) & 0x0f;

  // crc check
  uint8_t ck1 = 0, ck2 = 0;
  for (int i = 0; i < 4; i++)
  {
    ck1 += buffer[2 + i];
    ck2 += ck1;
  }

  // crc
  buffer[6] = ck1;
  buffer[7] = ck2;

#if (ENBALE_DEBUG_OUTPUT)
  std::cout << "Write data to yesense: " << std::endl;
  for (int i = 0; i < 8; i++) printf("%02X ", buffer[i]);
  std::cout << std::endl;
#endif  //

#if (ENABLE_SERIAL_INPUT)
  if (serial_.isOpen())
  {
    int size = serial_.write(buffer.get(), 8);
    if (size == 8)
    {
      {
        boost::mutex::scoped_lock lock(m_response_mutex_);
        // wait_response_flag_ = true;
        response.response_need = true;
        response.type = buffer[2];          // param_class_ = buffer[2];
        response.op_id = buffer[3] & 0x07;  // param_id_    = (buffer[3] &
                                            // 0x07);
      }
    }
  }
#endif  //
}

/*
 *查询输出频率
 */
void YesenseDriver::onFrequencyQuery(const std_msgs::msg::Empty::ConstSharedPtr msg) {
  boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

  param_prev_topic_id_ = "output frequency command";
  param_prev_topic_cmd_ = "query";

  // header
  buffer[0] = 0x59;
  buffer[1] = 0x53;

  // class
  buffer[2] = OUTPUT_FREEQUENCY;

  // id & length
  unsigned short id_length = 0;

  // 设置id 为 0（查询）
  //  id_length |= 0 << 0;

  // 长度设置为0
  //  id_length |= (1 << 3);

  buffer[3] = id_length & 0xff;
  buffer[4] = id_length >> 8 & 0xff;

  // crc check
  uint8_t ck1 = 0, ck2 = 0;
  for (int i = 0; i < 3; i++)
  {
    ck1 += buffer[2 + i];
    ck2 += ck1;
  }

  buffer[5] = ck1;
  buffer[6] = ck2;

#if (ENBALE_DEBUG_OUTPUT)
  std::cout << "Write data to yesense: " << std::endl;
  for (int i = 0; i < 7; i++) printf("%02X ", buffer[i]);
  std::cout << std::endl;
#endif  //

#if (ENABLE_SERIAL_INPUT)
  if (serial_.isOpen())
  {
    int size = serial_.write(buffer.get(), 7);
    if (size == 7)
    {
      {
        boost::mutex::scoped_lock lock(m_response_mutex_);
        // wait_response_flag_ = true;
        response.response_need = true;
        response.type = buffer[2];          // param_class_ = buffer[2];
        response.op_id = buffer[3] & 0x07;  // param_id_    = (buffer[3] &
                                            // 0x07);
      }
    }
  }
#endif  //
}

/*
 *设置输出频率
 */
void YesenseDriver::onFrequencySetting(const std_msgs::msg::UInt8::ConstSharedPtr msg) {
  boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

  param_prev_topic_id_ = "output frequency command";
  param_prev_topic_cmd_ = "setting";

  // header
  buffer[0] = 0x59;
  buffer[1] = 0x53;

  // class
  buffer[2] = OUTPUT_FREEQUENCY;

  // id & length
  unsigned short id_length = 0;

  // 设置id
  if (msg->data >> 7 & 0x01)
    id_length |= 1 << 0;  // 设置到memery    id设置为0
  else
    id_length |= 1 << 1;  // 设置到flash     id设置为1

  // 长度设置为1
  id_length |= (1 << 3);

  buffer[3] = id_length & 0xff;
  buffer[4] = id_length >> 8 & 0xff;

  // 低4位为参数值
  buffer[5] = (msg->data) & 0x0f;

  // crc check
  uint8_t ck1 = 0, ck2 = 0;
  for (int i = 0; i < 4; i++)
  {
    ck1 += buffer[2 + i];
    ck2 += ck1;
  }

  buffer[6] = ck1;
  buffer[7] = ck2;

#if (ENBALE_DEBUG_OUTPUT)
  std::cout << "Write data to yesense: " << std::endl;
  for (int i = 0; i < 8; i++) printf("%02X ", buffer[i]);
  std::cout << std::endl;
#endif  //

#if (ENABLE_SERIAL_INPUT)
  if (serial_.isOpen())
  {
    int size = serial_.write(buffer.get(), 8);
    if (size == 8)
    {
      {
        boost::mutex::scoped_lock lock(m_response_mutex_);
        // wait_response_flag_ = true;
        response.response_need = true;
        response.type = buffer[2];          // param_class_ = buffer[2];
        response.op_id = buffer[3] & 0x07;  // param_id_    = (buffer[3] &
                                            // 0x07);
      }
    }
  }
#endif  //
}

/*
 *查询输出内容
 */
void YesenseDriver::onOutputContentQuery(const std_msgs::msg::Empty::ConstSharedPtr msg) {
  boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

  param_prev_topic_id_ = "output contend command";
  param_prev_topic_cmd_ = "query";

  // header
  buffer[0] = 0x59;
  buffer[1] = 0x53;

  // class
  buffer[2] = OUTPUT_CONTENT;

  // id & length
  unsigned short id_length = 0;

  // 设置id 为 0（查询）
  //  id_length |= 0 << 0;

  // 长度设置为0
  //  id_length |= (1 << 3);

  buffer[3] = id_length & 0xff;
  buffer[4] = id_length >> 8 & 0xff;

  // crc check
  uint8_t ck1 = 0, ck2 = 0;
  for (int i = 0; i < 3; i++)
  {
    ck1 += buffer[2 + i];
    ck2 += ck1;
  }

  buffer[5] = ck1;
  buffer[6] = ck2;

#if (ENBALE_DEBUG_OUTPUT)
  std::cout << "Write data to yesense: " << std::endl;
  for (int i = 0; i < 7; i++) printf("%02X ", buffer[i]);
  std::cout << std::endl;
#endif  //

#if (ENABLE_SERIAL_INPUT)
  if (serial_.isOpen())
  {
    int size = serial_.write(buffer.get(), 7);

    if (size == 7)
    {
      {
        boost::mutex::scoped_lock lock(m_response_mutex_);
        // wait_response_flag_ = true;
        response.response_need = true;
        response.type = buffer[2];          // param_class_ = buffer[2];
        response.op_id = buffer[3] & 0x07;  // param_id_    = (buffer[3] &
                                            // 0x07);
      }
    }
  }
#endif  //
}

/*
 *设置输出内容
 */
void YesenseDriver::onOutputContentSetting(const std_msgs::msg::UInt8::ConstSharedPtr msg) {
  boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

  param_prev_topic_id_ = "output contend command";
  param_prev_topic_cmd_ = "setting";

  // header
  buffer[0] = 0x59;
  buffer[1] = 0x53;

  // class
  buffer[2] = OUTPUT_CONTENT;

  // id & length
  unsigned short id_length = 0;

  // 设置id
  if (msg->data >> 7 & 0x01)
    id_length |= 1 << 0;  // 设置到memery    id设置为0
  else
    id_length |= 1 << 1;  // 设置到flash     id设置为1

  // 长度设置为2
  id_length |= (1 << 4);

  buffer[3] = id_length & 0xff;
  buffer[4] = id_length >> 8 & 0xff;

  // 低4位为参数值
  uint8_t data_type = (msg->data) & 0x0f;
  if (data_type == 0x00)
  {
    // 全部不输出
    buffer[5] = 0x00;
    buffer[6] = 0x00;
  }
  else if (data_type == 0x01)
  {
    // 输出加计、陀螺、磁、欧拉、四元素
    buffer[5] = 0xf8;
    buffer[6] = 0x00;
  }
  else if (data_type == 0x02)
  {
    // 输出位置、速度、UTC、加计、陀螺、磁、欧拉、四元素
    buffer[5] = 0xff;
    buffer[6] = 0x00;
  }

  // crc check
  uint8_t ck1 = 0, ck2 = 0;
  for (int i = 0; i < 5; i++)
  {
    ck1 += buffer[2 + i];
    ck2 += ck1;
  }

  buffer[7] = ck1;
  buffer[8] = ck2;

#if (ENBALE_DEBUG_OUTPUT)
  std::cout << "Write data to yesense: " << std::endl;
  for (int i = 0; i < 9; i++) printf("%02X ", buffer[i]);
  std::cout << std::endl;
#endif  //

#if (ENABLE_SERIAL_INPUT)
  if (serial_.isOpen())
  {
    int size = serial_.write(buffer.get(), 9);

    if (size == 9)
    {
      {
        boost::mutex::scoped_lock lock(m_response_mutex_);
        // wait_response_flag_ = true;
        response.response_need = true;
        response.type = buffer[2];          // param_class_ = buffer[2];
        response.op_id = buffer[3] & 0x07;  // param_id_    = (buffer[3] &
                                            // 0x07);
      }
    }
  }
#endif  //
}

// 标准参数设置相关0x05
void YesenseDriver::onStandardParamQuery(const std_msgs::msg::UInt8::ConstSharedPtr msg) {
  boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

  param_prev_topic_id_ = "parameter command";
  param_prev_topic_cmd_ = "query";

  // header
  buffer[0] = 0x59;
  buffer[1] = 0x53;

  // class
  buffer[2] = STANDARD_PARAM;

  // id & length
  unsigned short id_length = 0;

  // 设置id 为 0（查询）
  //  id_length |= 0 << 0;

  // 长度设置为1
  id_length |= (1 << 3);

  buffer[3] = id_length & 0xff;
  buffer[4] = id_length >> 8 & 0xff;

  if (msg->data == 1)
    buffer[5] = 0x03;  // 陀螺用户零偏
  else if (msg->data == 2)
    buffer[5] = 0x81;  // 读静态阈值

  // crc check
  uint8_t ck1 = 0, ck2 = 0;
  for (int i = 0; i < 4; i++)
  {
    ck1 += buffer[2 + i];
    ck2 += ck1;
  }

  buffer[6] = ck1;
  buffer[7] = ck2;

#if (ENBALE_DEBUG_OUTPUT)
  std::cout << "Write data to yesense: " << std::endl;
  for (int i = 0; i < 8; i++) printf("%02X ", buffer[i]);
  std::cout << std::endl;
#endif  //

#if (ENABLE_SERIAL_INPUT)
  if (serial_.isOpen())
  {
    int size = serial_.write(buffer.get(), 8);

    if (size == 8)
    {
      {
        boost::mutex::scoped_lock lock(m_response_mutex_);
        // wait_response_flag_ = true;
        response.response_need = true;
        response.type = buffer[2];          // param_class_ = buffer[2];
        response.op_id = buffer[3] & 0x07;  // param_id_    = (buffer[3] &
                                            // 0x07);
      }
    }
  }
#endif  //
}

// 标准参数设置0x05
void YesenseDriver::onStandardParamSetting(const std_msgs::msg::UInt8::ConstSharedPtr msg) {
  boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

  int pkg_length = 0;
  param_prev_topic_id_ = "parameter command";
  param_prev_topic_cmd_ = "setting";

  // header
  buffer[0] = 0x59;
  buffer[1] = 0x53;

  // class
  buffer[2] = STANDARD_PARAM;

  // id & length
  unsigned short id_length = 0;

  // 设置id
  if (msg->data >> 7 & 0x01)
  {
    // 设置到memery
    buffer[3] = 0x71;
    buffer[4] = 0x00;

    buffer[5] = 0x03;
    buffer[6] = 0x0C;
    buffer[7] = 0x00;
    buffer[8] = 0x00;
    buffer[9] = 0x00;
    buffer[10] = 0x00;
    buffer[11] = 0x00;
    buffer[12] = 0x00;
    buffer[13] = 0x00;
    buffer[14] = 0x00;
    buffer[15] = 0x00;
    buffer[16] = 0x00;
    buffer[17] = 0x00;
    buffer[18] = 0x00;

    // crc check
    uint8_t ck1 = 0, ck2 = 0;
    for (int i = 0; i < 17; i++)
    {
      ck1 += buffer[2 + i];
      ck2 += ck1;
    }
    buffer[19] = ck1;
    buffer[20] = ck2;

    pkg_length = 21;
  }
  else
  {
    // 设置到flash     id设置为2
    uint8_t mode = msg->data & 0x0f;
    if (mode == 1)
    {
      // 姿态角设置为0
      buffer[3] = 0x32;
      buffer[4] = 0x00;

      buffer[5] = 0x11;
      buffer[6] = 0x04;
      buffer[7] = 0x00;
      buffer[8] = 0x00;
      buffer[9] = 0x00;
      buffer[10] = 0x00;

      // crc check
      uint8_t ck1 = 0, ck2 = 0;
      for (int i = 0; i < 9; i++)
      {
        ck1 += buffer[2 + i];
        ck2 += ck1;
      }
      buffer[11] = ck1;
      buffer[12] = ck2;

      pkg_length = 13;
    }
    else if (mode == 3)
    {
      // 陀螺用户零偏差置0
      buffer[3] = 0x72;
      buffer[4] = 0x00;

      buffer[5] = 0x03;
      buffer[6] = 0x0C;
      buffer[7] = 0x00;
      buffer[8] = 0x00;
      buffer[9] = 0x00;
      buffer[10] = 0x00;
      buffer[11] = 0x00;
      buffer[12] = 0x00;
      buffer[13] = 0x00;
      buffer[14] = 0x00;
      buffer[15] = 0x00;
      buffer[16] = 0x00;
      buffer[17] = 0x00;
      buffer[18] = 0x00;

      // crc check
      uint8_t ck1 = 0, ck2 = 0;
      for (int i = 0; i < 17; i++)
      {
        ck1 += buffer[2 + i];
        ck2 += ck1;
      }
      buffer[19] = ck1;
      buffer[20] = ck2;

      pkg_length = 21;
    }
  }

#if (ENBALE_DEBUG_OUTPUT)
  std::cout << "Write data to yesense[YesenseDriver::onStandardParamSetting]: " << std::endl;
  for (int i = 0; i < pkg_length; i++) printf("%02X ", buffer[i]);
  std::cout << std::endl;
#endif  //

#if (ENABLE_SERIAL_INPUT)
  if (serial_.isOpen())
  {
    int size = serial_.write(buffer.get(), pkg_length);

    if (size == pkg_length)
    {
      {
        boost::mutex::scoped_lock lock(m_response_mutex_);
        // wait_response_flag_ = true;
        response.response_need = true;
        response.type = buffer[2];          // param_class_ = buffer[2];
        response.op_id = buffer[3] & 0x07;  // param_id_    = (buffer[3] &
                                            // 0x07);
      }
    }
  }
#endif  //
}

// 模式设置相关0x4D
void YesenseDriver::onModeSettingQuery(const std_msgs::msg::UInt8::ConstSharedPtr msg) {
  boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

  param_prev_topic_id_ = "function mode command";
  param_prev_topic_cmd_ = "query";

  // header
  buffer[0] = 0x59;
  buffer[1] = 0x53;

  // class
  buffer[2] = MODE_SETTING;

  // id & length
  unsigned short id_length = 0;

  // 长度设置为1
  id_length |= (1 << 3);

  buffer[3] = id_length & 0xff;
  buffer[4] = id_length >> 8 & 0xff;

  if (msg->data == 1)
    buffer[5] = 0x02;
  else if (msg->data == 2)
    buffer[5] = 0x20;
  else if (msg->data == 3)
    buffer[5] = 0x4f;

  // crc check
  uint8_t ck1 = 0, ck2 = 0;
  for (int i = 0; i < 4; i++)
  {
    ck1 += buffer[2 + i];
    ck2 += ck1;
  }

  buffer[6] = ck1;
  buffer[7] = ck2;

#if (ENBALE_DEBUG_OUTPUT)
  std::cout << "Write data to yesense: " << std::endl;
  for (int i = 0; i < 8; i++) printf("%02X ", buffer[i]);
  std::cout << std::endl;
#endif  //

#if (ENABLE_SERIAL_INPUT)
  if (serial_.isOpen())
  {
    int size = serial_.write(buffer.get(), 8);

    if (size == 8)
    {
      {
        boost::mutex::scoped_lock lock(m_response_mutex_);
        // wait_response_flag_ = true;
        response.response_need = true;
        response.type = buffer[2];          // param_class_ = buffer[2];
        response.op_id = buffer[3] & 0x07;  // param_id_    = (buffer[3] &
                                            // 0x07);
      }
    }
  }
#endif  //
}

// 0x4D
void YesenseDriver::onModeSettingSetting(const std_msgs::msg::UInt8::ConstSharedPtr msg) {
  boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[64]);

  param_prev_topic_id_ = "function mode command";
  param_prev_topic_cmd_ = "setting";

  // header
  buffer[0] = 0x59;
  buffer[1] = 0x53;

  // class
  buffer[2] = MODE_SETTING;

  // id & length
  unsigned short id_length = 0;

  // 设置id
  if (msg->data >> 7 & 0x01)
    id_length |= 1 << 0;  // 设置到memery    id设置为0
  else
    id_length |= 1 << 1;  // 设置到flash     id设置为1

  // 长度设置为2
  id_length |= (1 << 4);

  buffer[3] = id_length & 0xff;
  buffer[4] = id_length >> 8 & 0xff;

  uint8_t mode = msg->data & 0x0f;
  if (mode == 1)
  {
    //
    buffer[5] = 0x50;
    buffer[6] = 0x01;
  }
  else
    return;

  // crc check
  uint8_t ck1 = 0, ck2 = 0;
  for (int i = 0; i < 5; i++)
  {
    ck1 += buffer[2 + i];
    ck2 += ck1;
  }

  buffer[7] = ck1;
  buffer[8] = ck2;

#if (ENBALE_DEBUG_OUTPUT)
  std::cout << "Write data to yesense: " << std::endl;
  for (int i = 0; i < 9; i++) printf("%02X ", buffer[i]);
  std::cout << std::endl;
#endif  //

#if (ENABLE_SERIAL_INPUT)
  if (serial_.isOpen())
  {
    int size = serial_.write(buffer.get(), 9);
    if (size == 9)
    {
      {
        boost::mutex::scoped_lock lock(m_response_mutex_);
        // wait_response_flag_ = true;
        response.response_need = true;
        response.type = buffer[2];          // param_class_ = buffer[2];
        response.op_id = buffer[3] & 0x07;  // param_id_    = (buffer[3] &
                                            // 0x07);
      }
    }
  }
#endif  //
}

// NMEA输出设置相关0x4E
void YesenseDriver::onNmeaQuery(const std_msgs::msg::Empty::ConstSharedPtr msg) {}

void YesenseDriver::onNmeaSetting(const std_msgs::msg::UInt8::ConstSharedPtr msg) {}

void YesenseDriver::on_gyro_bias_estimate(const std_msgs::msg::String::ConstSharedPtr msg) {
#define BIAS_EST_ON "\x59\x53\x4D\x11\x00\x51\x01\xB0\x68"
#define BIAS_EST_OFF "\x59\x53\x4D\x11\x00\x51\x02\xB1\x69"
#define BIAS_EST_GET "\x59\x53\x4D\x08\x00\x51\xA6\x9D"

  std::string cmd = boost::to_lower_copy(msg.get()->data);
  std::string err_msg = "";

  uint8_t *ys_cmd = NULL;
  size_t ys_cmd_len = 0;

  param_prev_topic_id_ = "yesense/gyro_bias_estimate";
  param_prev_topic_cmd_ = cmd;

  if (cmd == "enable")
  {
    ys_cmd = (uint8_t *)BIAS_EST_ON;
    ys_cmd_len = sizeof(BIAS_EST_ON);
  }
  else if (cmd == "disable")
  {
    ys_cmd = (uint8_t *)BIAS_EST_OFF;
    ys_cmd_len = sizeof(BIAS_EST_OFF);
  }
  else if (cmd == "query")
  {
    ys_cmd = (uint8_t *)BIAS_EST_GET;
    ys_cmd_len = sizeof(BIAS_EST_GET);
  }
  else
  {
    err_msg = "Invalid command: '" + cmd + "'";
    goto fail;
  }

  if (!serial_.isOpen())
  {
    err_msg = "serial port is not opened !";
    goto fail;
  }

  //
  {
    int size = serial_.write(ys_cmd, ys_cmd_len);
    if (size != ys_cmd_len)
    {
      err_msg = "serial write failed !, req_size != writed_size !";
      goto fail;
    }

    {
      boost::mutex::scoped_lock lock(m_response_mutex_);
      // wait_response_flag_ = true;
      response.response_need = true;
      response.type = ys_cmd[2];          // param_class_ = ys_cmd[2];
      response.op_id = ys_cmd[3] & 0x07;  // param_id_    = ys_cmd[3] & 0x07;
    }
  }

  return;

fail:
  yesense_imu::msg::YesenseImuCmdResp resp_msg;
  resp_msg.id = param_prev_topic_id_;
  resp_msg.cmd = param_prev_topic_cmd_;
  resp_msg.msg = err_msg;
  resp_msg.success = false;
  pub_cmd_exec_resp_->publish(resp_msg);
}

void YesenseDriver::_spin() {
  try
  { this->spin(); } catch (std::exception &err)
  { RCLCPP_ERROR(node_->get_logger(), "error in 'spin', msg: %s", err.what()); }
}

int YesenseDriver::proc_response(const yis_cmd_response_t &cmd, unsigned char *data) {
  yesense_imu::msg::YesenseImuCmdResp msg;
  unsigned short offset = sizeof(yis_tlv_header_t);

  if (NULL == data)
  {
    return -1;
  }

  msg.id = param_prev_topic_id_;
  msg.cmd = param_prev_topic_cmd_;

  if (CONFIG_MEMERY == cmd.op_id || CONFIG_FLASH == cmd.op_id)
  {
    msg.success = (cmd.len > 1) ? (((uint8_t)0 == data[1]) ? true : false) : (((uint8_t)0 == data[0]) ? true : false);
    msg.msg = msg.success ? "ok" : "fail";
  }
  else
  {
    msg.success = true;
    msg.msg = "ok";

    switch (cmd.type)
    {
      case PORDUCTION_INFO: {
        for (int i = 0; i < cmd.len - offset; i++)
        {
          msg.data.push_back(data[i + offset]);
        }
      }
      break;

      case BAUDRATE: {
        if ((unsigned char)1 == cmd.len)
        {
          msg.data.push_back(data[0]);
        }
        else
        {
          msg.msg = "len err";
        }
      }
      break;

      case OUTPUT_FREEQUENCY: {
        if ((unsigned char)1 == cmd.len)
        {
          msg.data.push_back(data[0]);
        }
        else
        {
          msg.msg = "len err";
        }
      }
      break;

      case OUTPUT_CONTENT: {
        if ((unsigned char)2 == cmd.len)
        {
          msg.data.push_back(data[0]);
          msg.data.push_back(data[1]);
        }
        else
        {
          msg.msg = "len err";
        }
      }
      break;

      case STANDARD_PARAM: {
        for (unsigned int i = 0; i < cmd.len - offset; i++)
        {
          msg.data.push_back(data[i + offset]);
        }
      }
      break;

      case MODE_SETTING: /*type(1B) + data(1B)*/
      {
        if ((unsigned char)2 == cmd.len)
        {
          msg.data.push_back(data[1]);
        }
        else
        {
          msg.msg = "len err";
        }
      }
      break;

      default: {
        msg.success = false;
        msg.msg = "unkown";
      }
      break;
    }
  }

  pub_cmd_exec_resp_->publish(msg);

  return 0;
}

void YesenseDriver::spin() {
  rclcpp::Rate rate(500);

#define RECV_TIMEOUT_THR 20 /*unit is equal to the above rate*/

  uint16_t tid = 0x00;
  uint16_t prev_tid = 0x00;

  uint32_t gps_header_sum;
  uint32_t recv_len = 0;
  uint32_t timeout_cnt = 0;

  while (configured_)
  {
    recv_len = data_buffer_ptr_->size();
    // Cap read so we never exceed message_in_ size (DATA_BUF_SIZE); otherwise
    // bytes_ + recv_len could exceed 1024 and cause buffer overflow / SIGSEGV.
    uint32_t to_read = recv_len;
    if (bytes_ + to_read > DATA_BUF_SIZE)
    {
      to_read = (bytes_ < DATA_BUF_SIZE) ? (DATA_BUF_SIZE - bytes_) : 0;
    }
    if (to_read > 0)
    {
      boost::mutex::scoped_lock lock(m_mutex_);
      for (unsigned int i = 0; i < to_read; i++)
      {
        message_in_[bytes_ + i] = data_buffer_ptr_->begin()[i];
      }
      bytes_ += to_read;
      data_buffer_ptr_->erase_begin(to_read);
    }

    /*analysis data from message_in_ when total recieved data length is bigger
     * than PROTOCOL_MIN_LEN*/
    if (bytes_ >= PROTOCOL_MIN_LEN)
    {
      int cnt = bytes_;
      int pos = 0;
      // ("cnt len %d", cnt);
      while (cnt > 0)
      {
        int ret = analysis_data(message_in_ + pos, cnt, &yesense_out, &response);

        /*未查找到帧头*/
        if (analysis_done == ret)
        {
          pos++;
          cnt--;
        }
        else if (data_len_err == ret)
        {
          if (timeout_cnt >= RECV_TIMEOUT_THR)
          {
            timeout_cnt = 0;
            cnt = 0;
          }

          break;
        }
        else if (crc_err == ret || analysis_ok == ret)
        {
          /*删除已解析完的完整一帧*/
          output_data_header_t *header = (output_data_header_t *)(message_in_ + pos);
          unsigned int frame_len = header->len + PROTOCOL_MIN_LEN;
          if (response.response_recv_done)
          {
            frame_len = response.len + PROTOCOL_MIN_LEN;
          }

          if (analysis_ok == ret)
          {
            // ("tid is %d -->-1", yesense_out.tid);
            if (response.response_recv_done)
            {
              // ("response recv");
              boost::mutex::scoped_lock lock(m_response_mutex_);
              response.response_need = false;
              response.response_found = false;
              response.response_recv_done = false;
              proc_response(response, message_in_ + pos + YIS_RESPONSE_CMD_DATA_POS);
            }
            else
            {
              // ("tid is %d -->0", yesense_out.tid);
              if (prev_tid != 0 && tid > prev_tid && prev_tid != tid - 1)
              {
                RCLCPP_INFO(node_->get_logger(), "Frame losed: prev_TID: %d, cur_TID: %d", prev_tid, tid);
              }

              prev_tid = tid;
              boost::mutex::scoped_lock lock(m_mutex_);
              // ("tid is %d -->1", yesense_out.tid);
              publish_imu(yesense_out);
            }
          }

          cnt -= frame_len;
          pos += frame_len;
        }
      }

      // ("analysis once"); if(bytes_ > cnt && cnt > 0)
      memcpy(message_in_, message_in_ + pos, cnt);
      bytes_ = cnt;
    }

    rate.sleep();
  }
}

#define EARTH_RADIUS 6378.137  // 地球半径
#define Angle_To_Rad(x) (((x) * 3.141592653589793) / 180.0)

void YesenseDriver::update_position_by_gps(const protocol_info_t &imu_data, geometry_msgs::msg::PoseStamped &pose) {
  // initial gps location
  static double initial_lon, initial_lat, initial_alt;

  // update initial gps location at first invoke
  if (initial_lon == 0 || initial_lat == 0 || initial_alt == 0)
  {
    RCLCPP_INFO(node_->get_logger(), "update initial gps location !");

    initial_lon = imu_data.gnss.location.longtidue;
    initial_lat = imu_data.gnss.location.latitude;
    initial_alt = imu_data.gnss.location.altidue;

    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    return;  // exit
  }

  // calcu relative location
  double radLat1, radLat2, radLong1, radLong2, delta_lat, delta_long;

  radLat1 = Angle_To_Rad(initial_lat);
  radLong1 = Angle_To_Rad(initial_lon);
  radLat2 = Angle_To_Rad(imu_data.gnss.location.latitude);
  radLong2 = Angle_To_Rad(imu_data.gnss.location.longtidue);

  // calcu x
  delta_lat = radLat2 - radLat1;
  delta_long = 0;
  double x = 2 * asin(sqrt(pow(sin(delta_lat / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(delta_long / 2), 2)));
  x = x * EARTH_RADIUS * 1000;

  // calcu y
  delta_lat = 0;
  delta_long = radLong1 - radLong2;
  double y = 2 * asin(sqrt(pow(sin(delta_lat / 2), 2) + cos(radLat2) * cos(radLat2) * pow(sin(delta_long / 2), 2)));
  y = y * EARTH_RADIUS * 1000;

  // calcu z
  double z = imu_data.gnss.location.altidue - initial_alt;

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
}

void YesenseDriver::publish_imu(const protocol_info_t &imu_data) {
  // publish imu message
  g_imu_.header.stamp = node_->now();

  tf2::Quaternion q;
  q.setRPY(imu_data.roll / 180.0 * M_PI, imu_data.pitch / 180.0 * M_PI, imu_data.yaw / 180.0 * M_PI);
  // std::cout << imu_data.roll << ", " << imu_data.pitch << ", " <<
  // imu_data.yaw << ", " << std::endl;

  g_imu_.angular_velocity.x = imu_data.angle_x / 180.0 * M_PI;
  g_imu_.angular_velocity.y = imu_data.angle_y / 180.0 * M_PI;
  g_imu_.angular_velocity.z = imu_data.angle_z / 180.0 * M_PI;

  g_imu_.linear_acceleration.x = imu_data.accel_x;
  g_imu_.linear_acceleration.y = imu_data.accel_y;
  g_imu_.linear_acceleration.z = imu_data.accel_z;

  g_imu_.orientation.x = q.x();
  g_imu_.orientation.y = q.y();
  g_imu_.orientation.z = q.z();
  g_imu_.orientation.w = q.w();
  imu_pub_->publish(g_imu_);

  // update imu marker (only when rviz is enabled)
  if (enableRviz_)
  {
    visualization_msgs::msg::Marker marker_info;
    marker_info.header.frame_id = "imu_link";
    marker_info.header.stamp = node_->now();

    // set namespace and id
    marker_info.ns = "basic_shapes";
    marker_info.id = 0;

    // set marker's shape
    marker_info.type = visualization_msgs::msg::Marker::CUBE;

    // set action: ADD
    marker_info.action = visualization_msgs::msg::Marker::ADD;

    // set imu pose
    marker_info.pose.position.x = 0.0;
    marker_info.pose.position.y = 0.0;
    marker_info.pose.position.z = 0.0;
    marker_info.pose.orientation.x = g_imu_.orientation.x;
    marker_info.pose.orientation.y = g_imu_.orientation.y;
    marker_info.pose.orientation.z = g_imu_.orientation.z;
    marker_info.pose.orientation.w = g_imu_.orientation.w;

    // set scale, unit: m
    marker_info.scale.x = 4.0;
    marker_info.scale.y = 4.0;
    marker_info.scale.z = 3.0;

    // set color
    marker_info.color.r = 0.2f;
    marker_info.color.g = 0.2f;
    marker_info.color.b = 0.2f;
    marker_info.color.a = 1.0;

    marker_info.lifetime.sec = 0;
    marker_info.lifetime.nanosec = 0;

    imu_marker_pub_->publish(marker_info);
  }

  /*-------------------publish inertial data topic----------------------*/
  yesense_imu::msg::YesenseImuInertialData inertial_data;

  inertial_data.tid = imu_data.tid;
  inertial_data.temperature = imu_data.imu_temp;
  inertial_data.sample_timestamp = imu_data.sample_timestamp;
  inertial_data.sync_timestamp = imu_data.out_sync_timestamp;

  inertial_data.accel.linear.x = imu_data.accel_x;
  inertial_data.accel.linear.y = imu_data.accel_y;
  inertial_data.accel.linear.z = imu_data.accel_z;
  inertial_data.accel.angular.x = imu_data.angle_x;
  inertial_data.accel.angular.y = imu_data.angle_y;
  inertial_data.accel.angular.z = imu_data.angle_z;

  inertial_data.euler_angle.roll = imu_data.roll;
  inertial_data.euler_angle.pitch = imu_data.pitch;
  inertial_data.euler_angle.yaw = imu_data.yaw;

  inertial_data.quaternion.q0 = imu_data.quaternion_data0;
  inertial_data.quaternion.q1 = imu_data.quaternion_data1;
  inertial_data.quaternion.q2 = imu_data.quaternion_data2;
  inertial_data.quaternion.q3 = imu_data.quaternion_data3;

  imu_inertial_data_pub_->publish(inertial_data);

  /*-------------------publish some topic----------------------*/
  yesense_imu::msg::YesenseImuAllData raw_data;
  yesense_imu::msg::YesenseImuNavData nav_data;

  raw_data.temperature = imu_data.imu_temp;
  raw_data.sample_timestamp = imu_data.sample_timestamp;
  raw_data.sync_timestamp = imu_data.out_sync_timestamp;

  raw_data.accel.linear.x = imu_data.accel_x;
  raw_data.accel.linear.y = imu_data.accel_y;
  raw_data.accel.linear.z = imu_data.accel_z;

  raw_data.accel.angular.x = imu_data.angle_x;
  raw_data.accel.angular.y = imu_data.angle_y;
  raw_data.accel.angular.z = imu_data.angle_z;

  raw_data.euler_angle.roll = imu_data.roll;
  raw_data.euler_angle.pitch = imu_data.pitch;
  raw_data.euler_angle.yaw = imu_data.yaw;

  raw_data.quaternion.q0 = imu_data.quaternion_data0;
  raw_data.quaternion.q1 = imu_data.quaternion_data1;
  raw_data.quaternion.q2 = imu_data.quaternion_data2;
  raw_data.quaternion.q3 = imu_data.quaternion_data3;

  raw_data.location.longtidue = imu_data.gnss.location.longtidue;
  raw_data.location.latitude = imu_data.gnss.location.latitude;
  raw_data.location.altidue = imu_data.gnss.location.altidue;

  /*gnss basic topic*/
  nav_data.tid = imu_data.tid;
  nav_data.location.longtidue = imu_data.gnss.location.longtidue;
  nav_data.location.latitude = imu_data.gnss.location.latitude;
  nav_data.location.altidue = imu_data.gnss.location.altidue;
  nav_data.utc_time.year = imu_data.gnss.utc_time.year;
  nav_data.utc_time.month = imu_data.gnss.utc_time.month;
  nav_data.utc_time.date = imu_data.gnss.utc_time.date;
  nav_data.utc_time.hour = imu_data.gnss.utc_time.hour;
  nav_data.utc_time.min = imu_data.gnss.utc_time.min;
  nav_data.utc_time.sec = imu_data.gnss.utc_time.sec;
  nav_data.utc_time.ms = imu_data.gnss.utc_time.ms;
  nav_data.status.fusion_status = imu_data.fusion_status & 0x0f;
  nav_data.status.gnss_status = (imu_data.fusion_status >> 4) & 0x0f;

  // raw_data.status.raw_code = imu_data.status;
  raw_data.status.fusion_status = imu_data.fusion_status & 0x0f;
  raw_data.status.gnss_status = (imu_data.fusion_status >> 4) & 0x0f;

  // gnss data
  raw_data.gnss.master.utc_time.year = imu_data.gnss.utc_time.year;
  raw_data.gnss.master.utc_time.month = imu_data.gnss.utc_time.month;
  raw_data.gnss.master.utc_time.date = imu_data.gnss.utc_time.date;
  raw_data.gnss.master.utc_time.hour = imu_data.gnss.utc_time.hour;
  raw_data.gnss.master.utc_time.min = imu_data.gnss.utc_time.min;
  raw_data.gnss.master.utc_time.sec = imu_data.gnss.utc_time.sec;
  raw_data.gnss.master.utc_time.ms = imu_data.gnss.utc_time.ms;

  raw_data.gnss.master.location.longtidue = imu_data.gnss.location.longtidue;
  raw_data.gnss.master.location.latitude = imu_data.gnss.location.latitude;
  raw_data.gnss.master.location.altidue = imu_data.gnss.location.altidue;

  raw_data.gnss.master.location_error.longtidue = imu_data.gnss.location_error.lon;
  raw_data.gnss.master.location_error.latitude = imu_data.gnss.location_error.lat;
  raw_data.gnss.master.location_error.altidue = imu_data.gnss.location_error.alt;

  raw_data.gnss.master.vel.v_e = imu_data.gnss.vel.vel_e;
  raw_data.gnss.master.vel.v_n = imu_data.gnss.vel.vel_n;
  raw_data.gnss.master.vel.v_u = imu_data.gnss.vel.vel_u;

  raw_data.gnss.master.speed = imu_data.gnss.ground_speed;
  raw_data.gnss.master.yaw = imu_data.gnss.yaw;
  raw_data.gnss.master.status = imu_data.gnss.status;
  raw_data.gnss.master.star_cnt = imu_data.gnss.star_cnt;
  raw_data.gnss.master.p_dop = imu_data.gnss.p_dop;
  raw_data.gnss.master.site_id = imu_data.gnss.site_id;

  raw_data.gnss.slave.dual_ant_yaw = imu_data.gnss_slave.dual_ant_yaw;
  raw_data.gnss.slave.dual_ant_yaw_error = imu_data.gnss_slave.dual_ant_yaw_error;
  raw_data.gnss.slave.dual_ant_baseline_len = imu_data.gnss_slave.dual_ant_baseline_len;

  for (std::map<uint32_t, std::string>::iterator inter = gps_raw.begin(); inter != gps_raw.end(); inter++)
  {
    raw_data.gps.raw_data.push_back(inter->second);
  }

  yesense_imu::msg::YesenseImuSensorData sensor_data;
  sensor_data.temperature = raw_data.temperature;
  sensor_data.sample_timestamp = raw_data.sample_timestamp;
  sensor_data.sync_timestamp = raw_data.sync_timestamp;
  sensor_data.accel = raw_data.accel;
  sensor_data.quaternion = raw_data.quaternion;
  sensor_data.euler_angle = raw_data.euler_angle;
  sensor_data.location = raw_data.location;
  imu_sensor_data_pub_->publish(sensor_data);
  imu_nav_pub_->publish(nav_data);

  imu_gps_data_pub_->publish(raw_data.gps);
  imu_gnss_data_pub_->publish(raw_data.gnss);
  imu_status_pub_->publish(raw_data.status);
}

}  // namespace yesense
