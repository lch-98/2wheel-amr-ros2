// ============================================
// YDLIDAR X4 커스텀 ROS2 노드
// tri_test.cpp 기반 (동작 검증된 초기화 설정 사용)
// ============================================
#include "CYdLidar.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <memory>
#include <string>
#include <cmath>
#include <vector>

using namespace ydlidar;

class YdlidarNode : public rclcpp::Node
{
public:
  YdlidarNode() : Node("ydlidar_node")
  {
    // ── 파라미터 ─────────────────────────────
    this->declare_parameter<std::string>("port", "/dev/ttyUSB1");
    this->declare_parameter<int>("baudrate", 128000);
    this->declare_parameter<std::string>("frame_id", "base_scan");
    this->declare_parameter<double>("frequency", 7.0);
    this->declare_parameter<double>("range_min", 0.10);
    this->declare_parameter<double>("range_max", 12.0);

    port_       = this->get_parameter("port").as_string();
    baudrate_   = this->get_parameter("baudrate").as_int();
    frame_id_   = this->get_parameter("frame_id").as_string();
    frequency_  = this->get_parameter("frequency").as_double();
    range_min_  = this->get_parameter("range_min").as_double();
    range_max_  = this->get_parameter("range_max").as_double();

    // ── /scan 퍼블리셔 ───────────────────────
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS());

    // ── 라이다 초기화 ────────────────────────
    if (!initLidar()) {
      RCLCPP_ERROR(this->get_logger(), "Lidar init failed");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "YDLIDAR started on %s", port_.c_str());

    // ── 스캔 루프 (별도 스레드) ──────────────
    scan_thread_ = std::thread(&YdlidarNode::scanLoop, this);
  }

  ~YdlidarNode()
  {
    running_ = false;
    if (scan_thread_.joinable()) scan_thread_.join();
    laser_.turnOff();
    laser_.disconnecting();
  }

private:
  bool initLidar()
  {
    // ══════ tri_test 와 동일한 설정 ══════
    // string
    laser_.setlidaropt(LidarPropSerialPort, port_.c_str(), port_.size());
    std::string ignore_array;
    laser_.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                       ignore_array.size());

    // int
    laser_.setlidaropt(LidarPropSerialBaudrate, &baudrate_, sizeof(int));
    int optval = TYPE_TRIANGLE;
    laser_.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
    optval = YDLIDAR_TYPE_SERIAL;
    laser_.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
    optval = 4;   // sample rate (two-way)
    laser_.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
    optval = 4;   // abnormal check count
    laser_.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
    optval = 8;   // intensity bit
    laser_.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

    // bool
    bool b = false;
    bool b_fixed = true;
    laser_.setlidaropt(LidarPropFixedResolution, &b_fixed, sizeof(bool));
    bool b_rev = true; // reversion option을 true로 바꿔서 앞뒤 180도 보정
    laser_.setlidaropt(LidarPropReversion, &b_rev, sizeof(bool));
    bool b_inv = true; // inverted option을 true로 바꿔서 좌우 반전 보정
    laser_.setlidaropt(LidarPropInverted, &b_inv, sizeof(bool));
    b = true;
    laser_.setlidaropt(LidarPropAutoReconnect, &b, sizeof(bool));
    bool single = false;
    laser_.setlidaropt(LidarPropSingleChannel, &single, sizeof(bool));
    b = false;
    laser_.setlidaropt(LidarPropIntenstiy, &b, sizeof(bool));
    b = true;
    laser_.setlidaropt(LidarPropSupportMotorDtrCtrl, &b, sizeof(bool));
    b = false;
    laser_.setlidaropt(LidarPropSupportHeartBeat, &b, sizeof(bool));

    // float
    float f = 180.0f;
    laser_.setlidaropt(LidarPropMaxAngle, &f, sizeof(float));
    f = -180.0f;
    laser_.setlidaropt(LidarPropMinAngle, &f, sizeof(float));
    f = 64.0f;
    laser_.setlidaropt(LidarPropMaxRange, &f, sizeof(float));
    f = 0.05f;
    laser_.setlidaropt(LidarPropMinRange, &f, sizeof(float));
    float freq = static_cast<float>(frequency_);
    laser_.setlidaropt(LidarPropScanFrequency, &freq, sizeof(float));

    laser_.enableGlassNoise(false);
    laser_.enableSunNoise(false);

    if (!laser_.initialize()) {
      RCLCPP_ERROR(this->get_logger(), "initialize: %s", laser_.DescribeError());
      return false;
    }
    if (!laser_.turnOn()) {
      RCLCPP_ERROR(this->get_logger(), "turnOn: %s", laser_.DescribeError());
      return false;
    }
    return true;
  }

  void scanLoop()
  {
    LaserScan scan;
    while (running_ && rclcpp::ok())
    {
      if (laser_.doProcessSimple(scan))
      {
        publishScan(scan);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Failed to get scan");
      }
    }
  }

  void publishScan(const LaserScan &scan)
  {
    auto msg = sensor_msgs::msg::LaserScan();

    // 스캔 시작 시각으로 보정 (스캔 소요시간만큼 과거)
    rclcpp::Time end_time = this->now();
    double scan_duration = scan.config.scan_time;
    msg.header.stamp = end_time - rclcpp::Duration::from_seconds(scan_duration);
    msg.header.frame_id = frame_id_;

    msg.angle_min = scan.config.min_angle;
    msg.angle_max = scan.config.max_angle;
    msg.angle_increment = scan.config.angle_increment;
    msg.scan_time = scan.config.scan_time;
    msg.time_increment = scan.config.time_increment;
    msg.range_min = range_min_;
    msg.range_max = range_max_;

    // 각도 범위로 배열 크기 계산
    int count = static_cast<int>(
        (scan.config.max_angle - scan.config.min_angle) /
        scan.config.angle_increment) + 1;
    if (count <= 0) return;

    msg.ranges.assign(count, std::numeric_limits<float>::infinity());
    msg.intensities.assign(count, 0.0f);

    for (const auto &p : scan.points)
    {
      int idx = static_cast<int>(
          (p.angle - scan.config.min_angle) / scan.config.angle_increment + 0.5);
      if (idx >= 0 && idx < count)
      {
        if (p.range >= range_min_ && p.range <= range_max_)
        {
          msg.ranges[idx] = p.range;
          msg.intensities[idx] = p.intensity;
        }
      }
    }

    scan_pub_->publish(msg);
  }

  // 멤버
  CYdLidar laser_;
  std::string port_;
  int baudrate_;
  std::string frame_id_;
  double frequency_;
  double range_min_;
  double range_max_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::thread scan_thread_;
  bool running_ = true;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ydlidar::os_init();
  auto node = std::make_shared<YdlidarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
