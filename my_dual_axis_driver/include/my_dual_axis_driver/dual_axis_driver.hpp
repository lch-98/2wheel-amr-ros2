#ifndef DUAL_AXIS_DRIVER_HPP_
#define DUAL_AXIS_DRIVER_HPP_

#include "canopen_core/driver_node.hpp"
#include "my_dual_axis_driver/node_interfaces/node_canopen_dual_axis_driver.hpp"

/*
  Dual Axis Driver - Empty Shell Define
  다축 드라이버의 껍데기 선언 부분
*/

namespace ros2_canopen
{

/**
 * @brief 2축 CiA402 드라이버 (ROS2 컴포넌트)
 *
 * Cia402Driver와 동일하게 CanopenDriver를 직접 상속하고,
 * 실제 로직은 NodeCanopenDualAxisDriver에 위임한다.
 */
class DualAxisDriver : public ros2_canopen::CanopenDriver
{
  std::shared_ptr<node_interfaces::NodeCanopenDualAxisDriver<rclcpp::Node>> node_driver_;

public:
  explicit DualAxisDriver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

  bool init_axis(size_t axis) { return node_driver_->init_axis(axis); }

  bool set_axis_mode(size_t axis, int8_t mode) { return node_driver_->set_axis_mode(axis, mode); }

  bool set_axis_target(size_t axis, double velocity)
  {
    return node_driver_->set_axis_target(axis, velocity);
  }

  bool halt_axis(size_t axis) { return node_driver_->halt_axis(axis); }

  double get_axis_speed(size_t axis) { return node_driver_->get_axis_speed(axis); }

  uint16_t get_axis_statusword(size_t axis) { return node_driver_->get_axis_statusword(axis); }
};

}  // namespace ros2_canopen

#endif  // DUAL_AXIS_DRIVER_HPP_
