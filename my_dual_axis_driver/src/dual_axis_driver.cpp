#include "my_dual_axis_driver/dual_axis_driver.hpp"
// 템플릿 구현을 이 번역 단위에서도 보이게 한다 (생성자 인스턴스화)
#include "my_dual_axis_driver/node_interfaces/node_canopen_dual_axis_driver_impl.hpp"
#include "rclcpp_components/register_node_macro.hpp"

/*
  Dual Axis Driver - Empty Shell Implement & Register
  껍데기 구현 및 등록 부분
*/

namespace ros2_canopen
{
DualAxisDriver::DualAxisDriver(rclcpp::NodeOptions node_options) : CanopenDriver(node_options)
{
  node_driver_ =
    std::make_shared<node_interfaces::NodeCanopenDualAxisDriver<rclcpp::Node>>(this);
  this->node_canopen_driver_ =
    std::static_pointer_cast<node_interfaces::NodeCanopenDriverInterface>(node_driver_);
}
}  // namespace ros2_canopen

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::DualAxisDriver)
