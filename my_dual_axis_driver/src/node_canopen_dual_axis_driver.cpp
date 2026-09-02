#include "my_dual_axis_driver/node_interfaces/node_canopen_dual_axis_driver_impl.hpp"

namespace ros2_canopen
{
namespace node_interfaces
{

template class NodeCanopenDualAxisDriver<rclcpp::Node>;
template class NodeCanopenDualAxisDriver<rclcpp_lifecycle::LifecycleNode>;

}  // namespace node_interfaces
}  // namespace ros2_canopen