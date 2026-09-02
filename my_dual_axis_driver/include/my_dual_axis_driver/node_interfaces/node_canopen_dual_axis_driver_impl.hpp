#ifndef NODE_CANOPEN_DUAL_AXIS_DRIVER_IMPL_HPP_
#define NODE_CANOPEN_DUAL_AXIS_DRIVER_IMPL_HPP_

#include "my_dual_axis_driver/node_interfaces/node_canopen_dual_axis_driver.hpp"

/*
  Dual Axis Driver - Brain Implement
  다축 드라이버의 두뇌 구현 부분
  템플릿 클래스는 구현이 헤더에 있어야 컴파일러가 인스턴스화할 수 있음
*/

namespace ros2_canopen
{
namespace node_interfaces
{

template <class NODETYPE>
NodeCanopenDualAxisDriver<NODETYPE>::NodeCanopenDualAxisDriver(NODETYPE * node)
: NodeCanopenProxyDriver<NODETYPE>(node)
{
  axes_[0].base = 0x6000;
  axes_[0].joint_name = "wheel_right_joint";
  axes_[1].base = 0x6800;
  axes_[1].joint_name = "wheel_left_joint";
}

// ─────────────────────────── 라이프사이클 ───────────────────────────

template <class NODETYPE>
void NodeCanopenDualAxisDriver<NODETYPE>::init(bool called_from_base)
{
  (void)called_from_base;
  NodeCanopenProxyDriver<NODETYPE>::init(false);   // Proxy 의 SDO/NMT 서비스 확보

  // this->node->template: 클래스가 템플릿일 때, 컴파일러에게 create_service는 템플릿 함수임을 명시 (C++ 문법 요구사항)
  joint_state_pub_ =
    this->node_->template create_publisher<sensor_msgs::msg::JointState>("~/axis_states", 10); 

  for (size_t i = 0; i < AXIS_COUNT; ++i)
  {
    const std::string ns = "axis" + std::to_string(i) + "/";

    init_services_[i] = this->node_->template create_service<std_srvs::srv::Trigger>(
      ns + "init",
      [this, i](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        res->success = this->init_axis(i);
        res->message = res->success ? "axis enabled" : "init failed";
      });

    velocity_mode_services_[i] = this->node_->template create_service<std_srvs::srv::Trigger>(
      ns + "velocity_mode",
      [this, i](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        res->success = this->set_axis_mode(i, MODE_PROFILED_VELOCITY);
        res->message = res->success ? "profiled velocity" : "mode switch failed";
      });

    halt_services_[i] = this->node_->template create_service<std_srvs::srv::Trigger>(
      ns + "halt",
      [this, i](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        res->success = this->halt_axis(i);
        res->message = res->success ? "halted" : "halt failed";
      });

    target_services_[i] =
      this->node_->template create_service<canopen_interfaces::srv::COTargetDouble>(
        ns + "target",
        [this, i](
          const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr req,
          canopen_interfaces::srv::COTargetDouble::Response::SharedPtr res)
        {
          res->success = this->set_axis_target(i, req->target);
        });
  }
}

template <class NODETYPE>
void NodeCanopenDualAxisDriver<NODETYPE>::configure(bool called_from_base)
{
  (void)called_from_base;
  NodeCanopenProxyDriver<NODETYPE>::configure(false);

  try
  {
    scale_vel_to_dev_ = this->config_["scale_vel_to_dev"].template as<double>();
    scale_vel_from_dev_ = this->config_["scale_vel_from_dev"].template as<double>();
  }
  catch (...)
  {
    RCLCPP_WARN(
      this->node_->get_logger(), "scale_vel_* not in bus.yml, using defaults (1000 / 0.001)");
  }
  RCLCPP_INFO(
    this->node_->get_logger(), "dual axis driver: to_dev=%f from_dev=%f", scale_vel_to_dev_,
    scale_vel_from_dev_);
}

template <class NODETYPE>
void NodeCanopenDualAxisDriver<NODETYPE>::activate(bool called_from_base)
{
  (void)called_from_base;
  NodeCanopenProxyDriver<NODETYPE>::activate(false);
  RCLCPP_INFO(this->node_->get_logger(), "dual axis driver activated (axis0=0x6000, axis1=0x6800)");
}

template <class NODETYPE>
void NodeCanopenDualAxisDriver<NODETYPE>::deactivate(bool called_from_base)
{
  (void)called_from_base;
  for (size_t i = 0; i < AXIS_COUNT; ++i)
  {
    halt_axis(i);   // 내려가기 전에 반드시 정지
  }
  NodeCanopenProxyDriver<NODETYPE>::deactivate(false);
}

// ─────────────────────────── 축 제어 API ───────────────────────────

template <class NODETYPE>
bool NodeCanopenDualAxisDriver<NODETYPE>::init_axis(size_t axis)
{
  if (axis >= AXIS_COUNT) return false;
  auto & ax = axes_[axis];

  try
  {
    // 06 → 07 → 0F 순서로 상태머신 기동
    this->lely_driver_->template universal_set_value<uint16_t>(
      ax.idx(cia402_offset::CONTROLWORD), 0, cia402_cw::SHUTDOWN);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    this->lely_driver_->template universal_set_value<uint16_t>(
      ax.idx(cia402_offset::CONTROLWORD), 0, cia402_cw::SWITCH_ON);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    this->lely_driver_->template universal_set_value<uint16_t>(
      ax.idx(cia402_offset::CONTROLWORD), 0, cia402_cw::ENABLE_OP);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->node_->get_logger(), "axis%zu init failed: %s", axis, e.what());
    return false;
  }

  const uint16_t sw = ax.statusword.load() & cia402_sw::MASK;
  ax.enabled = (sw == cia402_sw::OPERATION_ENABLED);
  RCLCPP_INFO(
    this->node_->get_logger(), "axis%zu init: statusword=0x%04X enabled=%d", axis,
    ax.statusword.load(), static_cast<int>(ax.enabled.load()));
  return true;
}

template <class NODETYPE>
bool NodeCanopenDualAxisDriver<NODETYPE>::set_axis_mode(size_t axis, int8_t mode)
{
  if (axis >= AXIS_COUNT) return false;
  auto & ax = axes_[axis];
  try
  {
    this->lely_driver_->template universal_set_value<int8_t>(
      ax.idx(cia402_offset::MODE_OP), 0, mode);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->node_->get_logger(), "axis%zu mode failed: %s", axis, e.what());
    return false;
  }
  RCLCPP_INFO(this->node_->get_logger(), "axis%zu mode -> %d", axis, static_cast<int>(mode));
  return true;
}

template <class NODETYPE>
bool NodeCanopenDualAxisDriver<NODETYPE>::set_axis_target(size_t axis, double velocity)
{
  if (axis >= AXIS_COUNT) return false;
  auto & ax = axes_[axis];

  const auto raw = static_cast<int32_t>(velocity * scale_vel_to_dev_);
  try
  {
    // 0x60FF(축0) 또는 0x68FF(축1) — 런타임 인덱스이므로 축 제약이 없다
    this->lely_driver_->template universal_set_value<int32_t>(
      ax.idx(cia402_offset::VEL_TARGET), 0, raw);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->node_->get_logger(), "axis%zu target failed: %s", axis, e.what());
    return false;
  }
  ax.target = velocity;
  return true;
}

template <class NODETYPE>
bool NodeCanopenDualAxisDriver<NODETYPE>::halt_axis(size_t axis)
{
  if (axis >= AXIS_COUNT) return false;
  auto & ax = axes_[axis];
  bool ok = set_axis_target(axis, 0.0);
  try
  {
    this->lely_driver_->template universal_set_value<uint16_t>(
      ax.idx(cia402_offset::CONTROLWORD), 0, cia402_cw::SHUTDOWN);
  }
  catch (const std::exception &)
  {
    ok = false;
  }
  ax.enabled = false;
  return ok;
}

template <class NODETYPE>
double NodeCanopenDualAxisDriver<NODETYPE>::get_axis_speed(size_t axis) const
{
  if (axis >= AXIS_COUNT) return 0.0;
  return static_cast<double>(axes_[axis].vel_actual_raw.load()) * scale_vel_from_dev_;
}

template <class NODETYPE>
uint16_t NodeCanopenDualAxisDriver<NODETYPE>::get_axis_statusword(size_t axis) const
{
  if (axis >= AXIS_COUNT) return 0;
  return axes_[axis].statusword.load();
}

// ─────────────────────────── 콜백 ───────────────────────────

template <class NODETYPE>
void NodeCanopenDualAxisDriver<NODETYPE>::on_rpdo(COData data)
{
  NodeCanopenProxyDriver<NODETYPE>::on_rpdo(data);   // Proxy 의 rpdo 퍼블리시 유지

  for (size_t i = 0; i < AXIS_COUNT; ++i)
  {
    auto & ax = axes_[i];
    if (data.index_ == ax.idx(cia402_offset::STATUSWORD))
    {
      ax.statusword = static_cast<uint16_t>(data.data_);
    }
    else if (data.index_ == ax.idx(cia402_offset::MODE_DISPLAY))
    {
      ax.mode_display = static_cast<int8_t>(data.data_);
    }
    else if (data.index_ == ax.idx(cia402_offset::VEL_ACTUAL))
    {
      ax.vel_actual_raw = static_cast<int32_t>(data.data_);
    }
  }
}

template <class NODETYPE>
void NodeCanopenDualAxisDriver<NODETYPE>::on_nmt(canopen::NmtState nmt_state)
{
  NodeCanopenProxyDriver<NODETYPE>::on_nmt(nmt_state);

  // 슬레이브가 Operational을 벗어나면 목표속도를 0 으로 (GND 단선 등 대비)
  if (nmt_state != canopen::NmtState::START)
  {
    for (auto & ax : axes_)
    {
      ax.target = 0.0;
      ax.enabled = false;
    }
    RCLCPP_WARN(this->node_->get_logger(), "slave left operational state - axes disabled");
  }
}

template <class NODETYPE>
void NodeCanopenDualAxisDriver<NODETYPE>::poll_timer_callback()
{
  NodeCanopenProxyDriver<NODETYPE>::poll_timer_callback();
  publish_joint_states();
}

template <class NODETYPE>
void NodeCanopenDualAxisDriver<NODETYPE>::publish_joint_states()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->node_->now();
  for (size_t i = 0; i < AXIS_COUNT; ++i)
  {
    msg.name.push_back(axes_[i].joint_name);
    msg.velocity.push_back(get_axis_speed(i));
    msg.position.push_back(0.0);   // TODO: 0x6064/0x6864 매핑 후 채우기
  }
  joint_state_pub_->publish(msg);
}

}  // namespace node_interfaces
}  // namespace ros2_canopen

#endif  // NODE_CANOPEN_DUAL_AXIS_DRIVER_IMPL_HPP_
