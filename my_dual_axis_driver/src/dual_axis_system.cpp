#include "my_dual_axis_driver/dual_axis_system.hpp"

#include <chrono>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_dual_axis_driver
{

// ───────────────────────── on_init ─────────────────────────

hardware_interface::CallbackReturn DualAxisSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // <hardware> 태그의 파라미터
  try
  {
    bus_config_ = info_.hardware_parameters.at("bus_config");
    master_config_ = info_.hardware_parameters.at("master_config");
    can_interface_ = info_.hardware_parameters.at("can_interface_name");
  }
  catch (const std::out_of_range & e)
  {
    RCLCPP_ERROR(
      logger_, "Missing hardware parameter. Need bus_config, master_config, can_interface_name");
    return hardware_interface::CallbackReturn::ERROR;
  }
  // master_bin 은 선택
  auto it = info_.hardware_parameters.find("master_bin");
  master_bin_ = (it != info_.hardware_parameters.end() && it->second != "\"\"") ? it->second : "";

  RCLCPP_INFO(
    logger_, "bus_config=%s can_interface=%s", bus_config_.c_str(), can_interface_.c_str());

  // <joint> 태그마다 node_id 와 axis 를 읽는다
  joint_data_.clear();
  for (auto & joint : info_.joints)
  {
    AxisJointData data;
    data.joint_name = joint.name;

    try
    {
      data.node_id = static_cast<uint8_t>(std::stoi(joint.parameters.at("node_id")));
      data.axis = static_cast<size_t>(std::stoi(joint.parameters.at("axis")));

      // wheel_radius 는 선택 파라미터 (없으면 기본값 0.0325 유지)
      auto wr = joint.parameters.find("wheel_radius");
      if (wr != joint.parameters.end())
      {
        data.wheel_radius = std::stod(wr->second);
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        logger_, "Joint '%s' needs both 'node_id' and 'axis' parameters", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      logger_, "joint '%s' -> node_id=%u axis=%zu wheel_radius=%.4f",
      data.joint_name.c_str(), data.node_id, data.axis, data.wheel_radius);
    joint_data_.push_back(data);
  }

  if (joint_data_.empty())
  {
    RCLCPP_ERROR(logger_, "No joints defined");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ───────────────────────── on_configure ─────────────────────────

void DualAxisSystem::spin()
{
  executor_->add_node(device_container_);
  executor_->spin();
  executor_->remove_node(device_container_);
}

void DualAxisSystem::initDeviceContainer()
{
  device_container_->init(can_interface_, master_config_, bus_config_, master_bin_);
}

hardware_interface::CallbackReturn DualAxisSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  device_container_ = std::make_shared<ros2_canopen::DeviceContainer>(executor_);

  spin_thread_ = std::make_unique<std::thread>(&DualAxisSystem::spin, this);
  init_thread_ = std::make_unique<std::thread>(&DualAxisSystem::initDeviceContainer, this);

  if (init_thread_->joinable())
  {
    init_thread_->join();
  }
  else
  {
    RCLCPP_ERROR(logger_, "Could not join init thread");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 등록된 드라이버에서 우리 DualAxisDriver 를 찾아 각 조인트에 연결
  auto drivers = device_container_->get_registered_drivers();
  for (auto & data : joint_data_)
  {
    auto found = drivers.find(static_cast<uint16_t>(data.node_id));
    if (found == drivers.end())
    {
      RCLCPP_ERROR(logger_, "No driver for node_id %u", data.node_id);
      return hardware_interface::CallbackReturn::ERROR;
    }

    auto casted = std::dynamic_pointer_cast<ros2_canopen::DualAxisDriver>(found->second);
    if (!casted)
    {
      RCLCPP_ERROR(
        logger_, "Driver for node_id %u is not a DualAxisDriver. Check bus.yml 'driver' entry.",
        data.node_id);
      return hardware_interface::CallbackReturn::ERROR;
    }

    data.driver = casted;
    RCLCPP_INFO(
      logger_, "joint '%s' bound to driver node_id=%u axis=%zu", data.joint_name.c_str(),
      data.node_id, data.axis);
  }

  RCLCPP_INFO(logger_, "configured: %zu joints", joint_data_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ───────────────────────── 인터페이스 노출 ─────────────────────────

std::vector<hardware_interface::StateInterface> DualAxisSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & data : joint_data_)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      data.joint_name, hardware_interface::HW_IF_POSITION, &data.actual_position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      data.joint_name, hardware_interface::HW_IF_VELOCITY, &data.actual_velocity));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DualAxisSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & data : joint_data_)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      data.joint_name, hardware_interface::HW_IF_VELOCITY, &data.target_velocity));
  }
  return command_interfaces;
}

// ───────────────────────── activate / deactivate ─────────────────────────

hardware_interface::CallbackReturn DualAxisSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & data : joint_data_)
  {
    if (!data.driver)
    {
      RCLCPP_ERROR(logger_, "No driver for joint '%s'", data.joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!data.driver->init_axis(data.axis))
    {
      RCLCPP_ERROR(logger_, "init_axis(%zu) failed", data.axis);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!data.driver->set_axis_mode(data.axis, 3))   // 3 = Profile Velocity
    {
      RCLCPP_ERROR(logger_, "set_axis_mode(%zu, 3) failed", data.axis);
      return hardware_interface::CallbackReturn::ERROR;
    }

    data.target_velocity = 0.0;
    RCLCPP_INFO(logger_, "joint '%s' (axis %zu) enabled", data.joint_name.c_str(), data.axis);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DualAxisSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & data : joint_data_)
  {
    if (data.driver)
    {
      data.driver->halt_axis(data.axis);
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ───────────────────────── read / write ─────────────────────────

hardware_interface::return_type DualAxisSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto & data : joint_data_)
  {
    data.read_state();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DualAxisSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto & data : joint_data_)
  {
    data.write_target();
  }
  return hardware_interface::return_type::OK;
}

// ───────────────────────── cleanup / shutdown ─────────────────────────

void DualAxisSystem::clean()
{
  if (device_container_)
  {
    device_container_->shutdown();
  }
  if (executor_)
  {
    executor_->cancel();
  }
  if (spin_thread_ && spin_thread_->joinable())
  {
    spin_thread_->join();
  }
  device_container_.reset();
  executor_.reset();
  spin_thread_.reset();
  init_thread_.reset();
}

hardware_interface::CallbackReturn DualAxisSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  clean();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DualAxisSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  clean();
  return hardware_interface::CallbackReturn::SUCCESS;
}

}  // namespace my_dual_axis_driver

PLUGINLIB_EXPORT_CLASS(
  my_dual_axis_driver::DualAxisSystem, hardware_interface::SystemInterface)
