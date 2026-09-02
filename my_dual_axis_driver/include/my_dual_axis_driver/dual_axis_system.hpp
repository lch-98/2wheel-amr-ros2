#ifndef MY_DUAL_AXIS_DRIVER__DUAL_AXIS_SYSTEM_HPP_
#define MY_DUAL_AXIS_DRIVER__DUAL_AXIS_SYSTEM_HPP_

#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "canopen_core/device_container.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "my_dual_axis_driver/dual_axis_driver.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_dual_axis_driver
{

/**
 * @brief 조인트 하나(=축 하나)에 대한 데이터
 *
 * RobotSystem 의 Cia402Data 에 해당하지만, 두 조인트가 같은 node_id 를
 * 공유하고 axis 로 구분된다는 점이 다르다.
 */
struct AxisJointData
{
  uint8_t node_id{0};
  size_t axis{0};                 ///< 0 = 0x6000, 1 = 0x6800
  std::string joint_name;
  double wheel_radius{0.0325};   ///< rad/s ↔ m/s 변환용

  std::shared_ptr<ros2_canopen::DualAxisDriver> driver;  ///< 두 조인트가 같은 인스턴스 공유

  // 상태 (하드웨어 → 컨트롤러)
  double actual_position{std::numeric_limits<double>::quiet_NaN()};
  double actual_velocity{std::numeric_limits<double>::quiet_NaN()};

  // 명령 (컨트롤러 → 하드웨어)
  double target_velocity{std::numeric_limits<double>::quiet_NaN()};

  void read_state()
  {
    if (!driver) return;
    // 드라이버는 m/s, ros2_control 조인트는 rad/s
    actual_velocity = driver->get_axis_speed(axis) / wheel_radius;
    actual_position = 0.0;   // TODO
  }

  void write_target()
  {
    if (!driver) return;
    if (std::isnan(target_velocity)) return;
    // 조인트 각속도(rad/s) → 바퀴 선속도(m/s)
    driver->set_axis_target(axis, target_velocity * wheel_radius);
  }
};

class DualAxisSystem : public hardware_interface::SystemInterface
{
public:
  DualAxisSystem() : hardware_interface::SystemInterface() {}
  ~DualAxisSystem() = default;

  /// URDF 의 <hardware>/<joint> 파라미터를 읽는다
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /// DeviceContainer 를 만들고 CANopen 스택을 스레드로 띄운다
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  /// 각 축을 CiA402 Operation Enabled + Profile Velocity 로 만든다
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ros2_canopen::DeviceContainer> device_container_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::unique_ptr<std::thread> spin_thread_;
  std::unique_ptr<std::thread> init_thread_;

  std::vector<AxisJointData> joint_data_;

  // URDF <hardware> 파라미터
  std::string bus_config_;
  std::string master_config_;
  std::string master_bin_;
  std::string can_interface_;

  rclcpp::Logger logger_ = rclcpp::get_logger("dual_axis_system");

private:
  void spin();
  void clean();
  void initDeviceContainer();
};

}  // namespace my_dual_axis_driver

#endif  // MY_DUAL_AXIS_DRIVER__DUAL_AXIS_SYSTEM_HPP_
