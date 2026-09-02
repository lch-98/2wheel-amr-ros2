#ifndef NODE_CANOPEN_DUAL_AXIS_DRIVER_HPP_
#define NODE_CANOPEN_DUAL_AXIS_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "canopen_interfaces/srv/co_target_double.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

/*
  Dual Axis Driver - Brain Define
  다축 드라이버의 두뇌 선언 부분
*/

namespace ros2_canopen
{
namespace node_interfaces
{

/// CiA402 오브젝트 오프셋 (축 base 에 더해서 사용)
namespace cia402_offset
{
constexpr uint16_t CONTROLWORD   = 0x040;  // 0x6040 / 0x6840
constexpr uint16_t STATUSWORD    = 0x041;  // 0x6041 / 0x6841
constexpr uint16_t MODE_OP       = 0x060;  // 0x6060 / 0x6860
constexpr uint16_t MODE_DISPLAY  = 0x061;  // 0x6061 / 0x6861
constexpr uint16_t VEL_ACTUAL    = 0x06C;  // 0x606C / 0x686C
constexpr uint16_t VEL_TARGET    = 0x0FF;  // 0x60FF / 0x68FF
}  // namespace cia402_offset

/// CiA402 Controlword 값 (STM32 펌웨어와 동일)
namespace cia402_cw
{
constexpr uint16_t SHUTDOWN      = 0x0006;
constexpr uint16_t SWITCH_ON     = 0x0007;
constexpr uint16_t ENABLE_OP     = 0x000F;
constexpr uint16_t DISABLE_VOLT  = 0x0000;
}  // namespace cia402_cw

/// CiA402 Statusword 마스크/값 (하위 비트만 비교)
namespace cia402_sw
{
constexpr uint16_t MASK                = 0x006F;
constexpr uint16_t SWITCH_ON_DISABLED  = 0x0040;
constexpr uint16_t READY_TO_SWITCH_ON  = 0x0021;
constexpr uint16_t SWITCHED_ON         = 0x0023;
constexpr uint16_t OPERATION_ENABLED   = 0x0027;
}  // namespace cia402_sw

constexpr int8_t MODE_PROFILED_VELOCITY = 3;

/**
 * @brief 축 하나의 런타임 상태
 *
 * base 만 다르게 주면 (0x6000 / 0x6800) 동일 로직으로 두 축을 다룰 수 있다.
 */
struct AxisContext
{
  uint16_t base{0x6000};              ///< 0x6000(축1) 또는 0x6800(축2)
  std::string joint_name{"wheel_joint"};

  // 슬레이브에서 올라온 값 (TPDO 수신으로 갱신)
  std::atomic<uint16_t> statusword{0};
  std::atomic<int8_t> mode_display{0};
  std::atomic<int32_t> vel_actual_raw{0};

  // 마스터가 내려보낸 값
  std::atomic<double> target{0.0};
  std::atomic<bool> enabled{false};

  uint16_t idx(uint16_t offset) const { return static_cast<uint16_t>(base + offset); }
};

template <class NODETYPE>
class NodeCanopenDualAxisDriver : public NodeCanopenProxyDriver<NODETYPE>
{
  // static_assert > "NODETYPE은 일반 노드거나 라이프사이클 노드여야 한다"는 컴파일 타임 검사
  static_assert(
    std::is_base_of<rclcpp::Node, NODETYPE>::value ||
      std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
    "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

protected:
  static constexpr size_t AXIS_COUNT = 2;
  std::array<AxisContext, AXIS_COUNT> axes_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // 축별 서비스 (axis0 / axis1 네임스페이스)
  std::array<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr, AXIS_COUNT> init_services_;
  std::array<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr, AXIS_COUNT> velocity_mode_services_;
  std::array<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr, AXIS_COUNT> halt_services_;
  std::array<rclcpp::Service<canopen_interfaces::srv::COTargetDouble>::SharedPtr, AXIS_COUNT>
    target_services_;

  // bus.yml 에서 읽어오는 스케일
  double scale_vel_to_dev_{1000.0};
  double scale_vel_from_dev_{0.001};

  /// 주기 루프: 상태 갱신 + JointState 발행
  virtual void poll_timer_callback() override;

  /// 슬레이브 TPDO 수신 (마스터 입장에서는 RPDO)
  virtual void on_rpdo(COData data) override;

  /// NMT 상태 변화 감지 → 이탈 시 안전 정지
  virtual void on_nmt(canopen::NmtState nmt_state) override;

  void publish_joint_states();

public:
  explicit NodeCanopenDualAxisDriver(NODETYPE * node);

  virtual void init(bool called_from_base) override;
  virtual void configure(bool called_from_base) override;
  virtual void activate(bool called_from_base) override;
  virtual void deactivate(bool called_from_base) override;

  // ── 축 제어 API (껍데기가 위임해서 호출) ──
  bool init_axis(size_t axis);
  bool set_axis_mode(size_t axis, int8_t mode);
  bool set_axis_target(size_t axis, double velocity);
  bool halt_axis(size_t axis);

  double get_axis_speed(size_t axis) const;
  uint16_t get_axis_statusword(size_t axis) const;
};

}  // namespace node_interfaces
}  // namespace ros2_canopen

#endif  // NODE_CANOPEN_DUAL_AXIS_DRIVER_HPP_
