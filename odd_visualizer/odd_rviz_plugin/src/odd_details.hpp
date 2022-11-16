#ifndef ODD_DETAILS_HPP_
#define ODD_DETAILS_HPP_

#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

// #include <autoware_auto_system_msgs/msg/autoware_state.hpp>
// #include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
// #include <tier4_control_msgs/msg/external_command_selector_mode.hpp>
// #include <tier4_control_msgs/msg/gate_mode.hpp>
// #include <tier4_external_api_msgs/msg/emergency.hpp>
// #include <tier4_external_api_msgs/msg/engage_status.hpp>
// #include <tier4_external_api_msgs/srv/engage.hpp>
// #include <tier4_external_api_msgs/srv/set_emergency.hpp>
// #include <tier4_planning_msgs/msg/approval.hpp>
// #include <tier4_planning_msgs/msg/velocity_limit.hpp>

namespace rviz_plugins
{
class ODDElementsPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ODDElementsPanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:  // NOLINT for Qt
  void onClickAutowareEngage();
  void onClickVelocityLimit();
  void onClickGateMode();
  void onClickPathChangeApproval();
  void onClickEmergencyButton();

protected:
  void onGateMode(const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg);
  void onSelectorMode(
    const tier4_control_msgs::msg::ExternalCommandSelectorMode::ConstSharedPtr msg);
  void onAutowareState(const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr msg);
  void onShift(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr msg);
  void onEmergencyStatus(const tier4_external_api_msgs::msg::Emergency::ConstSharedPtr msg);
  void onEngageStatus(const tier4_external_api_msgs::msg::EngageStatus::ConstSharedPtr msg);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<tier4_control_msgs::msg::GateMode>::SharedPtr sub_gate_mode_;
  rclcpp::Subscription<tier4_control_msgs::msg::ExternalCommandSelectorMode>::SharedPtr
    sub_selector_mode_;
  rclcpp::Subscription<autoware_auto_system_msgs::msg::AutowareState>::SharedPtr
    sub_autoware_state_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr sub_gear_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::EngageStatus>::SharedPtr sub_engage_;

  rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr client_engage_;
  rclcpp::Client<tier4_external_api_msgs::srv::SetEmergency>::SharedPtr client_emergency_stop_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::Emergency>::SharedPtr sub_emergency_;

  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr pub_velocity_limit_;
  rclcpp::Publisher<tier4_control_msgs::msg::GateMode>::SharedPtr pub_gate_mode_;
  rclcpp::Publisher<tier4_planning_msgs::msg::Approval>::SharedPtr pub_path_change_approval_;

  QLabel * gate_mode_label_ptr_;
  QLabel * selector_mode_label_ptr_;
  QLabel * autoware_state_label_ptr_;
  QLabel * gear_label_ptr_;
  QLabel * engage_status_label_ptr_;
  QPushButton * engage_button_ptr_;
  QPushButton * velocity_limit_button_ptr_;
  QPushButton * gate_mode_button_ptr_;
  QPushButton * path_change_approval_button_ptr_;
  QSpinBox * pub_velocity_limit_input_;
  QPushButton * emergency_button_ptr_;

  bool current_engage_{false};
  bool current_emergency_{false};
};

}  // namespace rviz_plugins

#endif  // AUTOWARE_STATE_PANEL_HPP_




#endif // ODD_DETAILS_HPP_