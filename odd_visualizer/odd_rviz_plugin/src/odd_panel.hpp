#ifndef ODD_PANEL_HPP_
#define ODD_PANEL_HPP_


#include <regex>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget> 
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <scenery_msgs/msg/odd_elements.hpp>
#include <scenery_msgs/srv/teleoperation.hpp>


/* TODO: populate the panel function
Attributes that are available in the sample map from AW tutorial 
[odd_visualizer_node-1] LaneID: 9463; attribute: location; urban
[odd_visualizer_node-1] LaneID: 9463; attribute: one_way; yes
[odd_visualizer_node-1] LaneID: 9463; attribute: speed_limit; 30
[odd_visualizer_node-1] LaneID: 9463; attribute: subtype; road
[odd_visualizer_node-1] LaneID: 9463; attribute: type; lanelet
*/


namespace rviz_plugins
{
class ODDPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ODDPanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:  // NOLINT for Qt
  // void onClickAutowareEngage();
  // void onClickVelocityLimit();
  // void onClickGateMode();
  // void onClickPathChangeApproval();
  // void onClickEmergencyButton();

  void onClickODDTeleoperation();

protected:
  void onODDSub(const scenery_msgs::msg::ODDElements::ConstSharedPtr msg);


  // void onGateMode(const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg);
  // void onSelectorMode(
  //   const tier4_control_msgs::msg::ExternalCommandSelectorMode::ConstSharedPtr msg);
  // void onAutowareState(const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr msg);
  // void onShift(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr msg);
  // void onEmergencyStatus(const tier4_external_api_msgs::msg::Emergency::ConstSharedPtr msg);
  // void onEngageStatus(const tier4_external_api_msgs::msg::EngageStatus::ConstSharedPtr msg);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<scenery_msgs::msg::ODDElements>::SharedPtr sub_odd_elements_;
  rclcpp::Client<scenery_msgs::srv::Teleoperation>::SharedPtr client_teleoperation_;



  // rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr pub_velocity_limit_;

  QLabel * current_lanelet_label_ptr_;
  QLabel * history_lanelet_label_ptr_;
  QLabel * next_lanelet_label_ptr_;

  QTableWidget * current_lanelet_attributes_table_prt_;
  QTableWidget * history_lanelet_attributes_table_prt_;
  QTableWidget * next_lanelet_attributes_table_prt_;


  QPushButton * teleoperation_button_ptr_;
  bool teleoperation_state_{false};

  // QLabel * gate_mode_label_ptr_;
  // QLabel * selector_mode_label_ptr_;
  // QLabel * autoware_state_label_ptr_;
  // QLabel * gear_label_ptr_;
  // QLabel * engage_status_label_ptr_;
  // QPushButton * engage_button_ptr_;
  // QPushButton * velocity_limit_button_ptr_;
  // QPushButton * gate_mode_button_ptr_;
  // QPushButton * path_change_approval_button_ptr_;
  // QPushButton * emergency_button_ptr_;

  // bool current_engage_{false};
  // bool current_emergency_{false};
};

}  // namespace rviz_plugins

#endif  // ODD_PANEL_HPP_
