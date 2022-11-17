
#include "odd_panel.hpp"

#include <QHBoxLayout>
#include <QString>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>

inline std::string Bool2String(const bool var) { return var ? "True" : "False"; }

using std::placeholders::_1;

namespace rviz_plugins
{
ODDPanel::ODDPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  item0 = new QTableWidgetItem("Init");
  item1 = new QTableWidgetItem("Init");

  auto * up_layout = new QHBoxLayout;

  // current lanelet
  current_lanelet_attributes_table_prt_ = new QTableWidget(this);
  current_lanelet_label_ptr_ = new QLabel("Current Lanelet");
  current_lanelet_label_ptr_->setAlignment(Qt::AlignLeft);
  auto * current_lanelet_layout = new QVBoxLayout;
  current_lanelet_layout->addWidget(current_lanelet_label_ptr_);
  current_lanelet_layout->addWidget(current_lanelet_attributes_table_prt_);

  // history lanelet
  history_lanelet_attributes_table_prt_ = new QTableWidget(this);
  auto * history_lanelet_label_ptr_ = new QLabel("History Lanelet");
  history_lanelet_label_ptr_->setAlignment(Qt::AlignLeft);
  auto * history_lanelet_layout = new QVBoxLayout;
  history_lanelet_layout->addWidget(history_lanelet_label_ptr_);
  history_lanelet_layout->addWidget(history_lanelet_attributes_table_prt_);

  up_layout->addLayout(current_lanelet_layout);
  up_layout->addLayout(history_lanelet_layout);

  // next lanelet
  auto * down_layout = new QVBoxLayout;

  auto * next_lanelet_attributes_table_prt_ = new QTableWidget(this);
  auto * next_lanelet_label_ptr_ = new QLabel("Next Lanelet");
  next_lanelet_label_ptr_->setAlignment(Qt::AlignLeft);
  down_layout->addWidget(next_lanelet_label_ptr_);
  down_layout->addWidget(next_lanelet_attributes_table_prt_);

  // teleoperation button
  teleoperation_button_ptr_ = new QPushButton("Teleoperation");
  connect(teleoperation_button_ptr_, SIGNAL(clicked()), SLOT(onClickODDTeleoperation()));

  // Layout
  auto * v_layout = new QVBoxLayout;
  v_layout->addLayout(up_layout);
  v_layout->addLayout(down_layout);
  v_layout->addWidget(teleoperation_button_ptr_);
  setLayout(v_layout);
}

void ODDPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  sub_odd_elements_ = raw_node_->create_subscription<scenery_msgs::msg::ODDElements>(
    "/odd_parameter/odd_elements", 10, std::bind(&ODDPanel::onODDSub, this, _1));

  client_teleoperation_ = raw_node_->create_client<scenery_msgs::srv::Teleoperation>(
    "/odd_parameter/teleoperation", rmw_qos_profile_services_default);

  // client_engage_ = raw_node_->create_client<tier4_external_api_msgs::srv::Engage>(
  //   "/api/external/set/engage", rmw_qos_profile_services_default);

  // client_emergency_stop_ = raw_node_->create_client<tier4_external_api_msgs::srv::SetEmergency>(
  //   "/api/autoware/set/emergency", rmw_qos_profile_services_default);

  // pub_velocity_limit_ = raw_node_->create_publisher<tier4_planning_msgs::msg::VelocityLimit>(
  //   "/planning/scenario_planning/max_velocity_default", rclcpp::QoS{1}.transient_local());

  // pub_gate_mode_ = raw_node_->create_publisher<tier4_control_msgs::msg::GateMode>(
  //   "/control/gate_mode_cmd", rclcpp::QoS{1}.transient_local());

  // pub_path_change_approval_ = raw_node_->create_publisher<tier4_planning_msgs::msg::Approval>(
  //   "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/"
  //   "path_change_approval",
  //   rclcpp::QoS{1}.transient_local());
}


void ODDPanel::onODDSub(const scenery_msgs::msg::ODDElements::ConstSharedPtr msg) {
  // std::list<QTableWidget*> tableList{current_lanelet_attributes_table_prt_,
  //                                    history_lanelet_attributes_table_prt_,
  //                                    next_lanelet_attributes_table_prt_};
  // size_t index = 0;
  // if (msg->laneletInfo.size() == 3) {
  //   for (auto & table : tableList) {
  //     size_t RowSize = msg->laneletInfo[index].attributes.size();
  //     table->setRowCount(RowSize);
  //     table->setColumnCount(2);
  //     size_t row = 0;
  //     for (auto & attr : msg->laneletInfo[index].attributes) {
  //       if (row < RowSize) {
  //         table->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(attr.attributeName)));
  //         table->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(attr.strValue)));
  //         row++;
  //       }
  //     }
  //     index++;
  //   }
  // }
  // int sizeLL = static_cast<int>(msg->laneletInfo.size());
  current_lanelet_attributes_table_prt_->setRowCount(msg->laneletInfo[0].attributes.size());
  current_lanelet_attributes_table_prt_->setColumnCount(2);
  int row = 0;
      for (auto & attr : msg->laneletInfo[0].attributes) {
        if (row < current_lanelet_attributes_table_prt_->rowCount()) {
          current_lanelet_attributes_table_prt_->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(attr.attributeName)));
          current_lanelet_attributes_table_prt_->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(attr.strValue)));
          row++;
        }
      }
  // current_lanelet_attributes_table_prt_->setItem(0, 0, item0);
  // current_lanelet_attributes_table_prt_->setItem(0, 1, item1);
  // item0->setText("some name");
  // // item0->setText(QString::fromStdString(msg->laneletInfo[0].attributes[0].attributeName));
  // item1->setText(QString::fromStdString(msg->laneletInfo[0].attributes[0].strValue));

}

void ODDPanel::onClickODDTeleoperation()
{
  using scenery_msgs::srv::Teleoperation;

  auto req = std::make_shared<Teleoperation::Request>();
  req->teleoperation = !teleoperation_state_;

  RCLCPP_INFO(raw_node_->get_logger(), "client request");

  if (!client_teleoperation_->service_is_ready()) {
    RCLCPP_INFO(raw_node_->get_logger(), "client is unavailable");
    return;
  }

  // client_teleoperation_->async_send_request(req, [this](rclcpp::Client<Teleoperation>::SharedFuture result) {
  //   RCLCPP_INFO(raw_node_->get_logger(), "response: %s", result.get()->strResponse);
  // });
}




// void ODDPanel::onGateMode(const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg)
// {
//   switch (msg->data) {
//     case tier4_control_msgs::msg::GateMode::AUTO:
//       gate_mode_label_ptr_->setText("AUTO");
//       gate_mode_label_ptr_->setStyleSheet("background-color: #00FF00;");
//       break;

//     case tier4_control_msgs::msg::GateMode::EXTERNAL:
//       gate_mode_label_ptr_->setText("EXTERNAL");
//       gate_mode_label_ptr_->setStyleSheet("background-color: #FFFF00;");
//       break;

//     default:
//       gate_mode_label_ptr_->setText("UNKNOWN");
//       gate_mode_label_ptr_->setStyleSheet("background-color: #FF0000;");
//       break;
//   }
// }



// void ODDPanel::onSelectorMode(
//   const tier4_control_msgs::msg::ExternalCommandSelectorMode::ConstSharedPtr msg)
// {
//   switch (msg->data) {
//     case tier4_control_msgs::msg::ExternalCommandSelectorMode::REMOTE:
//       selector_mode_label_ptr_->setText("REMOTE");
//       selector_mode_label_ptr_->setStyleSheet("background-color: #00FF00;");
//       break;

//     case tier4_control_msgs::msg::ExternalCommandSelectorMode::LOCAL:
//       selector_mode_label_ptr_->setText("LOCAL");
//       selector_mode_label_ptr_->setStyleSheet("background-color: #FFFF00;");
//       break;

//     case tier4_control_msgs::msg::ExternalCommandSelectorMode::NONE:
//       selector_mode_label_ptr_->setText("NONE");
//       selector_mode_label_ptr_->setStyleSheet("background-color: #FF0000;");
//       break;

//     default:
//       selector_mode_label_ptr_->setText("UNKNOWN");
//       selector_mode_label_ptr_->setStyleSheet("background-color: #FF0000;");
//       break;
//   }
// }

// void ODDPanel::onAutowareState(
//   const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr msg)
// {
//   if (msg->state == autoware_auto_system_msgs::msg::AutowareState::INITIALIZING) {
//     autoware_state_label_ptr_->setText("INITIALIZING");
//     autoware_state_label_ptr_->setStyleSheet("background-color: #FFFF00;");
//   } else if (msg->state == autoware_auto_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE) {
//     autoware_state_label_ptr_->setText("WAITING_FOR_ROUTE");
//     autoware_state_label_ptr_->setStyleSheet("background-color: #FFFF00;");
//   } else if (msg->state == autoware_auto_system_msgs::msg::AutowareState::PLANNING) {
//     autoware_state_label_ptr_->setText("PLANNING");
//     autoware_state_label_ptr_->setStyleSheet("background-color: #FFFF00;");
//   } else if (msg->state == autoware_auto_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE) {
//     autoware_state_label_ptr_->setText("WAITING_FOR_ENGAGE");
//     autoware_state_label_ptr_->setStyleSheet("background-color: #00FFFF;");
//   } else if (msg->state == autoware_auto_system_msgs::msg::AutowareState::DRIVING) {
//     autoware_state_label_ptr_->setText("DRIVING");
//     autoware_state_label_ptr_->setStyleSheet("background-color: #00FF00;");
//   } else if (msg->state == autoware_auto_system_msgs::msg::AutowareState::ARRIVED_GOAL) {
//     autoware_state_label_ptr_->setText("ARRIVED_GOAL");
//     autoware_state_label_ptr_->setStyleSheet("background-color: #FF00FF;");
//   } else if (msg->state == autoware_auto_system_msgs::msg::AutowareState::FINALIZING) {
//     autoware_state_label_ptr_->setText("FINALIZING");
//     autoware_state_label_ptr_->setStyleSheet("background-color: #FFFF00;");
//   }
// }

// void ODDPanel::onShift(
//   const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr msg)
// {
//   switch (msg->report) {
//     case autoware_auto_vehicle_msgs::msg::GearReport::PARK:
//       gear_label_ptr_->setText("PARKING");
//       break;
//     case autoware_auto_vehicle_msgs::msg::GearReport::REVERSE:
//       gear_label_ptr_->setText("REVERSE");
//       break;
//     case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE:
//       gear_label_ptr_->setText("DRIVE");
//       break;
//     case autoware_auto_vehicle_msgs::msg::GearReport::LOW:
//       gear_label_ptr_->setText("LOW");
//       break;
//   }
// }

// void ODDPanel::onEngageStatus(
//   const tier4_external_api_msgs::msg::EngageStatus::ConstSharedPtr msg)
// {
//   current_engage_ = msg->engage;
//   engage_status_label_ptr_->setText(QString::fromStdString(Bool2String(current_engage_)));
// }

// void ODDPanel::onEmergencyStatus(
//   const tier4_external_api_msgs::msg::Emergency::ConstSharedPtr msg)
// {
//   current_emergency_ = msg->emergency;
//   if (msg->emergency) {
//     emergency_button_ptr_->setText(QString::fromStdString("Clear Emergency"));
//     emergency_button_ptr_->setStyleSheet("background-color: #FF0000;");
//   } else {
//     emergency_button_ptr_->setText(QString::fromStdString("Set Emergency"));
//     emergency_button_ptr_->setStyleSheet("background-color: #00FF00;");
//   }
// }

// void ODDPanel::onClickVelocityLimit()
// {
//   auto velocity_limit = std::make_shared<tier4_planning_msgs::msg::VelocityLimit>();
//   velocity_limit->max_velocity = pub_velocity_limit_input_->value() / 3.6;
//   pub_velocity_limit_->publish(*velocity_limit);
// }

// void ODDPanel::onClickAutowareEngage()
// {
//   using tier4_external_api_msgs::srv::Engage;

//   auto req = std::make_shared<Engage::Request>();
//   req->engage = !current_engage_;

//   RCLCPP_INFO(raw_node_->get_logger(), "client request");

//   if (!client_engage_->service_is_ready()) {
//     RCLCPP_INFO(raw_node_->get_logger(), "client is unavailable");
//     return;
//   }

//   client_engage_->async_send_request(req, [this](rclcpp::Client<Engage>::SharedFuture result) {
//     RCLCPP_INFO(
//       raw_node_->get_logger(), "Status: %d, %s", result.get()->status.code,
//       result.get()->status.message.c_str());
//   });
// }

// void ODDPanel::onClickEmergencyButton()
// {
//   using tier4_external_api_msgs::msg::ResponseStatus;
//   using tier4_external_api_msgs::srv::SetEmergency;

//   auto request = std::make_shared<SetEmergency::Request>();
//   request->emergency = !current_emergency_;

//   RCLCPP_INFO(raw_node_->get_logger(), request->emergency ? "Set Emergency" : "Clear Emergency");

//   client_emergency_stop_->async_send_request(
//     request, [this](rclcpp::Client<SetEmergency>::SharedFuture result) {
//       const auto & response = result.get();
//       if (response->status.code == ResponseStatus::SUCCESS) {
//         RCLCPP_INFO(raw_node_->get_logger(), "service succeeded");
//       } else {
//         RCLCPP_WARN(
//           raw_node_->get_logger(), "service failed: %s", response->status.message.c_str());
//       }
//     });
// }
// void ODDPanel::onClickGateMode()
// {
//   const auto data = gate_mode_label_ptr_->text().toStdString() == "AUTO"
//                       ? tier4_control_msgs::msg::GateMode::EXTERNAL
//                       : tier4_control_msgs::msg::GateMode::AUTO;
//   RCLCPP_INFO(raw_node_->get_logger(), "data : %d", data);
//   pub_gate_mode_->publish(
//     tier4_control_msgs::build<tier4_control_msgs::msg::GateMode>().data(data));
// }

// void ODDPanel::onClickPathChangeApproval()
// {
//   pub_path_change_approval_->publish(
//     tier4_planning_msgs::build<tier4_planning_msgs::msg::Approval>()
//       .stamp(raw_node_->now())
//       .approval(true));
// }
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ODDPanel, rviz_common::Panel)
