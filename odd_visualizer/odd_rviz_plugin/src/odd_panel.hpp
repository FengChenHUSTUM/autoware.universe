#ifndef ODD_PANEL_HPP_
#define ODD_PANEL_HPP_


#include <regex>
#include <QTabWidget>
#include <QLabel>
#include <QIcon>
#include <QPushButton>
#include <QTableWidget> 
#include <QSizePolicy>
#include <QHeaderView>
#include <QEvent>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QString>
#include <QFrame>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_common/display_context.hpp>

#include <scenery_msgs/msg/odd_elements.hpp>
#include <scenery_msgs/msg/tele_state.hpp>
#include <scenery_msgs/srv/teleoperation.hpp>

#include <memory>
#include <string>
using scenery_msgs::srv::Teleoperation;


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

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<scenery_msgs::msg::ODDElements>::SharedPtr sub_odd_elements_;
  rclcpp::Client<scenery_msgs::srv::Teleoperation>::SharedPtr client_teleoperation_;

  // rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr pub_velocity_limit_;

  QString path_to_current_folder;
  std::vector<QString> attrVec;
  QTabWidget * ODDTab_prt_;
  QSize iconSize;
  QLabel * current_title_ptr_;
  QLabel * history_title_ptr_;
  QLabel * next_title_ptr_;
  QTableWidget * current_general_table_ptr_;
  QTableWidget * history_general_table_ptr_;
  QTableWidget * next_general_table_ptr_;


  QLabel * current_lanelet_label_ptr_;
  QLabel * current_lanelet_ID_label_ptr_;
  QLabel * history_lanelet_label_ptr_;
  QLabel * history_lanelet_ID_label_ptr_;
  QLabel * next_lanelet_label_ptr_;
  QLabel * next_lanelet_ID_label_ptr_;

  QTableWidget * current_lanelet_attributes_table_prt_;
  QTableWidget * history_lanelet_attributes_table_prt_;
  QTableWidget * next_lanelet_attributes_table_prt_;

  QPushButton * teleoperation_button_ptr_;
  bool teleoperation_button_on{false};

  void setIconTableStyle(QTableWidget * table);
  void setItemInTable(QTableWidget * table);
  QFrame *createLaneletIconFrame(QLabel *laneletTitle, QTableWidget *laneletTable);
  QWidget *setTableItemFromAttr(const QString &attr);
  void updateDetails(const scenery_msgs::msg::laneletODD &laneletInfo, QTableWidget *table);
};

}  // namespace rviz_plugins

#endif  // ODD_PANEL_HPP_
