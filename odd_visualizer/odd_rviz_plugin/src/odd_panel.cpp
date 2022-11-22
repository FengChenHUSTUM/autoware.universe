
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

  auto * up_layout = new QVBoxLayout;

  current_lanelet_attributes_table_prt_ = new QTableWidget(this);
  current_lanelet_label_ptr_ = new QLabel("Current Lanelet");
  current_lanelet_ID_label_ptr_ = new QLabel("NULL");
  current_lanelet_label_ptr_->setAlignment(Qt::AlignLeft);
  current_lanelet_attributes_table_prt_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  current_lanelet_attributes_table_prt_->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  current_lanelet_attributes_table_prt_->verticalHeader()->setVisible(false);
  up_layout->addWidget(current_lanelet_label_ptr_);
  up_layout->addWidget(current_lanelet_ID_label_ptr_);
  up_layout->addWidget(current_lanelet_attributes_table_prt_);

  auto * down_layout = new QHBoxLayout;

  // history lanelet
  history_lanelet_attributes_table_prt_ = new QTableWidget(this);
  history_lanelet_attributes_table_prt_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  history_lanelet_attributes_table_prt_->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  history_lanelet_attributes_table_prt_->verticalHeader()->setVisible(false);


  history_lanelet_label_ptr_ = new QLabel("History Lanelet");
  history_lanelet_ID_label_ptr_ = new QLabel("NULL");
  history_lanelet_label_ptr_->setAlignment(Qt::AlignLeft);
  auto * history_lanelet_layout = new QVBoxLayout;
  history_lanelet_layout->addWidget(history_lanelet_label_ptr_);
  history_lanelet_layout->addWidget(history_lanelet_ID_label_ptr_);
  history_lanelet_layout->addWidget(history_lanelet_attributes_table_prt_);

  // next lanelet
  next_lanelet_attributes_table_prt_ = new QTableWidget(this);
  next_lanelet_attributes_table_prt_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  next_lanelet_attributes_table_prt_->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  next_lanelet_attributes_table_prt_->verticalHeader()->setVisible(false);


  next_lanelet_label_ptr_ = new QLabel("Next Lanelet");
  next_lanelet_ID_label_ptr_ = new QLabel("NULL");
  next_lanelet_label_ptr_->setAlignment(Qt::AlignLeft);
  auto * next_lanelet_layout = new QVBoxLayout;
  next_lanelet_layout->addWidget(next_lanelet_label_ptr_);
  next_lanelet_layout->addWidget(next_lanelet_ID_label_ptr_);
  next_lanelet_layout->addWidget(next_lanelet_attributes_table_prt_);

  down_layout->addLayout(history_lanelet_layout);
  down_layout->addLayout(next_lanelet_layout);



  // teleoperation button
  teleoperation_button_ptr_ = new QPushButton("Teleoperation");
  teleoperation_button_ptr_->setStyleSheet("background-color: rgb(106, 117, 126)");
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
}


void ODDPanel::onODDSub(const scenery_msgs::msg::ODDElements::ConstSharedPtr msg) {
  std::list<std::pair<QLabel*, QTableWidget*>> laneletList{
    {history_lanelet_ID_label_ptr_, history_lanelet_attributes_table_prt_},
    {current_lanelet_ID_label_ptr_, current_lanelet_attributes_table_prt_},
    {next_lanelet_ID_label_ptr_, next_lanelet_attributes_table_prt_}};


  size_t index = 0;
  for (auto lanelet : laneletList) {
    lanelet.first->setText("Lanelet ID: " + QString::number(msg->laneletInfo[index].laneletID));
    size_t RowSize = msg->laneletInfo[index].attributes.size();
    lanelet.second->setRowCount(RowSize);
    lanelet.second->setColumnCount(2);
    QStringList tableHeaders;
    tableHeaders << "Attribute" << "Value";
    lanelet.second->setHorizontalHeaderLabels(tableHeaders);
    size_t row = 0;
    for (auto & attr : msg->laneletInfo[index].attributes) {
      if (row < RowSize) {
        lanelet.second->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(attr.attributeName)));
        lanelet.second->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(attr.strValue)));
        row++;
      }
    }
    index++;
  }
}

void ODDPanel::onClickODDTeleoperation()
{
  auto req = std::make_shared<Teleoperation::Request>();
  req->teleoperation_status = teleoperation_button_on;
  client_teleoperation_->async_send_request(req, [this](rclcpp::Client<Teleoperation>::SharedFuture response) {
    if (response.get()->teleoperation_ready == 1
        && teleoperation_button_on == false) {
      teleoperation_button_ptr_->setStyleSheet("background-color: rgb(159, 186, 54)");
      teleoperation_button_on = true;
    }
    else if (response.get()->teleoperation_ready == 0
            && teleoperation_button_on == true) {
      teleoperation_button_ptr_->setStyleSheet("background-color: rgb(106, 117, 126)");
      teleoperation_button_on = false;
    }
    else {
      teleoperation_button_on = false;
      teleoperation_button_ptr_->setStyleSheet("background-color: rgb(217, 81, 23)");
      RCLCPP_INFO(raw_node_->get_logger(), "teleoperation not allowed!");
    }
  });  

}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ODDPanel, rviz_common::Panel)

