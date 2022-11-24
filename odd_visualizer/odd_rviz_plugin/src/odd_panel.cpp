
#include "odd_panel.hpp"

#include <QHBoxLayout>
#include <QString>
#include <QFrame>
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
  ODDTab_prt_ = new QTabWidget(this);
  path_to_current_folder = QString::fromStdString(ament_index_cpp::get_package_share_directory("odd_rviz_plugin"));
  attrVec = {"location", "one_way", "speed_limit", "subtype", "type", "weather"};
  // general infomation
  auto * general_info_layout = new QVBoxLayout;
  auto * general_info_layout_up = new QVBoxLayout;
  auto * general_info_layout_down = new QHBoxLayout;

  QSizePolicy frameSizePolicy;
  frameSizePolicy.setHorizontalPolicy(QSizePolicy::Expanding);
  frameSizePolicy.setVerticalPolicy(QSizePolicy::Expanding);

  current_title_ptr_ = new QLabel("Current Lanelet: ");
  current_title_ptr_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  current_title_ptr_->setStyleSheet("border: 0px solid");
  current_title_ptr_->setMargin(5);
  // current_title_ptr_->setMouseTracking(true);
  // current_title_ptr_->setToolTip("test if it works");

  QSizePolicy labelSizePolicy;
  labelSizePolicy.setHorizontalPolicy(QSizePolicy::Minimum);
  current_title_ptr_->setSizePolicy(labelSizePolicy);

  current_general_table_ptr_ = new QTableWidget(this);
  setIconTableStyle(current_general_table_ptr_);

  general_info_layout_up->addWidget(current_title_ptr_);
  general_info_layout_up->addWidget(current_general_table_ptr_);
  // general_info_layout_up->addWidget(current_general_description_ptr_);

  auto upFrame = new QFrame(this);
  upFrame->setLayout(general_info_layout_up);
  upFrame->setStyleSheet("border: 1px solid");
  upFrame->setSizePolicy(frameSizePolicy);

  history_title_ptr_ = new QLabel("History Lanelet: ");
  history_title_ptr_->setSizePolicy(labelSizePolicy);
  history_title_ptr_->setStyleSheet("border: 0px solid");
  history_general_description_ptr_ = new QLabel("\n");
  history_general_description_ptr_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  
  next_title_ptr_ = new QLabel("Next Lanelet: ");
  next_title_ptr_->setSizePolicy(labelSizePolicy);
  next_title_ptr_->setStyleSheet("border: 0px solid");
  next_general_description_ptr_ = new QLabel("\n");
  next_general_description_ptr_->setAlignment(Qt::AlignTop | Qt::AlignLeft);

  auto * left_tmp_layout = new QVBoxLayout;
  left_tmp_layout->addWidget(history_title_ptr_);
  left_tmp_layout->addWidget(history_general_description_ptr_);
  auto leftFrame = new QFrame;
  leftFrame->setLayout(left_tmp_layout);
  leftFrame->setStyleSheet("border: 1px solid");
  leftFrame->setSizePolicy(frameSizePolicy);

  auto * right_tmp_layout = new QVBoxLayout;
  right_tmp_layout->addWidget(next_title_ptr_);
  right_tmp_layout->addWidget(next_general_description_ptr_);
  auto rightFrame = new QFrame;
  rightFrame->setLayout(right_tmp_layout);
  rightFrame->setStyleSheet("border: 1px solid");
  rightFrame->setSizePolicy(frameSizePolicy);

  general_info_layout_down->addWidget(leftFrame);
  general_info_layout_down->addWidget(rightFrame);

  general_info_layout->addWidget(upFrame);
  general_info_layout->addLayout(general_info_layout_down);

  auto generalTab = new QWidget();
  generalTab->setLayout(general_info_layout);

  ODDTab_prt_->addTab(generalTab, "General");

  // Details
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

  auto *laneletInfo_layout = new QVBoxLayout;
  laneletInfo_layout->addLayout(up_layout);
  laneletInfo_layout->addLayout(down_layout);

  auto laneletDetails = new QWidget();
  laneletDetails->setLayout(laneletInfo_layout);
  ODDTab_prt_->addTab(laneletDetails, "Details");

  // teleoperation button
  teleoperation_button_ptr_ = new QPushButton("Teleoperation");
  teleoperation_button_ptr_->setStyleSheet("background-color: rgb(106, 117, 126)");
  connect(teleoperation_button_ptr_, SIGNAL(clicked()), SLOT(onClickODDTeleoperation()));

  // Layout
  auto * v_layout = new QVBoxLayout;
  v_layout->addWidget(ODDTab_prt_);
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

void ODDPanel::setIconTableStyle(QTableWidget *table) {
  table->setShowGrid(false);
  // table->setIconSize(QSize(32,32));
  table->setColumnCount(5);
  table->setRowCount(2);
  table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table->horizontalHeader()->setVisible(false);
  table->verticalHeader()->setVisible(false);
  for (int i = 0; i < table->rowCount(); ++i) {
    for (int j = 0; j < table->columnCount(); ++j) {
      table->setCellWidget(i, j, setTableItemFromAttr(attrVec[0]));
    }
  }
}

QWidget * ODDPanel::setTableItemFromAttr(const QString &attr){
  QIcon ODDIcon(path_to_current_folder + "/images/" + attr + ".png");
  QWidget * tableItem = new QWidget();
  QLabel *iconLabel = new QLabel(tableItem);
  iconLabel->setMaximumSize(QSize(32,32));
  iconLabel->setPixmap(ODDIcon.pixmap(QSize(32,32)));
  iconLabel->setStyleSheet("border:0px;");
  QHBoxLayout *iconLayout = new QHBoxLayout;
  iconLayout->addWidget(iconLabel);
  iconLayout->setAlignment(Qt::AlignCenter);
  tableItem->setLayout(iconLayout);
  tableItem->setStyleSheet("border:0px;");
  // auto iconItem = new QTableWidgetItem;
  // iconItem->setSizeHint(QSize(32,32));
  // iconItem->setTextAlignment(Qt::AlignCenter);
  return tableItem;
}

void ODDPanel::onODDSub(const scenery_msgs::msg::ODDElements::ConstSharedPtr msg) {
  std::list<std::pair<QLabel*, QTableWidget*>> laneletList{
    {history_lanelet_ID_label_ptr_, history_lanelet_attributes_table_prt_},
    {current_lanelet_ID_label_ptr_, current_lanelet_attributes_table_prt_},
    {next_lanelet_ID_label_ptr_, next_lanelet_attributes_table_prt_}};

  // std::vector<QLabel*> generalLables{
  //   history_general_description_ptr_,
  //   current_general_description_ptr_,
  //   next_general_description_ptr_};

  // //test on current lanelet: msg->laneletInfo[1]
  // size_t curRowSize =  msg->laneletInfo[1].attributes.size() / 5 + 1;
  // current_lanelet_attributes_table_prt_->setRowCount(curRowSize);
  // size_t countRow = 0;
  // for (auto & attr : msg->laneletInfo[1].attributes) {

  // }



  size_t index = 0;
  for (auto lanelet : laneletList) {
    lanelet.first->setText("Lanelet ID: " + QString::number(msg->laneletInfo[index].laneletID));
    size_t RowSize = msg->laneletInfo[index].attributes.size();
    lanelet.second->setRowCount(RowSize);
    lanelet.second->setColumnCount(2);
    // lanelet.second->setFrameStyle(QFrame::NoFrame);
    // lanelet.second->setShowGrid(false);
    QStringList tableHeaders;
    tableHeaders << "Attribute" << "Value";
    lanelet.second->setHorizontalHeaderLabels(tableHeaders);
    size_t row = 0;
    QString geDescription{};
    for (auto & attr : msg->laneletInfo[index].attributes) {
      if (row < RowSize) {
        lanelet.second->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(attr.attributeName)));
        lanelet.second->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(attr.strValue)));
        row++;
      }
      // if (attr.attributeName == "one_way") {
      //   attr.strValue == "yes"? geDescription.append("\none-way") : geDescription.append("\ntwo-way");
      // }
      // if (attr.attributeName == "location") {
      //   geDescription.append("\nlocated in " + QString::fromStdString(attr.strValue));
      // }
    }
    // generalLables[index]->setText(geDescription);
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

