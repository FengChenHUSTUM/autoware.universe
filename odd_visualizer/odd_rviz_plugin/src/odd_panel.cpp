
#include "odd_panel.hpp"



inline std::string Bool2String(const bool var) { return var ? "True" : "False"; }

using std::placeholders::_1;

namespace rviz_plugins
{
ODDPanel::ODDPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  ODDTab_prt_ = new QTabWidget(this);
  path_to_current_folder = QString::fromStdString(ament_index_cpp::get_package_share_directory("odd_rviz_plugin"));

  // Edit the keys in attrVec to determin which attributes are supposed to be presented
  attrVec = {"location", "one_way", "speed_limit", "subtype", "type", "weather"};

  iconSize = QSize(32, 32);
  // general infomation
  auto * general_info_layout = new QVBoxLayout;

  current_title_ptr_ = new QLabel("Current Lanelet: ");
  history_title_ptr_ = new QLabel("History Lanelet: ");
  next_title_ptr_ = new QLabel("Next Lanelet: ");
  current_general_table_ptr_ = new QTableWidget(this);
  history_general_table_ptr_ = new QTableWidget(this);
  next_general_table_ptr_ = new QTableWidget(this);

  general_info_layout->addWidget(createLaneletIconFrame(current_title_ptr_, current_general_table_ptr_));
  general_info_layout->addWidget(createLaneletIconFrame(history_title_ptr_, history_general_table_ptr_));
  general_info_layout->addWidget(createLaneletIconFrame(next_title_ptr_, next_general_table_ptr_));

  auto generalTab = new QWidget();
  generalTab->setLayout(general_info_layout);
  ODDTab_prt_->addTab(generalTab, "General");

  // Details
  current_lanelet_attributes_table_prt_ = new QTableWidget(this);
  history_lanelet_attributes_table_prt_ = new QTableWidget(this);
  next_lanelet_attributes_table_prt_ = new QTableWidget(this);
  current_lanelet_label_ptr_ = new QLabel("Current Lanelet");
  history_lanelet_label_ptr_ = new QLabel("History Lanelet");
  next_lanelet_label_ptr_ = new QLabel("Next Lanelet");
  current_lanelet_ID_label_ptr_ = new QLabel("NULL");
  history_lanelet_ID_label_ptr_ = new QLabel("NULL");
  next_lanelet_ID_label_ptr_ = new QLabel("NULL");

  auto laneletInfo_layout = new QVBoxLayout;
  // auto currentBox = new QGroupBox;
  // auto historyBox = new QGroupBox;
  // auto nextBox = new QGroupBox;
  // currentBox->setLayout(createLaneletDetailsLayout(
  //   current_lanelet_attributes_table_prt_,
  //   current_lanelet_label_ptr_,
  //   current_lanelet_ID_label_ptr_));
  // historyBox->setLayout(createLaneletDetailsLayout(
  //   history_lanelet_attributes_table_prt_,
  //   history_lanelet_label_ptr_,
  //   history_lanelet_ID_label_ptr_));
  // nextBox->setLayout(createLaneletDetailsLayout(
  //   next_lanelet_attributes_table_prt_,
  //   next_lanelet_label_ptr_,
  //   next_lanelet_ID_label_ptr_));
  // auto toolboxfortest = new QToolBox;
  // toolboxfortest->addItem((QWidget*)currentBox, tr("current lanelet"));
  // toolboxfortest->addItem((QWidget*)historyBox, tr("history lanelet"));
  // toolboxfortest->addItem((QWidget*)nextBox, tr("next lanelet"));
  // laneletInfo_layout->addWidget(toolboxfortest);

  current_button_ptr_ = new QPushButton;
  current_button_ptr_->setText("current lanele");
  history_button_ptr_ = new QPushButton;
  history_button_ptr_->setText("history lanele");
  next_button_ptr_ = new QPushButton;
  next_button_ptr_->setText("next lanele");

  laneletInfo_layout->addWidget(current_button_ptr_);
  current_Frame_ptr_ = new QFrame;
  current_Frame_ptr_->setLayout(createLaneletDetailsLayout(
    current_lanelet_attributes_table_prt_,
    current_lanelet_label_ptr_,
    current_lanelet_ID_label_ptr_));
  laneletInfo_layout->addWidget(current_Frame_ptr_);

  laneletInfo_layout->addWidget(history_button_ptr_);
  history_Frame_ptr_ = new QFrame;
  history_Frame_ptr_->setLayout(createLaneletDetailsLayout(
    history_lanelet_attributes_table_prt_,
    history_lanelet_label_ptr_,
    history_lanelet_ID_label_ptr_));
  laneletInfo_layout->addWidget(history_Frame_ptr_);

  laneletInfo_layout->addWidget(next_button_ptr_);
  next_Frame_ptr_ = new QFrame;
  next_Frame_ptr_->setLayout(createLaneletDetailsLayout(
    next_lanelet_attributes_table_prt_,
    next_lanelet_label_ptr_,
    next_lanelet_ID_label_ptr_));
  laneletInfo_layout->addWidget(next_Frame_ptr_);
  laneletInfo_layout->addStretch();
  laneletInfo_layout->setMargin(0);
  laneletInfo_layout->setSpacing(0);
  connect(current_button_ptr_, SIGNAL(clicked()), SLOT(onClickCurrent()));
  connect(history_button_ptr_, SIGNAL(clicked()), SLOT(onClickHistory()));
  connect(next_button_ptr_, SIGNAL(clicked()), SLOT(onClickNext()));

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

QVBoxLayout *ODDPanel::createLaneletDetailsLayout(QTableWidget *table, QLabel *title, QLabel *id){
  auto * up_layout = new QVBoxLayout;
  title->setAlignment(Qt::AlignLeft);
  table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table->verticalHeader()->setVisible(false);
  up_layout->addWidget(title);
  up_layout->addWidget(id);
  up_layout->addWidget(table);
  up_layout->addStretch();
  return up_layout;
}

QFrame *ODDPanel::createLaneletIconFrame(QLabel *laneletTitle, QTableWidget *laneletTable) {
  auto * general_info_layout_up = new QVBoxLayout;
  QSizePolicy frameSizePolicy;
  frameSizePolicy.setHorizontalPolicy(QSizePolicy::Expanding);
  frameSizePolicy.setVerticalPolicy(QSizePolicy::Expanding);

  laneletTitle->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  laneletTitle->setStyleSheet("border: 0px solid");
  laneletTitle->setMargin(5);
  // current_title_ptr_->setMouseTracking(true);
  // current_title_ptr_->setToolTip("test if it works");

  QSizePolicy labelSizePolicy;
  labelSizePolicy.setHorizontalPolicy(QSizePolicy::Minimum);
  laneletTitle->setSizePolicy(labelSizePolicy);

  setIconTableStyle(laneletTable);

  general_info_layout_up->addWidget(laneletTitle);
  general_info_layout_up->addWidget(laneletTable);
  // general_info_layout_up->addWidget(current_general_description_ptr_);

  auto upFrame = new QFrame(this);
  upFrame->setLayout(general_info_layout_up);
  upFrame->setStyleSheet("border: 1px solid");
  upFrame->setSizePolicy(frameSizePolicy);
  return upFrame;
}

void ODDPanel::setIconTableStyle(QTableWidget * table) {
  table->setShowGrid(false);
  // table->setIconSize(QSize(32,32));
  table->setColumnCount(5);
  table->setRowCount(2);
  table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table->horizontalHeader()->setVisible(false);
  table->verticalHeader()->setVisible(false);
  setItemInTable(table);
}

void ODDPanel::setItemInTable(QTableWidget * table) {
  for (int i = 0; i < table->rowCount(); ++i) {
    for (int j = 0; j < table->columnCount(); ++j) {
      int index = i * table->columnCount() + j;
      if (index < static_cast<int>(attrVec.size()))
        table->setCellWidget(i, j, setTableItemFromAttr(attrVec[index]));
      else
        break;
    }
  }
}

QWidget * ODDPanel::setTableItemFromAttr(const QString &attr){
  QIcon ODDIcon(path_to_current_folder + "/images/" + attr + ".png");
  QWidget * tableItem = new QWidget();
  QLabel *iconLabel = new QLabel();
  iconLabel->setMaximumSize(iconSize);
  iconLabel->setPixmap(ODDIcon.pixmap(iconSize));
  iconLabel->setStyleSheet("border:0px;");
  QVBoxLayout *iconLayout = new QVBoxLayout;
  iconLayout->addWidget(iconLabel);
  iconLayout->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
  QVBoxLayout *itemLayout = new QVBoxLayout;
  itemLayout->setAlignment(Qt::AlignCenter);
  itemLayout->addLayout(iconLayout);
  itemLayout->addWidget(new QLabel(attr));
  tableItem->setLayout(itemLayout);
  tableItem->setStyleSheet("border:0px;");
  tableItem->setMouseTracking(true);
  tableItem->setToolTip(attr);
  // auto iconItem = new QTableWidgetItem;
  // iconItem->setSizeHint(QSize(32,32));
  // iconItem->setTextAlignment(Qt::AlignCenter);
  return tableItem;
}

void ODDPanel::updateDetails(const scenery_msgs::msg::laneletODD &laneletInfo, QTableWidget *table) {
  //test on current lanelet: msg->laneletInfo[1]
  setItemInTable(table);
  size_t countAttr = 0;
  for (int i = 0; i < table->rowCount(); ++i) {
    for (int j = 0; j < table->columnCount(); ++j) {
      int index = i * table->columnCount() + j;
      if (index < static_cast<int>(laneletInfo.attributes.size()) && 
          countAttr < laneletInfo.attributes.size()) {
        QString iconName = QString::fromStdString(laneletInfo.attributes[countAttr].attributeName);
        while (countAttr < laneletInfo.attributes.size()
               && !std::count(attrVec.begin(), attrVec.end(), iconName)){
          iconName = QString::fromStdString(laneletInfo.attributes[++countAttr].attributeName);
        }
        if (countAttr < laneletInfo.attributes.size()){
          QString iconValue = QString::fromStdString(laneletInfo.attributes[countAttr++].strValue);
          if (iconValue == "yes") iconValue = "one way";
          if (iconValue == "no") iconValue = "two way";
          if (iconValue == "30") iconValue = "speed limit";
          table->setCellWidget(i, j, setTableItemFromAttr(iconValue));
        } // if attribut is supported
      }// loop not exceed the size of lanelet attributes
    }
  }
}

void ODDPanel::onODDSub(const scenery_msgs::msg::ODDElements::ConstSharedPtr msg) {

  // update information in tab "General"
  updateDetails(msg->laneletInfo[1], current_general_table_ptr_);
  updateDetails(msg->laneletInfo[0], history_general_table_ptr_);
  updateDetails(msg->laneletInfo[2], next_general_table_ptr_);

  // update informations in tab "Details"
  std::list<std::pair<QLabel*, QTableWidget*>> laneletDetailsList{
    {history_lanelet_ID_label_ptr_, history_lanelet_attributes_table_prt_},
    {current_lanelet_ID_label_ptr_, current_lanelet_attributes_table_prt_},
    {next_lanelet_ID_label_ptr_, next_lanelet_attributes_table_prt_}};
  size_t index = 0;
  for (auto lanelet : laneletDetailsList) {
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

  void ODDPanel::onClickCurrent(){
    if (!current_btn_on) {
      current_Frame_ptr_->setVisible(false);
      current_btn_on = true;
    }
    else {
      current_Frame_ptr_->setVisible(true);
      current_btn_on = false;
    }
  }

  void ODDPanel::onClickHistory(){
    if (!history_btn_on) {
      history_Frame_ptr_->setVisible(false);
      history_btn_on = true;
    }
    else {
      history_Frame_ptr_->setVisible(true);
      history_btn_on = false;
    }
  }

  void ODDPanel::onClickNext(){
    if (!next_btn_on) {
      next_Frame_ptr_->setVisible(false);
      next_btn_on = true;
    }
    else {
      next_Frame_ptr_->setVisible(true);
      next_btn_on = false;
    }
  }



}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ODDPanel, rviz_common::Panel)

