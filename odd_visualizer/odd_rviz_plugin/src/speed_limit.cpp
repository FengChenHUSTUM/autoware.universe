
#include "speed_limit.hpp"

#include <QPainter>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_common/uniform_string_stream.hpp>

#include <X11/Xlib.h>

#include <algorithm>
#include <iomanip>
#include <memory>
#include <string>

namespace rviz_plugins
{
SpeedLimitDisplay::SpeedLimitDisplay()
: handle_image_(std::string(
                  ament_index_cpp::get_package_share_directory("odd_rviz_plugin") +
                  "/images/speed_limit_sign.png")
                  .c_str())
{
  const Screen * screen_info = DefaultScreenOfDisplay(XOpenDisplay(NULL));

  constexpr float height_4k = 2160.0;
  const float scale = static_cast<float>(screen_info->height) / height_4k;
  const auto left = static_cast<int>(std::round(2160 * scale));
  const auto top = static_cast<int>(std::round(128 * scale));
  const auto length = static_cast<int>(std::round(256 * scale));

  property_text_color_ = new rviz_common::properties::ColorProperty(
    "Text Color", QColor(0, 0, 0), "text color", this, SLOT(updateVisualization()), this);
  property_left_ = new rviz_common::properties::IntProperty(
    "left", left, "left of the plotter window", this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ = new rviz_common::properties::IntProperty(
    "Top", top, "Top of the plotter window", this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_length_ = new rviz_common::properties::IntProperty(
    "Length", length, "Length of the plotter window", this, SLOT(updateVisualization()), this);
  property_length_->setMin(10);
  property_value_height_offset_ = new rviz_common::properties::IntProperty(
    "Value height offset", 0, "Height offset of the plotter window", this,
    SLOT(updateVisualization()));
  property_value_scale_ = new rviz_common::properties::FloatProperty(
    "Value Scale", 1.0 / 6.667, "Value scale", this, SLOT(updateVisualization()), this);
  property_value_scale_->setMin(0.01);
  property_handle_angle_scale_ = new rviz_common::properties::FloatProperty(
    "Scale", 3.0, "Scale is steering angle to handle angle ", this, SLOT(updateVisualization()),
    this);
  property_handle_angle_scale_->setMin(0.1);
}

SpeedLimitDisplay::~SpeedLimitDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void SpeedLimitDisplay::onInitialize()
{
  RTDClass::onInitialize();
  static int count = 0;
  rviz_common::UniformStringStream ss;
  ss << "SpeedLimitDisplayObject" << count++;
  overlay_.reset(new jsk_rviz_plugins::OverlayObject(ss.str()));

  overlay_->show();

  updateVisualization();
}

void SpeedLimitDisplay::onEnable()
{
  subscribe();
  overlay_->show();
}

void SpeedLimitDisplay::onDisable()
{
  unsubscribe();
  reset();
  overlay_->hide();
}

void SpeedLimitDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  double steering = 0;
  {
    std::lock_guard<std::mutex> message_lock(mutex_);
    if (last_msg_ptr_) {
      steering = last_msg_ptr_->speedLimit * 1.0;
    }
  }

  QColor background_color;
  background_color.setAlpha(0);
  jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  if (!buffer.getPixelBuffer()) {
    return;
  }

  QImage hud = buffer.getQImage(*overlay_);
  hud.fill(background_color);

  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);
  QColor text_color(0, 0, 0);
  text_color.setAlpha(255);
  painter.setPen(QPen(text_color, static_cast<int>(2), Qt::SolidLine));

  const int w = overlay_->getTextureWidth();
  const int h = overlay_->getTextureHeight();

  painter.drawPixmap(
    0, 0, property_length_->getInt(), property_length_->getInt(), handle_image_);


  std::ostringstream speedLimitTitle, speedLimitValue;
  speedLimitTitle << std::fixed << "Speed Limit\n";
  speedLimitValue << std::fixed << std::setprecision(1) << steering << "km/h";

  QFont font = painter.font();
  int fontSize = static_cast<int>((static_cast<double>(w)) * property_value_scale_->getFloat());

  font.setPixelSize(std::max(static_cast<int>(std::round(fontSize)), 1));
  font.setBold(true);
  painter.drawText(
  0, std::min(property_value_height_offset_->getInt() - static_cast<int>(std::round(fontSize / 2)), h - 1), w,
  std::max(h - property_value_height_offset_->getInt(), 1), Qt::AlignCenter | Qt::AlignTop,
  speedLimitTitle.str().c_str());

  font.setPixelSize(
    std::max(fontSize, 1));
  font.setBold(true);
  painter.setFont(font);
  painter.drawText(
    0, std::min(property_value_height_offset_->getInt() + static_cast<int>(std::round(fontSize / 2)), h - 1), w,
    std::max(h - property_value_height_offset_->getInt(), 1), Qt::AlignCenter | Qt::AlignBaseline,
    speedLimitValue.str().c_str());

  painter.end();
}

void SpeedLimitDisplay::processMessage(
  const scenery_msgs::msg::speedLimitDisplay::ConstSharedPtr msg_ptr)
{
  if (!isEnabled()) {
    return;
  }

  {
    std::lock_guard<std::mutex> message_lock(mutex_);
    last_msg_ptr_ = msg_ptr;
  }

  queueRender();
}

void SpeedLimitDisplay::updateVisualization()
{
  overlay_->updateTextureSize(property_length_->getInt(), property_length_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::SpeedLimitDisplay, rviz_common::Display)
