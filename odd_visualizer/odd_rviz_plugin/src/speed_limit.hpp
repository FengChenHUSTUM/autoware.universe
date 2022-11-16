#ifndef _SPEED_LIMIT_HPP_
#define _SPEED_LIMIT_HPP_

#include <memory>
#include <mutex>

#ifndef Q_MOC_RUN
#include "jsk_overlay_utils.hpp"

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <scenery_msgs/msg/speed_limit_display.hpp>
#endif

namespace rviz_plugins
{
class SpeedLimitDisplay
: public rviz_common::RosTopicDisplay<scenery_msgs::msg::speedLimitDisplay>
{
  Q_OBJECT

public:
  SpeedLimitDisplay();
  ~SpeedLimitDisplay() override;

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void update(float wall_dt, float ros_dt) override;
  void processMessage(
    const scenery_msgs::msg::speedLimitDisplay::ConstSharedPtr msg_ptr) override;

  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz_common::properties::ColorProperty * property_text_color_;
  rviz_common::properties::IntProperty * property_left_;
  rviz_common::properties::IntProperty * property_top_;
  rviz_common::properties::IntProperty * property_length_;
  rviz_common::properties::FloatProperty * property_handle_angle_scale_;
  rviz_common::properties::IntProperty * property_value_height_offset_;
  rviz_common::properties::FloatProperty * property_value_scale_;
  QPixmap handle_image_;
  // QImage hud_;

private:
  std::mutex mutex_;
  scenery_msgs::msg::speedLimitDisplay::ConstSharedPtr last_msg_ptr_;
};

}  // namespace rviz_plugins

#endif  // _SPEED_LIMIT_HPP_
