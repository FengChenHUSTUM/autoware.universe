#ifndef ODD_SPEED_LIMIT_HPP_
#define ODD_SPEED_LIMIT_HPP_

#include <memory>
#include <mutex>

#ifndef Q_MOC_RUN
#include <tier4_vehicle_rviz_plugin/src/tools/jsk_overlay_utils.hpp>

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#endif

namespace rviz_plugins
{
class ODDSpeedLimitDisplay
: public rviz_common::RosTopicDisplay<>
{
    Q_OBJECT

public:
    ODDSpeedLimitDisplay();
    ~ODDSpeedLimitDisplay() override;
};



} // namespace rviz_plugins



#endif // ODD_SPEED_LIMIT_HPP_