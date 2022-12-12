#ifndef ODD_VISUALIZER_NODE_HPP_
#define ODD_VISUALIZER_NODE_HPP_

#include <vector>
#include <algorithm>
#include <memory>

// Ros2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Ros2 message
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <scenery_msgs/msg/lane_sequence_with_id.hpp>
#include <scenery_msgs/msg/speed_limit_display.hpp>
#include <scenery_msgs/msg/odd_elements.hpp>
#include <scenery_msgs/srv/teleoperation.hpp>

#include "utils.hpp"
#include "tele_state_machine.hpp"

// Autoware
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <behavior_path_planner/debug_utilities.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <motion_utils/resample/resample.hpp>

#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <autoware_auto_system_msgs/msg/hazard_status_stamped.hpp>
#include <autoware_auto_system_msgs/msg/autoware_state.hpp>

#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>

#include <tier4_external_api_msgs/msg/engage_status.hpp>

// #include <tier4_autoware_utils/ros/self_pose_listener.hpp>

// Lanelet
#include <lanelet2_io/Io.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

namespace odd_visualizer
{

using geometry_msgs::msg::PoseStamped;

using visualization_msgs::msg::MarkerArray;
using std_msgs::msg::ColorRGBA;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerOrientation;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;
using tier4_external_api_msgs::msg::EngageStatus;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Point;
using scenery_msgs::msg::laneSequenceWithID;


using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_system_msgs::msg::EmergencyState;
using autoware_auto_system_msgs::msg::AutowareState;
using autoware_auto_system_msgs::msg::HazardStatus;

using posesVec = std::vector<geometry_msgs::msg::Pose>;
using pointsVec = std::vector<geometry_msgs::msg::Point>;

// using autoware_auto_mapping_msgs::msg::HADMapBin; 
class OddVisualizer : public rclcpp::Node
{
public:
    explicit OddVisualizer(const std::string& node_name, const rclcpp::NodeOptions & node_options);
private:
    // ROS subscribers and publishers
    rclcpp::Subscription<HADMapBin>::SharedPtr map_subscriber_;
    rclcpp::Subscription<laneSequenceWithID>::SharedPtr lanelet_sequence_subscriber_;
    rclcpp::Subscription<EmergencyState>::SharedPtr emergency_state_subscriber_;
    rclcpp::Subscription<AutowareState>::SharedPtr autoware_state_subscriber_;
    rclcpp::Subscription<HazardStatus>::SharedPtr hazard_state_subscriber_;
    rclcpp::Subscription<EngageStatus>::SharedPtr engage_state_subscriber_;
    
    rclcpp::Publisher<MarkerArray>::SharedPtr odd_driveable_area_publisher_;
    rclcpp::Publisher<MarkerArray>::SharedPtr odd_adjacent_lane_publisher_;
    rclcpp::Publisher<scenery_msgs::msg::speedLimitDisplay>::SharedPtr odd_speed_limit_publisher_;
    rclcpp::Publisher<scenery_msgs::msg::ODDElements>::SharedPtr odd_elements_publisher_;

    rclcpp::Service<scenery_msgs::srv::Teleoperation>::SharedPtr odd_teleoperation_service_;

    tier4_autoware_utils::SelfPoseListener self_pose_listener_{this};


    // ROS node
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::shared_ptr<odd_tools::ODD_elements> odd_elements_;


    // Lanelet2 
    lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
    lanelet::LaneletMapPtr lanelet_map_ptr_;
    lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
    lanelet::ConstLanelets all_lanelets_;
    lanelet::ConstLanelets current_lanelets_;
    std::shared_ptr<lanelet::ConstLanelet> current_lanelet_;
    // the postion of current lanelet in the lanelet sequence
    size_t curIndex{0};


    TeleStateMachine *tele_state_machine_;
    std::mutex lock_state_machine_;
    // teleoperation service: "0" for off; "1" for on
    uint8_t teleoperation_status{0};
    rclcpp::Time engage_time_;
    bool engage_status{false};


    /**
     * @brief convert lanelet2 into Polygons in the format of geometry message.
     * This is a same implementation as the one in visualization.cpp in autoware.universe
     * 
     * @param ll        [input const lanelet::ConstLanelet]
     * @param polygon   [geometry_msgs::msg::Polygon]
     */
    void lanletToPolygonMsg(
        const lanelet::ConstLanelet & ll,
        geometry_msgs::msg::Polygon * polygon);
    
    /**
     * @brief execute the ODD elements checker and publish it
     * 
     */
    void run();

    void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);
    void laneletSequenceCallback(const laneSequenceWithID::ConstSharedPtr msg);
    void EmergencyStateCallback(const EmergencyState::ConstSharedPtr msg);
    void AutowareStateCallback(const AutowareState::ConstSharedPtr msg);
    void HazardStatusCallback(const HazardStatus::ConstSharedPtr msg);
    void EngageStatusCallback(const EngageStatus::ConstSharedPtr msg);

    void creareDrivableBoundaryMarkerArray(const lanelet::ConstLanelets laneletSequence);

    void onTeleoperationService(
        const scenery_msgs::srv::Teleoperation::Request::SharedPtr request,
        const scenery_msgs::srv::Teleoperation::Response::SharedPtr response);

    void getODDFromMap(const lanelet::ConstLanelets laneletSequence);
    void onAdjacentLanelet(const lanelet::ConstLanelet currentLanelet);
    MarkerArray createDriveableAreaBoundary();
    MarkerArray createAdjacentLaneBoundary(const pointsVec & points,
                                           const pointsVec & arrows,
                                           bool sameDirection = false);

    std_msgs::msg::ColorRGBA getColorRGBAmsg(const std::vector<int64_t> &odd_color);
    odd_tools::ODD_elements getParam();
};
} // namespace odd_visualizer

#endif // ODD_VISUALIZER_NODE_HPP_