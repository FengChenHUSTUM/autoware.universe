#ifndef ODD_VISUALIZER_NODE_HPP_
#define ODD_VISUALIZER_NODE_HPP_

#include <vector>

// Ros2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Ros2 message
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <scenery_msgs/msg/lane_sequence_with_id.hpp>

// #include <scenery_msgs/msg/edge_data.hpp>
// #include <scenery_msgs/msg/edge_primitive.hpp>
// #include <scenery_msgs/msg/lane_data.hpp>
// #include <scenery_msgs/msg/lane_primitive.hpp>
// #include <scenery_msgs/msg/regulatory_ele_data.hpp>
// #include <scenery_msgs/msg/regulatory_ele_primitive.hpp>

#include "utils.hpp"

// Autoware
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <behavior_path_planner/debug_utilities.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
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
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Point;
// using scenery_msgs::msg::EdgeData;
// using scenery_msgs::msg::LaneData;
// using scenery_msgs::msg::RegulatoryEleData;
using scenery_msgs::msg::laneSequenceWithID;
// using odd_tools::ODD_elements;


using autoware_auto_mapping_msgs::msg::HADMapBin;

// using autoware_auto_mapping_msgs::msg::HADMapBin; 
class OddVisualizer : public rclcpp::Node
{
public:
    explicit OddVisualizer(const std::string& node_name, const rclcpp::NodeOptions & node_options);
private:
    // ROS subscribers and publishers
    rclcpp::Subscription<HADMapBin>::SharedPtr map_subscriber_;
    rclcpp::Subscription<laneSequenceWithID>::SharedPtr lanelet_sequence_subscriber_;

    rclcpp::Publisher<MarkerArray>::SharedPtr odd_driveable_area_publisher_;
    // rclcpp::Publisher<EdgeData>::SharedPtr odd_edge_publisher_;
    // rclcpp::Publisher<LaneData>::SharedPtr odd_roadLane_publisher_;
    // rclcpp::Publisher<RegulatoryEleData>::SharedPtr odd_regulatory_elements_publisher_;

    // tier4_autoware_utils::SelfPoseListener self_pose_listener_{this};


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

    // lanetype
    lanelet::ConstLanelets road_lanelets_;
    lanelet::ConstLanelets walkway_lanelets_;
    lanelet::ConstLanelets crosswalk_lanelets_;
    lanelet::ConstLanelets bus_lanelets_;
    lanelet::ConstLanelets bicycle_lanelets_;
    lanelet::ConstLanelets emergency_lanelets_;
    

    // TODO: move all ODD elements into odd_elements_
    // regulatary elements
    lanelet::TrafficLightConstPtr trafficLights_;

    // edge
    lanelet::ConstLanelet roadShoulder_edge;

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

    void creareDrivableBoundaryMarkerArray(const lanelet::ConstLanelets laneletSequence);
    // void getCurrentLane();

    MarkerArray createDrivableAreaMarkerArray(const lanelet::ConstLineStrings3d & linestrings);

    std_msgs::msg::ColorRGBA getColorRGBAmsg(odd_tools::odd_colorRGBA &odd_color);
    void getColorConfig(odd_tools::odd_colorRGBA &odd_color);
    odd_tools::ODD_elements getParam();

    // // lane msgs
    // scenery_msgs::msg::LaneData createRoadLaneMsg();
    // scenery_msgs::msg::LaneData createWalkWayLaneMsg();
    // scenery_msgs::msg::LaneData createCrossWalkLaneMsg();
    // scenery_msgs::msg::LaneData createBUSLaneMsg();
    // scenery_msgs::msg::LaneData createBicycleLaneMsg();
    // scenery_msgs::msg::LaneData createEmergencyLaneMsg();

    // regulatary elements msgs


    // edge msgs

};
} // namespace odd_visualizer

#endif // ODD_VISUALIZER_NODE_HPP_