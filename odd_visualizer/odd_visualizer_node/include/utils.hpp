#ifndef ODD_VISUALIZER_UTILS_HPP_
#define ODD_VISUALIZER_UTILS_HPP_


#include <vector>
// Lanelet
#include <lanelet2_io/Io.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/GeometryHelper.h>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <route_handler/route_handler.hpp>
#include <tier4_autoware_utils/ros/self_pose_listener.hpp>

namespace odd_tools
{
    struct odd_colorRGBA {
        float r = 255.0;
        float g = 255.0;
        float b  =255.0;
        float a = 1.0;
    };
    struct tuning_params {
        float forward_path_length = 20.0;
        float backward_path_length = 10.0;
        odd_colorRGBA odd_rgba{};
    };
    struct ODD_elements {
        tuning_params params{};
        geometry_msgs::msg::PoseStamped::ConstSharedPtr self_pose{};
    };
    std::vector<int64_t> getLaneletIDsfromSequence(
        lanelet::ConstLanelet& current_lane,
        const route_handler::RouteHandler& route_handler_ontology,
        geometry_msgs::msg::Pose & pose);

    boost::optional<lanelet::ConstLanelet> getRightLanelet (
        const lanelet::ConstLanelet & lanelet,
        const lanelet::routing::RoutingGraphPtr routing_graph_ptr_);

    lanelet::Lanelets getRightOppositeLanelets(
        const lanelet::ConstLanelet & lanelet,
        const lanelet::LaneletMapPtr lanelet_map_ptr_);

    lanelet::ConstLineString3d getRightMostLinestring(
        const lanelet::ConstLanelet & lanelet,
        const lanelet::routing::RoutingGraphPtr routing_graph_ptr_,
        const lanelet::LaneletMapPtr lanelet_map_ptr_);

    // get left lanelet
    boost::optional<lanelet::ConstLanelet> getLeftLanelet (
        const lanelet::ConstLanelet & lanelet,
        const lanelet::routing::RoutingGraphPtr routing_graph_ptr_);

    lanelet::Lanelets getLeftOppositeLanelets(
        const lanelet::ConstLanelet & lanelet,
        const lanelet::LaneletMapPtr lanelet_map_ptr_);

    lanelet::ConstLineString3d getLeftMostLinestring(
        const lanelet::ConstLanelet & lanelet,
         lanelet::routing::RoutingGraphPtr routing_graph_ptr_,
         lanelet::LaneletMapPtr lanelet_map_ptr_);

    // Geometry functions
    struct BoundaryInfo {
        size_t posePointIDLanelet{0};
        size_t furtherestLaneletId{0};
        size_t furtherestPointId{0};
    };
    BoundaryInfo getBoundaryLineString(
        const lanelet::ConstLanelets laneletSequence,
        const lanelet::ConstLanelet currentLanelet,
        const geometry_msgs::msg::Pose & pose,
        const double forwardLength,
        const double backwardLength);
    size_t getRightProjectedPointId(const lanelet::ConstLanelet & currentLanelet,
                                    const size_t & pointID);
    size_t getLeftProjectedPointId(const lanelet::ConstLanelet & currentLanelet,
                                   const size_t & pointID);

    int getCurrentLaneletPosition(const lanelet::ConstLanelets & laneletSequence, 
                                  const geometry_msgs::msg::Pose & currentPose,
                                  const lanelet::ConstLanelet & currentLanelet);

    inline lanelet::ConstPoint3d toLaneletPoint(const geometry_msgs::msg::Point & pose)
    {
        return lanelet::Point3d(lanelet::InvalId, pose.x, pose.y, pose.z);
    }
    void onNonDrivableLane(const lanelet::ConstLanelet lanelet);
    void onEdge(const lanelet::ConstLanelet lanelet);
    void onFixedRoadStructures(const lanelet::ConstLanelet lanelet);
    void onRegulations(const lanelet::ConstLanelet lanelet);
    inline geometry_msgs::msg::Pose createPose(const double x, const double y, const double z) {
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        return pose;
    }
    double getArcLengthFromPoints(lanelet::ConstLineString3d::TwoDType inputLine);
    // lanelet::ArcCoordinates getArcCoordinates(
    //     const lanelet::ConstLanelets & laneletSequence,
    //     const geometry_msgs::msg::Pose & pose);
} // namespace odd_tools

#endif