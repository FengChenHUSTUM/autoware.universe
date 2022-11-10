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

#define bgPoint  boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>

namespace odd_tools
{
    struct tuning_params {
        float forward_path_length = 20.0;
        float backward_path_length = 10.0;
        std::vector<int64_t> odd_rgb{};
        std::vector<int64_t> oppositeLane_rgb{};
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


    std::vector<geometry_msgs::msg::Point> getLaneMarkerPointsFromLanelets(const lanelet::Lanelets & lanelets);

    size_t getRightProjectedPointId(const lanelet::ConstLanelet & currentLanelet,
                                    const size_t & pointID);
    size_t getLeftProjectedPointId(const lanelet::ConstLanelet & currentLanelet,
                                   const size_t & pointID);

    int getCurrentLaneletPosition(const lanelet::ConstLanelets & laneletSequence, 
                                  const geometry_msgs::msg::Pose & currentPose,
                                  const lanelet::ConstLanelet & currentLanelet);

    inline bool isNewPoseSet(const geometry_msgs::msg::PoseStamped* oldPose,
                             const geometry_msgs::msg::PoseStamped* newPose)
    {
        return (abs(oldPose->pose.position.x - newPose->pose.position.x) +
                abs(oldPose->pose.position.y - newPose->pose.position.y) > 20);
    }
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
    inline geometry_msgs::msg::Point createPoint(const double x, const double y, const double z) {
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        return point;
    }
    double getArcLengthFromPoints(lanelet::ConstLineString3d::TwoDType inputLine);

    /**
     * @brief resample a line string with an interval
     * 
     * @param curCenterLine 
     * @param interval 
     * @return std::vector<geometry_msgs::msg::Pose> 
     */
    std::vector<geometry_msgs::msg::Pose> resampleLine(const lanelet::ConstLineString3d::TwoDType & curCenterLine,
                                                       const double & interval);

    /**
     * @brief When the furtherest point is in current lanelet,
    * get the Furtherest Forward Point object from the center line of this lanelet
    * 
    * @param currentLanelet 
    * @param pose 
    * @param interval 
    * @param fixedLength 
    * @param curLength 
    * @return the position of the Furtherest Forward Point in the centerline 
    * (boost geometry point model) 
    */
    bgPoint getFurtherestForwardPoint(const lanelet::ConstLanelet & currentLanelet,
                                      const geometry_msgs::msg::Pose & pose,
                                      const double & fixedLength,
                                      double & curLength,
                                      const double & interval = 1.0);
    bgPoint getFurtherestBackwardPoint(const lanelet::ConstLanelet & currentLanelet,
                                       const geometry_msgs::msg::Pose & pose,
                                       const double & fixedLength,
                                       double & curLength,
                                       const double & interval = 1.0);
    /**
     * @brief Get the Furtherest Forward Point object
     * 
     * @param currentLanelet 
     * @param fixedLength 
     * @param curLength 
     * @param interval
     * @return the position of the Furtherest Forward Point in the centerline 
     * (boost geometry point model) 
     */
    bgPoint getFurtherestForwardPoint(const lanelet::ConstLanelet & currentLanelet,
                                      const double & fixedLength,
                                      double & curLength,
                                      const double & interval = 1.0);
    bgPoint getFurtherestBackwardPoint(const lanelet::ConstLanelet & currentLanelet,
                                       const double & fixedLength,
                                       double & curLength,
                                       const double & interval = 1.0);

    /**
     * @brief Get the Marker Points object from a lanlet boundary
     * 
     * @param bound 
     * @param startPoint 
     * @param endPoint 
     * @return std::vector<geometry_msgs::msg::Point> 
     */
    std::vector<geometry_msgs::msg::Point> getMarkerPoints(const std::vector<geometry_msgs::msg::Pose> & bound,
                                                           const bgPoint & startPoint,
                                                           const bgPoint & endPoint);
    // lanelet::ArcCoordinates getArcCoordinates(
    //     const lanelet::ConstLanelets & laneletSequence,
    //     const geometry_msgs::msg::Pose & pose);
} // namespace odd_tools

#endif