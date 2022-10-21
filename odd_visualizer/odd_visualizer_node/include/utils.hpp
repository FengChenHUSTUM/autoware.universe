#ifndef ODD_VISUALIZER_UTILS_HPP_
#define ODD_VISUALIZER_UTILS_HPP_


#include <vector>
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

#include <route_handler/route_handler.hpp>


namespace odd_tools
{
    struct odd_colorRGBA {
        float r = 255.0;
        float g = 255.0;
        float b  =255.0;
        float a = 1.0;
    };
    struct tuning_params {
        double forward_path_length = 20;
        double backward_path_length = 10;
        odd_colorRGBA odd_rgba{};
    };
    struct ODD_elements {
        tuning_params params{};
        // PoseStamped::ConstSharedPtr self_pose{};

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
} // namespace odd_tools

#endif