#include "../include/utils.hpp"

static double forward_path_length = 20;
static double backward_path_length = 10;

namespace odd_tools
{
    std::vector<int64_t> getLaneletIDsfromSequence(
        lanelet::ConstLanelet& current_lane,
        const route_handler::RouteHandler & route_handler_ontology,
        geometry_msgs::msg::Pose & pose) {

        route_handler_ontology.getClosestLaneletWithinRoute(pose, &current_lane);
        const std::vector<lanelet::ConstLanelet> current_lanes = route_handler_ontology.getLaneletSequence(
            current_lane, pose, backward_path_length, forward_path_length);
            
        std::vector<int64_t> IDs(current_lanes.size(), 0);
        for(long unsigned int i = 0; i < IDs.size(); ++i) {
            IDs[i] = current_lanes[i].id();
        }
        return IDs;
    }

    // get right lantlet
    boost::optional<lanelet::ConstLanelet> getRightLanelet (
        const lanelet::ConstLanelet & lanelet,
        const lanelet::routing::RoutingGraphPtr routing_graph_ptr_) {

        // routable lane
        const auto & right_lane = routing_graph_ptr_->right(lanelet);
        if (right_lane) {
            return right_lane;
        }

        // non-routable lane (e.g. lane change infeasible)
        const auto & adjacent_right_lane = routing_graph_ptr_->adjacentRight(lanelet);
        return adjacent_right_lane;
    }

    lanelet::Lanelets getRightOppositeLanelets(
        const lanelet::ConstLanelet & lanelet,
        const lanelet::LaneletMapPtr lanelet_map_ptr_)
    {
        const auto opposite_candidate_lanelets =
            lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound().invert());

        lanelet::Lanelets opposite_lanelets;
        for (const auto & candidate_lanelet : opposite_candidate_lanelets) {
            if (candidate_lanelet.leftBound().id() == lanelet.rightBound().id()) {
            continue;
            }

            opposite_lanelets.push_back(candidate_lanelet);
        }

        return opposite_lanelets;
    }

    lanelet::ConstLineString3d getRightMostLinestring(
        const lanelet::ConstLanelet & lanelet,
        const lanelet::routing::RoutingGraphPtr routing_graph_ptr_,
        const lanelet::LaneletMapPtr lanelet_map_ptr_)
    {
        const auto & same = getRightLanelet(lanelet, routing_graph_ptr_);
        const auto & opposite = getRightOppositeLanelets(lanelet, lanelet_map_ptr_);
        if (!same && opposite.empty()) {
            return lanelet.rightBound();
        } else if (same) {
            return getRightMostLinestring(same.get(), 
                                          routing_graph_ptr_,
                                          lanelet_map_ptr_);
        } else if (!opposite.empty()) {
            return getLeftMostLinestring(lanelet::ConstLanelet(opposite.front()),
                                         routing_graph_ptr_,
                                         lanelet_map_ptr_);
        }
        return {};
    }


    // get left lanelet
    boost::optional<lanelet::ConstLanelet> getLeftLanelet (
        const lanelet::ConstLanelet & lanelet,
        const lanelet::routing::RoutingGraphPtr routing_graph_ptr_) {

        // routable lane
        const auto & left_lane = routing_graph_ptr_->left(lanelet);
        if (left_lane) {
            return left_lane;
        }

        // non-routable lane (e.g. lane change infeasible)
        const auto & adjacent_left_lane = routing_graph_ptr_->adjacentLeft(lanelet);
        return adjacent_left_lane;
    }

    lanelet::Lanelets getLeftOppositeLanelets(
        const lanelet::ConstLanelet & lanelet,
        const lanelet::LaneletMapPtr lanelet_map_ptr_)
    {
        const auto opposite_candidate_lanelets =
            lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound().invert());

        lanelet::Lanelets opposite_lanelets;
        for (const auto & candidate_lanelet : opposite_candidate_lanelets) {
            if (candidate_lanelet.leftBound().id() == lanelet.rightBound().id()) {
            continue;
            }

            opposite_lanelets.push_back(candidate_lanelet);
        }

        return opposite_lanelets;
    }

    lanelet::ConstLineString3d getLeftMostLinestring(
        const lanelet::ConstLanelet & lanelet,
         lanelet::routing::RoutingGraphPtr routing_graph_ptr_,
         lanelet::LaneletMapPtr lanelet_map_ptr_)
    {
        const auto & same = getLeftLanelet(lanelet, routing_graph_ptr_);
        const auto & opposite = getLeftOppositeLanelets(lanelet, lanelet_map_ptr_);
        if (!same && opposite.empty()) {
            return lanelet.leftBound();
        } else if (same) {
            return getLeftMostLinestring(same.get(), 
                                          routing_graph_ptr_,
                                          lanelet_map_ptr_);
        } else if (!opposite.empty()) {
            return getRightMostLinestring(lanelet::ConstLanelet(opposite.front()),
                                         routing_graph_ptr_,
                                         lanelet_map_ptr_);
        }
        return {};
    }

} // namespace odd_tools
