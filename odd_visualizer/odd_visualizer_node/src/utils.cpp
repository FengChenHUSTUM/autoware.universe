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

    BoundaryInfo getBoundaryLineString(
        const lanelet::ConstLanelets laneletSequence,
        const lanelet::ConstLanelet currentLanelet,
        const geometry_msgs::msg::Pose & pose,
        const double forwardLength,
        const double backwardLength)
    {
        // std::cout << "forward: " << forwardLength << "; backward: " << backwardLength << '\n';

        // lanelet::ConstLanelet currentLanelet;
        int curLaneletPosition = getCurrentLaneletPosition(laneletSequence, pose, currentLanelet);
        if (curLaneletPosition == -1) {
            return {};
        }
        double curLength = backwardLength;

        // current position of the ego
        const auto poseLanelet = toLaneletPoint(pose.position);

        const auto curCenterLine = lanelet::utils::to2D(currentLanelet.centerline());
        const auto projectedPoint = lanelet::geometry::internal::signedDistanceImpl(curCenterLine, poseLanelet.basicPoint2d()).second;
        // find the corrsponding projected point of the current pose on the centerline
        // auto iterStartPoint = curCenterLine.begin();
        size_t poseCenterline = 0;
        size_t furtherstLaneletIdx = 0;
        size_t furtherstPointIdx = 0;
        for (size_t i = 0; i < curCenterLine.size(); ++i) {
            if (boost::geometry::equals(projectedPoint.result->segmentPoint1, curCenterLine[i].basicPoint2d())) {
                // iterStartPoint = itPoint;
                // std::cout << "这样找点确实可以, point id: " << itPoint->id()<<'\n';
                curLength = 0;
                poseCenterline = i;
            }
            if (i > 0) {
                curLength += lanelet::geometry::distance(curCenterLine[i].basicPoint2d(), 
                                                        (curCenterLine[i - 1]).basicPoint2d());
            }
            //check if the accumulate length in current lanelet has exceeded the fix forward lengt
            if (curLength >= forwardLength) {
                furtherstLaneletIdx = curLaneletPosition;
                return {poseCenterline, furtherstLaneletIdx, i};
            }
        }

        // if the the accumulate length is shorter than the fixed length, continue the above process
        // in the following lanlets in the sequence
        // for (auto itLanlet = laneletSequence.begin();
        int i = 0;
        // std::cout << "curLaneletPosition" <<curLaneletPosition<<'\n';
        for (size_t lanelet = curLaneletPosition + 1; lanelet < laneletSequence.size(); ++lanelet)
        {   
            const auto centerLine = lanelet::utils::to2D(laneletSequence[lanelet].centerline());
            furtherstLaneletIdx++;
            furtherstPointIdx = 0;
            for (auto itPoint = centerLine.begin(); itPoint != centerLine.end(); ++itPoint) {
                furtherstPointIdx++;

                if (itPoint != centerLine.begin()) {
                    curLength += lanelet::geometry::distance(itPoint->basicPoint2d(), (itPoint - 1)->basicPoint2d());
                }
                if (curLength >= forwardLength) {
                    return {poseCenterline, furtherstLaneletIdx, furtherstPointIdx};
                }
            }
            std::cout << "loop end: "<< i++ << '\n';
        }
        return {};
    }
    size_t getRightProjectedPointId(const lanelet::ConstLanelet & currentLanelet,
                                    const size_t & pointID)
    {
        const auto curCenterLine = lanelet::utils::to2D(currentLanelet.centerline());
        const auto rightProjection = lanelet::geometry::internal::signedDistanceImpl(currentLanelet.rightBound2d(), curCenterLine[pointID].basicPoint2d()).second;
        for (size_t i = 0; i < currentLanelet.rightBound2d().size(); ++i) {
            if(boost::geometry::equals(rightProjection.result->segmentPoint1, currentLanelet.rightBound2d()[i].basicPoint2d())) {
                return i;
            }
        }
        return 0;
    }

    size_t getLeftProjectedPointId(const lanelet::ConstLanelet & currentLanelet,
                                    const size_t & pointID)
    {
        const auto curCenterLine = lanelet::utils::to2D(currentLanelet.centerline());
        const auto leftProjection = lanelet::geometry::internal::signedDistanceImpl(currentLanelet.leftBound2d(), curCenterLine[pointID].basicPoint2d()).second;
        for (size_t i = 0; i < currentLanelet.leftBound2d().size(); ++i) {
            if(boost::geometry::equals(leftProjection.result->segmentPoint1, currentLanelet.leftBound2d()[i].basicPoint2d())) {
                return i;
            }
        }
        return 0;
    }

    int getCurrentLaneletPosition(const lanelet::ConstLanelets & laneletSequence, 
                                  const geometry_msgs::msg::Pose & currentPose,
                                  const lanelet::ConstLanelet & currentLanelet) 
    {
        int res = -1;
        lanelet::BasicPoint2d curPoint(currentPose.position.x, currentPose.position.y);
        for (size_t i = 0; i < laneletSequence.size(); ++i) {
            // if (lanelet::geometry::inside(laneletSequence[i], curPoint)) {
                if ( currentLanelet == laneletSequence[i]){
                // currentLanelet = laneletSequence[i];
                res = i;
                return res;
            }
        }
        return res;
    }

    void onNonDrivableLane(const lanelet::ConstLanelet lanelet) 
    {
        if (lanelet.hasAttribute("EmergencyLane")) {
            std::cout << "LaneID: " << lanelet.id() << " is emergency lane" << '\n';
            //TODO: populate the function here
        }
        for (auto attr : lanelet.attributes())
        {
            std::cout << "LaneID: " << lanelet.id() << "; attribute: "<< attr.first
            << "; "<< attr.second.value() << '\n';
        }
        // TODO: add other params checker
    }

    void onEdge(const lanelet::ConstLanelet lanelet) 
    {
        if (lanelet.hasAttribute("StopLine")) {
            std::cout << "LaneID: " << lanelet.id() << " has line marker: stop line" << '\n';
            //TODO: populate the function here
        }
        // TODO: add other params checker
    }

    void onFixedRoadStructures(const lanelet::ConstLanelet lanelet) 
    {
        if (lanelet.hasAttribute("Vegetation")) {
            std::cout << "LaneID: " << lanelet.id() << " has fixed structure: vegetation" << '\n';
            //TODO: populate the function here
        }
        // TODO: add other params checker
    }

    void onRegulations(const lanelet::ConstLanelet lanelet) 
    {
        int i = 0;
        for (auto regulation : lanelet.regulatoryElements())
        {
            std::cout << "regulation size "<<  i++ << " : "<<regulation->size()<< '\n';
            for (auto elem : regulation->attributes()) {
                std::cout << "Regulation attributes: " << elem.first
                << "; "<< elem.second.value() << '\n';
            }
        }
    }
} // namespace odd_tools
