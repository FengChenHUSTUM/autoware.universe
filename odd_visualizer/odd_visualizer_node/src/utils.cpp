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

    std::vector<geometry_msgs::msg::Point>  getLaneMarkerPointsFromLanelets(const lanelet::Lanelets & lanelets,
                                                                            bool sameDirection) {
        std::vector<geometry_msgs::msg::Point> res;
        if (sameDirection) {
            if (lanelets.size() > 1)
            {res.push_back(createPoint((lanelets.begin()->rightBound().basicLineString().end() - 1)->x(),
                                    (lanelets.begin()->rightBound().basicLineString().end() - 1)->y(),
                                    (lanelets.begin()->rightBound().basicLineString().end() - 1)->z()));
            for (auto & ll : lanelets) {
                const auto lineString = ll.leftBound().basicLineString();
                for (auto point = lineString.rbegin(); point != lineString.rend(); point++) {
                    res.push_back(createPoint(point->x(), point->y(), point->z()));
                }
            }
            res.push_back(createPoint((lanelets.rbegin()->rightBound().basicLineString().begin())->x(),
                                    (lanelets.rbegin()->rightBound().basicLineString().begin())->y(),
                                    (lanelets.rbegin()->rightBound().basicLineString().begin())->z()));
            }
            else {
                res.push_back(createPoint(lanelets[0].rightBound().basicLineString().begin()->x(),
                                    lanelets[0].rightBound().basicLineString().begin()->y(),
                                    lanelets[0].rightBound().basicLineString().begin()->z()));
                for (auto & point : lanelets[0].leftBound().basicLineString()){
                    res.push_back(createPoint(point.x(), point.y(), point.z()));
                }
                res.push_back(createPoint((lanelets[0].rightBound().basicLineString().end() - 1)->x(),
                                    (lanelets[0].rightBound().basicLineString().end() - 1)->y(),
                                    (lanelets[0].rightBound().basicLineString().end() - 1)->z()));
            }
        }
        else {
            std::vector<geometry_msgs::msg::Point> LeftPoints, RightPoints;
            for (auto ll : lanelets) {
                const auto leftLine = ll.leftBound().basicLineString();
                const auto rightLine = ll.rightBound().basicLineString();
                for (auto point = leftLine.rbegin(); point != leftLine.rend(); ++point) {
                    LeftPoints.push_back(createPoint(point->x(), point->y(), point->z()));
                }
                for (auto point = rightLine.rbegin(); point != rightLine.rend(); ++point) {
                    RightPoints.push_back(createPoint(point->x(), point->y(), point->z()));
                }
            }
            res.insert(res.end(), LeftPoints.begin(), LeftPoints.end());
            res.insert(res.end(), RightPoints.rbegin(), RightPoints.rend());
            res.push_back(LeftPoints[0]);
        }

        return res;
    }


    size_t getRightProjectedPointId(const lanelet::ConstLanelet & currentLanelet,
                                    const size_t & pointID)
    {
        const auto curCenterLine = lanelet::utils::to3D(currentLanelet.centerline());
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
        const auto curCenterLine = lanelet::utils::to3D(currentLanelet.centerline());
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
        // for (auto attr : lanelet.attributes())
        // {
        //     std::cout << "LaneID: " << lanelet.id() << "; attribute: "<< attr.first
        //     << "; "<< attr.second.value() << '\n';
        // }
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

    double getArcLengthFromPoints(lanelet::ConstLineString3d::ThreeDType inputLine)
    {
        double res = 0;
        for (size_t i = 0; i < inputLine.size(); ++i) {
            if (i > 0) {
                res += boost::geometry::distance(inputLine[i].basicPoint2d(),
                                                 inputLine[i - 1].basicPoint2d());
            }
        }
        return res;
    }

    std::vector<geometry_msgs::msg::Pose> resampleLine(const lanelet::ConstLineString3d::ThreeDType & curCenterLine,
                                                       const double & interval)
    {
        // convert points in current centerline into poses for resampling
        std::vector<geometry_msgs::msg::Pose> centerlinePoses(curCenterLine.size());
        for (size_t i = 0; i < curCenterLine.size(); ++i) {
            centerlinePoses[i] = createPose(curCenterLine[i].x(),
                                            curCenterLine[i].y(),
                                            curCenterLine[i].z());
        }
        // resample the centerline points
        // caculate the whole arc length of the line to be resampled
        std::vector<double> resampled_arclength;
        const double arcLength = getArcLengthFromPoints(curCenterLine);
        const auto duplicatePoint = [&](const auto & vec, const auto x) {
            if (vec.empty()) return false;
            const auto has_close = [&](const auto v) { return std::abs(v - x) < 0.01; };
            return std::find_if(vec.begin(), vec.end(), has_close) != vec.end();
        };
        for (double dis = 0.0; dis < arcLength; dis+= interval) {
            if (!duplicatePoint(resampled_arclength, dis)) {
            resampled_arclength.push_back(dis);
            }
        }

        return motion_utils::resamplePoseVector(centerlinePoses, resampled_arclength);
    }

    bgPoint getFurtherestForwardPoint(const lanelet::ConstLanelet & currentLanelet,
                                      const geometry_msgs::msg::Pose & pose,
                                      const double & fixedLength,
                                      double & curLength,
                                      const double & interval)
    {
        // current position of the ego
        const auto egoPoint = bgPoint(pose.position.x, pose.position.y, pose.position.z);
        const auto curCenterLine = lanelet::utils::to3D(currentLanelet.centerline());

        const auto resampledCenterline = resampleLine(curCenterLine, interval);

        // find the nearest projected point of the current pose of the ego in the centerline
        size_t poseInCenterline = 0;
        double minDis = std::numeric_limits<double>::max();
        for (size_t i = 0; i < resampledCenterline.size(); ++i) {
            bgPoint point1(resampledCenterline[i].position.x, resampledCenterline[i].position.y, resampledCenterline[i].position.z);

            double tmpDis = boost::geometry::distance(point1, egoPoint);
            if (tmpDis < minDis) {
            minDis = tmpDis;
            poseInCenterline = i;
            }
        }

        // caculate accumulate distance from the projected point to the furtherest point within the forward range one by one
        // return the furtherest point
        for (size_t i = poseInCenterline; i < resampledCenterline.size(); ++i) {
            if (i > 0) {
            curLength += boost::geometry::distance(bgPoint(resampledCenterline[i].position.x, resampledCenterline[i].position.y, resampledCenterline[i].position.z),
                                                    bgPoint(resampledCenterline[i - 1].position.x, resampledCenterline[i - 1].position.y, resampledCenterline[i - 1].position.z));
            }
            if (curLength >= fixedLength) {
            return bgPoint(resampledCenterline[i].position.x, resampledCenterline[i].position.y, resampledCenterline[i].position.z);
            }
        }
        return bgPoint(resampledCenterline.end()->position.x, resampledCenterline.end()->position.y, resampledCenterline.end()->position.z);
    }


    bgPoint getFurtherestForwardPoint(const lanelet::ConstLanelet & currentLanelet,
                                      const double & fixedLength,
                                      double & curLength,
                                      const double & interval)
    {
        // convert points in current centerline into poses for resampling
        const auto curCenterLine = lanelet::utils::to3D(currentLanelet.centerline());
        const auto resampledCenterline = resampleLine(curCenterLine, interval);

        // caculate accumulate distance from the projected point to the furtherest point within the forward range one by one
        // return the furtherest point
        for (size_t i = 0; i < resampledCenterline.size(); ++i) {
            if (i > 0) {
            curLength += boost::geometry::distance(bgPoint(resampledCenterline[i].position.x, resampledCenterline[i].position.y, resampledCenterline[i].position.z),
                                                    bgPoint(resampledCenterline[i - 1].position.x, resampledCenterline[i - 1].position.y, resampledCenterline[i - 1].position.z));
            }
            if (curLength >= fixedLength) {
            return bgPoint(resampledCenterline[i].position.x, resampledCenterline[i].position.y, resampledCenterline[i].position.z);
            }
        }
        return bgPoint(resampledCenterline.end()->position.x, resampledCenterline.end()->position.y, resampledCenterline.end()->position.z);
    }
    bgPoint getFurtherestBackwardPoint(const lanelet::ConstLanelet & currentLanelet,
                                      const geometry_msgs::msg::Pose & pose,
                                      const double & fixedLength,
                                      double & curLength,
                                      const double & interval)
    {
        // current position of the ego
        const auto egoPoint = bgPoint(pose.position.x, pose.position.y, pose.position.z);
        const auto curCenterLine = lanelet::utils::to3D(currentLanelet.centerline());

        const auto resampledCenterline = resampleLine(curCenterLine, interval);

        // find the nearest projected point of the current pose of the ego in the centerline
        size_t poseInCenterline = 0;
        double minDis = std::numeric_limits<double>::max();
        for (size_t i = 0; i < resampledCenterline.size(); ++i) {
            bgPoint point1(resampledCenterline[i].position.x, resampledCenterline[i].position.y, resampledCenterline[i].position.z);

            double tmpDis = boost::geometry::distance(point1, egoPoint);
            if (tmpDis < minDis) {
            minDis = tmpDis;
            poseInCenterline = i;
            }
        }

        // caculate accumulate distance from the projected point to the furtherest point within the forward range one by one
        // return the furtherest point
        for (size_t i = poseInCenterline; i > 0; --i) {
            if (i > 0) {
            curLength += boost::geometry::distance(bgPoint(resampledCenterline[i].position.x, resampledCenterline[i].position.y, resampledCenterline[i].position.z),
                                                    bgPoint(resampledCenterline[i - 1].position.x, resampledCenterline[i - 1].position.y, resampledCenterline[i - 1].position.z));
            }
            if (curLength >= fixedLength) {
            return bgPoint(resampledCenterline[i].position.x, resampledCenterline[i].position.y, resampledCenterline[i].position.z);
            }
        }
        return bgPoint(resampledCenterline.begin()->position.x, resampledCenterline.begin()->position.y, resampledCenterline.begin()->position.z);
    }


    bgPoint getFurtherestBackwardPoint(const lanelet::ConstLanelet & currentLanelet,
                                      const double & fixedLength,
                                      double & curLength,
                                      const double & interval)
    {
        // convert points in current centerline into poses for resampling
        const auto curCenterLine = lanelet::utils::to3D(currentLanelet.centerline());
        const auto resampledCenterline = resampleLine(curCenterLine, interval);

        // caculate accumulate distance from the projected point to the furtherest point within the forward range one by one
        // return the furtherest point
        for (size_t i = resampledCenterline.size() - 1; i > 0; --i) {
            if (i > 0) {
            curLength += boost::geometry::distance(bgPoint(resampledCenterline[i].position.x, resampledCenterline[i].position.y, resampledCenterline[i].position.z),
                                                    bgPoint(resampledCenterline[i - 1].position.x, resampledCenterline[i - 1].position.y, resampledCenterline[i - 1].position.z));
            }
            if (curLength >= fixedLength) {
            return bgPoint(resampledCenterline[i].position.x, resampledCenterline[i].position.y, resampledCenterline[i].position.z);
            }
        }
        return bgPoint(resampledCenterline.begin()->position.x, resampledCenterline.begin()->position.y, resampledCenterline.begin()->position.z);
    }

    std::vector<geometry_msgs::msg::Point> getMarkerPoints(const std::vector<geometry_msgs::msg::Pose> & bound,
                                                           const bgPoint & startPoint,
                                                           const bgPoint & endPoint)
    {
        double startMinDis = std::numeric_limits<double>::max();
        double endMinDis = startMinDis;
        size_t startPointIndex = 0;
        size_t endPointIndex = 0;

        for (size_t i = 0; i < bound.size(); ++i) {
            bgPoint point(bound[i].position.x,
                        bound[i].position.y,
                        bound[i].position.z);
            double tmpDis = boost::geometry::distance(point, startPoint);
            if (tmpDis < startMinDis) {
                startMinDis = tmpDis;
                startPointIndex = i;
            }
        }

        for (size_t i = 0; i < bound.size(); ++i) {
            bgPoint point(bound[i].position.x,
                        bound[i].position.y,
                        bound[i].position.z);
            double tmpDis = boost::geometry::distance(point, endPoint);
            if (tmpDis < endMinDis) {
                endMinDis = tmpDis;
                endPointIndex = i;
            }
        }
        std::vector<geometry_msgs::msg::Point> res;
        if (endPointIndex - startPointIndex > 0) {
            // res.reserve(endPointIndex - startPointIndex);
            for (size_t i = startPointIndex; i < endPointIndex; ++i) {
                res.push_back(bound[i].position);
            }
        }
        return res;
    }

    std::vector<geometry_msgs::msg::Point> getCenterPoint(const std::vector<geometry_msgs::msg::Point> & leftBound,
                                                          const std::vector<geometry_msgs::msg::Point> & rightBound)
    {
        std::vector<geometry_msgs::msg::Point> res(2);
        if (leftBound.size() > 2 && rightBound.size() > 3) {
            const auto leftPoint = leftBound.begin() + (leftBound.size() / 2);
            const auto rightPoint = rightBound.begin() + (rightBound.size() / 2);
            res[0] = createPoint(leftPoint->x / 2 + rightPoint->x / 2,
                                leftPoint->y / 2 + rightPoint->y / 2,
                                leftPoint->z / 2 + rightPoint->z / 2);
            res[1] = createPoint((leftPoint + 2)->x / 2 + (rightPoint + 2)->x / 2,
                                (leftPoint + 2)->y / 2 + (rightPoint + 2)->y / 2,
                                (leftPoint + 2)->z / 2 + (rightPoint + 2)->z / 2);
        }
        return res;
    }
    std::vector<geometry_msgs::msg::Point> getArrowsInOneLanelet(const std::vector<geometry_msgs::msg::Pose> & centerLine)
    {
        std::vector<geometry_msgs::msg::Point> arrows;
        if (centerLine.size() > 1) {
            arrows.reserve(centerLine.size());
        }
        for (size_t i = 0; i < centerLine.size(); i += 3) {
            if (i + 1 < centerLine.size()) {
                arrows.push_back(centerLine[i].position);
                arrows.push_back(centerLine[i + 2].position);
            }
        }
        return arrows;
    }
} // namespace odd_tools
