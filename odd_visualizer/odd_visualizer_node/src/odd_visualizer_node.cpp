#include "../include/odd_visualizer_node.hpp"

namespace odd_visualizer {
OddVisualizer::OddVisualizer(
    const std::string& node_name, const rclcpp::NodeOptions & node_options) 
: Node(node_name, node_options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    odd_elements_ = std::make_shared<odd_tools::ODD_elements>();
    *odd_elements_ = getParam();
    current_lanelet_ = std::make_shared<lanelet::ConstLanelet>();
    //TODO: edit topic path in configuration file
    std::cout << "odd node loaded! "<<'\n';
    odd_driveable_area_publisher_ = create_publisher<MarkerArray>("/odd_parameter/odd_drivable_area", 1);
    odd_adjacent_lane_publisher_ = create_publisher<MarkerArray>("/odd_parameter/adjacent_lane", 1);


    map_subscriber_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "/map/vector_map", 10,
    std::bind(&OddVisualizer::mapCallback, this, std::placeholders::_1));

    lanelet_sequence_subscriber_ = this->create_subscription<laneSequenceWithID>(
        "/planning/scenario_planning/lane_driving/behavior_planning/odd_visualizer/lanelet_sequence_IDs", 1,
    std::bind(&OddVisualizer::laneletSequenceCallback, this, std::placeholders::_1));

}
void OddVisualizer::run() {
    
}

std_msgs::msg::ColorRGBA OddVisualizer::getColorRGBAmsg(odd_tools::odd_colorRGBA &odd_color) {
  std_msgs::msg::ColorRGBA colorConfig;
  colorConfig.r = odd_color.r;
  colorConfig.g = odd_color.g;
  colorConfig.b = odd_color.b;
  colorConfig.a = odd_color.a;
  return colorConfig;
}

void OddVisualizer::getColorConfig(odd_tools::odd_colorRGBA &odd_color) {
  // RCLCPP_INFO(get_logger(), "Get Params Func Called!");
  odd_color.r = declare_parameter<float>("color_rgba_r");
  odd_color.g = declare_parameter<float>("color_rgba_g");
  odd_color.b = declare_parameter<float>("color_rgba_b");
  odd_color.a = declare_parameter<float>("color_rgba_a");
}


odd_tools::ODD_elements OddVisualizer::getParam() {
  odd_tools::ODD_elements elements{};
  getColorConfig(elements.params.odd_rgba);
  return elements;
}


void OddVisualizer::mapCallback(
    const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg) {
    std::cout <<"odd map call back "<<'\n';
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(
        *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
    lanelet::ConstLanelets all_lanelets_ = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
}

void OddVisualizer::laneletSequenceCallback(laneSequenceWithID::ConstSharedPtr msg) {
  (*current_lanelet_) = lanelet_map_ptr_->laneletLayer.get(msg->current_lane);
  bool newPoseSet = false;
  if (odd_elements_->self_pose.get() != nullptr){
    newPoseSet = odd_tools::isNewPoseSet(odd_elements_->self_pose.get(),
                                         self_pose_listener_.getCurrentPose().get());
    if (newPoseSet) {
      current_lanelets_.resize(0);
      newPoseSet = false;
    }
  }
  odd_elements_->self_pose = self_pose_listener_.getCurrentPose();
  // when a new estimated location of the ego is set but the goal position is not set, 
  // the current lanelt will be added into the route. This means the size of current 
  // lanlets is one at this moment To avoid memory leak and make this pkg not died, 
  // the check condition should be like the following
  if (msg->lane_ids.size() > 1) {
    for (const auto & laneID : msg->lane_ids) {
      const auto ll = lanelet_map_ptr_->laneletLayer.get(laneID);
      if (std::find(current_lanelets_.begin(),current_lanelets_.end(), ll) == current_lanelets_.end()) 
      {
        current_lanelets_.push_back(ll);
      }
    }
    odd_driveable_area_publisher_->publish(createDriveableAreaBoundary());
    onAdjacentLanelet(*current_lanelet_);
    // getODDFromMap(current_lanelets_);
  }
}

MarkerArray OddVisualizer::createDriveableAreaBoundary() {
  MarkerArray msg;
  Marker centerLineMarker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "shared_linestring_lanelets", 0l, Marker::LINE_STRIP,
    createMarkerScale(0.5, 0.5, 0.5), getColorRGBAmsg(odd_elements_->params.odd_rgba));
  centerLineMarker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);

  Marker poseMarker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "ego_pose", 0l, Marker::SPHERE,
    createMarkerScale(1.0, 1.0, 1.0), getColorRGBAmsg(odd_elements_->params.odd_rgba));
  // double forwardLength = odd_elements_->params.forward_path_length;
  // const lanelet::ConstLanelets laneletSequence = current_lanelets_;
  const lanelet::ConstLanelet currentLanelet = *current_lanelet_;
  const geometry_msgs::msg::Pose pose = odd_elements_->self_pose->pose;
  poseMarker.pose = pose;
  msg.markers.push_back(poseMarker);

  // current position of the ego
  const auto egoPoint = bgPoint(pose.position.x, pose.position.y, 0);
  const auto curCenterLine = lanelet::utils::to2D(currentLanelet.centerline());
  
  double curLength = 0;
  // the postion of current lanelet in the lanelet sequence
  size_t curIndex = 0;
  // get the arclength of each lanelet in the sequence
  std::vector<double> lengthsLaneltes;
  // lengthsLaneltes.reserve(current_lanelets_.size());
  for (size_t i = 0; i < current_lanelets_.size(); ++i) {
    if (current_lanelets_[i] == currentLanelet) curIndex = i;
    const auto curllCenterLine = lanelet::utils::to2D(current_lanelets_[i].centerline());
    lengthsLaneltes.push_back(odd_tools::getArcLengthFromPoints(curllCenterLine));
  }
  // #################################################################
  // ############### Find the furtherest forward point ###############
  auto forwardPoint = odd_tools::getFurtherestForwardPoint(currentLanelet,
                                                           pose,
                                                           10,
                                                           curLength);
  // operations on current lanelet                       
  const auto curLeftBound = lanelet::utils::to2D(current_lanelets_[curIndex].leftBound());
  const auto curRightBound = lanelet::utils::to2D(current_lanelets_[curIndex].rightBound());
  
  const auto curResampledLeft = odd_tools::resampleLine(curLeftBound, 0.5);
  const auto curResampledRight = odd_tools::resampleLine(curRightBound, 0.5);

  auto leftMarkerPoints = odd_tools::getMarkerPoints(curResampledLeft,
                                                      egoPoint,
                                                      forwardPoint);
  auto rightMarkerPoints = odd_tools::getMarkerPoints(curResampledRight,
                                                      egoPoint,
                                                      forwardPoint);

  // If the furtherest forward point is not in the current lanelet
  size_t dyIndex = curIndex; // the furtherest lanelet
  // find out in which lanelet the furtherest point is
  if (curLength < 10) {
    // if (false) {
    while (curLength < 10) {
      curLength += lengthsLaneltes[++dyIndex];
    }

    // Operations on lanelets between the current lanelet and the furtherest lanelet:
    // push back the boundary line strings of the lanelets before the furtherest lanelet
    if (curIndex + 1 < dyIndex) {
      for (size_t index = curIndex + 1; index < dyIndex; ++index) {
        const auto midLeftBound = lanelet::utils::to2D(current_lanelets_[index].leftBound());
        const auto midRightBound = lanelet::utils::to2D(current_lanelets_[index].rightBound());
        // leftMarkerPoints.reserve(leftMarkerPoints.size() + midLeftBound.size());
        // rightMarkerPoints.reserve(rightMarkerPoints.size() + midRightBound.size());
        for (auto point : midLeftBound) {
          leftMarkerPoints.push_back(odd_tools::createPoint(point.basicPoint().x(), 
                                                            point.basicPoint().y(),
                                                            0));
        }
        for (auto point : midRightBound) {
          rightMarkerPoints.push_back(odd_tools::createPoint(point.basicPoint().x(), 
                                                            point.basicPoint().y(),
                                                            0));
        }
      }
    }


    // Operations on the futherest lanelet
    // get the furtherest point in the furtherest lanelet
    curLength -= lengthsLaneltes[dyIndex];
    forwardPoint = odd_tools::getFurtherestForwardPoint(current_lanelets_[dyIndex],
                                                        10,
                                                        curLength);
    const auto furLeftBound = lanelet::utils::to2D(current_lanelets_[dyIndex].leftBound());
    const auto furRightBound = lanelet::utils::to2D(current_lanelets_[dyIndex].rightBound());
    
    const auto furResampledLeft = odd_tools::resampleLine(furLeftBound, 0.5);
    const auto furResampledRight = odd_tools::resampleLine(furRightBound, 0.5);


    bgPoint leftStartPoint((furResampledLeft.begin() + 1)->position.x,
                           (furResampledLeft.begin() + 1)->position.y,
                           (furResampledLeft.begin() + 1)->position.z);
    bgPoint rightStartPoint((furResampledRight.begin() + 1)->position.x,
                           (furResampledRight.begin() + 1)->position.y,
                           (furResampledRight.begin() + 1)->position.z);

    const auto furLeftMarkerPoints = odd_tools::getMarkerPoints(furResampledLeft,
                                                                leftStartPoint,
                                                                forwardPoint);
    const auto furRightMarkerPoints = odd_tools::getMarkerPoints(furResampledRight,
                                                                rightStartPoint,
                                                                forwardPoint);
    if (furLeftMarkerPoints.size() > 0 && furRightMarkerPoints.size() > 0) {
      leftMarkerPoints.insert(leftMarkerPoints.end(),
                              furLeftMarkerPoints.begin(),
                              furLeftMarkerPoints.end());
      rightMarkerPoints.insert(rightMarkerPoints.end(),
                              furRightMarkerPoints.begin(),
                              furRightMarkerPoints.end());
    }
  }
  // ############### End of finding the furtherest forward point ###############
  // ###########################################################################

  // ##################################################################
  // ############### Find the furtherest backward point ###############
  double backLength = 0;
  auto backwardPoint = odd_tools::getFurtherestBackwardPoint(currentLanelet,
                                                           pose,
                                                           5,
                                                           backLength);

  auto backLeftMarkerPoints = odd_tools::getMarkerPoints(curResampledLeft,
                                                     backwardPoint,
                                                     egoPoint);
  auto backRightMarkerPoints = odd_tools::getMarkerPoints(curResampledRight,
                                                     backwardPoint,
                                                     egoPoint);

  // If the furtherest forward point is not in the current lanelet
  size_t backIndex = curIndex; // the furtherest lanelet
  // find out in which lanelet the furtherest point is
  if (backIndex > 0 && backLength < 5) {
    while (backIndex > 0 && backLength < 5) {
      backIndex--;
      backLength += lengthsLaneltes[backIndex];
    }


    // Operations on lanelets between the current lanelet and the furtherest lanelet:
    // push back the boundary line strings of the lanelets before the furtherest lanelet
    if (curIndex - 1 > backIndex) {
      for (size_t index = curIndex - 1; index > backIndex; --index) {
        const auto midLeftBound = lanelet::utils::to2D(current_lanelets_[index].leftBound());
        const auto midRightBound = lanelet::utils::to2D(current_lanelets_[index].rightBound());
        for (auto point : midLeftBound) {
          backLeftMarkerPoints.insert(backLeftMarkerPoints.begin(),
                                      odd_tools::createPoint(point.basicPoint().x(), 
                                                            point.basicPoint().y(),
                                                            0));
        }
        for (auto point : midRightBound) {
          backRightMarkerPoints.insert(backRightMarkerPoints.begin(),
                                      odd_tools::createPoint(point.basicPoint().x(), 
                                                            point.basicPoint().y(),
                                                            0));
        }
      }
    }


    // Operations on the futherest lanelet
    // get the furtherest point in the furtherest lanelet
    backLength -= lengthsLaneltes[backIndex];

    backwardPoint = odd_tools::getFurtherestBackwardPoint(current_lanelets_[backIndex],
                                                        5,
                                                        backLength);

    const auto backLeftBound = lanelet::utils::to2D(current_lanelets_[backIndex].leftBound());
    const auto backRightBound = lanelet::utils::to2D(current_lanelets_[backIndex].rightBound());
    
    const auto furBackResampledLeft = odd_tools::resampleLine(backLeftBound, 0.5);
    const auto furBackResampledRight = odd_tools::resampleLine(backRightBound, 0.5);

    bgPoint leftEndPoint((furBackResampledLeft.end() - 1)->position.x,
                           (furBackResampledLeft.end() - 1)->position.y,
                           0);
    bgPoint rightEndPoint((furBackResampledRight.end() - 1)->position.x,
                           (furBackResampledRight.end() - 1)->position.y,
                           0);

    std::vector<geometry_msgs::msg::Point> furBackLeftMarkerPoints;
    std::vector<geometry_msgs::msg::Point> furBackRightMarkerPoints;
    if (boost::geometry::distance(backwardPoint, leftEndPoint) > 0.1 &&
        boost::geometry::distance(backwardPoint, rightEndPoint) > 0.1) {

      furBackLeftMarkerPoints = odd_tools::getMarkerPoints (furBackResampledLeft,
                                                            backwardPoint,
                                                            leftEndPoint);
      furBackRightMarkerPoints = odd_tools::getMarkerPoints(furBackResampledRight,
                                                            backwardPoint,
                                                            rightEndPoint);

      }

    if (furBackLeftMarkerPoints.size() != 0 && furBackRightMarkerPoints.size() != 0) {
      backLeftMarkerPoints.insert(backLeftMarkerPoints.begin(),
                              furBackLeftMarkerPoints.begin(),
                              furBackLeftMarkerPoints.end());
      backRightMarkerPoints.insert(backRightMarkerPoints.begin(),
                              furBackRightMarkerPoints.begin(),
                              furBackRightMarkerPoints.end());
    }
  }
  // ############### End of finding the furtherest backward point ###############
  // ############################################################################

  if (backLeftMarkerPoints.size() > 0 &&
      leftMarkerPoints.size() > 0 &&
      rightMarkerPoints.size() > 0 &&
      backRightMarkerPoints.size() > 0)
  {
    centerLineMarker.points.insert(centerLineMarker.points.end(), 
                                    backLeftMarkerPoints.begin(), backLeftMarkerPoints.end());
    centerLineMarker.points.insert(centerLineMarker.points.end(), 
                                    leftMarkerPoints.begin(), leftMarkerPoints.end());
    centerLineMarker.points.insert(centerLineMarker.points.end(), 
                                    rightMarkerPoints.rbegin(), rightMarkerPoints.rend());
    centerLineMarker.points.insert(centerLineMarker.points.end(), 
                                    backRightMarkerPoints.rbegin(), backRightMarkerPoints.rend());
    centerLineMarker.points.insert(centerLineMarker.points.end(), backLeftMarkerPoints[0]);
  }
  else {
    std::cout << "Error when create the markers!"
              << "\nbackLeftMarkerPoints.size: " << backLeftMarkerPoints.size()
              << "\nforwardLeftMarkerPoints.size: " << leftMarkerPoints.size()
              << "\nforwardRightMarkerPoints.size: " << rightMarkerPoints.size()
              << "\nbackRightMarkerPoints: " << backRightMarkerPoints.size()
              << '\n';

  }
                                  
  msg.markers.push_back(centerLineMarker);
  return msg;
}

MarkerArray OddVisualizer::createAdjacentLaneBoundary(const std::vector<geometry_msgs::msg::Point> & points) {
  MarkerArray msg;
  Marker laneBoundaryMarker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "opposite_lanelets", 3l, Marker::LINE_STRIP,
    createMarkerScale(0.5, 0.5, 0.5), getColorRGBAmsg(odd_elements_->params.odd_rgba));
  laneBoundaryMarker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  laneBoundaryMarker.points.insert(laneBoundaryMarker.points.end(), points.begin(), points.end());
  msg.markers.push_back(laneBoundaryMarker);
  return msg;
}

void OddVisualizer::lanletToPolygonMsg(
        const lanelet::ConstLanelet & ll,
        geometry_msgs::msg::Polygon * polygon) {

    if (polygon == nullptr) {
        std::cerr << __FUNCTION__ << ": polygon is null pointer!" << std::endl;
        return;
    }

    lanelet::CompoundPolygon3d ll_poly = ll.polygon3d();
    polygon->points.clear();
    polygon->points.reserve(ll_poly.size());

    for(const auto & pt : ll_poly) {
        geometry_msgs::msg::Point32 pt32;
        lanelet::utils::conversion::toGeomMsgPt32(pt.basicPoint(), &pt32);
        polygon->points.push_back(pt32);
    }
}

void OddVisualizer::getODDFromMap(const lanelet::ConstLanelets laneletSequence)
{
  for (auto lanelet : laneletSequence)
  {
    odd_tools::onNonDrivableLane(lanelet);
    odd_tools::onEdge(lanelet);
    odd_tools::onFixedRoadStructures(lanelet);
    odd_tools::onRegulations(lanelet);
  }
}

void OddVisualizer::onAdjacentLanelet(const lanelet::ConstLanelet currentLanelet)
{
  if (routing_graph_ptr_->adjacentRight(currentLanelet)) std::cout << "adjacentleft\n" ;
  if (routing_graph_ptr_->adjacentLeft(currentLanelet)) std::cout << "adjacenright\n" ;
  if (routing_graph_ptr_->left(currentLanelet)) std::cout << "left\n" ;
  if (routing_graph_ptr_->right(currentLanelet)) std::cout << "right\n" ;
  const auto & sameLeft = odd_tools::getLeftLanelet(currentLanelet, routing_graph_ptr_);
  const auto & sameRight = odd_tools::getRightLanelet(currentLanelet, routing_graph_ptr_);

  const auto & leftOpp = odd_tools::getLeftOppositeLanelets(currentLanelet, lanelet_map_ptr_);
  const auto & rightOpp = odd_tools::getRightOppositeLanelets(currentLanelet, lanelet_map_ptr_);

  if (!rightOpp.empty()) {
    std::cout <<"size of rightOpp: " << rightOpp.size() <<'\n';
    const auto rightOppoLine = odd_tools::getLaneMarkerPointsFromLanelets(rightOpp);
    std::cout << "size of the line to be passed to the msg: " <<rightOppoLine.size() << '\n';
    int i = 0;
    for (auto index : rightOppoLine) {
      std::cout << "\nx" << i <<" = "<< index.x << "\n"
                << "y" << i <<" = "<< index.y << "\n";
                i++;

    }
    odd_adjacent_lane_publisher_->publish(createAdjacentLaneBoundary(rightOppoLine));
  }
}
} // namespace odd_visualizer




void printUsage() {
    std::cout   << "Usage: \nros2 run odd_visualizer odd_visualizer_node"
                << "--ros--args -p map_file:=<path to osm file>"
                << std::endl;
}