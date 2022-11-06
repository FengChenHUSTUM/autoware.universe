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
    odd_driveable_area_publisher_ = 
        create_publisher<MarkerArray>("/odd_parameter/odd_drivable_area", 1);


    map_subscriber_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "/map/vector_map", 10,
    std::bind(&OddVisualizer::mapCallback, this, std::placeholders::_1));

    lanelet_sequence_subscriber_ = this->create_subscription<laneSequenceWithID>(
        "/planning/scenario_planning/lane_driving/behavior_planning/odd_visualizer/lanelet_sequence_IDs", 1,
    std::bind(&OddVisualizer::laneletSequenceCallback, this, std::placeholders::_1));

    // odd_roadLane_publisher_ = 
    // create_publisher<odd_visualizer::LaneData>(
    //     "odd_parameter/scenery/drivableAreas/roadLane", rclcpp::QoS{1}.transient_local());
}
void OddVisualizer::run() {
    
}

// void OddVisualizer::getCurrentLane() {
//     // odd_elements_->self_pose = self_pose_listener_.getCurrentPose();
    
// }

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
    // shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(all_lanelets);
}

void OddVisualizer::laneletSequenceCallback(laneSequenceWithID::ConstSharedPtr msg) {
    // std::cout <<"odd lanelet sequence call back "<<'\n';

    std::vector<lanelet::ConstLanelet> laneletSequence(msg->lane_ids.size());
    (*current_lanelet_) = lanelet_map_ptr_->laneletLayer.get(msg->current_lane);
    int i = 0;
    for (const auto & laneID : msg->lane_ids) {
        laneletSequence[i++] = lanelet_map_ptr_->laneletLayer.get(laneID);
    }
    current_lanelets_ = laneletSequence;
    odd_elements_->self_pose = self_pose_listener_.getCurrentPose();
    odd_driveable_area_publisher_->publish(createCenterlineInterest());

    // const auto pose = odd_elements_->self_pose->pose;
    // creareDrivableBoundaryMarkerArray(current_lanelets_);
    // getODDFromMap(laneletSequence);
    // onAdjacentLanelet(*current_lanelet_);
    // odd_tools::getRightBoundaryLineString(laneletSequence, pose, 200, 20);
}
void OddVisualizer::creareDrivableBoundaryMarkerArray(
    const lanelet::ConstLanelets laneletSequence) {
    
    //TODO: find lane adjacent to the current lane sequence(both sides)
    // with using the route graph provided in Lanelet2

    lanelet::ConstLineStrings3d linestring_shared;
    // getFurthestLinestring(https://github.com/autowarefoundation/autoware.universe/blob/8e92d176721e71943d7dff0722365cb9ee97379f/planning/route_handler/src/route_handler.cpp#L1029) 
    for (const auto & lanelet : laneletSequence) {
        lanelet::ConstLineStrings3d linestrings;
        linestrings.reserve(2);
        linestrings.emplace_back(odd_tools::getRightMostLinestring(
                                 lanelet,
                                 routing_graph_ptr_,
                                 lanelet_map_ptr_)); // right most line string
        linestrings.emplace_back(odd_tools::getLeftMostLinestring(
                                 lanelet,
                                 routing_graph_ptr_,
                                 lanelet_map_ptr_)); // left most line string
        linestring_shared.insert(linestring_shared.end(), 
                                 linestrings.begin(), 
                                 linestrings.end());
    } 
    const auto drivable_area_lines = createDrivableAreaMarkerArray(linestring_shared);

    // odd_tools::BoundaryInfo boundaryInfo_ = odd_tools::getBoundaryLineString(laneletSequence,
    //                                                 (*current_lanelet_),
    //                                                 odd_elements_->self_pose->pose,
    //                                                 odd_elements_->params.forward_path_length,
    //                                                 odd_elements_->params.backward_path_length);
    lanelet::ConstLineStrings3d lineStringBoundary;
    // std::cout << "sequence size: " << laneletSequence.size() << "; laneletIdx: " << boundaryInfo_.furtherestLaneletId
    //           << "; pointId: " << boundaryInfo_.furtherestPointId << '\n'
    //           << "forwardLength: " << odd_elements_->params.forward_path_length<<'\n'
    //           << "backwardLength: " << odd_elements_->params.backward_path_length<<'\n';
    for (auto curLanelet = laneletSequence.begin();
        curLanelet != laneletSequence.end();
        // curLanelet != laneletSequence.begin() + boundaryInfo_.laneletIdx;
        ++curLanelet) {
      lanelet::ConstLineStrings3d linestrings;
      linestrings.reserve(2);
      linestrings.emplace_back(curLanelet->rightBound());
      linestrings.emplace_back(curLanelet->leftBound());
      lineStringBoundary.insert(lineStringBoundary.end(),
                                linestrings.begin(),
                                linestrings.end());
    }
    
    
    // const auto drivableBoundaryMSG = createDrivableAreaMarkerArray(lineStringBoundary);
    // const auto drivableBoundaryMSG = createDrivableAreaMarkerArray(laneletSequence, boundaryInfo_);
    // odd_driveable_area_publisher_->publish(drivable_area_lines);
    
    // odd_driveable_area_publisher_->publish(createCenterlineInterest());
}

MarkerArray OddVisualizer::createCenterlineInterest() {
  MarkerArray msg;
  Marker centerLineMarker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "shared_linestring_lanelets", 0l, Marker::LINE_STRIP,
    createMarkerScale(1.0, 1.0, 1.0), getColorRGBAmsg(odd_elements_->params.odd_rgba));
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
  // get all the arclength of each lanelet in the sequence
  std::vector<double> lengthsLaneltes;
  lengthsLaneltes.reserve(current_lanelets_.size());
  for (size_t i = 0; i < current_lanelets_.size(); ++i) {
    if (current_lanelets_[i] == currentLanelet) curIndex = i;
    const auto curllCenterLine = lanelet::utils::to2D(current_lanelets_[i].centerline());
    lengthsLaneltes.push_back(odd_tools::getArcLengthFromPoints(curllCenterLine));
  }
  // ############### Find the furtherest forward point ###############
  auto forwardPoint = odd_tools::getFurtherestForwardPoint(currentLanelet,
                                                           pose,
                                                           10,
                                                           curLength);
  // If the furtherest forward point is not in the current lanelet
  size_t dyIndex = curIndex;
  if (curLength < 10) {
    while (curLength + lengthsLaneltes[dyIndex] < 10) {
      curLength += lengthsLaneltes[dyIndex++];
    }
    forwardPoint = odd_tools::getFurtherestForwardPoint(current_lanelets_[dyIndex],
                                                        10,
                                                        curLength);
  }
  // if the furtherest forward point is in the current lanelet
  else {
    const auto leftBound = lanelet::utils::to2D(current_lanelets_[curIndex].leftBound());
    const auto rightBound = lanelet::utils::to2D(current_lanelets_[curIndex].rightBound());
    
    const auto resampledLeft = odd_tools::resampleLine(leftBound, 1.0);
    const auto resampledRight = odd_tools::resampleLine(rightBound, 1.0);
    
    const auto leftMarkerPoints = odd_tools::getMarkerPoints(resampledLeft,
                                                             egoPoint,
                                                             forwardPoint);
    const auto rightMarkerPoints = odd_tools::getMarkerPoints(resampledRight,
                                                              egoPoint,
                                                              forwardPoint);
    centerLineMarker.points.insert(centerLineMarker.points.end(), 
                                   leftMarkerPoints.begin(), leftMarkerPoints.end());
    centerLineMarker.points.insert(centerLineMarker.points.end(), 
                                   rightMarkerPoints.rbegin(), rightMarkerPoints.rend());

  }
  
  msg.markers.push_back(centerLineMarker);
  return msg;


}

MarkerArray OddVisualizer::createDrivableAreaMarkerArray(const lanelet::ConstLineStrings3d & linestrings) {
  if (linestrings.empty()) {
    return MarkerArray();
  }
  // std_msgs::msg::ColorRGBA colorconfig;
  // colorconfig.r = 34;
  // colorconfig.g = 114;
  // colorconfig.b = 227;
  // colorconfig.a = 0.9;
  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "shared_linestring_lanelets", 0L, Marker::LINE_STRIP,
    createMarkerScale(0.3, 0.0, 0.0), getColorRGBAmsg(odd_elements_->params.odd_rgba));
    // createMarkerScale(0.3, 0.0, 0.0), colorconfig);
  marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);

  const auto reserve_size = linestrings.size() / 2;
  lanelet::ConstLineStrings3d lefts;
  lanelet::ConstLineStrings3d rights;
  lefts.reserve(reserve_size);
  rights.reserve(reserve_size);

  size_t total_marker_reserve_size{0};
  for (size_t idx = 1; idx < linestrings.size(); idx += 2) {
    rights.emplace_back(linestrings.at(idx - 1));
    lefts.emplace_back(linestrings.at(idx));

    for (const auto & ls : linestrings.at(idx - 1).basicLineString()) {
      total_marker_reserve_size += ls.size();
    }
    for (const auto & ls : linestrings.at(idx).basicLineString()) {
      total_marker_reserve_size += ls.size();
    }
  }

  if (!total_marker_reserve_size) {
    marker.points.reserve(total_marker_reserve_size);
  }

  const auto & first_ls = lefts.front().basicLineString();
  for (const auto & ls : first_ls) {
    marker.points.push_back(createPoint(ls.x(), ls.y(), ls.z()));
  }

  for (auto idx = lefts.cbegin() + 1; idx != lefts.cend(); ++idx) {
    Point front = createPoint(
      idx->basicLineString().front().x(), idx->basicLineString().front().y(),
      idx->basicLineString().front().z());
    Point front_inverted = createPoint(
      idx->invert().basicLineString().front().x(), idx->invert().basicLineString().front().y(),
      idx->invert().basicLineString().front().z());

    const auto & marker_back = marker.points.back();
    const bool isFrontNear = tier4_autoware_utils::calcDistance2d(marker_back, front) <
                             tier4_autoware_utils::calcDistance2d(marker_back, front_inverted);
    const auto & left_ls = (isFrontNear) ? idx->basicLineString() : idx->invert().basicLineString();
    for (auto ls = left_ls.cbegin(); ls != left_ls.cend(); ++ls) {
      marker.points.push_back(createPoint(ls->x(), ls->y(), ls->z()));
    }
  }

  for (auto idx = rights.crbegin(); idx != rights.crend(); ++idx) {
    Point front = createPoint(
      idx->basicLineString().front().x(), idx->basicLineString().front().y(),
      idx->basicLineString().front().z());
    Point front_inverted = createPoint(
      idx->invert().basicLineString().front().x(), idx->invert().basicLineString().front().y(),
      idx->invert().basicLineString().front().z());

    const auto & marker_back = marker.points.back();
    const bool isFrontFurther = tier4_autoware_utils::calcDistance2d(marker_back, front) >
                                tier4_autoware_utils::calcDistance2d(marker_back, front_inverted);
    const auto & right_ls =
      (isFrontFurther) ? idx->basicLineString() : idx->invert().basicLineString();
    for (auto ls = right_ls.crbegin(); ls != right_ls.crend(); ++ls) {
      marker.points.push_back(createPoint(ls->x(), ls->y(), ls->z()));
    }
  }

  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }

  MarkerArray msg;

  msg.markers.push_back(marker);
  return msg;
}
MarkerArray OddVisualizer::createDrivableAreaMarkerArray(const lanelet::ConstLanelets laneletSequence,
                                                          const odd_tools::BoundaryInfo boundInfo) {
  MarkerArray msg;
  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "shared_linestring_lanelets", 0L, Marker::LINE_STRIP,
    createMarkerScale(0.3, 0.0, 0.0), getColorRGBAmsg(odd_elements_->params.odd_rgba));
  marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);

  Marker furPoint = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "shared_linestring_lanelets", 99, Marker::SPHERE,
    createMarkerScale(1.0, 1.0, 1.0), getColorRGBAmsg(odd_elements_->params.odd_rgba));

    furPoint.pose.position.x = laneletSequence[boundInfo.furtherestLaneletId].centerline()[boundInfo.furtherestPointId].x();
    furPoint.pose.position.y = laneletSequence[boundInfo.furtherestLaneletId].centerline()[boundInfo.furtherestPointId].y();
    furPoint.pose.position.z = laneletSequence[boundInfo.furtherestLaneletId].centerline()[boundInfo.furtherestPointId].z();
    furPoint.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);

    msg.markers.push_back(furPoint);

//TODO:
// 1. get left boundString(composed of points) from pose to forward futherest
  // size_t reserveSizeLeft = 0;
  // for (auto laneletIDx : laneletSequence)
  // {
  //   reserveSizeLeft += laneletIDx.leftBound2d().size();
  // }
  // std::vector<lanelet::Point2d> leftBound(reserveSizeLeft);
  int curLaneletPosition = odd_tools::getCurrentLaneletPosition(laneletSequence, odd_elements_->self_pose->pose, *current_lanelet_);

  for (size_t i = boundInfo.posePointIDLanelet; i < laneletSequence[curLaneletPosition].leftBound().size(); ++i)
  {
    marker.points.push_back(createPoint(laneletSequence[curLaneletPosition].leftBound()[i].x(),
                                        laneletSequence[curLaneletPosition].leftBound()[i].y(), 
                                        laneletSequence[curLaneletPosition].leftBound()[i].z()));
  }
  if (curLaneletPosition == (int)(boundInfo.furtherestLaneletId)) 
  {
    msg.markers.push_back(marker);
    return msg;
  }
  for (size_t ll = curLaneletPosition + 1; ll <= boundInfo.furtherestLaneletId; ++ll)
  {
    if (ll != boundInfo.furtherestLaneletId) 
    {
      for (size_t i = 0; i < laneletSequence[ll].leftBound().size(); ++i)
      {
        marker.points.push_back(createPoint(laneletSequence[ll].leftBound()[i].x(),
                                            laneletSequence[ll].leftBound()[i].y(), 
                                            laneletSequence[ll].leftBound()[i].z()));
      }
    }
    for (size_t i = 0; i < boundInfo.furtherestPointId; ++i)
    {
      marker.points.push_back(createPoint(laneletSequence[ll].leftBound()[i].x(),
                                          laneletSequence[ll].leftBound()[i].y(), 
                                          laneletSequence[ll].leftBound()[i].z()));
      msg.markers.push_back(marker);
      std::cout << "\n##### Analyze #####\n"
                << "The "<< curLaneletPosition<<" in the sequence( "<<laneletSequence.size()<<")\n"
                << "futherest point id: "<<boundInfo.furtherestPointId<<"\n"
                << "futherest lanelet id: "<<boundInfo.furtherestLaneletId<<"\n"
                << "##### END #####\n";

      return msg;
    }
  }
  return msg;

// 2. get right from pose to forward furtherest
// 3. invert right
// 4. assign left and right to marker array
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
  const auto rightLanelet = routing_graph_ptr_->right(currentLanelet);
  const auto leftLanelets = routing_graph_ptr_->leftRelations(currentLanelet);
  const auto rightLanelets = routing_graph_ptr_->rightRelations(currentLanelet);
  //TODO: get Left Opposite Lanelets 
  std::cout << "##### left relationed lanelet #####\n";

  for (auto index : leftLanelets) 
  {
    switch (index.relationType)
    {
    case lanelet::routing::RelationType::Successor: std::cout << "Successor"<<'\n'; break;
    case lanelet::routing::RelationType::Left: std::cout << "Left"<<'\n'; break;
    case lanelet::routing::RelationType::Right: std::cout << "Right"<<'\n'; break;
    case lanelet::routing::RelationType::AdjacentLeft: std::cout << "AdjacentLeft"<<'\n'; break;
    case lanelet::routing::RelationType::AdjacentRight: std::cout << "AdjacentRight"<<'\n'; break;
    case lanelet::routing::RelationType::Conflicting: std::cout << "Conflicting"<<'\n'; break;
    case lanelet::routing::RelationType::Area: std::cout << "Area"<<'\n'; break;
    case lanelet::routing::RelationType::None: std::cout << "None"<<'\n'; break;
    default:
      break;
    }
  }
  std::cout << "##### End #####\n";

  std::cout << "##### right relationed lanelet #####\n";
  for (auto index : rightLanelets) 
  {
    switch (index.relationType)
    {
    case lanelet::routing::RelationType::Successor: std::cout << "Successor"<<'\n'; break;
    case lanelet::routing::RelationType::Left: std::cout << "Left"<<'\n'; break;
    case lanelet::routing::RelationType::Right: std::cout << "Right"<<'\n'; break;
    case lanelet::routing::RelationType::AdjacentLeft: std::cout << "AdjacentLeft"<<'\n'; break;
    case lanelet::routing::RelationType::AdjacentRight: std::cout << "AdjacentRight"<<'\n'; break;
    case lanelet::routing::RelationType::Conflicting: std::cout << "Conflicting"<<'\n'; break;
    case lanelet::routing::RelationType::Area: std::cout << "Area"<<'\n'; break;
    case lanelet::routing::RelationType::None: std::cout << "None"<<'\n'; break;
    default:
      break;
    }
  }
  std::cout << "##### End #####\n";

  if (rightLanelet) {
    std::cout << "##### right adjacent lanelet #####\n";
    odd_tools::onNonDrivableLane(rightLanelet.value());
    odd_tools::onEdge(rightLanelet.value());
    odd_tools::onFixedRoadStructures(rightLanelet.value());
    odd_tools::onRegulations(rightLanelet.value());
    std::cout << "##### End #####\n";
  }

}

// scenery_msgs::msg::LaneData OddVisualizer::createRoadLaneMsg() {
//     scenery_msgs::msg::LaneData laneData;
//     road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets_);
//     // TODO define what format of the lane data should be applied here
//     for(const auto& sgll: road_lanelets_) {
//         scenery_msgs::msg::LanePrimitive lanePrimitive;
//         lanePrimitive.laneID = sgll.id();
//         // lanePrimitive.laneType = sgll.attributes().find("subtype");
//         // lanePrimitive.laneType = scenery_msgs::msg::
//         // if(sgll.hasAttribute(lanelet::AttributeName::OneWay))
//         // Note: driving direction is determined when initialize, default to be one-way,
//         // the direction of the lane can be visualized using arrow markers
//         // https://github.com/autowarefoundation/autoware_common/blob/main/tmp/lanelet2_extension/lib/visualization.cpp#L1222
//         //lanePrimitive.laneDirection = sgll.attribute(lanelet::AttributeValueString::Lanelet)
//     }
//     laneData.dataBase.clear();
//     // TODO: define the data type of ODD params to be published
//     //laneData.dataBase.assign(roadLaneString.begin(), roadLaneString.end());
//     return laneData;
// }

// scenery_msgs::msg::LaneData OddVisualizer::createWalkWayLaneMsg() {
//     scenery_msgs::msg::LaneData laneData;
//     walkway_lanelets_ = lanelet::utils::query::walkwayLanelets(all_lanelets_);
//     laneData.dataBase.clear();

//     return laneData;
// }

// scenery_msgs::msg::LaneData OddVisualizer::createCrossWalkLaneMsg() {
//     scenery_msgs::msg::LaneData laneData;
//     crosswalk_lanelets_ = lanelet::utils::query::crosswalkLanelets(all_lanelets_);
//     laneData.dataBase.clear();

//     return laneData;
// }

// scenery_msgs::msg::LaneData OddVisualizer::createBUSLaneMsg() {
//     scenery_msgs::msg::LaneData laneData;
//     // TODO: add query function for bus lane
//     // bus_lanelets_ = lanelet::utils::query::busLanelets(all_lanelets_);
//     laneData.dataBase.clear();

//     return laneData;
// }

// scenery_msgs::msg::LaneData OddVisualizer::createBicycleLaneMsg() {
//     scenery_msgs::msg::LaneData laneData;
//     // TODO: add query function for bicycle lane
//     // bicycle_lanelets_ = lanelet::utils::query::bicycleLanelets(all_lanelets_);
//     laneData.dataBase.clear();

//     return laneData;
// }

// scenery_msgs::msg::LaneData OddVisualizer::createEmergencyLaneMsg() {
//     scenery_msgs::msg::LaneData laneData;
//     // TODO: add query function for emergency lane
//     // emergency_lanelets_ = lanelet::utils::query::emergencyLanelets(all_lanelets_);
//     laneData.dataBase.clear();

//     return laneData;
// }

} // namespace odd_visualizer




void printUsage() {
    std::cout   << "Usage: \nros2 run odd_visualizer odd_visualizer_node"
                << "--ros--args -p map_file:=<path to osm file>"
                << std::endl;
}