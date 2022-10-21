# Summary of related autoware pkgs

## Visualization
#### 1. Visualization of the map
corresponding packages or files:
> autoware/src/core/autoware_common/tmp/lanelet2_extension

three kinds of lanelet elements are conerted here:

- lanelet::Lanelet to **Triangle** Markers
- lanelet::LineString to **LineStrip** Markers
- TrafficLights to **Triangle** Markers
    
Each marker is defined in one [function](https://github.com/autowarefoundation/autoware_common/blob/ac39ea4524e983d9d422815996423707482d8494/tmp/lanelet2_extension/lib/visualization.cpp#L148) and inserted to the end of  [*map_marker_array*](https://github.com/autowarefoundation/autoware.universe/blob/eda487d39df7df764c39ed4a962d29446a90353e/map/map_loader/src/lanelet2_map_loader/lanelet2_map_visualization_node.cpp#L147) when the map is loaded. The way how the code is implemented  can also be applied in the implementation of odd visualization.  More details can be found [here](https://github.com/autowarefoundation/autoware_common/blob/88d942752ca9f68d4f51120d1dba59648fef24ef/tmp/lanelet2_extension/README.md).



#### 2. Visualization of the "ODD parameters"
run the visualizer in rviz
```shell
source ~/fengAutoware/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```
corresponding packages or files:
> autoware/src/universe/autoware.universe/planning/behavior_path_planner/src/utilities.cpp & debug_utilities.cpp


Some of the visualizations are implemented for [debugging](https://github.com/autowarefoundation/autoware.universe/blob/4e689036ef13df5f1bd784dc098bbf5b19c074c1/planning/behavior_path_planner/src/debug_utilities.cpp) in autoware. The following example shows the utility of it.
> drivable area boundary:
> 1. How to get the pose: [tier4_autoware_utils::SelfPoseListener](https://github.com/FengChenHUSTUM/autoware.universe/blob/342a959f9a323dac8b31898d53f0a9d0bd33c6b2/planning/behavior_path_planner/include/behavior_path_planner/behavior_path_planner_node.hpp#L104)
> 2. [The drivable are boundary](https://github.com/autowarefoundation/autoware.universe/blob/342a959f9a323dac8b31898d53f0a9d0bd33c6b2/planning/behavior_path_planner/src/behavior_path_planner_node.cpp#L72) is defined as a ros topic and its type is "MarkerArray". The implementation is [here](https://github.com/autowarefoundation/autoware.universe/blob/342a959f9a323dac8b31898d53f0a9d0bd33c6b2/planning/behavior_path_planner/src/behavior_path_planner_node.cpp#L626-L633).
> 3. lanelet_sequence_forward: [implementation](https://github.com/autowarefoundation/autoware.universe/blob/8e92d176721e71943d7dff0722365cb9ee97379f/planning/route_handler/src/route_handler.cpp#L422-L443) and its [definition](https://github.com/FengChenHUSTUM/autoware.universe/blob/5e0962c72677256273032a3c9f729a2f7cb4df9b/launch/tier4_planning_launch/config/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/behavior_path_planner.param.yaml#L4) in the yaml file
> **Note**: we can use this "Marker" to find the lanes in interest and change the forward area. Then visualize the ODD params related to these lanes.

Based on what is going to be visualized, a seperated window or a panel could be necessary for the visualization. An existing panel in autoware could be regarded as a [guideline](https://github.com/FengChenHUSTUM/autoware.universe/blob/c2096638a3a076326fa8748c97d7f6364655cf70/common/tier4_state_rviz_plugin/src/autoware_state_panel.hpp). There is also official [tutorial](http://docs.ros.org/en/melodic/api/rviz_plugin_tutorials/html/panel_plugin_tutorial.html) on it.


## Message definitions
corresponding packages or files:
> src/core/external/autoware_auto_msgs

All the messages and services files are stored in .idl files (*Interface Definition Language (IDL)*), which aims at "*[achieving the CORBA goal of interoperability between different languages and platforms*](https://mhanckow.students.wmi.amu.edu.pl/corba/IDL.html#281057)".

The data structure of the [map](https://github.com/tier4/autoware_auto_msgs/tree/83297dcb105b95be0d4a2dc1c4af0f7aaffcaeb3/autoware_auto_mapping_msgs/msg) message is as following:

> Compact Message Definition of *MapPrimitive*
> ```cpp
> int64 id
> string primitive_type
> ```
> Compact Message Definition of *HADMapSegment*
> ```cpp
> sequence<autoware_auto_mapping_msgs::msg::MapPrimitive> primitives
> int64 preferred_primitive_id
> ```
> Compact Message Definition of *HADMapBin*
> ```cpp
> std_msgs::msg::Header header
> uint8 map_format
> string format_version
> string map_version
> sequence < uint8 > data
> ```

There is still a problem not solved yet, the constants defined in idl file cannot be called in cpp files. An alternative way for this problem may be defining these constants manually.



## Behavior path planner
corresponding packages or files:
> autoware/src/universe/autoware.universe/planning/behavior_path_planner

This package is responsible to generate

- **path** based on the traffic situation,
- **drivable area** that the vehicle can move (defined in the path msg),
- **turn signal** command to be sent to the vehicle - interface.

The basic algorithms for **drivable area generation** is explained [here](https://github.com/autowarefoundation/autoware.universe/tree/9ff4416be99fcac540370f04b05d1913da4bbf12/planning/behavior_path_planner).

> The drivable area generation flow: 
[generateDrivableArea(...)](https://github.com/autowarefoundation/autoware.universe/blob/490cb4029936b3e1ba57531082c58ff88fb95183/planning/behavior_path_planner/src/utilities.cpp#L1074-L1209) 
-> [getPathScope()](https://github.com/autowarefoundation/autoware.universe/blob/490cb4029936b3e1ba57531082c58ff88fb95183/planning/behavior_path_planner/src/utilities.cpp#L283) 
-> [getNearestLaneId()](https://github.com/autowarefoundation/autoware.universe/blob/490cb4029936b3e1ba57531082c58ff88fb95183/planning/behavior_path_planner/src/utilities.cpp#L159) 
-> [getClosestLanelet()](https://github.com/autowarefoundation/autoware_common/blob/2fa6751cd505dae81b28aaeea9e31499d35d194c/tmp/lanelet2_extension/lib/query.cpp#L713)(first search by distance then search by angle in the result)
-> calculate lane **boundary** coordinates 
-> add lanes covers **initial and goal footprints**
(the drivabale area is basiclly generated from the planned path)
-> convert polygon to opencv type
-> create occupancygrid with opencv->convert opencv image to occupancygrid

**Six** different behaviors are implemented as separated modules in this package: Lane Following, Lane Change, Obstacle Avoidance, Pull Over, Pull Out and Side Shift.

A Design Tree is applied to manage which behavior should be applied in corresponding situations.