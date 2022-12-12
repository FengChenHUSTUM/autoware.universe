# ODD Visualizer
This repo aims at visualizing the ODD parameter based on Autoware.Universe at runtime.

## Subscriber
HDMAP or Vector map

## Publisher
This structure is based on the [Attribute.h](../../../../../opt/ros/galactic/include/lanelet2_core/Attribute.h#L336) defined in lanelet2_core rather than ODD definition from British.
```
ODDPparameter
|
└───Scenery
|   |
|   └───drivableAreas
|       |
|       └───LaneType
|           |   roadLane
|           |   walkwayLane
|           |   crosswalkLane
|           |   busLane
|           |   bicycleLane
|           |   emergencyLane
|           |   ...
|       |
|       └───RegulatpryElements
|           |   trafficLights
|           |   ...
|       |
|       └───Edge
|           |   roadShoulder
|           |   lineMarker
|           |   solidBarrier
|           |   ...
```
## Modification of packages in Autoware
1. set "use_takeover_request" to true
    > file location: [emergency_handler.param.yaml](../system/emergency_handler/config/emergency_handler.param.yaml#L8) 
    > **Note**: In case .yaml file doesn't work, modify the corresponding ros parameter directly [here](../system/emergency_handler/src/emergency_handler/emergency_handler_core.cpp#L27)
2. 

## Build the package alone
```shell
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to odd_visualizer_node
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select scenery_msgs odd_rviz_plugin  odd_visualizer_node

```