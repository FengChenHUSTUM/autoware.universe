# ODD Visualizer
This repo aims at visualizing the ODD parameter based on Autoware.Universe at runtime.

# Subscriber
HDMAP or Vector map

# Publisher
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

# Build the package alone
```shell
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to odd_visualizer_node
```