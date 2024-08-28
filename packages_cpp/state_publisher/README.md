# state_publisher
Node to publish the static and dynamic transforms of the vehicle sensors using the robot_state_publisher package. For information about the used package refer to https://github.com/ros/robot_state_publisher/tree/humble.

## Configurations
The node configurations are defined in state_publisher/launch/state_publisher.py and the coordinate transformations are stored in state_publisher/urdf/urdf.xml.

## Run 
```
ros2 launch state_publisher state_publisher.py
```