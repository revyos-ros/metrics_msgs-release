# metrics_msgs
A collection of useful ROS interfaces for measuring ::hand-wave:: things

No relation to the [ros_metrics](https://metrics.ros.org/) project.

## benchmark_msgs

Contains one message at the moment, [ComputeTime](benchmark_msgs/msg/ComputeTime.msg).

```
std_msgs/Header header
builtin_interfaces/Duration duration

string id         # optional
string parent_id  # optional
```
See [actual definition](benchmark_msgs/msg/ComputeTime.msg) for further description of the fields.

## benchmark_utils
Contains a helper class in [Python](benchmark_utils/benchmark_utils/__init__.py) and [C++](benchmark_utils/include/benchmark_utils/benchmark_publisher.hpp) for publishing `ComputeTime` messages, including support for nested computation.

## collision_msgs
Contains the [Collisions](collision_msgs/msg/Collisions.msg) message definition for tracking collisions between objects.

```
std_msgs/Header header
collision_msgs/Collision[] collisions
    string entity0
    string entity1
```

It is a simplified version of [gazebo_msgs/ContactsState.msg](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/5e718169353e2c21f85e15fd4b743011b3ad9b57/gazebo_msgs/msg/ContactsState.msg) but has the following features.
 * Not Gazebo specific
 * Can be used in non-simulation contexts
 * Cleaner memory footprint (aka fewer fields)
