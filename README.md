# metrics_msgs
A collection of useful ROS interfaces for measuring ::hand-wave:: things

No relation to the [ros_metrics](https://metrics.ros.org/) project.

## metro_benchmark_msgs

Contains one message at the moment, [ComputeTime](metro_benchmark_msgs/msg/ComputeTime.msg).

```
std_msgs/Header header
builtin_interfaces/Duration duration

string id         # optional
string parent_id  # optional
```
See [actual definition](metro_benchmark_msgs/msg/ComputeTime.msg) for further description of the fields.

## metro_benchmark_pub
Contains a helper class in [Python](metro_benchmark_pub/metro_benchmark_pub/__init__.py) and [C++](metro_benchmark_pub/include/metro_benchmark_pub/benchmark_publisher.hpp) for publishing `ComputeTime` messages, including support for nested computation.

## collision_log_msgs
Contains the [NamedCollisions](collision_log_msgs/msg/NamedCollisions.msg) message definition for tracking collisions between objects.

```
std_msgs/Header header
collision_log_msgs/NamedCollisions[] collisions
    string entity0
    string entity1
```

It is a simplified version of [gazebo_msgs/ContactsState.msg](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/5e718169353e2c21f85e15fd4b743011b3ad9b57/gazebo_msgs/msg/ContactsState.msg) but has the following features.
 * Not Gazebo specific
 * Can be used in non-simulation contexts
 * Cleaner memory footprint (aka fewer fields)
