# Interface for recording the amount of time some compute function took
# In the header, the frame_id should be the name of the function/task i.e. ComputePathToPose
#                and the stamp should be the *completion* time
std_msgs/Header header
# The duration should be populated with the total amount of time the function took.
# This implies that the function started at header.stamp - duration
builtin_interfaces/Duration duration

# The id (optional) is for providing a unique identifier for the instance of the function running
# e.g. ComputePathToPose0
string id

# The parent id (optional) allows you to create nested benchmarked functions
string parent_id

# Example: Assume running a ComputePathToPose action has two parts: updating the costmap, and running A*.
#   Here is a plausible sequence of messages.
#     {header: {stamp: {nanosec: 4e8}, frame_id: costmap_update}, duration: {nanosec: 2e8}, parent_id: global_plan0}
#     {header: {stamp: {nanosec: 5e8}, frame_id: astar         }, duration: {nanosec: 1e8}, parent_id: global_plan0}
#     {header: {stamp: {nanosec: 6e8}, frame_id: global_plan   }, duration: {nanosec: 5e8}, id: global_plan0}
#
#   Here is the same data in timeline form
#   01234567 Time (x1e8)
#    o----o  global_plan0
#     o-o    costmap_update
#       oo   global_plan
#
#   With the nested structure, we can deduce that global_plan0 took 5e8 overall, with 3e8 for the nested functions
#   and 2e8 in the global_plan call itself.
