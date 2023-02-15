# hateb_planner_people_interface

This package contains executables that convert ROS standardized message types `perople_msgs/People` to messages interpretable by [`HATeb Planner`](https://github.com/harmishhk/hateb_local_planner).

The `perople_msgs/People` contain very basic data about a tracked person but `tagnames` and `tags` fields allow putting additional information in a `string` form. These fields were used to carry tracking and group data.

This package introduces 1 executable:
- `tracked_persons_to_hateb_node`.

There is a similar package for HATeb-derived planners at [`cohan_planner_people_interface`](https://github.com/rayvburn/cohan_planner_people_interface).
