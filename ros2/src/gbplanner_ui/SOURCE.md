# GBPlanner RViz panel

The six commands and PCI service contracts come from `gbplanner_ui/src/gbplanner_ui.cpp` at GBPlanner 2 commit `7301b5350915cf3a4055072ad559842db37b4ca9`. The upstream package.xml license field says `TODO`; the repository root carries the BSD-3-Clause license copied alongside this file. This ROS 2 widget is a new implementation of those controls, not an unchanged ROS 1 source build.

The panel uses rviz_common/pluginlib, private rclcpp clients, and nonblocking Qt polling. A pending request does not disable Stop. Service rejection is displayed separately from transport success; request acceptance is not completion of robot motion. The independently subscribed control_status shows the current PCI state. Namespaces and frontier selection are saved by RViz. The optional Ignore Time Budget box defaults to false; all other global request defaults are preserved. Waypoint planning retains the Trigger service, and the supplied RViz SetGoal tool publishes to the PCI's existing `/move_base_simple/goal` topic.

Launch after the planner: `ros2 launch gbplanner_ui control.launch.py`. Choose the robot namespace in the panel; `/` is the default. Real-time operation can pass `use_sim_time:=false`. The panel itself uses steady time for reply deadlines, so Stop remains available when simulation time is paused.
