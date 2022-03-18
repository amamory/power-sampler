# power-sampler
A simple application to sample some power data for embedded platforms

## workflow

http://trac.gateworks.com/wiki/Yocto/packages#PackageDevelopmentWorkflow

### in the host 
petalinux-build
petalinux-build -c package-index

### in the target

dnf clean packages
dnf remove energy-sensors
dnf install energy-sensors


## INA readings

https://github.com/jparkerh/xilinx-linux-power-utility/blob/master/src/ina_bm.cc

## cross-compile ROS2

https://embeddeduse.com/2017/06/03/cmake-cross-compilation-based-on-yocto-sdk/

https://docs.ros.org/en/foxy/How-To-Guides/Cross-compilation.html   <== official 
https://github.com/cyberbotics/epuck_ros2/tree/master/installation/cross_compile
https://answers.ros.org/question/382582/what-is-the-correct-way-of-installing-ros2-foxy-on-a-raspberry-pi-zero/
https://github.com/irobot-ros/ros2-cross-compilation
https://github.com/ros2-for-arm/ros2/blob/master/aarch64_toolchainfile.cmake
https://github.com/process1183/roomba-rpi/blob/master/docs/ros2_rpizw_build.md toolchain.cmkae
https://hackmd.io/@st9540808/S1VpB3-cB?type=view
 
## initial ROS2-based version

reference node and its recipe

https://github.com/ros2/demos/blob/mas-ter/dummy_robot/dummy_sensors/CMakeLists.txt
https://github.com/ros/meta-ros/blob/master/meta-ros2-foxy/generated-recipes/demos/dummy-sensors_0.9.3-1.bb


https://github.com/ros2/example_interfaces
https://github.com/ros/meta-ros/tree/zeus/meta-ros2-foxy/generated-recipes/example-interfaces

## diagnostic based version

https://github.com/ros/meta-ros/blob/master/meta-ros2-foxy/generated-recipes/diagnostics/diagnostic-aggregator_2.0.8-2.bb
https://github.com/ros/meta-ros/blob/master/meta-ros2-foxy/generated-recipes/diagnostics/diagnostic-updater_2.0.8-2.bb
https://github.com/ros/meta-ros/blob/master/meta-ros2-foxy/generated-recipes/rqt-runtime-monitor/rqt-runtime-monitor_1.0.0-1.bb

rqt_robot_monitor displays diagnostics_agg topics messages that are published by diagnostic_aggregator.
https://github.com/ros-visualization/rqt_robot_monitor
https://github.com/ros/meta-ros/tree/master/meta-ros2-foxy/generated-recipes/rqt-robot-monitor

https://github.com/ros-visualization/rqt_top
https://github.com/ros/meta-ros/blob/master/meta-ros2-foxy/generated-recipes/rqt-top/rqt-top_1.0.2-1.bb

## plotjuggler

https://github.com/ros/meta-ros/blob/master/meta-ros2-foxy/generated-recipes/plotjuggler/plotjuggler_3.3.2-1.bb
https://github.com/ros/meta-ros/tree/master/meta-ros2-foxy/generated-recipes/plotjuggler-ros
https://github.com/ros/meta-ros/tree/master/meta-ros2-foxy/generated-recipes/plotjuggler-msgs

## realtime 

https://github.com/ros/meta-ros/tree/master/meta-ros2-foxy/generated-recipes/realtime-tools
https://github.com/ros/meta-ros/tree/master/meta-ros2-foxy/generated-recipes/ros2-tracing

https://github.com/ros2/demos/tree/master/topic_monitor

## DDS

https://github.com/ros/meta-ros/tree/master/meta-ros2-foxy/generated-recipes/rmw-cyclonedds
https://github.com/ros/meta-ros/tree/master/meta-ros2-foxy/generated-recipes/rmw-fastrtps

## micro-ros

https://github.com/ros/meta-ros/tree/master/meta-ros2-foxy/generated-recipes/micro-ros-diagnostics
https://github.com/ros/meta-ros/tree/master/meta-ros2-foxy/generated-recipes/micro-ros-msgs

## Yocto

https://wiki.yoctoproject.org/wiki/TipsAndTricks/Creating_Recipes_for_ROS_modules
