MICVISION
===

# Description

This package provide a lot functions for robot operation both in real and
stage environment.

The main functions list:
1. Autonomous navigation
2. Autonomous mapping
3. Self-localization
4. Autonomous obstacle avoidance
5. Dynamic path planning

# Dependency

1. [navigation](https://github.com/tyuownu/navigation)
2. [cartographer](https://github.com/tyuownu/cartographer)
3. [cartographer_ros](https://github.com/tyuownu/cartographer_ros)
4. [cartographer_turtlebot](https://github.com/tyuownu/cartographer_turtlebot)
5. [ceres-solver](https://github.com/tyuownu/ceres-solver)
6. [stage_ros](https://github.com/tyuownu/stage_ros)

# Install

First, we need to download packages. Run:

```shell
cd docs/scripts/
bash download.sh
```

Second, we build the packages.
```shell
# change to you catkin workspace
cd you-catkin-workspace
catkin_make_isolated
```

# Packages

__The package list in this repository.__

## micvision

### micvision_exploration

`micvision_exploration`: to explore the whole map of unknown environment.

### micvision_localization

`micvision_localization`: to localize the robot in a map.

## micvision_sim

This package using `stage` to simulate the real environment, you can using the
`stage_ros` from the ROS or using the `stage_ros` our provide. The difference
is that what the ROS's scan output is accurate data without any error, but in
the package we gave, we also adding error to the scan output.

The package also have two part: `mapping` and `navigation`.

### mapping

usage:

```shell
# willow world
roslaunch micvision_sim willow-mapping-sim.launch

# or using maze world below
# roslaunch micvision_sim maze-mapping-sim.launch

# then, in other terminal
rosservice call /StartExploration

# we also provide other command to control the robot
# Stop the exploration action
rosservice call /StopExploration

# Stop the robot
rosservice call /Stop

# Pause the robot
rosservice call /Pause
```

### navigation

usage:

```shell
# willow world
roslaunch micvision_sim willow-navigation-sim.launch
# or using maze world instead
# roslaunch micvision_sim maze-navigation-sim.launch

# find the location of the robot
rosservice call /StartLocalization

# click on the map using rviz, so the robot move to it
```

## micvision_mapping

__This package provide the real world & robot for mapping.__

You can use it with `turtlebot2(kobuki)` or `turtlebot3`.

Usage:

```shell
# turtlebot2
roslaunch micvision_mapping mapping_turtlebot2.launch

# turtlebot3
roslaunch micvision_mapping mapping_turtlebot3.launch

# other command please reference to the micvision_sim mapping function
```

## micvision_navigation

__This package provide the real world and robot for navigation.__

You can use it with `turtlebot2(kobuki)` or `turtlebot3`.

Usage:

```shell
# turtlebot2
roslaunch micvision_navigation navigation_turtlebot2.launch

# turtlebot3
roslaunch micvision_navigation navigation_turtlebot3.launch

# other command please reference to the micvision_sim navigation function
```

