# ROS mapping simulation

This repository is the result of our Teams work during and after the ROB-N
workshop in 2020. This workshop is part of our studies in "applied research
in engineering sciences" and was held remote because of the ongoing COVID-19
pandemic.

## Starting

You will need a ROS instalation at `/opt/ros/melodic/`.

Run the following commands:

```sh
mkdir workspace
cd workspace
git clone --recursive git@github.com:M4GNV5/ROS-Mapping-Simulation src
catkin_make

roscore &
source devel/setup.bash
cd src
./start_all.sh
```

## Preview

You can view the simulation inputs, the tracked position and the generated map
using rviz.

![](https://i.m4gnus.de/112deb.png)
