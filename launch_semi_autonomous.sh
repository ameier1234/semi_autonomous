#!/bin/bash
##launches the teleop controler and the semi autonomous addon, then manualy launches stage_ros
roslaunch semi_autonomous semi_autonomous.launch
rosrun stage_ros stageros $(rospack find stage_ros)/world.willow-erratic.world
