#!/bin/bash

# Launch Baxter init script
rosrun baxter_tools enable_robot.py -e
rosrun baxter_tools tuck_arms.py -u
rosrun dtu_jul24 baxter_setup