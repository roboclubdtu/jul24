#!/bin/bash

# Launch baxter shutdown sequence scripts
rosrun baxter_tools tuck_arms.py -t
rosrun baxter_tools enable_robot.py -S