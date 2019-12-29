# Problem statement
A* with differential constraints
## File information
| Source files       	|                                   Purpose                                   	|
|--------------------	|:---------------------------------------------------------------------------:	|
| A_star_diff_const.py | Main file to run  									  |
| README.md            | Instruction about the code       					  |

## Directory structure
```
proj2_prateek_arora_python.zip
├── A_star_diff_const.py
├── README.md
├── video.mp4
└── rrl_map.world
```
### Map coordinates
The input to map coordinates are in cm and the origin is at bottom left and not at center of map.

### Instruction on how to run the code
#### For first terminal
wrapper.py is the main file to run and it can be run using default parameters using the following command:
```
python A_star_diff_const.py
```
Commandline options:
* --src, description: location of starting point in cm, Default: (30,30).)(int) Example:
```
python A_star_diff_const.py --src 30 30
```
* --goal, description: location of goal point in cm, Default: (600,800). Example:
```
python A_star_diff_const.py --goal 600 800
```
* --clear, description: Clearance to maintain from obstacle(in integer). Default: 0.10 meter(float)
```
python wrapper.py  --clear 0.1
```
* An example to use multiple commandline arguments
```
python wrapper.py --src 10 10 --goal 240 140 --animation True --robot 3
```

#### For second terminal
Open another termianl to launch gazebo
for the world_file parameter enter the path to rrl_map.world file.
gazebo didn't open properly when relative path was given. ROBOT_INITIAL_POSE="-x -5.25 -y -4.35 -Y 1.57" remains same if the initial path is near 0,0

```
source /opt/ros/kinetic/setup.bash
```
```
ROBOT_INITIAL_POSE="-x -5.25 -y -4.35 -Y 1.57" roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=file:///home/pratique/Desktop/rrl_map.world 

```


### Start and goal node limits
start and goal are in the range [0,0] to [1110,1010]. Format for start and goal input is (x-coordinate,y-coordinate)
### Obstacle map color scheme
1. Start: Crimson Red star
2. Goal: Cyan star

## Assumption
- obstacle that are quadrilaterals are not rounded at corners
- start and goal are marked in star, the center of the star is the actual node.
- Boundary of map is considered obstacle.
- Due to resolution location of node is convert into int. Thus, a valid start or goal node may not be valid due to the conversion(or downsampling).

### Note
- The Final map with the path will be displayed for 2 seconds.
- The code runs perfectly on python 2. It hasn't been tested for python 3

### Header files used
* import numpy as np
* import cv2
* import glob
* import random
* import matplotlib.pyplot as plt
* import time
* import math
* import argparse
* import heapq
* import copy
* from math import sqrt
* import os
* import sys
* import rospy
* from geometry_msgs.msg import Twist
* from nav_msgs.msg import Odometry
