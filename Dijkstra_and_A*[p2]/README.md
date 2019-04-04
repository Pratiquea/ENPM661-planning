# Problem statement
A* and Dijkstra algroithm for point robot and rigid robot.
## File information
| Source files       	|                                   Purpose                                   	|
|--------------------	|:---------------------------------------------------------------------------:	|
| wrapper.py         	| Main file to run 	                                                            |
| README.md           | Instruction about the code                                                    |

## Directory structure
```
proj2_prateek_arora_python.zip
├── wrapper.py
└── README.md
```

### Instruction on how to run the code
wrapper.py is the main file to run and it can be run using default parameters using the following command:
```
python wrapper.py
```
Commandline options:
* --src, description: location of starting point, Default: (10,10). Example:
```
python wrapper.py --src 10 10
```
* --goal, description: location of goal point, Default: (50,50). Example:
```
python wrapper.py --goal 50 50
```
* --animation, description: Flag to show animation;True=show animation, False = don't show animation. Default: True
```
 python wrapper.py --animation True
```
* --algo,description: Algorithm that you want to use; a* or dij (short for dijkstra)")
```
python wrapper.py --algo a*
```
* --heuristic,description: heuristic used by A* algorithm; options: cheby or euc")
```
python wrapper.py --heuristic euc
```
* --robot, description: Type of robot(integer): 0(for point robot) or radius of robot(for circular robot)
```
python wrapper.py  --robot 1
```
* --clear, description: Clearance to maintain from obstacle(in integer). Default: 0
```
python wrapper.py  --clear 1
```
* --res',description: resolution of the map. Greater the resolution lesser the number of nodes(res>=1). Default:0
```
python wrapper.py  --res 1
```
* An example to use multiple commandline arguments
```
python wrapper.py --src 10 10 --goal 240 140 --animation True --robot 3
```

### Start and goal node limits
start and goal are in the range [0,0] to [250,150]. Format for start and goal input is (x-coordinate,y-coordinate)
### Obstacle map color scheme
1. Start: Crimson Red star
2. Goal: Cyan star
3. Visited node: yellow
4. Active node(inside heap): Red
5. Obstacle: black

## Assumption
- obstacle at corner are not rounded
- start and goal are marked in star, the center of the star is the actual node.
- Boundary of map is considered obstacle.
- Due to resolution location of node is convert into int. Thus, a valid start or goal node may not be valid due to the conversion(or downsampling).

### Note
- The Final map with the path will be displayed for 5 seconds.
- The code runs perfectly on python 2. It hasn't been tested for python 3
