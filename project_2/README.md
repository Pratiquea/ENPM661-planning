# Problem statement

## File information
| Source files       	|                                   Purpose                                   	|
|--------------------	|:---------------------------------------------------------------------------:	|
| wrapper.py         	| Main file to run 	                                                            |
|                    	|                                                                             	|

## Directory structure
```
project_2.zip
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
* --animation, description: Flag to show animation;True=show animation, False = don't show animation")
```

```
* --algo,description: Algorithm that you want to use; a* or dij (short for dijkstra)")
```

```
* --heuristic,description: heuristic used by A* algorithm; options: cheby or euc")
```

```
* --robot, description: Type of robot(integer): 0(for point robot) or radius of robot(for circular robot)
```

```
### Start and goal node limits

### Obstacle map color scheme
1. Start: Crimson Red star
2. Goal: Cyan star
3. Visited node: yellow
4. Active node(inside heap): Red

## Assumption
- obstacle at corner not rounded
- black area is obstacle
- start and goal are marked in star, the center of the star is the actual node.
- Boundary of map is considered obstacle.
- Due to resolution location of node is convert into int. Thus, a valid start or goal node may not be valid due to the conversion(or downsampling).
