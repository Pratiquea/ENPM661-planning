# Problem statement

# File information
| Source files       	|                                   Purpose                                   	|
|--------------------	|:---------------------------------------------------------------------------:	|
| wrapper.py         	| Main file to run 	                                                            |
|                    	|                                                                             	|

# Directory structure
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
```
--src, default= [0,0], help='location of starting point, Default: (0,0)')
--goal, default= [10,10], help='location of goal point, Default: (10,10)')
--animation,default='False',help="Flag to show animation;True=show animation, False = don't show animation")
--algo,default='a*',help="Algorithm that you want to use; a* or dij (short for dijkstra)")
--heuristic,default='cheby',help="heuristic used by A* algorithm; options: cheby or euc")
--robot, default=0,help="Type of robot(integer): 0(for point robot) or radius of robot(for circular robot)
```

## Assumption
- obstacle at corner not rounded
- black area is obstacle
- start and goal are marked in star, the center of the star is the actual node.
