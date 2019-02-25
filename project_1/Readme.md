The code is meant to solve 8 puzzle problem. The table below shows the overview of files.

| Source files       	|                                   Purpose                                   	|
|--------------------	|:---------------------------------------------------------------------------:	|
| main.cpp           	| Defines the main procedure. This includes brute force search implementation 	|
| 8_puzzle_problem.h 	| Includes node structure and all the helper function to execute main.cpp     	|
|                    	|                                                                             	|


### Instruction on how to run the code

#### Compiling the code
open a terminal(in linux) and go to the directory where the code is located. Then run the following command:
```
 g++ main.cpp -o a -std=c++11
```
#### Running the code
After the code has been compiled, it can be run using the following command:
```
./a {{1,2,3},{4,5,6},{7,8,0}} {{1,2,3},{4,5,0},{7,8,6}}

```
The first 2D matrix is ```{{1,2,3},{4,5,6},{7,8,0}}``` is the Initial state of the puzzle and the second 2D matrix```{{1,2,3},{4,5,0},{7,8,6}}``` is the final state.
