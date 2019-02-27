The code is meant to solve 8 puzzle problem. I tried exploring all the paths using Brute force search but there were instances when it took too long for the algorithm to calculate a solution and the process got killed by the operating system. Therefore to circumvent this problem, I used have cost associated with each node to make the search faster. The table below shows the overview of files.

| Source files       	|                                   Purpose                                   	|
|--------------------	|:---------------------------------------------------------------------------:	|
| main.cpp           	| Defines the main procedure. This includes brute force search implementation 	|
| 8_puzzle_problem.h 	| Includes node structure and all the helper function to execute main.cpp     	|
|                    	|                                                                             	|


---
**NOTE**

In the node_info.txt file the parent node_no for the root node(first node) is written as zero even though it is set to **NULL** in the code. When I tried to write **NULL** to file, I got a segmentation fault.

---


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


### Visual Aid and debugger
There are two flags defined in the 8_puzzle_problem.h file

| Source files       	|                                   Purpose                                   	|
|--------------------	|:---------------------------------------------------------------------------:	|
| main.cpp           	| Defines the main procedure. This includes brute force search implementation 	|
| 8_puzzle_problem.h 	| Includes node structure and all the helper function to execute main.cpp     	|
|                    	|                                                                             	|

By default both the flag are not set.
