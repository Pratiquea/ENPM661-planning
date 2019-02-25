#include <bits/stdc++.h> 
// #include <unordered_set>
using namespace std; 
#define dim 3
// flag to show print statement at all levels
#define Debug_low_level 0
// flag to show print statement at high level
#define Debug_high_level 1

// A structure for state space tree 
struct Node
{
	// this variable stores the parent node from which the current node is generated
	Node* parent;

	// stores the cost to come i.e. number of misplaces tiles
	int cost_to_come;
	
	// store the puzzle state
	int map[dim][dim];
	
	// store the position of blank tile
	int x,y;
	
	//stores the cost so far
	int accumulated_cost;
};


// Function to create/allocate a new node
Node* create_new_node(int map[dim][dim], int x, int y, int move_to_x, int move_to_y, int accumulated_cost, Node* parent)
{
	Node* node = new Node;
	
	//Copy the puzzle map data from parent node to current node
	memcpy(node->map,map,sizeof node->map);

	// Move the location of tile from x,y to move_to_x.move_to_y
	swap(node->map[x][y], node->map[move_to_x][move_to_y]);

	// set the cost_to_come(number of misplaced tiles) to max value of int
	node->cost_to_come = INT_MAX;

	// set the accumulated cost 
	node->accumulated_cost = accumulated_cost;

	// update current position of blank tile
	node->x = move_to_x;
	node->y = move_to_y;

	// create a pointer to parent node
	node->parent = parent;

	return node;
}


// In order to connect nodes we need the following comparator
struct compare_nodes
{
	bool operator()(const Node* a, Node* b) const
	{
		int total_cost_a = a->cost_to_come + a->accumulated_cost;
		int total_cost_b = b->cost_to_come + b->accumulated_cost;

		return(total_cost_a > total_cost_b);
	}
};


// Function to print dim x dim matrix
void print_puzzle(int map[dim][dim])
{
	cout<<endl;
	for(int i=0 ; i<= dim-1 ; i++)
	{
		cout<<'\t';
		for(int j=0 ; j<= dim-1 ; j++)
		{
			cout<<map[i][j];
		}
		cout<<endl;
	}
}


// row and colomn move values for bottom, left, top, right repectively
int row[] = {1, 0, -1, 0};
int col[] = {0, -1, 0, 1};

//Function to calculate the cost to come or number of misplaced tiles i.e. number of tiles(except the blank one) not in their goal position
int calculate_cost_to_come(int init_map[dim][dim], int goal_map[dim][dim])
{
	int cost = 0;
	for(int i=0 ; i<=dim-1 ; i++)
	{
		for(int j=0 ; j<=dim-1 ; j++)
		{
			if(goal_map[i][j] && goal_map[i][j] != init_map[i][j])
				cost++;
		}
	}	
	return cost;
}

// Function to check validity of a position(coordinate)
int is_valid(int x, int y)
{
	return(x>=0 && x<dim && y>=0 && y<dim);	
}


//Print the solution i.e the path from goal node to root node
void print_solution(Node* root)
{
	if(root == NULL)
		return;

	// recursively call the function untill the first node
	print_solution(root->parent);
	print_puzzle(root->map);

	cout<<endl;
}


//Puzzle solver code
void solve(int init_state[dim][dim],int goal_state[dim][dim],int x, int y)
{
	cout<<endl<<"solving puzzle"<<endl;


	// Building a priority queue structure to store active nodes of search tree
	priority_queue<Node*, std::vector<Node*>, compare_nodes> Q;

	// create a root node and calculate its cost
	Node* root = create_new_node(init_state,x,y,x,y,0,NULL);
	//cost of this node is number of misplaces tiles between init state and goal state
	root->cost_to_come = calculate_cost_to_come(init_state, goal_state);

	// Add root to Queue
	Q.push(root);

	// Find a live node with least cost, add its children to list of active nodes and finally delete it from the list.
	while(!Q.empty())
	{	
		// cout<<"Building tree"<<endl;
		// find a live node with least cost
		Node* least_cost_node = Q.top();

		//The minimum cost node is deleted from active node list
		Q.pop();

		//If the least cost node is the answer then print the path
		if (least_cost_node->cost_to_come==0)
		{
			print_solution(least_cost_node);
			return;
		}

		
		int newX, newY;

		//Repeat the entire process for the children of this node
		// Since there are 4 actions, total 4 child nodes are possible
		for(int i=0 ; i<4 ; i++)
		{
			int newX = least_cost_node->x + row[i];
			newY = least_cost_node->y + col[i];
			
			if(is_valid(newX, newY))
			{
				Node* child = create_new_node(least_cost_node->map, least_cost_node->x, least_cost_node->y, newX, newY, least_cost_node->accumulated_cost+1,least_cost_node);
				child->cost_to_come=calculate_cost_to_come(child->map, goal_state);

				Q.push(child);
			}
		}
 	}
 	cout<<endl<<"End of search"<<endl;
}


//The following code is refered from GeeksForGeeks
// A utility function to count inversions in given array 'arr[]' 
int getInvCount(int arr[]) 
{ 
    int inv_count = 0; 
    for (int i = 0; i < 9 - 1; i++) 
        for (int j = i+1; j < 9; j++) 
             // Value 0 is used for empty space 
             if (arr[j] && arr[i] &&  arr[i] > arr[j]) 
             {
             	if(Debug_low_level)
             	{
             		cout<<endl<<arr[i]<<" > "<<arr[j]<<endl;
             	}
             	inv_count++;
             }
    return inv_count; 
} 
  
// This function returns true if given 8 puzzle is solvable. 
bool isSolvable(int puzzle[3][3]) 
{ 
    // Count inversions in given 8 puzzle 
    int invCount = getInvCount((int *)puzzle);
    if(Debug_low_level)
	{
		cout<<endl<<"invCount = "<<invCount<<endl;
	}
    // return true if inversion count is even. 
    return (invCount%2 == 0); 
}




bool is_puzzle_valid(int map[dim][dim])
{
	int n = dim*dim; 
	int row=0,col=0;
	int zero_counter = 0;
    // Put all array elements in a map 
    unordered_set<int> s; 
    for (int i = 0; i < n; i++) 
    { 
        s.insert(map[row][col]);
        if(map[row][col] == 0)
        {
        	zero_counter++;
        }

        col++;
		if((col)%3 == 0)
		{
			row++;
			col=0;
		}
    } 
	// check if there is a blank tile.
	if(zero_counter == 0)
	{
		cout<<endl<<"Invalid state since there is no blank tile"<<endl;
		return false;
	}  

	//

    // If all elements are distinct, size of 
    // set should be same array.
    return (s.size() == n);
}
