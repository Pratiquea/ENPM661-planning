// Program to print path from root node to destination node 
// for N*N -1 puzzle algorithm using Brute force search
// The solution assumes that instance of puzzle is solvable 
#include <bits/stdc++.h> 
using namespace std; 
#define dim 3 
  
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
		total_cost_a = a->cost_to_come + a->accumulated_cost;
		total_cost_b = b->cost_to_come + b->accumulated_cost;

		return(total_cost_a > total_cost_b)
	}
};


// Function to print dim x dim matrix
void print_puzzle(int map[dim][dim])
{
	cout<<endl;
	for(i=0 ; i<= dim-1 ; i++)
	{
		for(j=0 ; j<= dim-1 ; j++)
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
	for(i=0 ; i<=dim-1 ; i++)
	{
		for(j=0 ; j<=dim-1 ; j++)
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
	// Building a queue structure to store active nodes of search tree
	node_queue<Node*, std::vector<Node*>, compare_nodes> Q;

	// create a root node and calculate its cost
	Node* root = create_new_node(init_state,x,y,x,y,0,NUL);
	//cost of this node is number of nodes 
	root->cost_to_come = calculate_cost_to_come(init_state, goal_state);
}




int main()
{
	return;
}