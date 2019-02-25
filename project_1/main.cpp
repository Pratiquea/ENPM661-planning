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


// Function to create/allocate a new node
Node* new_node(int map[dim][dim], int x, int y, int move_to_x, int move_to_y, int accumulated_cost, Node* parent)
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


// row and colomn values for bottom, left, top, right repectively



int main()
{
	return;
}