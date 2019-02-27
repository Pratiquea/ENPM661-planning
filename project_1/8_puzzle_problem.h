#include <bits/stdc++.h> 
// #include <unordered_set>
using namespace std; 
#define dim 3
// flag to show print statement at all levels
#define Debug_low_level 0
// flag to show print statement at high level
#define Debug_high_level 1



////////////////////////////////////////////////////
//////////// 	  Global variables	   /////////////
////////////////////////////////////////////////////
// Path to store nodes involved in solution
const char *Nodes_path_txt_file = "/home/pratique/Downloads/ENPM661-planning/project_1/nodePath.txt";
const char *Nodes_txt_file = "/home/pratique/Downloads/ENPM661-planning/project_1/Nodes.txt";
const char *Nodes_info_txt_file = "/home/pratique/Downloads/ENPM661-planning/project_1/NodesInfo.txt";

//file opening variables
std::ofstream node_path_f;
std::ofstream nodes_f;
std::ofstream node_info_f;

////////////////////////////////////////////////////
//////////// 	  Helper functions	   /////////////
////////////////////////////////////////////////////

// A structure for state space tree 
struct Node
{
	//stores a unique number to represent a node
	int node_no;

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
Node* create_new_node(int map[dim][dim], int x, int y, int move_to_x, int move_to_y, int accumulated_cost, Node* parent, int node_no)
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
	node->node_no = node_no;

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
float* print_puzzle(int map[dim][dim])
{
	cout<<endl;
	static float flat_map[9];
	// std::vector<int> flat_map;
	int index = 0;
	for(int i=0 ; i<= dim-1 ; i++)
	{
		cout<<'\t';
		for(int j=0 ; j<= dim-1 ; j++)
		{
			cout<<map[i][j];
			flat_map[index] = float(map[j][i]);
			index++;
		}
		cout<<endl;
	}
	// v.push_back(flat_map)
	return flat_map;
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


//Function to write puzzle to file
void write_puzzle_to_file(std::ofstream& f,int map[dim][dim])
{
	for(int i=0 ; i<3 ; i++)
	{
		for(int j=0 ; j<3 ; j++)
		{
			f<<std::fixed<<float(map[j][i])<<'\t';
		}
	}
	f<<'\n';
}


//Print the solution i.e the path from goal node to root node
void print_solution(Node* root)
{
	// const char *path="/home/user/file.txt";
 //    std::ofstream file(path); //open in constructor
 //    std::string data("data to write to file");
 //    file << data;

	if(root == NULL)
	{	
		node_path_f.open(Nodes_path_txt_file);
		if(Debug_low_level)
		{
			cout<<"opened file"<<endl;
		}
		return;
	}  //cout<<endl<<"return"<<endl;


	// recursively call the function untill the first node
	print_solution(root->parent);
	if(Debug_low_level)
	{
		cout<<endl<<"before print solution"<<endl;
	}
	
	//A pointer to store the outpu of print_puzzle
	float* flat_node;
	flat_node = print_puzzle(root->map);
	
	if(Debug_low_level)
	{
		cout<<endl<<"after print solution"<<endl;
	}
		
	cout<<endl;
	//retrieve the elements of 1D array

	// if(Debug_low_level)
	// {
		for (int i=0; i<9; i++)
		{
			node_path_f<<std::fixed<<*(flat_node+i)<<'\t';
		}
		node_path_f<<'\n';
	// }

	cout<<endl;	
}


//Puzzle solver code
void solve(int init_state[dim][dim],int goal_state[dim][dim],int x, int y)
{
	cout<<endl<<"solving puzzle"<<endl;

	int iterator = 1;
	// Building a priority queue structure to store active nodes of search tree
	priority_queue<Node*, std::vector<Node*>, compare_nodes> Q;
	nodes_f.open(Nodes_txt_file);
	node_info_f.open(Nodes_info_txt_file);
	
	// create a root node and calculate its cost
	Node* root = create_new_node(init_state,x,y,x,y,0,NULL,iterator);
	//cost of this node is number of misplaces tiles between init state and goal state
	root->cost_to_come = calculate_cost_to_come(init_state, goal_state);
	
	//Write node data to node info file
	node_info_f<<std::fixed
			   <<float(root->node_no)<<'\t'
			   <<float(0)<<'\t'
			   <<float(root->cost_to_come)<<'\n';

	//Write root node to text file containg list of explored nodes
	write_puzzle_to_file(nodes_f,root->map);


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
			if(Debug_high_level)
			{
				cout<<"total nodes explored = "<<iterator<<endl;
				cout<<"closing all files"<<endl;
			}
			node_path_f.close();
			node_info_f.close();
			nodes_f.close();
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
				iterator++;
				Node* child = create_new_node(least_cost_node->map, least_cost_node->x, least_cost_node->y, newX, newY, least_cost_node->accumulated_cost+1,least_cost_node, iterator);

				// Calculate cost for the child node
				child->cost_to_come=calculate_cost_to_come(child->map, goal_state);

				//Write child to text file containg list of explored nodes
				write_puzzle_to_file(nodes_f,child->map);


				//Write node data to node info file
				node_info_f<<std::fixed
						   <<float(child->node_no)<<'\t'
			   			   <<float(child->parent->node_no)<<'\t'
			   			   <<float(child->cost_to_come)<<'\n';

				Q.push(child);
			}
		}
 	}
 	//End of function
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
