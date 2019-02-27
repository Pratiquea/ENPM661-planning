// Program to print path from root node to destination node 
// for N*N -1 puzzle algorithm using Brute force search
// The solution assumes that instance of puzzle is solvable 

#include "8_puzzle_problem.h"




int main(int argc, char** argv)
{
	//initialize the initial state of puzzle
 	int init_state[dim][dim] =	{
 			 						{1,2,3},
			 						{4,5,6},
			 						{7,8,0}
		 					 	};
	
	//initialize the goal state of puzzle
	int goal_state[dim][dim] =	{
			 						{1,2,0},
			 						{4,5,3},
			 						{7,8,6}
			 					};
	if (argc < 19)
	{
		cout <<endl
			 << "Enter the following arguments ..\n"
			 <<"1) initial state of the puzzle in the following format:\n"
			 <<"\t{\n\t  {1,2,3},\n\t  {4,5,0},\n\t  {6,8,7},\n\t}\n"
			 <<"\t(refer Readme for example)\n"
             <<"2) goal state of the puzzle (format = same as initial state format)\n"
             <<endl<<'\t'<<"Exiting now..."<<endl<<endl;
        return -1;
	}



	if(Debug_low_level)
	{
		for (int i = 0; i < argc; ++i) 
	        cout << "argc = "<< i<<"  value = "<<argv[i] << "\n"; 
	}

	//saving the input from user
	int row=0;
	int col=0;
	for(int i=1 ; i<= (argc-1)/2 ; i++)
	{
		init_state[row][col] = atoi(argv[i]);
		if(Debug_low_level)
		{
			cout<<"init_state["<<row<<"]["<<col<<"] = "<<argv[i]<<endl;
		}

		goal_state[row][col] = atoi(argv[i+9]);
		if(Debug_low_level)
		{
			cout<<"goal_state["<<row<<"]["<<col<<"] = "<<argv[i+9]<<endl;
		}

		col++;
		if((col)%3 == 0)
		{
			row++;
			col=0;
		}
	}
	//check if init state entered by user is valid
	if(!is_puzzle_valid(init_state))
	{
		return -1;
	}

	//check if goal state entered by user is valid
	if(!is_puzzle_valid(goal_state))
	{
		return -1;
	}

	
	//If both states are valid, check if goal state is solvable or not
	if(isSolvable(goal_state))
	{
		cout<<endl<<"Puzzle seems to be Solvable"<<endl;
	}
	else
	{
		cout<<endl<<"Puzzle is not Solvable"<<endl<<"Exiting..."<<endl<<endl;
		return -1;
	}




	//print init state
	if(Debug_high_level)
	{
		cout<<endl<<"Initial state"<<endl;
		print_puzzle(init_state);
		cout<<endl<<"Goal state"<<endl;
		print_puzzle(goal_state);
	}

	

	//Variable to store coordinates
	int x=-1, y=-1;

	//finding coordinates of blank tile i.e value 0  in init_state matrix
	for(int i=0 ; i<= dim-1 ; i++)
	{
		for(int j=0 ; j<= dim-1 ; j++)
		{
			if(!init_state[i][j])
			{
				if(Debug_low_level)
				{
					cout<<"found blank tile coordinates"<<endl;
				}
				//save coordinates
				x=i;
				y=j;
			}
		}
	}


	solve(init_state, goal_state, x, y);

	return 0;
}