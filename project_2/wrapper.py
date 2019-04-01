
#!/usr/bin/env python
# coding: utf-8

import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
import argparse
import heapq
import copy
from math import pow,sqrt

#Global variables
diagonal_cost = 14
straight_cost = 10
# resolution = 1

#dimensions of map
length = 250
breadth = 150
#direction vectors
dy = [-1,1,0,0,-1,1,-1,1]
dx = [0,0,-1,1,-1,-1,1,1]
#color codes
heap_node_color = [220,20,60]
visited_node_color = [255,219,28]
path_color = [50,90,255]
goal_color = (0,190,255)
start_color = (255,0,0)
#weight multiplier for heuristic
w = 1




class node:
    def __init__(self,name,loc,parent_name = None,cost = float("inf")):
        self.name = name
        self.loc = loc
        self.parent_name = parent_name
        self.cost = cost



def inside_rect(x,y,pad):
    if(not pad):
        if(y-112.5<=0 and x-100<=0 and -y+67.5<=0 and -x+50<=0):
            return True
        else:
            return False
    else:
        if(y-117.5<=0 and x-105<=0 and -y+62.5<=0 and -x+45<=0):
            return True
        else:
            return False

def inside_circle(x,y,pad):
    if(not pad):
        if((x-190)**2+(y-130)**2-15**2<=0):
            return True
        else:
            return False
    else:
        if((x-190)**2+(y-130)**2-20**2<=0):
            return True
        else:
            return False

def inside_ellipse(x,y,pad):
    if(not pad):
        if((x-140)**2*(6**2)+(y-120)**2*(15**2)-(15**2)*(6**2)<=0):
            return True
        else:
            return False
    else:
        if((x-140)**2*(11**2)+(y-120)**2*(20**2)-(20**2)*(11**2)<=0):
            return True
        else:
            return False


def inside_poly(x,y,pad):
    if(not pad):
        if(-41*x-25*y+6525<=0 and 38*x+23*y-8530<=0 and 37*x-20*y-6101<=0 and -y+15<=0 and (4*x+38*y-2628<=0 or -38*x+7*y+5830<=0)):
            return True
        else:
            return False
    else:
        if(-41*x-25*y+6284.9<=0 and 38*x+23*y-8752.09<=0 and 37*x-20*y-6311.29<=0 and -y+10<=0 and (4*x+38*y-2819.05<=0 or -38*x+7*y+5636.8<=0)):

            return True
        else:
            return False



# pad= False
# for i in range(0,breadth):
#     for j in range(0,length):
#         if(inside_rect(j,i,pad) or inside_circle(j,i,pad) or inside_ellipse(j,i,pad) or inside_poly(j,i,pad)):
#             map[i][j]=1

def generate_map(breadth,length,pad):
    map = np.zeros((breadth,length))
    for i in range(0,breadth):
        for j in range(0,length):
            if(inside_rect(j,i,pad) or inside_circle(j,i,pad) or inside_ellipse(j,i,pad) or inside_poly(j,i,pad)):
                map[i][j]=1
    map = cv2.flip(map.copy(),0)
    return map
    



def mark_start_and_goal(visual_b_map,start,goal):
    cv2.circle(visual_b_map,(start[1],start[0]), 1, start_color, -1)
    cv2.circle(visual_b_map,(goal[1],goal[0]), 1, goal_color, -1)

def visualize_path(visual_b_map,start,goal,path):
    # visual_b_map[start[0]][start[1]] = start_color
    # visual_b_map[goal[0]][goal[1]] = goal_color
    for point in path:
        visual_b_map[point[0]][point[1]] = path_color

    cv2.circle(visual_b_map,(start[1],start[0]), 1, start_color, -1)
    cv2.circle(visual_b_map,(goal[1],goal[0]), 1, goal_color, -1)
    
    visual_b_map = cv2.cvtColor(visual_b_map,cv2.COLOR_RGB2BGR)
    cv2.namedWindow('map',cv2.WINDOW_NORMAL)
    cv2.imshow('map',visual_b_map)
    cv2.waitKey(0)
    # plt.imshow(visual_b_map)
    # plt.show()



def is_valid(b_map, state):
    if(state[0]<0 or state[0]>breadth-1 or state[1]<0 or state[1]>length-1):
        print('\n\tState is invalid. Out of bounds')
        print('\tEnter state as "row,col" where row is in range [0,'+str(breadth-1)+'] col is in range [0,'+str(length-1)+'].\n')
        return False
    if(b_map[state[0]][state[1]] ):
        print('\n\tState is invalid i.e inside obstacle\n')
        return False
    return True



def create_visual_map(b_map):
    temp = copy.deepcopy(b_map)
    temp[temp==0]=255
    # temp = np.asarray(temp,dtype=int)
    visual_b_map = temp
    visual_b_map = np.dstack((visual_b_map,temp,temp))
    visual_b_map = np.ascontiguousarray(visual_b_map, dtype=np.uint8)
    return visual_b_map



###############################################################################
####################### Dijkstra and helper functions #########################
###############################################################################

'''
input:
    node: item of class node
    dx: x component of direction vector
    dy: y component of direction vector

output:
    cost to come to new node created using direction vectors
'''
def cal_cost_to_come(node, dx, dy):
    if(dy and dx):
        #diagonal
        return node.cost + diagonal_cost
    else:
        #straight path i.e. left, right, top, bottom
        return node.cost + straight_cost


def explore_neighbours(r,c,Q,b_map,visited_nodes,node_map,visual_b_map,start,goal,show_animation):
    for i in range(0,8):
        new_c = c+dx[i]
        new_r = r+dy[i]

        #check if the new location is within bounds
        if(new_c<0 or new_r<0 or new_c>=length or new_r>=breadth):
            continue

        #check if state is valid i.e not an obstacle
        if(b_map[new_r][new_c]):
            continue

        #check is the node is visited
        if(visited_nodes[new_r][new_c]):
            continue

        new_index = new_r*length+new_c
        index = r*length+c
        
        #calculate cost to come for newly generated node
        curr_cost = cal_cost_to_come(node_map[index], dx[i], dy[i])
        
        if(curr_cost<node_map[new_index].cost):
            #update cost of new node
            node_map[new_index].cost = curr_cost
            #update parent(name) of new node
            node_map[new_index].parent_name = node_map[index].name
            #add current node to heap
            heapq.heappush(Q,(node_map[new_index].cost, node_map[new_index].name, node_map[new_index]))
            # if(show_animation):
            visual_b_map[new_r][new_c] = heap_node_color
    
    visited_nodes[r][c] = 1
    visual_b_map[r][c] = visited_node_color
    if(show_animation):
        # print(show_animation)
        mark_start_and_goal(visual_b_map,start,goal)
        visual_b_map = cv2.cvtColor(visual_b_map,cv2.COLOR_RGB2BGR)
        cv2.namedWindow('map',cv2.WINDOW_NORMAL)
        # cv2.resizeWindow(',map', 720,1080)
        cv2.imshow('map',visual_b_map)
        cv2.waitKey(1)
        # time.sleep(1)
    
    return Q




def Dijkstra(start,goal,node_map,visited_nodes,b_map,visual_b_map,show_animation):
    Q = []
    heapq.heapify(Q) 
    # if not list(Q):
    #     print('empty2')
    index = start[0]*length+start[1]
    node_map[index].cost = 0
    heapq.heappush(Q,(node_map[index].cost, node_map[index].name, node_map[index]))
    while(list(Q)):
        current_node = heapq.heappop(Q)
        #print(current_node[0],current_node[1],current_node[2])
        curr_r = current_node[2].loc[0]
        curr_c = current_node[2].loc[1]
        #check node for duplicity in heap
        if(visited_nodes[curr_r][curr_c]):
            continue
        #check if the goal is reached
        if(curr_r == goal[0] and curr_c == goal[1]):
            print('\tSuccess. Path found\n')
            return 'Success'

        #update heap after exploring neighbours
        Q = explore_neighbours(curr_r,curr_c,Q,b_map,visited_nodes,node_map,visual_b_map,start,goal,show_animation)
    return 'Failure'




###############################################################################
########################## A* and helper functions ############################
###############################################################################

def cal_chebyshev_distance(row, col, goal):
    return max(abs(row-goal[0]),abs(col-goal[1]))

def cal_l2_dist(row, col, goal):
    return sqrt(pow((row-goal[0]),2) + pow((col-goal[1]),2))


def explore_neighbours_a_star(r,c,Q,b_map,visited_nodes,node_map,visual_b_map,start,goal,show_animation,heuristic):
    for i in range(0,8):
        new_c = c+dx[i]
        new_r = r+dy[i]

        #check if the new location is within bounds
        if(new_c<0 or new_r<0 or new_c>=length or new_r>=breadth):
            continue

        #check if state is valid i.e not an obstacle
        if(b_map[new_r][new_c]):
            continue

        #check is the node is visited
        if(visited_nodes[new_r][new_c]):
            continue

        new_index = new_r*length+new_c
        index = r*length+c
        
        #calculate cost to come for newly generated node
        if(heuristic=='cheby'):
            curr_cost = cal_cost_to_come(node_map[index], dx[i], dy[i]) + w*cal_chebyshev_distance(new_r, new_c, goal)
        elif(heuristic=='euc'):
            curr_cost = cal_cost_to_come(node_map[index], dx[i], dy[i]) + w*cal_l2_dist(new_r, new_c, goal)
        
        if(curr_cost<node_map[new_index].cost):
            #update cost of new node
            node_map[new_index].cost = curr_cost
            #update parent(name) of new node
            node_map[new_index].parent_name = node_map[index].name
            #add current node to heap
            heapq.heappush(Q,(node_map[new_index].cost, node_map[new_index].name, node_map[new_index]))
            # if(show_animation):
            visual_b_map[new_r][new_c] = heap_node_color
    visited_nodes[r][c] = 1
    visual_b_map[r][c] = visited_node_color
    if(show_animation):
        mark_start_and_goal(visual_b_map,start,goal)
        cv2.namedWindow('map',cv2.WINDOW_NORMAL)
        visual_b_map = cv2.cvtColor(visual_b_map,cv2.COLOR_RGB2BGR)
        # cv2.resizeWindow(',map', 720,1080)
        cv2.imshow('map',visual_b_map)
        cv2.waitKey(1)
        # time.sleep(1)
    
    return Q




def A_star(start,goal,node_map,visited_nodes,b_map,visual_b_map,show_animation,heuristic):
    Q = []
    heapq.heapify(Q) 
    # if not list(Q):
    #     print('empty2')
    index = start[0]*length+start[1]
    node_map[index].cost = 0
    heapq.heappush(Q,(node_map[index].cost, node_map[index].name, node_map[index]))
    while(list(Q)):
        current_node = heapq.heappop(Q)
        #print(current_node[0],current_node[1],current_node[2])
        curr_r = current_node[2].loc[0]
        curr_c = current_node[2].loc[1]
        #check node for duplicity in heap
        if(visited_nodes[curr_r][curr_c]):
            continue
        #check if the goal is reached
        if(curr_r == goal[0] and curr_c == goal[1]):
            print('\tSuccess. Path found\n')
            return 'Success'

        #update heap after exploring neighbours
        Q = explore_neighbours_a_star(curr_r,curr_c,Q,b_map,visited_nodes,node_map,visual_b_map,start,goal,show_animation,heuristic)
    return 'Failure'






def main():


    Parser = argparse.ArgumentParser()
    Parser.add_argument('--src', type=int, nargs="+", default= [0,0], help='location of starting point, Default: (0,0)')
    Parser.add_argument('--goal', type=int, nargs="+", default= [10,10], help='location of goal point, Default: (10,10)')
    Parser.add_argument('--animation',default='False',help="Flag to show animation;True=show animation, False = don't show animation")
    Parser.add_argument('--algo',default='a*',help="Algorithm that you want to use; a* or dij (short for dijkstra)")
    Parser.add_argument('--heuristic',default='cheby',help="heuristic used by A* algorithm; options: cheby or euc")

    Args = Parser.parse_args()
    
    ################# Some checks on parsed inputs ###################
    if(np.shape(Args.src)[0]>2 or np.shape(Args.src)[0]<2):
        print('Expected two values. Enter start coordintes, eg: if start location(row,col) is (10, 10) then the input format wil be\n "--src 10 10"')
        return
    if(np.shape(Args.goal)[0]>2 or np.shape(Args.goal)[0]<2):
        print('Expected two values. Enter goal coordintes, eg: if goal location(row,col) is (10, 10) then the input format wil be\n "--goal 10 10"')
        return
    
    ################# Storing parsed data ##################
    start = Args.src
    goal = Args.goal
    if(Args.animation.lower() == 'false'):
        show_animation = False
    elif(Args.animation.lower() == 'true'):
        show_animation = True
    else:
        print('--animation should either be "True" of "False"')
        return
    if(not(Args.heuristic.lower()=='cheby' or Args.heuristic.lower()=='euc')):
        print('Unable to recognize input heuristic; try "cheby" or "euc"')
        return
    heuristic=Args.heuristic
    algo = Args.algo
    print(algo)
    print('start location = '+str(start))
    print('goal location = '+str(goal))

    
    ################### Code starts here ###################
    

    # create obstacle map
    b_map = generate_map(breadth,length,False)

    #create obstacle map for visualization
    visual_b_map = create_visual_map(b_map)

    #mark start and goal nodes
    mark_start_and_goal(visual_b_map,start,goal)
    
    #intialize nodes
    node_map = []
    # node location formula = i*length+j  ; i = row, j=col
    for i in range(0,breadth):
        for j in range(0,length):
            node_map.append(node(i*length+j+1,(i,j)))

    # Create 2D array to keep track of visited nodes
    visited_nodes = np.zeros_like(b_map)
    
    # checking validity of start node

    if(not is_valid(b_map, start)):
        return
    # checking validity of goal node
    if (not is_valid(b_map, goal)): 
        return

    #Start solving 
    if(algo.lower()=='dij'):
        ans = Dijkstra(start,goal,node_map,visited_nodes,b_map,visual_b_map,show_animation)
    elif(algo.lower()=='a*'):
        ans = A_star(start,goal,node_map,visited_nodes,b_map,visual_b_map,show_animation, heuristic)
    else:
        print('Unable to recognize the input algorithm. exiting...')
        print('Enter either "a*" or "dij" (short for dijkstra)')
        return
    if(ans=='Success'):
        path = []
        index = goal[0]*length+goal[1]
        while(not node_map[index].loc==tuple(start)):
            # print(index)
            path.append(np.asarray(node_map[index].loc))
            index = node_map[index].parent_name-1
        visualize_path(visual_b_map,start,goal,path[1:])
        cv2.destroyAllWindows()
    else:
        print('No path found')





if __name__ == '__main__':
	main()
