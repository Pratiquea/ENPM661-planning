
#!/usr/bin/env python
# coding: utf-8

import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
import argparse
import heapq
import copy
from math import sqrt

#Global variables
diagonal_cost = 1.41
straight_cost = 1



#direction vectors
# dy = [-1,1,0,0,-1,1,-1,1]
# dx = [0,0,-1,1,-1,-1,1,1]
dy = [-1,1,-1,1,-1,1,0,0]
dx = [-1,-1,1,1,0,0,-1,1]
#color codes
heap_node_color = [220,20,60]
visited_node_color = [255,219,28]
path_color = [50,90,255]
goal_color = (0,190,255)
start_color = (255,0,0)
#weight multiplier for heuristic
w = 1
w_cc = 1




class node:
    def __init__(self,name,loc,parent_name = None,cost = float("inf")):
        self.name = name
        self.loc = loc
        self.parent_name = parent_name
        self.cost = cost



def solid_robot(x,y,pad,resolution,length,breadth):
    x = x*resolution
    y = y*resolution
    if(not pad):
        return False
    else:
        if(x<pad or y<pad or x>length-pad-1 or y> breadth-pad-1 ):
            return True
        else:
            return False

def inside_rect(x,y,pad,resolution):
    x = x*resolution
    y = y*resolution
    if(not pad):
        if(y-112.5<=0 and x-100<=0 and -y+67.5<=0 and -x+50<=0):
            return True
        else:
            return False
    else:
        if(y-112.5-pad<=0 and x-100-pad<=0 and -y+67.5-pad<=0 and -x+50-pad<=0):
            return True
        else:
            return False

def inside_circle(x,y,pad,resolution):
    x = x*resolution
    y = y*resolution
    if(not pad):
        if((x-190)**2+(y-130)**2-15**2<=0):
            return True
        else:
            return False
    else:
        if((x-190)**2+(y-130)**2-(15+pad)**2<=0):
            return True
        else:
            return False

def inside_ellipse(x,y,pad,resolution):
    x = x*resolution
    y = y*resolution
    if(not pad):
        if((x-140)**2*(6**2)+(y-120)**2*(15**2)-(15**2)*(6**2)<=0):
            return True
        else:
            return False
    else:
        if((x-140)**2*((6+pad)**2)+(y-120)**2*((15+pad)**2)-((15+pad)**2)*((6+pad)**2)<=0):
            return True
        else:
            return False


def inside_poly(x,y,pad,resolution):
    x = x*resolution
    y = y*resolution
    if(not pad):
        if(-41*x-25*y+6525<=0 and 38*x+23*y-8530<=0 and 37*x-20*y-6101<=0 and -y+15<=0 and (4*x+38*y-2628<=0 or -38*x+7*y+5830<=0)):
            return True
        else:
            return False
    else:
        if(-41*x-25*y+6525-pad*sqrt(41**2 + 25**2)<=0 and 38*x+23*y-8530-pad*sqrt(38**2 + 23**2)<=0 and 37*x-20*y-6101-pad*sqrt(20**2 + 37**2)<=0 and -y+15-pad<=0 and (4*x+38*y-2628-pad*sqrt(4**2 + 38**2)<=0 or -38*x+7*y+5830-pad*sqrt(38**2 + 7**2)<=0)):

            return True
        else:
            return False



# pad= False
# for i in range(0,breadth):
#     for j in range(0,length):
#         if(inside_rect(j,i,pad) or inside_circle(j,i,pad) or inside_ellipse(j,i,pad) or inside_poly(j,i,pad)):
#             map[i][j]=1

def generate_map(breadth,length,pad,resolution):
    map = np.zeros((int(breadth),int(length)))
    for i in range(0,int(breadth)):
        for j in range(0,int(length)):
            if(inside_rect(j,i,pad,resolution) or inside_circle(j,i,pad,resolution) or inside_ellipse(j,i,pad,resolution) or inside_poly(j,i,pad,resolution) or solid_robot(j,i,pad,resolution,length,breadth)):
                map[i][j]=1
    # map = cv2.flip(map.copy(),0)
    # plt.imshow(map)
    # plt.show()
    return map
    



def mark_start_and_goal(visual_b_map,start,goal):
    cv2.circle(visual_b_map,(start[1],start[0]), 1, start_color, -1)
    cv2.circle(visual_b_map,(goal[1],goal[0]), 1, goal_color, -1)

def visualize_path(visual_b_map,start,goal,path,show_animation):
    # visual_b_map[start[0]][start[1]] = start_color
    # visual_b_map[goal[0]][goal[1]] = goal_color
    for point in path:
        visual_b_map[point[0]][point[1]] = path_color

    cv2.circle(visual_b_map,(start[1],start[0]), 1, start_color, -1)
    cv2.circle(visual_b_map,(goal[1],goal[0]), 1, goal_color, -1)
    
    # visual_b_map = cv2.cvtColor(visual_b_map,cv2.COLOR_RGB2BGR)
    # cv2.namedWindow('map',cv2.WINDOW_NORMAL)
    # cv2.imshow('map',visual_b_map)
    # cv2.waitKey(0)
    # plt.gca().invert_yaxis()
    ax = plt.gca()
    plt.imshow(visual_b_map)
    if(not show_animation):
        ax.set_ylim(ax.get_ylim()[::-1])
    # plt.show()
    plt.pause(5)
    plt.close()


def is_valid(b_map, state,resolution,length,breadth):
    if(state[0]<0 or state[0]>(breadth-1) or state[1]<0 or state[1]>(length-1)):
        print('\n\tState is invalid. Out of bounds')
        print('\tEnter state as "row,col" where row is in range [0,'+str(breadth-1)+'] col is in range [0,'+str(length-1)+'].\n')
        return False
    if(b_map[state[0]][state[1]] ):
        print('\n\tStart or goal state is invalid i.e inside obstacle\n')
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
        return w_cc*(node.cost + diagonal_cost)
    else:
        #straight path i.e. left, right, top, bottom
        return w_cc*(node.cost + straight_cost)


def explore_neighbours(r,c,Q,b_map,visited_nodes,node_map,visual_b_map,start,goal,show_animation,ind,length,breadth,plot_step):
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
        if(ind%plot_step ==0):
            # print(show_animation)
            mark_start_and_goal(visual_b_map,start,goal)
            # visual_b_map = cv2.cvtColor(visual_b_map,cv2.COLOR_RGB2BGR)
            # cv2.namedWindow('map',cv2.WINDOW_NORMAL)
            # cv2.imshow('map',visual_b_map)
            # cv2.waitKey(1)
            # plt.gca().invert_yaxis()
            ax = plt.gca()
            plt.imshow(visual_b_map)
            if(ind == 0):
                ax.set_ylim(ax.get_ylim()[::-1])
            plt.show(block=False)
            plt.pause(0.1)
            # plt.close()
    
    return Q




def Dijkstra(start,goal,node_map,visited_nodes,b_map,visual_b_map,show_animation,length,breadth,plot_step):
    Q = []
    heapq.heapify(Q) 
    # if not list(Q):
    #     print('empty2')
    index = start[0]*length+start[1]
    node_map[index].cost = 0
    heapq.heappush(Q,(node_map[index].cost, node_map[index].name, node_map[index]))
    ind = 0
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
        Q = explore_neighbours(curr_r,curr_c,Q,b_map,visited_nodes,node_map,visual_b_map,start,goal,show_animation,ind,length,breadth,plot_step)
        ind+=1
    return 'Failure'




###############################################################################
########################## A* and helper functions ############################
###############################################################################

def cal_chebyshev_distance(row, col, goal):
    return max(abs(row-goal[0]),abs(col-goal[1]))

def cal_l2_dist(row, col, goal):
    ans = sqrt(((row-goal[0])**2) + ((col-goal[1])**2))
    # print('sqrt('+str(row)+'-'+str(goal[0])+')^2'+ans)
    return ans


def explore_neighbours_a_star(r,c,Q,b_map,visited_nodes,node_map,visual_b_map,start,goal,show_animation,heuristic,ind,resolution,length,breadth,plot_step):
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
        
        curr_cost = cal_cost_to_come(node_map[index], dx[i], dy[i])
        

        if(curr_cost<node_map[new_index].cost):
            #update cost of new node
            node_map[new_index].cost = curr_cost
            #update parent(name) of new node
            node_map[new_index].parent_name = node_map[index].name
            #add current node to heap
            if(heuristic=='cheby'):
                h = w*cal_chebyshev_distance(new_r, new_c, goal)
            elif(heuristic=='euc'):
                h = w*cal_l2_dist(new_r, new_c, goal)
            heapq.heappush(Q,(curr_cost+h, node_map[new_index].name, node_map[new_index]))
            # heapq.heappush(Q,(node_map[new_index].cost, node_map[new_index].name, node_map[new_index]))
            # if(show_animation):
            visual_b_map[new_r][new_c] = heap_node_color
    visited_nodes[r][c] = 1
    visual_b_map[r][c] = visited_node_color
    if(show_animation):
        if(ind%plot_step ==0):
            mark_start_and_goal(visual_b_map,start,goal)
            # cv2.namedWindow('map',cv2.WINDOW_NORMAL)
            # visual_b_map = cv2.cvtColor(visual_b_map,cv2.COLOR_RGB2BGR)
            # cv2.imshow('map',visual_b_map)
            # cv2.waitKey(1)
            # plt.gca().invert_yaxis()
            ax = plt.gca()
            plt.imshow(visual_b_map)
            if(ind == 0):
                ax.set_ylim(ax.get_ylim()[::-1])
            plt.show(block=False)
            plt.pause(0.1)
            # plt.close()
    # if(not list(Q)):
    #     print('\nlist empty\n')
    return Q




def A_star(start,goal,node_map,visited_nodes,b_map,visual_b_map,show_animation,heuristic,resolution,length,breadth,plot_step):
    Q = []
    heapq.heapify(Q) 
    # if not list(Q):
    #     print('empty2')
    index = start[0]*length+start[1]
    node_map[index].cost = 0
    heapq.heappush(Q,(node_map[index].cost, node_map[index].name, node_map[index]))
    ind =0
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
        Q = explore_neighbours_a_star(curr_r,curr_c,Q,b_map,visited_nodes,node_map,visual_b_map,start,goal,show_animation,heuristic,ind,resolution,length,breadth,plot_step)
        ind+=1
    return 'Failure'






def main():


    Parser = argparse.ArgumentParser()
    Parser.add_argument('--src', type=int, nargs="+", default= [10,10], help='location of starting point, Default: (10,10)')
    Parser.add_argument('--goal', type=int, nargs="+", default= [50,50], help='location of goal point, Default: (50,50)')
    Parser.add_argument('--animation',default='True',help="Flag to show animation;True=show animation, False = don't show animation")
    Parser.add_argument('--algo',default='a*',help="Algorithm that you want to use; a* or dij (short for dijkstra)")
    Parser.add_argument('--heuristic',default='cheby',help="heuristic used by A* algorithm; options: cheby or euc")
    Parser.add_argument('--robot',type = int,default=0,help="Type of robot(integer): 0(for point robot) or radius of robot(for circular robot)")
    Parser.add_argument('--clear',type = int,default=0,help="Clearance to maintain from obstacle(in integer)")
    Parser.add_argument('--res',type = int,default=1,help="resolution of the map. Greater the resolution lesser the number of nodes(res>=1)")

    Args = Parser.parse_args()
    
    ################# Some checks on parsed inputs ###################
    if(np.shape(Args.src)[0]>2 or np.shape(Args.src)[0]<2):
        print('Expected two values. Enter start coordintes, eg: if start location(row,col) is (10, 10) then the input format wil be\n "--src 10 10"')
        return
    if(np.shape(Args.goal)[0]>2 or np.shape(Args.goal)[0]<2):
        print('Expected two values. Enter goal coordintes, eg: if goal location(row,col) is (10, 10) then the input format wil be\n "--goal 10 10"')
        return
    
    ################# Storing parsed data ##################
    resolution = Args.res
    if(resolution<1):
        print('resolution should be greater than equal to 1')
        return
    start = Args.src
    start =  np.divide(start,resolution)
    goal = Args.goal
    goal =  np.divide(goal,resolution)
    if(start[0]==goal[0] and start[1]==goal[1]):
        print('start and goal are same. Enter different start and goal')
        return
    #dimensions of map
    length = 251/resolution
    breadth = 151/resolution
    #Number of iteration after which we plot the progress map
    plot_step = int(sqrt((start[0]-goal[0])**2+(start[1]-goal[1])**2))*3
    print(plot_step)

    print('start location = '+str(start))
    print('goal location = '+str(goal))
    start[0],start[1] = start[1],start[0]
    goal[0],goal[1] = goal[1],goal[0]
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
    pad = (Args.robot+Args.clear)/resolution
    # print(pad)
    # print(algo)

    
    ################### Code starts here ###################
    

    # create obstacle map
    b_map = generate_map(breadth,length,pad,resolution)

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

    if(not is_valid(b_map, start,resolution,length,breadth)):
        return
    # checking validity of goal node
    if (not is_valid(b_map, goal,resolution,length,breadth)): 
        return
    start_time=time.time()
    #Start solving 
    if(algo.lower()=='dij'):
        ans = Dijkstra(start,goal,node_map,visited_nodes,b_map,visual_b_map,show_animation,length,breadth,plot_step)
        end_time=time.time()
        print('Time to obtain solution = '+str(end_time - start_time))
    elif(algo.lower()=='a*'):
        ans = A_star(start,goal,node_map,visited_nodes,b_map,visual_b_map,show_animation, heuristic,resolution,length,breadth,plot_step)
        end_time=time.time()
        print('Time to obtain solution = '+str(end_time - start_time))

    else:
        print('Unable to recognize the input algorithm. exiting...')
        print('Enter either "a*" or "dij" (short for dijkstra)')
        return
    # print('ans = '+ ans)
    if(ans=='Success'):
        path = []
        index = goal[0]*length+goal[1]
        while(not node_map[index].loc==tuple(start)):
            # print(index)
            path.append(np.asarray(node_map[index].loc))
            index = node_map[index].parent_name-1
        visualize_path(visual_b_map,start,goal,path[1:],show_animation)
        # cv2.destroyAllWindows()
    else:
        print('\n\tNo path found. Goal cannot be reached\n')
        # path = []
        # index = goal[0]*length+goal[1]
        # while(not node_map[index].loc==tuple(start)):
        #     # print(index)
        #     path.append(np.asarray(node_map[index].loc))
        #     print('2',node_map[index].parent_name)
        #     index = node_map[index].parent_name-1
        # if(path):
        #     visualize_path(visual_b_map,start,goal,path[1:],show_animation)
        # else:
        #     print('matplotlib visualization error')





if __name__ == '__main__':
	main()
