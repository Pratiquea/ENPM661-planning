#!/usr/bin/env python

import numpy as np
import cv2
import glob
import random
import matplotlib.pyplot as plt
import time
import math
import argparse
import heapq
import copy
from math import sqrt
import os
import sys
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


#Global variables
resolution = 1
poww=(10**2)


heap_node_color = [220,20,60]
visited_node_color = [255,219,28]
path_color = [50,90,255]
goal_color = (0,190,255)
start_color = (255,0,0)
#weight multiplier for heuristic
w = 1
w_cc = 1
robot_rad = .177 #in m


class node:
    def __init__(self,x,y,theta, parent_name=None,g=float("inf"),h=float("inf")):
        self.x = x
        self.y = y
        self.name = 'x' + str(int(self.x)) + 'y' + str(int(self.y))
        self.theta = theta
        self.parent_name = parent_name
        self.g = np.round(g,2)
        self.h = np.round(h,2)



###############################################################################
#################  Helper function to generate map using     ##################
#################  half planes and semi-algebraic equations  ##################
###############################################################################
def inside_pillars(x,y,pad):
    if(not pad):
        if( (x-3.9*poww)**2+(y-9.65*poww)**2- (0.405*poww)**2<=0 or
            (x-4.38*poww)**2+(y-7.36*poww)**2- (0.405*poww)**2<=0 or
            (x-4.38*poww)**2+(y-2.74*poww)**2- (0.405*poww)**2<=0 or
            (x-3.9*poww)**2+(y-0.45*poww)**2- (0.405*poww)**2<=0):
            return True
        else:
            return False
    else:
        if( (x-3.9*poww)**2+(y-9.65*poww)**2- ((0.405+pad)*poww)**2<=0 or
            (x-4.38*poww)**2+(y-7.36*poww)**2- ((0.405+pad)*poww)**2<=0 or
            (x-4.38*poww)**2+(y-2.74*poww)**2- ((0.405+pad)*poww)**2<=0 or
            (x-3.9*poww)**2+(y-0.45*poww)**2- ((0.405+pad)*poww)**2<=0):
            return True
        else:
            return False


def inside_round_table(x,y,pad):
    semi1=False
    semi2=False
    rect = False
    if(not pad):
        if x-3.097*poww>=0:
            if( (x-3.097*poww)**2 + (y-8.3*poww)**2 - (0.7995*poww)**2 <=0 ):
                semi2=True
        if x-1.4995*poww<=0:
            if( (x-1.4995*poww)**2 + (y-8.3*poww)**2 - (0.7995*poww)**2 <=0 ):
                semi1=True
        if( y-9.1*poww<=0 and y-7.501*poww>=0 and x-3.0973*poww<=0 and x-1.4995*poww>=0 ):
            rect=True
        if(semi2 or semi1 or rect):
            return True
        else:
            return False
    else:
        if x-3.097*poww>=0:
            if( (x-3.097*poww)**2 + (y-8.3*poww)**2 - ((0.7995+pad)*poww)**2 <=0 ):
                semi2=True
        if x-1.4995*poww<=0:
            if( (x-1.4995*poww)**2 + (y-8.3*poww)**2 - ((0.7995+pad)*poww)**2 <=0 ):
                semi1=True
        if( y-(9.1+pad)*poww<=0 and y-(7.501-pad)*poww>=0 and x-3.0973*poww<=0 and x-1.4995*poww>=0 ):
            rect=True
        if(semi2 or semi1 or rect):
            return True
        else:
            return False
        

def inside_first_quad(x,y,pad):
    if(x-(8.32-pad)*poww>=0 and x-(9.18+pad)*poww<=0 and y-(10.1+pad)*poww<=0 and y-(8.27-pad)*poww>=0 or
       x-(9.83-pad)*poww>=0 and x-(10.26+pad)*poww<=0 and y-(10.1+pad)*poww<=0 and y-(9.19-pad)*poww>=0 or
       x-(7.44-pad)*poww>=0 and x-(11.1+pad)*poww<=0 and y-(6.97+pad)*poww<=0 and y-(6.21-pad)*poww>=0 or
       x-(10.52-pad)*poww>=0 and x-(11.1+pad)*poww<=0 and y-(5.655+pad)*poww<=0 and y-(4.485-pad)*poww>=0 ):
        return True
    else:
        return False


def inside_fourth_quad(x,y,pad):
    if(x-(10.19-pad)*poww>=0 and x-(11.1+pad)*poww<=0 and y-(4.48+pad)*poww<=0 and y-(3.62-pad)*poww>=0 or
       x-(10.52-pad)*poww>=0 and x-(11.1+pad)*poww<=0 and y-(2.9525+pad)*poww<=0 and y-(1.7825-pad)*poww>=0 or
       x-(9.27-pad)*poww>=0 and x-(11.1+pad)*poww<=0 and y-(1.11+pad)*poww<=0 and y-(0.35-pad)*poww>=0 or
       x-(6.28-pad)*poww>=0 and x-(11.1+pad)*poww<=0 and y-(0.35+pad)*poww<=0 and y-(0.0-pad)*poww>=0 or
       x-(7.845-pad)*poww>=0 and x-(9.365+pad)*poww<=0 and y-(4.07+pad)*poww<=0 and y-(2.9-pad)*poww>=0 or
       x-(7.79-pad)*poww>=0 and x-(8.96+pad)*poww<=0 and y-(0.93+pad)*poww<=0 and y-(0.35-pad)*poww>=0):
        return True
    else:
        return False


def inside_third_quad(x,y,pad):
    if(x-(4.38-pad)*poww>=0 and x-(5.29+pad)*poww<=0 and y-(4.98+pad)*poww<=0 and y-(3.15-pad)*poww>=0 or
       x-(5.29-pad)*poww>=0 and x-(7.12+pad)*poww<=0 and y-(3.41+pad)*poww<=0 and y-(2.65-pad)*poww>=0 or
       x-(4.74-pad)*poww>=0 and x-(7.48+pad)*poww<=0 and y-(1.87+pad)*poww<=0 and y-(0.35-pad)*poww>=0):
        return True
    else:
        return False


def pad_wall(x,y,pad):
    if(pad):
        if(x-(pad)*poww<=0 or x-(11.1-pad)*poww>=0 or y-(10.1-pad)*poww>=0 or y-(0+pad)*poww<=0):
            return True
        else:
            return False
    else:
        return False

###############################################################################
#################### Main function to generate binary map #####################
###############################################################################

def generate_map(breadth,length,pad,resolution):
    map = np.zeros((int(breadth),int(length)))
    for i in range(0,int(breadth)):
        for j in range(0,int(length)):
            if(inside_round_table(j,i,pad) or inside_pillars(j,i,pad) or
               inside_first_quad(j,i,pad) or inside_fourth_quad(j,i,pad) or
               inside_third_quad(j,i,pad) or pad_wall(j,i,pad)):
                map[i][j]=True
    # plt.imshow(map,cmap="binary")
    # plt.show()
    return map


###############################################################################
######################### functions for visualization ##########################
###############################################################################




def visualize_path(visual_b_map,start,goal,path,show_animation):
    # visual_b_map[start[0]][start[1]] = start_color
    # visual_b_map[goal[0]][goal[1]] = goal_color
    for point in path:
        # print point
        # visual_b_map[int(point[0])][int(point[1])] = path_color
        cv2.circle(visual_b_map,(int(point[1]),int(point[0])), 5, path_color, -1)


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
    plt.pause(2)
    plt.close()


def create_visual_map(b_map):
    temp = copy.deepcopy(b_map)
    temp[temp==0]=255
    # temp = np.asarray(temp,dtype=int)
    visual_b_map = temp
    visual_b_map = np.dstack((visual_b_map,temp,temp))
    visual_b_map = np.ascontiguousarray(visual_b_map, dtype=np.uint8)
    return visual_b_map


###############################################################################
########################## A* and helper functions ############################
###############################################################################

def is_valid(b_map, state,resolution,length,breadth):
    if(state[0]<0 or state[0]>(breadth-1) or state[1]<0 or state[1]>(length-1)):
        print('\n\tState is invalid. Out of bounds')
        print('\tEnter state as "row,col" where row is in range [0,'+str(breadth-1)+'] col is in range [0,'+str(length-1)+'].\n')
        return False
    if(b_map[state[0]][state[1]] ):
        print('\n\tStart or goal state is invalid i.e inside obstacle\n')
        return False
    return True


class Plan:

    # Wheel Radius (cm)
    wr = 3.80
    # Wheel Base length (cm)
    wb = 24.00
    # left wheel velocity in (rev/min)
    rpm1 = 50.0
    # left wheel velocity in (rev/min)
    rpm2 = 100.0;
    # Motion model based on wheel RPM
    du = [ 0,   rpm1, rpm1, 0,   rpm2, rpm2, rpm1, rpm2]
    dv = [ rpm1, 0,   rpm1, rpm2, 0,   rpm2, rpm2, rpm1]

    # sampling frequency in Hz
    f=1.0
    # robot radius in cm
    rr = 17.7
    # raduis of circle around goal. If robot is in this circle goal is considered reached
    goal_pad = 20.0
    #color codes
    resolution = 1.0
    # some params
    poww=(10**2)
    length = float(11.1*poww/resolution)
    breadth = float(10.1*poww/resolution)

    def __init__(self,start_node,goal_node,heuristic,b_map,visual_b_map,show_animation):
        self.start_node = start_node
        self.goal_node = goal_node
        self.b_map = b_map
        self.visual_b_map = visual_b_map
        # Number of iteration after which we plot the progress map
        self.plot_step = int(sqrt((start_node.x-goal_node.x)**2+(start_node.y-goal_node.y)**2))*3
        self.show_animation = show_animation
        self.heuristic = heuristic
        


    def mark_start_and_goal(self,start,goal):
        cv2.circle(self.visual_b_map,(start[1],start[0]), 10, start_color, -1)
        cv2.circle(self.visual_b_map,(goal[1],goal[0]), 10, goal_color, -1)
        # return selfvisual_b_map



    def goal_reached(self, curr_node):
        # check if current node has reached goal or is in proximity
        return math.sqrt((curr_node.x - self.goal_node.x)**2 + (curr_node.y - self.goal_node.y)**2) - self.goal_pad<0
    
    def next_node(self,curr_node,ind):
        # print ind
        aux_du = self.du[ind]
        aux_dv = self.dv[ind]
        # print aux_dv,aux_du
        # converting rev/min to rad/sec
        ul = 2*math.pi*aux_du/60
        ur = 2*math.pi*aux_dv/60
        # print(ur)
        #  Defining multiplier constants
        k1 = (self.wr/2) * (ul + ur)
        k2 = (self.wr/self.wb) * (ur - ul) 
        # print(k1,k2)
        dtheta = k2 * (180/math.pi) * (1/self.f)
        #  limiting dtheta to [0,360]
        dtheta = dtheta - (np.round(dtheta/360, 0)*360)
        # Converting dtheta range from [0,360] to [-180,180]
        if dtheta > 180:
            dtheta = dtheta - 360
        elif dtheta < -180:
            dtheta = dtheta + 360
        # Updating value of theta 
        theta = dtheta + curr_node.theta
        #  limiting dtheta to [-180,180]
        if theta > 180:
            theta = theta - 360
        elif theta <= -180:
            theta = theta + 360

        if k2 == 0:
            dx = k1 * math.cos(curr_node.theta*(math.pi/180)) * (1/self.f)
            dy = k1 * math.sin(curr_node.theta*(math.pi/180)) * (1/self.f)
        else:
            dx = (k1/k2) * (math.sin(theta*(math.pi/180)) - math.sin(curr_node.theta*(math.pi/180)))
            dy = -(k1/k2) * (math.cos(theta*(math.pi/180)) - math.cos(curr_node.theta*(math.pi/180)))
        # dx = (self.wr/2) * (ul + ur) 
        
        # print("x,y = "+str(dx)+' '+str(dy))

        # updating location
        x = np.round(dx/self.resolution,0)*self.resolution + curr_node.x
        y = np.round(dy/self.resolution,0)*self.resolution + curr_node.y
        # print("x,y = "+str(x)+' '+str(y))
        # Updating cost to go
        g = curr_node.g + (k1 * (1/self.f))
        # updating heuristic cost
        h = 10*math.sqrt((self.goal_node.x - x)**2 + (self.goal_node.y - y)**2)+np.absolute(theta - self.goal_node.theta)/180.0
        # (name,loc,theta, parent_name=None,g=float("inf"),h=float("inf"))
        n = node(x,y,theta,curr_node.name,g,h)
        return n

    # def cal_cost_to_come(self,parent_node,dx,dy):
    #     new_cost  = np.sqrt(dx**2+dy**2)
    #     total_cost = (parent_node.g + new_cost)# node.g
    #     return 

    # these functions aren't being used right now. Only for testing purpose
    def cal_chebyshev_distance(self,row, col):
        return max(abs(row-self.goal[0]),abs(col-self.goal[1]))

    def cal_l2_dist(self,row, col):
        ans = sqrt(((row-self.goal[0])**2) + ((col-self.goal[1])**2))
        # print('sqrt('+str(row)+'-'+str(goal[0])+')^2'+ans)
        return ans

    def generate_path(self,visited_nodes):
        goal_id = self.goal_node.name
        path = []
        path_nodes = []
        # print visited_nodes
        while(goal_id!=self.start_node.name):
            # print('path element = '+str(goal_id))
            node_curr = visited_nodes[goal_id]
            path_nodes.append(node_curr)
            path.append(np.array([node_curr.x,node_curr.y]))
            goal_id = node_curr.parent_name

        return path,path_nodes

    def explore_neighbours_a_star(self,cnode,Q,visited_nodes,open_nodes,ind):
        for i in range(0,8):
            # new_c = cnode.x+dx[i]
            # new_r = cnode.y+dy[i]

            # creating new child node
            child_node = self.next_node(cnode,i)
            
            # getting location of child node
            new_c = child_node.x
            new_r = child_node.y
            # print new_r,new_c
            #check if the location of new node is within bounds
            if(new_c<0 or new_r<0 or new_c>=self.length or new_r>=self.breadth):
                continue

            #check if state is valid i.e not an obstacle
            if(self.b_map[int(new_c)][int(new_r)]):
                continue

            #check is the node is visited
            if(child_node.name in visited_nodes):
                # self.visual_b_map[child_node.x][child_node.y] = visited_node_color
                # print('a')
                continue
            elif(child_node.name in open_nodes):
                # print open_nodes
                # print('b')

                # new cost to come for child node 
                new_g = child_node.g
                # current cost to come for child node
                curr_g = open_nodes[child_node.name].g

                if(new_g<curr_g):
                    # #update cost of new node
                    # open_nodes[child_node.name].g= child_node.g
                    # #update parent(name) of new node
                    # open_nodes[child_node.name].parent_name = child_node.parent_name

                    #update child node in dict of open node
                    open_nodes[child_node.name]= child_node

                    #add current node to heap
                    # if(heuristic=='cheby'):
                    #     h = w*cal_chebyshev_distance(new_r, new_c)
                    # elif(heuristic=='euc'):
                    #     h = w*cal_l2_dist(new_r, new_c)
                    # node_map[new_index].h = h

                    heapq.heappush(Q,(child_node.h+child_node.g,child_node))
                    # heapq.heappush(Q,(node_map[new_index].cost, node_map[new_index].name, node_map[new_index]))
                    # if(show_animation):
                    self.visual_b_map[int(child_node.x)][int(child_node.y)] = heap_node_color
                else:
                    continue

            else:
                # print('c')
                open_nodes[child_node.name] = child_node
                # print(child_node.name)
                heapq.heappush(Q,(child_node.h+child_node.g,child_node))
                # print('inside = ', Q)

            # visited_nodes[r][c] = 1
            # visual_b_map[r][c] = visited_node_color
            if(self.show_animation):
                if(ind%self.plot_step ==0):
                    start = np.array([self.start_node.x,self.start_node.y])
                    goal = np.array([self.goal_node.x,self.goal_node.y])
                    self.mark_start_and_goal(start,goal)
                    # cv2.namedWindow('map',cv2.WINDOW_NORMAL)
                    # visual_b_map = cv2.cvtColor(visual_b_map,cv2.COLOR_RGB2BGR)
                    # cv2.imshow('map',visual_b_map)
                    # cv2.waitKey(1)
                    # plt.gca().invert_yaxis()
                    ax = plt.gca()
                    plt.imshow(self.visual_b_map)
                    if(ind == 0):
                        ax.set_ylim(ax.get_ylim()[::-1])
                    plt.show(block=False)
                    plt.pause(0.1)
            # new_index = new_r*length+new_c
            # index = r*length+c
            
                # plt.close()
        # if(not list(Q)):
        #     print('\nlist empty\n')
        return Q




    def A_star(self):
        # initializing Queue, open node list and visited node list
        Q = []
        open_nodes = dict()
        visited_nodes = dict()
        
        # draw start and goal points on map
        start = np.array([self.start_node.x,self.start_node.y])
        print('\n')
        # print('start loc ',start[0],start[1])
        goal = np.array([self.goal_node.x,self.goal_node.y])
        self.mark_start_and_goal(start,goal)
        
        # initiate heap
        heapq.heapify(Q)
        # print Q

        # adding the start node to opened list
        open_nodes[self.start_node.name] = self.start_node

        # adding start node to heap
        heapq.heappush(Q,(self.start_node.g+self.start_node.h, self.start_node))  #first elem after Q is node.g
        ind =0
        path = []
        while(list(Q)):
            idd, current_node = heapq.heappop(Q)
            #print(current_node[0],current_node[1],current_node[2])
            curr_r = current_node.y
            curr_c = current_node.x
            #check if the goal is reached
            if(self.goal_reached(current_node)):
                # print("goal check")
                print('\tSuccess. Path found\n')
                self.goal_node.parent_name = current_node.parent_name
                self.goal_node.x = current_node.x
                self.goal_node.y = current_node.y
                self.goal_node.theta = current_node.theta
                self.g = current_node.g
                self.h = current_node.h
                self.goal_node.name = current_node.name
                visited_nodes[current_node.name] = current_node
                # print("Goal: ",str())
                path,path_nodes = self.generate_path(visited_nodes)
                return path,path_nodes,self.visual_b_map

            #check node for duplicity in heap
            if(current_node.name in visited_nodes):
                # print('skipping')
                continue
            else:
                # print('searching')
                visited_nodes[current_node.name] = current_node
                # open_nodes.pop[]
                #update heap after exploring neighbours
                Q = self.explore_neighbours_a_star(current_node,Q,visited_nodes,open_nodes,ind)

            # print('Q = ',Q)
            # print ind
            ind+=1
        return path,path_nodes,self.visual_b_map





def simulate(path_list,path_nodes):
    print("Simulation started")
    rospy.init_node('Motion_command',anonymous=True)
    vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
    # pos_pub = rospy.Publisher('/odom', Odometry, queue_size=1000)

    vel_msg = Twist()

    # pos_pub.publish(pos_msg)
    r = rospy.Rate(0.5)
    # dt = 2
    prev_x=0
    prev_y=0
    prev_theta=0
    for i,node in enumerate(path_nodes):
        # node = path_list[p]
        v = math.sqrt((path_list[i][0] - prev_x)**2 + (path_list[i][1]- prev_y)**2)
        w = node.theta - prev_theta
        w=math.radians(w)
        vel_msg.linear.x = v/100
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = w
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            t1 = rospy.Time.now().to_sec()
            elapsed = t1 - t0
            print("elapsed: ", elapsed)
            if elapsed >= 1:
                break
            vel_pub.publish(vel_msg)
            print("published velocity: ",vel_msg.linear.x)
            r.sleep()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        vel_pub.publish(vel_msg)
        prev_x = path_list[i][0]
        prev_y = path_list[i][1]
        prev_theta = node.theta
    rospy.sleep(r)




def main():
    Parser = argparse.ArgumentParser()
    Parser.add_argument('--src', type=int, nargs="+", default= [30,30], help='location of starting point, Default: (30,30)')
    Parser.add_argument('--goal', type=int, nargs="+", default= [600,600], help='location of goal point, Default: (600,800)')
    Parser.add_argument('--animation',default='False',help="Flag to show animation;True=show animation, False = don't show animation")
    Parser.add_argument('--heuristic',default='cheby',help="heuristic used by A* algorithm; options: cheby or euc")
    Parser.add_argument('--clear',type = float,default=0.1,help="Clearance to maintain from obstacle(in meter)")
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
    animation = Args.animation
    animation = False
    if(start[0]==goal[0] and start[1]==goal[1]):
        print('start and goal are same. Enter different start and goal')
        return
    start[0],start[1] = start[1],start[0]
    goal[0],goal[1] = goal[1],goal[0]
    # pad= 0.2
    pad = (robot_rad+Args.clear)/resolution
    # print pad
    heuristic = Args.heuristic

    length = 11.1*poww/resolution
    breadth = 10.1*poww/resolution
    ################### Code starts here ###################
    exists = os.path.isfile('./bmap.npz')
    if(exists):
        b_map = np.load('bmap.npz')

    else:
        # create obstacle map
        b_map = generate_map(breadth,length,pad,resolution)
        np.save('bmap',b_map)
        # print('binary map created')
    # b_map = generate_map(breadth,length,pad,resolution)
    
    exists = os.path.isfile('./vmap.npz')
    if(exists):
        visual_b_map = np.load('vmap.npz')
    else:
        #create obstacle map for visualization
        visual_b_map = create_visual_map(b_map)
        np.save('vmap',visual_b_map)



    # checking validity of start node
    if(not is_valid(b_map, start,resolution,length,breadth)):
        return
    # checking validity of goal node
    if (not is_valid(b_map, goal,resolution,length,breadth)): 
        return 


    #mark start and goal nodes
    # mark_start_and_goal(visual_b_map,start,goal)
    # node(x,y,theta, parent_name=None,g=float("inf"),h=float("inf"))
    start_node = node(start[0],start[1],0,None,0,math.sqrt( (start[0]-goal[0])**2+(start[1]-goal[1])**2 ))
    goal_node = node(goal[0],goal[1],0,None,float("inf"),0)

    # Start planning
    plan = Plan(start_node,goal_node,heuristic,b_map,visual_b_map,animation)

    path,path_nodes,visual_b_map = plan.A_star()
    path.reverse()
    if(len(path)):
        print('')
        # print('path found')
    else:
        print('No path found')
    visualize_path(visual_b_map,start,goal,path,animation)

    # print('end')
    path = np.array(path)
    # print(path[0])
    path_list = path-np.array([505,555])
    print(len(path_list))
    simulate(path,path_nodes)
    # plt.rcParams["figure.figsize"] = (10,10)

    # ax = plt.gca()
    # plt.imshow(b_map)#,cmap='binary'
    # ax.set_ylim(ax.get_ylim()[::-1])
    # plt.show()

if __name__ == '__main__':
	main()