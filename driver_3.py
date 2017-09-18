# -*- coding: utf-8 -*-
"""
Created on Thu Feb  9 11:47:15 2017

@author: Juan E
"""
import sys
import numpy as np
import math
from collections import deque
import heapq
import os
import time

try:
    import psutil
    process = psutil.Process(os.getpid())
    print("starting process " + str(process))
    getMemUsage = lambda:process.memory_info().rss
except ImportError:
    import resource
    getMemUsage = lambda:resource.getrusage(resource.RUSAGE_SELF).ru_maxrss

start_time=time.time()

#Test if the user input 2 Arg values
if len(sys.argv)==3:
    user_args=sys.argv[1:]
    #Set method and initial_state with user argv
    method,initial_state=user_args #Guarda el primer argumento

#Default method if no input arg
else:
    method="ida"
    initial_state="6,2,5,8,7,4,1,0,3"
#Remove the "," from the list         
initial_state=initial_state.split(",")
#Transform into integers
initial_state=list(map(int,initial_state))
#Transforms into array
initial_state=np.asarray(initial_state)
#Get the n size of the array by taking de sqrt
n=int(math.sqrt(len(initial_state)))
#Reshape the initial_state into a n x n matrix
initial_state=initial_state.reshape(n,n)
#Creat the goal_state as a n x n matrix
goal_state=np.arange(n*n).reshape(n,n)
#initialize a combined explored and frontier dictionay
explored_or_frontier=set()
#Global variable for max_search_depth
max_search_depth=0



def bfs(start,goal):
    """Search the shallowest nodes in the search tree first."""
    global explored_or_frontier
    frontier=deque()
    frontier.append(Node(start,None,None,0,h(start,goal)))
    nodes_expanded = 0
    max_fringe_size = 0
    search_depth = 0
    cost = 0
    while len(frontier)>0:
        # take the node from the front of the queue
        current_node=frontier.popleft()
        explored_or_frontier.add(str(current_node.state))
#        print(current_node.state)
#        input("Press Enter to continue...")
        # Append the move we made to moves
		 # if this node is the goal, return the moves it took to get here.
        if np.array_equal(current_node.state,goal):
            search_depth=current_node.depth
            moves,cost=path_cost(current_node)
            return (moves,cost,nodes_expanded,len(frontier),max_fringe_size,
                    search_depth)
        #Expand the Frontier
        frontier.extend(expand_node(current_node))
        nodes_expanded+=1
        if len(frontier)>max_fringe_size:
            max_fringe_size=len(frontier)
    return (None,cost,nodes_expanded,len(frontier),max_fringe_size,search_depth)

def dfs(start,goal):
    """Search the shallowest nodes in the search tree first."""
    global explored_or_frontier
    frontier=deque()
    frontier.append(Node(start,None,None,0,h(start,goal)))
    nodes_expanded = 0
    max_fringe_size = 0
    search_depth = 0
    cost = 0
    rv_expanded_node=deque()
    while len(frontier)>0:
        # take the node from the front of the queue
        current_node=frontier.pop()
        explored_or_frontier.add(str(current_node.state))
        #print(current_node.state)
        #input("Press Enter to continue...")
        # Append the move we made to moves
		 # if this node is the goal, return the moves it took to get here.
        if np.array_equal(current_node.state,goal):
            search_depth=current_node.depth
            moves,cost=path_cost(current_node)
            return (moves,cost,nodes_expanded,len(frontier),max_fringe_size,
                    search_depth)
        #Expand the Frontier
        rv_expanded_node=expand_node(current_node)
        rv_expanded_node.reverse()
        frontier.extend(rv_expanded_node)
        nodes_expanded+=1
        if len(frontier)>max_fringe_size:
            max_fringe_size=len(frontier)
    return (None,cost,nodes_expanded,len(frontier),max_fringe_size,search_depth)

def ast(start,goal):
    """Search the shallowest nodes in the search tree first."""
    global explored_or_frontier
    frontier=[]
    heapq.heappush(frontier,(h(start,goal),0,Node(start,None,None,0,h(start,goal))))
    nodes_expanded = 0
    max_fringe_size = 0
    search_depth = 0
    cost = 0
    rv_expanded_node=deque()
    while len(frontier)>0:
        # take the node from the front of the queue
        frontier=sorted(frontier)
        h_cost, priority, current_node=heapq.heappop(frontier)
        explored_or_frontier.add(str(current_node.state))
        #print(current_node.state)
        #input("Press Enter to continue...")
        # Append the move we made to moves
		 # if this node is the goal, return the moves it took to get here.
        if np.array_equal(current_node.state,goal):
            search_depth=current_node.depth
            moves,cost=path_cost(current_node)
            return (moves,cost,nodes_expanded,len(frontier),max_fringe_size,
                    search_depth)
        #Expand the Frontier
        rv_expanded_node=expand_node(current_node)
        
        for node in rv_expanded_node:
            heapq.heappush(frontier,(node.cost+node.depth,node.priority,node))
            
        nodes_expanded+=1
        if len(frontier)>max_fringe_size:
            max_fringe_size=len(frontier)
    return (None,cost,nodes_expanded,len(frontier),max_fringe_size,search_depth) 

def ida(start,goal):
    """Search the shallowest nodes in the search tree first."""
    global explored_or_frontier
    limit = h(start,goal)

    
    while limit>=h(start,goal):
        explored_or_frontier=set()
        frontier=deque()
        frontier.append(Node(start,None,None,0,h(start,goal)))
        nodes_expanded = 0
        max_fringe_size = 0
        search_depth = 0
        cost = 0
        rv_expanded_node=deque()
        list_limit=[]
        while len(frontier)>0:
            # take the node from the front of the queue
            current_node=frontier.pop()
            explored_or_frontier.add(str(current_node.state))
            #print(current_node.state)
            #input("Press Enter to continue...")
            # Append the move we made to moves
    		 # if this node is the goal, return the moves it took to get here.
            if np.array_equal(current_node.state,goal):
                search_depth=current_node.depth
                moves,cost=path_cost(current_node)
                return (moves,cost,nodes_expanded,len(frontier),max_fringe_size,
                        search_depth)
            #Expand the Frontier 
            rv_expanded_node=expand_node(current_node)
            rv_expanded_node.reverse()
            for node in rv_expanded_node:
                if node.cost+node.depth<=limit:
                    frontier.append(node)
                if node.cost+node.depth>limit:
                    list_limit.append(node.cost+node.depth)
            nodes_expanded+=1
            if len(frontier)>max_fringe_size:
                max_fringe_size=len(frontier)


        limit=min(list_limit)    
                    
    return (None,cost,nodes_expanded,len(frontier),max_fringe_size,search_depth)                

    
    
    # Node data structure
class Node:
    def __init__( self, state, parent, operator, depth, cost ):
        # Contains the state of the node
        self.state = state
        # Contains the node that generated this node
        self.parent = parent
		 # Contains the operation that generated this node from the parent
        self.operator = operator
		 # Contains the depth of this node (parent.depth +1)
        self.depth = depth
		 # Contains the path cost of this node from depth 0. Not used for depth/breadth first.
        self.cost = cost
        if operator == None:
            self.priority = 0
        elif operator == "Up":
            self.priority = 1
        elif operator == "Down":
            self.priority = 2
        elif operator == "Left":
            self.priority = 3
        elif operator == "Rigth":
            self.priority = 4
            
            
    def __lt__(self, other):
        return self.depth + self.cost < other.depth + other .cost
    
    
   

def expand_node(node):
    #Creates a temporal node testing an "UDLR" move , then check for validity
    #of the move and finaly compare if the new state was already in the frontier
    #or explored dictionary
    global explored_or_frontier
    global max_search_depth
    global goal_state
    expanded_nodes=[]

    x=Node(move_up( node.state ), node, "Up", node.depth + 1, 0 ) 
    if x.state is not None:
        if str(x.state) not in explored_or_frontier:
            x.cost=h(x.state,goal_state)
            expanded_nodes.append(x)
            explored_or_frontier.add(str(x.state))
    x=Node( move_down( node.state ), node, "Down", node.depth + 1, 0 ) 
    if x.state is not None:
        if str(x.state) not in explored_or_frontier:
            x.cost=h(x.state,goal_state)
            expanded_nodes.append(x)
            explored_or_frontier.add(str(x.state))
    x=Node( move_left( node.state ), node, "Left", node.depth + 1, 0 )
    if x.state is not None:
        if str(x.state) not in explored_or_frontier:
            x.cost=h(x.state,goal_state)
            expanded_nodes.append(x)
            explored_or_frontier.add(str(x.state))
    x= Node( move_right( node.state), node, "Rigth", node.depth + 1, 0 ) 
    if x.state is not None:
        if str(x.state) not in explored_or_frontier:
            x.cost=h(x.state,goal_state)
            expanded_nodes.append(x)
            explored_or_frontier.add(str(x.state))
    for node in expanded_nodes:
        if node.depth>max_search_depth:
            max_search_depth=node.depth
                            
                                
                                 
    return expanded_nodes

def move_up(state):
    new_state=np.copy(state)
    row,col=np.where(new_state==0)
    if row > 0:
        #Swap values
        temp=new_state[row-1,col]
        new_state[row-1,col]=new_state[row,col]
        new_state[row,col]=temp
        return new_state
    else:
        #can't move, return None
        return None
    
def move_down(state):
   new_state=np.copy(state)
   row,col=np.where(new_state==0)
   if row < (n-1):
        #Swap values
        temp=new_state[row+1,col]
        new_state[row+1,col]=new_state[row,col]
        new_state[row,col]=temp
        return new_state
   else:
        #can't move, return None
        return None

def move_left(state):
    new_state=np.copy(state)
    row,col=np.where(new_state==0)
    if col > 0:
        #Swap values
        temp=new_state[row,col-1]
        new_state[row,col-1]=new_state[row,col]
        new_state[row,col]=temp
        return new_state
    else:
        #can't move, return None
        return None 
    
def move_right(state):
    new_state=np.copy(state)
    row,col=np.where(new_state==0)
    if col < (n-1):
        #Swap values
        temp=new_state[row,col+1]
        new_state[row,col+1]=new_state[row,col]
        new_state[row,col]=temp
        return new_state
    else:
        #can't move, return None
        return None
#Return de moves and path cost to get from node to root    
def path_cost(node):
    cost=0
    moves=[]
    temp=node
    while temp.depth>0:
        moves.insert(0,temp.operator)
        cost+=1
        temp=temp.parent
    return moves,cost

def h(state,goal):
    h_cost=0
    for x in range(1,n*n):            
        row_node,col_node=np.where(state==x)
        row_goal,col_goal=np.where(goal==x)
        h_cost += abs(row_node-row_goal)+abs(col_node-col_goal)
    return int(h_cost)
        
    
    



# Main method
def main():
    
    
    if method=="bfs":
        moves,cost,nodes_expanded,fringe_size,max_fringe_size,search_depth=bfs(initial_state,goal_state)
    elif method=="dfs":
        moves,cost,nodes_expanded,fringe_size,max_fringe_size,search_depth=dfs(initial_state,goal_state)
    elif method=="ast":
        moves,cost,nodes_expanded,fringe_size,max_fringe_size,search_depth=ast(initial_state,goal_state)
    elif method=="ida":
        moves,cost,nodes_expanded,fringe_size,max_fringe_size,search_depth=ida(initial_state,goal_state)
    

    print("path_to_goal: ",moves)
    print("cost_of_path: ",cost)
    print("nodes_expanded: ",nodes_expanded)
    print("fringe_size: ",fringe_size)
    print("max_fringe_size: ", max_fringe_size)
    print("search_depth: ",search_depth)
    print("max_search_depth: ",max_search_depth)
    print("running_time: ", time.time()-start_time)
    print("max_ram_usage: ",getMemUsage()/2.**20)
    
    f = open('output.txt', 'w')

    f.write("path_to_goal: " + str(moves) + "\n")
    f.write("cost_of_path: " + str(cost) + "\n")
    f.write("nodes_expanded: " + str(nodes_expanded) + "\n")
    f.write("fringe_size: " + str(fringe_size) + "\n")
    f.write("max_fringe_size: " + str(max_fringe_size) + "\n")
    f.write("search_depth: " + str(search_depth) + "\n")
    f.write("max_search_depth: " + str(max_search_depth) + "\n")
    f.write("running_time: " + str(time.time()-start_time) + "\n")
    f.write("max_ram_usage: " + str(getMemUsage()/2.**20) + "\n")

    f.close()
    


if __name__ == "__main__":
	main()
