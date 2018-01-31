from queue import PriorityQueue
from math import *
import heapq

def shortest_path(M,start,goal):
    
    frontier = PriorityQueue()  
    frontier.put(start,0)    
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0     # first cost from start to start is 0
    
    fail = 0
    
    while not frontier.empty():
        current = frontier.get()
    
        if (current == goal):     
            return reconstruct_path_non(came_from,start,current)
        
        #Where A* happens...
        for next in M.roads[current]:
            new_cost = cost_so_far[current] + distance_between(M,current,next); # your G  
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                                                                                        
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic_cost_estimate(M, goal, next) #Your F 
                frontier.put(next,priority)
                came_from[next] = current
                
                
    return fail
    print("This should not print")
  
#Gives incorrect path....

def reconstruct_path_non(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path


def heuristic_cost_estimate(M, a, goal):  #THE HURISTIC..ALSO DISTANCE FORMULA
    goal = M.intersections[goal]
    h = sqrt(abs(M.intersections[a][0] - goal[0])**2 + abs(M.intersections[a][1] - goal[1])**2)
    return h

def distance_between(M, a, b):  #THE DISTANCE FORMULA 
    d = sqrt(abs(M.intersections[a][0] - M.intersections[b][0])**2 + abs(M.intersections[a][1] - M.intersections[b][1])**2)
    return d
    

print("shortest path called")
