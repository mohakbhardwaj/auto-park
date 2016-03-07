#!/usr/bin/python
# ----------
# User Instructions:
# 
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal. 
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------
import sys
from heapq import heappush, heappop
"""grid = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0],
        [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        [0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0],
        [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        [0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]"""

grid = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]    #GRID 2
goal = [0, 0]
start = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one
cost_p = 2
delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

"""parking_spot_indices = [(2,3), (2,5), (2,7),(2,9), (2,11), (2,13),
                         (4,3), (4,5), (4,7),(4,9), (4,11), (4,13),
                         (7,3), (7,5), (7,7),(7,9), (7,11), (7,13), 
                         (9,3), (9,5), (9,7),(9,9), (9,11), (9,13)]""" #FOR GRID 1
parking_spot_indices = [(2,3), (2,4),  (2,5),(2,6), (2,7), (2,8),
                         (4,3), (4,4), (4,5),(4,6), (4,7), (4,8),
                         (7,3), (7,4), (7,5),(7,6), (7,7), (7,8), 
                         (9,3), (9,4), (9,5),(9,6), (9,7), (9,8)]

def compute_value(grid,goal,cost):
    # ----------------------------------------
    # insert code below
    # ----------------------------------------
    
    # make sure your function returns a grid of values as 
    # demonstrated in the previous video.
    value = [[99 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    x = goal[0]
    y = goal[1]
    value[x][y] = 0
    open = [[x,y]]
   
    closed[x][y] = 1
    finished = False
    resign = False
    while finished == False and resign == False:
        if len(open)==0:
            resign = True
        else :
            
            curr = open.pop(0)
            x = curr[0]
            y = curr[1]
            
            if x == start[0] and y == start[1]:
                finished = True
            else:
                for i in range(len(delta)):
                    x2 = x - delta[i][0]
                    y2 = y - delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):   
                        
                        if grid[x2][y2] == 0 and closed[x2][y2] == 0:
                            
                            """if (x,y) in parking_spot_indices and (x2,y2) in parking_spot_indices:
                                value[x2][y2] = value[x][y] + cost_p
                            else:
                                value[x2][y2] = value[x][y] + cost"""
                            if (x,y) in parking_spot_indices and (x2,y2) in parking_spot_indices:
                                continue
                            else:
                                value[x2][y2] = value[x][y] + cost
                            closed[x2][y2] = 1
                            open.append([x2,y2])
            
                
    return value 
value = compute_value(grid,goal,cost)
print "Value of each grid cell is : "
for i in range(len(value)):
    print value[i]

spot_with_value = []
#Stored as [(value1, (spot_coords), input_idx)...]
for j in range(len(parking_spot_indices)):
    
    heappush(spot_with_value, (value[parking_spot_indices[j][0]][parking_spot_indices[j][1]], parking_spot_indices[j], j))


"""for spot_val in spot_with_value:
    print spot_val,"""

