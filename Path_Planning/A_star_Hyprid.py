import math
import numpy as np
import matplotlib.pyplot as plt
import time
# from scipy.odr import odrpack as odr
# from scipy.odr import models



D = 1
NUM_THETA_CELLS = 90
Alfa_Change = 5
Map_Reslution = 1
obs = 0
free = 1

### CLASSES
class state:
    def __init__(self, x=0, y=0, theta=0, g=0, h=0):
        self.x=x
        self.y=y
        self.theta=theta
        self.g=g
        self.h=h
    def __str__(self):
        return str(self.__class__) + ": " + str(self.__dict__)

class current:
    def __init__(self, X=0, Y=0, TH=0, G=0, F=0):
        self.x = X
        self.y = Y
        self.theta= TH
        self.g = G
        self.f = F

class my_path():
  def __init__(self,closed = 0,came_from = 0,final = 0):
    self.closed=closed
    self.came_from=came_from
    self.final=final


### NORMALIZE THETA
def normalize_theta(theta):
  if( theta<0 ):
    theta +=( 2*np.pi )
  elif( theta>2*np.pi ):
    theta %=( 2*np.pi)
  return theta

# http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#
# heuristics-for-grid-maps
### Manhattan distance D=1
def heuristic(x, y, goal):

    dx = abs(x - goal[0])
    dy = abs(y - goal[1])
    return D * math.sqrt(dx * dx + dy * dy)


def theta_to_stack_number(theta):
    #Takes an angle (in radians) and returns which "stack" in the 3D configuration space
    #this angle corresponds to. Angles near 0 go in the lower stacks while angles near
    #2 * pi go in the higher stacks.
    PI = 3.14159
    new_theta = math.fmod((theta + 2 * PI),(2 * PI))
    stack_number = (round(new_theta * NUM_THETA_CELLS / (2*PI))) % NUM_THETA_CELLS
    return stack_number

def idx(float_num) :
    #Returns the index into the grid for continuous position. So if x is 3.621, then this
    #would return 3 to indicate that 3.621 corresponds to array index 3.
    return math.floor(float_num)

# Update cost at new state when expanding the search
def update_cost(current_g, x, y, goal):
    g = current_g + 1
    f = g + heuristic(x, y, goal)
    return g, f

# Update next state of model when expanding the search
def update_state(current_x, current_y, current_theta, SPEED, DT, delta):

    # ---Begin bicycle model--- MODEL NO.1
    LENGTH = 0.74
    delta_rad  = math.radians(delta) # convert to radian
    omega      = SPEED/LENGTH * math.tan(delta_rad)
    theta      = normalize_theta(current_theta + omega)
    dx         = (SPEED * math.cos(theta) * DT) * Map_Reslution
    dy         = (SPEED * math.sin(theta) * DT) * Map_Reslution
    x          = current_x + dx
    y          = current_y + dy
    # print (x, y)
    # ---End bicycle model-----

    return x, y, theta

def expand(state_value, goal, SPEED, DT):
    next_states = []
    for delta in range(-25, 25, Alfa_Change):
        # Create a trajectory with delta as
        # the steering angle using the bicycle model:
        # Update from current
        new_state = state()
        new_state.x, new_state.y, new_state.theta = update_state(state_value.x, state_value.y, state_value.theta, SPEED, DT, delta)

        # Update cost function
        new_state.g, new_state.f = update_cost(state_value.g, new_state.x, new_state.y, goal)

        # Create a new State object with all of the "next" values.
        next_states.append(new_state)

    return next_states

def search(grid, start_point, goal, SPEED, DT, Map_Reslution):
    # The opened array keeps track of the stack of States objects we are
    # searching through.
    opened = []
    # 3D array of zeros with dimensions:
    # (NUM_THETA_CELLS, grid x size, grid y size).
    closed = [[[0 for x in range(len(grid[0]))] for y in range(len(grid))] for cell in range(NUM_THETA_CELLS)]
    # 3D array with same dimensions. Will be filled with State() objects to keep
    # track of the path through the grid.
    came_from = [[[0 for x in range(len(grid[0]))] for y in range(len(grid))] for cell in range(NUM_THETA_CELLS)]

    state_value = state()
    # Create new state object to start the search with.
    state_value.x = start_point[0]
    state_value.y = start_point[1]
    state_value.theta = start_point[2]
    state_value.g = 0
    state_value.f = heuristic(state_value.x, state_value.y, goal)
    opened.append(state_value)

    # The range from 0 to 2pi has been discretized into NUM_THETA_CELLS cells.
    # Here, theta_to_stack_number returns the cell that theta belongs to.
    # Smaller thetas (close to 0 when normalized  into the range from 0 to 2pi)
    # have lower stack numbers, and larger thetas (close to 2pi whe normalized)
    # have larger stack numbers.
    stack_number = theta_to_stack_number(state_value.theta)
    closed[stack_number][idx(state_value.x)][idx(state_value.y)] = 1

    # Store our starting state. For other states, we will store the previous state
    # in the path, but the starting state has no previous.
    came_from[stack_number][idx(state_value.x)][idx(state_value.y)] = state_value
    # While there are still states to explore:
    while opened:
        # Sort the states by f-value and start search using the state with the
        # lowest f-value. This is crucial to the A* algorithm; the f-value
        # improves search efficiency by indicating where to look first.
        opened.sort(key=lambda state : float(state_value.f))
        current = opened.pop(0)

        # Check if the x and y coordinates are in the same grid cell as the goal.
        # (Note: The idx function returns the grid index for a given coordinate.)
        if (idx(current.x) == idx(goal[0])) and (idx(current.y) == idx(goal[1])):
            # If so, the trajectory has reached the goal.
            path = my_path()
            print ("goal is found ")
            path.came_from = came_from
            path.closed = closed
            path.final = current
            # print ("the last : ",current.x, current.y, current.theta)
            return path, True

        # Otherwise, expand the current state to get a list of possible next states.
        next_states = expand(current, goal, SPEED, DT)
        for next_state in next_states:
            # If we have expanded outside the grid, skip this next_state.
            # print (next_state.x,next_state.y,next_state.theta)
            if(next_state.x < 0 or next_state.x >= len(grid) or next_state.y < 0 or next_state.y >= len(grid[0])) : #next_states is not in the grid:
                continue
            # Otherwise, check that we haven't already visited this cell and
            # that there is not an obstacle in the grid there.
            stack_number = theta_to_stack_number(next_state.theta)
            if closed[stack_number][idx(next_state.x)][idx(next_state.y)] == 0 and grid[idx(next_state.x)][idx(next_state.y)] == free :
                # The state can be added to the opened stack.
                opened.append(next_state)
                # The stack_number, idx(next_state.x), idx(next_state.y) tuple
                # has now been visited, so it can be closed.
                closed[stack_number][idx(next_state.x)][idx(next_state.y)] = 1
                # print ("the saved : ",current.x, current.y, current.theta)
                # The next_state came from the current state, and that is recorded.
                came_from[stack_number][idx(next_state.x)][idx(next_state.y)] = current

    print ('pass is not found')
    path = my_path()
    path.came_from = came_from
    path.closed = closed
    path.final = current
    return path, False

### RECONSTRUCT PATH
def reconstruct_path(came_from, start, final, Map_Reslution):
    # start              = scale_point(start, Map_Reslution)
    path                 = [(final)]
    stack                = theta_to_stack_number(final.theta)
    final_theta          = final.theta
    current              = came_from[stack][idx(final.x)][idx(final.y)]
    stack                = theta_to_stack_number(current.theta)
    X = current.x
    Y = current.y
    while [X, Y] != [(start[0]), (start[1])] :
        # print([(current.x), (current.y), current.theta])
        # print("start:",start)
        path.append(current)
        current              = came_from[stack][idx(X)][idx(Y)]
        X = current.x
        Y = current.y
        stack                = theta_to_stack_number(current.theta)
    path.append(current)
    current              = came_from[stack][idx(X)][idx(Y)]
    stack                = theta_to_stack_number(current.theta)
    x = []
    y = []
    for the_way in path :
        x.append((the_way.x) / Map_Reslution)
        y.append((the_way.y) / Map_Reslution)
        #print (the_way.x, the_way.y, the_way.theta)
    final_state = [x[0], y[0], final_theta]
    x.reverse()
    y.reverse()
    return x, y, final_state

def Creat_Map_Size(grid, Map_Reslution):
    MAP =[[1 for x in range(len(grid[0]) * Map_Reslution )] for y in range(len(grid)* Map_Reslution)]
    for i in range (len(grid)):
        for j in range (len(grid[0])):
            if (grid[i][j]) == 0 :
                for k in range ((i*Map_Reslution),(i*Map_Reslution+Map_Reslution)):
                    for m in range ((j*Map_Reslution),(j*Map_Reslution+Map_Reslution)):
                        MAP[k][m] = 0
    return MAP

def scale_point(point, Map_Reslution):
    scaled_point = point
    scaled_point[0] = point[0] * Map_Reslution
    scaled_point[1] = point[1] * Map_Reslution
    return scaled_point


# grid = [[_,X,X,_,_,_,_,_,_,_,X,X,_,_,_,_],
#         [_,X,X,_,_,_,_,_,_,X,X,_,_,_,_,_],
#         [_,X,X,_,_,_,_,_,X,X,_,_,_,_,_,_],
#         [_,X,X,_,_,_,_,X,X,_,_,_,X,X,X,_],
#         [_,X,X,_,_,_,X,X,_,_,_,X,X,X,_,_],
#         [_,X,X,_,_,X,X,_,_,_,X,X,X,_,_,_],
#         [_,X,X,_,X,X,_,_,_,X,X,X,_,_,_,_],
#         [_,X,X,X,X,_,_,_,X,X,X,_,_,_,_,_],
#         [_,X,X,X,_,_,_,X,X,X,_,_,_,_,_,_],
#         [_,X,X,_,_,_,X,X,X,_,_,X,X,X,X,X],
#         [_,X,_,_,_,X,X,X,_,_,X,X,X,X,X,X],
#         [_,_,_,_,X,X,X,_,_,X,X,X,X,X,X,X],
#         [_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,X],
#         [_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,X],
#         [_,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_],
#         [X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_]
#         ]
# SPEED = 2
# DT = 2

# X = 0
# _ = 1

# grid = [[_,_,_,X,X,X,X,X,X,X,X,X,X,X,X],
#         [_,_,_,X,X,X,X,X,X,X,X,X,X,X,X],
#         [_,_,_,X,X,X,X,X,X,X,X,X,X,X,X],
#         [_,_,_,X,X,X,X,X,X,X,X,X,X,X,X],
#         [_,_,_,X,X,X,X,X,X,X,X,X,X,X,X],
#         [_,_,_,X,X,X,X,X,X,X,X,X,X,X,X],
#         [_,_,_,_,_,_,_,_,_,_,_,_,_,_,_],
#         [_,_,_,_,_,_,_,_,_,_,_,_,_,_,_],
#         [_,_,_,X,X,X,X,X,X,X,X,X,X,X,X],
#         [_,_,_,X,X,X,X,X,X,X,X,X,X,X,X],
#         ]


# # print(grid)
# # print (MAP)
# MAP = Creat_Map_Size(grid, Map_Reslution)

# print ("-----------------------------------------------------------------------")
# start = [0 , 0, 0]
# goal = [6, 12]
# print ("the start :", start)
# print ("the end :", goal)
# start = scale_point (start, Map_Reslution)
# goal = scale_point (goal, Map_Reslution)
# start_time = time.time()
# PATH, check = search(MAP, start, goal, SPEED, DT, Map_Reslution)
# x, y, final_state =  reconstruct_path(PATH.came_from, start, PATH.final, Map_Reslution)

# print("X : ", x)
# print("Y : ", y)
# print ("the lenght : " , len(x))
# SPEED = 1
# DT = 0.5
# x_cor = [start[0]]
# y_cor = [start[1]]
# x_cof = [start[0]]
# y_cof = [start[1]]
# end_state = []
# for i in range (1, len(x)):
#     if i == 1:
#         start_point = start
#     else :
#         start_point = end_state
#     end_point = [x[i], y[i]]
#     if [idx(start_point[0]), idx(start_point[1])] == [idx(end_point[0]), idx(end_point[1])]:
#         continue
#     print ("-----------------------------------------------------------------------")
#     print ("the start :", start_point)
#     print ("the end :", end_point)
#     scaled_start_point = scale_point(start_point, Map_Reslution)
#     scaled_end_point = scale_point(end_point, Map_Reslution)
#     print ("the scaled start :", start_point)
#     print ("the scald end :", end_point)
#     PATH, check = search(MAP, scaled_start_point, scaled_end_point, H_grid, Map_Reslution)
#     print ("finale state:",end_state)
#     if check == False :
#         print ("#################################-PASS IS NOT FOUND-#################################")
#         continue
#     x_path, y_path, end_state =  reconstruct_path(PATH.came_from, scaled_start_point, PATH.final)
#     print ("finale state:",end_state)
#     # print("X : ", x_path)
#     # print("Y : ", y_path)
#     # print ("the lenght : " , len(x_path))
#     coff = np.polyfit(x_path,y_path,3)
#     new_y = np.polyval(coff,x_path)
#     print ("the coff :",coff)

#     for j in range (1,len(x_path)):
#         x_cof.append(x_path[j])
#         y_cof.append(new_y[j])
#         x_cor.append(x_path[j])
#         y_cor.append(y_path[j])

# print("X : ", x_path)
# print("Y : ", y_path)
# print ("the lenght : " , len(x_path))
# print ("the lenght : " , len(x_cor))

# # coff_SIPY , err = poly_lsq(x,y,3,itmax=20000)
# # print ("sipy coff :",coff_SIPY , err)
# #poly = np.poly1d(coff)
# #new_x = np.linspace(x[0], x[-1])
# # new_y = np.polyval(coff,x)
# # print('in')
# # s_y = np.polyval(coff_SIPY,x)

# end_time = time.time()
# print ("The total time : ",end_time - start_time)
# obs_x = []
# obs_y = []
# for i in range (len(grid)) :
#     for j in range (len(grid[0])) :
#         if (grid[i][j]) == 0 :
#             obs_x.append(i)
#             obs_y.append(j)


# plt.plot(obs_x,obs_y,'o')
# plt.plot(x,y)
# plt.plot(x_cor,y_cor)
# plt.plot(x_cof,y_cof)
# plt.show()
