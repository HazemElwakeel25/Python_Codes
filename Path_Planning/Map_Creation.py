
import osmnx as ox
import numpy as np
import pandas as pd
import math
import networkx as nx
import matplotlib.pyplot as plt
import time
import utm
import A_star_Hyprid
# from numpy import savetxt
# from operator import itemgetter

## lat    lon
##  y      x
obs = 0
free = 1

D = 1
LENGTH = 0.74
NUM_THETA_CELLS = 90
Alfa_Change = 5
PI = 3.14159


def idx(float_num) :
    #Returns the index into the grid for continuous position. So if x is 3.621, then this
    #would return 3 to indicate that 3.621 corresponds to array index 3.
    return math.floor(float_num)

def draw_line(mat, x0, y0, x1, y1, i,inplace=True ):
    if not (0 <= x0 < mat.shape[0] and 0 <= x1 < mat.shape[0] and
            0 <= y0 < mat.shape[1] and 0 <= y1 < mat.shape[1]):
        return
    if not inplace:
        mat = mat.copy()
    if (x0, y0) == (x1, y1):
        mat[x0, y0] = free
        return mat if not inplace else None
    # Swap axes if Y slope is smaller than X slope
    transpose = abs(x1 - x0) < abs(y1 - y0)
    if transpose:
        mat = mat.T
        x0, y0, x1, y1 = y0, x0, y1, x1
    # Swap line direction to go left-to-right if necessary
    if x0 > x1:
        x0, y0, x1, y1 = x1, y1, x0, y0
    # Write line ends
    mat[x0, y0] = free
    mat[x1, y1] = free
    # Compute intermediate coordinates using line equation
    x = np.arange(x0 + 1, x1)
    y = np.round(((y1 - y0) / (x1 - x0)) * (x - x0) + y0).astype(x.dtype)
    # Write intermediate coordinates
    mat[x, y] = free
    if not inplace:
        return mat if not transpose else mat.T

def boundry ( maxlat, maxlon, minlat, minlon, edges ):

    for i in range (len (edges)):
        if maposm.nodes[edges[i][0]]['y'] < minlat:
            minlat = maposm.nodes[edges[i][0]]['y']
        elif maposm.nodes[edges[i][0]]['y'] > maxlat :
            maxlat = maposm.nodes[edges[i][0]]['y']
        if maposm.nodes[edges[i][1]]['y'] < minlat:
            minlat = maposm.nodes[edges[i][1]]['y']
        elif maposm.nodes[edges[i][1]]['y'] > maxlat :
            maxlat = maposm.nodes[edges[i][1]]['y']

        if maposm.nodes[edges[i][0]]['x'] < minlon:
            minlon = maposm.nodes[edges[i][0]]['x']
        elif maposm.nodes[edges[i][0]]['x'] > maxlon :
            maxlon = maposm.nodes[edges[i][0]]['x']
        if maposm.nodes[edges[i][1]]['x'] < minlon:
            minlon = maposm.nodes[edges[i][1]]['x']
        elif maposm.nodes[edges[i][1]]['x'] > maxlon :
            maxlon = maposm.nodes[edges[i][1]]['x']
    print ("min_y",minlat)
    print ("min_x",minlon)
    return maxlat, maxlon, minlat, minlon

def made_map( maxlat, maxlon, minlat, minlon, scale_value = 1 ):

    max_cordinate = list(utm.from_latlon(maxlat,maxlon))
    min_cordinate = list(utm.from_latlon((minlat),(minlon)))
    lenght_X = math.floor((max_cordinate[0]-min_cordinate[0])*scale_value) + 15
    lenght_Y = math.floor((max_cordinate[1]-min_cordinate[1])*scale_value) + 15
    grid = [[obs]*(lenght_Y)]*(lenght_X)
    map = np.array(grid)
    print ("map size:",len(grid), len(grid[0]))
    return map

def map_creation (map, edges, minlat, minlon, scale_value = 1 ) :
    min_cordinate = list(utm.from_latlon((minlat),(minlon)))
    array_edges1 = []
    array_edges2 = []
    for i in range (len (edges)):
        point1 = list(utm.from_latlon(maposm.nodes[edges[i][0]]['y'],maposm.nodes[edges[i][0]]['x']))
        point1[0] = (point1[0] - min_cordinate[0] )*scale_value
        point1[0] = math.floor(point1[0])
        # if ((point1[0]-int(point1[0]))>0.5):
        #     point1[0] = math.floor(point1[0])
        # else :
        #     point1[0] = math.floor(point1[0])

        point1[1] = (point1[1] - min_cordinate[1])*scale_value
        point1[1] = math.floor(point1[1])
        # if ((point1[1]-int(point1[1]))>0.5):
        #     point1[1] = math.floor(point1[1])
        # else :
        #     point1[1] = math.floor(point1[1])

        array_edges1.append ([point1[0],point1[1]])

        point2 = list(utm.from_latlon(maposm.nodes[edges[i][1]]['y'],maposm.nodes[edges[i][1]]['x']))
        point2[0] = (point2[0] - min_cordinate[0])*scale_value
        point2[0] = math.floor(point2[0])
        # if ((point2[0]-int(point2[0]))>0.5):
        #     point2[0] = math.floor(point2[0])
        # else :
        #     point2[0] = math.floor(point2[0])

        point2[1] = (point2[1]- min_cordinate[1])*scale_value
        point2[1] = math.floor(point2[1])
        # if ((point2[1]-int(point2[1]))>0.5):
        #     point2[1] = math.floor(point2[1])
        # else :
        #     point2[1] = math.floor(point2[1])

        array_edges2.append ([point2[0],point2[1]])
    for i in range (len (array_edges1)):
        draw_line(map, array_edges1[i][0],array_edges1[i][1],array_edges2[i][0],array_edges2[i][1],i)
    return map

def get_point(map, point, minlat, minlon, scale_value = 1):
    min_cordinate = list(utm.from_latlon((minlat),(minlon)))
    utm_point = list(utm.from_latlon(point[0],point[1]))
    utm_point[0] = (utm_point[0] - min_cordinate[0] )*scale_value
    utm_point[1] = (utm_point[1] - min_cordinate[1])*scale_value
    temp = [math.floor(utm_point[0]),math.floor(utm_point[1])]
    state = []

    if map[math.floor(utm_point[0])][math.floor(utm_point[1])] == free :
        # print ("the output :",[math.floor(utm_point[0]), math.floor(utm_point[1])])
        return [math.floor(utm_point[0]), math.floor(utm_point[1])] ,utm_point
    else :
        temp = [math.floor(utm_point[0]),math.floor(utm_point[1])]
        # print (temp)
        moves = [[-1, 0], [0, -1], [1, 0], [0, 1], [1, 1], [-1, 1], [1, -1], [-1, -1]]
        closed = [[temp[0], temp[1]]]
        closed_state = False
        state = [[0, temp[0], temp[1]]]
        while  map[temp[0]][temp[1]] != free :
            g = state[0][0]+1
            for i in range (len(moves)):
                temp[0] = state[0][1] + moves[i][0]
                temp[1] = state[0][2] + moves[i][1]
                if (len(map)-1 >= temp[0] >= 0 and len(map[0])-1 >= temp[1] >= 0):
                    if map[temp[0]][temp[1]] == free:
                        # print ("steps :", g)
                        # print ("the output :", temp)
                        # print ("the utm_out :",[math.floor(utm_point[0]), math.floor(utm_point[1])])
                        return temp , utm_point
                    for i in range (len(closed)) :
                        if closed[i] == temp :
                            closed_state = True
                            # print("closed:",closed[i])
                    if closed_state == False :
                        # print (temp)
                        state.append([g, temp[0], temp[1]])
                        closed.append([temp[0], temp[1]])
                    closed_state = False
            state.pop(0)
            state.sort()
            # state.reverse()
        #goal.sort

def Ceil_Or_Floor(val):
    # if ((val-int(val))>0.5):
    #     val = math.ceil(val)
    # else :
    #     val = math.floor(val)
    return math.floor(val)

def street_size(grid):
    MAP =[[obs for x in range(len(grid[0]))] for y in range(len(grid))]
    moves = [[-1, 0], [0, -1], [1, 0], [0, 1], [1, 1], [-1, 1], [1, -1], [-1, -1]]
    for i in range (len(grid)):
        for j in range (len(grid[0])):
            if (grid[i][j]) == free :
                for k in range (len(moves)):
                    x = i + moves[k][0]
                    y = j + moves[k][1]
                    if (len(grid)-1 >= x >= 0 and len(grid[0])-1 >= y >= 0):
                        MAP[x][y] = free

    return MAP

def Grid_boundry(x, y):
    min_x = min(x)
    max_x = max(x)
    min_y = min(y)
    max_y = max(y)

    print ("min_x :",math.floor(min_x))
    print ("min_y :",math.floor(min_y))
    print ("max_x :",math.ceil(max_x))
    print ("max_y :",math.ceil(max_y))
    return math.floor(min_x), math.floor(min_y), math.ceil(max_x), math.ceil(max_y)

def Made_Grid(MAP, min_x, min_y, max_x, max_y, scale_value = 1 ):

    max_cordinate = [max_x, max_y]
    min_cordinate = [min_x, min_y]
    lenght_X = math.floor((max_cordinate[0]-min_cordinate[0])*scale_value) + 10
    lenght_Y = math.floor((max_cordinate[1]-min_cordinate[1])*scale_value) + 10
    grid = [[free]*(lenght_Y)]*(lenght_X)
    map = np.array(grid)
    for i in range(len(grid)-1):
        for j in range (len(grid[0])-1):
            map[i][j] = MAP[i+min_x][j+min_y]

    print ("Grid size:",len(grid), len(grid[0]))
    return map

def refreance(x_path, y_path, min_x, min_y, minlat, minlon):
    X = np.array(x_path) - min_x
    Y = np.array(y_path) - min_y
    min_cordinate = list(utm.from_latlon((minlat),(minlon)))
    ref_x = min_x + min_cordinate[0]
    ref_y = min_y + min_cordinate[1]
    return X, Y, ref_x, ref_y

def print_map(Map):
    obs_x = []
    obs_y = []
    for i in range (len(Map)-1) :
        for j in range (len(Map[0])-1) :
            if (Map[i][j]) == 1 :
                obs_x.append(i)
                obs_y.append(j)
    return obs_x, obs_y

### installing the map

north = 29.97334
south = 29.96850
east = 31.00508
west = 30.99686

# north = 29.9914
# south = 29.9849
# east = 30.9583
# west = 30.9531

# north = 29.9961
# south = 29.9768
# east = 30.9768
# west = 30.9439

maposm = ox.graph_from_bbox(north, south, east, west, network_type='drive', simplify = False)


start_time = time.time()

edges = list (maposm.edges)
maxlat = north
maxlon = east
minlat = south
minlon = west

maxlat, maxlon, minlat, minlon = boundry ( maxlat, maxlon, minlat, minlon, edges )

GRID= made_map( maxlat, maxlon, minlat, minlon )

street_map = map_creation(GRID, edges, minlat, minlon )


# np.savetxt("D:\Studies\Code\A_Star\Map.csv", street_map, delimiter=',')


point1 = [29.971978, 31.001178]
# point1 = [29.989620, 30.955267]
start_point, start_utm_point = get_point(street_map, point1, minlat, minlon, scale_value = 1)


point2 = [29.971687, 31.001123]
# point2 = [29.965662, 31.001066]
# point2 = [29.969107, 31.997309]
# point2 = [29.971120, 31.002354]
# point2 = [29.970823, 31.003882]

# point2 = [29.989859, 30.956989]
end_point, end_utm_point = get_point(street_map, point2, minlat, minlon, scale_value = 1)

Map = street_size(street_map)



Map_Reslution = 1
SPEED = 1
DT = 5

MAP = A_star_Hyprid.Creat_Map_Size(Map, Map_Reslution)
print ("-----------------------------------------------------------------------")
start = [start_point[0] , start_point[1], 0 ]
goal = [end_point[0], end_point[1]]
print ("the start :", start)
print ("the end :", goal)
start = A_star_Hyprid.scale_point (start, Map_Reslution)
goal = A_star_Hyprid.scale_point (goal, Map_Reslution)
PATH, check = A_star_Hyprid.search(MAP, start, goal, SPEED, DT, Map_Reslution)
x_path, y_path, end_state =  A_star_Hyprid.reconstruct_path(PATH.came_from, start, PATH.final, Map_Reslution)

print("X : ", x_path)
print("Y : ", y_path)
print ("the lenght : " , len(x_path))



min_x, min_y, max_x, max_y = Grid_boundry(x_path, y_path)
Grid_map = Made_Grid(MAP, min_x, min_y, max_x, max_y)
X, Y, ref_x, ref_y = refreance(x_path, y_path, min_x, min_y, minlat, minlon)

print("X : ", X)
print("Y : ", Y)
print ("the lenght : " , len(X))


SPEED_grid = 1
DT_grid= 1
grid_Reslution = 1
sized_grid =  A_star_Hyprid.Creat_Map_Size(Grid_map, grid_Reslution)
x_cor = [X[0]]
y_cor = [Y[1]]
x_cof = [X[0]]
y_cof = [Y[1]]
end_state = []
for i in range (1, len(X),2):
    s_time = time.time()
    if i == 1:
        start_point = [X[0], Y[0], 0]
    else :
        start_point = end_state
    end_point = [X[i], Y[i]]
    if [idx(start_point[0]), idx(start_point[1])] == [idx(end_point[0]), idx(end_point[1])]:
        continue
    print ("-----------------------------------------------------------------------")
    print ("the start :", start_point)
    print ("the end :", end_point)
    scaled_start_point = A_star_Hyprid.scale_point(start_point, grid_Reslution)
    scaled_end_point = A_star_Hyprid.scale_point(end_point, grid_Reslution)
    print ("the scaled start :", start_point)
    print ("the scald end :", end_point)
    PATH, check = A_star_Hyprid.search(sized_grid, scaled_start_point, scaled_end_point, SPEED_grid, DT_grid, grid_Reslution)
    if check == False :
        start_point = A_star_Hyprid.scale_point(start_point, 1/grid_Reslution)
        end_point =  A_star_Hyprid.scale_point(end_point, grid_Reslution)
        print ("#################################-PASS IS NOT FOUND-#################################")
        continue
    way_x, way_y, end_state =  A_star_Hyprid.reconstruct_path(PATH.came_from, scaled_start_point, PATH.final, grid_Reslution)
    print ("finale state:",end_state)
    coff = np.polyfit(way_x,way_y,3)
    new_y = np.polyval(coff,way_x)
    print ("the coff :",coff)

    for j in range (1,len(way_x)):
        x_cof.append(way_x[j])
        y_cof.append(new_y[j])
        x_cor.append(way_x[j])
        y_cor.append(way_y[j])

    e_time = time.time()
    print ("time taken in the etration :", (e_time - s_time)/60)
    print ("itration no :", i)
print("X : ", x_cof)
print("Y : ", y_cof)
print ("the lenght : " , len(y_cof))



obs_x, obs_y = print_map(Map)
obs_grid_x, obs_grid_y = print_map(Grid_map)





fig1, ax1 = plt.subplots()
ax1.plot(obs_x, obs_y,'b.')

ax1.plot(start[0], start[1], 'go')
ax1.plot(goal[0], goal[1], 'go')


ax1.plot(x_path, y_path, 'r*')

fig2, ax2 = plt.subplots()
ax2.plot(obs_grid_x, obs_grid_y,'b.')
ax2.plot(X, Y, 'r*')
ax2.plot(x_cor,y_cor,'y')
ax2.plot(x_cof,y_cof,'g')

end_time = time.time()

print ("time taken :", (end_time - start_time)/60)
# ox.plot_graph(maposm)

plt.show()
