from math import pi , acos , sin , cos
from heapq import heappush, heappop, heapify
import sys
from time import perf_counter
import tkinter as tk

def calcd(node1, node2):
   y1, x1 = node1
   y2, x2 = node2
   R   = 3958.76 # miles = 6371 km
   y1 *= pi/180.0
   x1 *= pi/180.0
   y2 *= pi/180.0
   x2 *= pi/180.0
   return acos( sin(y1)*sin(y2) + cos(y1)*cos(y2)*cos(x2-x1) ) * R # approximate great circle distance with law of cosines

with open("rrNodes.txt") as f: # make dict of coordinates of each edge
    node_coordinates = dict()
    for line in f:
        line = line.strip().split()
        node_coordinates[line[0]] = (float(line[1]), float(line[2]))
with open("rrNodeCity.txt") as f:
    city_names = dict()
    for line in f:
        line = line.strip().split()
        city_names[" ".join(line[1:])] = line[0]
with open("rrEdges.txt") as f: # create train network as dict
    train_network = dict()
    for line in f:
        line = line.strip().split()
        distance = calcd(node_coordinates[line[0]], node_coordinates[line[1]])
        if line[0] in train_network.keys():
            train_network[line[0]].add((line[1], distance))
        else:
            train_network[line[0]] = {(line[1], distance)}
        if line[1] in train_network.keys():
            train_network[line[1]].add((line[0], distance))
        else:
            train_network[line[1]] = {(line[0], distance)}

def dijkstra(starting_node, ending_node, root, canvas):
    update = 800
    closed, start, fringe, goal = set(), (0, starting_node, [starting_node]), [], ending_node
    heappush(fringe, start)
    while fringe:
        distance, curr, parents = heappop(fringe)
        if goal == curr:
            while len(parents) > 1:
                parent = parents.pop()
                canvas.itemconfig(map_dict[(parent, curr)], fill="green")
                canvas.itemconfig(map_dict[(curr, parent)], fill="green")
                curr = parent
            root.update()
            return distance
        if curr not in closed:
            closed.add(curr)
            for child in train_network[curr]:
                if child not in closed:
                    canvas.itemconfig(map_dict[(curr, child[0])], fill="red") #changes color to red
                    if update==0:
                        root.update() #update frame
                        update = 800
                    update -= 1
                    heappush(fringe, (distance+child[1], child[0], parents + [curr]))
    return None

def a_star(starting_node, ending_node, root, canvas):
    update = 600
    closed, start, fringe, goal = set(), (calcd(node_coordinates[starting_node], node_coordinates[ending_node]), starting_node, 0, [starting_node]), [], ending_node
    heappush(fringe, start)
    while fringe:
        distance, curr, actual_distance, parents = heappop(fringe)
        if goal == curr:
            while len(parents) >1:
                parent = parents.pop()
                canvas.itemconfig(map_dict[(curr, parent)], fill="green")
                canvas.itemconfig(map_dict[(parent, curr)], fill="green")
                curr = parent
            root.update()
            return actual_distance
        if curr not in closed:
            closed.add(curr)
            for child in train_network[curr]:
                if child not in closed:
                    canvas.itemconfig(map_dict[(curr, child[0])], fill="red") #changes color to red
                    if update==0:
                        root.update() #update frame
                        update = 600
                    update -= 1
                    heappush(fringe, (actual_distance+child[1]+calcd(node_coordinates[child[0]], node_coordinates[ending_node]), child[0], actual_distance+child[1], parents + [curr]))
    return None

def dfs(starting_node, ending_node, root, canvas):
    goal, update = ending_node, 300
    fringe, depth, ancestors = [], 0, set()
    fringe.append((starting_node, depth, ancestors, [starting_node]))
    ancestors.add(starting_node)
    while fringe:
        curr, currdistance, currancestors, parents = fringe.pop()
        if curr == goal:
            while len(parents) >1:
                parent = parents.pop()
                canvas.itemconfig(map_dict[(curr, parent)], fill="green")
                canvas.itemconfig(map_dict[(parent, curr)], fill="green")
                curr = parent
            root.update()
            return currdistance
        for child in train_network[curr]:
            if child[0] not in currancestors:
                newancestors = currancestors
                newancestors.add(child[0])
                canvas.itemconfig(map_dict[(curr, child[0])], fill="red") #changes color to red
                canvas.itemconfig(map_dict[(child[0], curr)], fill="red")
                if update==0:
                    root.update() #update frame
                    update = 300
                update -= 1
                fringe.append((child[0], currdistance+child[1], newancestors, parents + [curr]))
    return None

def kdfs(starting_node, ending_node, root, canvas, k):
    goal, update = ending_node, 500
    fringe, depth, ancestors = [], 0, set()
    fringe.append((starting_node, depth, ancestors, [starting_node]))
    ancestors.add(starting_node)
    while fringe:
        curr, currdistance, currancestors, parents = fringe.pop()
        if curr == goal:
            while len(parents) >1:
                parent = parents.pop()
                canvas.itemconfig(map_dict[(curr, parent)], fill="green")
                canvas.itemconfig(map_dict[(parent, curr)], fill="green")
                curr = parent
            root.update()
            print(currdistance)
            return currdistance
        if currdistance < k:
            for child in train_network[curr]:
                if child[0] not in currancestors:
                    newancestors = currancestors
                    newancestors.add(child[0])
                    canvas.itemconfig(map_dict[(curr, child[0])], fill="red") #changes color to red
                    canvas.itemconfig(map_dict[(child[0], curr)], fill="red")
                    if update==0:
                        root.update() #update frame
                        update = 500
                    update -= 1
                    fringe.append((child[0], currdistance+child[1], newancestors, parents + [curr]))
    return None

def iddfs(starting_node, ending_node, root, canvas):
    max_depth = 500
    result = None
    while result == None:
        create_map(canvas)
        result = kdfs(starting_node, ending_node, root, canvas, max_depth)
        max_depth += 500
    return result

def bidirectional_dijkstra(starting_node, ending_node, root, canvas): # make dijkstra, currently j bfs
    goal, update = ending_node, 200
    sourcefringe, goalfringe, sourcevisited, goalvisited = [], [], set(), set()
    heappush(sourcefringe, (calcd(node_coordinates[starting_node], node_coordinates[ending_node]), starting_node, [starting_node]))
    heappush(goalfringe, (calcd(node_coordinates[starting_node], node_coordinates[ending_node]), goal, [ending_node]))
    sourcevisited.add(starting_node)
    goalvisited.add(goal)

    while sourcefringe and goalfringe:
        scurrdistance, scurr, sparents = heappop(sourcefringe)
        gcurrdistance, gcurr, gparents = heappop(goalfringe)
        
        if scurr in goalvisited or gcurr in sourcevisited:
            while len(gparents) >1:
                parent = gparents.pop()
                canvas.itemconfig(map_dict[(gcurr, parent)], fill="green")
                canvas.itemconfig(map_dict[(parent, gcurr)], fill="green")
                gcurr = parent
            while len(sparents) >1:
                parent = sparents.pop()
                canvas.itemconfig(map_dict[(scurr, parent)], fill="green")
                canvas.itemconfig(map_dict[(parent, scurr)], fill="green")
                scurr = parent
            root.update()
            return 1

        for child in train_network[scurr]:
            if child not in sourcevisited:
                canvas.itemconfig(map_dict[(scurr, child[0])], fill="red") #changes color to red
                canvas.itemconfig(map_dict[(child[0], scurr)], fill="red")
                if update==0:
                    root.update() #update frame
                    update = 200
                update -= 1
                heappush(sourcefringe, (distance+child[1], child[0], sparents + [scurr]))
                sourcevisited.add(child)

        for child in train_network[gcurr]:
            if child not in goalvisited:
                canvas.itemconfig(map_dict[(gcurr, child[0])], fill="red") #changes color to red
                canvas.itemconfig(map_dict[(child[0], gcurr)], fill="red")
                if update==0:
                    root.update() #update frame
                    update = 200
                update -= 1
                heappush(goalfringe, (distance+child[1], child[0], gparents + [gcurr]))
                goalvisited.add(child)
    return "0"

def reverse_astar(starting_node, ending_node, root, canvas): 
    update = 400
    closed, start, fringe, goal = set(), (calcd(node_coordinates[starting_node], node_coordinates[ending_node])*-1, starting_node, 0, [starting_node]), [], ending_node
    heappush(fringe, start)
    while fringe:
        distance, curr, actual_distance, parents = heappop(fringe)
        if goal == curr:
            while len(parents) >1:
                parent = parents.pop()
                canvas.itemconfig(map_dict[(curr, parent)], fill="green")
                canvas.itemconfig(map_dict[(parent, curr)], fill="green")
                curr = parent
            root.update()
            return actual_distance
        if curr not in closed:
            closed.add(curr)
            for child in train_network[curr]:
                if child not in closed:
                    canvas.itemconfig(map_dict[(curr, child[0])], fill="red") #changes color to red
                    if update==0:
                        root.update() #update frame
                        update = 400
                    update -= 1
                    heappush(fringe, (actual_distance+(child[1]+calcd(node_coordinates[child[0]], node_coordinates[ending_node]))*-1, child[0], actual_distance+(child[1]*-1), parents + [curr]))
    return None

def bidirectional_astar(starting_node, ending_node, root, canvas):
    goal, update = ending_node, 200
    sourcefringe, goalfringe, sourcevisited, goalvisited = [], [], set(), set()
    heappush(sourcefringe, (0, starting_node, [starting_node]))
    heappush(goalfringe, (0, goal, [ending_node]))
    sourcevisited.add(starting_node)
    goalvisited.add(goal)

    while sourcefringe and goalfringe:
        scurrdistance, scurr, sparents = heappop(sourcefringe)
        gcurrdistance, gcurr, gparents = heappop(goalfringe)
        
        if scurr in goalvisited or gcurr in sourcevisited:
            while len(gparents) >1:
                parent = gparents.pop()
                canvas.itemconfig(map_dict[(gcurr, parent)], fill="green")
                canvas.itemconfig(map_dict[(parent, gcurr)], fill="green")
                gcurr = parent
            while len(sparents) >1:
                parent = sparents.pop()
                canvas.itemconfig(map_dict[(scurr, parent)], fill="green")
                canvas.itemconfig(map_dict[(parent, scurr)], fill="green")
                scurr = parent
            root.update()
            return 1

        for child in train_network[scurr]:
            if child not in sourcevisited:
                canvas.itemconfig(map_dict[(scurr, child[0])], fill="red") #changes color to red
                canvas.itemconfig(map_dict[(child[0], scurr)], fill="red")
                if update==0:
                    root.update() #update frame
                    update = 200
                update -= 1
                heappush(sourcefringe, (scurrdistance+child[1]+calcd(node_coordinates[child[0]], node_coordinates[ending_node]), child[0], sparents + [scurr]))
                sourcevisited.add(child)

        for child in train_network[gcurr]:
            if child not in goalvisited:
                canvas.itemconfig(map_dict[(gcurr, child[0])], fill="red") #changes color to red
                canvas.itemconfig(map_dict[(child[0], gcurr)], fill="red")
                if update==0:
                    root.update() #update frame
                    update = 200
                update -= 1
                heappush(goalfringe, (gcurrdistance+child[1]+calcd(node_coordinates[child[0]], node_coordinates[starting_node]), child[0], gparents + [gcurr]))
                goalvisited.add(child)
    return "0"

map_dict = dict()
def create_map(canvas):
    for station in train_network:
        for route in train_network[station]:
            map_dict[(station, route[0])] = canvas.create_line([(node_coordinates[route[0]][1]*14+1835, (800-node_coordinates[route[0]][0])*14-10300), (node_coordinates[station][1]*14+1835, (800-node_coordinates[station][0])*14-10300)], tag='map_line')

start_node, end_node = sys.argv[1], sys.argv[2]
algorithm = input("Algorithm to run?"+"\n"+"Type 0 for Dijkstra"+"\n"+"Type 1 for A*"+"\n"+"Type 2 for DFS"+"\n"+"Type 3 for ID-DFS"+"\n"+"Type 4 for Bidirectional Dijkstra"+"\n"+"Type 5 for Reverse A*"+"\n"+"Type 6 for Bidirectional A*"+"\n")
root = tk.Tk() #creates the frame
canvas = tk.Canvas(root, height=800, width=1000, bg='white') #creates a canvas widget, which can be used for drawing lines and shapes
create_map(canvas)
canvas.pack(expand=True) #packing widgets places them on the board2
if algorithm == "0":
    solution = dijkstra(city_names[start_node], city_names[end_node], root, canvas)
elif algorithm == "1":
    solution = a_star(city_names[start_node], city_names[end_node], root, canvas)
elif algorithm == "2":
    solution = dfs(city_names[start_node], city_names[end_node], root, canvas)
elif algorithm == "3":
    solution = iddfs(city_names[start_node], city_names[end_node], root, canvas)
elif algorithm == "4":
    solution = bidirectional_dijkstra(city_names[start_node], city_names[end_node], root, canvas)
elif algorithm == "5":
    solution = reverse_astar(city_names[start_node], city_names[end_node], root, canvas)
elif algorithm == "6":
    solution = bidirectional_astar(city_names[start_node], city_names[end_node], root, canvas)
else:
    print("invalid choice")
root.mainloop()

# python trainroutes.py