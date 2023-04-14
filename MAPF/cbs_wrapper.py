#!/usr/bin/python
from pathlib import Path
from MAPF.cbs import CBSSolver


SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals

def transform_paths(paths):

    #print(paths)

    transformed_paths = []

    for path in paths:
        new_path = []

        for p in path:
            new_p = [-1, -1]
            new_p[0] = p[1]
            new_p[1] = p[0] + 2 * (2 - p[0])
            new_path.append(new_p)
        
        transformed_paths.append(new_path)
    
    #print(transformed_paths)
    return transformed_paths

class CBS_WRAPPER(object):

    def __init__(self):
        file = "MAPF/instances/demo.txt"
        print("***Import an instance***")
        self.my_map, self.starts, self.goals = import_mapf_instance(file)
        print_mapf_instance(self.my_map, self.starts, self.goals)

    def solve(self):
        print("***Run CBS***")
        cbs = CBSSolver(self.my_map, self.starts, self.goals)
        paths = cbs.find_solution()

        return transform_paths(paths)

    

        
