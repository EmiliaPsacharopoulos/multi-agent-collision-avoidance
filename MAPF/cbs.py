import time as timer
import heapq
import random
import copy
from MAPF.single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    
    maxTime = max(len(path1), len(path2))
    for t in range(maxTime):
        if get_location(path1, t) == get_location(path2, t):
            return {'a1': -1, 'a2': -1, 'loc': [get_location(path1, t)], 'timestep': t}
        if get_location(path1, t) == get_location(path2, t+1) and get_location(path1, t+1) == get_location(path2, t):
            return {'a1': -1, 'a2': -1, 'loc': [get_location(path1, t),get_location(path2,t)], 'timestep': t + 1}
    
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    collisions = []

    for i in range(len(paths)):
        for j in range(i+1, len(paths)):
            collision = detect_collision(paths[i], paths[j])
            if collision is not None:
                collision['a1'] = i
                collision['a2'] = j
                collisions.append(collision)
    
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    constraints = []

    if len(collision['loc']) == 1:
        constraints.append({'agent': collision['a1'],'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints.append({'agent': collision['a2'],'loc': collision['loc'], 'timestep': collision['timestep']})
    else:
        constraints.append({'agent': collision['a1'],'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints.append({'agent': collision['a2'],'loc': collision['loc'][::-1], 'timestep': collision['timestep']})
    
    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


def find_union(constraints1, constraint2):
        """Find the union of two sets of constraints"""
        
        found = False

        for constr in constraints1:
            if constr['agent'] != constraint2['agent']:
                continue
            elif constr['timestep'] != constraint2['timestep']:
                continue
            elif len(constr['loc']) != len(constraint2['loc']):
                continue

            if len(constr['loc']) == 1:
                if constr['loc'][0] == constraint2['loc'][0]:
                    found = True
                    break
            else:
                if constr['loc'][0] == constraint2['loc'][0] and constr['loc'][1] == constraint2['loc'][1]:
                    found = True
                    break
        
        if not found:
            constraints1.append(constraint2)
        
        return constraints1


def optimize_paths(paths):
    """Optimize the paths by removing redundant waypoints"""

    for path in paths:
        for i in range(len(path) - 1, 0, -1):
            if path[i] == path[i-1]:
                path.pop(i)
            else:
                break  
        
    return paths


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))

    def find_solution(self, disjoint=False):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        if disjoint:
            self.splitting = disjoint_splitting
        else:
            self.splitting = standard_splitting


        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        
        largest_path_length = 0

        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

            if len(path) > largest_path_length:
                largest_path_length = len(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        #print(root['collisions'])

        # Task 3.2: Testing
        #for collision in root['collisions']:
            #print(self.splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            node = self.pop_node()
            if len(node['collisions']) == 0:
                
                node['paths'] = optimize_paths(node['paths'])
                
                self.print_results(node)
                return node['paths']

            collision = node['collisions'][0]
            constraints = self.splitting(collision)

            for constraint in constraints:
                child = copy.deepcopy(node)
                child['constraints'] = find_union(child['constraints'], constraint)
                
                agent = constraint['agent']

                path = a_star(my_map = self.my_map, 
                              start_loc = self.starts[agent], 
                              goal_loc = self.goals[agent], 
                              h_values = self.heuristics[agent],
                              agent = agent,
                              constraints = child['constraints'])
                
                if largest_path_length < len(path):
                    largest_path_length = len(path)

                if path is None:
                    raise BaseException('No solutions')
                
                child['paths'][agent] = path

                child['cost'] = get_sum_of_cost(child['paths'])
                child['collisions'] = detect_collisions(child['paths'])
                self.push_node(child)

        # self.print_results(root)
        # return root['paths']
