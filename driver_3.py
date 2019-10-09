# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import queue as Q

import time

import resource

import sys

import math

#### SKELETON CODE ####

## The Class that Represents the Puzzle

class PuzzleState(object):

    """docstring for PuzzleState"""

    def __init__(self, config, n, parent=None, action="Initial", cost=0):

        if n*n != len(config) or n < 2:

            raise Exception("the length of config is not correct!")

        self.n = n

        self.cost = cost

        self.parent = parent

        self.action = action

        self.dimension = n

        self.config = config

        self.children = []

        for i, item in enumerate(self.config):

            if item == 0:

                self.blank_row = i // self.n

                self.blank_col = i % self.n

                break

    def display(self):

        for i in range(self.n):

            line = []

            offset = i * self.n

            for j in range(self.n):

                line.append(self.config[offset + j])

            print(line)

    def move_left(self):

        if self.blank_col == 0:

            return None

        else:

            blank_index = int(self.blank_row * self.n + self.blank_col)

            target = int(blank_index - 1)

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Left", cost=self.cost + 1)

    def move_right(self):

        if self.blank_col == self.n - 1:

            return None

        else:

            blank_index = int(self.blank_row * self.n + self.blank_col)

            target = int(blank_index + 1)

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Right", cost=self.cost + 1)

    def move_up(self):

        if self.blank_row == 0:

            return None

        else:

            blank_index = int(self.blank_row * self.n + self.blank_col)

            target = int(blank_index - self.n)

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Up", cost=self.cost + 1)

    def move_down(self):

        if self.blank_row == self.n - 1:

            return None

        else:

            blank_index = int(self.blank_row * self.n + self.blank_col)

            target = int(blank_index + self.n)

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Down", cost=self.cost + 1)

    def expand(self):

        """expand the node"""

        # add child nodes in order of UDLR

        if len(self.children) == 0:

            up_child = self.move_up()

            if up_child is not None:

                self.children.append(up_child)

            down_child = self.move_down()

            if down_child is not None:

                self.children.append(down_child)

            left_child = self.move_left()

            if left_child is not None:

                self.children.append(left_child)

            right_child = self.move_right()

            if right_child is not None:

                self.children.append(right_child)

        return self.children

# Function that Writes to output.txt

### Students need to change the method to have the corresponding parameters

def writeOutput(path, cost, expanded_nodes_num, search_depth, max_search_depth, running_time):
    
    """writeoutput"""
    
    output_file = open("output.txt", "w")
    
    output_file.write("path_to_goal: " + str(path))
    output_file.write("\n" + "cost_of_path: " + str(cost))
    output_file.write("\n" + "nodes_expanded: " + str(expanded_nodes_num))
    output_file.write("\n" + "search_depth: " + str(search_depth))
    output_file.write("\n" + "max_search_depth: " + str(max_search_depth))
    output_file.write("\n" + "running_time: " + str(running_time))
    
    usage = resource.getrusage(resource.RUSAGE_SELF)
    name = "ru_maxrss"
    max_ram_usage = getattr(usage, name)/(1024*1024)
    
    output_file.write ("\n" + "max_ram_usage: " + str(max_ram_usage))
    
    output_file.close()
    
    return output_file

def bfs_search(initial_state):

    """BFS search"""
    
    start_time = time.time()
    
    frontier_set = Q.Queue(0)
    frontier_set.put(initial_state)
    explored_set = []
    parent_to_child_dict = {}
    goal_path = []
    route_taken = []
    
    while not frontier_set.empty():
        fringe = frontier_set.get()
        
        if test_goal(fringe):
            break
        else:
            discovered_nodes = []
            expanded_set = fringe.expand()
            
            for i in expanded_set:
                if i not in (parent_to_child_dict and explored_set):
                    discovered_nodes += [i]
                    frontier_set.put(i)
            
            parent_to_child_dict[fringe] = discovered_nodes
            explored_set += [fringe]
    
    goal_state = fringe
    search_depth = 0
    route_taken += [fringe]
    
    while goal_state != initial_state:
        for i in parent_to_child_dict.keys():
            if goal_state in parent_to_child_dict[i]:
                route_taken += [i]
                search_depth += 1
                goal_state = i
                break
    
    route_taken.reverse()
    
    cost = calculate_total_cost(initial_state, fringe, parent_to_child_dict)
    
    for i in range(len(route_taken)-1):
        if route_taken[i].move_up() != None:
            if route_taken[i].move_up().config == route_taken[i+1].config:
                goal_path += ['Up']
        if route_taken[i].move_down() != None:
            if route_taken[i].move_down().config == route_taken[i+1].config:
                goal_path += ['Down']
        if route_taken[i].move_left() != None:
            if route_taken[i].move_left().config == route_taken[i+1].config:
                goal_path += ['Left']
        if route_taken[i].move_right() != None:
            if route_taken[i].move_right().config == route_taken[i+1].config:
                goal_path += ['Right']
    
    dead_end_nodes = []
    
    for i in parent_to_child_dict.keys():
        if parent_to_child_dict[i] == []:
            dead_end_nodes += [i]

    dead_end_nodes += [fringe]
    
    while not frontier_set.empty():
        dead_end_nodes += [frontier_set.get()]
    
    current_node = dead_end_nodes[0]
    max_search_depth = 0
    max_search_depth_temp = 0
    
    while dead_end_nodes != []:
        for i in parent_to_child_dict.keys():
            if current_node in parent_to_child_dict[i]:
                max_search_depth_temp += 1
                current_node = i
                break
        
        if current_node == initial_state:
            if max_search_depth_temp > max_search_depth:
                max_search_depth = max_search_depth_temp
                max_search_depth_temp = 0
            
            del dead_end_nodes[0]
            
            if dead_end_nodes != []:
                current_node = dead_end_nodes[0]
    
    expanded_set_num = len(explored_set)
    
    end_time = time.time()
    
    time_taken = end_time - start_time
    
    return writeOutput(goal_path, cost, expanded_set_num, search_depth, max_search_depth, time_taken)

def dfs_search(initial_state):

    """DFS search"""

    start_time = time.time()

    frontier_set = Q.LifoQueue(0)
    frontier_set.put(initial_state)
    explored_set = []
    parent_to_child_dict = {}
    goal_path = []
    route_taken = []
    
    while not frontier_set.empty():
        fringe = frontier_set.get()
        
        if test_goal(fringe):
            break
        else:
            discovered_nodes = []
            expanded_set = fringe.expand()
            expanded_set.reverse()
            
            for i in expanded_set:
                if i not in (parent_to_child_dict and explored_set):
                    discovered_nodes += [i]
                    frontier_set.put(i)
            
            parent_to_child_dict[fringe] = discovered_nodes
            explored_set += [fringe]
    
    goal_state = fringe
    search_depth = 0
    route_taken += [fringe]
    
    while goal_state != initial_state:
        for i in parent_to_child_dict.keys():
            if goal_state in parent_to_child_dict[i]:
                route_taken += [i]
                search_depth += 1
                goal_state = i
                break
    
    route_taken.reverse()
    
    cost = calculate_total_cost(initial_state, fringe, parent_to_child_dict)
    
    for i in range(len(route_taken)-1):
        if route_taken[i].move_up() != None:
            if route_taken[i].move_up().config == route_taken[i+1].config:
                goal_path += ['Up']
        if route_taken[i].move_down() != None:
            if route_taken[i].move_down().config == route_taken[i+1].config:
                goal_path += ['Down']
        if route_taken[i].move_left() != None:
            if route_taken[i].move_left().config == route_taken[i+1].config:
                goal_path += ['Left']
        if route_taken[i].move_right() != None:
            if route_taken[i].move_right().config == route_taken[i+1].config:
                goal_path += ['Right']

    dead_end_nodes = []
    
    for i in parent_to_child_dict.keys():
        if parent_to_child_dict[i] == []:
            dead_end_nodes += [i]

    dead_end_nodes += [fringe]
    
    while not frontier_set.empty():
        dead_end_nodes += [frontier_set.get()]
    
    current_node = dead_end_nodes[0]
    max_search_depth = 0
    max_search_depth_temp = 0
    
    while dead_end_nodes != []:
        for i in parent_to_child_dict.keys():
            if current_node in parent_to_child_dict[i]:
                max_search_depth_temp += 1
                current_node = i
                break
        
        if current_node == initial_state:
            if max_search_depth_temp > max_search_depth:
                max_search_depth = max_search_depth_temp
                max_search_depth_temp = 0
            
            del dead_end_nodes[0]
            
            if dead_end_nodes != []:
                current_node = dead_end_nodes[0]

    expanded_set_num = len(explored_set)
    
    end_time = time.time()
    
    time_taken = end_time - start_time

    return writeOutput(goal_path, cost, expanded_set_num, search_depth, max_search_depth, time_taken)

def A_star_search(initial_state):

    """A * search"""

    start_time = time.time()

    frontier_set = Q.PriorityQueue(0)
    initial_state_cost = 0
    index = 0
    
    for i in initial_state.config:
        initial_state_cost += calculate_manhattan_dist(index, i, initial_state.n)
        index += 1
    
    frontier_set.put((initial_state_cost, initial_state))
    explored_set = []
    parent_to_child_dict = {}
    goal_path = []
    route_taken = []
    
    while not frontier_set.empty():
        fringe_n_cost = frontier_set.get()
        fringe = fringe_n_cost[1]
        
        if test_goal(fringe):
            break
        else:
            discovered_nodes = []
            expanded_set = fringe.expand()
            
            for i in expanded_set:
                if i not in (parent_to_child_dict and explored_set):
                    discovered_nodes += [i]
            
            parent_to_child_dict[fringe] = discovered_nodes
            
            for i in discovered_nodes:
                state_cost = 0
                index = 0
                
                for value in i.config:
                    state_cost += calculate_manhattan_dist(index, value, i.n)
                    index += 1
                
                state_cost += calculate_total_cost(initial_state, i, parent_to_child_dict)
                frontier_set.put((state_cost, i))
                
            explored_set += [fringe]

    goal_state = fringe
    search_depth = 0
    route_taken += [fringe]
    
    while goal_state != initial_state:
        for i in parent_to_child_dict.keys():
            if goal_state in parent_to_child_dict[i]:
                route_taken += [i]
                search_depth += 1
                goal_state = i
                break
    
    route_taken.reverse()
    
    cost = calculate_total_cost(initial_state, fringe, parent_to_child_dict)
    
    for i in range(len(route_taken)-1):
        if route_taken[i].move_up() != None:
            if route_taken[i].move_up().config == route_taken[i+1].config:
                goal_path += ['Up']
        if route_taken[i].move_down() != None:
            if route_taken[i].move_down().config == route_taken[i+1].config:
                goal_path += ['Down']
        if route_taken[i].move_left() != None:
            if route_taken[i].move_left().config == route_taken[i+1].config:
                goal_path += ['Left']
        if route_taken[i].move_right() != None:
            if route_taken[i].move_right().config == route_taken[i+1].config:
                goal_path += ['Right']
    
    dead_end_nodes = []
    
    for i in parent_to_child_dict.keys():
        if parent_to_child_dict[i] == []:
            dead_end_nodes += [i]

    dead_end_nodes += [fringe]
    
    while not frontier_set.empty():
        dead_end_nodes += [frontier_set.get()]
    
    current_node = dead_end_nodes[0]
    max_search_depth = 0
    max_search_depth_temp = 0
    
    while dead_end_nodes != []:
        for i in parent_to_child_dict.keys():
            if current_node in parent_to_child_dict[i]:
                max_search_depth_temp += 1
                current_node = i
                break
        
        if current_node == initial_state:
            if max_search_depth_temp > max_search_depth:
                max_search_depth = max_search_depth_temp
                max_search_depth_temp = 0
            
            del dead_end_nodes[0]
            
            if dead_end_nodes != []:
                current_node = dead_end_nodes[0]
    
    expanded_set_num = len(explored_set)
    
    end_time = time.time()
    
    time_taken = end_time - start_time
    
    return writeOutput(goal_path, cost, expanded_set_num, search_depth, max_search_depth, time_taken)

def calculate_total_cost(initial_state, state, parent_child_dictionary):

    """calculate the total estimated cost of a state"""
    
    cost = 0
    
    while state != initial_state:
        for i in parent_child_dictionary.keys():
            if state in parent_child_dictionary[i]:
                cost += 1
                state = i
                break
    
    return cost

def calculate_manhattan_dist(idx, value, n):

    """calculatet the manhattan distance of a tile"""

    idx_value = value
    config = [1,2,3,4,5,6,7,8]
    config_test = tuple(config.insert(idx, 0))
    test_puzzle_state = PuzzleState(config_test, n)
    config_goal = tuple(config.insert(idx_value, 0))
    goal_puzzle_state = PuzzleState(config_goal, n)

    frontier_set = Q.Queue(0)
    frontier_set.put(test_puzzle_state)
    explored_set = []
    parent_to_child_dict = {}
    
    while not frontier_set.empty():
        fringe = frontier_set.get()
        
        if fringe.config == config_goal:
            break
        else:
            discovered_nodes = []
            expanded_set = fringe.expand()
            
            for i in expanded_set:
                if i not in frontier_set and explored_set:
                    discovered_nodes += [i]
                    frontier_set.put(i)
            
            parent_to_child_dict[fringe] = discovered_nodes
            explored_set += [fringe]

    manhattan_dist = calculate_total_cost(test_puzzle_state, goal_puzzle_state, parent_to_child_dict)
    
    return manhattan_dist

def test_goal(puzzle_state):

    """test the state is the goal state or not"""
    
    x = puzzle_state.dimension
    final_state = []
    
    for i in range(x*x):
        final_state += [i]
    
    final_state_tuple = tuple(final_state)
    
    if puzzle_state.config == final_state_tuple:
        return True
    else:
        return False

# Main Function that reads in Input and Runs corresponding Algorithm

def main():

    sm = sys.argv[1].lower()

    begin_state = sys.argv[2].split(",")

    begin_state = tuple(map(int, begin_state))

    size = int(math.sqrt(len(begin_state)))

    hard_state = PuzzleState(begin_state, size)

    if sm == "bfs":

        bfs_search(hard_state)

    elif sm == "dfs":

        dfs_search(hard_state)

    elif sm == "ast":

        A_star_search(hard_state)

    else:

        print("Enter valid command arguments !")

if __name__ == '__main__':

    main()