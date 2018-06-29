import os
import sys
sys.path.append('..')
sys.path.append('../..')
import argparse
import utils
import queue as que
from student_utils_sp18 import *

"""
======================================================================
  Complete the following function.
======================================================================
"""


def solve(list_of_kingdom_names, starting_kingdom, adjacency_matrix, file, params=[]):
    """
    Write your algorithm here.
    Input:
        list_of_kingdom_names: An list of kingdom names such that node i of the graph corresponds to name index i in the list
        starting_kingdom: The name of the starting kingdom for the walk
        adjacency_matrix: The adjacency matrix from the input file

    Output:
        Return 2 things. The first is a list of kingdoms representing the walk, and the second is the set of kingdoms that are conquered
    """
    cost = float('inf')
    kingdom_costs = {}
    count = 0
    for i in list_of_kingdom_names:
        kingdom_costs[count] = adjacency_matrix[count][count]
        count += 1

    conquer_total = 0
    distance_total = 0
    edge_total = 0
    for i in range(len(list_of_kingdom_names)):
        for j in range(len(list_of_kingdom_names)):
            if (i == j):
                conquer_total += adjacency_matrix[i][j]
            else:
                if (adjacency_matrix[i][j] != 'x'):
                    distance_total += adjacency_matrix[i][j]
                    edge_total += 1
    conquer_average = conquer_total / len(list_of_kingdom_names)
    distance_average = distance_total / edge_total

    # make lists for the variables to change to find and optimal solution
    # variables to change: max_cycle, conquered_weight, distance_weight, kingdom_benefit, unconquered_benefit, unconquered_penalty
    max_cycle_list = [2, 3]
    conquered_weight_list = [conquer_average * 0.25, conquer_average]
    distance_weight_list = [distance_average * 0.25, distance_average]
    kingdom_benefit_list = [(conquer_average + distance_average) / 8, (conquer_average + distance_average) / 2]
    unconquered_benefit_list = [0, distance_average * 0.25]
    unconquered_penalty_list = [0]

    # iterate through all the variables
    for a in max_cycle_list:
        for b in conquered_weight_list:
            for c in distance_weight_list:
                for d in kingdom_benefit_list:
                    for e in unconquered_benefit_list:
                        for f in unconquered_penalty_list:
                            variable_list = [a] + [b] + [c] + [d] + [e] + [f]

                            cost1, closed_walk1, conquered_kingdoms1 = conquer_first(list_of_kingdom_names, starting_kingdom, adjacency_matrix, kingdom_costs, variable_list)
                            cost2, closed_walk2, conquered_kingdoms2 = conquer_free(list_of_kingdom_names, starting_kingdom, adjacency_matrix, kingdom_costs, variable_list)
                            
                            if ((cost1 < cost2) & (cost1 < cost)):
                                closed_walk = closed_walk1
                                conquered_kingdoms = conquered_kingdoms1
                                cost = cost1
                            elif ((cost2 <= cost1) & (cost2 < cost)):
                                closed_walk = closed_walk2
                                conquered_kingdoms = conquered_kingdoms2 
                                cost = cost2

    # converting from index to actual name
    for i in range(len(closed_walk)):
        k = closed_walk[i]
        item = list_of_kingdom_names[k]
        closed_walk[i] = item
    conquered_copy = conquered_kingdoms
    conquered_kingdoms = set()
    for i in conquered_copy:
        item = list_of_kingdom_names[i]
        conquered_kingdoms.add(item)

    print(cost)

    file.write(str(cost))
    file.write("\n")
    return closed_walk, conquered_kingdoms

# Forces a conquer on the first node and runs through the graph
def conquer_first(list_of_kingdom_names, starting_kingdom, adjacency_matrix, kingdom_costs, variable_list):
    max_cycle = variable_list[0]
    conquered_weight = variable_list[1]

    starting_index = list_of_kingdom_names.index(starting_kingdom)

    current_index = starting_index
    conquered_kingdoms = set()
    covered_kingdoms = set()
    closed_walk = [current_index]
    cost = 0

    # Conquer the first node
    conquered_kingdoms.add(current_index)   
    cost += kingdom_costs[current_index]
    covered_kingdoms = conquer_node(adjacency_matrix, current_index, covered_kingdoms)

    # prevent cycles by making a last move counter
    last_move = 0
    # while some kingdoms are unconquered
    while (len(covered_kingdoms) != len(list_of_kingdom_names)):
        # Look 2 jumps away for an unconquered kingdom
        if (not two_step(covered_kingdoms, adjacency_matrix, current_index)):
            # Move to the closest unconquered node
            next_index, nearest_cost, nearest_path = nearest_unconquered(adjacency_matrix, current_index, covered_kingdoms)
            current_index = next_index
            cost += nearest_cost
            closed_walk += nearest_path
            last_move = 0

        # If we moved several times before, we move to the nearest unconquered node
        if (last_move > max_cycle):
            next_index, nearest_cost, nearest_path = nearest_unconquered(adjacency_matrix, current_index, covered_kingdoms)
            current_index = next_index
            cost += nearest_cost
            closed_walk += nearest_path
            # We conquer this new node
            conquered_kingdoms.add(current_index)
            cost += kingdom_costs[current_index]
            covered_kingdoms = conquer_node(adjacency_matrix, current_index, covered_kingdoms)
            last_move = 0
            continue

        next_index, next_heuristic = next_kingdom(covered_kingdoms, adjacency_matrix, kingdom_costs, current_index, variable_list)
        current_heuristic = heuristic(covered_kingdoms, adjacency_matrix, kingdom_costs, current_index, variable_list)
        # if the current kindgom is conquered we move to next kingdom
        # if is optimal to move we move to next kingdom
        next_total = next_heuristic
        if (current_index in covered_kingdoms):
            next_total += conquered_weight
        if ((next_total > current_heuristic) | (current_index in conquered_kingdoms)):
            closed_walk += [next_index]
            cost += adjacency_matrix[current_index][next_index]
            current_index = next_index
            last_move += 1
        # otherwise, if it is not beneficial, we will conquer the current node
        else:
            cost += kingdom_costs[current_index]
            covered_kingdoms = conquer_node(adjacency_matrix, current_index, covered_kingdoms)
            conquered_kingdoms.add(current_index)
            last_move = 0
        # print(closed_walk)
        # print(conquered_kingdoms)
        # print(covered_kingdoms)

    # when all are conquered, we find the way back to the start node
    cost_back, path_back = dijkstra(adjacency_matrix, current_index, starting_index)
    closed_walk += path_back
    cost += cost_back
    return cost, closed_walk, conquered_kingdoms

# Forces a conquer on the first node and runs through the graph
def conquer_free(list_of_kingdom_names, starting_kingdom, adjacency_matrix, kingdom_costs, variable_list):
    max_cycle = variable_list[0]
    conquered_weight = variable_list[1]

    starting_index = list_of_kingdom_names.index(starting_kingdom)

    current_index = starting_index
    conquered_kingdoms = set()
    covered_kingdoms = set()
    closed_walk = [current_index]
    cost = 0

    # prevent cycles by making a last move counter
    last_move = 0
    # while some kingdoms are unconquered
    while (len(covered_kingdoms) != len(list_of_kingdom_names)):
        # Look 2 jumps away for an unconquered kingdom
        if (not two_step(covered_kingdoms, adjacency_matrix, current_index)):
            # Move to the closest unconquered node
            next_index, nearest_cost, nearest_path = nearest_unconquered(adjacency_matrix, current_index, covered_kingdoms)
            current_index = next_index
            cost += nearest_cost
            closed_walk += nearest_path
            last_move = 0

        # If we moved several times before, we move to the nearest unconquered node
        if (last_move > max_cycle):
            next_index, nearest_cost, nearest_path = nearest_unconquered(adjacency_matrix, current_index, covered_kingdoms)
            current_index = next_index
            cost += nearest_cost
            closed_walk += nearest_path
            # We conquer this new node
            conquered_kingdoms.add(current_index)
            cost += kingdom_costs[current_index]
            covered_kingdoms = conquer_node(adjacency_matrix, current_index, covered_kingdoms)
            last_move = 0
            continue

        next_index, next_heuristic = next_kingdom(covered_kingdoms, adjacency_matrix, kingdom_costs, current_index, variable_list)
        current_heuristic = heuristic(covered_kingdoms, adjacency_matrix, kingdom_costs, current_index, variable_list)
        # if the current kindgom is conquered we move to next kingdom
        # if is optimal to move we move to next kingdom
        next_total = next_heuristic
        if (current_index in covered_kingdoms):
            next_total += conquered_weight
        if ((next_total > current_heuristic) | (current_index in conquered_kingdoms)):
            closed_walk += [next_index]
            cost += adjacency_matrix[current_index][next_index]
            current_index = next_index
            last_move += 1
        # otherwise, if it is not beneficial, we will conquer the current node
        else:
            cost += kingdom_costs[current_index]
            covered_kingdoms = conquer_node(adjacency_matrix, current_index, covered_kingdoms)
            conquered_kingdoms.add(current_index)
            last_move = 0
        # print(closed_walk)
        # print(conquered_kingdoms)
        # print(covered_kingdoms)

    # when all are conquered, we find the way back to the start node
    cost_back, path_back = dijkstra(adjacency_matrix, current_index, starting_index)
    closed_walk += path_back
    cost += cost_back
    return cost, closed_walk, conquered_kingdoms

# Marks the adjacent nodes to the conquered node as a surrendered node
def conquer_node(adjacency_matrix, current_index, covered_kingdoms):
    for i in range(len(adjacency_matrix[current_index])):
        if (adjacency_matrix[current_index][i] != 'x'):
            covered_kingdoms.add(i)
    return covered_kingdoms

# Looks 2 steps away and returns true if there is an unconquered node
def two_step(covered_kingdoms, adjacency_matrix, current_index):
    for i in range(len(adjacency_matrix[current_index])):
        if ((adjacency_matrix[current_index][i] != 'x') & (i not in covered_kingdoms)):
            return True
        for j in range(len(adjacency_matrix[i])):
            if ((adjacency_matrix[i][j] != 'x') & (j not in covered_kingdoms)):
                return True
    return False

# Returns the path and cost to the nearest unconquered node
def nearest_unconquered(adjacency_matrix, start, covered_kingdoms):
    q = que.PriorityQueue()
    seen = set()

    # node_dict stores the distance and the path so far
    node_dict = {}
    for kingdom in range(len(adjacency_matrix[0])):
        node_dict[kingdom] = (float('inf'), [])

    # starting distance is 0 with an empty path
    q.put(start, 0)
    node_dict[start] = (0, [])

    while (not q.empty()): 
        node = q.get()
        if (not node in seen):
            seen.add(node)

            # new_node is the node label and child is the distance of the node
            new_node = 0
            for child in adjacency_matrix[node]:
                if (child != 'x'):
                    # replace the distance to new_node if we find a shorter path 
                    curr_distance = node_dict[new_node][0]
                    new_dist = node_dict[node][0] + child
                    if (curr_distance > new_dist):
                        q.put(new_node, new_dist)
                        node_dict[new_node] = (new_dist, node_dict[node][1] + [new_node])
                    else:
                        q.put(new_node, curr_distance)
                new_node += 1

    # Using the dictionary, finds the closest node and distance
    minimum_dist = float('inf')
    closest_node = start
    for i in range(len(adjacency_matrix[start])):
        if (i != start):
            if ((node_dict[start][0] < minimum_dist) & (i not in covered_kingdoms)):
                minimum_dist = node_dict[start][0]
                closest_node = i
    return closest_node, node_dict[closest_node][0], node_dict[closest_node][1]

# Tells the benefit of conquering the current kingdom
def heuristic(covered_kingdoms, adjacency_matrix, kingdom_costs, kingom_index, variable_list):
    kingdom_benefit = variable_list[3]

    total = 0
    for i in range(len(adjacency_matrix[kingom_index])):
        # Ignores conquered kingdoms
        if i in covered_kingdoms:
            continue
        else:
            # subtract from the total the cost of conquering self
            if (i == kingom_index):
                total -= kingdom_costs[i]
            # add to the total the cost of conquering/travel
            else:
                if (adjacency_matrix[kingom_index][i] != 'x'):
                    total += adjacency_matrix[kingom_index][i]
                    total += kingdom_benefit
                total += kingdom_costs[i]
    return total

# Tells which node is most optimal to travel to next and how optimal it is
def next_kingdom(covered_kingdoms, adjacency_matrix, kingdom_costs, kingom_index, variable_list):
    distance_weight = variable_list[2]
    unconquered_benefit = variable_list[4]
    unconquered_penalty = variable_list[5]

    max_val = -float('inf')
    index = -1
    for i in range(len(adjacency_matrix[kingom_index])):
        if (i == kingom_index):
            continue
        else:
            total = 0
            # lower the total if the next kingdom is far
            if (adjacency_matrix[kingom_index][i] != 'x'):
                total -= distance_weight * adjacency_matrix[kingom_index][i]
                
                # raise the total if the next kingdom has a high heuristic
                total += heuristic(covered_kingdoms, adjacency_matrix, kingdom_costs, i, variable_list)
                
                # raise the total if the next kingdom is unconquered
                if kingom_index in covered_kingdoms:
                    total += unconquered_benefit
                else:
                    total -= unconquered_penalty
                # replace with the next kingdom if it is more beneficial
                if (total > max_val):
                    max_val = total
                    index = i
    return index, max_val

# Returns the path from start to end, excluding start vertex and including end vertex
# Stores the shortest distances to every node in a dictionary
def dijkstra(adjacency_matrix, start, end):
    q = que.PriorityQueue()
    seen = set()

    # node_dict stores the distance and the path so far
    node_dict = {}
    for kingdom in range(len(adjacency_matrix[0])):
        node_dict[kingdom] = (float('inf'), [])

    # starting distance is 0 with an empty path
    q.put(start, 0)
    node_dict[start] = (0, [])

    while (not q.empty()): 
        node = q.get()
        if (not node in seen):
            seen.add(node)

            # new_node is the node label and child is the distance of the node
            new_node = 0
            for child in adjacency_matrix[node]:
                if (child != 'x'):
                    # replace the distance to new_node if we find a shorter path 
                    curr_distance = node_dict[new_node][0]
                    new_dist = node_dict[node][0] + child
                    if (curr_distance > new_dist):
                        q.put(new_node, new_dist)
                        node_dict[new_node] = (new_dist, node_dict[node][1] + [new_node])
                    else:
                        q.put(new_node, curr_distance)
                new_node += 1
    return node_dict[end][0], node_dict[end][1]

"""
======================================================================
   No need to change any code below this line
======================================================================
"""


def solve_from_file(input_file, output_directory, file, params=[]):
    print('Processing', input_file)
    
    input_data = utils.read_file(input_file)
    number_of_kingdoms, list_of_kingdom_names, starting_kingdom, adjacency_matrix = data_parser(input_data)
    closed_walk, conquered_kingdoms = solve(list_of_kingdom_names, starting_kingdom, adjacency_matrix, file, params=params)

    basename, filename = os.path.split(input_file)
    output_filename = utils.input_to_output(filename)
    output_file = f'{output_directory}/{output_filename}'
    if not os.path.exists(output_directory):
        os.makedirs(output_directory)
    utils.write_data_to_file(output_file, closed_walk, ' ')
    utils.write_to_file(output_file, '\n', append=True)
    utils.write_data_to_file(output_file, conquered_kingdoms, ' ', append=True)

def solve_all(input_directory, output_directory, params=[]):
    input_files = utils.get_files_with_extension(input_directory, 'in')
    file = open("costs.csv", "w")

    for input_file in input_files:
        solve_from_file(input_file, output_directory, file, params=params)
    file.close()

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Parsing arguments')
    parser.add_argument('--all', action='store_true', help='If specified, the solver is run on all files in the input directory. Else, it is run on just the given input file')
    parser.add_argument('input', type=str, help='The path to the input file or directory')
    parser.add_argument('output_directory', type=str, nargs='?', default='.', help='The path to the directory where the output should be written')
    parser.add_argument('params', nargs=argparse.REMAINDER, help='Extra arguments passed in')
    args = parser.parse_args()
    output_directory = args.output_directory
    if args.all:
        input_directory = args.input
        solve_all(input_directory, output_directory, params=args.params)
    else:
        input_file = args.input
        solve_from_file(input_file, output_directory, params=args.params)
