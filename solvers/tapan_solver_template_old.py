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


def solve(list_of_kingdom_names, starting_kingdom, adjacency_matrix, params=[]):
    """
    Write your algorithm here.
    Input:
        list_of_kingdom_names: An list of kingdom names such that node i of the graph corresponds to name index i in the list
        starting_kingdom: The name of the starting kingdom for the walk
        adjacency_matrix: The adjacency matrix from the input file

    Output:
        Return 2 things. The first is a list of kingdoms representing the walk, and the second is the set of kingdoms that are conquered
    """
    # variables to change: max_cycle, conquered_weight, kingdom_benefit, unconquered_benefit
    kingdom_costs = {}
    count = 0
    for i in list_of_kingdom_names:
        kingdom_costs[count] = adjacency_matrix[count][count]
        count += 1

    starting_index = list_of_kingdom_names.index(starting_kingdom)

    current_index = starting_index
    conquered_kingdoms = set()
    covered_kingdoms = set()
    closed_walk = [current_index]
    print(current_index)
    cost = 0

    # prevent cycles by making a last move counter
    last_move = 0
    max_cycle = 3
    # while some kingdoms are unconquered
    while (len(covered_kingdoms) != len(list_of_kingdom_names)):
        next_index, next_heuristic = next_kingdom(covered_kingdoms, adjacency_matrix, kingdom_costs, current_index)
        current_heuristic = heuristic(covered_kingdoms, adjacency_matrix, kingdom_costs, current_index)
        # if the current kindgom is conquered we move to next kingdom
        # if is optimal to move we move to next kingdom
        # if a kingdom has been visited before, we conquer it
        conquered_weight = 0
        if (current_index in covered_kingdoms):
            conquered_weight = 20
        if ((next_heuristic + conquered_weight > current_heuristic) & (last_move < max_cycle)):
            closed_walk += [next_index]
            if (adjacency_matrix[current_index][next_index] != 'x'):
                cost += adjacency_matrix[current_index][next_index]
            current_index = next_index
            last_move += 1
        # otherwise, if it is not beneficial, we will conquer the current node
        else:
            conquered_kingdoms.add(current_index)
            for i in range(len(adjacency_matrix[current_index])):
                if (adjacency_matrix[current_index][i] != 'x'):
                    covered_kingdoms.add(i)
            cost += kingdom_costs[current_index]
            last_move = 0
        # print(closed_walk)
        # print(conquered_kingdoms)
        # print(covered_kingdoms)

    cost_back, path_back = dijkstra(adjacency_matrix, current_index, starting_index)
    closed_walk += path_back
    cost += cost_back
    print(cost, closed_walk)

    # closed_walk = [0, 2, 4, 5, 7, 8, 7, 5, 4, 2, 0]
    # conquered_kingdoms = [0, 2, 5, 8]

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
    return closed_walk, conquered_kingdoms

# Tells the benefit of conquering the current kingdom
def heuristic(covered_kingdoms, adjacency_matrix, kingdom_costs, kingom_index):
    total = 0
    for i in range(len(adjacency_matrix[kingom_index])):
        # Ignores conquered kingdoms
        if i in covered_kingdoms:
            continue
        else:
            # subtract from the total the cost of conquering self
            if (i == kingom_index):
                total -= adjacency_matrix[kingom_index][i]
            # add to the total the cost of conquering/travel
            else:
                kingdom_benefit = 2
                total += kingdom_benefit
                if (adjacency_matrix[kingom_index][i] != 'x'):
                    total += adjacency_matrix[kingom_index][i]
                total += kingdom_costs[i]
    return total

# Tells which node is most optimal to travel to next and how optimal it is
def next_kingdom(covered_kingdoms, adjacency_matrix, kingdom_costs, kingom_index):
    max_val = -float('inf')
    index = 0
    for i in range(len(adjacency_matrix[kingom_index])):
        if (i == kingom_index):
            continue
        else:
            total = -float('inf')
            # lower the total if the next kingdom is far
            if (adjacency_matrix[kingom_index][i] != 'x'):
                total -= adjacency_matrix[kingom_index][i]
                # raise the total if the next kingdom has a high heuristic
                total += heuristic(covered_kingdoms, adjacency_matrix, kingdom_costs, i)
                # raise the total if the next kingdom is unconquered
                if kingom_index in covered_kingdoms:
                    unconquered_benefit = 5
                    total += unconquered_benefit
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


def solve_from_file(input_file, output_directory, params=[]):
    print('Processing', input_file)
    
    input_data = utils.read_file(input_file)
    number_of_kingdoms, list_of_kingdom_names, starting_kingdom, adjacency_matrix = data_parser(input_data)
    closed_walk, conquered_kingdoms = solve(list_of_kingdom_names, starting_kingdom, adjacency_matrix, params=params)

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

    for input_file in input_files:
        solve_from_file(input_file, output_directory, params=params)


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
