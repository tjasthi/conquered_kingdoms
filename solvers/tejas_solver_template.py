import os
import sys
sys.path.append('..')
sys.path.append('../..')
import argparse
import utils
from student_utils_sp18 import *
from operator import sub
import random

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

    # Initializes variables
    cost = float('inf')
    closed_walk = []
    conquered_kingdoms = []
    iterations = 1000
    counter = 0

    # Initializes variables for individual iteration
    for j in range(0, iterations):
         starting_node = list_of_kingdom_names.index(starting_kingdom) # Sets starting_node to starting_kingdom
         location = 0 + starting_node 
         temp_cost = 0
         visited = []
         surrendered = set()
         conquered = []
         nodes_connected = [0]*len(list_of_kingdom_names)
         for k in range(0, len(list_of_kingdom_names)):
            for l in range(0, len(list_of_kingdom_names)):
                if adjacency_matrix[k][l] != 'x':
                    nodes_connected[k] += 1
         # print(nodes_connected)
         random_list = [0, 1, 2, 3]

         while len(surrendered) != len(list_of_kingdom_names): # While all kingdoms have not surrendered...
             # print("Conquered:",conquered)
             # print("Location:",list_of_kingdom_names[location])
            # Possible moves are moves not looked at yet, kingdom moves are all possible moves (kingdoms adjacent to current location)
             possible_moves = []
             kingdom_moves = adjacency_matrix[location]

             # For each kingdom, if it connects another kingdom that has not already been conquered, add that kingdom to possible moves
             for i in range(0, len(list_of_kingdom_names)):
                 if kingdom_moves[i] != 'x':
                     if i == location:
                         if list_of_kingdom_names[i] not in conquered:
                             possible_moves.append(i) # If the location's kingdom isn't conquered, then conquering it is a possibility
                     else:
                        possible_moves.append(i) # All connected kingdoms are possibilities
             # print("Possible Moves:",possible_moves)
             move_cost = [0]*len(possible_moves)
             move_benefit = [0]*len(possible_moves)
             for move in possible_moves:
                # print(move)
                if move == location: # Conquers current kingdom
                    move_cost[possible_moves.index(move)] = adjacency_matrix[move][move]
                    counter = 0
                else: # Moves to another kingdom
                    move_cost[possible_moves.index(move)] = adjacency_matrix[location][move] #+ adjacency_matrix[move][move] 
                    counter += 1
                for i in range(0, len(list_of_kingdom_names)):
                    if list_of_kingdom_names[i] not in surrendered:
                        # print(adjacency_matrix[move][i])
                        if adjacency_matrix[move][i] != 'x':
                            move_benefit[possible_moves.index(move)] += adjacency_matrix[move][i] + adjacency_matrix[i][i]
             net_moves = list(map(sub, move_cost, move_benefit))
             # print ("Moves' Cost:",move_cost)
             # print("Moves' Benefit:",move_benefit)
             # print("Net Moves:",net_moves)
             # print("Optimal Move:",list_of_kingdom_names[possible_moves[net_moves.index(min(net_moves))]])
             decision = random.choice(random_list)
             if decision != 0:
                if counter < 1:
                    random_choice = possible_moves[net_moves.index(min(net_moves))] # Greedily selects one of the moves
                else:
                    random_choice = random.choice(possible_moves)
             else:
                random_choice = random.choice(possible_moves) # Randomly selects one of the moves
             # print(random_choice)
             # if counter < 1:
             #    random_choice = possible_moves[net_moves.index(min(net_moves))] # Greedily selects one of the moves
             # else:
             #    random_choice = random.choice(possible_moves)


             if random_choice == location: # If the selected move was to conquer the current kingdom...
                 # print("Conquering:",list_of_kingdom_names[location])
                 temp_cost += adjacency_matrix[location][random_choice] # Add cost of conquering current kingdom to this iteration's cost
                 conquered.append(list_of_kingdom_names[location]) # Add kingdom to list of kingdoms conquered
                 for i in range(0, len(list_of_kingdom_names)):
                     if adjacency_matrix[location][i] != 'x':
                         surrendered.add(list_of_kingdom_names[i]) # Add adjacent kingdoms to list of kingdoms surrendered

             else: # If the selected move was to move to another kingdom...
                 temp_cost += adjacency_matrix[location][random_choice] # Add cost of moving to kingdom to this iteration's cost
                 location = random_choice # Update location to the kingdom moved to
                 # print("Moving to:",list_of_kingdom_names[location])
                 visited.append(list_of_kingdom_names[random_choice]) # Add kingdom to list of kingdoms visited

         while location != starting_node: # Now that all kingdoms have surrendered, we return to starting node
             possible_moves = [] 
             kingdom_moves = adjacency_matrix[location] # Gets all possible moves from current kingdom
             if kingdom_moves[starting_node] != 'x': # If current location is connected to starting node...
                 temp_cost += adjacency_matrix[location][starting_node] # Adds cost of moving back to this iteration's total cost
                 location = starting_node # Update location to starting node
                 visited.append(list_of_kingdom_names[starting_node]) # Add starting node to list of visited kingdoms

             else: # If not currently at starting node...
                 for i in range(0, len(list_of_kingdom_names)): # For all kingdoms...
                     if kingdom_moves[i] != 'x':
                         if i == location:
                             if list_of_kingdom_names[i] not in conquered:
                                 possible_moves.append(i)
                         else:
                             possible_moves.append(i)
                 random_choice = random.choice(possible_moves) # Randomly select one of the possible moves
                 if random_choice == location:
                     x = 5 
                 else:
                     temp_cost += adjacency_matrix[location][random_choice]
                     location = random_choice
                     visited.append(list_of_kingdom_names[random_choice]) # Update location to kingdom moved to, add cost to this iteration's cost

         if temp_cost < cost: # If this iteration's cost is lowest yet, update final results to reflect this

             closed_walk = visited[:]
             conquered_kingdoms = conquered[:]
             cost = temp_cost

         print(j) # Print iteration number
         # print(closed_walk) # Print tour
         # print(conquered_kingdoms) # Print kingdoms directly conquered
         print(cost) # Print cost
         
    closed_walk = [starting_kingdom] + closed_walk
    return closed_walk, conquered_kingdoms

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

def next_kingdom(covered_kingdoms, adjacency_matrix, kingdom_costs, kingom_index):
    max_val = -float('inf')
    index = 0
    for i in range(len(adjacency_matrix[kingom_index])):
        if (i == kingom_index):
            continue
        else:
            total = 0
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

def djikstra(adjacency_matrix, start, goal):
    distances = []
    visited = []
    queue = []
    for kingdom in list_of_kingdom_names:
        distances.append(float('inf'))
        visited.append(0)
    distances[start] = 0
    for i in range(0, len(list_of_kingdom_names)):
        queue.append(1)
    while sum(queue) != 0:
        not_visited = []
        min_dist = float('inf')
        for i in range(0, len(queue)):
            if queue[i] == 1:
                not_visited.append(i)
        node = 0
        for num in not_visited:
            if distances[num] <= min_dist:
                min_dist = distances[num]
                node = num
        queue[node] = 0
        neighbors = []
        for i in range(0, len(adjacency_matrix[node])):
            if adjacency_matrix[node][i] != 'x':
                neighbors.append(i)
        for neighbor in neighbors:
            if distances[neighbor] > distances[node] + adjacency_matrix[node][neighbor]:
                distances[neighbor] = distances[node] + adjacency_matrix[node][neighbor]
    return distances[goal] 




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
