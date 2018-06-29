import os
import sys
sys.path.append('..')
sys.path.append('../..')
import argparse
import utils
from student_utils_sp18 import *
import random
import queue as que

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





    # cost = 0
    # visited = []
    # surrendered = set()
    # conquered = []
    # location = list_of_kingdom_names.index(starting_kingdom)
    # while len(surrendered) != len(list_of_kingdom_names):
    #     move = greedy_move2(list_of_kingdom_names, location, adjacency_matrix, conquered, surrendered)
    #     if move != location:
    #         cost += adjacency_matrix[location][move]
    #         cost += adjacency_matrix[move][move]
    #         location = move
    #         for neighbor in get_neighbors(location, adjacency_matrix):
    #             surrendered.add(neighbor)
    #         conquered.append(move)
    #         visited.append(move)
    #     else:
    #         cost += adjacency_matrix[move][move]
    #         for neighbor in get_neighbors(location, adjacency_matrix):
    #             surrendered.add(neighbor)            
    #         conquered.append(move)
    # print(visited)
    # print(conquered)
    # cost += djikstra(adjacency_matrix, location, list_of_kingdom_names.index(starting_kingdom))
    # print(cost)

    
    cost = 0
    visited = []
    surrendered = set()
    conquered = []
    location = list_of_kingdom_names.index(starting_kingdom)
    while len(surrendered) != len(list_of_kingdom_names):
        move = two_layer_greedy(list_of_kingdom_names, location, adjacency_matrix, conquered, surrendered)
        if len(move) == 1:
            move = move[0]
            if move != location:
                cost += adjacency_matrix[location][move]
                cost += adjacency_matrix[move][move]
                location = move
                for neighbor in get_neighbors(location, adjacency_matrix):
                    surrendered.add(neighbor)
                conquered.append(move)
                visited.append(move)
            else:
                cost += adjacency_matrix[move][move]
                for neighbor in get_neighbors(location, adjacency_matrix):
                    surrendered.add(neighbor)            
                conquered.append(move)           
        elif len(move) == 2:
            move0 = move[0]
            move1 = move[1]
            cost += adjacency_matrix[location][move0]
            cost += adjacency_matrix[move0][move1]
            cost += adjacency_matrix[move1][move1]
            location = move1
            for neighbor in get_neighbors(move1, adjacency_matrix):
                surrendered.add(neighbor)
            conquered.append(move1)
            visited.append(move0)
            visited.append(move1)
        elif len(move) == 3:
            cost += move[2]
            location = move[1]
            for node in dijkstra(adjacency_matrix, location, move[1]):
                visited.append(node)

    print(visited)
    print(conquered)
    cost += dijkstra_distance(adjacency_matrix, location, list_of_kingdom_names.index(starting_kingdom))
    print(cost)





"""
Brute Force
"""



    # cost = float('inf')
    # closed_walk = []
    # conquered_kingdoms = []
    # iterations = 1000000
    # djikstra_distance = float('inf')
    # djikstra_node = 0
    # djikstra_conquered = []
    # for j in range(0, iterations):
    #     starting_node = list_of_kingdom_names.index(starting_kingdom)
    #     location = 0 + starting_node
    #     temp_cost = 0
    #     visited = []
    #     surrendered = set()
    #     conquered = []
    #     while len(surrendered) != len(list_of_kingdom_names):
    #         possible_moves = []
    #         kingdom_moves = adjacency_matrix[location]
    #         for i in range(0, len(list_of_kingdom_names)):
    #             if kingdom_moves[i] != 'x':
    #                 if i == location:  
    #                     if list_of_kingdom_names[i] not in conquered:
    #                         possible_moves.append(i)
    #                 else:
    #                     possible_moves.append(i)
    #         random_choice = random.choice(possible_moves)
    #         if random_choice == location:
    #             temp_cost += adjacency_matrix[location][random_choice]
    #             conquered.append(list_of_kingdom_names[location])
    #             for i in range(0, len(list_of_kingdom_names)):
    #                 if adjacency_matrix[location][i] != 'x':
    #                     surrendered.add(list_of_kingdom_names[i])
    #         else:
    #             temp_cost += adjacency_matrix[location][random_choice]
    #             location = random_choice
    #             visited.append(list_of_kingdom_names[random_choice])
    #     if djikstra(adjacency_matrix, location, starting_node) + temp_cost < djikstra_distance:
    #         djikstra_distance = djikstra(adjacency_matrix, location, starting_node) + temp_cost
    #         djikstra_node = location
    #         djikstra_conquered = conquered
    #     while location != starting_node:
    #         possible_moves = []
    #         kingdom_moves = adjacency_matrix[location]
    #         if kingdom_moves[starting_node] != 'x':
    #             temp_cost += adjacency_matrix[location][starting_node]
    #             location = starting_node
    #             visited.append(list_of_kingdom_names[starting_node])
    #         else:
    #             for i in range(0, len(list_of_kingdom_names)):
    #                 if kingdom_moves[i] != 'x':
    #                     if i == location: 
    #                         if list_of_kingdom_names[i] not in conquered:
    #                             possible_moves.append(i)
    #                     else:
    #                         possible_moves.append(i)
    #             random_choice = random.choice(possible_moves)
    #             if random_choice == location:
    #                 x = 5
    #             else:
    #                 temp_cost += adjacency_matrix[location][random_choice]
    #                 location = random_choice
    #                 visited.append(list_of_kingdom_names[random_choice])        
    #     if temp_cost < cost:

    #         closed_walk = visited[:] 
    #         conquered_kingdoms = conquered[:]
    #         cost = temp_cost

    #     print(j)
    # print(closed_walk)
    # print(conquered_kingdoms)
    # print(djikstra_distance)
    # print("Location", djikstra_node)
    # print("Djikstra conquered", djikstra_conquered)
    # print(cost)
    # return closed_walk, conquered_kingdoms



def get_possible_moves(location, adjacency_matrix):
    possible_moves = []
    for i in range(0, len(adjacency_matrix[location])):
        if adjacency_matrix[location][i] != 'x':
            possible_moves.append(i)
    return possible_moves


def dijkstra_distance(adjacency_matrix, start, goal):
    distances = []
    visited = []
    queue = []
    for kingdom in adjacency_matrix:
        distances.append(float('inf'))
        visited.append(0)
    distances[start] = 0
    for i in range(0, len(adjacency_matrix)):
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
def dijkstra(adjacency_matrix, start, end):
    q = que.PriorityQueue()
    seen = set()
    node_dict = {}

    for kingdom in range(len(adjacency_matrix[0])):
        node_dict[kingdom] = (float('inf'), [])

    q.put(start, 0)
    node_dict[start] = (0, [])

    while (not q.empty()): 
        node = q.get()
        if (not node in seen):
            seen.add(node)

            new_node = 0
            for child in adjacency_matrix[node]:
                if (child != 'x'):
                    curr_distance = node_dict[new_node][0]
                    new_dist = node_dict[node][0] + child
                    if (curr_distance > new_dist):
                        q.put(new_node, new_dist)
                        #print(q.get())
                        node_dict[new_node] = (new_dist, node_dict[node][1] + [new_node])
                    else:
                        q.put(new_node, curr_distance)
                new_node += 1
    return node_dict[end][1]    


# def greedy_move1(list_of_kingdom_names, location, adjacency_matrix, conquered_kingdoms, surrendered):
#     best_cost = float('inf')
#     best_move = -1
#     moves = get_possible_moves(location, adjacency_matrix)
#     for move in moves:
#         temp_cost = 0
#         if list_of_kingdom_names[move] not in conquered_kingdoms:
#             temp_cost += adjacency_matrix[location][move] 
#             temp_cost += adjacency_matrix[move][move]
#             surrendering_kingdoms = get_possible_moves(move, adjacency_matrix)
#             net_gain = []
#             for surrender in surrendering_kingdoms:
#                 if surrender not in surrendered:
#                     net_gain.append(surrender)
#             for item in net_gain:
#                 temp_cost -= adjacency_matrix[move][item]
#                 temp_cost -= adjacency_matrix[item][item
#             if temp_cost < best_cost:
#                 best_cost = temp_cost
#                 best_move = move



def get_neighbors(location, adjacency_matrix):
    neighbors = []
    for i in range(0, len(adjacency_matrix[location])):
        if adjacency_matrix[location][i] != 'x':
                neighbors.append(i)
    return neighbors

#conquer most number of nodes possible (must conquer at every step)
def greedy_move2(list_of_kingdom_names, location, adjacency_matrix, conquered_kingdoms, surrendered):
    num_nodes = 0
    best_move = -1
    moves = get_possible_moves(location, adjacency_matrix)
    for move in moves:
        count = 0
        if move not in conquered_kingdoms:
            neighbors = get_possible_moves(move, adjacency_matrix)
            for neighbor in neighbors:
                if neighbor not in surrendered:
                    count += 1
        if count > num_nodes:
            num_nodes = count
            best_move = move
    return best_move



def two_layer_greedy(list_of_kingdom_names, location, adjacency_matrix, conquered_kingdoms, surrendered):
    moves = get_possible_moves(location, adjacency_matrix)
    best_efficiency = float('-inf')
    best_move = [-1]
    # print("Moves", moves)
    # print("Conquered", conquered_kingdoms)
    # print("Surrendered", surrendered)
    total_number = 0
    total_surrendered = 0
    for move in moves:
        total_number += 1
        if move in surrendered:
            total_surrendered += 1
        second_moves = get_possible_moves(move, adjacency_matrix)
        for move in second_moves:
            total_number += 1
            if move in surrendered:
                total_surrendered += 1

    if total_number != total_surrendered:
        for move in moves:
            if move not in conquered_kingdoms:
                temp_efficiency = efficiency(location, [move], adjacency_matrix, surrendered)
                if temp_efficiency > best_efficiency:
                    best_efficiency = temp_efficiency
                    best_move = [move]
            second_moves = get_possible_moves(move, adjacency_matrix)
            for second_move in second_moves:
                if second_move not in conquered_kingdoms and second_move != move:
                    temp_efficiency = efficiency(move, [move, second_move], adjacency_matrix, surrendered)
                    if temp_efficiency > best_efficiency:
                        best_efficiency = temp_efficiency
                        best_move = [move, second_move]
    else:
        non_surrendered = []
        for i in range(0, len(list_of_kingdom_names)):
            if i not in surrendered:
                non_surrendered.append(i)
        closest_unsurrendered_node = -1
        closest_distance = float('inf')
        for node in non_surrendered:
            node_distance = dijkstra_distance(adjacency_matrix, location, node)
            if node_distance < closest_distance:
                closest_distance = node_distance
                closest_unsurrendered_node = node
        best_move = [-1, closest_unsurrendered_node, closest_distance]

    return best_move





def efficiency(location, goal, adjacency_matrix, surrendered):
    if len(goal) == 1:
        if location == goal[0]:
            cost = adjacency_matrix[goal[0]][goal[0]]
            gains = 0
            num_surrendered = 1
            neighbors = get_neighbors(goal[0], adjacency_matrix)
            for neighbor in neighbors:
                if neighbor not in surrendered:
                    num_surrendered += 1
                    gains += adjacency_matrix[neighbor][neighbor]
            return (gains - cost) / num_surrendered
        else:
            cost = adjacency_matrix[location][goal[0]] + adjacency_matrix[goal[0]][goal[0]]
            gains = 0
            num_surrendered = 1
            neighbors = get_neighbors(goal[0], adjacency_matrix)
            for neighbor in neighbors:
                if neighbor not in surrendered:
                    num_surrendered += 1
                    gains += adjacency_matrix[neighbor][neighbor]
            return (gains - cost) / num_surrendered

    else:
        cost = adjacency_matrix[location][goal[0]] + adjacency_matrix[goal[0]][goal[1]] + adjacency_matrix[goal[1]][goal[1]]
        gains = 0
        num_surrendered = 1
        neighbors = get_neighbors(goal[1], adjacency_matrix)
        for neighbor in neighbors:
            if neighbor not in surrendered:
                num_surrendered += 1
                gains += adjacency_matrix[neighbor][neighbor]
        return (gains - cost) / num_surrendered










    











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
