import os
import sys
import queue as que

# Test file
# http://graphonline.ru/en/?graph=jeTytMjWpxJZWqcU
adjacency_matrix = [['x', 5, 'x', 6, 'x', 'x', 'x', 'x', 11, 'x', 'x', 'x'], \
[5, 'x', 5, 3, 1, 'x', 'x', 'x', 'x', 'x', 'x', 'x'], \
['x', 5, 'x', 'x', 5, 1, 'x', 'x', 'x', 'x', 'x', 'x'], \
[6, 3, 'x', 'x', 3, 'x', 'x', 5, 6, 'x', 'x', 'x'], \
['x', 1, 5, 3, 'x', 'x', 'x', 7, 'x', 'x', 'x', 11], \
['x', 'x', 1, 'x', 'x', 'x', 3, 'x', 'x', 'x', 'x', 'x'], \
['x', 'x', 'x', 'x', 'x', 3, 'x', 'x', 'x', 'x', 'x', 11], \
['x', 'x', 'x', 5, 7, 'x', 'x', 'x', 'x', 'x', 5, 'x'], \
[11, 'x', 'x', 6, 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x'], \
['x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 7], \
['x', 'x', 'x', 'x', 'x', 'x', 'x', 5, 'x', 'x', 'x', 3], \
['x', 'x', 'x', 'x', 11, 'x', 11, 'x', 'x', 7, 3, 'x', ]]


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
    print(node_dict[end])
    return node_dict[end][1]

if __name__=="__main__":
    dijkstra(adjacency_matrix, 1, 1)