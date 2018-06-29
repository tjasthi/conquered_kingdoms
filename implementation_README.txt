CS 170 Project Report
I. Algorithms
For solving the inputs, we decided to implement a greedy strategy to find the optimal path and conquering set throughout the graph of kingdoms, and then used Djikstra’s algorithm to find the minimum-length path to return to the starting kingdom. We designed six different greedy heuristics using the distances and conquering costs of kingdoms, as well as the benefits of conquering those kingdoms (not having to travel to faraway kingdoms, not having to conquer expensive kingdoms, etc.):
 
1)   (Sum of Conquering Costs of Neighboring Non-Surrendered Kingdoms – Conquering Cost of Kingdom) / Number of Kingdoms that Surrender by Conquering
2)   Sum of Conquering Costs of Neighboring Non-Surrendered Kingdoms – Conquering Cost of Kingdom
3)   (Sum of Conquering Costs + Distances of Neighboring Non-Surrendered Kingdoms – Conquering Cost of Kingdom) / Number of Kingdoms that Surrender by Conquering
4)   Sum of Conquering Costs + Distances of Neighboring Non-Surrendered Kingdoms – Conquering Cost of Kingdom
5)   (Sum of Conquering Costs + 2*Distances of Neighboring Non-Surrendered Kingdoms – Conquering Cost of Kingdom) / Number of Kingdoms that Surrender by Conquering
6)   Sum of Conquering Costs + 2*Distances of Neighboring Non-Surrendered Kingdoms – Conquering Cost of Kingdom
 
We take twice the distances because that heuristic takes into account saving the distance there as well as the distance needed to travel back (although we may not necessarily go back). We apply of these heuristics, and then a number of iterations in which we randomly select one of these heuristics, to find the optimal path and conquering set for the graph. Our output will be the minimum-cost output out of all these iterations.
 
We also employed an alternative algorithm using a similar greedy algorithm with a varying heuristic. In this algorithm, we selected to either move to a neighbor or conquer our current kingdom based on a heuristic: a1 * (cost of neighbors) + a2 * (number of neighbors) - a3 * (distance to neighbors) - a4 * (cost of current kingdom). ai refers to a constant to weight each of these factors. We decided to move or conquer by comparing the heuristic of the optimal neighbor with that current kingdom and scaling the heuristic of the current kingdom by another constant a5. The constants values can be edited within the input list which is passed in to give varying outputs. 

With all this information, we ran this algorithm using varying combinations of varying weights to get different solutions. This is seen with the consecutive for loops to try all possible combinations of weights. To avoid cycles, we made a counter that forced a conquer or forced a move to the nearest unconquered node every 2 or 3 consecutive moves. For each input, we compared the cost of each combination and chose the tour that gave the most optimal solution.
This found the optimal solution for a lot of outputs and found a better solution than the other algorithm on inputs specifically: 5, 37, 38, 49, 67, 126, 184, 222, 230, 240, 241, 242, 273, 274, 275, 294, 296, 344, 371, 380, 383, 396, 400, 408, 409, 429, 431, 353, 354, 355, 474, 475, 476, 486, 487, 488, 508, 514, 515, 516, 517, 531, 537, 538, 566, 615, 630, 632, 638, 675, 685.
 
We used one of these two algorithms for the majority of the outputs, but there were a number of outputs in which it wasn’t optimal. Therefore, we hand-solved a few of the input files: 0, 3, 23, 39, 74, 101, 138, 139, 140, 196, 221, 328, 740.
 
II. Thought Process
We knew we were going to approach the problem with a greedy algorithm from the beginning, and we realized that since no single heuristic was ideal, we should develop several and add an element of randomness. We looked at attempting to reduce the problem to the Set Cover problem, which gave us the inspiration for dividing by the number of kingdoms acquired. We also looked at whether or not we wanted to apply greedy looking only one move ahead, or extend it to two. We used a combination of both, but found the two-step approach to be better in general.
In terms of dividing up the work, we all initially attempted to develop a solver. We worked on our personal laptops, meeting up to share our work and ideas. We then amalgamated our individual solvers into the final product, many of the heuristics implemented by Bill. After we ran this solver on the inputs, we saw several inputs that our solver did poorly on, so Tapan developed an alternative solver and Tejas went through input files we were doing particularly poorly on and solving them by hand. We also noticed a pattern of moving aimlessly was an issue in our solver for certain graphs, so we began to go through our output files and removing these extraneous patterns.
 
III. Conclusion
In conclusion, our approaches were a combination of greedy, randomized, and brute force algorithms to solve this problem. In cases in which these approaches proved ineffective, we would analyze the graphs on a case-by-case basis.

Graph Ideas:
Expensive leaf nodes to cause backtracking
Convince a group to take a suboptimal strategy by using the greedy heuristic to our advantage
Play with the size of the distances between nodes to encourage/discourage backtracking
Triangle inequality

Greedy:
Heuristic: Kingdom + distance to kingdom - adjacent kingdoms cost + adjacent distance costs
Reward for going to unconquered kingdoms
Consider looking 2+ steps ahead to give the opportunity to backtrack
Eliminate all nodes that only give kingdoms that have already surrendered
Consider a brute force approach because there’s no time constraints (random elements and choose the best opportunity)

Brute force the starting node by creating one case for each adjacent node as well as the starting node itself that runs the greedy solution after conquering that node

For optimizing our solution with brute force:
Choose a probability p such that with p probability we will use a greedy algorithm to select our next node and 1 - p probability we will randomly select a node adjacent to our current node to travel to. 
When we encounter our randomness case we have the option to travel to multiple random nodes n and continue our process from there.
If we go to multiple random nodes at this step we can run the algorithm through multiple branches or decide on a best node to travel to based on the heuristic.



Project Ideas
Play with the randomness to choose best iteration
Change the frequency of randomness
~90% greedy algorithm
Conquering vs moving
Count number of moves to balance conquering vs moving
Depending on the cycles in the path found, we may need to change the incentive for conquering
The incentive may need to change with the average edge weight
1 layer looking deep
Consider looking at 2 layers, then implement dijkstra’s to avoid being surrounded by conquered kingdoms
Dijkstra’s path specific version to find the nearest unconquered node 

Factors to consider in a heuristic 
Cost of conquering the current nodes
Cost of conquering all adjacent nodes
Focus on adjacent nodes that are unconquered
Cost of traveling to all adjacent nodes
Focus on adjacent nodes that are unconquered
Consider scaling distance by 2 for the travel there and travel back
Add a reward for forcing an adjacent node to surrender (optimal to force 5 nodes to surrender as opposed to 4)
Add a number for each heuristic
Divide the heuristic by the number of adjacent nodes

