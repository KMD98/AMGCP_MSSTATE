# coding=utf-8
"""
This file is your main submission that will be graded against. Only copy-paste
code on the relevant classes included here. Do not add any classes or functions
to this file that are not part of the classes that we want.
"""

import heapq
import os
import pickle
import math
#Use heapq library to implement priority queue
import heapq

class PriorityQueue(object):
    """
    A queue structure where each element is served in order of priority.

    Elements in the queue are popped based on the priority with higher priority
    elements being served before lower priority elements.  If two elements have
    the same priority, they will be served in the order they were added to the
    queue.

    Traditionally priority queues are implemented with heaps, but there are any
    number of implementation options.

    (Hint: take a look at the module heapq)

    Attributes:
        queue (list): Nodes added to the priority queue.
    """

    def __init__(self):
        """Initialize a new Priority Queue."""

        self.queue = []
        self.count = 0

    def pop(self):
        """
        Pop top priority node from queue.

        Returns:
            The node with the highest priority.
        """

        # TODO: finish this function!
        #implement most functions with heapq because it satisfies O(logn) pop
        # NEED TO CREATE AN ALGO THAT POP BASED ON INSERTION ORDER WHEN COST ARE THE SAME. Make sure the tuple
        # that is popped goes from three element to two element
        three_tuple = heapq.heappop(self.queue) # three tuple as the form (cost, priority, key) with O(log n ) complexity. Popped elements are removed
        two_tuple = (three_tuple[0], three_tuple[2]) #grab only the first element and last element, (cost,key)
        return two_tuple #return cost and key

    def remove(self, node):
        """
        Remove a node from the queue.

        Hint: You might require this in ucs. However, you may
        choose not to use it or to define your own method.

        Args:
            node (tuple): The node to remove from the queue.
        """
        #We will use remove in UCS search. Note that this will be called after assuring that the node already
        #exists in the queue and it has a higher path cost than the one we are trying to replace. In short, this
        #is the function we will call when we are trying to replace identical nodes with different COSTS.
        #IMPORTANT: A QUEUE SHOULD NEVER HAVE IDENTICAL KEYS/element[-1] or INSERTION ORDER; IT MAY HAVE IDENTICAL COST
        #BUT NOT SECOND OR THIRD ELEMENT. WE ESTABLISH THIS BY ONLY ADDING TO QUEUE/FRONTIER WHEN NODE KEY IS NOT IN
        #EXPLORED AND FRONTIER
        for element in self.queue: #O(n) complexity
            if element[2] == node[1]: #compare the key of node being passed in with key of element in queue
                self.queue.remove(element) #remove the element when found

    def __iter__(self):
        """Queue iterator."""

        return iter(sorted(self.queue))

    def __str__(self):
        """Priority Queue to string."""

        return 'PQ:%s' % self.queue

    def append(self, node):
        """
        Append a node to the queue.

        Args:
            node: Comparable Object to be added to the priority queue.
        """

        # TODO: finish this function!
        #raise NotImplementedError
        #We can use heap insert because it is O(1)
        self.count += 1
        actual_node = (node[0],self.count,node[1]) #create a three-element tuple that has priority order as second element
        heapq.heappush(self.queue, actual_node)

    def __contains__(self, node):
        """
        Containment Check operator for 'in'

        Args:
            key: The key to check for in the queue.

        Returns:
            True if key is found in queue, False otherwise.
        """
        in_queue = False
        lesser_flag = False
        for element in self.queue:
            if element[-1] == node[1]:
                in_queue = True
                if (element[0]> node[0]):
                    lesser_flag = True
                    self.queue.remove(element)
        return in_queue, lesser_flag

    def __eq__(self, other):
        """
        Compare this Priority Queue with another Priority Queue.

        Args:
            other (PriorityQueue): Priority Queue to compare against.

        Returns:
            True if the two priority queues are equivalent.
        """

        return self.queue == other.queue

    def size(self):
        """
        Get the current size of the queue.

        Returns:
            Integer of number of items in queue.
        """

        return len(self.queue)

    def clear(self):
        """Reset queue to empty (no nodes)."""

        self.queue = []

    def top(self):
        """
        Get the top item in the queue.

        Returns:
            The first item stored in the queue.
        """

        return self.queue[0]


def breadth_first_search(graph, start, goal):
    """
    Warm-up exercise: Implement breadth-first-search.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start (str): Key for the start node.
        goal (str): Key for the end node.

    Returns:
        The best path as a list from the start and goal nodes (including both).
    """

    # TODO: finish this function!
    #We can implement BFS by using list comprehension
    if start == goal:
        return []
    #initialize
    frontier_and_path = []
    real_frontier = []
    explored_nodes = []
    neigh_sorted = []
    #Initialize frontier with start and the saved front + path
    frontier_and_path.append(start)
    real_frontier.append(start)
    #As long as frontier is not empty, perform search
    while len(frontier_and_path) > 0:
        #pop the first element in the frontier where first element will always be the lowest depth in the list
        #the first element can be a single node or the most recent lowest cost path, assuming that the cost function
        #is a nondecreasing function with respect to d
        path_tracker = frontier_and_path.pop(0)
        end_node = path_tracker[-1] #return the last element inside that single node list or path list
        explored_nodes.append(end_node) #keep track of all explored nodes
        real_frontier.remove(end_node) #remove the about to be expanded node because it is no longer the real frontier
        #Now expand the nearest neighbor in whatever order the explorable_graph.py generates. The document show that
        #the class graph has function .neighbors that takes in a node and return all of its neighbors.
        neighbors = graph.neighbors(end_node)
        neigh_sorted.clear() #clear the temporary list for sorted neighbors
        for neighbor in neighbors: #append the neighbors to a list
            neigh_sorted.append(neighbor)
        neigh_sorted.sort() #sort the list of neighbors in order
        for neighbor in neigh_sorted: #iterate through all neighbors
            if neighbor not in explored_nodes and neighbor not in real_frontier:
                temporary_tracker = list(path_tracker) #store pre-expanded path tracked into a temp
                temporary_tracker.append(neighbor) #expand the temporary path tracker
                if neighbor == goal:
                    return temporary_tracker #this will return the path tracked for the node that is the goal
                else:
                    frontier_and_path.append(temporary_tracker) #if goal is not found, put the new path into frontiers path
                    temp_value = frontier_and_path[-1]
                    real_frontier.append(temp_value[-1]) #add nodes in the frontier


def uniform_cost_search(graph, start, goal):
    """
    Warm-up exercise: Implement uniform_cost_search.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start (str): Key for the start node.
        goal (str): Key for the end node.

    Returns:
        The best path as a list from the start and goal nodes (including both).
    """

    # TODO: finish this function!
    parent_dict = {}
    parent_dict[start] = (0,None) #for any child, store their cost from start and their parent. Initialize start state
    optimal_path = []
    frontier = PriorityQueue()
    frontier.append((0,start))
    explored_nodes = []
    parent_node = None
    while frontier.size() > 0:
        if start == goal:
            return []
        node = frontier.pop()
        if node[1] == goal:
            temp = node[1] #store the node into temp so we can expand the "tree" for the optimal path
            parent_node = parent_dict[temp][1] #grab the parent of the node
            while parent_node: #keep going until there is no more parent or we are at the root/start
                optimal_path.append(temp) #append current node to optimal path
                temp = parent_node #store the parent of the current evaluated node into temp
                parent_node = parent_dict[temp][1] #find the parent of the parent of this node
            optimal_path.append(temp) #append the root
            return optimal_path[::-1] #invert the list and return the optimal path

        elif node[1] not in explored_nodes:
            parentpathCost, current_node = node #get the current node name and the parent path cost
            explored_nodes.append(current_node)
            for child in graph.neighbors(current_node):
                pathcost = parentpathCost + graph.get_edge_weight(current_node, child) #get the total cost from start to the child
                in_frontier_flag, lesser_flag = frontier.__contains__((pathcost, child)) #if child already in frontier and the cost of child is lesser than the one in frontier then removed
                if child not in parent_dict or pathcost < parent_dict[child][0]: #if the child in the discovered path has a higher cost than the new path replaced
                    parent_dict[child] = (pathcost,current_node) #keeping track of all children and their respective parent path cost
                if child not in explored_nodes and not in_frontier_flag:
                    frontier.append((pathcost, child)) #push into frontier if child is not in frontier AND explored
                elif child not in explored_nodes and in_frontier_flag and lesser_flag: #if the child is already in explored then the lowest path to that child is already found. Do not add to frontier to explore.
                    frontier.append((pathcost, child)) #push into frontier if child IS IN frontier AND the child has lesser cost than the child already in frontier

def null_heuristic(graph, v, goal):
    """
    Null heuristic used as a base line.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        v (str): Key for the node to calculate from.
        goal (str): Key for the end node to calculate to.

    Returns:
        0
    """

    return 0


def euclidean_dist_heuristic(graph, v, goal):
    """
    Warm-up exercise: Implement the euclidean distance heuristic.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        v (str): Key for the node to calculate from.
        goal (str): Key for the end node to calculate to.

    Returns:
        Euclidean distance between `v` node and `goal` node
    """

    # TODO: finish this function!
    if v == goal:
        return 0
    v_pos = graph.nodes[v]['pos'] #these are tuples
    goal_pos = graph.nodes[goal]['pos'] #these are tuples
    distance = math.sqrt(math.pow((goal_pos[0] - v_pos[0]), 2) + math.pow((goal_pos[1] - v_pos[1]), 2))
    return distance

def a_star(graph, start, goal, heuristic=euclidean_dist_heuristic):
    """
    Warm-up exercise: Implement A* algorithm.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start (str): Key for the start node.
        goal (str): Key for the end node.
        heuristic: Function to determine distance heuristic.
            Default: euclidean_dist_heuristic.

    Returns:
        The best path as a list from the start and goal nodes (including both).
    """

    # TODO: finish this function!
    parent_dict = {}
    parent_dict[start] = (0,None) #for any child, store their cost from start and their parent. Initialize start state
    optimal_path = []
    frontier = PriorityQueue()
    h_initial = euclidean_dist_heuristic(graph,start,goal)
    frontier.append((0 + h_initial, start))
    explored_nodes = []
    parent_node = None
    while frontier.size() > 0:
        if start == goal:
            return []
        node = frontier.pop()
        if node[1] == goal:
            temp = node[1] #store the node into temp so we can expand the "tree" for the optimal path
            parent_node = parent_dict[temp][1] #grab the parent of the node
            while parent_node: #keep going until there is no more parent or we are at the root/start
                optimal_path.append(temp) #append current node to optimal path
                temp = parent_node #store the parent of the current evaluated node into temp
                parent_node = parent_dict[temp][1] #find the parent of the parent of this node
            optimal_path.append(temp) #append the root
            return optimal_path[::-1] #invert the list and return the optimal path

        elif node[1] not in explored_nodes:
            current_node = node[1] #get the current node name from frontier BUT DO NOT USE THE PATHCOST FROM FRONTIER BECAUSE IT HAS h
            explored_nodes.append(current_node)
            parentpathCost = parent_dict[current_node][0] #get the pathcost g to calculate parentpathcost from parent_dict because it only stores g and parent of current node
            for child in graph.neighbors(current_node):
                g = parentpathCost + graph.get_edge_weight(current_node, child) #get the total cost from start to the child
                h = euclidean_dist_heuristic(graph, child, goal) #heuristic estimate
                in_frontier_flag, lesser_flag = frontier.__contains__((g+h, child)) #if child already in frontier and the estimtaed cost of child, g+h is lesser than the one in frontier then removed
                if child not in parent_dict or g < parent_dict[child][0]: #if the child in the discovered path,g, has a higher cost than the new path,g, replaced
                    parent_dict[child] = (g,current_node) #keeping track of all children and their respective parent path cost, g
                if child not in explored_nodes and not in_frontier_flag:
                    frontier.append((g+h, child)) #push into frontier if child is not in frontier AND explored
                elif child not in explored_nodes and in_frontier_flag and lesser_flag:
                    # push into frontier if child IS IN frontier AND the child has lesser cost than the child already in
                    # frontier. Note that if this execute, the child with the higher cost in the frontier is already removed
                    # by the previous __contain__ call
                    frontier.append((g+h, child))


def bidirectional_ucs(graph, start, goal):
    """
    Exercise 1: Bidirectional Search.
    See README.md for exercise description.
    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start (str): Key for the start node.
        goal (str): Key for the end node.
    Returns:
        The best path as a list from the start and goal nodes (including both).
    """

    # TODO: finish this function!
    # raise NotImplementedError

    if start == goal:
        return []
    explored_start, explored_goal = {}, {}
    parent_dict_start, parent_dict_goal = {}, {} #store path cost and parent. Act as a hashtable
    optimal_path = []
    parent_node = None

    intersected_node = (None, float('inf'))

    frontier_start = PriorityQueue()
    frontier_goal = PriorityQueue()
    frontier_start.append((0, start))
    frontier_goal.append((0, goal))
    parent_dict_start[start] = (0, None)
    parent_dict_goal[goal] = (0, None)

    while frontier_start.size() > 0 or frontier_goal.size() > 0:
        if start == goal:
            return []
        # from start
        frontier_cost, current_node = frontier_start.pop()
        if current_node in explored_goal:
            explored_goal = merge_explored_frontier(explored_goal, frontier_goal, parent_dict_goal) #Union frontier and explore to takecare of crossovers
            explored_start[current_node] = parent_dict_start.pop(current_node) #Add current node to explored because it was explored by the goal section
            intersected_node = (current_node, frontier_cost + explored_goal[current_node][0]) #package the current intersected node with its complete path cost
            intersected_node = find_best_intersection(intersected_node, explored_goal, explored_start) #compare the found total cost with all the other crossovers
            break #exit while loop because we found an intersection, whatever that will intersect after this condition execution will not be more optimal that what is already found.
        elif current_node not in explored_start:
            #the parent dictionary values will be popped into explored start instead of keeping its value to save memory space
            explored_start[current_node] = parent_dict_start.pop(current_node) #explored start will always get the lowest cost path stored due to conditions set in for loop
            parent_cost = explored_start[current_node][0]
            for child in graph.neighbors(current_node):
                pathcost = parent_cost + graph.get_edge_weight(current_node,child)  # get the total cost from current to the child
                if child in parent_dict_start and pathcost < parent_dict_start[child][0]:  # if the child in the discovered path has a higher cost than the new path replaced
                    del parent_dict_start[child]  # delete because we dont want to consider it again when storing into explore.
                if child not in explored_start and child not in parent_dict_start: # if a child is already explored, and it is not in parent_dict_start then that means its shortest path already been found due to it being explored. We do not append it to frontier.
                    parent_dict_start[child] = (pathcost, current_node) #add to parent_dict because of the previous delete or not present in dict
                    frontier_start.append((pathcost, child))  # push into frontier if child is not in explored. The frontier will have two of the same node but that doesnt matter because
                    #the nodes WITHOUT the lowest cost will never be explored due to the lowest cost node already in explored. This change is better
                    #than the implementation from single A* and UCS because we do not call _contain_, which is complexity O(n) whereas pop is only O(logn)

        # from goal
        frontier_cost, current_node = frontier_goal.pop()
        if current_node in explored_start:
            explored_start = merge_explored_frontier(explored_start, frontier_start, parent_dict_start)
            explored_goal[current_node] = parent_dict_goal.pop(current_node)
            intersected_node = (current_node, frontier_cost + explored_start[current_node][0])
            intersected_node = find_best_intersection(intersected_node, explored_start, explored_goal)
            break
        elif current_node not in explored_goal:
            explored_goal[current_node] = parent_dict_goal.pop(current_node)
            parent_cost = explored_goal[current_node][0]
            for child in graph.neighbors(current_node):
                pathcost = parent_cost + graph.get_edge_weight(current_node, child)
                if child in parent_dict_goal and pathcost < parent_dict_goal[child][0]:
                    del parent_dict_goal[child]
                if child not in explored_goal and child not in parent_dict_goal:
                    parent_dict_goal[child] = (pathcost, current_node)
                    frontier_goal.append((pathcost, child))

    #return the optimal path
    temp_node = intersected_node[0]
    parent_node = explored_start[temp_node][1]
    while parent_node:
        optimal_path.append(temp_node)
        temp_node = parent_node
        parent_node = explored_start[temp_node][1]
    optimal_path.append(temp_node)
    optimal_path = optimal_path[::-1] #invert it
    parent_node = explored_goal[intersected_node[0]][1] #now just append the rest
    while parent_node:
        optimal_path.append(parent_node)
        parent_node = explored_goal[parent_node][1]
    return optimal_path



#Compare the best meet with other best meets by comparing it to the cost of intersected nodes of start_explored and goal_explored
def find_best_intersection(intersected_node, merged_explored, current_explored):
    temp_cost = float('inf')
    for node in current_explored:
        if node in merged_explored:
            temp_cost = current_explored[node][0] + merged_explored[node][0]
            if temp_cost < intersected_node[1]:
                intersected_node = (node, temp_cost)
    return intersected_node


# Union the explorer and frontier of goal or start explored to take care of possible crossover nodes
def merge_explored_frontier(explored, frontier, parent_dict):
    while frontier.size() > 0: #pop til empty
        node = frontier.pop()
        temp = node[1]
        if temp not in explored:
            explored[temp] = parent_dict[temp] #merging frontier and explored as long as frontier node not already in explored
    return explored



def bidirectional_a_star(graph, start, goal,
                         heuristic=euclidean_dist_heuristic):
    """
    Exercise 2: Bidirectional A*.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start (str): Key for the start node.
        goal (str): Key for the end node.
        heuristic: Function to determine distance heuristic.
            Default: euclidean_dist_heuristic.

    Returns:
        The best path as a list from the start and goal nodes (including both).
    """

    # TODO: finish this function!
    if start == goal:
        return []
    explored_start, explored_goal = {}, {}
    parent_dict_start, parent_dict_goal = {}, {} #store path cost and parent. Act as a hashtable
    optimal_path = []
    parent_node = None

    intersected_node = (None, float('inf'))

    frontier_start = PriorityQueue()
    frontier_goal = PriorityQueue()
    h_initial_start = euclidean_dist_heuristic(graph, start, goal)
    h_initial_goal = euclidean_dist_heuristic(graph, goal, start)
    frontier_start.append((0 + h_initial_start, start))
    frontier_goal.append((0 + h_initial_goal, goal))
    parent_dict_start[start] = (0, None) #only appends g because this is only asking for the path cost without heuristic
    parent_dict_goal[goal] = (0, None) #only appends g

    while frontier_start.size() > 0 or frontier_goal.size() > 0:
        if start == goal:
            return []
        # from start
        frontier_cost, current_node = frontier_start.pop()
        if current_node in explored_goal:
            explored_goal = merge_explored_frontier(explored_goal, frontier_goal, parent_dict_goal)
            explored_start[current_node] = parent_dict_start.pop(current_node)
            intersected_node = (current_node, frontier_cost + explored_goal[current_node][0])
            intersected_node = find_best_intersection(intersected_node, explored_goal, explored_start)
            break
        elif current_node not in explored_start:
            explored_start[current_node] = parent_dict_start.pop(current_node) #explored start only holds g value because it will be used to record path costs and return optimal path
            parent_cost = explored_start[current_node][0]
            for child in graph.neighbors(current_node):
                g = parent_cost + graph.get_edge_weight(current_node,child)
                h = euclidean_dist_heuristic(graph, child, goal)
                if child in parent_dict_start and g < parent_dict_start[child][0]:
                    del parent_dict_start[child]
                if child not in explored_start and child not in parent_dict_start:
                    parent_dict_start[child] = (g, current_node)
                    frontier_start.append((g+h, child))

        frontier_cost, current_node = frontier_goal.pop()
        if current_node in explored_start:
            explored_start = merge_explored_frontier(explored_start, frontier_start, parent_dict_start)
            explored_goal[current_node] = parent_dict_goal.pop(current_node)
            intersected_node = (current_node, frontier_cost + explored_start[current_node][0])
            intersected_node = find_best_intersection(intersected_node, explored_start, explored_goal)
            break
        elif current_node not in explored_goal:
            explored_goal[current_node] = parent_dict_goal.pop(current_node)
            parent_cost = explored_goal[current_node][0]
            for child in graph.neighbors(current_node):
                g = parent_cost + graph.get_edge_weight(current_node, child)
                h = euclidean_dist_heuristic(graph, child, start)
                if child in parent_dict_goal and g < parent_dict_goal[child][0]:
                    del parent_dict_goal[child]
                if child not in explored_goal and child not in parent_dict_goal:
                    parent_dict_goal[child] = (g, current_node)
                    frontier_goal.append((g+h, child))

    #return the optimal path
    temp_node = intersected_node[0]
    parent_node = explored_start[temp_node][1]
    while parent_node:
        optimal_path.append(temp_node)
        temp_node = parent_node
        parent_node = explored_start[temp_node][1]
    optimal_path.append(temp_node)
    optimal_path = optimal_path[::-1] #invert it
    parent_node = explored_goal[intersected_node[0]][1] #now just append the rest
    while parent_node:
        optimal_path.append(parent_node)
        parent_node = explored_goal[parent_node][1]
    return optimal_path

def get_path(intersected_node, explored_start, explored_goal):
    optimal_path = []
    temp_node = intersected_node[0]
    parent_node = explored_start[temp_node][1]
    while parent_node:
        optimal_path.append(temp_node)
        temp_node = parent_node
        parent_node = explored_start[temp_node][1]
    optimal_path.append(temp_node)
    optimal_path = optimal_path[::-1] #invert it
    parent_node = explored_goal[intersected_node[0]][1] #now just append the rest
    while parent_node:
        optimal_path.append(parent_node)
        parent_node = explored_goal[parent_node][1]
    return optimal_path

def combine_explore_frontier(explored,frontier,parent_dict):
    #need to return a temporary explorer
    temp_explorer = {k: v for k, v in explored.items()} #initalize with the original dict
    for node in frontier.__iter__():
        if node[2] not in temp_explorer:
            temp_explorer[node[2]] = parent_dict[node[2]] #add in frontier into the temp dict
    return temp_explorer

#this code is far from optimized and require different stoping condition because there are a lot of frontiers a,b,c where it tops values are the same, it should have been empty if terminating conditions were met.
def tridirectional_search(graph, goals):
    """
    Exercise 3: Tridirectional UCS Search

    See README.MD for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        goals (list): Key values for the 3 goals

    Returns:
        The best path as a list from one of the goal nodes (including both of
        the other goal nodes).
    """
    # TODO: finish this function
    if goals[0] == goals[1] and goals[1] == goals[2]:
        return []
    sorted_goals = [goals[0], goals[1], goals[2]]
    sorted_goals.sort()
    frontier_a, frontier_b, frontier_c = PriorityQueue(), PriorityQueue(), PriorityQueue()
    explored_a, explored_b, explored_c = {}, {}, {}
    parent_dict_a, parent_dict_b, parent_dict_c = {}, {}, {}
    paths = [[0, []], [0, []], [0, []]]
    optimal_path = []
    frontier_a.append((0, sorted_goals[0]))
    frontier_b.append((0, sorted_goals[1]))
    frontier_c.append((0, sorted_goals[2]))
    parent_dict_a[sorted_goals[0]] = (0, None)
    parent_dict_b[sorted_goals[1]] = (0, None)
    parent_dict_c[sorted_goals[2]] = (0, None)
    found_ab, found_ac, found_bc = False, False, False
    while not (found_ab and found_ac and found_bc):
        temp_list = [None, None, None]
        #pop the lowest value out of three frontiers
        if frontier_a.size() > 0:
            temp_list[0] = frontier_a.top()[0]
        else:
            temp_list[0] = float('inf')
        if frontier_b.size() > 0:
            temp_list[1] = frontier_b.top()[0]
        else:
            temp_list[1] = float('inf')
        if frontier_c.size() > 0:
            temp_list[2] = frontier_c.top()[0]
        else:
            temp_list[2] = float('inf')
        minimum_cost = temp_list[0]
        frontier_choice = 0
        for i in range(1, 3):
            if temp_list[i] < minimum_cost:
                frontier_choice = i
        #find the least frontier and pop that one for now.
        if frontier_choice == 0:
            if frontier_a.size() > 0:
                cost, current_node = frontier_a.pop()
        elif frontier_choice == 1:
            if frontier_b.size() > 0:
                cost, current_node = frontier_b.pop()
        elif frontier_choice == 2:
            if frontier_c.size() > 0:
                cost, current_node = frontier_c.pop()
        #print(found_ab, found_ac, found_bc)
        if frontier_choice == 0:
            if not (found_ab and found_ac):
                #print("a", current_node)
                if current_node in explored_b and not found_ab:
                    best_intersected = (current_node, cost + explored_b[current_node][0])
                    temp_explored_b = combine_explore_frontier(explored_b, frontier_b, parent_dict_b) #merge frontier and explored into temp for crossover comparison
                    if current_node not in explored_a:
                        explored_a[current_node] = parent_dict_a.pop(current_node)
                        parent_cost = explored_a[current_node][0]
                        for child in graph.neighbors(current_node): #we add in the children to our frontier. This is unescessary but I dont have time
                            pathcost = parent_cost + graph.get_edge_weight(current_node, child)
                            if child in parent_dict_a and pathcost < parent_dict_a[child][0]:
                                del parent_dict_a[child]
                            if child not in explored_a and child not in parent_dict_a:
                                parent_dict_a[child] = (pathcost, current_node)
                                frontier_a.append((pathcost, child))
                    best_intersected = find_best_intersection(best_intersected, temp_explored_b, explored_a) #determine the best intersected after merged
                    paths[0] = [best_intersected[1], get_path(best_intersected, explored_a, temp_explored_b)] #update path 0
                    found_ab = True
                elif current_node in explored_c and not found_ac:
                    best_intersected = (current_node, cost + explored_c[current_node][0])
                    temp_explored_c = combine_explore_frontier(explored_c, frontier_c,parent_dict_c)
                    if current_node not in explored_a:
                        explored_a[current_node] = parent_dict_a.pop(current_node)
                        parent_cost = explored_a[current_node][0]
                        for child in graph.neighbors(current_node):
                            pathcost = parent_cost + graph.get_edge_weight(current_node, child)
                            if child in parent_dict_a and pathcost < parent_dict_a[child][0]:
                                del parent_dict_a[child]
                            if child not in explored_a and child not in parent_dict_a:
                                parent_dict_a[child] = (pathcost, current_node)
                                frontier_a.append((pathcost, child))
                    best_intersected = find_best_intersection(best_intersected, temp_explored_c, explored_a)
                    paths[1] = [best_intersected[1], get_path(best_intersected, explored_a, temp_explored_c)]
                    found_ac = True
                elif current_node not in explored_a:
                    explored_a[current_node] = parent_dict_a.pop(current_node)
                    parent_cost = explored_a[current_node][0]
                    for child in graph.neighbors(current_node):
                        pathcost = parent_cost + graph.get_edge_weight(current_node,child)
                        if child in parent_dict_a and pathcost < parent_dict_a[child][0]:
                            del parent_dict_a[child]
                        if child not in explored_a and child not in parent_dict_a:
                            parent_dict_a[child] = (pathcost, current_node)
                            frontier_a.append((pathcost, child))
        if frontier_choice == 1:
            if not (found_ab and found_bc):
                #print("b", current_node)
                if current_node in explored_a and not found_ab:
                    best_intersected = (current_node, cost + explored_a[current_node][0])
                    temp_explored_a = combine_explore_frontier(explored_a, frontier_a, parent_dict_a)
                    if current_node not in explored_b:
                        explored_b[current_node] = parent_dict_b.pop(current_node)
                        parent_cost = explored_b[current_node][0]
                        for child in graph.neighbors(current_node):
                            pathcost = parent_cost + graph.get_edge_weight(current_node,child)
                            if child in parent_dict_b and pathcost < parent_dict_b[child][0]:
                                del parent_dict_b[child]
                            if child not in explored_b and child not in parent_dict_b:
                                parent_dict_b[child] = (pathcost, current_node)
                                frontier_b.append((pathcost, child))
                    best_intersected = find_best_intersection(best_intersected, temp_explored_a, explored_b)
                    paths[0] = [best_intersected[1], get_path(best_intersected, temp_explored_a, explored_b)]
                    found_ab = True
                elif current_node in explored_c and not found_bc:
                    best_intersected = (current_node, cost + explored_c[current_node][0])
                    temp_explored_c = combine_explore_frontier(explored_c, frontier_c, parent_dict_c)
                    if current_node not in explored_b:
                        explored_b[current_node] = parent_dict_b.pop(current_node)
                        parent_cost = explored_b[current_node][0]
                        for child in graph.neighbors(current_node):
                            pathcost = parent_cost + graph.get_edge_weight(current_node,child)
                            if child in parent_dict_b and pathcost < parent_dict_b[child][0]:
                                del parent_dict_b[child]
                            if child not in explored_b and child not in parent_dict_b:
                                parent_dict_b[child] = (pathcost, current_node)
                                frontier_b.append((pathcost, child))
                    best_intersected = find_best_intersection(best_intersected, temp_explored_c, explored_b)
                    paths[2] = [best_intersected[1], get_path(best_intersected, explored_b, temp_explored_c)]
                    found_bc = True
                elif current_node not in explored_b:
                    explored_b[current_node] = parent_dict_b.pop(current_node)
                    parent_cost = explored_b[current_node][0]
                    for child in graph.neighbors(current_node):
                        pathcost = parent_cost + graph.get_edge_weight(current_node,child)
                        if child in parent_dict_b and pathcost < parent_dict_b[child][0]:
                            del parent_dict_b[child]
                        if child not in explored_b and child not in parent_dict_b:
                            parent_dict_b[child] = (pathcost, current_node)
                            frontier_b.append((pathcost, child))
        if frontier_choice == 2:
            if not (found_ac and found_bc):
                #print("c", current_node)
                if current_node in explored_a and not found_ac:
                    best_intersected = (current_node, cost + explored_a[current_node][0])
                    temp_explored_a = combine_explore_frontier(explored_a, frontier_a, parent_dict_a)
                    if current_node not in explored_c:
                        explored_c[current_node] = parent_dict_c.pop(current_node)
                        parent_cost = explored_c[current_node][0]
                        for child in graph.neighbors(current_node):
                            pathcost = parent_cost + graph.get_edge_weight(current_node, child)
                            if child in parent_dict_c and pathcost < parent_dict_c[child][0]:
                                del parent_dict_c[child]
                            if child not in explored_c and child not in parent_dict_c:
                                parent_dict_c[child] = (pathcost, current_node)
                                frontier_c.append((pathcost, child))
                    best_intersected = find_best_intersection(best_intersected, temp_explored_a, explored_c)
                    paths[1] = [best_intersected[1], get_path(best_intersected, temp_explored_a, explored_c)]
                    found_ac = True
                elif current_node in explored_b and not found_bc:
                    best_intersected = (current_node, cost + explored_b[current_node][0])
                    temp_explored_b = combine_explore_frontier(explored_b,frontier_b,parent_dict_b)
                    if current_node not in explored_c:
                        explored_c[current_node] = parent_dict_c.pop(current_node)
                        parent_cost = explored_c[current_node][0]
                        for child in graph.neighbors(current_node):
                            pathcost = parent_cost + graph.get_edge_weight(current_node, child)
                            if child in parent_dict_c and pathcost < parent_dict_c[child][0]:
                                del parent_dict_c[child]
                            if child not in explored_c and child not in parent_dict_c:
                                parent_dict_c[child] = (pathcost, current_node)
                                frontier_c.append((pathcost, child))
                    best_intersected = find_best_intersection(best_intersected, temp_explored_b, explored_c)
                    paths[2] = [best_intersected[1], get_path(best_intersected, temp_explored_b, explored_c)]
                    found_bc = True
                elif current_node not in explored_c:
                    explored_c[current_node] = parent_dict_c.pop(current_node)
                    parent_cost = explored_c[current_node][0]
                    for child in graph.neighbors(current_node):
                        pathcost = parent_cost + graph.get_edge_weight(current_node, child)
                        if child in parent_dict_c and pathcost < parent_dict_c[child][0]:
                            del parent_dict_c[child]
                        if child not in explored_c and child not in parent_dict_c:
                            parent_dict_c[child] = (pathcost, current_node)
                            frontier_c.append((pathcost, child))
    #Now find the two paths based on least cost to greatest
    '''print("this is path a to b", paths[0][1])
    print("cost is", paths[0][0])
    print("this is path a to c", paths[1][1])
    print("cost is", paths[1][0])
    print("this is path b to c", paths[2][1])
    print("cost is", paths[2][0])
    print(explored_a)
    print(explored_b)
    print(explored_c)'''
    best_min = paths[0] #random, anything will do within the 3 paths, this is to initialize
    stored_index = 0
    for i in range(0, 3):
        if paths[i][0] < best_min[0]:
            best_min = paths[i]
            stored_index = i
    # now find second best
    best_second = paths[(stored_index + 2) % 3] #intialize, this time must be this specific index for the if state to work
    if paths[(stored_index + 1) % 3][0] < best_second[0]: #this only works because we always have three paths in tri-directional
        best_second = paths[(stored_index + 1) % 3]
    #Now return the path based on order
    if best_min[1][-1] == best_second[1][0]:
        del best_min[1][-1]
        optimal_path = best_min[1] + best_second[1]
    elif best_min[1][-1] == best_second[1][-1]:
        del best_min[1][-1]
        best_second[1] = best_second[1][::-1]
        optimal_path = best_min[1] + best_second[1]
    elif best_min[1][0] == best_second[1][0]:
        del best_min[1][0]
        best_second[1] = best_second[1][::-1]
        optimal_path = best_second[1] + best_min[1]
    elif best_min[1][0] == best_second[1][-1]:
        del best_min[1][0]
        optimal_path = best_second[1] + best_min[1]
    #print(optimal_path)
    return optimal_path

#this code does not work because I did not combine the h heuristic
def tridirectional_upgraded(graph, goals, heuristic=euclidean_dist_heuristic, landmarks=None):
    """
    Exercise 4: Upgraded Tridirectional Search

    See README.MD for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        goals (list): Key values for the 3 goals
        heuristic: Function to determine distance heuristic.
            Default: euclidean_dist_heuristic.
        landmarks: Iterable containing landmarks pre-computed in compute_landmarks()
            Default: None

    Returns:
        The best path as a list from one of the goal nodes (including both of
        the other goal nodes).
    """
    # TODO: finish this function
    if goals[0] == goals[1] and goals[1] == goals[2]:
        return []
    sorted_goals = [goals[0], goals[1], goals[2]]
    sorted_goals.sort()
    frontier_a, frontier_b, frontier_c = PriorityQueue(), PriorityQueue(), PriorityQueue()
    explored_a, explored_b, explored_c = {}, {}, {}
    parent_dict_a, parent_dict_b, parent_dict_c = {}, {}, {}
    paths = [[0, []], [0, []], [0, []]]
    optimal_path = []
    frontier_a.append((0, sorted_goals[0]))
    frontier_b.append((0, sorted_goals[1]))
    frontier_c.append((0, sorted_goals[2]))
    parent_dict_a[sorted_goals[0]] = (0, None)
    parent_dict_b[sorted_goals[1]] = (0, None)
    parent_dict_c[sorted_goals[2]] = (0, None)
    found_ab, found_ac, found_bc = False, False, False
    while not (found_ab and found_ac and found_bc):
        #print(found_ab,found_ac,found_bc)
        if not (found_ab and found_ac):
            if frontier_a.size() > 0:
                cost, current_node = frontier_a.pop()
                #print("a", current_node)
                if current_node in explored_b and not found_ab:
                    best_intersected = (current_node, cost + explored_b[current_node][0])
                    temp_explored_b = combine_explore_frontier(explored_b, frontier_b, parent_dict_b)
                    if current_node not in explored_a:
                        explored_a[current_node] = parent_dict_a.pop(current_node)
                        parent_cost = explored_a[current_node][0]
                        for child in graph.neighbors(current_node):
                            g = parent_cost + graph.get_edge_weight(current_node, child)
                            h1 = euclidean_dist_heuristic(graph, child, sorted_goals[1])
                            h2 = euclidean_dist_heuristic(graph, child, sorted_goals[2])
                            if child in parent_dict_a and g < parent_dict_a[child][0]:
                                del parent_dict_a[child]
                            if child not in explored_a and child not in parent_dict_a:
                                parent_dict_a[child] = (g, current_node)
                                frontier_a.append((g + h1, child))
                                frontier_a.append((g + h2, child))
                    best_intersected = find_best_intersection(best_intersected, temp_explored_b, explored_a)
                    paths[0] = [best_intersected[1], get_path(best_intersected, explored_a, temp_explored_b)]
                    found_ab = True
                elif current_node in explored_c and not found_ac:
                    best_intersected = (current_node, cost + explored_c[current_node][0])
                    temp_explored_c = combine_explore_frontier(explored_c, frontier_c,parent_dict_c)
                    if current_node not in explored_a:
                        explored_a[current_node] = parent_dict_a.pop(current_node)
                        parent_cost = explored_a[current_node][0]
                        for child in graph.neighbors(current_node):
                            g = parent_cost + graph.get_edge_weight(current_node, child)
                            h1 = euclidean_dist_heuristic(graph, child, sorted_goals[1])
                            h2 = euclidean_dist_heuristic(graph, child, sorted_goals[2])
                            if child in parent_dict_a and g < parent_dict_a[child][0]:
                                del parent_dict_a[child]
                            if child not in explored_a and child not in parent_dict_a:
                                parent_dict_a[child] = (g, current_node)
                                frontier_a.append((g + h1, child))
                                frontier_a.append((g + h2, child))
                    best_intersected = find_best_intersection(best_intersected, temp_explored_c, explored_a)
                    paths[1] = [best_intersected[1], get_path(best_intersected, explored_a, temp_explored_c)]
                    found_ac = True
                elif current_node not in explored_a:
                    explored_a[current_node] = parent_dict_a.pop(current_node)
                    parent_cost = explored_a[current_node][0]
                    for child in graph.neighbors(current_node):
                        g = parent_cost + graph.get_edge_weight(current_node,child)
                        h1 = euclidean_dist_heuristic(graph, child, sorted_goals[1])
                        h2 = euclidean_dist_heuristic(graph, child, sorted_goals[2])
                        if child in parent_dict_a and g < parent_dict_a[child][0]:
                            del parent_dict_a[child]
                        if child not in explored_a and child not in parent_dict_a:
                            parent_dict_a[child] = (g, current_node)
                            frontier_a.append((g + h1, child))
                            frontier_a.append((g + h2, child))
        if not (found_ab and found_bc):
            if frontier_b.size() > 0:
                cost, current_node = frontier_b.pop()
                #print("b", current_node)
                if current_node in explored_a and not found_ab:
                    best_intersected = (current_node, cost + explored_a[current_node][0])
                    temp_explored_a = combine_explore_frontier(explored_a, frontier_a, parent_dict_a)
                    if current_node not in explored_b:
                        explored_b[current_node] = parent_dict_b.pop(current_node)
                        parent_cost = explored_b[current_node][0]
                        for child in graph.neighbors(current_node):
                            g = parent_cost + graph.get_edge_weight(current_node,child)
                            h1 = euclidean_dist_heuristic(graph, child, sorted_goals[0])
                            h2 = euclidean_dist_heuristic(graph, child, sorted_goals[2])
                            if child in parent_dict_b and g < parent_dict_b[child][0]:
                                del parent_dict_b[child]
                            if child not in explored_b and child not in parent_dict_b:
                                parent_dict_b[child] = (g, current_node)
                                frontier_b.append((g + h1, child))
                                frontier_b.append((g + h2, child))
                    best_intersected = find_best_intersection(best_intersected, temp_explored_a, explored_b)
                    paths[0] = [best_intersected[1], get_path(best_intersected, temp_explored_a, explored_b)]
                    found_ab = True
                elif current_node in explored_c and not found_bc:
                    best_intersected = (current_node, cost + explored_c[current_node][0])
                    temp_explored_c = combine_explore_frontier(explored_c, frontier_c, parent_dict_c)
                    if current_node not in explored_b:
                        explored_b[current_node] = parent_dict_b.pop(current_node)
                        parent_cost = explored_b[current_node][0]
                        for child in graph.neighbors(current_node):
                            g = parent_cost + graph.get_edge_weight(current_node,child)
                            h1 = euclidean_dist_heuristic(graph, child, sorted_goals[0])
                            h2 = euclidean_dist_heuristic(graph, child, sorted_goals[2])
                            if child in parent_dict_b and g < parent_dict_b[child][0]:
                                del parent_dict_b[child]
                            if child not in explored_b and child not in parent_dict_b:
                                parent_dict_b[child] = (g, current_node)
                                frontier_b.append((g + h1, child))
                                frontier_b.append((g + h2, child))
                    best_intersected = find_best_intersection(best_intersected, temp_explored_c, explored_b)
                    paths[2] = [best_intersected[1], get_path(best_intersected, explored_b, temp_explored_c)]
                    found_bc = True
                elif current_node not in explored_b:
                    explored_b[current_node] = parent_dict_b.pop(current_node)
                    parent_cost = explored_b[current_node][0]
                    for child in graph.neighbors(current_node):
                        g = parent_cost + graph.get_edge_weight(current_node,child)
                        h1 = euclidean_dist_heuristic(graph, child, sorted_goals[0])
                        h2 = euclidean_dist_heuristic(graph, child, sorted_goals[2])
                        if child in parent_dict_b and g < parent_dict_b[child][0]:
                            del parent_dict_b[child]
                        if child not in explored_b and child not in parent_dict_b:
                            parent_dict_b[child] = (g, current_node)
                            frontier_b.append((g + h1, child))
                            frontier_b.append((g + h2, child))
        if not (found_ac and found_bc):
            if frontier_c.size() > 0:
                cost, current_node = frontier_c.pop()
                #print("c", current_node)
                if current_node in explored_a and not found_ac:
                    best_intersected = (current_node, cost + explored_a[current_node][0])
                    temp_explored_a = combine_explore_frontier(explored_a, frontier_a, parent_dict_a)
                    if current_node not in explored_c:
                        explored_c[current_node] = parent_dict_c.pop(current_node)
                        parent_cost = explored_c[current_node][0]
                        for child in graph.neighbors(current_node):
                            g = parent_cost + graph.get_edge_weight(current_node, child)
                            h1 = euclidean_dist_heuristic(graph, child, sorted_goals[0])
                            h2 = euclidean_dist_heuristic(graph, child, sorted_goals[1])
                            if child in parent_dict_c and g < parent_dict_c[child][0]:
                                del parent_dict_c[child]
                            if child not in explored_c and child not in parent_dict_c:
                                parent_dict_c[child] = (g, current_node)
                                frontier_c.append((g + h1, child))
                                frontier_c.append((g + h2, child))
                    best_intersected = find_best_intersection(best_intersected, temp_explored_a, explored_c)
                    paths[1] = [best_intersected[1], get_path(best_intersected, temp_explored_a, explored_c)]
                    found_ac = True
                elif current_node in explored_b and not found_bc:
                    best_intersected = (current_node, cost + explored_b[current_node][0])
                    temp_explored_b = combine_explore_frontier(explored_b,frontier_b,parent_dict_b)
                    if current_node not in explored_c:
                        explored_c[current_node] = parent_dict_c.pop(current_node)
                        parent_cost = explored_c[current_node][0]
                        for child in graph.neighbors(current_node):
                            g = parent_cost + graph.get_edge_weight(current_node, child)
                            h1 = euclidean_dist_heuristic(graph, child, sorted_goals[0])
                            h2 = euclidean_dist_heuristic(graph, child, sorted_goals[1])
                            if child in parent_dict_c and g < parent_dict_c[child][0]:
                                del parent_dict_c[child]
                            if child not in explored_c and child not in parent_dict_c:
                                parent_dict_c[child] = (g, current_node)
                                frontier_c.append((g + h1, child))
                                frontier_c.append((g + h2, child))
                    best_intersected = find_best_intersection(best_intersected, temp_explored_b, explored_c)
                    paths[2] = [best_intersected[1], get_path(best_intersected, temp_explored_b, explored_c)]
                    found_bc = True
                elif current_node not in explored_c:
                    explored_c[current_node] = parent_dict_c.pop(current_node)
                    parent_cost = explored_c[current_node][0]
                    for child in graph.neighbors(current_node):
                        g = parent_cost + graph.get_edge_weight(current_node, child)
                        h1 = euclidean_dist_heuristic(graph, child, sorted_goals[0])
                        h2 = euclidean_dist_heuristic(graph, child, sorted_goals[1])
                        if child in parent_dict_c and g < parent_dict_c[child][0]:
                            del parent_dict_c[child]
                        if child not in explored_c and child not in parent_dict_c:
                            parent_dict_c[child] = (g, current_node)
                            frontier_c.append((g + h1, child))
                            frontier_c.append((g + h2, child))
    #Now find the two paths based on least cost to greatest
    '''print("this is path a to b", paths[0][1])
    print("cost is", paths[0][0])
    print("this is path a to c", paths[1][1])
    print("cost is", paths[1][0])
    print("this is path b to c", paths[2][1])
    print("cost is", paths[2][0])
    print(explored_a)
    print(explored_b)
    print(explored_c)'''
    best_min = paths[0] #random, anything will do within the 3 paths, this is to initialize
    stored_index = 0
    for i in range(0, 3):
        if paths[i][0] < best_min[0]:
            best_min = paths[i]
            stored_index = i
    # now find second best
    best_second = paths[(stored_index + 2) % 3] #intialize, this time must be this specific index for the if state to work
    if paths[(stored_index + 1) % 3][0] < best_second[0]: #this only works because we always have three paths in tri-directional
        best_second = paths[(stored_index + 1) % 3]

    #Now return the path based on order
    if best_min[1][-1] == best_second[1][0]:
        del best_min[1][-1]
        optimal_path = best_min[1] + best_second[1]
    elif best_min[1][-1] == best_second[1][-1]:
        del best_min[1][-1]
        best_second[1] = best_second[1][::-1]
        optimal_path = best_min[1] + best_second[1]
    elif best_min[1][0] == best_second[1][0]:
        del best_min[1][0]
        best_second[1] = best_second[1][::-1]
        optimal_path = best_second[1] + best_min[1]
    elif best_min[1][0] == best_second[1][-1]:
        del best_min[1][0]
        optimal_path = best_second[1] + best_min[1]
    #print(optimal_path)
    return optimal_path


def return_your_name():
    """Return your name from this function"""
    # TODO: finish this function
    return ("Kha Manh Dan")

def compute_landmarks(graph):
    """
    Feel free to implement this method for computing landmarks. We will call
    tridirectional_upgraded() with the object returned from this function.

    Args:
        graph (ExplorableGraph): Undirected graph to search.

    Returns:
    List with not more than 4 computed landmarks.
    """
    return None


def custom_heuristic(graph, v, goal):
    """
       Feel free to use this method to try and work with different heuristics and come up with a better search algorithm.
       Args:
           graph (ExplorableGraph): Undirected graph to search.
           v (str): Key for the node to calculate from.
           goal (str): Key for the end node to calculate to.
       Returns:
           Custom heuristic distance between `v` node and `goal` node
       """
    pass


# Extra Credit: Your best search method for the race
def custom_search(graph, start, goal, data=None):
    """
    Race!: Implement your best search algorithm here to compete against the
    other student agents.

    If you implement this function and submit your code to Gradescope, you'll be
    registered for the Race!

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start (str): Key for the start node.
        goal (str): Key for the end node.
        data :  Data used in the custom search.
            Will be passed your data from load_data(graph).
            Default: None.

    Returns:
        The best path as a list from the start and goal nodes (including both).
    """

    # TODO: finish this function!
    raise NotImplementedError


def load_data(graph, time_left):
    """
    Feel free to implement this method. We'll call it only once
    at the beginning of the Race, and we'll pass the output to your custom_search function.
    graph: a networkx graph
    time_left: function you can call to keep track of your remaining time.
        usage: time_left() returns the time left in milliseconds.
        the max time will be 10 minutes.

    * To get a list of nodes, use graph.nodes()
    * To get node neighbors, use graph.neighbors(node)
    * To get edge weight, use graph.get_edge_weight(node1, node2)
    """

    # nodes = graph.nodes()
    return None


def haversine_dist_heuristic(graph, v, goal):
    """
    Note: This provided heuristic is for the Atlanta race.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        v (str): Key for the node to calculate from.
        goal (str): Key for the end node to calculate to.

    Returns:
        Haversine distance between `v` node and `goal` node
    """

    # Load latitude and longitude coordinates in radians:
    vLatLong = (math.radians(graph.nodes[v]["pos"][0]), math.radians(graph.nodes[v]["pos"][1]))
    goalLatLong = (math.radians(graph.nodes[goal]["pos"][0]), math.radians(graph.nodes[goal]["pos"][1]))

    # Now we want to execute portions of the formula:
    constOutFront = 2 * 6371  # Radius of Earth is 6,371 kilometers
    term1InSqrt = (math.sin((goalLatLong[0] - vLatLong[0]) / 2)) ** 2  # First term inside sqrt
    term2InSqrt = math.cos(vLatLong[0]) * math.cos(goalLatLong[0]) * (
                (math.sin((goalLatLong[1] - vLatLong[1]) / 2)) ** 2)  # Second term
    return constOutFront * math.asin(math.sqrt(term1InSqrt + term2InSqrt))  # Straight application of formula