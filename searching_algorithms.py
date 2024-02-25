# contain depth_first_search, breadth_first_search, id_depth_first_search, best_first_search 
import heapq
from collections import deque

def depth_first_search(graph, start, goal, path=[], visited=set()):
    # Add the start node to the path
    path = path + [start]
    # Mark the start node as visited
    visited.add(start)

    # Check if the start node is the goal node
    if start == goal:
        return path

    # If the start node is not in the graph, return None
    if start not in graph:
        return None

    # Explore neighbors
    for neighbor in graph[start]:
        # If neighbor hasn't been visited
        if neighbor not in visited:
            # Recursively search from the neighbor
            new_path = depth_first_search(graph, neighbor, goal, path, visited)
            # If a path is found, return it
            if new_path:
                return new_path

    # If no path is found, return None
    return None


def breadth_first_search(graph, start, goal, path=[], visited=set()):
    # Queue for BFS
    queue = deque([(start, path + [start])])  # (node,path)
    # Visited set to keep track of visited nodes
    visited = set()

    while queue:
        # Get the current node and its path
        node, path = queue.popleft()
        # Mark the node as visited
        visited.add(node)

        # Check if the current node is the goal
        if node == goal:
            return path

        # Explore neighbors
        for neighbor in graph.get(node, []):
            if neighbor not in visited:
                # Add the neighbor and its path to the queue
                queue.append((neighbor, path + [neighbor]))

    # If no path is found, return None
    return None

def id_depth_first_search(graph, start, goal,path=[],visited=set()):
    # Start with depth limit 0
    depth_limit = 0

    # Iterate until goal is found or depth limit is exceeded
    while True:
        # Perform depth-limited search
        result = dfs_recursive(graph, start, goal, depth_limit)
        # If goal is found, return the path
        if result is not None:
            return result
        # If goal is not found at this depth, increase depth limit for next iteration
        depth_limit += 1

        # If depth limit exceeds the maximum depth of the graph, break the loop
        if depth_limit > len(graph):
            break

    # If goal is not found within the depth limit, return None
    return None

def dfs_recursive(graph, node, goal, depth_limit, path=[]):
    # Append the current node to the path
    path = path + [node]

    # If the current node is the goal, return the path
    if node == goal:
        return path

    # If the depth limit is reached, stop searching
    if depth_limit == 0:
        return None

    # Recursively explore neighbors if not at depth limit
    for neighbor in graph.get(node, []):
        # Avoid revisiting nodes already in the path to prevent cycles
        if neighbor not in path:
            # Perform DFS recursively with decreased depth limit
            result = dfs_recursive(graph, neighbor, goal, depth_limit - 1, path)
            # If goal is found, return the path
            if result is not None:
                return result

    # If goal is not found within the depth limit, return None
    return None


def best_first_search(graph, start, goal, heuristic):
    # Priority queue for BFS
    pq = [(heuristic(start, goal), [start])]  # Initialize with start node and its path
    # Visited set to keep track of visited nodes
    visited = set()
    
    while pq:
        # Get the node with the lowest estimated cost and its path
        _, path = heapq.heappop(pq)
        node = path[-1]  # Get the last node in the path
        
        # If the node is the goal, return the path
        if node == goal:
            return path
        
        # Mark the node as visited
        visited.add(node)
        
        # Explore neighbors
        for neighbor in graph.get(node, []):
            if neighbor not in visited:
                # Calculate the estimated cost to the goal for the neighbor
                cost = heuristic(neighbor, goal)
                # Add the neighbor and its path to the priority queue with its estimated cost
                heapq.heappush(pq, (cost, path + [neighbor]))
    
    # If no path is found, return None
    return None

def astar_search(graph, start, goal, heuristic):
    # Priority queue for A* search
    # queue of tuples (f_score, g_score, node, path)
    # f_score = g_score + heuristic cost from start to goal
    # g_score = cost from the start of the path to this node
    pq = [(0 + heuristic(start, goal), 0, start, [])]  # (f_score, g_score, node, path)
    # Visited set to keep track of visited nodes
    visited = set()
    
    while pq:
        _, g_score, node, path = heapq.heappop(pq)
        
        # If the node is the goal, return the path
        if node == goal:
            return path + [node]
        
        # Mark the node as visited
        visited.add(node)
        
        # Explore neighbors
        for neighbor in graph.get(node, []):
            if neighbor not in visited:
                # Calculate the tentative g_score for the neighbor (cost from the start of the path to this neighbor node)
                tentative_g_score = g_score + heuristic(node,neighbor)
                # Calculate the f_score for the neighbor
                f_score = tentative_g_score + heuristic(neighbor, goal)
                # Add the neighbor and its path to the priority queue with its f_score and g_score
                heapq.heappush(pq, (f_score, tentative_g_score, neighbor, path + [node]))
    
    # If no path is found, return None
    return None