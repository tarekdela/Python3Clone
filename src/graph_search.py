import numpy as np
from .graph import Cell
from .utils import trace_path
from collections import deque

"""
General graph search instructions:

First, define the correct data type to keep track of your visited cells
and add the start cell to it. If you need to initialize any properties
of the start cell, do that too.

Next, implement the graph search function. When you find a path, use the
trace_path() function to return a path given the goal cell and the graph. You
must have kept track of the parent of each node correctly and have implemented
the graph.get_parent() function for this to work. If you do not find a path,
return an empty list.

To visualize which cells are visited in the navigation webapp, save each
visited cell in the list in the graph class as follows:
     graph.visited_cells.append(Cell(cell_i, cell_j))
where cell_i and cell_j are the cell indices of the visited cell you want to
visualize.
"""


def depth_first_search(graph, start_cell, goal_cell):
    """Depth First Search (DFS) algorithm. This algorithm is optional for P3.
    Args:
        graph: The graph class.
        start_cell: Start cell as a Cell object.
        goal_cell: Goal cell as a Cell object.
    
    Returns:
        A list of Cell objects representing the path from start to goal,
        or an empty list if no path is found.
    """
    graph.init_graph()  # Make sure all the node values are reset.
    
    # Validate start and goal cells
    if not graph.is_cell_in_bounds(start_cell.i, start_cell.j):
        return []
    
    if not graph.is_cell_in_bounds(goal_cell.i, goal_cell.j):
        return []
    
    if graph.check_collision(start_cell.i, start_cell.j):
        return []
    
    if graph.check_collision(goal_cell.i, goal_cell.j):
        return []
    
    # Initialize DFS stack (LIFO using regular list with pop())
    stack = []
    start_node = graph.nodes[start_cell.j][start_cell.i]
    start_node.visited = True
    stack.append(start_node)
    
    # DFS main loop
    while stack:
        # Pop from end of stack (LIFO)
        current_node = stack.pop()
        graph.visited_cells.append(Cell(current_node.i, current_node.j))
        
        # Check if goal is reached
        if current_node.i == goal_cell.i and current_node.j == goal_cell.j:
            # Reconstruct path
            path = []
            node = current_node
            while node is not None:
                path.append(node)
                node = node.parent
            path.reverse()
            # Convert nodes back to Cell objects for return
            return [Cell(n.i, n.j) for n in path]
        
        # Explore neighbors
        neighbors = graph.find_neighbors(current_node.i, current_node.j)
        
        for neighbor_cell in neighbors:
            neighbor_node = graph.nodes[neighbor_cell.j][neighbor_cell.i]
            
            # Skip if already visited
            if neighbor_node.visited:
                continue
            
            # Skip if in collision
            if graph.check_collision(neighbor_cell.i, neighbor_cell.j):
                continue
            
            # Mark as visited and set parent
            neighbor_node.visited = True
            neighbor_node.parent = current_node
            stack.append(neighbor_node)
    
    # If no path was found, return an empty list.
    return []




def breadth_first_search(graph, start_cell, goal_cell):
    """Performs Breadth First Search (BFS) on the given graph.
    
    Args:
        graph: A GridGraph object containing the map and node data.
        start_cell: A Cell object representing the start position.
        goal_cell: A Cell object representing the goal position.
    
    Returns:
        A list of Cell objects representing the path from start to goal,
        or an empty list if no path is found.
    """
    # Initialize the graph nodes
    graph.init_graph()
    
    # Validate start and goal cells
    if not graph.is_cell_in_bounds(start_cell.i, start_cell.j):
        return []
    
    if not graph.is_cell_in_bounds(goal_cell.i, goal_cell.j):
        return []
    
    if graph.check_collision(start_cell.i, start_cell.j):
        return []
    
    if graph.check_collision(goal_cell.i, goal_cell.j):
        return []
    
    # Initialize BFS queue with regular list (FIFO using pop(0))
    queue = []
    start_node = graph.nodes[start_cell.j][start_cell.i]
    start_node.visited = True
    queue.append(start_node)  # Store node object directly
    
    # BFS main loop
    while queue:
        # Pop from front of queue (FIFO)
        current_node = queue.pop(0)
        graph.visited_cells.append(Cell(current_node.i, current_node.j))
        
        # Check if goal is reached
        if current_node.i == goal_cell.i and current_node.j == goal_cell.j:
            # Reconstruct path
            path = []
            node = current_node
            while node is not None:
                path.append(node)
                node = node.parent
            path.reverse()
            # Convert nodes back to Cell objects for return
            return [Cell(n.i, n.j) for n in path]
        
        # Explore neighbors
        neighbors = graph.find_neighbors(current_node.i, current_node.j)
        
        for neighbor_cell in neighbors:
            neighbor_node = graph.nodes[neighbor_cell.j][neighbor_cell.i]
            
            # Skip if already visited
            if neighbor_node.visited:
                continue
            
            # Skip if in collision (don't add collision cells to visit list)
            if graph.check_collision(neighbor_cell.i, neighbor_cell.j):
                continue
            
            # Mark as visited and set parent
            neighbor_node.visited = True
            neighbor_node.parent = current_node
            queue.append(neighbor_node)  # Store node object directly
    
    # No path found
    return []


def a_star_search(graph, start_cell, goal_cell):
    """A* Search algorithm.
    Args:
        graph: The graph class.
        start_cell: Start cell as a Cell object.
        goal_cell: Goal cell as a Cell object.
    
    Returns:
        A list of Cell objects representing the path from start to goal,
        or an empty list if no path is found.
    """
    graph.init_graph()  # Make sure all the node values are reset.
    
    # Validate start and goal cells
    if not graph.is_cell_in_bounds(start_cell.i, start_cell.j):
        return []
    
    if not graph.is_cell_in_bounds(goal_cell.i, goal_cell.j):
        return []
    
    if graph.check_collision(start_cell.i, start_cell.j):
        return []
    
    if graph.check_collision(goal_cell.i, goal_cell.j):
        return []
    
    # Heuristic function (Euclidean distance)
    def heuristic(i, j):
        return ((i - goal_cell.i)**2 + (j - goal_cell.j)**2)**0.5
    
    # Initialize A* with priority queue (open list)
    # Store as list of (f_cost, node) tuples
    open_list = []
    start_node = graph.nodes[start_cell.j][start_cell.i]
    start_node.g_cost = 0
    start_node.h_cost = heuristic(start_node.i, start_node.j)
    start_node.f_cost = start_node.g_cost + start_node.h_cost
    start_node.visited = False  # In A*, visited means in closed list
    open_list.append((start_node.f_cost, start_node))
    
    # A* main loop
    while open_list:
        # Find node with lowest f_cost (manual priority queue)
        min_idx = 0
        for i in range(1, len(open_list)):
            if open_list[i][0] < open_list[min_idx][0]:
                min_idx = i
        
        # Remove node with lowest f_cost
        _, current_node = open_list.pop(min_idx)
        current_node.visited = True  # Mark as in closed list
        graph.visited_cells.append(Cell(current_node.i, current_node.j))
        
        # Check if goal is reached
        if current_node.i == goal_cell.i and current_node.j == goal_cell.j:
            # Reconstruct path
            path = []
            node = current_node
            while node is not None:
                path.append(node)
                node = node.parent
            path.reverse()
            # Convert nodes back to Cell objects for return
            return [Cell(n.i, n.j) for n in path]
        
        # Explore neighbors
        neighbors = graph.find_neighbors(current_node.i, current_node.j)
        
        for neighbor_cell in neighbors:
            neighbor_node = graph.nodes[neighbor_cell.j][neighbor_cell.i]
            
            # Skip if in closed list (already visited)
            if neighbor_node.visited:
                continue
            
            # Skip if in collision
            if graph.check_collision(neighbor_cell.i, neighbor_cell.j):
                continue
            
            # Calculate tentative g_cost
            # Cost is 1.0 for cardinal moves, ~1.414 for diagonal moves
            dx = abs(neighbor_cell.i - current_node.i)
            dy = abs(neighbor_cell.j - current_node.j)
            move_cost = (dx**2 + dy**2)**0.5
            tentative_g = current_node.g_cost + move_cost
            
            # Check if this path to neighbor is better
            if tentative_g < neighbor_node.g_cost:
                # Update neighbor
                neighbor_node.parent = current_node
                neighbor_node.g_cost = tentative_g
                neighbor_node.h_cost = heuristic(neighbor_node.i, neighbor_node.j)
                neighbor_node.f_cost = neighbor_node.g_cost + neighbor_node.h_cost
                
                # Add to open list if not already there
                in_open_list = any(node is neighbor_node for _, node in open_list)
                if not in_open_list:
                    open_list.append((neighbor_node.f_cost, neighbor_node))

    # If no path was found, return an empty list.
    return []



