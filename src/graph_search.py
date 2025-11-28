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


def depth_first_search(graph, start, goal):
    """Depth First Search (DFS) algorithm. This algorithm is optional for P3.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    """TODO (P3): Implement DFS (optional)."""

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


def a_star_search(graph, start, goal):
    """A* Search (BFS) algorithm.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    """TODO (P3): Implement A*."""

    # If no path was found, return an empty list.
    return []
