"""
Graph Search Algorithms
Author: Samir Acharya
"""
import numpy as np
from .graph import Cell
from .utils import trace_path
from collections import deque
import heapq

"""
General graph search implementation guidelines:

Begin by selecting an appropriate data structure to maintain a record of
visited cells and include the starting cell. Initialize any required
properties for the start cell as needed.

Then, develop the graph search implementation. Upon discovering a path,
utilize the trace_path() function to generate a path using the goal cell
and graph. Proper parent tracking for each node and a correctly implemented
graph.get_parent() function are essential. If no path exists, return
an empty list.

For visualizing visited cells in the navigation webapp, append each
visited cell to the graph's list using:
     graph.visited_cells.append(Cell(cell_i, cell_j))
where cell_i and cell_j represent the cell indices of the cell to visualize.
"""


def depth_first_search(graph, start, goal):
    """Depth First Search (DFS) implementation. Optional algorithm for P3.
    Args:
        graph: Graph instance to search.
        start: Starting cell as a Cell object.
        goal: Target cell as a Cell object.
    """
    graph.init_graph()  # Reset all node values to their initial state.

    """TODO (P3): Implement DFS (optional)."""
    if graph.check_cell_collision(start.i, start.j):
        print("Start cell in collision")
        return []
    if graph.check_collision(goal.i, goal.j):
        return []
    
    stack = [start]

    graph.nodes[start.j, start.i].visited = True
    graph.nodes[start.j, start.i].distance = 0

    while stack:
        current = stack.pop()

        graph.visited_celss.append(Cell(current.i, current.j))

        if current.i == goal.i and current.j == goal.j:
            return trace_path(goal, graph)
        
        neighbors = graph.find_neighbors(current.i, current.j)
        for neighbor in neighbors:
            if graph.nodes[neighbor.j, neighbor.i].visited:
                continue
            if graph.check_collision(neighbor.i, neighbor.j):
                continue

            graph.nodes[neighbor.j, neighbor.i].visited = True
            graph.nodes[neighbor.j, neighbor.i].parent = Cell(current.i, current.j)
            graph.nodes[neighbor.j, neighbor.i].distance = graph.nodes[current.j, current.i].distance + 1

            stack.append(neighbor)
        

    # Return empty list when no valid path exists.
    return []


def breadth_first_search(graph, start, goal):
    """Breadth First Search (BFS) implementation.
    Args:
        graph: Graph instance to search.
        start: Starting cell as a Cell object.
        goal: Target cell as a Cell object.
    """
    graph.init_graph()  # Reset all node values to their initial state.

    """TODO (P3): Implement BFS."""
    if graph.check_collision(start.i, start.j):
        print("Start cell in collision")
        return []
    if graph.check_collision(goal.i, goal.j):
        return []
    
    queue = deque([start])

    graph.nodes[start.j, start.i].visited = True
    graph.nodes[start.j, start.i].distance = 0

    while queue:
        current = queue.popleft()

        graph.visited_cells.append(Cell(current.i, current.j))

        if current.i == goal.i and current.j == goal.j:
            return trace_path(goal, graph)
        
        neighbors = graph.find_neighbors(current.i, current.j)
        for neighbor in neighbors:
            if graph.nodes[neighbor.j, neighbor.i].visited:
                continue
            if graph.check_collision(neighbor.i, neighbor.j):
                continue

            graph.nodes[neighbor.j, neighbor.i].visited = True
            graph.nodes[neighbor.j, neighbor.i].parent = Cell(current.i, current.j)
            graph.nodes[neighbor.j, neighbor.i].distance = graph.nodes[current.j, current.i].distance + 1

            queue.append(neighbor)


    # Return empty list when no valid path exists.
    return []

def heuristic(cell1, cell2):
    return abs(cell1.i - cell2.i) + abs(cell1.j - cell2.j)


def a_star_search(graph, start, goal):
    """A* Search implementation.
    Args:
        graph: Graph instance to search.
        start: Starting cell as a Cell object.
        goal: Target cell as a Cell object.
    """
    graph.init_graph()  # Reset all node values to their initial state.

    """TODO (P3): Implement A*."""
    if graph.check_collision(start.i, start.j):
        print("Start cell in collision")
        return []
    if graph.check_collision(goal.i, goal.j):
        return []
    
    counter = 0
    pq = [(0, counter, start)]

    graph.nodes[start.j, start.i].g_cost = 0
    graph.nodes[start.j, start.i].h_cost = heuristic(start, goal)
    graph.nodes[start.j, start.i].cost = graph.nodes[start.j, start.i].g_cost + graph.nodes[start.j, start.i].h_cost
    graph.nodes[start.j, start.i].visited = True

    while pq:
        f_cost, _, current = heapq.heappop(pq)

        graph.visited_cells.append(Cell(current.i, current.j))

        if current.i == goal.i and current.j == goal.j:
            return trace_path(goal, graph)
        
        current_g = graph.nodes[current.j, current.i].g_cost

        neighbors = graph.find_neighbors(current.i, current.j)
        for neighbor in neighbors:
            if graph.check_collision(neighbor.i, neighbor.j):
                continue

            tentative_g = current_g + 1

            if tentative_g < graph.nodes[neighbor.j, neighbor.i].g_cost:
                graph.nodes[neighbor.j, neighbor.i].parent = Cell(current.i, current.j)
                graph.nodes[neighbor.j, neighbor.i].g_cost = tentative_g
                graph.nodes[neighbor.j, neighbor.i].h_cost = heuristic(neighbor, goal)
                graph.nodes[neighbor.j, neighbor.i].cost = graph.nodes[neighbor.j, neighbor.i].g_cost + graph.nodes[neighbor.j, neighbor.i].h_cost

                if not graph.nodes[neighbor.j, neighbor.i].visited:
                    counter += 1
                    heapq.heappush(pq, (graph.nodes[neighbor.j, neighbor.i].cost, counter, neighbor))
                    graph.nodes[neighbor.j, neighbor.i].visited = True

    # Return empty list when no valid path exists.
    return []
