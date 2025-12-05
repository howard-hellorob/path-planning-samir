"""
Simple test script to verify the path planning algorithms work correctly.
This creates a small test map and runs all three algorithms.
"""

import numpy as np
import sys
import os

# Add the src directory to path (adjust as needed for your structure)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.graph import GridGraph, Cell
from src.graph_search import breadth_first_search, depth_first_search, a_star_search

def create_test_map():
    """Create a simple test map with some obstacles."""
    width, height = 10, 10
    origin = (0, 0)
    meters_per_cell = 0.05
    
    # Create empty map (all cells free)
    cell_odds = np.full((height, width), -100, dtype=np.int8)
    
    # Add some obstacles (walls)
    # Vertical wall from (5, 2) to (5, 7)
    for j in range(2, 8):
        cell_odds[j, 5] = 100
    
    # Create small gap at (5, 5)
    cell_odds[5, 5] = -100
    
    return GridGraph(
        width=width,
        height=height,
        origin=origin,
        meters_per_cell=meters_per_cell,
        cell_odds=cell_odds,
        collision_radius=0.02,
        threshold=0
    )

def print_map_with_path(graph, path, start, goal):
    """Print a visual representation of the map with the path."""
    # Create character map
    char_map = []
    for j in range(graph.height):
        row = []
        for i in range(graph.width):
            if graph.is_cell_occupied(i, j):
                row.append('#')  # Obstacle
            else:
                row.append('.')  # Free space
        char_map.append(row)
    
    # Mark path
    for cell in path:
        if not (cell.i == start.i and cell.j == start.j) and not (cell.i == goal.i and cell.j == goal.j):
            char_map[cell.j][cell.i] = '*'
    
    # Mark start and goal
    char_map[start.j][start.i] = 'S'
    char_map[goal.j][goal.i] = 'G'
    
    # Print map
    print("Map (S=Start, G=Goal, *=Path, #=Obstacle, .=Free):")
    for row in char_map:
        print(' '.join(row))
    print()

def test_algorithm(graph, start, goal, algo_name, algo_func):
    """Test a single algorithm."""
    print(f"\n{'='*50}")
    print(f"Testing {algo_name}")
    print(f"{'='*50}")
    
    path = algo_func(graph, start, goal)
    
    if path:
        print(f"✓ Path found! Length: {len(path)} cells")
        print(f"✓ Visited {len(graph.visited_cells)} cells during search")
        print_map_with_path(graph, path, start, goal)
        
        # Verify path
        if path[0].i == start.i and path[0].j == start.j:
            print("✓ Path starts at start cell")
        else:
            print("✗ ERROR: Path doesn't start at start cell")
        
        if path[-1].i == goal.i and path[-1].j == goal.j:
            print("✓ Path ends at goal cell")
        else:
            print("✗ ERROR: Path doesn't end at goal cell")
        
        # Check path continuity
        valid_path = True
        for i in range(len(path) - 1):
            curr = path[i]
            next_cell = path[i + 1]
            # Check if adjacent (Manhattan distance = 1)
            dist = abs(curr.i - next_cell.i) + abs(curr.j - next_cell.j)
            if dist != 1:
                print(f"✗ ERROR: Path discontinuity between ({curr.i},{curr.j}) and ({next_cell.i},{next_cell.j})")
                valid_path = False
                break
        
        if valid_path:
            print("✓ Path is continuous")
        
    else:
        print("✗ No path found!")
    
    return path

def main():
    print("\n" + "="*60)
    print("PATH PLANNING ALGORITHM TEST")
    print("="*60)
    
    # Create test map
    print("\nCreating test map...")
    graph = create_test_map()
    
    # Define start and goal
    start = Cell(1, 5)  # Left side of wall
    goal = Cell(8, 5)   # Right side of wall
    
    print(f"Start: ({start.i}, {start.j})")
    print(f"Goal: ({goal.i}, {goal.j})")
    
    # Test BFS
    bfs_path = test_algorithm(graph, start, goal, "Breadth-First Search (BFS)", breadth_first_search)
    
    # Test DFS
    dfs_path = test_algorithm(graph, start, goal, "Depth-First Search (DFS)", depth_first_search)
    
    # Test A*
    astar_path = test_algorithm(graph, start, goal, "A* Search", a_star_search)
    
    # Compare results
    print(f"\n{'='*50}")
    print("COMPARISON")
    print(f"{'='*50}")
    
    if bfs_path:
        print(f"BFS:  Path length = {len(bfs_path)}, Cells visited = {len(graph.visited_cells)}")
    
    # Need to re-run to get fresh visited cells count
    graph.init_graph()
    dfs_path_test = depth_first_search(graph, start, goal)
    if dfs_path_test:
        print(f"DFS:  Path length = {len(dfs_path_test)}, Cells visited = {len(graph.visited_cells)}")
    
    graph.init_graph()
    astar_path_test = a_star_search(graph, start, goal)
    if astar_path_test:
        print(f"A*:   Path length = {len(astar_path_test)}, Cells visited = {len(graph.visited_cells)}")
    
    print("\nNote: BFS and A* should find optimal (shortest) paths.")
    print("      DFS may find a longer path but might visit fewer cells.")
    print("\n✓ All tests completed!")

if __name__ == "__main__":
    main()