"""
Robot Path Planning Script
Author: Samir Acharya
"""
import argparse
from mbot_bridge.api import MBot

from src.graph import GridGraph, Cell
from src.graph_search import a_star_search, breadth_first_search, depth_first_search
from src.utils import generate_plan_file


def cells_to_poses(path, g):
    return [[*g.cell_to_pos(c.i, c.j), 0] for c in path]


def parse_args():
    parser = argparse.ArgumentParser(description="HelloRob Path Planning on the Robot.")
    parser.add_argument("-m", "--map", type=str, default="/home/mbot/current.map", help="Path to the map file.")
    parser.add_argument("--goal", type=float, nargs=2, default=[0, 0], help="Goal position.")
    parser.add_argument("-r", "--collision-radius", type=float, default=15, help="Collision radius (meters).")
    parser.add_argument("--algo", type=str, default="astar", choices=["bfs", "dfs", "astar"],
                        help="Algorithm to use for path planning.")


    args = parser.parse_args()

    return args


if __name__ == "__main__":
    args = parse_args()

    # Instantiate the GridGraph with the robot's map. NOTE: Consider adjusting
    # the collision radius from the default value.
    graph = GridGraph(args.map, collision_radius=args.collision_radius)
    goal = graph.pos_to_cell(*args.goal)

    # Create an MBot instance.
    robot = MBot()
    # Retrieve the current SLAM pose and transform it into a starting cell.
    start_pose = robot.read_slam_pose()
    start = graph.pos_to_cell(*start_pose[:2])

    path = []
    # Execute the graph search algorithm and store the result in path.
    if args.algo == "astar":
        path = a_star_search(graph, start, goal)
    elif args.algo == "bfs":
        path = breadth_first_search(graph, start, goal)
    elif args.algo == "dfs":
        path = depth_first_search(graph, start, goal)
    else:
        print("Invalid option:", args.algo)
        exit(1)

    if not path:
        print("ERROR: No path found to the goal!")
        print("Possible reasons:")
        print("  - Goal is in collision (obstacle or too close to walls)")
        print("  - Goal is unreachable (blocked by obstacles)")
        print("  - Start position is in collision")
        print("\nTry:")
        print("  - Choosing a different goal position")
        print("  - Reducing the collision radius with -r flag")
        exit(1)

    



    # Command the robot to follow the computed path.
    print(f"Found path of length {len(path)}. Driving to the goal!")
    robot.drive_path(cells_to_poses(path, graph))

    # Create the visualization file for the planned path.
    generate_plan_file(graph, start, goal, path, algo=args.algo)
