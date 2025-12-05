# Project 3: Path Planning - Python (F23)

**Author: Samir Acharya**

This repository contains the implementation for Project 3 (Path Planning) in Python. 
For detailed project instructions, visit the [project page](https://hellorob.org/projects/p3).

## Overview

This project implements various graph search algorithms (A*, BFS, DFS) for robot path planning. 
The codebase includes both a command-line interface for testing on maps and a robot integration 
script for executing paths on physical robots.

## Testing Path Planning Algorithms on Maps

A visualization tool is provided to test path planning algorithms and view results in the 
[navigation web application](https://hellorob.org/nav-app/). For a detailed tutorial on using 
the web app, see [this guide](https://hellorob.org/tutorials/app).

### Basic Usage

Run the path planner from your local machine (requires Python 3):

```bash
python path_planner_cli.py -m [PATH/TO/MAP] --start [START_i START_j] --goal [GOAL_i GOAL_j]
```

### Parameters

- **Map file**: Use any `.map` file from the `data/` directory, or create your own. Maps from 
  the robot can also be used for testing.
- **Start and Goal**: Specify cell indices (i, j) for the start and goal positions. You can 
  upload maps to the web app to interactively select appropriate start and goal cells.
- **Algorithm**: Choose the search algorithm using `--algo [astar | bfs | dfs]`. The default 
  algorithm is `bfs`.

### Example Commands

```bash
# Test A* algorithm on maze1
python3 path_planner_cli.py -m data/maze1.map --start 10 10 --goal 50 50 --algo astar
```

**A* Algorithm Demo Video**: Watch a demonstration of the A* path planning algorithm in action: [Google Drive Video](https://drive.google.com/file/d/1YjBsyd4LMe29a9l-XCTw-uWRbrITsDlI/view?usp=sharing)

```bash
# Test BFS on a different map
python3 path_planner_cli.py -m data/maze2.map --start 5 5 --goal 30 30 --algo bfs

# Test DFS (optional algorithm)
python3 path_planner_cli.py -m data/test_map.map --start 0 0 --goal 20 20 --algo dfs
```

The script generates an `out.planner` file that can be uploaded to the web app for visualization.

For additional options, run:
```bash
python path_planner_cli.py -h
```

## Running Path Planning on the Robot

To execute path planning on a physical robot, you must first complete the mapping and localization 
process. Refer to the [mapping tutorial](https://hellorob.org/mbot/mapping) for instructions.

### Basic Robot Commands

Once the robot is mapped and localized, run:

```bash
python robot_plan_path.py --goal [GOAL_x GOAL_y]
```

### Robot Command Parameters

- **Goal position**: Provide the target coordinates in meters (x, y). Use the robot's web interface 
  to click on a desired cell and note its coordinates.
- **Collision radius**: Adjust the collision radius (in meters) using `-r [RADIUS]`. The default 
  is 0.15 meters. You may need to modify this based on your robot's size and environment.
- **Algorithm**: Select the path planning algorithm with `--algo [astar | bfs | dfs]`. The default 
  is `astar`.

### Example Robot Commands

```bash
# Navigate to a goal position using A* (default)
python robot_plan_path.py --goal 2.5 3.0

# Use BFS with a custom collision radius
python robot_plan_path.py --goal 1.5 2.0 --algo bfs -r 0.2

# Use DFS with a smaller collision radius
python robot_plan_path.py --goal 3.0 1.5 --algo dfs -r 0.1
```

For complete usage information, run:
```bash
python robot_plan_path.py -h
```

## Implementation Details

The project includes implementations of:
- **A* Search**: Optimal pathfinding using heuristics
- **Breadth-First Search (BFS)**: Guarantees shortest path in unweighted graphs
- **Depth-First Search (DFS)**: Explores paths deeply before backtracking (optional)

All algorithms support collision checking with configurable robot radius and work with occupancy 
grid maps.
