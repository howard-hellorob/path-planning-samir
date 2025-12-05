"""
Utility Functions for Path Planning
Author: Samir Acharya
"""
import json
from .graph import Cell


def trace_path(cell, graph):
    """Reconstructs a path by following parent references from the given cell."""
    path = []
    while cell is not None:
        path.append(Cell(cell.i, cell.j))
        cell = graph.get_parent(cell)
    # Invert the path order since it was built from goal back to start.
    path.reverse()
    return path


def generate_plan_file(graph, start, goal, path, algo="", out_name="out.planner"):
    """Creates a planner output file for use with the navigation web application.

    Access the web app at: hellorob.org/nav-app
    """
    print(f"Saving planning data to file: {out_name}")

    path_data = [[cell.i, cell.j] for cell in path]
    visited_cells_data = [[cell.i, cell.j] for cell in graph.visited_cells]

    plan = {
        "path": path_data,
        "visited_cells": visited_cells_data,
        "dt": [],
        "map": graph.as_string(),  # Convert graph to string representation
        "start": [start.i, start.j],
        "goal": [goal.i, goal.j],
        "planning_algo": algo
    }

    with open(out_name, 'w') as outfile:
        json.dump(plan, outfile)
