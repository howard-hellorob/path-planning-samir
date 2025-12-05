"""
Graph Representation for Path Planning
Author: Samir Acharya
"""
import os
import numpy as np


class Cell(object):
    def __init__(self, i, j):
        self.i = i  # Column index along x-axis
        self.j = j  # Row index along y-axis


"""NOTE: Consider creating a class to hold node information. Define it
here if needed."""

class Node(object):
    def __init__(self):
        self.parent = None
        self.distance = float('inf')
        self.cost = float('inf')
        self.g_cost = float('inf')
        self.h_cost = 0
        self.visited = False


class GridGraph:
    """Utility class for representing an occupancy grid map as a graph structure."""
    def __init__(self, file_path=None, width=-1, height=-1, origin=(0, 0),
                 meters_per_cell=0, cell_odds=None, collision_radius=0.15, threshold=-100):
        """Constructor for the GridGraph class.

        Args:
            file_path: File path for loading the map. When specified, all map
                       properties are read from this file.
            width: Width of the map measured in cells.
            height: Height of the map measured in cells.
            origin: The (x, y) position in meters corresponding to cell (0, 0).
            meters_per_cell: Physical size of a single cell in meters.
            cell_odds: Array of size width * height with occupancy probabilities
                       in range [127, -128]. Higher values indicate greater
                       likelihood of occupancy.
            collision_radius: Radius used for collision detection in meters.
            threshold: Occupancy threshold above which cells are considered obstacles.
        """
        if file_path is not None:
            # When a file path is given, load the map data.
            assert self.load_from_file(file_path)
        else:
            self.width = width
            self.height = height
            self.origin = origin
            self.meters_per_cell = meters_per_cell
            self.cell_odds = cell_odds

        self.threshold = threshold
        self.set_collision_radius(collision_radius)
        self.visited_cells = []  # Maintains ordered list of visited cells for visualization.

        # Define additional member variables for storing node information.
        self.nodes = None

    def as_string(self):
        """Converts the map data into a string format suitable for visualization."""
        map_list = self.cell_odds.astype(str).tolist()
        rows = [' '.join(row) for row in map_list]
        cell_data = ' '.join(rows)
        header_data = f"{self.origin[0]} {self.origin[1]} {self.width} {self.height} {self.meters_per_cell}"
        return ' '.join([header_data, cell_data])

    def load_from_file(self, file_path):
        """Reads and loads map data from the specified file."""
        if not os.path.isfile(file_path):
            print(f'ERROR: loadFromFile: Failed to load from {file_path}')
            return False

        with open(file_path, 'r') as file:
            header = file.readline().split()
            origin_x, origin_y, self.width, self.height, self.meters_per_cell = map(float, header)
            self.origin = (origin_x, origin_y)
            self.width = int(self.width)
            self.height = int(self.height)

            # Validate parameter values.
            if self.width < 0 or self.height < 0 or self.meters_per_cell < 0.0:
                print('ERROR: loadFromFile: Incorrect parameters')
                return False

            # Initialize the occupancy odds array.
            self.cell_odds = np.zeros((self.height, self.width), dtype=np.int8)

            # Process each cell's occupancy value.
            for r in range(self.height):
                row = file.readline().strip().split()
                for c in range(self.width):
                    self.cell_odds[r, c] = np.int8(row[c])

        return True

    def pos_to_cell(self, x, y):
        """Transforms a global position into its corresponding graph cell.
        Args:
            x: Global x-coordinate in meters.
            y: Global y-coordinate in meters.
        Returns:
            Cell coordinates within the graph.
        """
        i = int(np.floor((x - self.origin[0]) / self.meters_per_cell))
        j = int(np.floor((y - self.origin[1]) / self.meters_per_cell))

        return Cell(i, j)

    def cell_to_pos(self, i, j):
        """Transforms graph cell coordinates into their corresponding global position.
        Args:
            i: Column index (x-axis) of the cell in the graph.
            j: Row index (y-axis) of the cell in the graph.
        Returns:
            Tuple with global position coordinates (x, y)."""
        x = (i + 0.5) * self.meters_per_cell + self.origin[0]
        y = (j + 0.5) * self.meters_per_cell + self.origin[1]
        return x, y

    def is_cell_in_bounds(self, i, j):
        """Verifies if the specified cell lies within the graph boundaries."""
        return i >= 0 and i < self.width and j >= 0 and j < self.height

    def is_cell_occupied(self, i, j):
        """Determines if the specified graph cell is occupied (exceeds the threshold)."""
        return self.cell_odds[j, i] >= self.threshold

    def set_collision_radius(self, r):
        """Configures the collision radius and precomputes values for efficient
        collision checking.
        Args:
            r: Collision radius specified in meters.
        """
        r_cells = int(np.ceil(r / self.meters_per_cell))  # Convert radius to cell units.
        # Generate indices for a mask that covers the robot's footprint.
        r_indices, c_indices = np.indices((2 * r_cells - 1, 2 * r_cells - 1))
        c = r_cells - 1  # Mask center coordinate.
        dists = (r_indices - c)**2 + (c_indices - c)**2  # Squared distances from center.
        # Identify indices that fall within the collision radius.
        self._coll_ind_j, self._coll_ind_i = np.nonzero(dists <= (r_cells - 1)**2)

        # Store the radius configuration.
        self.collision_radius = r
        self.collision_radius_cells = r_cells

    def check_collision(self, i, j):
        """Determines if this cell results in a collision using the configured
        collision radius."""
        # Apply the precomputed robot radius mask to verify if any surrounding
        # cells within the radius are occupied.
        j_inds = self._coll_ind_j + j - (self.collision_radius_cells - 1)
        i_inds = self._coll_ind_i + i - (self.collision_radius_cells - 1)

        # Compute which mask indices remain within grid boundaries after
        # shifting to the target cell.
        in_bounds = np.bitwise_and(np.bitwise_and(j_inds >= 0, j_inds < self.height),
                                   np.bitwise_and(i_inds >= 0, i_inds < self.width))

        return np.any(self.is_cell_occupied(i_inds[in_bounds], j_inds[in_bounds]))

    def get_parent(self, cell):
        """Retrieves a Cell object for the parent of the specified cell, or
        None if no parent exists. Used for path reconstruction following
        graph search."""
        # Return the parent node for the cell.
        if self.nodes is None:
            return None
        
        return self.nodes[cell.j, cell.i].parent

    def init_graph(self):
        """Prepares node data structures in the graph before executing graph search.

        At this point, the graph has already loaded its structural properties
        such as width, height, and cell occupancy values. Use these properties
        to set up additional node attributes like distances and node objects."""
        self.visited_cells = []  # Clear the visited cells list for new search.

        # Initialize all graph nodes.
        self.nodes = np.empty((self.height, self.width), dtype=object)
        for j in range(self.height):
            for i in range(self.width):
                self.nodes[j,i] = Node()

    def find_neighbors(self, i, j):
        """Generates a list of neighboring cells for the specified cell. Excludes
        any cells that fall outside the graph boundaries."""
        nbrs = []
        # Collect all valid neighbors of the cell at (i, j), ensuring
        # they remain within graph boundaries.
        directions = [
            (0,1),
            (0,-1),
            (1,0),
            (-1,0)
        ]

        for di, dj in directions:
            ni , nj = i + di, j + dj
            if self.is_cell_in_bounds(ni, nj):
                nbrs.append(Cell(ni, nj))

        # Use is_cell_in_bounds() to filter out invalid neighbors.
        return nbrs
