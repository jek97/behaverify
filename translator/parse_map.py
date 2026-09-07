"""
Parses a problem's obstacles_generated.pl (obstacle_polygon/2 facts, as
produced by occgrid_to_problog.py from map.yaml/map.pgm) into a rasterized,
per-grid-cell CLEARANCE table -- distance (in whole grid cells, floored so
the abstraction never OVER-estimates safety) to the nearest obstacle
boundary, capped at `cap_cells` since ObstacleInBound/ObstacleOnPath
thresholds are always small distances; cells farther than the cap all
share one 'far away' default value and never need an individual table
entry.

The raw map.pgm/occupancy_data.npy are NOT used -- obstacles_generated.pl
is already the same polygon geometry basic_action_theory.pl itself
consumes, so it's the authoritative source; the pgm/npy stay optional, for
visualization only.
"""
import re

from . import geometry

_POLY_RE = re.compile(
    r"obstacle_polygon\(\s*([A-Za-z0-9_]+)\s*,\s*\[(.*?)\]\s*\)\.",
    re.DOTALL,
)
_POINT_RE = re.compile(r"point\(\s*([-\d.eE]+)\s*,\s*([-\d.eE]+)\s*\)")


def parse_obstacles(path):
    """Returns a list of (obstacle_id, [(x,y), ...]) in the file's own units (metres)."""
    with open(path, 'r', encoding='utf-8') as f:
        text = f.read()
    obstacles = []
    for match in _POLY_RE.finditer(text):
        obstacle_id = match.group(1)
        points_text = match.group(2)
        vertices = [(float(x), float(y)) for x, y in _POINT_RE.findall(points_text)]
        if len(vertices) >= 3:
            obstacles.append((obstacle_id, vertices))
    return obstacles


class Grid:
    """
    A bounded, integer grid-cell coordinate system plus a static obstacle
    clearance table over it. x in [min_x, max_x], y in [min_y, max_y].
    """

    def __init__(self, min_x, max_x, min_y, max_y, clearance_by_cell, cap_cells):
        self.min_x, self.max_x = min_x, max_x
        self.min_y, self.max_y = min_y, max_y
        self.clearance_by_cell = clearance_by_cell  # {(x,y): clearance_cells}, only entries < cap
        self.cap_cells = cap_cells

    @property
    def width(self):
        return self.max_x - self.min_x + 1

    @property
    def height(self):
        return self.max_y - self.min_y + 1

    def flat_index(self, x, y):
        return (x - self.min_x) * self.height + (y - self.min_y)

    def clearance(self, x, y):
        return self.clearance_by_cell.get((x, y), self.cap_cells)


def compute_grid_bounds(obstacles_m, extra_points_m, config, margin_cells=2):
    """
    obstacles_m: list of (id, [(x,y)_metres, ...])
    extra_points_m: list of (x,y)_metres (start point, every goal point named
    in the tree) that must also fall inside the grid.
    """
    xs, ys = [], []
    for _, vertices in obstacles_m:
        for (x, y) in vertices:
            xs.append(x)
            ys.append(y)
    for (x, y) in extra_points_m:
        xs.append(x)
        ys.append(y)
    if not xs:
        raise ValueError('No obstacles or reference points given -- cannot size the grid.')
    min_x = config.to_cell(min(xs)) - margin_cells
    max_x = config.to_cell(max(xs)) + margin_cells
    min_y = config.to_cell(min(ys)) - margin_cells
    max_y = config.to_cell(max(ys)) + margin_cells
    return min_x, max_x, min_y, max_y


def build_clearance_grid(obstacles_m, config, bounds, cap_cells=5):
    min_x, max_x, min_y, max_y = bounds
    step = config.disc_step_position
    polygons = [vertices for (_, vertices) in obstacles_m]
    clearance_by_cell = {}
    for cx in range(min_x, max_x + 1):
        for cy in range(min_y, max_y + 1):
            # cell (cx,cy) represents the world point (cx*step, cy*step)
            clearance_m = geometry.clearance_to_obstacles(cx * step, cy * step, polygons)
            clearance_cells = int(clearance_m // step)  # floor -- never over-report safety
            if clearance_cells < cap_cells:
                clearance_by_cell[(cx, cy)] = clearance_cells
    return Grid(min_x, max_x, min_y, max_y, clearance_by_cell, cap_cells)
