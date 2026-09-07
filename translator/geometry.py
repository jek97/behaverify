"""
Plain-Python geometry helpers -- no external dependencies (shapely etc.),
since the translator should run with only the same dependencies BehaVerify
itself already requires.
"""
import math


def point_segment_distance(px, py, ax, ay, bx, by):
    """Distance from point (px,py) to segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def point_in_polygon(px, py, vertices):
    """Standard ray-casting point-in-polygon test. vertices: list of (x,y)."""
    inside = False
    n = len(vertices)
    for i in range(n):
        ax, ay = vertices[i]
        bx, by = vertices[(i + 1) % n]
        if ((ay > py) != (by > py)):
            x_intersect = ax + (py - ay) * (bx - ax) / (by - ay)
            if px < x_intersect:
                inside = not inside
    return inside


def distance_to_polygon_boundary(px, py, vertices):
    """Min distance from (px,py) to the polygon's own boundary edges."""
    n = len(vertices)
    return min(
        point_segment_distance(px, py, vertices[i][0], vertices[i][1],
                                vertices[(i + 1) % n][0], vertices[(i + 1) % n][1])
        for i in range(n)
    )


def clearance_to_obstacles(px, py, polygons):
    """
    Signed-ish clearance to the nearest obstacle, in the same units as
    `polygons`' own coordinates (metres here): the distance to the nearest
    obstacle boundary, or 0.0 if (px,py) is actually inside some obstacle.
    polygons: list of list of (x,y) vertex tuples.
    """
    best = math.inf
    for vertices in polygons:
        if point_in_polygon(px, py, vertices):
            return 0.0
        best = min(best, distance_to_polygon_boundary(px, py, vertices))
    return best


def bresenham_cells(x0, y0, x1, y1):
    """
    Integer Bresenham line from grid cell (x0,y0) to (x1,y1) inclusive of
    both endpoints -- used to check every cell a one-tick move crosses
    (ObstacleOnPath needs the whole step's path, not just its endpoint).
    """
    cells = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return cells
