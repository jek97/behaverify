#!/usr/bin/env python3
"""
planners.py

Lives in module/theory/ alongside basic_action_theory.pl and
collision_geometry.py -- see this project's top-level layout note
(basic_action_theory.pl's own header comment) for why these are grouped
together. module/contracts/bt_actions.py (the BT.cpp-facing side of the
same node set) imports the plain-Python API below from here.

TWO independent roles, kept in ONE file because they share the exact
same underlying A*/spline computation and neither should reimplement it:

  1. PROBLOG EXTERNAL-PREDICATE MODULE -- imported by ProbLog's own
     :- use_module('./planners.py'). directive inside
     basic_action_theory.pl (see problog.clausedb.ClauseDB.load_external_
     module, which execs this file as a plain Python module the moment
     that directive is loaded, triggering the
     @problog_export_nondet(...)-decorated functions below to register
     themselves as callable ProbLog predicates:
         plan_astar(+SX,+SY,+GX,+GY, -ControlPoints)
         plan_straight(+SX,+SY,+GX,+GY, -ControlPoints)
         plan_voronoi(+SX,+SY,+GX,+GY, -ControlPoints)
         follow_boarder(+SX,+SY,+ObstacleId,+Offset, -ControlPoints)
     ControlPoints = [point(X0,Y0), ...] ProbLog terms, length 3k+1, in
     EXACTLY the format basic_action_theory.pl's spline_point/4 expects.
     plan_voronoi is a generalized-Voronoi-diagram planner, SAME
     interface as plan_astar/plan_straight -- see
     _voronoi_control_points' own header for the full geometry.
     follow_boarder is a boundary-following planner shared by every
     Bug-algorithm variant: it just plans a full clockwise loop around
     an obstacle's offset boundary and does NOT decide when to leave it
     -- that decision belongs to whichever TRIGGER halts the subsequent
     moveto_leg walking this planner's own output (see collision_
     geometry.py's "BUG-ALGORITHM BOUNDARY-LEAVE PRIMITIVES" section).
     There is deliberately no Goal parameter here -- see
     _follow_boarder_control_points's own header.

  2. PLAIN-PYTHON API for a BT.cpp / bt_actions.py caller that has
     nothing to do with ProbLog -- plan_astar_points/plan_straight_points/
     plan_voronoi_points/follow_boarder_points below, returning plain
     (x,y) float tuples, no ProbLog Term objects and no ProbLog import
     required to call them. The `import problog` needed for role 1 is
     wrapped in a try/except (_HAVE_PROBLOG) so that importing this
     module from a pure BT.cpp bridge that never installs ProbLog still
     works -- only the ProbLog predicates themselves become unavailable
     in that case, exactly mirroring how plan_astar already degrades
     gracefully (fails rather than crashing) when the map itself fails
     to load.

Each role funnels through the SAME plain-Python core per planner
(_astar_control_points / _straight_control_points /
_voronoi_control_points / _follow_boarder_control_points) -- no
duplicated logic between the ProbLog-facing and BT.cpp-facing entry
points for any of the four.

FAILURE: if A* finds no path (start/goal unreachable, or the map failed
to load at import time), the core returns None -- the ProbLog predicate
then returns zero solutions (i.e. the predicate call FAILS, exactly like
any other "this doesn't happen" case elsewhere in this theory, e.g.
first_collision_time), and the plain-Python function returns None for
its own BT.cpp caller to check. plan_straight/plan_straight_points only
fail on degenerate (non-finite) input; a straight line between two
ordinary points always exists.

Map location: loaded ONCE, at import time (i.e. once per process), from
<problem>/map.yaml, where <problem> is problems/problem0/ by default or
whichever directory the BT_PROBLEM_DIR environment variable names --
main.py sets BT_PROBLEM_DIR before ProbLog loads this module, so a run
against a different --problem picks up that problem's own map without
this file ever changing. See MAP_YAML_PATH below. If the map can't be
loaded, both the ProbLog predicate and the plain-Python function still
work, but astar-based planning will always fail (no map to plan
against) rather than crashing at import time.

MAP/A*/SPLINE LIBRARY CODE below (OccupancyGridMap, load_map_yaml,
inflate_obstacles, astar, fit_spline, bspline_to_bezier_chain) used to
live in a separate standalone interactive click-planner tool
(occupancy_grid_planner.py, now removed -- its own GUI/CLI were never
part of the live pipeline, only these library functions were still in
use, imported from here); merged directly into this file since this
module is now their only caller.
"""
import heapq
import math
import os
import re

import numpy as np
from scipy.interpolate import splprep, insert
from scipy.ndimage import binary_dilation
from scipy.spatial import Voronoi

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))

# Which problem's own data (map.yaml, obstacles_generated.pl) to load --
# set by main.py via BT_PROBLEM_DIR before this module is imported;
# defaults to problems/problem0/ so this module still works standalone
# (e.g. `problog basic_action_theory.pl` run directly, or this file
# imported directly for testing) with no environment variable set.
_DEFAULT_PROBLEM_DIR = os.path.join(_PROJECT_ROOT, "problems", "problem0")
_PROBLEM_DIR = os.environ.get("BT_PROBLEM_DIR", _DEFAULT_PROBLEM_DIR)


# --------------------------------------------------------------------------
# Map representation + loading (formerly occupancy_grid_planner.py)
# --------------------------------------------------------------------------
class OccupancyGridMap:
    """
    Minimal stand-in for the information carried by a nav_msgs/OccupancyGrid
    message.

    data       : 2D int array, shape (height, width). data[row, col] is the
                 occupancy of the cell whose world position is
                 (origin_x + col*resolution, origin_y + row*resolution).
                 Values: 0..100 = occupancy probability, -1 = unknown.
                 (This is exactly msg.data reshaped to (msg.info.height,
                 msg.info.width), i.e. row-major with row 0 = min y.)
    resolution : meters per cell (msg.info.resolution)
    origin     : [x, y, yaw] world pose of cell (0,0)'s corner (msg.info.origin)
    width      : number of columns  (msg.info.width)
    height     : number of rows     (msg.info.height)
    """

    def __init__(self, data, resolution, origin, width, height):
        self.data = data
        self.resolution = float(resolution)
        self.origin = list(origin)
        self.width = int(width)
        self.height = int(height)

    def world_to_grid(self, x, y):
        col = int(math.floor((x - self.origin[0]) / self.resolution))
        row = int(math.floor((y - self.origin[1]) / self.resolution))
        return row, col

    def grid_to_world(self, row, col):
        x = self.origin[0] + (col + 0.5) * self.resolution
        y = self.origin[1] + (row + 0.5) * self.resolution
        return x, y

    def in_bounds(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    def is_free(self, row, col, occ_thresh=50, unknown_is_occupied=True):
        if not self.in_bounds(row, col):
            return False
        v = self.data[row, col]
        if v == -1:
            return not unknown_is_occupied
        return v < occ_thresh

    def copy_with_data(self, new_data):
        return OccupancyGridMap(new_data, self.resolution, self.origin,
                                 self.width, self.height)


def load_map_yaml(yaml_path):
    """Load a standard ROS map_server yaml + image pair."""
    import yaml
    from PIL import Image

    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    image_path = meta["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    resolution = float(meta["resolution"])
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    negate = int(meta.get("negate", 0))
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))

    img = Image.open(image_path)
    if img.mode != "L":
        img = img.convert("L")
    img_arr = np.array(img, dtype=np.float64)  # row 0 = top of image

    if negate:
        occ = img_arr / 255.0
    else:
        occ = (255.0 - img_arr) / 255.0

    grid = np.full(occ.shape, -1, dtype=np.int8)
    grid[occ > occupied_thresh] = 100
    grid[occ < free_thresh] = 0
    # cells in between stay -1 (unknown)

    # Image row 0 is the top of the picture (max y). OccupancyGrid row 0 is
    # y = origin_y (min y). Flip vertically to match the OccupancyGrid convention.
    grid = np.flipud(grid).copy()

    height, width = grid.shape
    return OccupancyGridMap(grid, resolution, origin, width, height)


def inflate_obstacles(grid_map, radius_m):
    """Return a copy of grid_map where obstacles (and unknown cells) have
    been grown by radius_m (a simple circular Minkowski dilation), useful
    for keeping a finite-size robot away from walls."""
    if radius_m <= 0:
        return grid_map
    radius_cells = max(1, int(round(radius_m / grid_map.resolution)))
    occ_mask = (grid_map.data == 100) | (grid_map.data == -1)

    yy, xx = np.ogrid[-radius_cells:radius_cells + 1, -radius_cells:radius_cells + 1]
    disk = (xx ** 2 + yy ** 2) <= radius_cells ** 2

    inflated_mask = binary_dilation(occ_mask, structure=disk)
    new_data = grid_map.data.copy()
    new_data[inflated_mask] = 100
    return grid_map.copy_with_data(new_data)


# --------------------------------------------------------------------------
# A* (formerly occupancy_grid_planner.py)
# --------------------------------------------------------------------------
def astar(grid_map, start_rc, goal_rc, occ_thresh=50, connectivity=8,
          unknown_is_occupied=True):
    """8- or 4-connected A* over grid_map. Returns a list of (row, col)
    cells from start to goal (inclusive), or None if no path exists."""

    if not grid_map.is_free(*start_rc, occ_thresh, unknown_is_occupied):
        return None
    if not grid_map.is_free(*goal_rc, occ_thresh, unknown_is_occupied):
        return None

    if connectivity == 8:
        steps = [(-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)),
                 (0, -1, 1.0), (0, 1, 1.0),
                 (1, -1, math.sqrt(2)), (1, 0, 1.0), (1, 1, math.sqrt(2))]
    else:
        steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]

    def heuristic(rc):
        return math.hypot(rc[0] - goal_rc[0], rc[1] - goal_rc[1])

    open_heap = [(heuristic(start_rc), 0.0, start_rc)]
    came_from = {}
    g_score = {start_rc: 0.0}
    closed = set()

    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal_rc:
            return _reconstruct_path(came_from, current)
        closed.add(current)

        for dr, dc, step_cost in steps:
            nr, nc = current[0] + dr, current[1] + dc
            neighbor = (nr, nc)
            if not grid_map.is_free(nr, nc, occ_thresh, unknown_is_occupied):
                continue
            # don't let the path cut diagonally through a corner of two obstacles
            if dr != 0 and dc != 0:
                if not grid_map.is_free(current[0] + dr, current[1], occ_thresh, unknown_is_occupied):
                    continue
                if not grid_map.is_free(current[0], current[1] + dc, occ_thresh, unknown_is_occupied):
                    continue

            tentative_g = g + step_cost
            if tentative_g < g_score.get(neighbor, float("inf")):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f_score = tentative_g + heuristic(neighbor)
                heapq.heappush(open_heap, (f_score, tentative_g, neighbor))

    return None


def _reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# --------------------------------------------------------------------------
# Spline fitting (formerly occupancy_grid_planner.py)
# --------------------------------------------------------------------------
def fit_spline(path_xy, degree=3, smoothing=0.0):
    """Fit a parametric B-spline through the (x, y) waypoints.
    Returns (tck, u) as produced by scipy.interpolate.splprep, where
    tck = (knots t, coefficients c=[cx, cy], degree k)."""
    path_xy = np.asarray(path_xy, dtype=float)
    x, y = path_xy[:, 0], path_xy[:, 1]
    n = len(x)
    if n < 2:
        raise ValueError("Need at least 2 waypoints to fit a spline")
    k = max(1, min(degree, n - 1))
    tck, u = splprep([x, y], k=k, s=smoothing)
    return tck, u


def bspline_to_bezier_chain(tck):
    """
    Convert a scipy parametric B-spline tck=(t,c,k) into an EXACT chain of
    degree-k Bezier segments, via full knot insertion: every interior knot
    is raised to multiplicity == k (the degree), at which point consecutive
    groups of (k+1) control points ARE the Bezier control points of one
    segment, with consecutive segments sharing their boundary point --
    i.e. exactly the "length 3k+1 for k cubic segments" format
    basic_action_theory.pl expects (for the standard, and only currently
    supported, case k=3).

    This is a LOSSLESS conversion, not a resampling/approximation -- the
    resulting Bezier chain reproduces the original B-spline curve exactly
    (verified numerically to floating-point precision). It says nothing
    about the ORIGINAL knot vector's spacing being preserved as segment
    boundaries in basic_action_theory.pl's own u-parametrization: that
    file always treats each segment as an equal 1/k share of its own u in
    [0,1] (see its spline_point/4), using arc-length integration -- NOT
    this B-spline's own (possibly non-uniform) knot spacing -- to
    determine timing. That's fine: only the CURVE SHAPE needs to transfer
    exactly, which it does; timing/speed along it is independently
    handled by basic_action_theory.pl's own arc-length machinery either
    way.

    Returns (control_points, degree) where control_points is a list of
    (x,y) tuples of length 3*num_segments + 1 for degree k=3.
    Raises ValueError if the fitted spline's degree isn't 3, since that's
    the only degree basic_action_theory.pl's Bezier evaluator supports.
    """
    t, c, k = tck
    if k != 3:
        raise ValueError(
            f"bspline_to_bezier_chain: got degree k={k}, but "
            f"basic_action_theory.pl only supports CUBIC (k=3) Bezier "
            f"segments.")

    t = list(t)
    c = [list(comp) for comp in c]
    tck2 = (t, c, k)

    # interior knots = all knots strictly between the (k+1)-fold repeated
    # boundary knots at each end
    interior_knots = sorted(set(t[k + 1: len(t) - (k + 1)]))
    for knot in interior_knots:
        current_mult = t.count(knot)
        needed = k - current_mult
        if needed > 0:
            tck2 = insert(knot, tck2, m=needed, per=0)
            t = list(tck2[0])

    t_final, c_final, k_final = tck2
    n_ctrl = len(t_final) - k_final - 1
    cx, cy = c_final[0][:n_ctrl], c_final[1][:n_ctrl]
    control_points = list(zip(cx, cy))

    if (len(control_points) - 1) % k_final != 0:
        raise ValueError(
            "bspline_to_bezier_chain: extraction did not yield a clean "
            "3k+1-length control point list -- this shouldn't happen for "
            "a clamped cubic B-spline; check the input spline's knot "
            "vector.")

    return control_points, k_final


MAP_YAML_PATH = os.path.join(_PROBLEM_DIR, "map.yaml")
_OBSTACLES_PATH = os.path.join(_PROBLEM_DIR, "obstacles_generated.pl")

# Same default inflation as this project's own former --inflate default.
PLANNING_INFLATE_M = 0.5
OCC_THRESH = 50
CONNECTIVITY = 8

# -- load the map ONCE at import time (i.e. once per process) ------------
try:
    _RAW_MAP = load_map_yaml(MAP_YAML_PATH)
    _PLANNING_MAP = inflate_obstacles(_RAW_MAP, PLANNING_INFLATE_M) \
        if PLANNING_INFLATE_M > 0 else _RAW_MAP
    _MAP_LOAD_ERROR = None
except Exception as exc:  # noqa: BLE001 -- deliberately broad: ANY load
    # failure should degrade to "astar planning always fails", not crash
    # the caller (ProbLog's grounding process, or a BT.cpp bridge).
    _RAW_MAP = None
    _PLANNING_MAP = None
    _MAP_LOAD_ERROR = exc


# -- load obstacle polygons ONCE at import time, for follow_boarder ------
# Same source file, same tiny regex parser as collision_geometry.py's own
# OBSTACLE_POLYGONS -- DELIBERATELY duplicated rather than imported from
# there: both files are independently `:- use_module(...)`'d by ProbLog
# (problog.clausedb.load_external_module execs each one fresh via its own
# SourceFileLoader, with problog_export.database pointed at whichever
# ClauseDB is loading it at that moment), and having one black-box module
# `import` the other would run its @problog_export_nondet-decorated
# predicates a second time under an unverified context -- not a risk
# worth taking for ~15 lines of stable parsing code. Every other black
# box in this project already owns its own data loading independently
# (this file's own map.yaml load above, collision_geometry.py's
# obstacles_generated.pl load) -- this follows the same convention.
_POINT_RE = re.compile(r"point\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)")


def _strip_prolog_comments(text):
    return re.sub(r"%.*$", "", text, flags=re.MULTILINE)


def _parse_obstacle_polygons(text):
    """Returns [(id, [(x,y),...]), ...] -- see collision_geometry.py's
    own identical function for the full rationale."""
    polys = []
    for m in re.finditer(r"obstacle_polygon\(([^,]+),\s*\[(.*?)\]\s*\)\s*\.", text, re.S):
        obstacle_id = m.group(1).strip()
        pts = [(float(x), float(y)) for x, y in _POINT_RE.findall(m.group(2))]
        if len(pts) >= 3:
            polys.append((obstacle_id, pts))
    return polys


try:
    with open(_OBSTACLES_PATH) as f:
        _OBSTACLE_POLYGONS = _parse_obstacle_polygons(_strip_prolog_comments(f.read()))
except FileNotFoundError:
    _OBSTACLE_POLYGONS = []


# =====================================================================
# SHARED CORE -- plain Python, no ProbLog types anywhere here. Both the
# ProbLog-facing predicates and the BT.cpp-facing plain functions below
# call straight into this.
# =====================================================================
def _straight_control_points(sx, sy, gx, gy):
    """A single cubic Bezier segment, collinear control points -- reduces
    EXACTLY to a straight line (see bspline_to_bezier_chain's own note on
    this being the degenerate case of a Bezier chain)."""
    dx, dy = gx - sx, gy - sy
    p0 = (sx, sy)
    p1 = (sx + dx / 3.0, sy + dy / 3.0)
    p2 = (sx + 2.0 * dx / 3.0, sy + 2.0 * dy / 3.0)
    p3 = (gx, gy)
    return [p0, p1, p2, p3]


def _astar_control_points(sx, sy, gx, gy):
    """Black-box A* planner: world-frame (sx,sy) -> (gx,gy), via the map
    loaded at import time. Returns None if the map didn't load, if
    start/goal fall outside the map or on an obstacle cell, or if A*
    finds no path at all -- the two callers below turn that into
    whatever "no path" signal fits their own interface (a failed
    ProbLog predicate call, or a plain None)."""
    if _PLANNING_MAP is None:
        return None

    start_rc = _PLANNING_MAP.world_to_grid(sx, sy)
    goal_rc = _PLANNING_MAP.world_to_grid(gx, gy)

    if not _PLANNING_MAP.in_bounds(*start_rc) or not _PLANNING_MAP.in_bounds(*goal_rc):
        return None

    if start_rc == goal_rc:
        # Already there -- a zero-length, degenerate but VALID Bezier
        # (all four control points identical).
        return [(sx, sy)] * 4

    path_rc = astar(_PLANNING_MAP, start_rc, goal_rc,
                     occ_thresh=OCC_THRESH, connectivity=CONNECTIVITY,
                     unknown_is_occupied=True)
    if path_rc is None:
        return None  # no path found

    path_xy = [_PLANNING_MAP.grid_to_world(r, c) for r, c in path_rc]

    if len(path_xy) < 2:
        return [(sx, sy)] * 4

    try:
        tck, _u = fit_spline(path_xy, degree=3, smoothing=0.0)
        control_points, _k = bspline_to_bezier_chain(tck)
    except ValueError:
        # Degenerate/too-short path for a cubic fit -- fall back to a
        # straight line between the actual start/goal rather than failing
        # outright (A* DID find connectivity; only the spline fit failed).
        control_points = _straight_control_points(sx, sy, gx, gy)

    return control_points


# =====================================================================
# FOLLOW_BOARDER -- boundary-following planner shared by every Bug-
# algorithm variant. From the CURRENT position, traces a FULL
# CLOCKWISE loop around one named obstacle's own boundary, offset
# outward by a caller-given distance -- and, UNLIKE an earlier version
# of this planner, does NOT decide when to stop: WHICH bug variant a
# leg implements (Bug0's "leave as soon as goal is visible" vs Bug2's
# "leave where the walk re-crosses the original start-goal line, closer
# to goal than where it left it") is entirely a matter of which
# TRIGGER halts the subsequent moveto_leg(CP,[...]) that walks this
# planner's own output -- see collision_geometry.py's "BUG-ALGORITHM
# BOUNDARY-LEAVE PRIMITIVES" section and basic_action_theory.pl's
# trigger_crossing_time/10 clauses for line_of_sight_clear/
# crosses_segment. Reuses the SAME fit_spline/bspline_to_bezier_chain
# fitting step _astar_control_points uses above, so its own output is a
# chained-cubic-Bezier control_points list in exactly the same format,
# no special-casing needed downstream.
# =====================================================================
def _polygon_edges(points):
    closed = list(points) + [points[0]]
    return list(zip(closed[:-1], closed[1:]))


def _edge_crosses(px, py, ax, ay, bx, by):
    if (ay > py and by <= py) or (by > py and ay <= py):
        x_cross = ax + (py-ay)/(by-ay)*(bx-ax)
        return px < x_cross
    return False


def _inside_polygon(px, py, points):
    count = sum(1 for (ax, ay), (bx, by) in _polygon_edges(points)
                if _edge_crosses(px, py, ax, ay, bx, by))
    return count % 2 == 1


def _signed_polygon_area(points):
    return sum(ax*by - bx*ay for (ax, ay), (bx, by) in _polygon_edges(points)) / 2.0


# Geometry constants for the boundary walk -- planner-internal tuning,
# not routed through config.yaml, matching this file's own existing
# precedent (PLANNING_INFLATE_M/OCC_THRESH/CONNECTIVITY above are
# plain module constants too, not config.yaml-sourced -- these are
# planner-geometry knobs, not action-theory tunables that flow into
# ProbLog's own noise/trigger model).
_BOUNDARY_SAMPLES_PER_EDGE = 8
_NORMAL_PROBE_EPS = 1.0e-3


def _offset_boundary_clockwise(polygon, offset):
    """Dense samples along `polygon`'s own boundary, each pushed
    outward by `offset` along its own edge's outward normal (a
    per-edge offset, not a true mitred polygon offset -- a reasonable
    approximation at this project's abstraction level, given the
    result gets spline-fit afterward anyway, same spirit as A*'s own
    grid-path-then-fit-spline pipeline above), ordered so that walking
    the returned list IN ORDER traces the boundary CLOCKWISE in world
    (x right, y up) coordinates -- regardless of polygon's own vertex
    winding order (obstacle_polygon/2 facts make no winding
    guarantee). The outward direction for each edge is picked
    empirically (whichever of the two perpendiculars, probed a small
    epsilon out from the edge midpoint, lands OUTSIDE the polygon) --
    robust to either winding and to mild non-convexity, unlike relying
    on a fixed sign convention."""
    vertices = list(polygon)
    if _signed_polygon_area(vertices) > 0.0:
        # Positive signed area = vertices given counterclockwise (in a
        # y-up frame); reverse to walk them clockwise instead.
        vertices = list(reversed(vertices))

    samples = []
    for (ax, ay), (bx, by) in _polygon_edges(vertices):
        edx, edy = bx-ax, by-ay
        elen = math.hypot(edx, edy)
        if elen <= 1.0e-9:
            continue
        edx, edy = edx/elen, edy/elen
        n1 = (-edy, edx)
        n2 = (edy, -edx)
        mx, my = (ax+bx)/2.0, (ay+by)/2.0
        probe_x, probe_y = mx + n1[0]*_NORMAL_PROBE_EPS, my + n1[1]*_NORMAL_PROBE_EPS
        outward = n1 if not _inside_polygon(probe_x, probe_y, vertices) else n2
        for k in range(_BOUNDARY_SAMPLES_PER_EDGE):
            frac = k / _BOUNDARY_SAMPLES_PER_EDGE
            px, py = ax + edx*elen*frac, ay + edy*elen*frac
            samples.append((px + outward[0]*offset, py + outward[1]*offset))
    return samples


def _follow_boarder_control_points(sx, sy, obstacle_id, offset):
    """Core computation -- see follow_boarder_points below for the
    full contract. Looks up obstacle_id's own polygon, builds its
    offset boundary, and starts from whichever sample is closest to
    the robot's ACTUAL current position -- NOT assumed to already sit
    exactly on the offset curve (it generally won't, by a small
    bisection-tolerance residual -- see this project's own discussion
    of first_on_path_crossing_time's CROSSING_EPS). The small gap this
    leaves, if any, is absorbed into the planner's own first fitted
    spline segment, with no separate "join" step needed. Then walks a
    FULL clockwise loop back to that same starting sample -- no
    stopping condition of its own; see this section's own header for
    why. Returns [(x,y), ...] control points, or None if obstacle_id
    names no known obstacle or its polygon is degenerate."""
    polygon = None
    for oid, pts in _OBSTACLE_POLYGONS:
        if oid == obstacle_id:
            polygon = pts
            break
    if polygon is None or len(polygon) < 3:
        return None

    boundary = _offset_boundary_clockwise(polygon, offset)
    if not boundary:
        return None

    n = len(boundary)
    start_idx = min(range(n),
                     key=lambda i: (boundary[i][0]-sx)**2 + (boundary[i][1]-sy)**2)
    path_xy = [(sx, sy)] + [boundary[(start_idx+i) % n] for i in range(n+1)]

    try:
        tck, _u = fit_spline(path_xy, degree=3, smoothing=0.0)
        control_points, _k = bspline_to_bezier_chain(tck)
    except ValueError:
        # Degenerate/too-short path for a cubic fit -- fall back to a
        # straight line from the actual start to the loop's own last
        # point, same fallback _astar_control_points uses above.
        control_points = _straight_control_points(sx, sy, path_xy[-1][0], path_xy[-1][1])

    return control_points


# =====================================================================
# PLAN_VORONOI -- generalized-Voronoi-diagram planner, SAME interface
# as plan_astar/plan_straight (bare-atom Algorithm, (SX,SY,GX,GY) ->
# ControlPoints, no extra params) -- a THIRD instance of the exact
# "add one more plan_astar-style function plus one more pair of
# plan_call/8 clauses" recipe, needing NO new dispatch machinery in
# bt_to_prolog.py/bt_actions.py at all: "voronoi" is just one more
# valid value of the single consolidated PlanWith action's own
# algorithm port (see schema.yaml's own note on why every algorithm
# collapsed into one BT.cpp node), reusing the SAME "planWith"/
# plan_with_term branches astar/straight already use.
#
# Builds a roadmap from scipy's Voronoi diagram over points densely
# sampled along every obstacle's own boundary (reusing _OBSTACLE_
# POLYGONS, the SAME data follow_boarder above already loads) -- a
# standard technique: the Voronoi diagram of points sampled along
# obstacle boundaries approximates the true generalized Voronoi
# diagram (medial axis) of free space, since every Voronoi edge is by
# construction equidistant from its nearest sites, i.e. maximally
# clear of the obstacles that produced them. Ridge edges that cross
# INTO an obstacle (a real possibility with sparse/non-convex sites --
# a plain Voronoi tessellation has no notion of "obstacle interior")
# are filtered out, leaving only genuinely free-space roadmap edges.
# The CURRENT position and goal are then connected to the CLOSEST
# POINT ON the closest EDGE of that roadmap -- not the closest VERTEX,
# a real, deliberate distinction: an edge's closest point is generally
# partway along it, found by splitting that edge at the projection.
# Shortest path through the resulting graph is a plain Dijkstra (no
# networkx dependency -- a small hand-rolled search, same spirit as
# this file's own astar()).
# =====================================================================
_VORONOI_SAMPLES_PER_EDGE = 6
_VORONOI_EDGE_CHECK_SAMPLES = 6


def _voronoi_sites():
    """Dense points sampled along every obstacle polygon's own
    boundary -- the SITES scipy.spatial.Voronoi builds its diagram
    from. Same per-edge sampling idea as follow_boarder's own
    _offset_boundary_clockwise above, just without any outward
    offset."""
    sites = []
    for _oid, poly in _OBSTACLE_POLYGONS:
        for (ax, ay), (bx, by) in _polygon_edges(poly):
            for k in range(_VORONOI_SAMPLES_PER_EDGE):
                frac = k / _VORONOI_SAMPLES_PER_EDGE
                sites.append((ax + (bx-ax)*frac, ay + (by-ay)*frac))
    return sites


def _segment_crosses_any_obstacle(ax, ay, bx, by):
    """True iff the segment (ax,ay)-(bx,by), sampled at
    _VORONOI_EDGE_CHECK_SAMPLES points, passes through ANY obstacle's
    interior -- the filter that turns a plain Voronoi tessellation
    into a roadmap using only free-space edges."""
    for k in range(_VORONOI_EDGE_CHECK_SAMPLES + 1):
        frac = k / _VORONOI_EDGE_CHECK_SAMPLES
        x, y = ax + (bx-ax)*frac, ay + (by-ay)*frac
        for _oid, poly in _OBSTACLE_POLYGONS:
            if _inside_polygon(x, y, poly):
                return True
    return False


def _closest_point_on_segment(px, py, ax, ay, bx, by):
    sdx, sdy = bx-ax, by-ay
    len2 = sdx*sdx + sdy*sdy
    if len2 <= 1.0e-9:
        return ax, ay
    t = max(0.0, min(1.0, ((px-ax)*sdx + (py-ay)*sdy) / len2))
    return ax + t*sdx, ay + t*sdy


def _voronoi_roadmap_edges():
    """[(p1,p2), ...] -- every Voronoi ridge between two FINITE
    vertices whose connecting segment doesn't cross any obstacle.
    Unbounded ridges (scipy marks one endpoint -1) are dropped outright
    -- they only matter infinitely far from any obstacle, never a
    useful shortcut through a bounded map. Returns None if there are
    too few sites for scipy to build a diagram at all (e.g. no
    obstacles at all, or too few/degenerate ones)."""
    sites = _voronoi_sites()
    if len(sites) < 4:
        return None
    try:
        vor = Voronoi(sites)
    except Exception:
        return None

    edges = []
    for v1, v2 in vor.ridge_vertices:
        if v1 == -1 or v2 == -1:
            continue
        p1 = tuple(vor.vertices[v1])
        p2 = tuple(vor.vertices[v2])
        if _segment_crosses_any_obstacle(p1[0], p1[1], p2[0], p2[1]):
            continue
        edges.append((p1, p2))
    return edges


def _insert_point_into_roadmap(edges, px, py):
    """Finds the closest POINT on the closest EDGE (not vertex) of
    `edges` to (px,py), splits that edge there, and returns new_edges
    -- the original closest edge replaced by its two halves, plus a
    new edge connecting (px,py) to the projection point. Returns None
    if `edges` is empty."""
    if not edges:
        return None
    best = None
    for i, (p1, p2) in enumerate(edges):
        cx, cy = _closest_point_on_segment(px, py, p1[0], p1[1], p2[0], p2[1])
        d2 = (cx-px)**2 + (cy-py)**2
        if best is None or d2 < best[0]:
            best = (d2, i, cx, cy)
    _d2, idx, cx, cy = best
    p1, p2 = edges[idx]
    new_edges = edges[:idx] + edges[idx+1:]
    new_edges.append((p1, (cx, cy)))
    new_edges.append(((cx, cy), p2))
    new_edges.append(((px, py), (cx, cy)))
    return new_edges


def _dijkstra_shortest_path(edges, start, goal):
    """Plain Dijkstra over an undirected weighted graph given as a
    list of (p1,p2) edges (weight = Euclidean length) -- no networkx
    dependency, same "small hand-rolled search" spirit as this file's
    own astar() import above. Returns [start, ..., goal] or None if
    goal is unreachable from start."""
    adj = {}
    for p1, p2 in edges:
        w = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
        adj.setdefault(p1, []).append((p2, w))
        adj.setdefault(p2, []).append((p1, w))

    dist = {start: 0.0}
    prev = {}
    visited = set()
    heap = [(0.0, start)]
    while heap:
        d, u = heapq.heappop(heap)
        if u in visited:
            continue
        visited.add(u)
        if u == goal:
            break
        for v, w in adj.get(u, []):
            nd = d + w
            if v not in dist or nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))
    if goal not in dist:
        return None
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    path.reverse()
    return path


def _voronoi_control_points(sx, sy, gx, gy):
    """Core computation -- see plan_voronoi_points below for the full
    contract. Returns [(x,y), ...] control points, a straight line if
    there are no obstacles (or too few sites) to build a meaningful
    diagram from -- same "always succeeds for finite input" spirit as
    _straight_control_points, since there's nothing to route around --
    or None if the roadmap exists but start/goal are genuinely
    disconnected within it (a real, if unlikely, possibility for a
    disconnected free-space topology)."""
    start = (sx, sy)
    goal = (gx, gy)
    edges = _voronoi_roadmap_edges()
    if not edges:
        return _straight_control_points(sx, sy, gx, gy)

    edges = _insert_point_into_roadmap(edges, sx, sy)
    edges = _insert_point_into_roadmap(edges, gx, gy)

    path = _dijkstra_shortest_path(edges, start, goal)
    if path is None:
        return None  # roadmap built, but start/goal are disconnected within it

    if len(path) < 2:
        return [(sx, sy)] * 4
    try:
        tck, _u = fit_spline(path, degree=3, smoothing=0.0)
        control_points, _k = bspline_to_bezier_chain(tck)
    except ValueError:
        control_points = _straight_control_points(sx, sy, gx, gy)

    return control_points


# =====================================================================
# PLAIN-PYTHON API -- ALWAYS available, no ProbLog dependency. This is
# what bt_actions.py (and, eventually, a BT.cpp/pybind11 bridge) calls.
# =====================================================================
def plan_astar_points(sx, sy, gx, gy):
    """Plain-Python A* planner. Returns [(x,y), ...] control points
    (length 3k+1), or None if no path exists / the map isn't loaded."""
    return _astar_control_points(float(sx), float(sy), float(gx), float(gy))


def plan_straight_points(sx, sy, gx, gy):
    """Plain-Python straight-line planner. Always returns 4 control
    points for any finite (sx,sy),(gx,gy)."""
    return _straight_control_points(float(sx), float(sy), float(gx), float(gy))


def plan_voronoi_points(sx, sy, gx, gy):
    """Plain-Python generalized-Voronoi-diagram planner. Returns
    [(x,y), ...] control points routed along the free-space Voronoi
    roadmap built from the obstacle map, connecting (sx,sy) and
    (gx,gy) to the closest POINT on the closest EDGE of that roadmap
    (not just the closest vertex) -- see _voronoi_control_points'
    own header for the full geometry. Degrades to a straight line if
    there are no obstacles to route around; returns None only if a
    roadmap exists but start/goal are genuinely disconnected within
    it."""
    return _voronoi_control_points(float(sx), float(sy), float(gx), float(gy))


def follow_boarder_points(sx, sy, obstacle_id, offset):
    """Plain-Python boundary-following planner, shared by every
    Bug-algorithm variant. Returns [(x,y), ...] control points (length
    3k+1) tracing a FULL CLOCKWISE LOOP around `obstacle_id`'s own
    boundary, offset outward by `offset`, starting and ending at the
    CURRENT position's own nearest point on that offset curve --
    deliberately with NO stopping condition and NO goal parameter: this
    planner does not decide when to leave the boundary at all. That
    decision belongs entirely to whichever TRIGGER halts the
    SUBSEQUENT moveto_leg(CP,[...]) that walks this planner's own
    output -- line_of_sight_clear(ObstacleId,GX,GY) for Bug0,
    crosses_segment(SX,SY,GX,GY) for Bug2 (see collision_geometry.py's
    "BUG-ALGORITHM BOUNDARY-LEAVE PRIMITIVES" section) -- so the
    bug-variant choice is a matter of that Triggers list, not a
    different planner call. `offset` is typically unified with the
    SAME Threshold as whichever obstacle_on_path(Threshold)/
    obstacle_in_bound(Threshold) trigger or condition supplied
    `obstacle_id`, so the boundary-following path stays exactly as far
    out as the condition that triggered this replan -- see this
    project's own discussion on why a small residual gap between the
    robot's actual position and that nominal offset is harmless
    (absorbed into the first fitted spline segment, not a source of
    error that compounds).
    Returns None if obstacle_id names no known obstacle."""
    return _follow_boarder_control_points(
        float(sx), float(sy), str(obstacle_id), float(offset))


# =====================================================================
# PROBLOG-FACING PREDICATES -- only registered if ProbLog is actually
# importable. basic_action_theory.pl's :- use_module(...) directive always
# has ProbLog present (ProbLog itself is doing the loading), so this is
# never a problem for the ProbLog side; it only matters for a pure
# BT.cpp/Python caller that imports this module directly without ever
# installing ProbLog -- that caller only ever wants plan_astar_points/
# plan_straight_points above, and importing this module must not fail
# just because `problog` itself isn't installed in that environment.
# =====================================================================
try:
    from problog.extern import problog_export_nondet
    from problog.logic import Term, Constant
    _HAVE_PROBLOG = True
except ImportError:
    _HAVE_PROBLOG = False

if _HAVE_PROBLOG:

    def _control_points_to_terms(control_points):
        """[(x,y), ...] -> [point(x,y), ...] as ProbLog Term objects."""
        return [Term("point", Constant(float(x)), Constant(float(y)))
                for x, y in control_points]

    @problog_export_nondet("+float", "+float", "+float", "+float", "-list")
    def plan_astar(sx, sy, gx, gy):
        """Black-box A* planner (ProbLog predicate) -- see
        _astar_control_points above for the actual computation. Fails
        (returns []) if the map didn't load, if start/goal fall outside
        the map or on an obstacle cell, or if A* finds no path at all."""
        control_points = _astar_control_points(sx, sy, gx, gy)
        if control_points is None:
            return []  # no path found -- predicate FAILS, not an exception
        return [_control_points_to_terms(control_points)]

    @problog_export_nondet("+float", "+float", "+float", "+float", "-list")
    def plan_straight(sx, sy, gx, gy):
        """Black-box straight-line planner (ProbLog predicate) -- no map
        lookup at all, always succeeds for any finite (sx,sy),(gx,gy)."""
        control_points = _straight_control_points(sx, sy, gx, gy)
        return [_control_points_to_terms(control_points)]

    @problog_export_nondet("+float", "+float", "+float", "+float", "-list")
    def plan_voronoi(sx, sy, gx, gy):
        """Black-box generalized-Voronoi-diagram planner (ProbLog
        predicate) -- see _voronoi_control_points above for the actual
        computation. Fails (returns []) only if a roadmap exists but
        start/goal are genuinely disconnected within it; degrades to a
        straight line (never fails) when there are no obstacles to
        route around at all."""
        control_points = _voronoi_control_points(sx, sy, gx, gy)
        if control_points is None:
            return []
        return [_control_points_to_terms(control_points)]

    # ObstacleId arrives as a bare Prolog ATOM Term (e.g. obs5) -- "+term"
    # is the right spec (not "+str", which would reject an atom), and
    # .functor is the plain Python string for a zero-arity atom Term
    # (verified directly against problog.logic.Term -- see this
    # project's own testing convention of checking rather than
    # assuming). Offset is "+term" too, for the same bare-int-vs-float
    # robustness every other numeric arg in this theory already takes.
    @problog_export_nondet("+float", "+float", "+term", "+term", "-list")
    def follow_boarder(sx, sy, obstacle_id, offset):
        """Black-box boundary-following planner (ProbLog predicate),
        shared by every Bug-algorithm variant -- see
        _follow_boarder_control_points above for the actual
        computation. Fails (returns []) if obstacle_id names no known
        obstacle. Deliberately NO Goal parameter -- see that function's
        own header for why the stopping decision lives in a trigger,
        not here."""
        control_points = _follow_boarder_control_points(
            sx, sy, obstacle_id.functor, float(offset))
        if control_points is None:
            return []
        return [_control_points_to_terms(control_points)]
