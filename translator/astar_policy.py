"""
Precomputes, for a FIXED goal cell, a per-cell "which single step gets
closer to the goal, avoiding obstacles" policy over the translator's own
grid -- via one reverse Dijkstra flood from the goal.

WHY A FULL-GRID POLICY, NOT ONE A* CALL: the real system's PlanWith(astar)
plans once, from wherever the robot happens to be standing at the moment
that BT leaf is ticked. In THIS translator's per-tick model, verification
must consider every state nuXmv can possibly reach, so "wherever the robot
happens to be" ranges over every grid cell -- there is no single fixed
start to hand a normal point-to-point A* call. Flooding once from the
goal (symmetric edge costs, so a single reverse search suffices) yields
the same shortest-path routing A* would compute from ANY of those
starts, in one pass.

WHY THIS DOESN'T IMPORT planners.py DIRECTLY: planners.py unconditionally
imports scipy (scipy.interpolate/ndimage/spatial) at module load time, for
its own spline-fitting machinery -- which this translator's 1-cell-per-tick
model has no use for (there is no spline to fit; MoveTo just steps cell by
cell). Adding a scipy + Pillow dependency to BehaVerify's own install just
to reach astar()'s ~30-line grid search would be a heavy, one-sided cost.
Instead, the search itself (8-connected steps, diagonal costs, and the
"don't cut a diagonal through a blocked corner" rule) is ported line-for-line
from planners.py's own astar(), run against the SAME obstacle source
(obstacles_generated.pl's polygons, via geometry.clearance_to_obstacles)
this translator already rasterizes obstacle_clearance from -- not the raw
map.pgm pixel grid planners.py reads directly, which is a reasonable-fidelity
substitute since obstacles_generated.pl was itself derived from that same
pixel grid by occgrid_to_problog.py.

VORONOI, HONESTLY: this is NOT a port of planners.py's own
_voronoi_control_points (scipy.spatial.Voronoi over sampled obstacle-boundary
points, then Dijkstra over the resulting roadmap graph) -- that construction
is continuous-geometry and doesn't reduce to a per-cell policy the way A*
already does, and would reintroduce the scipy dependency this module exists
to avoid. Instead, `clearance_weight` adds a cost PENALTY inversely
proportional to a cell's own obstacle clearance to the SAME flood used for
astar, biasing the search away from tight passages and toward high-clearance
corridors -- a standard potential-field technique that is a genuine, if
approximate, discrete stand-in for "prefer the medial axis of free space"
(literally: the shortest path once cells near obstacles cost more to cross),
not a literal Voronoi diagram. See make_plan_voronoi's own note in
leaf_library.py.
"""
import heapq
import math

from . import geometry

# Same inflation radius planners.py's own PLANNING_INFLATE_M constant
# uses, so a cell counts as blocked under the same safety margin the real
# astar() would apply.
PLANNING_INFLATE_M = 0.5

# Tuning constant for the Voronoi-flavored flood (see module docstring's
# "VORONOI, HONESTLY" note) -- how strongly a cell's own obstacle clearance
# discourages routing through it. 0.0 (astar's own value) means no bias at
# all: a plain shortest path, indifferent to how close it hugs an obstacle
# beyond the bare inflation margin.
VORONOI_CLEARANCE_WEIGHT = 2.0

_STEPS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def _step_cost(dx, dy):
    return math.sqrt(2) if dx != 0 and dy != 0 else 1.0


def compute_policy(polygons_m, step, bounds, goal_cell, inflate_m=PLANNING_INFLATE_M, clearance_weight=0.0):
    """
    polygons_m: list of (id, [(x,y)_metres, ...]) -- obstacle_polygon data.
    step: config.disc_step_position (metres per cell).
    bounds: (min_x, max_x, min_y, max_y) grid bounds, in cells.
    goal_cell: (gx, gy) the fixed destination, in cells.
    clearance_weight: 0.0 for a plain shortest-path policy (astar); > 0 to
        additionally penalize routing near obstacles, biasing the search
        toward high-clearance corridors (voronoi -- see module docstring).

    Returns a dict {(x,y): (dx,dy)} -- for every cell FROM which the goal
    is reachable (including the goal cell itself, mapped to (0,0)), the
    single 8-connected step that starts a shortest (or clearance-weighted)
    obstacle-avoiding path to the goal. A cell absent from the dict means
    the goal is unreachable from there (blocked itself, or disconnected)
    -- callers should treat that as "stay put" (see leaf_library.py's own
    note on make_plan_astar/make_plan_voronoi).
    """
    min_x, max_x, min_y, max_y = bounds
    vertex_lists = [vertices for (_id, vertices) in polygons_m]

    def blocked(cx, cy):
        clearance_m = geometry.clearance_to_obstacles(cx * step, cy * step, vertex_lists)
        return clearance_m < inflate_m

    def clearance_penalty(cx, cy):
        if clearance_weight <= 0:
            return 0.0
        clearance_m = geometry.clearance_to_obstacles(cx * step, cy * step, vertex_lists)
        return clearance_weight / (clearance_m + 0.1)  # +0.1 avoids blowing up right at the inflation boundary

    dist = {goal_cell: 0.0}
    policy = {goal_cell: (0, 0)}
    visited = set()
    heap = [(0.0, goal_cell)]
    while heap:
        d, cur = heapq.heappop(heap)
        if cur in visited:
            continue
        visited.add(cur)
        cx, cy = cur
        for dx, dy in _STEPS:
            nx, ny = cx - dx, cy - dy  # a predecessor that steps (dx,dy) to reach cur
            if not (min_x <= nx <= max_x and min_y <= ny <= max_y):
                continue
            if blocked(nx, ny):
                continue
            if dx != 0 and dy != 0:
                # same corner-cutting guard as planners.py's astar(): don't allow
                # a diagonal step whose two flanking orthogonal cells are blocked.
                if blocked(nx + dx, ny) or blocked(nx, ny + dy):
                    continue
            neighbor = (nx, ny)
            nd = d + _step_cost(dx, dy) + clearance_penalty(nx, ny)
            if nd < dist.get(neighbor, float('inf')):
                dist[neighbor] = nd
                policy[neighbor] = (dx, dy)
                heapq.heappush(heap, (nd, neighbor))
    return policy
