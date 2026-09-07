#!/usr/bin/env python3
"""
collision_geometry.py

PROBLOG EXTERNAL-PREDICATE MODULE -- imported directly by ProbLog's own
:- use_module('./collision_geometry.py'). directive inside
basic_action_theory.pl, same mechanism as planners.py's own
:- use_module(...) directive (see that file's header for the mechanics).
Lives NEXT TO basic_action_theory.pl in module/theory/, not in
module/contracts/ -- this isn't a BT.cpp-facing node/condition, it's an
internal performance black box for the action theory's own
obstacle-clearance geometry.

WHY THIS EXISTS: basic_action_theory.pl used to compute
first_threshold_crossing_time/6 (the earliest time a noisy trajectory
comes within a given distance of any obstacle -- the basis for the
`collision` trigger and, now, the generic `obstacle_in_bound(Threshold)`
trigger too) entirely in Prolog: a
60-sample bracket scan, each sample checking clearance against every
obstacle polygon's every edge, followed by a bisection refinement. None
of that computation is actually PROBABILISTIC once Z (the resolved
lateral-noise draw) is fixed -- it's a deterministic function of
(ControlPoints, T0, Duration, Z, Threshold, the obstacle set). But
ProbLog's grounding materializes a full proof-tree node for every one of
those bracket/bisection steps anyway, for every resolved Z, which is
what made collision detection scale so badly with obstacle count/
complexity (see basic_action_theory.pl's own note above dist/5). Moving it
here collapses that whole grounding subtree into ONE black-box call per
resolved world -- exactly the same "stateless computation, no frame
problem, so no reason to pay Reiter's machinery's cost" argument already
used to justify planWith/plan_call being a Python black box instead of
Prolog clauses.

CORRECTNESS: the algorithm below is a LINE-FOR-LINE port of
basic_action_theory.pl's former Prolog implementation (same bracket-scan
sample count, same bisection epsilon, same spline/noise formulas) --
not a re-derivation. bracket_samples, crossing_eps, and position_sigma
are read directly out of the problem's own config.yaml at import time
(the SAME file module/translators/config_to_prolog.py turns into
config_generated.pl for the Prolog side, see that module's own header)
rather than hardcoded a second time here, so config.yaml stays the
single source of truth for both the Prolog and Python halves of the
theory with no risk of the two drifting apart. Which problem's
config.yaml is read is controlled by BT_PROBLEM_DIR, same as
_OBSTACLES_PATH below (see this module's own _PROBLEM_DIR note).

Exposes SEVEN predicates to ProbLog, all INSTANTANEOUS and stateless,
exactly like planners.py's plan_astar/plan_straight:

    first_threshold_crossing_time(+ControlPoints,+T0,+Duration,+Z,+Zt,
                                   +Threshold, -Tcross, -ObstacleId)
    obstacle_within_threshold(+X,+Y,+Threshold)
    first_on_path_crossing_time(+ControlPoints,+T0,+Duration,+Z,+Zt,
                                 +Threshold, -Tcross, -ObstacleId)
    obstacle_on_path_within_threshold(+ControlPoints,+T0,+Duration,+Z,+Zt,
                                       +X,+Y,+Threshold)
    first_line_of_sight_clear_time(+ControlPoints,+T0,+Duration,+Z,+Zt,
                                    +ObstacleId,+GX,+GY, -Tcross)
    line_of_sight_clear(+X,+Y,+ObstacleId,+GX,+GY)
    first_segment_crossing_time(+ControlPoints,+T0,+Duration,+Z,+Zt,
                                 +SX,+SY,+GX,+GY, -Tcross)

The last three back the Bug-algorithm boundary-LEAVE triggers/condition
(line_of_sight_clear = Bug0's rule, crosses_segment/first_segment_
crossing_time = Bug2's rule) -- see the "BUG-ALGORITHM BOUNDARY-LEAVE
PRIMITIVES" section below for why these are TRIGGER-side machinery,
not a new planner: the planner (planners.py's follow_boarder)
just walks a full clockwise loop around an obstacle's offset boundary
unconditionally now; WHICH bug variant a MoveTo leg implements is a
matter of which of these two triggers its own Triggers list names,
exactly the same "one action, swappable Triggers list" shape collision/
battery/obstacle_in_bound/etc. already have -- not two separate planner
functions with the stopping rule baked in (an earlier version of this
file's own history did that; see git log for why it changed).

Z and Zt are the TWO independent, already-resolved per-walk noise
draws _walk_noisy_point combines (normal/lateral and tangential/
along-path respectively -- see that function's own header for the
formula and why Duration itself stays independent of both). Every
trajectory-searching predicate here needs both; obstacle_within_threshold
and line_of_sight_clear are the exceptions -- they only ever check a
single ALREADY-COMPUTED point, so they have no noise draws of their
own to take.

first_threshold_crossing_time is the TRIGGER-side primitive: searches a
whole future trajectory (bracket scan + bisection) for the earliest
crossing. obstacle_within_threshold is the CONDITION-side primitive:
checks ONE point (the current situation) directly, no search at all --
it's what backs basic_action_theory.pl's holds(obstacle_in_bound(...),S),
the exact same underlying test (within_obstacle_threshold/
_min_clearance_all) the trigger-side search calls repeatedly, called
here just once. This is the "reuse the underlying machinery" the
obstacle_in_bound(Threshold) trigger/condition pair was built around --
see basic_action_theory.pl's own TRIGGERS section note.

first_on_path_crossing_time / obstacle_on_path_within_threshold are the
SAME trigger/condition pair again, for obstacle_on_path(Threshold):
"is Threshold-close to an obstacle the trajectory ACTUALLY ENTERS
somewhere along this walk" rather than any obstacle at all. Built by
restricting the SAME two functions above to a FILTERED obstacle set
(see _path_obstacle_polygons) -- no new geometry, just a different
input list.

ObstacleId is the SAME atom as the crossed obstacle's obstacle_polygon/2
Id (e.g. obs7) -- whichever polygon achieves the minimum clearance at
the exact (bisected) crossing point, i.e. an argmin over obstacles, not
just the min distance. This is what lets basic_action_theory.pl's
trigger_crossing_time/9 report crashed(ObstacleId)/
obstacle_in_bound(Threshold,ObstacleId) instead of a bare atom -- see
this module's own _min_clearance_all for where the argmin actually
happens.

first_threshold_crossing_time FAILS (returns 0 ProbLog solutions) if
the trajectory never comes within Threshold of an obstacle in this
resolved world; obstacle_within_threshold FAILS if the current point
isn't within Threshold right now -- "never/not happening" is
represented by absence in both, not a sentinel value, same convention
as every other exact-detection predicate in this theory.

Obstacle polygons: loaded ONCE, at import time, directly from
<problem>/obstacles_generated.pl (see _PROBLEM_DIR above) -- the EXACT
SAME file basic_action_theory.pl's own problem_data.pl bootstrap
consults (see Section 0's own comment, near the top of that file). By
the time this module is imported, that consult has already either
succeeded or already aborted the whole load with a clearer error, so
there is no meaningful "obstacles file missing" case to handle
gracefully here (unlike planners.py's map, which is genuinely
optional). If you hand-add extra obstacle_polygon/2 facts somewhere
else in the theory instead of through occgrid_to_problog.py's generated
file, this black box will not see them -- keep obstacles_generated.pl
the single source of truth, exactly as planners.py's own map
loading already assumes for map.yaml.

Everything below the constant/obstacle loading is PLAIN PYTHON, with no
ProbLog types anywhere -- first_threshold_crossing_time_value is the
testable core; the ProbLog import itself is wrapped in a try/except
(_HAVE_PROBLOG), same pattern as planners.py, purely so
this module can be exercised directly (e.g. to numerically compare
against the old Prolog implementation) without needing a full ProbLog
run.
"""
import math
import os
import re
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))

# Which problem's own data (config.yaml, obstacles_generated.pl) to load
# -- set by main.py via BT_PROBLEM_DIR before this module is imported;
# defaults to problems/problem0/ so this module still works standalone
# with no environment variable set (same convention as planners.py's
# own _PROBLEM_DIR).
_DEFAULT_PROBLEM_DIR = os.path.join(_PROJECT_ROOT, "problems", "problem0")
_PROBLEM_DIR = os.environ.get("BT_PROBLEM_DIR", _DEFAULT_PROBLEM_DIR)

_OBSTACLES_PATH = os.path.join(_PROBLEM_DIR, "obstacles_generated.pl")

_TRANSLATORS_DIR = os.path.join(_PROJECT_ROOT, "module", "translators")
if _TRANSLATORS_DIR not in sys.path:
    sys.path.insert(0, _TRANSLATORS_DIR)
from config_to_prolog import load_config  # noqa: E402

POINT_RE = re.compile(r"point\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)")


def _strip_prolog_comments(text):
    """Remove '%'-to-end-of-line comments before regex-parsing obstacle
    facts -- same defensive technique main.py
    already uses for the same reason (a header comment showing example
    syntax could otherwise match before the real fact)."""
    return re.sub(r"%.*$", "", text, flags=re.MULTILINE)


def _parse_obstacle_polygons(text):
    """Returns [(id, [(x,y),...]), ...] -- id is kept (not just the
    point list) so a crossing can be reported AGAINST a specific
    obstacle, not just "some obstacle" -- see _min_clearance_all."""
    polys = []
    for m in re.finditer(r"obstacle_polygon\(([^,]+),\s*\[(.*?)\]\s*\)\s*\.", text, re.S):
        obstacle_id = m.group(1).strip()
        pts = [(float(x), float(y)) for x, y in POINT_RE.findall(m.group(2))]
        if len(pts) >= 3:
            polys.append((obstacle_id, pts))
    return polys


_config = load_config(config_path=os.path.join(_PROBLEM_DIR, "config.yaml"))
BRACKET_SAMPLES = int(_config["verification"]["bracket_samples"])
CROSSING_EPS = float(_config["verification"]["crossing_eps"])
SIGMA = float(_config["position"]["lateral"]["sigma"])
SIGMA_TANGENTIAL = float(_config["position"]["tangential"]["sigma"])

try:
    with open(_OBSTACLES_PATH) as f:
        OBSTACLE_POLYGONS = _parse_obstacle_polygons(_strip_prolog_comments(f.read()))
except FileNotFoundError:
    # Only reachable if this module is imported standalone (e.g. for
    # testing) before any map has been generated -- basic_action_theory.pl
    # itself would already have aborted loading earlier at its own
    # :- consult(...) of this same file. "No obstacles" is the correct
    # degrade here, matching obstacle_polygon/2's own Prolog fallback
    # clause (see basic_action_theory.pl section 0).
    OBSTACLE_POLYGONS = []


# =====================================================================
# SPLINE + NOISE -- line-for-line port of basic_action_theory.pl's
# bezier_point/spline_point/spline_tangent/perp_unit/walk_noisy_point.
# =====================================================================
def _bezier_point(p0, p1, p2, p3, u):
    mu = 1.0 - u
    x = mu**3*p0[0] + 3*mu*mu*u*p1[0] + 3*mu*u*u*p2[0] + u**3*p3[0]
    y = mu**3*p0[1] + 3*mu*mu*u*p1[1] + 3*mu*u*u*p2[1] + u**3*p3[1]
    return x, y


def _bezier_tangent(p0, p1, p2, p3, u):
    mu = 1.0 - u
    dx = 3*mu*mu*(p1[0]-p0[0]) + 6*mu*u*(p2[0]-p1[0]) + 3*u*u*(p3[0]-p2[0])
    dy = 3*mu*mu*(p1[1]-p0[1]) + 6*mu*u*(p2[1]-p1[1]) + 3*u*u*(p3[1]-p2[1])
    return dx, dy


def _spline_segment(control_points, u):
    n_segs = (len(control_points) - 1) // 3
    seg_len = 1.0 / n_segs
    seg_idx = min(n_segs - 1, int(math.floor(u / seg_len)))
    local_u = (u - seg_idx*seg_len) / seg_len
    local_u = max(0.0, min(1.0, local_u))
    p0, p1, p2, p3 = control_points[3*seg_idx:3*seg_idx+4]
    return p0, p1, p2, p3, local_u


def _spline_point(control_points, u):
    p0, p1, p2, p3, local_u = _spline_segment(control_points, u)
    return _bezier_point(p0, p1, p2, p3, local_u)


def _spline_tangent(control_points, u):
    p0, p1, p2, p3, local_u = _spline_segment(control_points, u)
    return _bezier_tangent(p0, p1, p2, p3, local_u)


def _perp_unit(norm, dx, dy):
    if norm <= 1.0e-9:
        return 0.0, 0.0
    return -dy/norm, dx/norm


def _tangent_unit(norm, dx, dy):
    if norm <= 1.0e-9:
        return 0.0, 0.0
    return dx/norm, dy/norm


def _walk_noisy_point(control_points, t0, duration, z, zt, t):
    """Position along the spline at time T, given TWO already-resolved,
    INDEPENDENT per-walk noise draws: z (normal/lateral, unchanged from
    before) and zt (tangential/along-path, new). Both use the exact same
    Brownian-bridge growth law (deviation ~ noise*sigma*sqrt(Duration)*
    Frac), just applied along different unit directions -- z along the
    perpendicular (steering-error style), zt along the tangent itself
    (speed/timing-error style: further along or behind the nominal
    schedule, in a straight line at whatever direction the curve was
    pointing right there -- NOT a reparametrization of the curve, see
    basic_action_theory.pl's own note on why that option was rejected).
    Duration itself is UNAFFECTED by either noise source -- it stays a
    pure function of the nominal spline's own arc length (see
    walk_duration/2 in basic_action_theory.pl); letting either noise draw
    feed back into Duration would make the sqrt(Duration) scaling above
    circular (Duration would depend on a deviation that itself depends
    on sqrt(Duration))."""
    elapsed0 = t - t0
    elapsed = max(0.0, min(elapsed0, duration))
    frac = elapsed / duration
    nx, ny = _spline_point(control_points, frac)
    dx, dy = _spline_tangent(control_points, frac)
    norm = math.sqrt(dx*dx + dy*dy)
    perp_x, perp_y = _perp_unit(norm, dx, dy)
    tan_x, tan_y = _tangent_unit(norm, dx, dy)
    normal_dev = z * SIGMA * math.sqrt(duration) * frac
    tangent_dev = zt * SIGMA_TANGENTIAL * math.sqrt(duration) * frac
    return (nx + normal_dev*perp_x + tangent_dev*tan_x,
            ny + normal_dev*perp_y + tangent_dev*tan_y)


# =====================================================================
# OBSTACLE-CLEARANCE GEOMETRY -- line-for-line port of
# point_segment_dist/polygon_edges/dist_to_polygon/inside_polygon/
# signed_clearance/min_clearance_all/within_obstacle_threshold.
# =====================================================================
def _dist(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)


def _point_segment_dist(px, py, ax, ay, bx, by):
    sdx, sdy = bx-ax, by-ay
    len2 = sdx*sdx + sdy*sdy
    if len2 <= 1.0e-9:
        return _dist(px, py, ax, ay)
    t0 = ((px-ax)*sdx + (py-ay)*sdy) / len2
    t = max(0.0, min(1.0, t0))
    cx, cy = ax + t*sdx, ay + t*sdy
    return _dist(px, py, cx, cy)


def _polygon_edges(points):
    closed = list(points) + [points[0]]
    return list(zip(closed[:-1], closed[1:]))


def _dist_to_polygon(px, py, points):
    return min(_point_segment_dist(px, py, ax, ay, bx, by)
               for (ax, ay), (bx, by) in _polygon_edges(points))


def _edge_crosses(px, py, ax, ay, bx, by):
    if (ay > py and by <= py) or (by > py and ay <= py):
        x_cross = ax + (py-ay)/(by-ay)*(bx-ax)
        return px < x_cross
    return False


def _inside_polygon(px, py, points):
    count = sum(1 for (ax, ay), (bx, by) in _polygon_edges(points)
                if _edge_crosses(px, py, ax, ay, bx, by))
    return count % 2 == 1


def _signed_clearance(px, py, points):
    d_edge = _dist_to_polygon(px, py, points)
    return -d_edge if _inside_polygon(px, py, points) else d_edge


def _min_clearance_all(px, py, obstacle_polygons):
    """obstacle_polygons: [(id, points), ...]. Returns (min_distance,
    nearest_obstacle_id) -- an ARGMIN over obstacles, not just the min
    distance, so callers can report WHICH obstacle a crossing is
    against, not just that one exists. nearest_obstacle_id is None only
    when there are no obstacles at all (min_distance is then the
    original "so far away it never matters" sentinel, unreachable by
    any real threshold)."""
    if not obstacle_polygons:
        return 1000000.0, None
    return min((_signed_clearance(px, py, poly), obstacle_id)
               for obstacle_id, poly in obstacle_polygons)


def _within_obstacle_threshold(px, py, threshold, obstacle_polygons):
    d, _ = _min_clearance_all(px, py, obstacle_polygons)
    return d <= threshold


# =====================================================================
# BRACKET SCAN + BISECTION -- line-for-line port of
# first_unsafe_sample(_from)/bisect_crossing/first_threshold_crossing_time.
# =====================================================================
def _first_unsafe_sample(control_points, t0, duration, z, zt, threshold, obstacle_polygons, n):
    for i in range(0, n + 1):
        frac = i / n
        t = t0 + duration*frac
        x, y = _walk_noisy_point(control_points, t0, duration, z, zt, t)
        if _within_obstacle_threshold(x, y, threshold, obstacle_polygons):
            return i
    return None


def _bisect_crossing(control_points, t0, duration, z, zt, threshold, tlo, thi, eps, obstacle_polygons):
    while thi - tlo > eps:
        tmid = (tlo + thi) / 2.0
        x, y = _walk_noisy_point(control_points, t0, duration, z, zt, tmid)
        if _within_obstacle_threshold(x, y, threshold, obstacle_polygons):
            thi = tmid
        else:
            tlo = tmid
    return (tlo + thi) / 2.0


def _first_threshold_crossing_time(control_points, t0, duration, z, zt, threshold, obstacle_polygons):
    """Returns (Tcross, ObstacleId) or None. ObstacleId is resolved by
    ONE extra _min_clearance_all argmin call at the final crossing
    point -- the bracket scan/bisection loop itself only needs the
    boolean within/not-within-threshold test, so this stays a single
    added lookup, not a change to the search itself."""
    n = BRACKET_SAMPLES
    i = _first_unsafe_sample(control_points, t0, duration, z, zt, threshold, obstacle_polygons, n)
    if i is None:
        return None
    if i == 0:
        tcross = t0
    else:
        tlo = t0 + duration*((i-1)/n)
        thi = t0 + duration*(i/n)
        tcross = _bisect_crossing(control_points, t0, duration, z, zt, threshold, tlo, thi,
                                   CROSSING_EPS, obstacle_polygons)
    x, y = _walk_noisy_point(control_points, t0, duration, z, zt, tcross)
    _, obstacle_id = _min_clearance_all(x, y, obstacle_polygons)
    return tcross, obstacle_id


# =====================================================================
# "ON PATH" -- obstacle_on_path(Threshold): unlike obstacle_in_bound
# (proximity to ANY obstacle, whether or not the trajectory actually
# goes near it), this only cares about obstacles the trajectory itself
# actually enters (_inside_polygon, not just close to the boundary) at
# SOME point across the walk's own full span -- then asks the SAME
# "within Threshold right now" question, restricted to just that
# obstacle set. Reuses _within_obstacle_threshold/_first_threshold_
# crossing_time UNCHANGED: the only new piece is the bracket-scan that
# finds WHICH obstacles are on the path at all; everything downstream
# is the existing machinery called with a FILTERED obstacle_polygons
# list instead of the full one.
# =====================================================================
def _trajectory_obstacle_ids(control_points, t0, duration, z, zt, obstacle_polygons):
    """Which obstacle ids does this resolved trajectory's noisy position
    actually go INSIDE (not just near) at any bracket-sampled instant
    across [t0, t0+duration]? Same sample count as the rest of this
    module's bracket scans (BRACKET_SAMPLES) -- same discretization,
    same aliasing risk as every other bracket-scan check here, not a
    new limitation. Returns a (possibly empty) set of ids."""
    n = BRACKET_SAMPLES
    hit_ids = set()
    for i in range(n + 1):
        frac = i / n
        t = t0 + duration*frac
        x, y = _walk_noisy_point(control_points, t0, duration, z, zt, t)
        for obstacle_id, poly in obstacle_polygons:
            if _inside_polygon(x, y, poly):
                hit_ids.add(obstacle_id)
    return hit_ids


def _path_obstacle_polygons(control_points, t0, duration, z, zt, obstacle_polygons):
    """obstacle_polygons FILTERED down to just the ones the trajectory
    actually enters -- the single restriction point _first_threshold_
    crossing_time/_within_obstacle_threshold are then called with,
    unchanged, everywhere below."""
    path_ids = _trajectory_obstacle_ids(control_points, t0, duration, z, zt, obstacle_polygons)
    return [(oid, poly) for oid, poly in obstacle_polygons if oid in path_ids]


def _first_on_path_crossing_time(control_points, t0, duration, z, zt, threshold, obstacle_polygons):
    """Same shape/return as _first_threshold_crossing_time (Tcross,
    ObstacleId) or None -- None immediately if the trajectory never
    enters any obstacle at all (nothing to restrict to), otherwise
    delegates straight to the existing search over the restricted set."""
    restricted = _path_obstacle_polygons(control_points, t0, duration, z, zt, obstacle_polygons)
    if not restricted:
        return None
    return _first_threshold_crossing_time(control_points, t0, duration, z, zt, threshold, restricted)


def _obstacle_on_path_within_threshold(control_points, t0, duration, z, zt, px, py, threshold, obstacle_polygons):
    """The CONDITION-side counterpart: is (px,py) within threshold of
    an obstacle the trajectory actually enters, RIGHT NOW -- one call,
    no search, exactly mirroring _within_obstacle_threshold's own
    relationship to _first_threshold_crossing_time."""
    restricted = _path_obstacle_polygons(control_points, t0, duration, z, zt, obstacle_polygons)
    if not restricted:
        return False
    return _within_obstacle_threshold(px, py, threshold, restricted)


# =====================================================================
# BUG-ALGORITHM BOUNDARY-LEAVE PRIMITIVES -- the two rules that decide
# WHEN a boundary-following MoveTo leg (planners.py's
# follow_boarder(ObstacleId,Offset) planner -- a full clockwise loop
# around the obstacle's offset boundary, no stopping logic of its own)
# should stop circling and hand back off to a straight-line planner.
# Deliberately TRIGGER-side machinery (like first_threshold_crossing_
# time above), not baked into the planner: this is what makes Bug0 vs
# Bug2 a matter of WHICH trigger a leg's own Triggers list names, not
# two different planner functions -- exactly the same shape collision/
# obstacle_in_bound/battery_below already have (one action, swappable
# Triggers).
#
#   line_of_sight_clear(ObstacleId,GX,GY) -- Bug0's rule: fires the
#       instant ObstacleId stops occluding a straight line from the
#       CURRENT (noisy) position to (GX,GY). Also a genuine CONDITION
#       (line_of_sight_clear/5 below) -- a clean point-in-time question
#       ("is it occluded RIGHT NOW"), same shape as obstacle_in_bound.
#
#   crosses_segment(SX,SY,GX,GY) -- Bug2's rule: fires the first time
#       the walked trajectory RE-CROSSES the straight segment from
#       (SX,SY) (wherever this leg's own circling began) to (GX,GY),
#       AT a point CLOSER to (GX,GY) than (SX,SY) itself was -- the
#       "makes actual progress" condition genuine Bug2 requires (a
#       crossing that doesn't get closer to goal isn't a valid leave
#       point; the search keeps going past it, exactly like a robot
#       that circles back over its own outbound track without yet
#       having rounded the obstacle). Deliberately TRIGGER-ONLY, no
#       holds(crosses_segment(...),S) condition: "has my trajectory
#       CROSSED this segment" is fundamentally an event over an
#       interval of motion, not a fact true at a single instant the
#       way obstacle_in_bound/line_of_sight_clear are -- forcing a
#       point-in-time reading (e.g. "is the CURRENT position on the
#       segment") would be a near-measure-zero, not-generally-useful
#       question, unlike its trigger form.
# =====================================================================
def _orient(ax, ay, bx, by, cx, cy):
    return (bx-ax)*(cy-ay) - (by-ay)*(cx-ax)


def _segments_intersect(p1, p2, p3, p4):
    """Standard orientation-based proper-crossing test -- same as
    planners.py's own copy (kept separately per this module's
    own no-cross-import-between-black-boxes rule; see that file's
    header note by its own _OBSTACLE_POLYGONS load for why). Collinear
    /touching edge cases are treated as NOT intersecting."""
    ax, ay = p1
    bx, by = p2
    cx, cy = p3
    dx, dy = p4
    o1 = _orient(ax, ay, bx, by, cx, cy)
    o2 = _orient(ax, ay, bx, by, dx, dy)
    o3 = _orient(cx, cy, dx, dy, ax, ay)
    o4 = _orient(cx, cy, dx, dy, bx, by)
    return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)


def _find_polygon(obstacle_id, obstacle_polygons):
    for oid, poly in obstacle_polygons:
        if oid == obstacle_id:
            return poly
    return None


def _segment_crosses_polygon(px, py, gx, gy, polygon):
    """True iff the straight segment from (px,py) to (gx,gy) crosses
    `polygon`'s own boundary -- i.e. the obstacle still occludes a
    direct line of sight to (gx,gy) from here."""
    return any(_segments_intersect((px, py), (gx, gy), a, b)
               for a, b in _polygon_edges(polygon))


def _first_clear_sample(control_points, t0, duration, z, zt, polygon, gx, gy, n):
    for i in range(0, n + 1):
        frac = i / n
        t = t0 + duration*frac
        x, y = _walk_noisy_point(control_points, t0, duration, z, zt, t)
        if not _segment_crosses_polygon(x, y, gx, gy, polygon):
            return i
    return None


def _bisect_los_crossing(control_points, t0, duration, z, zt, polygon, gx, gy, tlo, thi, eps):
    while thi - tlo > eps:
        tmid = (tlo + thi) / 2.0
        x, y = _walk_noisy_point(control_points, t0, duration, z, zt, tmid)
        if not _segment_crosses_polygon(x, y, gx, gy, polygon):
            thi = tmid
        else:
            tlo = tmid
    return (tlo + thi) / 2.0


def _first_line_of_sight_clear_time(control_points, t0, duration, z, zt, obstacle_id, gx, gy, obstacle_polygons):
    """Returns Tcross, or None if line of sight to (gx,gy) is never
    clear of obstacle_id anywhere in this walk. obstacle_id naming no
    known obstacle degrades to "nothing to occlude" -- clear from t0
    itself -- same lenient-unknown-name spirit as an unrecognized
    Trigger name elsewhere in this theory, not a hard error."""
    polygon = _find_polygon(obstacle_id, obstacle_polygons)
    if polygon is None:
        return t0
    n = BRACKET_SAMPLES
    i = _first_clear_sample(control_points, t0, duration, z, zt, polygon, gx, gy, n)
    if i is None:
        return None
    if i == 0:
        return t0
    tlo = t0 + duration*((i-1)/n)
    thi = t0 + duration*(i/n)
    return _bisect_los_crossing(control_points, t0, duration, z, zt, polygon, gx, gy, tlo, thi, CROSSING_EPS)


def _bisect_segment_crossing(control_points, t0, duration, z, zt, sx, sy, gx, gy, tlo, thi, eps):
    """Bisects on the SIGN of _orient(sx,sy,gx,gy, position(T)) -- the
    side of the (sx,sy)-(gx,gy) LINE the walked position sits on --
    rather than a within/not-within-threshold boolean, since crossing
    a line (not a bounded region) is what's being detected here."""
    def side(t):
        x, y = _walk_noisy_point(control_points, t0, duration, z, zt, t)
        return _orient(sx, sy, gx, gy, x, y) > 0
    side_lo = side(tlo)
    while thi - tlo > eps:
        tmid = (tlo + thi) / 2.0
        if side(tmid) == side_lo:
            tlo = tmid
        else:
            thi = tmid
    return (tlo + thi) / 2.0


def _first_segment_crossing_time(control_points, t0, duration, z, zt, sx, sy, gx, gy):
    """Returns Tcross, or None if the walked trajectory never
    re-crosses segment (sx,sy)-(gx,gy) at a point CLOSER to (gx,gy)
    than (sx,sy) itself was -- see this section's own header for why
    that distance condition is part of the definition, not an add-on.
    Walks bracket-sample-to-bracket-sample EDGES of the trajectory
    (not single points -- a LINE crossing is an event between two
    positions, unlike a threshold crossing which is a property of one
    position at a time), bisecting each candidate edge for precision
    and skipping any crossing that fails the distance test to keep
    searching for a later one."""
    n = BRACKET_SAMPLES
    dep_dist = _dist(sx, sy, gx, gy)
    prev_t = t0
    prev_x, prev_y = _walk_noisy_point(control_points, t0, duration, z, zt, t0)
    for i in range(1, n + 1):
        frac = i / n
        t = t0 + duration*frac
        x, y = _walk_noisy_point(control_points, t0, duration, z, zt, t)
        if _segments_intersect((sx, sy), (gx, gy), (prev_x, prev_y), (x, y)):
            tcross = _bisect_segment_crossing(control_points, t0, duration, z, zt,
                                               sx, sy, gx, gy, prev_t, t, CROSSING_EPS)
            cx, cy = _walk_noisy_point(control_points, t0, duration, z, zt, tcross)
            if _dist(cx, cy, gx, gy) < dep_dist:
                return tcross
            # crossed the line, but no closer to goal than the
            # departure point -- not a valid Bug2 leave point, keep
            # scanning forward for a later crossing that qualifies.
        prev_t, prev_x, prev_y = t, x, y
    return None


# =====================================================================
# PLAIN-PYTHON API -- ALWAYS available, testable without ProbLog.
# =====================================================================
def first_threshold_crossing_time_value(control_points, t0, duration, z, zt, threshold):
    """control_points: [(x,y), ...]. Returns (Tcross, ObstacleId), or
    None if the trajectory never comes within `threshold` of any
    obstacle in this resolved world. ObstacleId is the obstacle_polygon/2
    Id (e.g. "obs7") nearest at the exact crossing point."""
    return _first_threshold_crossing_time(
        control_points, float(t0), float(duration), float(z), float(zt), float(threshold),
        OBSTACLE_POLYGONS)


def obstacle_within_threshold_value(x, y, threshold):
    """A SINGLE-POINT check -- is (x,y) within `threshold` of any
    obstacle right now -- reusing _within_obstacle_threshold directly
    (the SAME primitive _first_unsafe_sample/_bisect_crossing call
    repeatedly above; this just calls it once). This is what backs the
    obstacle_in_bound(Threshold) CONDITION in basic_action_theory.pl
    (holds(obstacle_in_bound(...),S)), as opposed to the
    obstacle_in_bound(Threshold) TRIGGER, which searches a whole future
    trajectory via first_threshold_crossing_time_value above. Takes no
    noise draws at all -- it's a single-point check against a position
    already computed (by at/4, which already folds in both z and zt),
    not a trajectory search."""
    return _within_obstacle_threshold(float(x), float(y), float(threshold), OBSTACLE_POLYGONS)


def first_on_path_crossing_time_value(control_points, t0, duration, z, zt, threshold):
    """control_points: [(x,y), ...]. Returns (Tcross, ObstacleId), or
    None if the trajectory never comes within `threshold` of an
    obstacle IT ACTUALLY ENTERS somewhere along this walk (as opposed
    to first_threshold_crossing_time_value, which considers every
    obstacle regardless of whether the path goes near it at all)."""
    return _first_on_path_crossing_time(
        control_points, float(t0), float(duration), float(z), float(zt), float(threshold),
        OBSTACLE_POLYGONS)


def obstacle_on_path_within_threshold_value(control_points, t0, duration, z, zt, x, y, threshold):
    """A SINGLE-POINT check, same relationship to
    first_on_path_crossing_time_value as obstacle_within_threshold_value
    has to first_threshold_crossing_time_value: is (x,y) within
    threshold of an obstacle the trajectory actually enters, right
    now."""
    return _obstacle_on_path_within_threshold(
        control_points, float(t0), float(duration), float(z), float(zt),
        float(x), float(y), float(threshold), OBSTACLE_POLYGONS)


def first_line_of_sight_clear_time_value(control_points, t0, duration, z, zt, obstacle_id, gx, gy):
    """control_points: [(x,y), ...]. Returns Tcross, or None if line of
    sight from the trajectory to (gx,gy) is never clear of obstacle_id
    anywhere in this walk."""
    return _first_line_of_sight_clear_time(
        control_points, float(t0), float(duration), float(z), float(zt),
        str(obstacle_id), float(gx), float(gy), OBSTACLE_POLYGONS)


def line_of_sight_clear_value(x, y, obstacle_id, gx, gy):
    """A SINGLE-POINT check -- is (x,y) NOT occluded from (gx,gy) by
    obstacle_id right now -- backs holds(line_of_sight_clear(...),S),
    same relationship to first_line_of_sight_clear_time_value as
    obstacle_within_threshold_value has to first_threshold_crossing_
    time_value."""
    polygon = _find_polygon(str(obstacle_id), OBSTACLE_POLYGONS)
    if polygon is None:
        return True
    return not _segment_crosses_polygon(float(x), float(y), float(gx), float(gy), polygon)


def first_segment_crossing_time_value(control_points, t0, duration, z, zt, sx, sy, gx, gy):
    """control_points: [(x,y), ...]. Returns Tcross, or None if the
    trajectory never re-crosses segment (sx,sy)-(gx,gy) at a point
    closer to (gx,gy) than (sx,sy) itself was."""
    return _first_segment_crossing_time(
        control_points, float(t0), float(duration), float(z), float(zt),
        float(sx), float(sy), float(gx), float(gy))


# =====================================================================
# PROBLOG-FACING PREDICATE -- only registered if ProbLog is importable
# (see planners.py's header for why this is guarded
# rather than a hard import).
# =====================================================================
try:
    from problog.extern import problog_export_nondet
    _HAVE_PROBLOG = True
except ImportError:
    _HAVE_PROBLOG = False

if _HAVE_PROBLOG:

    # T0 in particular can arrive as a bare Prolog INTEGER (now(s0,0) is
    # the very first walk's T0) rather than a float -- "+term" accepts
    # either representation; first_threshold_crossing_time_value below
    # does the float() conversion itself. Duration/Z/Zt/Threshold are
    # always the result of `is` arithmetic or float literals in
    # practice, but are taken as "+term" too for the same robustness.
    # walk_noisy_point(+ControlPoints,+T0,+Duration,+Z,+Zt,+T,-X,-Y):
    # position along the spline at time T, given two already-resolved
    # noise draws Z (lateral) and Zt (tangential) -- see this module's
    # own _walk_noisy_point for the formula. PURE deterministic
    # arithmetic once Z/Zt are resolved, with no probabilistic content
    # of its own, same rationale as first_threshold_crossing_time above
    # for why this lives here rather than as native Prolog: previously
    # basic_action_theory.pl carried its OWN, separate Bezier/spline/
    # perp-tangent-unit Prolog reimplementation of this exact formula
    # (bezier_point/tangent, spline_point/tangent, perp_unit,
    # tangent_unit), kept "identical" to this file only by discipline,
    # not by construction -- a real duplication/drift risk. Now
    # basic_action_theory.pl's own walk_noisy_point/8 IS this predicate
    # (no Prolog clause of that name exists there any more; ProbLog
    # resolves the call directly against this module, exactly like
    # first_threshold_crossing_time/8 already does), and spline_point/4
    # (the deterministic/nominal-path case) is a two-line Prolog
    # wrapper calling this SAME predicate with Z=0.0, Zt=0.0 -- there is
    # only one copy of the spline arithmetic in the whole theory now.
    # Always succeeds (pure function, no crossing search, no "no
    # result" case), so always returns exactly one solution.
    @problog_export_nondet("+list", "+term", "+term", "+term", "+term", "+term", "-float", "-float")
    def walk_noisy_point(control_points, t0, duration, z, zt, t):
        cp = [(float(p.args[0]), float(p.args[1])) for p in control_points]
        x, y = _walk_noisy_point(cp, float(t0), float(duration), float(z), float(zt), float(t))
        return [(x, y)]

    @problog_export_nondet("+list", "+term", "+term", "+term", "+term", "+term", "-float", "-str")
    def first_threshold_crossing_time(control_points, t0, duration, z, zt, threshold):
        cp = [(float(p.args[0]), float(p.args[1])) for p in control_points]
        result = first_threshold_crossing_time_value(
            cp, float(t0), float(duration), float(z), float(zt), float(threshold))
        if result is None:
            return []  # no crossing in this world -- predicate FAILS
        tcross, obstacle_id = result
        return [(tcross, obstacle_id)]

    # obstacle_within_threshold(+X,+Y,+Threshold): a pure boolean check,
    # no output ports at all -- backs holds(obstacle_in_bound(...),S) in
    # basic_action_theory.pl. A ProbLog nondet predicate with ZERO "-"
    # specs succeeds (one solution) by returning [()] (a single empty
    # tuple -- "here is one solution binding zero extra outputs") or
    # fails (no solutions) by returning [].
    @problog_export_nondet("+term", "+term", "+term")
    def obstacle_within_threshold(x, y, threshold):
        if obstacle_within_threshold_value(x, y, threshold):
            return [()]
        return []

    # first_on_path_crossing_time(+ControlPoints,+T0,+Duration,+Z,+Zt,
    # +Threshold,-Tcross,-ObstacleId): backs the obstacle_on_path(Threshold)
    # TRIGGER. Same shape/signature as first_threshold_crossing_time
    # above, just restricted (inside collision_geometry.py itself) to
    # obstacles the trajectory actually enters.
    @problog_export_nondet("+list", "+term", "+term", "+term", "+term", "+term", "-float", "-str")
    def first_on_path_crossing_time(control_points, t0, duration, z, zt, threshold):
        cp = [(float(p.args[0]), float(p.args[1])) for p in control_points]
        result = first_on_path_crossing_time_value(
            cp, float(t0), float(duration), float(z), float(zt), float(threshold))
        if result is None:
            return []
        tcross, obstacle_id = result
        return [(tcross, obstacle_id)]

    # obstacle_on_path_within_threshold(+ControlPoints,+T0,+Duration,+Z,
    # +Zt,+X,+Y,+Threshold): backs the obstacle_on_path(Threshold)
    # CONDITION. Same zero-output boolean shape as
    # obstacle_within_threshold above.
    @problog_export_nondet("+list", "+term", "+term", "+term", "+term", "+term", "+term", "+term")
    def obstacle_on_path_within_threshold(control_points, t0, duration, z, zt, x, y, threshold):
        cp = [(float(p.args[0]), float(p.args[1])) for p in control_points]
        if obstacle_on_path_within_threshold_value(cp, t0, duration, z, zt, x, y, threshold):
            return [()]
        return []

    # ObstacleId arrives as a bare Prolog ATOM Term (e.g. obs5) --
    # .functor is the plain Python string for a zero-arity atom Term
    # (same pattern planners.py's follow_boarder already uses).
    #
    # first_line_of_sight_clear_time(+ControlPoints,+T0,+Duration,+Z,
    # +Zt,+ObstacleId,+GX,+GY,-Tcross): backs the line_of_sight_clear
    # (ObstacleId,GX,GY) TRIGGER (Bug0's leave rule).
    # NOTE: for a SINGLE "-" output, problog's own problog_export_nondet
    # wrapper (extern.py) treats each element of the returned list as
    # the BARE output value itself, not a 1-tuple -- unlike the
    # zero-output ([()]) and multi-output ([(a,b)]) cases above/below.
    # Verified directly (a bare-tuple return silently bound Tcross to
    # the tuple object itself instead of the float) before shipping
    # this, same testing discipline as everywhere else in this project.
    @problog_export_nondet("+list", "+term", "+term", "+term", "+term", "+term", "+term", "+term", "-float")
    def first_line_of_sight_clear_time(control_points, t0, duration, z, zt, obstacle_id, gx, gy):
        cp = [(float(p.args[0]), float(p.args[1])) for p in control_points]
        tcross = first_line_of_sight_clear_time_value(
            cp, t0, duration, z, zt, obstacle_id.functor, gx, gy)
        if tcross is None:
            return []
        return [tcross]

    # line_of_sight_clear(+X,+Y,+ObstacleId,+GX,+GY): backs the
    # line_of_sight_clear(ObstacleId,GX,GY) CONDITION. Same zero-output
    # boolean shape as obstacle_within_threshold above.
    @problog_export_nondet("+term", "+term", "+term", "+term", "+term")
    def line_of_sight_clear(x, y, obstacle_id, gx, gy):
        if line_of_sight_clear_value(x, y, obstacle_id.functor, gx, gy):
            return [()]
        return []

    # first_segment_crossing_time(+ControlPoints,+T0,+Duration,+Z,+Zt,
    # +SX,+SY,+GX,+GY,-Tcross): backs the crosses_segment(SX,SY,GX,GY)
    # TRIGGER (Bug2's leave rule) -- TRIGGER-ONLY, deliberately no
    # condition counterpart, see this module's own "BUG-ALGORITHM
    # BOUNDARY-LEAVE PRIMITIVES" section header for why.
    # Same single-output bare-value convention as
    # first_line_of_sight_clear_time above -- see its own note.
    @problog_export_nondet("+list", "+term", "+term", "+term", "+term", "+term", "+term", "+term", "+term", "-float")
    def first_segment_crossing_time(control_points, t0, duration, z, zt, sx, sy, gx, gy):
        cp = [(float(p.args[0]), float(p.args[1])) for p in control_points]
        tcross = first_segment_crossing_time_value(cp, t0, duration, z, zt, sx, sy, gx, gy)
        if tcross is None:
            return []
        return [tcross]
