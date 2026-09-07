#!/usr/bin/env python3
"""
bt_actions.py

Lives in module/contracts/ alongside schema.yaml -- vocabulary.yaml and
goal_formula_check.py are its siblings there too (see this project's
top-level layout note for why these are grouped as "the schemas the
translators/validators check against").

Canonical action/condition implementations matching schema.yaml,
written to be usable from TWO different callers:

  1. Our own ProbLog-based verification pipeline. planners.py's (in
     module/theory/) plan_astar_points/plan_straight_points ARE the
     shared plain-Python planning core, imported and REUSED here
     unchanged, never duplicated -- basic_action_theory.pl itself
     keeps calling the SEPARATE ProbLog-facing plan_astar/plan_straight
     predicates (also defined in planners.py, on top of the same core)
     directly, unaffected by anything in this file.

  2. A future BehaviorTree.cpp integration. A pybind11 (or ctypes, or
     ROS2 behaviortree_ros2) bridge could register the bt_-prefixed
     functions below directly as C++ node tick() callbacks: their
     signatures and return shapes match schema.yaml's port
     declarations exactly, using PLAIN Python types throughout
     (float / list of (x,y) tuples / str / bool / dict) -- never a
     ProbLog Term object. This file (and the plain-Python half of
     planners.py it calls into) has NO ProbLog import anywhere, so a
     BT.cpp bridge that never installs ProbLog can still import and
     call bt_plan_astar/bt_plan_straight -- see planners.py's own
     header for why its ProbLog-specific half is wrapped in a
     try/except instead of a hard import.

MoveTo (and both conditions) are DELIBERATELY NOT given a directly
-executable Python implementation here. MoveTo's real behaviour is
the STOCHASTIC action theory in basic_action_theory.pl -- noisy
position, noisy battery, exact trigger-crossing detection via
closed-form algebra or bracket-scan+bisection. There is no correct
way to "run" that in a plain Python function without reimplementing
the entire probabilistic model outside ProbLog, and a naive
deterministic stand-in would silently misrepresent what the theory
actually says happens -- worse than no implementation at all.
DistanceBelow/DistanceEqual/DistanceOver/HaltedWith are native Prolog
conditions over a situation; Python has no situation to evaluate them
against on its own.

What IS provided for all three is their INTERFACE (matching
schema.yaml's ports exactly) plus a TERM BUILDER -- a function
translating bound port values into the corresponding
basic_action_theory.pl term text. This is the piece a future
BT-tree-to-Prolog translator needs: given a BT.cpp node's bound
inputs, produce the Prolog subterm to splice into a
seq_node(...)/fallback_node(...) list. Building that translator
itself (parsing a whole BT.cpp XML tree) is a separate, larger step
-- not done here (see module/translators/bt_to_prolog.py, which
implements this translator directly rather than through this file);
this file only provides the per-node building blocks it documents.
"""
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_THEORY_DIR = os.path.join(os.path.dirname(_THIS_DIR), "theory")
if _THEORY_DIR not in sys.path:
    sys.path.insert(0, _THEORY_DIR)

from planners import (
    plan_astar_points, plan_straight_points, plan_voronoi_points, follow_boarder_points,
)


# =====================================================================
# ACTIONS -- callable implementations, one per PlanWith algorithm
# =====================================================================
def bt_plan_astar(sx, sy, gx, gy):
    """
    BT.cpp-compatible wrapper around planners.py's
    plan_astar_points -- matches PlanWith's three output ports in
    schema.yaml exactly (algorithm="astar" case), returned together as
    one dict:
        {control_points, reason, status}
    control_points is [] and reason is "no_path" if A* found no path
    (unreachable goal, or the map failed to load) -- see
    planners.py's own _astar_control_points for exactly which
    cases that covers.
    """
    control_points = plan_astar_points(sx, sy, gx, gy)
    if control_points is None:
        return {"control_points": [], "reason": "no_path", "status": False}
    return {
        "control_points": [(float(x), float(y)) for x, y in control_points],
        "reason": "completed",
        "status": True,
    }


def bt_plan_straight(sx, sy, gx, gy):
    """BT.cpp-compatible wrapper around plan_straight_points -- same
    shape and rationale as bt_plan_astar above; a straight line between
    two finite points essentially always succeeds."""
    control_points = plan_straight_points(sx, sy, gx, gy)
    return {
        "control_points": [(float(x), float(y)) for x, y in control_points],
        "reason": "completed",
        "status": True,
    }


def bt_plan_voronoi(sx, sy, gx, gy):
    """BT.cpp-compatible wrapper around planners.py's
    plan_voronoi_points -- same shape/rationale as bt_plan_astar above.
    control_points is [] and reason is "no_path" only if a roadmap
    exists but start/goal are genuinely disconnected within it;
    degrades to a straight line (never fails) when there are no
    obstacles to route around."""
    control_points = plan_voronoi_points(sx, sy, gx, gy)
    if control_points is None:
        return {"control_points": [], "reason": "no_path", "status": False}
    return {
        "control_points": [(float(x), float(y)) for x, y in control_points],
        "reason": "completed",
        "status": True,
    }


def bt_follow_boarder(sx, sy, obstacle_id, offset):
    """BT.cpp-compatible wrapper around planners.py's
    follow_boarder_points -- matches PlanWith's obstacle_id/offset
    input ports (algorithm="follow_boarder" case) exactly (no goal
    port -- this planner doesn't decide when to leave the boundary,
    see follow_boarder_points's own docstring). control_points is []
    and reason is "no_path" only if obstacle_id names no known
    obstacle."""
    control_points = follow_boarder_points(sx, sy, obstacle_id, offset)
    if control_points is None:
        return {"control_points": [], "reason": "no_path", "status": False}
    return {
        "control_points": [(float(x), float(y)) for x, y in control_points],
        "reason": "completed",
        "status": True,
    }


_PLAN_ALGORITHM_FUNCS = {
    "astar": bt_plan_astar,
    "straight": bt_plan_straight,
    "voronoi": bt_plan_voronoi,
}


def bt_plan_with(algorithm, sx, sy, gx=None, gy=None, obstacle_id=None, offset=None):
    """ONE dispatch entry point covering every PlanWith algorithm --
    the direct Python-callable analogue of basic_action_theory.pl's own
    planWith(Algorithm,Goal,CP,ActionCode)/plan_call/8 dispatch (see
    schema.yaml's own note on why the four algorithms collapsed into
    one BT.cpp action). Routes to bt_plan_astar/bt_plan_straight/
    bt_plan_voronoi (gx,gy required) for those three algorithm values,
    or bt_follow_boarder (obstacle_id,offset required) for
    "follow_boarder" -- same {control_points, reason, status} return
    shape either way."""
    if algorithm == "follow_boarder":
        return bt_follow_boarder(sx, sy, obstacle_id, offset)
    if algorithm in _PLAN_ALGORITHM_FUNCS:
        return _PLAN_ALGORITHM_FUNCS[algorithm](sx, sy, gx, gy)
    raise ValueError(f"Unknown PlanWith algorithm '{algorithm}'.")


# =====================================================================
# ACTIONS -- interface-only (MoveTo): term builder, not an executor
# =====================================================================
def moveto_leg_term(control_points, triggers):
    """
    Build the basic_action_theory.pl TERM TEXT for one MoveTo node's
    bound inputs -- moveto_leg(ControlPoints,Triggers). Triggers is
    REQUIRED, matching basic_action_theory.pl's own moveto_leg/2 (there is
    deliberately no sugar/default form on either side -- every leg
    states its own protection level explicitly; pass [] for a
    genuinely unprotected leg).

    control_points: list of (x,y) pairs.
    triggers: list of strings (e.g. ["collision","battery"]).

    Returns Prolog source text, e.g.:
        "moveto_leg([point(1.0,2.0),point(3.0,4.0)],[collision,battery])"
    """
    cp_text = "[" + ",".join(
        f"point({float(x)},{float(y)})" for x, y in control_points) + "]"
    trig_text = "[" + ",".join(str(t) for t in triggers) + "]"
    return f"moveto_leg({cp_text},{trig_text})"


# =====================================================================
# ACTIONS -- term builder for the consolidated planner (PlanWith)
# =====================================================================
def plan_with_term(algorithm, goal, cp_var, action_code):
    """
    Build the basic_action_theory.pl TERM TEXT for one PlanWith node's
    bound inputs, algorithm in {"astar","straight","voronoi"} --
    planWith(Algorithm,point(GoalX,GoalY),CPVar,ActionCode) -- matching
    planWith's own 4-arg signature (Algorithm, Goal, CP, ActionCode) in
    basic_action_theory.pl. Use follow_boarder_term below instead for
    algorithm="follow_boarder" (no goal port, a compound Algorithm term
    instead). CPVar is left as a FREE PROLOG VARIABLE NAME (e.g. "CP"),
    not a value, since ControlPoints is this node's own OUTPUT, meant
    to be shared forward with a subsequent MoveTo node using the SAME
    variable name -- see basic_action_theory.pl's own note on the
    "leave a variable free, let a prior step bind it" pattern. Pass a
    distinct cp_var (e.g. "CP1", "CP2") when building more than one
    planning call in the same plan, per the fallback_node variable-
    sharing gotcha documented in basic_action_theory.pl. action_code
    (e.g. "a3") identifies THIS PlanWith occurrence -- same per-
    occurrence code moveto_leg_term's own ActionCode already uses --
    and rides through into the RECORDED Reason (completed(Algorithm,
    Goal,ActionCode)/no_path(Algorithm,Goal,ActionCode)), not just this
    call term, via tag_reason/3 (see do_node(planWith(...))'s own note).

    algorithm: "astar", "straight", or "voronoi" (a bare Prolog atom,
        unquoted).
    goal: an (x,y) pair.
    cp_var: a free Prolog variable name, as text (e.g. "CP").
    action_code: a free Prolog variable name or bound atom, as text
        (e.g. "a3").

    Returns Prolog source text, e.g.:
        "planWith(astar,point(17.0,17.0),CP,a3)"
    """
    gx, gy = goal
    return f"planWith({algorithm},point({float(gx)},{float(gy)}),{cp_var},{action_code})"


def follow_boarder_term(obstacle_id, offset, cp_var, action_code):
    """Build the basic_action_theory.pl TERM TEXT for one PlanWith
    node's bound inputs when algorithm="follow_boarder" --
    planWith(follow_boarder(ObstacleId,Offset), point(0.0,0.0), CPVar,
    ActionCode), same "leave CP free"/ActionCode convention as
    plan_with_term above. The point(0.0,0.0) is a PLACEHOLDER, not a
    real goal -- follow_boarder takes no goal port at all (it doesn't
    decide when to leave the boundary; see planners.py's
    follow_boarder_points docstring), but planWith/4's Goal slot is
    part of the shared template every algorithm sits inside, so
    something has to fill it; plan_call/8's own follow_boarder clauses
    ignore it outright (the RECORDED Reason reports the honest atom
    `none` as this call's own Goal instead -- see do_node(planWith
    (...))'s own note in basic_action_theory.pl). obstacle_id is
    written VERBATIM as Prolog text (a bare atom, e.g. "obs5"), same
    convention as halted_with_cond_term's own reason argument below --
    NOT quoted, NOT float-parsed.

    obstacle_id: a Prolog atom, as text (e.g. "obs5").
    offset: distance to maintain from the obstacle's own boundary,
        metres -- typically the SAME Threshold as whichever trigger/
        condition supplied obstacle_id in the first place.
    cp_var: a free Prolog variable name, as text (e.g. "CP").
    action_code: a free Prolog variable name or bound atom, as text
        (e.g. "a4").

    Returns Prolog source text, e.g.:
        "planWith(follow_boarder(obs5,0.6),point(0.0,0.0),CP,a4)"
    """
    return f"planWith(follow_boarder({obstacle_id},{float(offset)}),point(0.0,0.0),{cp_var},{action_code})"


# =====================================================================
# CONDITIONS -- interface-only: term builders
# =====================================================================
def distance_below_cond_term(goal, threshold):
    """cond(distance_below(GX,GY,Threshold)) term text -- matches
    DistanceBelow's goal/threshold ports in schema.yaml. PARAMETRIZED,
    same as obstacle_in_bound_cond_term/battery_below_cond_term below
    -- there is no global "the goal" fact this reads instead.

    goal: an (x,y) pair.
    threshold: distance threshold, metres.
    """
    gx, gy = goal
    return f"cond(distance_below({float(gx)},{float(gy)},{float(threshold)}))"


def distance_equal_cond_term(goal, threshold):
    """cond(distance_equal(GX,GY,Threshold)) term text -- matches
    DistanceEqual's goal/threshold ports in schema.yaml. Same shape as
    distance_below_cond_term above, exact-equality comparison."""
    gx, gy = goal
    return f"cond(distance_equal({float(gx)},{float(gy)},{float(threshold)}))"


def distance_over_cond_term(goal, threshold):
    """cond(distance_over(GX,GY,Threshold)) term text -- matches
    DistanceOver's goal/threshold ports in schema.yaml. Same shape as
    distance_below_cond_term above, ">" comparison."""
    gx, gy = goal
    return f"cond(distance_over({float(gx)},{float(gy)},{float(threshold)}))"


def halted_with_cond_term(reason):
    """cond(halted_with_cond(Reason)) term text -- matches
    HaltedWith's reason port in schema.yaml. `reason` is written
    VERBATIM as Prolog text, unquoted: a bare atom for
    completed/battery_depleted/a trigger name, or "crashed(_)" /
    "crashed(obs5)" / "obstacle_in_bound(_,_)" / "battery_under(20)"
    (etc.) for the Reasons that carry extra info -- see schema.yaml's
    own note on HaltedWith's reason port. A bare "crashed" (no
    obstacle argument) no longer matches anything."""
    return f"cond(halted_with_cond({reason}))"


def obstacle_in_bound_cond_term(threshold):
    """cond(obstacle_in_bound(Threshold)) term text -- matches
    ObstacleInBound's threshold port in schema.yaml."""
    return f"cond(obstacle_in_bound({float(threshold)}))"


def obstacle_on_path_cond_term(threshold):
    """cond(obstacle_on_path(Threshold)) term text -- matches
    ObstacleOnPath's threshold port in schema.yaml. Distinct from
    obstacle_in_bound_cond_term above: this only fires for obstacles
    the CURRENT walk's trajectory actually enters, not any nearby
    obstacle."""
    return f"cond(obstacle_on_path({float(threshold)}))"


def battery_below_cond_term(threshold):
    """cond(battery_below(Threshold)) term text -- matches
    BatteryBelow's threshold port in schema.yaml."""
    return f"cond(battery_below({float(threshold)}))"


def battery_equal_cond_term(threshold):
    """cond(battery_equal(Threshold)) term text -- matches
    BatteryEqual's threshold port in schema.yaml."""
    return f"cond(battery_equal({float(threshold)}))"


def battery_over_cond_term(threshold):
    """cond(battery_over(Threshold)) term text -- matches
    BatteryOver's threshold port in schema.yaml."""
    return f"cond(battery_over({float(threshold)}))"


def line_of_sight_clear_cond_term(obstacle_id, goal):
    """cond(line_of_sight_clear(ObstacleId,GX,GY)) term text -- matches
    LineOfSightClear's obstacle_id/goal ports in schema.yaml. Bug0's
    own boundary-leave rule as a standalone condition; no
    crosses_segment counterpart exists -- see schema.yaml's own note
    on LineOfSightClear for why. obstacle_id is written VERBATIM as
    Prolog text (a bare atom), same convention as halted_with_cond_term
    above."""
    gx, gy = goal
    return f"cond(line_of_sight_clear({obstacle_id},{float(gx)},{float(gy)}))"


# =====================================================================
# Registry -- maps schema.yaml's IDs to their implementation here.
# Not required for either caller to function (both can call the
# functions above directly), but gives one place that stays
# consistent with schema.yaml, and a natural hook for future
# consistency-checking or XML/tree-translation tooling.
# =====================================================================
ACTIONS = {
    "MoveTo": {
        "kind": "interface_only",
        "prolog_action": "moveto_leg",
        "term_builder": moveto_leg_term,
    },
    # NOT a single "term_builder" entry here -- which one applies
    # depends on this node's own algorithm VALUE, not something fixed
    # per schema id the way every other action/condition here is: use
    # plan_with_term for astar/straight/voronoi, follow_boarder_term
    # for algorithm="follow_boarder" (see each builder's own docstring
    # above).
    "PlanWith": {
        "kind": "callable",
        "prolog_action": "planWith",
        "func": bt_plan_with,
    },
}

CONDITIONS = {
    "DistanceBelow": {
        "kind": "interface_only",
        "prolog_condition": "distance_below",
        "term_builder": distance_below_cond_term,
    },
    "DistanceEqual": {
        "kind": "interface_only",
        "prolog_condition": "distance_equal",
        "term_builder": distance_equal_cond_term,
    },
    "DistanceOver": {
        "kind": "interface_only",
        "prolog_condition": "distance_over",
        "term_builder": distance_over_cond_term,
    },
    "HaltedWith": {
        "kind": "interface_only",
        "prolog_condition": "halted_with_cond",
        "term_builder": halted_with_cond_term,
    },
    "ObstacleInBound": {
        "kind": "interface_only",
        "prolog_condition": "obstacle_in_bound",
        "term_builder": obstacle_in_bound_cond_term,
    },
    "ObstacleOnPath": {
        "kind": "interface_only",
        "prolog_condition": "obstacle_on_path",
        "term_builder": obstacle_on_path_cond_term,
    },
    "BatteryBelow": {
        "kind": "interface_only",
        "prolog_condition": "battery_below",
        "term_builder": battery_below_cond_term,
    },
    "BatteryEqual": {
        "kind": "interface_only",
        "prolog_condition": "battery_equal",
        "term_builder": battery_equal_cond_term,
    },
    "BatteryOver": {
        "kind": "interface_only",
        "prolog_condition": "battery_over",
        "term_builder": battery_over_cond_term,
    },
    "LineOfSightClear": {
        "kind": "interface_only",
        "prolog_condition": "line_of_sight_clear",
        "term_builder": line_of_sight_clear_cond_term,
    },
}