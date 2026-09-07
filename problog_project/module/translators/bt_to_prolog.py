#!/usr/bin/env python3
"""
module/translators/bt_to_prolog.py

The BT.cpp XML tree -> Prolog do_node term translator flagged as
"not yet built" in this project's own history. Reads a real
BehaviorTree.cpp v4 XML tree (a problem's own behavior_tree.xml),
validates it against module/contracts/schema.yaml (every leaf node must
be a known, correctly-instantiated schema action/condition; every
control-flow node must be one of the BT.cpp built-ins this project
supports -- Sequence/Fallback, which map 1:1 to basic_action_theory.pl's
seq_node/fallback_node and never catch/redescend a reactive(_) status on
their own; ReactiveSequence/ReactiveFallback, which map to
reactivesequence(Code)/reactivefallback(Code) and DO catch/locally
redescend one whose own code matches -- see _REACTIVE_CONTROL_FLOW's own
note and basic_action_theory.pl's own CONTROL-FLOW REDESCEND TARGETS
note for the full mechanism; and Inverter, BT.cpp's single-child
negation decorator, which maps to inverter(Child) -- flips true/false,
passes a reactive(_) status straight through unchanged, never itself
reactive), and translates it into the nested do_node/4 term text
basic_action_theory.pl's plan/1 expects, plus one reactive_children/2
fact per ReactiveSequence/ReactiveFallback (see generate_plan_pl's own
note on why those live separately).

WHERE THE RESULT GOES: generate_plan_pl() writes the problem's own
plan_generated.pl, a single plan/1 FACT (not a clause with a body --
see below), which basic_action_theory.pl consults (via its
problem_data.pl bootstrap -- see that file's Section 0) instead of
hand-defining plan/1 itself. This mirrors the existing
generated-artifact pattern (config_generated.pl, obstacles_generated.pl,
both written by this file's own sibling translators into the same
problem directory): the XML is the single source of truth for the
POLICY'S SHAPE from now on -- change the tree by editing the XML and
re-running, not by hand-editing basic_action_theory.pl.
main.py regenerates it automatically before every run, exactly like it
already does for config_generated.pl.

WHY A FACT, NOT A CLAUSE WITH A BODY: the old hand-written plan/1 was
`plan(seq_node([moveto_leg(CP,Triggers)])) :- control_points(CP),
default_triggers(Triggers).` -- CP had to be bound by a BODY goal
because the Node term itself only ever REFERENCED CP, never bound it.
Once a plan's own PlanWith leaf computes ControlPoints
itself (via planWith(Algorithm,Goal,CP)), CP is bound INSIDE the Node
term the moment do_node actually runs it -- there is nothing left for a
body to bind, so the generated plan/1 is a plain fact.

BLACKBOARD -> PROLOG VARIABLE TRANSLATION: BT.cpp wires one node's
output port to another's input port by giving both the SAME
"{blackboard_key}" attribute value (this is a REAL, standard BT.cpp
convention, not something invented for this project). control_points is
the one port in this schema that is ALWAYS wired this way -- PlanWith
computes it, it never receives it as a literal, and
MoveTo's own leg has no way to invent it, so a literal control_points
value is a hard error, never a valid input here. Each distinct
blackboard key becomes ONE Prolog variable, shared across every node
that references it, via straightforward unification -- e.g. "{cp}" on
both a PlanWith and a MoveTo node becomes the SAME Prolog variable CP
in planWith(astar,point(GX,GY),CP) and moveto_leg(CP,[...]) -- the
direct Prolog analogue of BT.cpp's blackboard, and exactly the existing
"leave a variable free, let a prior step bind it" pattern already
documented in basic_action_theory.pl for hand-written multi-leg plans.
Never reuse one key across two DIFFERENT PlanWith calls
that should compute independent paths -- see basic_action_theory.pl's own
note on giving fallback_node branches distinct CP1/CP2 variables; the
same Prolog-variable-scope reasoning applies here.

OTHER PORT ENCODINGS (this project's own choice; schema.yaml describes
port TYPES, not a serialization -- see its own note pointing here):
    Point               "X;Y"                  e.g. goal="11.675;11.525"
    vector<std::string> ";"-separated           e.g. triggers="collision;battery"
    double / string     the attribute's own text, parsed by Python's
                         float()/left as-is respectively

CONTROL-FLOW GUARD DERIVATION: a MoveTo's own Triggers list is no
longer entirely hand-typed. For every <MoveTo>, this file now walks
UP the tree from it to the root; at each ReactiveSequence/
ReactiveFallback ancestor, every LEFT SIBLING of the branch leading to
the MoveTo (optionally wrapped in one or more <Inverter>) that reduces
to a single Condition leaf becomes an automatically-derived guard --
a Sequence-shaped ancestor requires its left siblings to stay TRUE
(interrupts on becoming false), a Fallback-shaped one requires them to
stay FALSE (interrupts on becoming true; each <Inverter> flips this
once), and the guard is tagged with THAT SPECIFIC ancestor's own code,
not necessarily the nearest enclosing reactive composite (two nested
reactive ancestors contributing guards to the same MoveTo get two
DIFFERENT codes -- see _reduce_guard_condition's own note). Rather
than looking up a pre-built "opposite" trigger name per condition
(which would need both crossing directions hand-implemented for every
condition, and silently do the wrong thing for any gap), the required
condition is built by NEGATING the actual Condition term when the
guard's polarity calls for it (reusing holds/2's own neg/1
combinator), and basic_action_theory.pl's guard_break(Cond,Code)
trigger + holds_leg/9 do a GENERIC bracket-scan+bisection search for
when THAT EXACT term stops holding -- see that file's own note above
holds_leg/9. A left sibling that is a memory-level (plain Sequence/
Fallback) guard, or that reduces to a HISTORY-based condition (e.g.
HaltedWith -- see _NON_CONTINUOUS_CONDITIONS), or that reduces to
neither a Condition leaf nor an <Inverter> chain over one, produces no
Triggers entry / a hard BTValidationError respectively -- see
_reduce_guard_condition.

Every MoveTo also gets `collision` and (when this problem's own
config.yaml has battery.enabled: true) `battery` ADDED AUTOMATICALLY,
regardless of what its own triggers="..." attribute says -- these are
universal physical hazards, not BT-structural guards, so the tree
author no longer has to spell them out (see _translate_leaf's own
moveto_leg branch). The triggers port itself is now OPTIONAL and
purely ADDITIVE: still there for leg-intrinsic termination conditions
that aren't derivable from tree structure at all (e.g. a Bug-algorithm
leg's own line_of_sight_clear/crosses_segment stopping rule).

VALIDATION IS A HARD FAILURE, not a warning: an unknown node tag, a
missing required port, an unrecognized attribute (anything not a
declared port, other than BT.cpp's own universal `name` display
attribute), or a MoveTo/moveto_leg control_points key with no
corresponding PlanWith producer anywhere in the tree
(which would silently leave CP unbound) all raise BTValidationError and
stop the run -- these are structural errors in the tree itself, not a
tunable value, so there is nothing sensible to warn-and-continue with
(same reasoning as module/translators/config_to_prolog.py's own hard
requirements, as opposed to its two non-fatal numeric warnings).
"""
import os
import re
import xml.etree.ElementTree as ET

import yaml

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))
_DEFAULT_PROBLEM_DIR = os.path.join(_PROJECT_ROOT, "problems", "problem0")
DEFAULT_XML_PATH = os.path.join(_DEFAULT_PROBLEM_DIR, "behavior_tree.xml")
DEFAULT_SCHEMA_PATH = os.path.join(_PROJECT_ROOT, "module", "contracts", "schema.yaml")
DEFAULT_OUTPUT_PATH = os.path.join(_DEFAULT_PROBLEM_DIR, "plan_generated.pl")

# Every schema action's `id` maps to how it's dispatched below: which
# do_node/4 Prolog functor it becomes.
_ACTION_DISPATCH = {
    "MoveTo": {"kind": "moveto_leg"},
    # ONE consolidated planner action -- see schema.yaml's own note on
    # why PlanAstar/PlanStraight/PlanVoronoi/FollowBoarder collapsed
    # into this single PlanWith id (Algorithm was always just a
    # runtime value plan_call/8 dispatches on, never four different
    # predicates). Which concrete algorithm runs is read off THIS
    # node's own `algorithm` attribute at translation time, not fixed
    # per schema entry -- see _PLAN_ALGORITHMS below and the "planWith"
    # kind's own branch in _translate_leaf.
    "PlanWith": {"kind": "planWith"},
}

# The only valid values for PlanWith's own `algorithm` port. astar/
# straight/voronoi take a `goal` port and become a BARE Prolog atom;
# follow_boarder takes `obstacle_id`/`offset` instead and becomes the
# COMPOUND term follow_boarder(ObstacleId,Offset) plan_call/8's own
# follow_boarder clauses dispatch on (see basic_action_theory.pl).
_PLAN_ALGORITHMS = {"astar", "straight", "voronoi", "follow_boarder"}
# "single_float_port": the shared shape of every cond(Functor(Value))
# condition whose one port is a plain float -- ObstacleInBound and
# BatteryBelow/Equal/Over all reduce to this, just with different
# functor/port names, so they share ONE translation branch below
# instead of several near-identical ones.
_CONDITION_DISPATCH = {
    # distance_below/distance_equal/distance_over(GX,GY,Threshold) --
    # PARAMETRIZED (no global "the goal" fact to read instead), so they
    # need their own dispatch kind, not the shared single_float_port
    # shape (a Point port AND a float port, not just one float) -- but
    # all THREE share that one kind, same "one shared shape, several
    # functors" idea single_float_port itself already uses for
    # ObstacleInBound/BatteryBelow/etc.
    "DistanceBelow": {"kind": "distance_cond", "functor": "distance_below"},
    "DistanceEqual": {"kind": "distance_cond", "functor": "distance_equal"},
    "DistanceOver": {"kind": "distance_cond", "functor": "distance_over"},
    "ObstacleInBound": {"kind": "single_float_port", "functor": "obstacle_in_bound", "port": "threshold"},
    "ObstacleOnPath": {"kind": "single_float_port", "functor": "obstacle_on_path", "port": "threshold"},
    "BatteryBelow": {"kind": "single_float_port", "functor": "battery_below", "port": "threshold"},
    "BatteryEqual": {"kind": "single_float_port", "functor": "battery_equal", "port": "threshold"},
    "BatteryOver": {"kind": "single_float_port", "functor": "battery_over", "port": "threshold"},
    "HaltedWith": {"kind": "halted_with_cond"},
    # line_of_sight_clear(ObstacleId,GX,GY) -- obstacle_id verbatim
    # Prolog text (like HaltedWith's reason), goal a Point literal.
    "LineOfSightClear": {"kind": "line_of_sight_clear_cond"},
}
_CONTROL_FLOW = {"Sequence": "seq_node", "Fallback": "fallback_node"}
# ReactiveSequence/ReactiveFallback are NOT simple functor-renames like
# Sequence/Fallback above -- translating one means assigning it a fresh,
# unique code, recursing into its OWN children with that code as the
# "current enclosing reactive composite" for anything underneath (so
# every reactive-classified trigger inside gets tagged with it -- see
# _REACTIVE_TRIGGER_FUNCTORS below), and factoring those children OUT
# into their own reactive_children/2 fact rather than inlining them --
# see basic_action_theory.pl's own CONTROL-FLOW REDESCEND TARGETS note
# (above do_node(reactivesequence(...))) for why a separately-resolved
# fact is required (the SAME "fresh variables on every resolution"
# reasoning plan(Node) itself already relies on). Handled as its own
# branch in _translate_node, not via _CONTROL_FLOW's simple lookup.
_REACTIVE_CONTROL_FLOW = {"ReactiveSequence": "reactivesequence", "ReactiveFallback": "reactivefallback"}

# Condition ids that are HISTORY-based rather than a live, continuous
# fluent -- they cannot change WHILE a leg is running (nothing appends
# to the situation history until the CURRENT leg itself halts), so
# there is nothing for a crossing-search to ever watch: the composite
# that contains one already checked it, ONCE, before ever descending
# into this branch. Excluded from automatic guard-trigger derivation
# (see _reduce_guard_condition) for exactly that reason -- NOT because
# it's unsupported, but because "make it reactive" would be a no-op at
# best; basic_action_theory.pl's holds_leg/9 (the generic engine behind
# guard derivation) deliberately has NO clause for it either, so this
# exclusion also prevents a missing-clause silently reading as "already
# false at T0" there. A future condition added to schema.yaml that is
# similarly history-based (not a function of the CURRENT leg's own
# position/battery) belongs here too.
_NON_CONTINUOUS_CONDITIONS = {"HaltedWith"}

# Trigger-list functors that are REACTIVE-classified in leg_status/9
# (basic_action_theory.pl) -- i.e. everything except the two original,
# unparametrized, never-reactive names 'collision' and 'battery' (which
# classify straight to false, via crashed(_)/battery_depleted). Every
# token using one of THESE functors, in a MoveTo's own triggers list,
# gets the current enclosing reactive composite's own code appended as
# an extra trailing argument (e.g. "battery_below(70)" in the XML
# becomes battery_below(70,rc3) in the generated Prolog) -- see
# trigger_crossing_time/11's own note in basic_action_theory.pl.
_REACTIVE_TRIGGER_FUNCTORS = {
    "obstacle_in_bound", "obstacle_on_path",
    "battery_below", "battery_equal", "battery_over",
    "line_of_sight_clear", "crosses_segment",
}

# BT.cpp's own universal attribute, present on any node purely for
# display/debugging -- never a real port, always allowed, never
# validated against a schema port list.
_ALWAYS_ALLOWED_ATTRS = {"name"}

_BLACKBOARD_RE = re.compile(r"^\{(\w+)\}$")
_VALID_PROLOG_VAR_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


class BTValidationError(Exception):
    """Raised for any structural problem in the BT XML relative to the
    schema -- always fatal, never a warning (see module docstring)."""


def load_schema(schema_path=DEFAULT_SCHEMA_PATH):
    with open(schema_path) as f:
        return yaml.safe_load(f)


def _schema_port_index(schema):
    """id -> {port_name: port_spec} for every action AND condition."""
    index = {}
    for kind_key in ("actions", "conditions"):
        for entry in schema.get(kind_key, []):
            index[entry["id"]] = {p["name"]: p for p in entry.get("ports", [])}
    return index


def _is_blackboard_ref(value):
    return _BLACKBOARD_RE.match(value) is not None


def _blackboard_key(value):
    return _BLACKBOARD_RE.match(value).group(1)


class _VarPool:
    """Maps each distinct blackboard key to ONE Prolog variable name,
    reused every time that key is seen again -- see the module
    docstring's "BLACKBOARD -> PROLOG VARIABLE TRANSLATION" section.

    ALSO tracks everything needed for ReactiveSequence/ReactiveFallback
    translation: a counter for assigning each one a fresh, unique code;
    the accumulated (code, children_terms) pairs to emit as separate
    reactive_children/2 facts (see _REACTIVE_CONTROL_FLOW's own note);
    and, per blackboard key, EVERY reactive-scope it was touched
    (produced or consumed) under -- a key touched under more than one
    distinct scope (including plain "outside any reactive composite",
    recorded as None) would mean a producer/consumer pair straddles a
    reactive_children/2 boundary, which breaks the SAME way reusing one
    CP across two fallback_node branches already does (see
    basic_action_theory.pl's own "IMPORTANT GOTCHA" note) -- checked
    once, after the whole tree is translated, in translate_tree."""

    def __init__(self):
        self._map = {}
        self.producers = set()   # blackboard keys with an OUTPUT-port producer
        self.consumers = set()   # blackboard keys read by an INPUT port
        self._reactive_counter = 0
        self.reactive_facts = []       # [(code, [child_term, ...]), ...]
        self.key_scopes = {}           # key -> set of codes (None = outside any)
        self._action_counter = 0
        self.reason_patterns_by_action = {}   # action_code -> [reason_pattern_text, ...]
        self.action_labels = {}   # action_code -> human-readable "what/where" label,
                                   # e.g. "MoveTo [TryGoal]" or "PlanWith(straight, goal=point(22.275,2.075)) [TryGoal]"
                                   # -- see next_action_code()'s own note; purely for
                                   # main.py's printed action-code legend, never
                                   # consulted by anything Prolog-side.
        self._condition_counter = 0
        self.condition_labels = {}   # condition_code -> human-readable "what/where" label,
                                      # e.g. "DistanceBelow(2.275,2.075,0.3) [GoHome]" --
                                      # see next_condition_code()'s own note.

    def var_for(self, key):
        if key not in self._map:
            var = key.upper()
            if not _VALID_PROLOG_VAR_RE.match(var):
                raise BTValidationError(
                    f"Blackboard key '{key}' does not translate to a valid "
                    f"Prolog variable name ('{var}') -- use a plain "
                    f"alphanumeric/underscore key starting with a letter.")
            self._map[key] = var
        return self._map[key]

    def note_key_scope(self, key, reactive_code):
        self.key_scopes.setdefault(key, set()).add(reactive_code)

    def next_reactive_code(self):
        self._reactive_counter += 1
        return f"rc{self._reactive_counter}"

    def next_action_code(self):
        """A fresh, unique code for one <MoveTo> OCCURRENCE -- embedded
        by basic_action_theory.pl's do_node(moveto_leg(CP,Triggers,
        ActionCode),...) into startMoveto's own action term, and from
        there tagged onto the leg's own final halt Reason (see
        tag_reason/3) so a safety query can distinguish WHICH MoveTo in
        the tree produced a given halt (e.g. halted_with_cond(crashed
        (ObstacleId,a1)) vs. (...,a2)) -- same per-occurrence-counter
        pattern as next_reactive_code() above, just for actions rather
        than reactive composites."""
        self._action_counter += 1
        return f"a{self._action_counter}"

    def next_condition_code(self):
        """A fresh, unique code for one <Condition> LEAF occurrence --
        embedded by basic_action_theory.pl's do_node(cond(C,Code),...)
        into its own checked(Code,C,Status) marker (see that predicate's
        own note), so a safety query can distinguish WHICH cond() leaf
        in the tree a given check/outcome came from -- same per-
        occurrence-counter idiom as next_action_code()/
        next_reactive_code() above, just for condition leaves rather
        than actions/reactive composites. Deliberately a SEPARATE
        namespace/prefix (c1, c2, ...) from ActionCode (a1, a2, ...) --
        the two are never compared against each other, only ever used
        as opaque keys into their own respective dicts, so there's no
        need for them to share one counter."""
        self._condition_counter += 1
        return f"c{self._condition_counter}"


def _validate_ports(tag, elem, port_specs):
    """Raise BTValidationError if a required port is missing or an
    unrecognized attribute is present (other than BT.cpp's own `name`).
    Returns the element's attributes dict for convenience."""
    attrs = dict(elem.attrib)
    unknown = set(attrs) - set(port_specs) - _ALWAYS_ALLOWED_ATTRS
    if unknown:
        raise BTValidationError(
            f"<{tag}> has unrecognized attribute(s) {sorted(unknown)} -- "
            f"not a declared port in schema.yaml for '{tag}'.")
    missing = [name for name, spec in port_specs.items()
               if spec.get("required") and name not in attrs]
    if missing:
        raise BTValidationError(
            f"<{tag}> is missing required port(s) {missing} "
            f"(schema.yaml: module/contracts/schema.yaml's '{tag}' entry).")
    return attrs


def _point_literal(text, tag, port_name):
    x, y = _point_xy(text, tag, port_name)
    return f"point({x},{y})"


def _point_xy(text, tag, port_name):
    """Same "X;Y" parsing as _point_literal, but returns the raw
    (x,y) floats instead of a wrapped point(X,Y) term -- for the rarer
    case (line_of_sight_clear(ObstacleId,GX,GY), notably) where the
    target Prolog predicate takes GX,GY as flat arguments rather than
    a nested point/2 term."""
    parts = text.split(";")
    if len(parts) != 2:
        raise BTValidationError(
            f"<{tag}>'s '{port_name}' port ('{text}') is not a valid Point "
            f"-- expected \"X;Y\", e.g. \"11.675;11.525\".")
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        raise BTValidationError(
            f"<{tag}>'s '{port_name}' port ('{text}') has non-numeric "
            f"X/Y -- expected \"X;Y\" with two floats.")


def _is_battery_trigger(token):
    """True for a Triggers-list token that's battery-related -- the
    bare atom 'battery' or any battery_<whatever>(...) functor
    (battery_below(N), battery_over(N), battery_equal(N), and any
    future one, matched by prefix rather than an enumerated list so a
    new battery_* trigger added later is covered automatically). Used
    to strip battery out of a leg's own Triggers list entirely when
    this problem's own config.yaml sets battery.enabled: false -- see
    that flag's own comment for why "not considering battery in the
    problem" has to happen HERE, at translation time, not just by
    dropping the any_battery_depletion query: a Triggers list is baked
    into plan_generated.pl, a problem-specific generated file, exactly
    like config_generated.pl, so it's the right place for a
    problem-specific flag to take effect, and it's the only way to
    stop battery from ever being able to HALT a walk (as opposed to
    merely not being reported on)."""
    functor = token.split("(", 1)[0].strip()
    return functor == "battery" or functor.startswith("battery_")


_BATTERY_CONDITION_RE = re.compile(r"\bbattery_(below|equal|over)\(")


def _guard_condition_mentions_battery(cond_term):
    """True if cond_term (an auto-derived guard's condition text, e.g.
    "battery_over(70.0)" or "neg(battery_over(70.0))") tests a battery
    threshold -- the SAME "battery.enabled: false means battery can
    never halt a walk" rule _is_battery_trigger enforces for manually-
    typed trigger tokens above, extended to cover auto-derived
    guard_break(Cond,Code) ones, whose OUTER functor is guard_break/
    neg, not battery_*, so _is_battery_trigger's own functor-prefix
    check wouldn't catch it."""
    return _BATTERY_CONDITION_RE.search(cond_term) is not None


def _with_branch_suffix(label, branch_name):
    """Appends " [BranchName]" to an action's own human-readable label
    (see _VarPool.action_labels's own note) when it sits under a named
    Sequence/Fallback/ReactiveSequence/ReactiveFallback ancestor --
    branch_name is the NEAREST such name, threaded down through
    _translate_node (an unnamed ancestor doesn't override an outer
    named one -- see that function's own note). Purely cosmetic, for
    main.py's printed action-code legend -- never affects the
    generated Prolog term itself."""
    return f"{label} [{branch_name}]" if branch_name else label


def _append_reactive_code(token, reactive_code):
    """token is a Triggers-list entry whose functor is in
    _REACTIVE_TRIGGER_FUNCTORS (e.g. "battery_below(70)" or the bare
    "obstacle_in_bound(0.6)") -- append reactive_code as an extra
    trailing argument (e.g. "battery_below(70,rc3)"). Every one of
    these functors already takes at least one argument (see
    _REACTIVE_TRIGGER_FUNCTORS's own note), so this is always "insert
    before the final close-paren", never "wrap a bare atom in ()"."""
    assert token.endswith(")"), token
    return f"{token[:-1]},{reactive_code})"


def _trigger_args(token):
    """Parse a SIMPLE (non-nested-paren) trigger token's own argument
    list as raw text, e.g. "obstacle_in_bound(0.6)" -> ["0.6"],
    "collision" -> []. Safe for every one of the fixed trigger names
    below -- none of them ever nest parens in their OWN arguments
    (unlike guard_break, whose Cond argument can nest arbitrarily and
    is handled separately, directly from its own already-known pieces,
    never by re-parsing formatted text)."""
    if "(" not in token:
        return []
    inner = token[token.index("(") + 1:-1]
    return [a.strip() for a in inner.split(",")]


def _reason_pattern_for_manual_trigger(token):
    """The UNTAGGED Reason shape (see tag_reason/3 in basic_action_
    theory.pl) a manually-typed Triggers-list token (BEFORE any
    reactive Code got appended -- see _append_reactive_code) produces
    when it actually fires -- mirrors trigger_crossing_time/11's own
    Reason construction exactly, functor-for-functor (note
    battery_below's own Reason functor is battery_under, not
    battery_below -- the one place a trigger name and its own Reason
    functor genuinely differ). Any part of the Reason that's only ever
    known at RUNTIME (an argmin ObstacleId for collision/obstacle_in_
    bound/obstacle_on_path) is written as the GROUND ATOM 'wild' here,
    NOT a genuine Prolog variable ('_') -- basic_action_theory.pl's own
    match_wild/2 unwraps 'wild' internally; a real free variable here
    would instead make the resulting query(...) declaration itself
    non-ground, and ProbLog reports one result row PER DISTINCT
    GROUNDING of a non-ground query rather than aggregating them into
    one probability (verified directly against ProbLog's own engine),
    silently turning e.g. "P(crashed on a1)" into a separate row per
    obstacle ID instead of one combined number. Everything else (a
    Threshold, a literal ObstacleId/GX/GY the tree author wrote) is
    already known at translation time and kept LITERAL, so
    module/contracts/goal_formula_check.py's generate_safety_queries
    can match it exactly. Used ONLY to build the safety-query universe
    ahead of time -- has no bearing on run-time correctness of
    anything trigger_crossing_time itself computes."""
    functor = token.split("(", 1)[0].strip()
    args = _trigger_args(token)
    if functor == "collision":
        return "crashed(wild)"
    if functor == "battery":
        return "battery_depleted"
    if functor == "obstacle_in_bound":
        return f"obstacle_in_bound({args[0]},wild)"
    if functor == "obstacle_on_path":
        return f"obstacle_on_path({args[0]},wild)"
    if functor == "battery_below":
        return f"battery_under({args[0]})"
    if functor == "battery_equal":
        return f"battery_equal({args[0]})"
    if functor == "battery_over":
        return f"battery_over({args[0]})"
    if functor == "line_of_sight_clear":
        return f"line_of_sight_clear({args[0]},{args[1]},{args[2]})"
    if functor == "crosses_segment":
        return f"crosses_segment({args[0]},{args[1]},{args[2]},{args[3]})"
    raise BTValidationError(
        f"Unknown trigger '{token}' -- no known Reason-pattern mapping "
        f"for safety-query generation; add one to "
        f"_reason_pattern_for_manual_trigger.")


def _leaf_condition_term(tag, attrs):
    """The BARE Prolog condition term for a <Condition> leaf (e.g.
    "battery_over(70.0)"), WITHOUT the cond(...) wrapper -- shared by
    _translate_leaf (which wraps it in cond(...) for a genuine, one-
    shot cond() leaf) and _reduce_guard_condition below (which wraps
    it in neg(...) instead, or leaves it bare, depending on the
    required guard polarity). attrs must already be validated (see
    _validate_ports) against tag's own port_specs."""
    info = _CONDITION_DISPATCH[tag]
    if info["kind"] == "single_float_port":
        value = float(attrs[info["port"]])
        return f"{info['functor']}({value})"
    if info["kind"] == "halted_with_cond":
        reason = attrs["reason"].strip()
        return f"halted_with_cond({reason})"
    if info["kind"] == "line_of_sight_clear_cond":
        obstacle_id = attrs["obstacle_id"].strip()
        gx, gy = _point_xy(attrs["goal"], tag, "goal")
        return f"line_of_sight_clear({obstacle_id},{gx},{gy})"
    if info["kind"] == "distance_cond":
        gx, gy = _point_xy(attrs["goal"], tag, "goal")
        threshold = float(attrs["threshold"])
        return f"{info['functor']}({gx},{gy},{threshold})"
    raise BTValidationError(f"Unhandled condition kind for <{tag}>.")


def _reduce_guard_condition(elem, required_polarity, schema_ports):
    """A left sibling (under a ReactiveSequence/ReactiveFallback) of
    the branch leading to some reactively-guarded Action, reduced to
    the SINGLE Prolog condition term that must stay TRUE for the guard
    to keep holding -- see the module docstring's CONTROL-FLOW GUARD
    DERIVATION note. required_polarity is what Step 1 (Sequence-shaped
    ancestor -> left siblings must SUCCEED -> True; Fallback-shaped ->
    must FAIL -> False) demands BEFORE accounting for any <Inverter>
    wrapping -- each Inverter layer flips it once, since Inverter(C)
    succeeds iff C fails.

    Returns None (SKIP -- no guard derived, no error) for a left
    sibling that can't be automatically watched: an Action (e.g.
    PlanWith, a completely ordinary left sibling of a MoveTo -- see
    problem4's own TryGoal branch) or a composite, because once it has
    SUCCEEDED it stays succeeded for the rest of the leg (an already-
    completed action never retroactively fails), so it can never
    supply the SUCCESS->FAILURE / FAILURE->SUCCESS transition a guard
    interrupt needs -- there is nothing WRONG with such a sibling, it
    simply isn't a source of a live interrupt, same as a condition in
    _NON_CONTINUOUS_CONDITIONS (history-based, e.g. HaltedWith -- it
    already gated entry into this branch, once, and can't change mid-
    leg either). Only a (possibly Inverter-wrapped) Condition leaf that
    CAN vary within a leg produces an actual guard term.

    Still raises BTValidationError for a genuinely MALFORMED <Inverter>
    (BT.cpp decorators always take exactly one child) -- a real
    structural bug, independent of guard derivation."""
    tag = elem.tag
    if tag == "Inverter":
        children = list(elem)
        if len(children) != 1:
            raise BTValidationError(
                f"<Inverter> must have exactly one child (found "
                f"{len(children)}).")
        return _reduce_guard_condition(children[0], not required_polarity, schema_ports)
    if tag in _NON_CONTINUOUS_CONDITIONS or tag not in _CONDITION_DISPATCH:
        return None
    attrs = _validate_ports(tag, elem, schema_ports[tag])
    cond_term = _leaf_condition_term(tag, attrs)
    return cond_term if required_polarity else f"neg({cond_term})"


def _translate_leaf(tag, elem, dispatch, port_specs, var_pool, battery_enabled, reactive_code, guard_stack, branch_name):
    attrs = _validate_ports(tag, elem, port_specs)

    if tag in _ACTION_DISPATCH:
        info = _ACTION_DISPATCH[tag]
        if info["kind"] == "moveto_leg":
            cp_value = attrs["control_points"]
            if not _is_blackboard_ref(cp_value):
                raise BTValidationError(
                    f"<MoveTo>'s control_points port ('{cp_value}') must be "
                    f"a blackboard reference like \"{{cp}}\" -- it is always "
                    f"computed by a planner, never a literal (see this "
                    f"module's own header).")
            key = _blackboard_key(cp_value)
            var_pool.consumers.add(key)
            var_pool.note_key_scope(key, reactive_code)
            cp_var = var_pool.var_for(key)

            # Manual, EXPLICIT extras only -- collision/battery are no
            # longer written here (see below); the port itself is now
            # OPTIONAL (schema.yaml's triggers required: false), so an
            # absent attribute is just "no extras".
            manual_tokens = [t.strip() for t in attrs.get("triggers", "").split(";") if t.strip()]
            if not battery_enabled:
                manual_tokens = [t for t in manual_tokens if not _is_battery_trigger(t)]
            tagged_manual = []
            for t in manual_tokens:
                functor = t.split("(", 1)[0].strip()
                if functor in ("collision", "battery"):
                    # Backward-compatible with older trees that still
                    # spell these out -- covered by default_tokens
                    # below either way, so skip rather than duplicate.
                    continue
                if functor in _REACTIVE_TRIGGER_FUNCTORS:
                    if reactive_code is None:
                        raise BTValidationError(
                            f"<MoveTo>'s triggers port includes '{t}', a "
                            f"reactive-classified trigger, but this MoveTo is "
                            f"not enclosed by any <ReactiveSequence>/"
                            f"<ReactiveFallback> -- there is nowhere for its "
                            f"reactive(_) halt to ever be caught, so it would "
                            f"redescend all the way to the root and be "
                            f"reported as plan_outcome(reactive_escaped) "
                            f"(see basic_action_theory.pl's own CONTROL-FLOW "
                            f"REDESCEND TARGETS note). Wrap this MoveTo (or "
                            f"an ancestor of it) in a ReactiveSequence/"
                            f"ReactiveFallback, or drop '{t}' from triggers.")
                    tagged_manual.append(_append_reactive_code(t, reactive_code))
                else:
                    tagged_manual.append(t)

            # Structural guards, auto-derived from every enclosing
            # ReactiveSequence/ReactiveFallback's own left siblings --
            # see the module docstring's CONTROL-FLOW GUARD DERIVATION
            # note. Each entry in guard_stack is already the exact,
            # polarity-adjusted condition term (see
            # _reduce_guard_condition) paired with the SPECIFIC
            # ancestor level's own code (NOT necessarily the nearest
            # enclosing one -- two nested reactive ancestors can
            # contribute two guards with two DIFFERENT codes here).
            derived_tokens = [
                f"guard_break({cond_term},{code})"
                for cond_term, code in guard_stack
                if battery_enabled or not _guard_condition_mentions_battery(cond_term)
            ]

            # Universal physical hazards -- ALWAYS collision, battery
            # (the fixed 0%-depletion one) only if this problem models
            # battery at all. No longer something the tree author has
            # to write.
            default_tokens = ["collision"] + (["battery"] if battery_enabled else [])

            triggers = "[" + ",".join(default_tokens + tagged_manual + derived_tokens) + "]"
            action_code = var_pool.next_action_code()

            # The FULL universe of untagged Reason shapes this leg can
            # possibly halt with -- see module/contracts/goal_formula_
            # check.py's own generate_safety_queries, which turns this
            # into query(any_reason_pattern_by_action(Pattern,
            # ActionCode)) facts for every problem automatically,
            # instead of anyone hand-picking which reasons are worth
            # asking about. "completed" (natural end, no trigger fired)
            # is always possible for every leg, regardless of Triggers.
            reason_patterns = ["completed"]
            reason_patterns += [_reason_pattern_for_manual_trigger(t) for t in default_tokens]
            reason_patterns += [_reason_pattern_for_manual_trigger(t) for t in manual_tokens
                                 if t.split("(", 1)[0].strip() not in ("collision", "battery")]
            # guard_break's own pattern embeds cond_term VERBATIM,
            # whatever text _reduce_guard_condition already built for
            # it -- unlike a functor-only grouping, this needs no
            # separate unwrapping step to tell two different guards
            # apart (guard_break(battery_over(70.0)) and guard_break
            # (neg(obstacle_in_bound(0.6))) are already two distinct
            # patterns here, not one shared "guard_break" bucket).
            # Scope caveat, stated plainly: this handles everything the
            # guard-derivation translator can actually produce today (a
            # bare condition or one neg(...) around it -- never and/or,
            # since _reduce_guard_condition never emits those). If
            # guard derivation is later extended to combine conditions,
            # this line still needs no change (it just embeds whatever
            # cond_term text exists), but that's a coincidence of using
            # the whole pattern rather than a functor -- not a
            # guarantee this scope note should be taken to extend to
            # every future consumer of cond_term.
            reason_patterns += [f"guard_break({cond_term})" for cond_term, _code in guard_stack
                                 if battery_enabled or not _guard_condition_mentions_battery(cond_term)]
            var_pool.reason_patterns_by_action[action_code] = reason_patterns
            var_pool.action_labels[action_code] = _with_branch_suffix(f"MoveTo({cp_var})", branch_name)

            return f"moveto_leg({cp_var},{triggers},{action_code})"

        if info["kind"] == "planWith":
            algorithm = attrs["algorithm"].strip()
            if algorithm not in _PLAN_ALGORITHMS:
                raise BTValidationError(
                    f"<{tag}>'s algorithm port ('{algorithm}') is not one of "
                    f"{sorted(_PLAN_ALGORITHMS)}.")

            cp_value = attrs["control_points"]
            if not _is_blackboard_ref(cp_value):
                raise BTValidationError(
                    f"<{tag}>'s control_points port ('{cp_value}') must be "
                    f"a blackboard reference like \"{{cp}}\" -- it is this "
                    f"node's own OUTPUT, never a literal.")
            key = _blackboard_key(cp_value)
            var_pool.producers.add(key)
            var_pool.note_key_scope(key, reactive_code)
            cp_var = var_pool.var_for(key)

            if algorithm == "follow_boarder":
                extraneous = {"goal"} & set(attrs)
                if extraneous:
                    raise BTValidationError(
                        f"<{tag}> has algorithm=\"follow_boarder\", which "
                        f"takes no goal port -- unexpected {sorted(extraneous)}.")
                if "obstacle_id" not in attrs or "offset" not in attrs:
                    raise BTValidationError(
                        f"<{tag}> has algorithm=\"follow_boarder\", which "
                        f"requires BOTH obstacle_id and offset.")
                # obstacle_id is written VERBATIM as Prolog text (a bare
                # atom), same convention as HaltedWith's own reason port
                # below -- NOT quoted, NOT blackboard-ref-checked
                # (nothing in this schema produces obstacle_id as its
                # own port yet; a future producer would need this
                # branch extended the same way control_points already
                # is).
                obstacle_id = attrs["obstacle_id"].strip()
                offset = float(attrs["offset"])
                algorithm_term = f"follow_boarder({obstacle_id},{offset})"
                # planWith/4's own Goal slot is part of the SHARED
                # template every algorithm sits inside (do_node(planWith
                # (Algorithm,Goal,CP,ActionCode),...) in basic_action_
                # theory.pl) -- follow_boarder itself has no goal point
                # (see this action's own schema.yaml note) and plan_call/
                # 8's own follow_boarder clauses ignore this slot
                # outright, so a placeholder point(0.0,0.0) is spliced
                # in here purely to satisfy that shared shape, never
                # read (the do_node clause that actually handles
                # follow_boarder reports the honest atom `none` as this
                # call's own Goal inside its RECORDED Reason instead --
                # see that predicate's own note).
                goal_term = "point(0.0,0.0)"
                reason_goal_text = "none"
            else:
                extraneous = {"obstacle_id", "offset"} & set(attrs)
                if extraneous:
                    raise BTValidationError(
                        f"<{tag}> has algorithm=\"{algorithm}\", which takes "
                        f"no obstacle_id/offset port -- unexpected "
                        f"{sorted(extraneous)}.")
                if "goal" not in attrs:
                    raise BTValidationError(
                        f"<{tag}> has algorithm=\"{algorithm}\", which "
                        f"requires a goal port.")
                algorithm_term = algorithm
                goal_term = _point_literal(attrs["goal"], tag, "goal")
                reason_goal_text = goal_term

            action_code = var_pool.next_action_code()
            # See do_node(planWith(...))'s own note in basic_action_
            # theory.pl for why ActionCode is fully concrete here too
            # (never 'wild'): unlike an argmin ObstacleId, ActionCode/
            # Algorithm/Goal are ALL already known at translation time
            # for a given PlanWith occurrence -- there's nothing
            # runtime-random about which occurrence this is, only
            # whether it actually succeeds, which the by-action query
            # already reports as a probability regardless.
            var_pool.reason_patterns_by_action[action_code] = [
                f"completed({algorithm_term},{reason_goal_text})",
                f"no_path({algorithm_term},{reason_goal_text})",
            ]
            plan_label = (f"PlanWith({algorithm_term})" if algorithm == "follow_boarder"
                          else f"PlanWith({algorithm_term}, goal={reason_goal_text})")
            var_pool.action_labels[action_code] = _with_branch_suffix(plan_label, branch_name)

            return f"planWith({algorithm_term},{goal_term},{cp_var},{action_code})"

    if tag in _CONDITION_DISPATCH:
        condition_code = var_pool.next_condition_code()
        # Generic label built straight from the schema id + its own bound
        # ports (e.g. "DistanceBelow(goal=2.275;2.075,threshold=0.3)") --
        # works for ANY condition tag automatically, no per-condition-type
        # special-casing, same spirit as _leaf_condition_term's own dispatch.
        port_text = ",".join(f"{k}={v}" for k, v in attrs.items() if k != "name")
        var_pool.condition_labels[condition_code] = _with_branch_suffix(
            f"{tag}({port_text})", branch_name)
        return f"cond({_leaf_condition_term(tag, attrs)},{condition_code})"

    raise BTValidationError(f"Unhandled schema entry '{tag}' -- add it to "
                             f"_ACTION_DISPATCH/_CONDITION_DISPATCH.")


def _translate_node(elem, schema_ports, var_pool, battery_enabled, reactive_code, guard_stack, branch_name=None):
    tag = elem.tag

    if tag == "Inverter":
        # BT.cpp's built-in single-child negation decorator ->
        # inverter(ChildTerm) (see basic_action_theory.pl's own
        # do_node(inverter(Child),...) clauses). Like plain Sequence/
        # Fallback -- NOT a new reactive scope, and it never itself
        # contributes a guard -- reactive_code and guard_stack pass
        # through to its one child UNCHANGED. This is a SEPARATE code
        # path from _reduce_guard_condition's own <Inverter> handling
        # (which unwraps one around a left-sibling CONDITION into a
        # polarity flip, for guard derivation specifically); this one
        # instead produces a REAL do_node term for <Inverter> appearing
        # anywhere else in the tree (wrapping an action, a whole
        # branch, another composite, ...). The two coexist without
        # conflict: guard derivation only ever calls the other one, on
        # left siblings, before this function ever sees them.
        if elem.attrib.keys() - _ALWAYS_ALLOWED_ATTRS:
            raise BTValidationError(
                f"<Inverter> takes no ports of its own (it's a BT.cpp "
                f"built-in decorator node) -- unexpected attribute(s) "
                f"{sorted(elem.attrib.keys() - _ALWAYS_ALLOWED_ATTRS)}.")
        children = list(elem)
        if len(children) != 1:
            raise BTValidationError(
                f"<Inverter> must have exactly one child (found "
                f"{len(children)}).")
        child_term = _translate_node(children[0], schema_ports, var_pool, battery_enabled,
                                      reactive_code, guard_stack, branch_name)
        return f"inverter({child_term})"

    if tag in _CONTROL_FLOW:
        if elem.attrib.keys() - _ALWAYS_ALLOWED_ATTRS:
            raise BTValidationError(
                f"<{tag}> takes no ports of its own (it's a BT.cpp "
                f"built-in control node) -- unexpected attribute(s) "
                f"{sorted(elem.attrib.keys() - _ALWAYS_ALLOWED_ATTRS)}.")
        children = list(elem)
        if not children:
            raise BTValidationError(f"<{tag}> has no children.")
        # Plain Sequence/Fallback -- pass the CURRENT reactive_code AND
        # guard_stack through UNCHANGED. They never catch/redescend on
        # their own, so they don't start a new reactive scope (see
        # basic_action_theory.pl's own CONTROL-FLOW REDESCEND TARGETS
        # note); their own left siblings are ONE-SHOT (checked once on
        # descent, via the cond() already sitting in the tree at that
        # position) rather than live guards, so they contribute nothing
        # to guard_stack either -- see the module docstring's CONTROL-
        # FLOW GUARD DERIVATION note. own_branch_name: THIS node's own
        # name="..." attribute if it has one, else whatever named
        # ancestor was already in scope -- see _with_branch_suffix's
        # own note; purely cosmetic (main.py's action-code legend),
        # never affects the generated Prolog term.
        own_branch_name = elem.attrib.get("name", branch_name)
        child_terms = [_translate_node(c, schema_ports, var_pool, battery_enabled, reactive_code,
                                        guard_stack, own_branch_name)
                       for c in children]
        functor = _CONTROL_FLOW[tag]
        return f"{functor}([{','.join(child_terms)}])"

    if tag in _REACTIVE_CONTROL_FLOW:
        if elem.attrib.keys() - _ALWAYS_ALLOWED_ATTRS:
            raise BTValidationError(
                f"<{tag}> takes no ports of its own (it's a BT.cpp "
                f"built-in control node) -- unexpected attribute(s) "
                f"{sorted(elem.attrib.keys() - _ALWAYS_ALLOWED_ATTRS)}.")
        children = list(elem)
        if not children:
            raise BTValidationError(f"<{tag}> has no children.")
        own_branch_name = elem.attrib.get("name", branch_name)
        # A NEW reactive scope starts here -- fresh code, and every
        # descendant (until a NESTED ReactiveSequence/ReactiveFallback
        # starts its own) gets tagged with THIS one. Children are
        # translated and then factored OUT into their own
        # reactive_children/2 fact (see _VarPool's own note) rather
        # than inlined -- the returned term references only the code.
        #
        # ALSO: this is exactly where automatic guard derivation
        # happens (see the module docstring's CONTROL-FLOW GUARD
        # DERIVATION note). required_polarity is this level's own
        # baseline: a ReactiveSequence's left siblings must all
        # SUCCEED (True), a ReactiveFallback's must all FAIL (False).
        # For each child index i, every EARLIER sibling (index < i) is
        # reduced to a guard condition and appended to a FRESH
        # guard_stack used ONLY for translating child i -- siblings
        # AFTER i are never guards on it (Step 1), and each child's own
        # guard entries are tagged with THIS level's own_code, added on
        # top of (not replacing) whatever guard_stack already carried
        # in from enclosing levels, so a MoveTo nested under two
        # reactive ancestors accumulates guards -- and codes -- from
        # BOTH. A sibling that _reduce_guard_condition can't turn into
        # a live guard (e.g. an ordinary Action like PlanWith --
        # see its own note) returns None and is simply skipped, not an
        # error: most Sequence/ReactiveSequence children are actions,
        # not conditions, and that's completely normal.
        own_code = var_pool.next_reactive_code()
        required_polarity = (tag == "ReactiveSequence")
        child_terms = []
        for i, child in enumerate(children):
            own_level_guards = []
            for sibling in children[:i]:
                cond_term = _reduce_guard_condition(sibling, required_polarity, schema_ports)
                if cond_term is not None:
                    own_level_guards.append((cond_term, own_code))
            child_terms.append(_translate_node(
                child, schema_ports, var_pool, battery_enabled, own_code,
                guard_stack + own_level_guards, own_branch_name))
        var_pool.reactive_facts.append((own_code, child_terms))
        functor = _REACTIVE_CONTROL_FLOW[tag]
        return f"{functor}({own_code})"

    if tag not in schema_ports:
        raise BTValidationError(
            f"<{tag}> is not a recognized node -- not Sequence/Fallback/"
            f"ReactiveSequence/ReactiveFallback/Inverter and not an "
            f"action/condition 'id' in module/contracts/schema.yaml.")

    return _translate_leaf(tag, elem, None, schema_ports[tag], var_pool, battery_enabled, reactive_code, guard_stack, branch_name)


def _find_tree_root(xml_root):
    bt_elems = xml_root.findall("BehaviorTree")
    if not bt_elems:
        raise BTValidationError("No <BehaviorTree> element found under <root>.")
    main_id = xml_root.attrib.get("main_tree_to_execute")
    chosen = None
    if main_id:
        for bt in bt_elems:
            if bt.attrib.get("ID") == main_id:
                chosen = bt
                break
        if chosen is None:
            raise BTValidationError(
                f"<root main_tree_to_execute=\"{main_id}\"> but no "
                f"<BehaviorTree ID=\"{main_id}\"> exists.")
    else:
        if len(bt_elems) > 1:
            raise BTValidationError(
                "Multiple <BehaviorTree> elements but no "
                "main_tree_to_execute attribute on <root> to disambiguate.")
        chosen = bt_elems[0]
    children = list(chosen)
    if len(children) != 1:
        raise BTValidationError(
            f"<BehaviorTree ID=\"{chosen.attrib.get('ID')}\"> must have "
            f"EXACTLY ONE root child node (found {len(children)}).")
    return children[0]


def translate_tree(xml_path=DEFAULT_XML_PATH, schema_path=DEFAULT_SCHEMA_PATH,
                    battery_enabled=True):
    """Parse + validate + translate the BT XML into ONE Prolog term
    (Node's own text, e.g. "seq_node([planWith(...),moveto_leg(...)])")
    PLUS the separate reactive_children/2 facts any ReactiveSequence/
    ReactiveFallback in the tree needs, PLUS the full per-action Reason
    universe for safety-query generation, PLUS a human-readable label
    per action code AND per condition code -- returns (node_text,
    reactive_facts, reason_patterns_by_action, action_labels,
    condition_labels), where reactive_facts is [(code, [child_term,...]),
    ...], reason_patterns_by_action is {action_code: [reason_pattern_
    text, ...]} (see _VarPool's own note and module/contracts/goal_
    formula_check.py's generate_safety_queries, the actual consumer),
    action_labels is {action_code: label_text} and condition_labels is
    {condition_code: label_text} (see _VarPool.action_labels/
    condition_labels's own notes -- main.py prints these as small
    "which action/condition has which code" legends, purely cosmetic,
    no Prolog-side consumer; condition_labels' own codes are ALSO what
    generate_safety_queries uses to emit one any_condition_status(Code,
    true)/(Code,false) query pair per condition occurrence). Raises
    BTValidationError on any structural problem.

    battery_enabled=False strips every battery-related trigger name
    (battery, battery_below(...), battery_over(...), battery_equal(...)
    -- see _is_battery_trigger's own note) out of every MoveTo's own
    Triggers list -- the problem's own config.yaml battery.enabled
    flag, threaded in by generate_plan_pl's caller (main.py/
    diagnose_pipeline.py)."""
    schema = load_schema(schema_path)
    schema_ports = _schema_port_index(schema)

    try:
        xml_root = ET.parse(xml_path).getroot()
    except ET.ParseError as e:
        raise BTValidationError(f"Malformed XML in {xml_path}: {e}")

    tree_root_elem = _find_tree_root(xml_root)
    var_pool = _VarPool()
    node_text = _translate_node(tree_root_elem, schema_ports, var_pool, battery_enabled, None, [])

    # A MoveTo whose control_points key has no PlanWith
    # producer anywhere in the tree would silently leave CP unbound --
    # catch it here rather than let ProbLog fail confusingly later.
    dangling = var_pool.consumers - var_pool.producers
    if dangling:
        raise BTValidationError(
            f"control_points blackboard key(s) {sorted(dangling)} are read "
            f"by a MoveTo node but never produced by any PlanWith "
            f"node in the tree -- CP would be unbound.")

    # A blackboard key produced/consumed under more than one reactive
    # scope (including "outside any ReactiveSequence/ReactiveFallback",
    # recorded as None) would straddle a reactive_children/2 boundary --
    # see _VarPool's own note on why that silently breaks the same way
    # reusing one CP across two fallback_node branches already does.
    straddling = {key: scopes for key, scopes in var_pool.key_scopes.items()
                  if len(scopes) > 1}
    if straddling:
        def _label(scope):
            return scope or "outside any reactive composite"
        details = "; ".join(
            f"'{key}' touched under {sorted(_label(s) for s in scopes)}"
            for key, scopes in straddling.items())
        raise BTValidationError(
            f"control_points blackboard key(s) cross a ReactiveSequence/"
            f"ReactiveFallback boundary -- a producer/consumer pair must "
            f"live ENTIRELY inside the same reactive composite (or entirely "
            f"outside all of them): {details}.")

    return (node_text, var_pool.reactive_facts, var_pool.reason_patterns_by_action,
            var_pool.action_labels, var_pool.condition_labels)


def generate_plan_pl(xml_path=DEFAULT_XML_PATH, schema_path=DEFAULT_SCHEMA_PATH,
                      output_path=DEFAULT_OUTPUT_PATH, battery_enabled=True):
    """Returns (output_path, reason_patterns_by_action, action_labels,
    condition_labels) -- reason_patterns_by_action AND condition_labels'
    own keys are both what main.py hands to module/contracts/goal_
    formula_check.py's generate_safety_queries right after goal_formula
    .pl's own validation, to build this problem's own queries_generated
    .pl; action_labels/condition_labels ({code: label_text} each) are
    what main.py prints as its own action-/condition-code legends -- see
    translate_tree's own note for all four."""
    node_text, reactive_facts, reason_patterns_by_action, action_labels, condition_labels = translate_tree(
        xml_path, schema_path, battery_enabled=battery_enabled)
    lines = [
        "% AUTO-GENERATED by module/translators/bt_to_prolog.py from",
        f"% {os.path.relpath(xml_path, os.path.dirname(output_path))} -- DO NOT HAND-EDIT,",
        "% edit the XML tree instead and regenerate (main.py does this",
        "% automatically before every run).",
    ]
    if not battery_enabled:
        lines += [
            "%",
            "% battery.enabled: false in this problem's own config.yaml --",
            "% every battery-related trigger name has been stripped from",
            "% every leg's own Triggers list below (see bt_to_prolog.py's",
            "% own _is_battery_trigger).",
        ]
    if action_labels:
        lines += ["%", "% Action code legend (see main.py's own printed copy of this):"]
        lines += [f"%   {code}: {action_labels[code]}" for code in sorted(action_labels)]
    if condition_labels:
        lines += ["%", "% Condition code legend (see main.py's own printed copy of this):"]
        lines += [f"%   {code}: {condition_labels[code]}" for code in sorted(condition_labels)]
    lines += [
        "",
        f"plan({node_text}).",
        "",
    ]
    if reactive_facts:
        lines += [
            "% One reactive_children/2 fact per <ReactiveSequence>/",
            "% <ReactiveFallback> in the tree, keyed by the same code",
            "% embedded in plan/1's own reactivesequence(Code)/",
            "% reactivefallback(Code) markers above -- see basic_action_",
            "% theory.pl's own CONTROL-FLOW REDESCEND TARGETS note for why",
            "% these live as SEPARATE facts rather than being inlined.",
            "",
        ]
        for code, child_terms in reactive_facts:
            lines.append(f"reactive_children({code}, [{','.join(child_terms)}]).")
        lines.append("")
    with open(output_path, "w") as f:
        f.write("\n".join(lines))
    return output_path, reason_patterns_by_action, action_labels, condition_labels


if __name__ == "__main__":
    out, _reason_patterns_by_action, _action_labels, _condition_labels = generate_plan_pl()
    print(f"Wrote {out}")
