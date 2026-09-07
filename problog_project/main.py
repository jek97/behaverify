#!/usr/bin/env python3
"""
main.py

Runs the continuous-time / continuous-space single-moveto() ProbLog
action theory (module/theory/basic_action_theory.pl) against one
problem (problems/<name>/, default "problem0") and produces a full
safety report, analogous in spirit to run_plan_weave_safety.py but
re-indexed from "discrete grid step N" to "sampled instant I along the
one continuous walk", and from "grid obstacle cells" to "obstacle
polygons" (as produced by module/translators/occgrid_to_problog.py).

Prints a COMPACT summary (the problem's own goal_formula.pl, plus TWO
probability tables) rather than the earlier verbose per-sample report.
basic_action_theory.pl's own Section 10 no longer hardcodes a QUERIES
list at all -- module/contracts/goal_formula_check.py's
generate_safety_queries writes this problem's own queries_generated.pl
automatically, right after goal_formula.pl's own validation (see that
module's own header), from the per-action Reason universe module/
translators/bt_to_prolog.py worked out while translating behavior_
tree.xml. Table 1 (print_compact_summary) is the five ALWAYS-relevant,
tree-shape-independent queries:
  - verify_goal_formula: P(the problem's own goal_formula.pl holds at
    the final situation)
  - plan_outcome(true) / plan_outcome(false) / plan_outcome
    (world_too_large): the BT's own three possible outcomes
  - plan_outcome(reactive_escaped): safety net for the localized
    reactive-redescend mechanism (reactivesequence(Code)/
    reactivefallback(Code) in basic_action_theory.pl) -- a `reactive(_)`
    status escaping all the way to the root is always a translator bug,
    so this should read 0.00% on every problem; a nonzero reading here
    means some reactive-classified trigger's code has no matching
    enclosing reactivesequence/reactivefallback in the tree.
Table 2 (print_reason_breakdown) is the auto-generated, per-action
breakdown of EVERY OTHER Reason this problem's own tree can actually
produce (the old hand-picked any_collision/any_battery_depletion are
now just two rows of this table, generated the same way as every other
Reason instead of being hand-maintained specially) -- e.g. a leg-a1/
leg-a2 tree with 30% total collision probability shows "crashed 30%"
with "a1 20.00%" / "a2 10.00%" indented beneath it.

hit_by/1, first_hit/1, on_track/1, verify_safe/0, and plan_route_
blocked/0 are all still DEFINED in basic_action_theory.pl -- only their
query(...) declarations (and this script's own per-sample report
formatting) were removed, not the underlying predicates. Re-add
whichever query(...) line(s) are wanted again, and a matching report
section, to bring per-sample hazard/drift reporting back.

Usage:
    python3 main.py [--problem NAME]
        (default NAME: problem0 -- see problems/problem0/ for its
        config.yaml, behavior_tree.xml, goal_formula.pl, and map.yaml)

Before running inference, this script, for the SELECTED problem:
  1. regenerates <problem>/obstacles_generated.pl from
     <problem>/map.yaml (see module/translators/occgrid_to_problog.py's
     own header) -- map.yaml is the single source of truth for the
     obstacle layout.
  2. regenerates <problem>/config_generated.pl from <problem>/config.yaml
     (see module/translators/config_to_prolog.py's own header) --
     config.yaml is the single source of truth for every tunable
     constant in the theory (noise sigmas, the Z discretization
     tables, battery drain rates, robot/safety thresholds, tolerances,
     verification resolution, and the robot's own starting position).
  3. translates <problem>/behavior_tree.xml -- a real BT.cpp v4 tree,
     the single source of truth for the POLICY'S SHAPE -- into
     <problem>/plan_generated.pl, validating it against
     module/contracts/schema.yaml on the way (see module/translators/
     bt_to_prolog.py's own header).
  4. validates <problem>/goal_formula.pl -- the hand-authored
     verification goal for THIS particular plan, and the ONLY place
     goal information lives in this theory -- against module/contracts/
     vocabulary.yaml (see module/contracts/goal_formula_check.py's own
     header): every predicate it calls must be a known fluent, and the
     whole formula must be uniform in one situation (Reiter's own
     sense).
  5. (re)writes module/theory/problem_data.pl, a small bootstrap file
     basic_action_theory.pl itself consults (see that file's Section 0)
     that points -- via absolute paths -- at the four files above, so
     the SAME theory file serves whichever problem was selected. Also
     sets the BT_PROBLEM_DIR environment variable to the selected
     problem's own directory, which module/theory/planners.py and
     module/theory/collision_geometry.py (Python black boxes ProbLog
     loads directly) read at import time for the same reason.
All five steps mean a normal run always reflects whatever is currently
in the selected problem's own map.yaml / config.yaml / behavior_tree.xml
/ goal_formula.pl, with no separate regeneration step needed.

Requires: `problog` importable/runnable on PATH.

Saves a timestamped log file with a compact query-result summary --
no image/plot is produced (see print_compact_summary).
"""

import argparse
import os
import re
import shutil
import sys
from datetime import datetime

from problog.errors import ProbLogError

from pipeline_stages import run_staged_inference, write_problem_data_pl, StageTimeout

W = 68

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MODULE_DIR = os.path.join(_THIS_DIR, "module")
THEORY_DIR = os.path.join(MODULE_DIR, "theory")
TRANSLATORS_DIR = os.path.join(MODULE_DIR, "translators")
CONTRACTS_DIR = os.path.join(MODULE_DIR, "contracts")
PROBLEMS_DIR = os.path.join(_THIS_DIR, "problems")
THEORY_PATH = os.path.join(THEORY_DIR, "basic_action_theory.pl")
OUTPUT_DIR = os.path.join(_THIS_DIR, "output")


# -----------------------------------------------------------------------
# Run ProbLog via its PYTHON API (not the CLI/subprocess) and return a
# results dict directly. str(term) for a query like plan_outcome(true)
# is EXACTLY the same string PARSE_RESULTS used to extract from CLI
# text output -- so everything downstream (print_compact_summary)
# needs zero changes; only how the numbers get INTO the dict changes.
#
# Delegates to pipeline_stages.run_staged_inference for the actual
# parse/ground/compile/evaluate work -- see that module's own header
# for why this is staged (one [STAGE] log line + timeout per phase)
# rather than the single opaque call this function used to make
# directly. elapsed here is the SUM of the four stage timings, kept
# for the "Finished: ...(elapsed)" line further down in main(); the
# per-stage breakdown is already visible in the [STAGE] lines
# run_staged_inference logs as it goes.
# -----------------------------------------------------------------------
def run_problog_api(plan_file, tee, phase_timeout=300):
    results, timings = run_staged_inference(plan_file, tee, phase_timeout=phase_timeout)
    return results, sum(timings.values())


# -----------------------------------------------------------------------
# Report printing
# -----------------------------------------------------------------------
class Tee:
    def __init__(self, log_fh):
        self._log = log_fh
    def __call__(self, text=""):
        print(text)
        self._log.write(text + "\n")
        self._log.flush()


def banner(tee, text, char="="):
    tee(char * W); tee(f"  {text}"); tee(char * W)

def section(tee, text):
    tee(f"\n{'-'*W}"); tee(f"  {text}"); tee(f"{'-'*W}")

def extract_goal_formula_text(goal_formula_path):
    """The problem's own goal_formula.pl, stripped down to its actual
    clause(s) -- drops '%'-prefixed comment lines and blank lines, so
    the compact report can show WHAT was actually verified without
    dumping that file's own (often long) header comment. Returns the
    remaining lines joined with '\n', or a placeholder if the file
    turns out to be all comments/blank (shouldn't happen in practice,
    but this is report formatting, not validation -- goal_formula_check
    .py already validated the real file earlier in this same run)."""
    with open(goal_formula_path) as f:
        lines = [ln.rstrip() for ln in f
                 if ln.strip() and not ln.strip().startswith("%")]
    return "\n".join(lines) if lines else "(no clause found)"


# The five ALWAYS-relevant, tree-shape-independent queries -- kept in
# sync BY CONSTRUCTION with module/contracts/goal_formula_check.py's
# own generate_safety_queries (_ALWAYS_QUERIES there), not by hand:
# every OTHER safety query (any_reason_pattern/1, any_reason_pattern_
# by_action/2 -- Table 2, see print_reason_breakdown below) is derived
# automatically from the problem's own behavior_tree.xml instead of
# being a fixed list anyone maintains here.
SUMMARY_QUERIES = [
    "verify_goal_formula",
    "plan_outcome(true)",
    "plan_outcome(false)",
    "plan_outcome(world_too_large)",
    "plan_outcome(reactive_escaped)",
]


def print_action_legend(tee, action_labels):
    """Small "which action has which code" table -- action_labels
    ({action_code: label_text}) comes straight from bt_to_prolog.py's
    own generate_plan_pl (see that function's own note); label_text
    already carries the action's own kind/args (e.g.
    "PlanWith(straight, goal=point(22.275,2.075))") and, when it sits
    under a named Sequence/Fallback/ReactiveSequence/ReactiveFallback,
    that ancestor's own name in brackets (e.g. "[TryGoal]") -- printed
    here so a code like a3 in Table 2 below is traceable back to WHERE
    in the tree it came from without re-reading plan_generated.pl by
    hand. Skipped entirely (no section header either) if this problem's
    plan wasn't translated from behavior_tree.xml at all (e.g.
    diagnose_pipeline.py's own hand-written-plan path never calls this)."""
    if not action_labels:
        return
    section(tee, "Action codes")
    code_w = max(len(code) for code in action_labels)
    for code in sorted(action_labels):
        tee(f"  {code:<{code_w}}   {action_labels[code]}")


def print_condition_legend(tee, condition_labels):
    """Small "which condition has which code" table -- the direct
    analogue of print_action_legend above, for cond(C,Code)'s own Code
    (see basic_action_theory.pl's own checked(Code,C,Status) marker
    note). condition_labels ({condition_code: label_text}) comes
    straight from bt_to_prolog.py's own generate_plan_pl; label_text is
    built generically from the schema id + its own bound ports (e.g.
    "DistanceBelow(goal=2.275;2.075,threshold=0.3)"), plus a branch
    suffix same as action labels. Skipped entirely if this problem's
    plan wasn't translated from behavior_tree.xml at all."""
    if not condition_labels:
        return
    section(tee, "Condition codes")
    code_w = max(len(code) for code in condition_labels)
    for code in sorted(condition_labels):
        tee(f"  {code:<{code_w}}   {condition_labels[code]}")


_CONDITION_STATUS_RE = re.compile(r"^any_condition_status\((\w+),(true|false)\)$")


def print_condition_breakdown(tee, results, condition_labels):
    """Table: P(true)/P(false) per condition occurrence, flattened from
    every any_condition_status(Code,true/false) query result (see
    basic_action_theory.pl's own any_condition_status/2) -- the direct
    analogue of print_reason_breakdown above, but for CONDITIONS rather
    than action Reasons. A code whose own true+false falls SHORT of
    100% means its own cond() leaf wasn't reached in every world (e.g.
    it sits in a Fallback branch that isn't always tried) -- shown as
    its own "never reached" remainder rather than silently hidden, same
    "absence is informative" posture as the rest of this report. A code
    whose own true+false EXCEEDS 100% means the opposite problem: this
    cond() leaf sits somewhere that gets CHECKED MORE THAN ONCE per
    world on average -- a reactive guard condition (e.g. BatteryOver as
    a ReactiveSequence's own left sibling) that a redescend loop
    re-evaluates fresh on every restart is the typical cause; true and
    false are then independent per-CHECK rates, not a mutually
    exclusive partition of "what happened to this world" -- flagged
    explicitly rather than printing a nonsensical negative remainder."""
    by_code = {}   # condition_code -> {"true": prob, "false": prob}
    for key, prob in results.items():
        m = _CONDITION_STATUS_RE.match(key)
        if not m:
            continue
        code, status = m.groups()
        by_code.setdefault(code, {})[status] = prob
    if not by_code:
        return
    section(tee, "Condition check breakdown")
    for code in sorted(by_code):
        label = condition_labels.get(code, code)
        p_true = by_code[code].get("true", 0.0)
        p_false = by_code[code].get("false", 0.0)
        tee(f"  {code}  {label}")
        tee(f"    {'true':<40} {p_true*100:6.2f}%")
        tee(f"    {'false':<40} {p_false*100:6.2f}%")
        total = p_true + p_false
        if total < 0.9995:
            tee(f"    {'(never reached)':<40} {(1.0 - total)*100:6.2f}%")
        elif total > 1.0005:
            tee(f"    (checked more than once per world on average -- "
                f"true/false are independent per-check rates, not a partition)")


def print_compact_summary(tee, results, goal_formula_path, action_labels=None, condition_labels=None):
    section(tee, "Goal formula")
    tee(f"  {goal_formula_path}")
    for line in extract_goal_formula_text(goal_formula_path).split("\n"):
        tee(f"    {line}")

    print_action_legend(tee, action_labels or {})
    print_condition_legend(tee, condition_labels or {})

    section(tee, "Query results")
    label_w = max(len(name) for name in SUMMARY_QUERIES)
    tee(f"  {'Query':<{label_w}}   Probability")
    tee(f"  {'-'*label_w}   -----------")
    for name in SUMMARY_QUERIES:
        if name not in results:
            tee(f"  {name:<{label_w}}   N/A (not queried)")
            continue
        p = results[name]
        tee(f"  {name:<{label_w}}   {p*100:6.2f}%")

    print_reason_breakdown(tee, results)
    print_condition_breakdown(tee, results, condition_labels or {})
    print_outcome_enumeration(tee, results)


def _split_top_level(text, seps=","):
    """Split text on any character in `seps` that sits at bracket depth
    0 -- tracks BOTH ()/[] (unlike _split_last_top_level_arg below,
    which only ever needs to track () for a Pattern's own nested
    compound terms; outcome_signature/1's own list entries can nest a
    compound term like point(11.675,11.525) inside a list, so both
    bracket kinds matter here). Each returned piece is stripped -- see
    print_outcome_enumeration's own note on why ProbLog renders a
    list's own top-level commas WITH a trailing space (unlike a
    compound term's own commas, always dense) -- verified directly
    against ProbLog's own term-to-string output before relying on it."""
    parts = []
    depth = 0
    start = 0
    for i, ch in enumerate(text):
        if ch in "([":
            depth += 1
        elif ch in ")]":
            depth -= 1
        elif ch in seps and depth == 0:
            parts.append(text[start:i])
            start = i + 1
    parts.append(text[start:])
    return [p.strip() for p in parts]


def _split_code_value(entry_text):
    """Split one outcome_signature/1 list entry (e.g.
    "a1-completed(astar,point(11.675,11.525))") into (Code, Value) on
    its own FIRST top-level '-' -- Code is always a bare atom (a1, c3,
    ...) coming from next_action_code()/next_condition_code(), which
    never themselves contain '(' or '-', so the first top-level '-' is
    always the right split point regardless of what Value looks like."""
    depth = 0
    for i, ch in enumerate(entry_text):
        if ch in "([":
            depth += 1
        elif ch in ")]":
            depth -= 1
        elif ch == "-" and depth == 0:
            return entry_text[:i], entry_text[i + 1:]
    raise ValueError(f"no top-level '-' found in {entry_text!r}")


_OUTCOME_SIGNATURE_RE = re.compile(r"^outcome_signature\(\[(.*)\]\)$")


def print_outcome_enumeration(tee, results):
    """Table: every DISTINCT combination of Reason/Condition values
    actually reached across all resolved worlds, most probable first --
    see basic_action_theory.pl's own outcome_signature/1 for how each
    combination (a list of Code-Value pairs, one per action Reason and
    per condition check reached in that world) is derived, and why
    query(outcome_signature(_)) is deliberately non-ground (ProbLog
    reports one result row per distinct grounding rather than
    aggregating -- exactly the wanted behavior here, same mechanism
    already used for the per-obstacle detail rows in print_reason_
    breakdown above). This is the JOINT distribution over every tracked
    Reason/Condition together; Table 2/3 above are its own MARGINALS
    (summed over everything else) -- probabilities across rows here sum
    to 100% (every resolved world produces exactly one signature),
    unlike Table 2/3's own rows, which can each fall short of 100% when
    a given action/condition wasn't reached in every world."""
    rows = []   # (probability, [(code, value_text), ...])
    for key, prob in results.items():
        m = _OUTCOME_SIGNATURE_RE.match(key)
        if not m:
            continue
        inner = m.group(1)
        entries = [_split_code_value(e) for e in _split_top_level(inner)] if inner else []
        rows.append((prob, entries))
    if not rows:
        return
    section(tee, "Full outcome enumeration (every Reason/Condition combination)")
    for prob, entries in sorted(rows, key=lambda row: -row[0]):
        tee(f"  {prob*100:6.2f}%")
        for code, value in entries:
            tee(f"    {code}: {_display_pattern(value)}")


def _split_last_top_level_arg(inner_text):
    """Given the text INSIDE a 2-arg call's own parens (e.g.
    "guard_break(battery_over(70.0)),a1"), split off the LAST
    top-level argument (ActionCode) from everything before it
    (the Pattern), respecting nested parens -- a naive split(",") would
    break on a Pattern that itself contains commas, e.g.
    line_of_sight_clear(obs1,11.675,11.525)."""
    depth = 0
    for i in range(len(inner_text) - 1, -1, -1):
        ch = inner_text[i]
        if ch == ")":
            depth += 1
        elif ch == "(":
            depth -= 1
        elif ch == "," and depth == 0:
            return inner_text[:i], inner_text[i + 1:]
    raise ValueError(f"no top-level comma found in {inner_text!r}")


def _display_pattern(pattern_text):
    """Cosmetic-only: swap the literal 'wild' marker atom (see
    basic_action_theory.pl's halted_with_pattern/3 and match_wild/2) for
    a more readable '*' wherever a Pattern is printed. Never touches the
    actual Prolog query text, only what gets shown in a label."""
    return re.sub(r"\bwild\b", "*", pattern_text)


def print_reason_breakdown(tee, results):
    """Table 2: every any_reason_pattern_by_action(Pattern,ActionCode)
    result, grouped by Pattern's own outer functor. A functor with only
    ONE distinct Pattern anywhere in the tree (the common case --
    "crashed", "battery_depleted", ...) gets a clean bare-functor
    header; a functor with SEVERAL distinct Patterns (e.g. guard_break
    firing on two genuinely different underlying conditions) gets each
    Pattern its OWN, fully-distinguishing header instead of silently
    merging them into one number that would hide which guard actually
    broke -- see basic_action_theory.pl's own halted_with_pattern/3 and
    match_wild/2 for why the underlying query is exact-pattern-based,
    not merely functor-based, in the first place.

    Beneath EACH (Pattern, ActionCode) row, further nests every
    any_reason_pattern_detail_by_action result sharing that same
    functor+action -- the per-concrete-configuration breakdown (e.g.
    WHICH obstacle, under "crashed"/a1's own 20%) generate_safety_
    queries only emits a detail query for when the aggregate Pattern
    actually contains 'wild' somewhere (see basic_action_theory.pl's
    own halted_with_pattern_detail/3) -- a Pattern with nothing
    runtime-only in it (battery_under(20), completed, ...) simply has
    no detail rows to nest, and none are printed for it. Matched by
    (functor, action_code) rather than the exact Pattern text -- exact
    for every problem this project currently has (each functor+action
    combination has exactly one Pattern in practice); a tree where the
    SAME functor produced two genuinely different Patterns for the
    SAME action would see both patterns' detail rows nested together,
    since there is no query-level ambiguity to resolve that finely
    without a real per-pattern detail key -- not a concern for any
    tree this project can currently generate."""
    by_pattern = {}   # pattern_text -> {action_code: probability}
    by_functor_action_detail = {}   # (functor, action_code) -> {detail_pattern_text: probability}

    agg_prefix = "any_reason_pattern_by_action("
    detail_prefix = "any_reason_pattern_detail_by_action("
    for key, prob in results.items():
        if key.startswith(agg_prefix) and key.endswith(")"):
            inner = key[len(agg_prefix):-1]
            pattern_text, action_code = _split_last_top_level_arg(inner)
            by_pattern.setdefault(pattern_text, {})[action_code] = prob
        elif key.startswith(detail_prefix) and key.endswith(")"):
            inner = key[len(detail_prefix):-1]
            detail_pattern_text, action_code = _split_last_top_level_arg(inner)
            functor = detail_pattern_text.split("(", 1)[0].strip()
            by_functor_action_detail.setdefault((functor, action_code), {})[detail_pattern_text] = prob

    if not by_pattern:
        return

    by_functor = {}
    for pattern_text in by_pattern:
        functor = pattern_text.split("(", 1)[0].strip()
        by_functor.setdefault(functor, []).append(pattern_text)

    section(tee, "Safety query breakdown (per action)")
    for functor in sorted(by_functor):
        for pattern_text in sorted(by_functor[functor]):
            # Only collapse to the bare functor when this Pattern has a
            # 'wild' marker in it -- i.e. there IS a lower detail level
            # (any_reason_pattern_detail_by_action) that still shows the
            # specifics one level down. A Pattern with no 'wild' (e.g.
            # guard_break(battery_over(70.0)), battery_under(20)) has no
            # such fallback, so its whole identity would be lost by
            # collapsing to the functor -- always show it in full.
            has_wild = "wild" in pattern_text
            label = (functor if (has_wild and len(by_functor[functor]) == 1)
                     else _display_pattern(pattern_text))
            per_action = by_pattern[pattern_text]
            total = sum(per_action.values())
            tee(f"  {label:<40} {total*100:6.2f}%")
            for action_code in sorted(per_action):
                tee(f"    {action_code:<38} {per_action[action_code]*100:6.2f}%")
                details = by_functor_action_detail.get((functor, action_code))
                if not details:
                    continue
                # A detail pattern that never actually resolved in ANY
                # world (probability 0) leaves ProbLog's own internal
                # placeholder variable name in its argument (e.g.
                # "crashed(X2)") instead of a real obstacle -- filter
                # these out, they carry no diagnostic information (the
                # 0.00% is already fully represented by the per-action
                # row just above, which is never itself omitted).
                for detail_pattern_text in sorted(details):
                    if details[detail_pattern_text] <= 0.0:
                        continue
                    tee(f"      {detail_pattern_text:<36} {details[detail_pattern_text]*100:6.2f}%")


# -----------------------------------------------------------------------
# main
# -----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--problem", default="problem0",
                     help="Name of the problem to run -- a subdirectory "
                          "of problems/ holding this problem's own "
                          "config.yaml, behavior_tree.xml, "
                          "goal_formula.pl, and map.yaml (default: "
                          "problem0).")
    ap.add_argument("--phase-timeout", type=int, default=300,
                     help="Per-stage timeout in seconds for the ProbLog "
                          "resolution pipeline -- parse/ground/compile/"
                          "evaluate each get their own budget (default: "
                          "300 = 5 minutes). See pipeline_stages.py.")
    args = ap.parse_args()

    problem_dir = os.path.join(PROBLEMS_DIR, args.problem)

    # Each run's own report lives in output/<problem>/, wiped and
    # recreated fresh every time -- this is a REPORT of the run, not an
    # input any other file depends on, so there's no reason to keep
    # stale runs around the way problems/<name>/'s own generated
    # Prolog facts are (those get committed; this doesn't -- see
    # .gitignore).
    problem_output_dir = os.path.join(OUTPUT_DIR, args.problem)
    if os.path.isdir(problem_output_dir):
        shutil.rmtree(problem_output_dir)
    os.makedirs(problem_output_dir)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(problem_output_dir, f"{args.problem}_{ts}.log")

    with open(log_path, "w", encoding="utf-8") as fh:
        tee = Tee(fh)
        banner(tee, f"ProbLog Continuous-Space Safety Verification - {datetime.now():%Y-%m-%d %H:%M:%S}")
        tee(f"  Log file    : {log_path}")
        tee(f"  Problem     : {args.problem} ({problem_dir})")
        tee(f"  Theory file : {THEORY_PATH}")

        if not os.path.isdir(problem_dir):
            tee(f"\n  [ERROR] No such problem directory: {problem_dir}")
            sys.exit(1)
        if not os.path.isfile(THEORY_PATH):
            tee(f"\n  [ERROR] File not found: {THEORY_PATH}")
            sys.exit(1)

        # Every Python black box basic_action_theory.pl :- use_module()s
        # (planners.py, collision_geometry.py) reads BT_PROBLEM_DIR at
        # IMPORT time to find this problem's own map.yaml/config.yaml/
        # obstacles_generated.pl -- must be set before ProbLog ever
        # loads the theory (see run_problog_api below).
        os.environ["BT_PROBLEM_DIR"] = problem_dir

        # Regenerate <problem>/obstacles_generated.pl from
        # <problem>/map.yaml BEFORE anything reads the theory --
        # map.yaml is the single source of truth for the obstacle
        # layout (see module/translators/occgrid_to_problog.py), same
        # automatic-every-run treatment config.yaml/behavior_tree.xml
        # already get.
        if TRANSLATORS_DIR not in sys.path:
            sys.path.insert(0, TRANSLATORS_DIR)
        try:
            from occgrid_to_problog import generate as generate_obstacles
            generated_obstacles_path = generate_obstacles(
                yaml_path=os.path.join(problem_dir, "map.yaml"),
                output_path=os.path.join(problem_dir, "obstacles_generated.pl"))
            tee(f"  Obstacles   : {generated_obstacles_path} (regenerated from "
                f"{os.path.join(problem_dir, 'map.yaml')})")
        except Exception as e:
            tee(f"\n  [ERROR] Could not regenerate obstacles_generated.pl: {e}")
            sys.exit(1)

        # Regenerate <problem>/config_generated.pl from
        # <problem>/config.yaml BEFORE anything reads the theory --
        # config.yaml is the single source of truth for every tunable
        # constant (see module/translators/config_to_prolog.py), so
        # every run picks up whatever is currently there with no
        # separate step.
        try:
            from config_to_prolog import generate as generate_config, load_config
            generated_config_path = generate_config(
                config_path=os.path.join(problem_dir, "config.yaml"),
                output_path=os.path.join(problem_dir, "config_generated.pl"))
            battery_enabled = load_config(
                os.path.join(problem_dir, "config.yaml")
            ).get("battery", {}).get("enabled", True)
            tee(f"  Config      : {generated_config_path} (regenerated from "
                f"{os.path.join(problem_dir, 'config.yaml')})")
        except Exception as e:
            tee(f"\n  [ERROR] Could not regenerate config_generated.pl: {e}")
            sys.exit(1)

        # Translate <problem>/behavior_tree.xml (the real BT.cpp v4
        # tree that is now the single source of truth for the POLICY'S
        # SHAPE) into <problem>/plan_generated.pl, validating it
        # against module/contracts/schema.yaml on the way -- see
        # module/translators/bt_to_prolog.py's own header. Any
        # structural problem (unknown node, missing/unrecognized port,
        # a control_points blackboard key with no producer) is a hard
        # failure here, same as a missing config fact above; there is
        # no sensible way to run inference against a tree that doesn't
        # actually match its own schema. battery_enabled (this
        # problem's own config.yaml battery.enabled, read above) is
        # threaded through so a disabled problem gets every battery-
        # related trigger name stripped from its own Triggers lists --
        # see bt_to_prolog.py's own generate_plan_pl/_is_battery_trigger.
        try:
            from bt_to_prolog import generate_plan_pl, BTValidationError
            generated_plan_path, reason_patterns_by_action, action_labels, condition_labels = generate_plan_pl(
                xml_path=os.path.join(problem_dir, "behavior_tree.xml"),
                schema_path=os.path.join(CONTRACTS_DIR, "schema.yaml"),
                output_path=os.path.join(problem_dir, "plan_generated.pl"),
                battery_enabled=battery_enabled)
            tee(f"  Plan (BT)   : {generated_plan_path} (translated + validated "
                f"from {os.path.join(problem_dir, 'behavior_tree.xml')})")
        except BTValidationError as e:
            tee(f"\n  [ERROR] behavior_tree.xml failed validation: {e}")
            sys.exit(1)
        except Exception as e:
            tee(f"\n  [ERROR] Could not translate behavior_tree.xml: {e}")
            sys.exit(1)

        # Validate <problem>/goal_formula.pl against
        # module/contracts/vocabulary.yaml -- same "structural
        # validation is a hard failure, not a warning" posture as
        # behavior_tree.xml's own validation just above; see
        # module/contracts/goal_formula_check.py's own header.
        if CONTRACTS_DIR not in sys.path:
            sys.path.insert(0, CONTRACTS_DIR)
        try:
            from goal_formula_check import (validate_goal_formula, generate_safety_queries,
                                             GoalFormulaValidationError)
            goal_formula_path = os.path.join(problem_dir, "goal_formula.pl")
            validate_goal_formula(
                goal_formula_path=goal_formula_path,
                vocab_path=os.path.join(CONTRACTS_DIR, "vocabulary.yaml"))
            tee(f"  Goal formula: {goal_formula_path} (validated against "
                f"{os.path.join(CONTRACTS_DIR, 'vocabulary.yaml')})")
        except GoalFormulaValidationError as e:
            tee(f"\n  [ERROR] goal_formula.pl failed validation: {e}")
            sys.exit(1)
        except Exception as e:
            tee(f"\n  [ERROR] Could not validate goal_formula.pl: {e}")
            sys.exit(1)

        # Generate <problem>/queries_generated.pl -- every safety
        # query this problem's own tree can actually produce, derived
        # from reason_patterns_by_action above (see bt_to_prolog.py's
        # own generate_plan_pl) -- called right after goal_formula.pl's
        # own validation since the two always run back-to-back (see
        # module/contracts/goal_formula_check.py's own header).
        try:
            generated_queries_path = generate_safety_queries(
                reason_patterns_by_action,
                output_path=os.path.join(problem_dir, "queries_generated.pl"),
                condition_codes=condition_labels.keys())
            tee(f"  Queries     : {generated_queries_path} (auto-generated from "
                f"this tree's own per-action Reason universe)")
        except Exception as e:
            tee(f"\n  [ERROR] Could not generate queries_generated.pl: {e}")
            sys.exit(1)

        # Rewrite module/theory/problem_data.pl -- the small bootstrap
        # file basic_action_theory.pl itself :- consult()s (see that
        # file's Section 0) to find the four problem-specific files
        # above. Written with ABSOLUTE paths since problems/<name>/ is
        # nowhere near module/theory/ on disk, and regenerated fresh
        # every run so basic_action_theory.pl never has to change to
        # serve a different --problem.
        problem_data_path = os.path.join(THEORY_DIR, "problem_data.pl")
        write_problem_data_pl(problem_data_path, problem_dir, goal_formula_path,
                               run_label=f"main.py --problem {args.problem}, run {ts}",
                               tee=tee)

        tee(f"\n  Started : {datetime.now():%H:%M:%S}  "
            f"(phase timeout: {args.phase_timeout}s per stage)")
        try:
            results, elapsed = run_problog_api(THEORY_PATH, tee, args.phase_timeout)
        except StageTimeout:
            tee(f"\n  [ERROR] Aborting -- a pipeline stage exceeded its "
                f"{args.phase_timeout}s timeout (see [STAGE] line above for which).")
            sys.exit(1)
        except ProbLogError as e:
            tee(f"\n  [ERROR] ProbLog error: {e}")
            sys.exit(1)
        except Exception as e:
            tee(f"\n  [ERROR] Unexpected error running the model: {e}")
            sys.exit(1)
        tee(f"  Finished: {datetime.now():%H:%M:%S}  ({elapsed:.3f}s)")

        if not results:
            tee("\n  [warn] No results returned from ProbLog -- check the "
                "file has query(...) declarations.")
            sys.exit(1)

        print_compact_summary(tee, results, goal_formula_path, action_labels, condition_labels)

        tee("")
        banner(tee, f"Log : {log_path}")


if __name__ == "__main__":
    main()
