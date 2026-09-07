#!/usr/bin/env python3
"""
module/contracts/goal_formula_check.py

Validates a problem's own goal_formula.pl against
module/contracts/vocabulary.yaml BEFORE it's ever consulted/queried by
ProbLog -- exactly the same "structural validation is a hard failure,
not a warning" posture module/translators/bt_to_prolog.py already takes
for behavior_tree.xml against module/contracts/schema.yaml (see that
file's own header; this module is its sibling for the goal-formula side
of the pipeline).

TWO checks, matching this project's own discussion of what "a
well-formed goal formula" means:

  1. Every predicate goal_formula.pl's body calls (by name/arity) is a
     KNOWN predicate in vocabulary.yaml -- catches a typo, or a
     reference to a fluent that was renamed/removed/never existed, the
     same class of error an unknown BT.cpp node tag already catches
     for behavior_tree.xml.

  2. The formula is UNIFORM in its own head's situation argument
     (Reiter's own sense, "Knowledge in Action": the SAME single
     situation term appears in every situation-argument slot across
     the whole formula, never two different ones -- see
     vocabulary.yaml's own header for the full definition and why the
     situation-argument POSITION has to be looked up per-predicate,
     not assumed). Catches an accidentally-introduced second situation
     variable, which would silently change the formula's meaning from
     "check this property at ONE situation" to something else.

Uses ProbLog's own Prolog parser (problog.program.PrologString) to get
a REAL parsed term structure for goal_formula.pl's clause, rather than
a hand-rolled regex over the text -- the same reasoning this project
already used for bt_to_prolog.py's own XML parsing
(xml.etree.ElementTree, not a hand-rolled tag scanner).

Usage:
    python3 module/contracts/goal_formula_check.py
        (validates the default goal_formula.pl against the default
        vocabulary.yaml, printing OK or a validation error)

main.py calls validate_goal_formula() itself before every run, so you
don't normally need to run this by hand.

ALSO HOME TO generate_safety_queries() (see its own docstring, near the
bottom of this file) -- main.py calls it immediately after
validate_goal_formula(), writing this problem's own queries_generated.pl
from the per-action Reason universe module/translators/bt_to_prolog.py
worked out while translating behavior_tree.xml. Kept in this same file
specifically because the two steps always run back-to-back as one unit
in main.py's own pipeline.
"""
import os
import re
import sys

import yaml
from problog.logic import And, Not, Var
from problog.program import PrologString

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))
DEFAULT_VOCAB_PATH = os.path.join(_THIS_DIR, "vocabulary.yaml")
DEFAULT_GOAL_FORMULA_PATH = os.path.join(
    _PROJECT_ROOT, "problems", "problem0", "goal_formula.pl")


class GoalFormulaValidationError(Exception):
    """Raised for any structural problem in goal_formula.pl relative to
    vocabulary.yaml -- always fatal, never a warning (same posture as
    bt_to_prolog.py's own BTValidationError)."""


def load_vocabulary(vocab_path=DEFAULT_VOCAB_PATH):
    with open(vocab_path) as f:
        vocab = yaml.safe_load(f)
    index = {}
    for entry in vocab.get("predicates", []):
        index[(entry["name"], entry["arity"])] = entry
    return index


def _iter_conjuncts(body):
    """Flatten a right-associative And(...) tree into a flat list of
    leaf subgoals to VALIDATE, in source order. Negation (\\+/1) is
    transparent here -- \\+ crashed_in(S) is still perfectly uniform
    (the situation argument inside is still just S), \\+ itself is a
    logical connective, not a vocabulary predicate, so it's unwrapped
    and its own argument is recursed into rather than looked up.
    Disjunction (';'/2) is NOT handled yet -- goal_formula.pl is
    expected to be a plain conjunction (possibly negated), see this
    module's own header on what's NOT supported."""
    if isinstance(body, And):
        left, right = body.args
        return _iter_conjuncts(left) + _iter_conjuncts(right)
    if isinstance(body, Not):
        return _iter_conjuncts(body.args[0])
    return [body]


def _arity(term):
    return len(term.args) if term.args else 0


def _situation_arg_position(vocab_index, term, goal_formula_path):
    """Looks up term's own functor/arity in the vocabulary. Returns
    the 1-based situation-argument position, or None if this
    predicate has no situation argument at all (a plain fact/helper --
    see vocabulary.yaml's own note on situation_arg: null). Raises
    GoalFormulaValidationError if the predicate isn't in the
    vocabulary at all (check 1)."""
    arity = _arity(term)
    key = (term.functor, arity)
    if key not in vocab_index:
        raise GoalFormulaValidationError(
            f"{goal_formula_path} calls '{term.functor}/{arity}', which is "
            f"not a known predicate in vocabulary.yaml -- typo, or a "
            f"fluent that isn't (yet) documented there? See "
            f"vocabulary.yaml's own predicates list for what's available.")
    return vocab_index[key].get("situation_arg")


def validate_goal_formula(goal_formula_path=DEFAULT_GOAL_FORMULA_PATH,
                           vocab_path=DEFAULT_VOCAB_PATH):
    """Parses goal_formula_path, checks it defines EXACTLY ONE
    goal_formula/1 clause whose own head argument is a variable,
    checks every subgoal in its body against vocab_path (check 1), and
    checks the whole formula is uniform in that one head variable
    (check 2). Raises GoalFormulaValidationError on any failure;
    returns nothing on success."""
    vocab_index = load_vocabulary(vocab_path)

    with open(goal_formula_path) as f:
        text = f.read()

    clauses = list(PrologString(text))
    goal_clauses = [c for c in clauses
                    if getattr(c, "functor", None) == ":-"
                    and c.args[0].functor == "goal_formula"]
    if len(goal_clauses) != 1:
        raise GoalFormulaValidationError(
            f"{goal_formula_path} must define EXACTLY ONE goal_formula/1 "
            f"clause (found {len(goal_clauses)}). Local helper predicates "
            f"alongside it are not yet supported by this checker -- see "
            f"vocabulary.yaml's own 'CURRENT LIMITATION' note.")

    clause = goal_clauses[0]
    head, body = clause.args
    if _arity(head) != 1:
        raise GoalFormulaValidationError(
            f"{goal_formula_path} defines goal_formula/{_arity(head)} -- "
            f"must be goal_formula/1, one argument, the situation to "
            f"check the formula at.")
    situation_var = head.args[0]
    if not isinstance(situation_var, Var):
        raise GoalFormulaValidationError(
            f"{goal_formula_path}'s goal_formula(...) argument must be a "
            f"VARIABLE (e.g. 'S'), not a ground term ('{situation_var}') "
            f"-- it has to be applicable at whichever situation the "
            f"caller supplies (final_situation, in practice via "
            f"basic_action_theory.pl's own verify_goal_formula wrapper), "
            f"not hardwired to one here.")

    for term in _iter_conjuncts(body):
        pos = _situation_arg_position(vocab_index, term, goal_formula_path)
        if pos is None:
            continue
        arity = _arity(term)
        if pos < 1 or pos > arity:
            raise GoalFormulaValidationError(
                f"vocabulary.yaml's entry for {term.functor}/{arity} "
                f"declares situation_arg={pos}, out of range for this "
                f"predicate's own arity -- fix vocabulary.yaml.")
        actual = term.args[pos - 1]
        if actual != situation_var:
            raise GoalFormulaValidationError(
                f"{goal_formula_path} is not uniform in its own situation "
                f"argument '{situation_var}': {term.functor}/{arity} is "
                f"applied to '{actual}' (argument {pos}) instead. Every "
                f"fluent in a goal formula must be checked at the SAME "
                f"situation -- see vocabulary.yaml's own header for "
                f"Reiter's 'uniform in s' definition. A formula that "
                f"genuinely needs to relate two different situations "
                f"(e.g. 'visited A before visited B') isn't supported yet.")


# -----------------------------------------------------------------------
# SAFETY QUERY GENERATION -- deliberately kept in this same file, right
# alongside goal_formula.pl's own validation, since main.py calls both
# back-to-back as one step: validate the goal formula, THEN generate
# the safety queries that get consulted alongside it (see main.py's own
# call site and pipeline_stages.write_problem_data_pl, which now also
# consults this function's own output file).
#
# WHY THIS REPLACES basic_action_theory.pl's old hardcoded Section 10:
# a problem-INDEPENDENT theory file can't itself vary per-problem, but
# the file it consults (via problem_data.pl) can -- the SAME reasoning
# config_to_prolog.py already used for conditionally emitting
# query(any_battery_depletion), generalized to EVERY safety query
# instead of just that one hand-picked case. basic_action_theory.pl's
# own halted_with_pattern/3, any_reason_pattern/1, and
# any_reason_pattern_by_action/2 (see that file's Section 9) are the
# fixed, problem-independent MACHINERY; this function decides, per
# problem, WHICH Reason patterns are actually worth asking about --
# exactly the ones module/translators/bt_to_prolog.py already worked
# out are possible for each MoveNode while translating the tree (see
# its own _VarPool.reason_patterns_by_action).
# -----------------------------------------------------------------------

_ALWAYS_QUERIES = [
    "verify_goal_formula",
    "plan_outcome(true)",
    "plan_outcome(false)",
    "plan_outcome(world_too_large)",
    "plan_outcome(reactive_escaped)",
]


def generate_safety_queries(reason_patterns_by_action, output_path, condition_codes=()):
    """Writes output_path (this problem's own queries_generated.pl):
    the five ALWAYS-relevant, tree-shape-independent queries, then one
    query(any_reason_pattern(Pattern)) per DISTINCT Reason pattern
    reachable ANYWHERE in the tree (the auto-generated replacement for
    hand-picking any_collision/any_battery_depletion -- any_collision
    is now any_reason_pattern(crashed(_)), any_battery_depletion is
    any_reason_pattern(battery_depleted), and every OTHER reason this
    problem's own tree can produce gets the same treatment, not just
    the two someone thought to hand-write), then one
    query(any_reason_pattern_by_action(Pattern,ActionCode)) per
    (pattern, action) pair actually possible -- the per-node breakdown.

    reason_patterns_by_action is {action_code: [reason_pattern_text,
    ...]}, exactly module/translators/bt_to_prolog.py's own
    generate_plan_pl second return value. Pass {} (e.g. for a hand-
    written plan_generated.pl that bypasses translation entirely --
    see diagnose_pipeline.py) to still get the five ALWAYS queries with
    no per-reason breakdown.

    ALSO emits, for every (pattern, action) pair whose own pattern
    contains the 'wild' marker somewhere, a companion query(any_reason_
    pattern_detail_by_action(...)) with 'wild' replaced by a genuine
    Prolog variable ('_') -- see basic_action_theory.pl's own
    halted_with_pattern_detail/3 for why THIS query is deliberately
    non-ground: ProbLog enumerates one result row per distinct
    grounding for it, which is exactly what turns e.g.
    "crashed(wild) on a1 = 30%" into its own further breakdown by
    WHICH concrete obstacle. A pattern with no 'wild' in it at all
    (battery_under(20), guard_break(Cond), completed, ...) has nothing
    runtime-only to enumerate, so gets no companion query.

    condition_codes is an iterable of every condition-leaf code this
    problem's own tree produced (module/translators/bt_to_prolog.py's
    own generate_plan_pl fourth return value's keys, i.e.
    condition_labels) -- for each one, emits BOTH
    query(any_condition_status(Code,true)) and query(any_condition_
    status(Code,false)), mirroring the (pattern,action) pairs above but
    for CONDITIONS (see basic_action_theory.pl's own any_condition_
    status/2). Defaults to () for the same "hand-written plan, nothing
    to derive" case reason_patterns_by_action's own {} default covers.

    ALSO always emits query(outcome_signature(_)) -- theory-level,
    generic machinery needing no per-problem derivation at all (see
    that predicate's own note in basic_action_theory.pl): one result
    row per DISTINCT combination of Reason/Condition values actually
    reached, the full joint enumeration main.py's own "Full outcome
    enumeration" table is built from."""
    all_patterns = sorted(set(
        pattern
        for patterns in reason_patterns_by_action.values()
        for pattern in patterns))

    lines = [
        "% AUTO-GENERATED by module/contracts/goal_formula_check.py's",
        "% generate_safety_queries -- DO NOT HAND-EDIT, this is derived",
        "% entirely from the problem's own behavior_tree.xml (via",
        "% bt_to_prolog.py) and regenerated by main.py before every run.",
        "",
    ]
    lines += [f"query({q})." for q in _ALWAYS_QUERIES]
    # outcome_signature/1 is theory-level, GENERIC machinery (see its own
    # note in basic_action_theory.pl) -- unlike everything else here, it
    # needs no per-problem derivation at all, so it's always emitted,
    # even when reason_patterns_by_action/condition_codes are both {}/()
    # (e.g. diagnose_pipeline.py's hand-written-plan path). Deliberately
    # NON-ground, like the _detail queries below -- one result row per
    # DISTINCT outcome combination actually reached.
    lines.append("query(outcome_signature(_)).")
    lines.append("")
    lines += [f"query(any_reason_pattern({pattern}))." for pattern in all_patterns]
    lines.append("")
    for action_code in sorted(reason_patterns_by_action):
        for pattern in reason_patterns_by_action[action_code]:
            lines.append(f"query(any_reason_pattern_by_action({pattern},{action_code})).")
            if re.search(r"\bwild\b", pattern):
                detail_pattern = re.sub(r"\bwild\b", "_", pattern)
                lines.append(f"query(any_reason_pattern_detail_by_action({detail_pattern},{action_code})).")
    lines.append("")
    for condition_code in sorted(condition_codes):
        lines.append(f"query(any_condition_status({condition_code},true)).")
        lines.append(f"query(any_condition_status({condition_code},false)).")
    lines.append("")

    with open(output_path, "w") as f:
        f.write("\n".join(lines))
    return output_path


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_GOAL_FORMULA_PATH
    try:
        validate_goal_formula(path)
    except GoalFormulaValidationError as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)
    print(f"{path}: OK")
