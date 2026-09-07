#!/usr/bin/env python3
"""
diagnose_pipeline.py

Runs the SAME staged, logged, per-phase-timeout-bounded ProbLog
pipeline main.py uses (see pipeline_stages.py) against one problem's
EXISTING plan_generated.pl DIRECTLY, WITHOUT translating/validating
behavior_tree.xml first.

Why this exists: main.py's normal flow re-translates
<problem>/behavior_tree.xml into <problem>/plan_generated.pl on every
run, via module/translators/bt_to_prolog.py, and refuses to proceed if
that fails validation against module/contracts/schema.yaml --
correctly, for a NORMAL run, since plan_generated.pl must always
reflect the current behavior_tree.xml.

problem3 is the deliberate exception: its own plan_generated.pl is
HAND-WRITTEN (see that file's own header) specifically BECAUSE its
behavior_tree.xml uses <Inverter> and a couple of other shapes
bt_to_prolog.py doesn't support yet -- so `main.py --problem problem3`
fails at the translation step, before ever reaching ProbLog at all.
This script skips that translation step entirely and consults
<problem>/plan_generated.pl exactly as it already sits on disk,
regenerating only obstacles_generated.pl/config_generated.pl
(map.yaml/config.yaml aren't affected by the Inverter gap) and
re-validating goal_formula.pl (also unaffected, and cheap).

Usage (problem3, the case this was built for):
    python3 diagnose_pipeline.py --problem problem3

    # Hard-kill guarantee (see pipeline_stages.py's own header on why a
    # SIGALRM-based per-stage timeout alone can't always force a stuck
    # C-extension call to return): wrap the whole process in the
    # shell's own timeout too, generously above --phase-timeout*4:
    timeout 1300 python3 diagnose_pipeline.py --problem problem3 --phase-timeout 300

No plotting, no formatted safety report -- just the [STAGE] log lines
from pipeline_stages.py (which phase is running, how long each took or
whether it timed out) and a raw dump of every query(...)'s final
probability. Use main.py for the full report once a problem's own
behavior_tree.xml translates cleanly.
"""

import argparse
import os
import sys
from datetime import datetime

from problog.errors import ProbLogError

from pipeline_stages import run_staged_inference, write_problem_data_pl, StageTimeout

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MODULE_DIR = os.path.join(_THIS_DIR, "module")
THEORY_DIR = os.path.join(MODULE_DIR, "theory")
TRANSLATORS_DIR = os.path.join(MODULE_DIR, "translators")
CONTRACTS_DIR = os.path.join(MODULE_DIR, "contracts")
PROBLEMS_DIR = os.path.join(_THIS_DIR, "problems")
THEORY_PATH = os.path.join(THEORY_DIR, "basic_action_theory.pl")


def tee(text=""):
    print(text)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--problem", required=True,
                     help="Problem subdirectory of problems/ -- its "
                          "EXISTING plan_generated.pl is consulted "
                          "as-is; behavior_tree.xml is NOT re-translated.")
    ap.add_argument("--phase-timeout", type=int, default=300,
                     help="Per-stage timeout in seconds (default: 300 "
                          "= 5 minutes). Each of parse/ground/compile/"
                          "evaluate gets its own independent budget.")
    args = ap.parse_args()

    problem_dir = os.path.join(PROBLEMS_DIR, args.problem)
    plan_path = os.path.join(problem_dir, "plan_generated.pl")
    goal_formula_path = os.path.join(problem_dir, "goal_formula.pl")

    if not os.path.isdir(problem_dir):
        tee(f"[ERROR] No such problem directory: {problem_dir}")
        sys.exit(1)
    if not os.path.isfile(plan_path):
        tee(f"[ERROR] {plan_path} does not exist -- this script consults "
            f"whatever plan_generated.pl is ALREADY on disk, it does not "
            f"translate behavior_tree.xml. Run main.py once for a problem "
            f"whose tree translates cleanly, or hand-write plan_generated.pl "
            f"(see problems/problem3/plan_generated.pl for the pattern).")
        sys.exit(1)

    # planners.py / collision_geometry.py (loaded by basic_action_theory.pl
    # via :- use_module(...)) read this at IMPORT time -- must be set
    # before ProbLog ever loads the theory, same requirement as main.py's.
    os.environ["BT_PROBLEM_DIR"] = problem_dir

    tee("=" * 68)
    tee(f"  Staged ProbLog pipeline diagnostic - {datetime.now():%Y-%m-%d %H:%M:%S}")
    tee(f"  Problem      : {args.problem} ({problem_dir})")
    tee(f"  Plan (as-is) : {plan_path}")
    tee(f"                 [NOT re-translated from behavior_tree.xml -- "
        f"consulted exactly as it sits on disk]")
    tee(f"  Phase timeout: {args.phase_timeout}s per stage")
    tee("=" * 68)

    if TRANSLATORS_DIR not in sys.path:
        sys.path.insert(0, TRANSLATORS_DIR)
    try:
        from occgrid_to_problog import generate as generate_obstacles
        generate_obstacles(
            yaml_path=os.path.join(problem_dir, "map.yaml"),
            output_path=os.path.join(problem_dir, "obstacles_generated.pl"))
        tee(f"  [STAGE] Obstacles regenerated from map.yaml: done")
    except Exception as e:
        tee(f"  [ERROR] Could not regenerate obstacles_generated.pl: {e}")
        sys.exit(1)

    try:
        from config_to_prolog import generate as generate_config
        generate_config(
            config_path=os.path.join(problem_dir, "config.yaml"),
            output_path=os.path.join(problem_dir, "config_generated.pl"))
        tee(f"  [STAGE] Config regenerated from config.yaml: done")
    except Exception as e:
        tee(f"  [ERROR] Could not regenerate config_generated.pl: {e}")
        sys.exit(1)

    if CONTRACTS_DIR not in sys.path:
        sys.path.insert(0, CONTRACTS_DIR)
    try:
        from goal_formula_check import (validate_goal_formula, generate_safety_queries,
                                         GoalFormulaValidationError)
        validate_goal_formula(
            goal_formula_path=goal_formula_path,
            vocab_path=os.path.join(CONTRACTS_DIR, "vocabulary.yaml"))
        tee(f"  [STAGE] goal_formula.pl validated: done")
    except GoalFormulaValidationError as e:
        tee(f"  [ERROR] goal_formula.pl failed validation: {e}")
        sys.exit(1)
    except Exception as e:
        tee(f"  [ERROR] Could not validate goal_formula.pl: {e}")
        sys.exit(1)

    # behavior_tree.xml is SKIPPED here (see this script's own header),
    # so there's no bt_to_prolog.py-derived reason_patterns_by_action
    # to build a per-action breakdown from -- generate the five ALWAYS
    # queries only (see generate_safety_queries' own {} note); the
    # hand-written plan_generated.pl's own Triggers lists just won't
    # get a Table 2 breakdown here.
    try:
        generate_safety_queries({}, output_path=os.path.join(problem_dir, "queries_generated.pl"))
        tee(f"  [STAGE] queries_generated.pl written (universal queries only -- "
            f"no BT translation happened to derive a per-action breakdown from): done")
    except Exception as e:
        tee(f"  [ERROR] Could not generate queries_generated.pl: {e}")
        sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    problem_data_path = os.path.join(THEORY_DIR, "problem_data.pl")
    write_problem_data_pl(
        problem_data_path, problem_dir, goal_formula_path,
        run_label=(f"diagnose_pipeline.py --problem {args.problem}, run {ts} "
                   f"-- behavior_tree.xml SKIPPED, plan_generated.pl consulted as-is"),
        tee=tee)
    tee(f"  [STAGE] plan.pl formed (plan_generated.pl consulted as-is): done")

    tee("")
    try:
        results, timings = run_staged_inference(
            THEORY_PATH, tee, phase_timeout=args.phase_timeout)
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

    tee("")
    tee("=" * 68)
    tee("  Results")
    tee("=" * 68)
    for name, prob in sorted(results.items()):
        tee(f"  {name:<40} {prob:.6f}")
    total = sum(timings.values())
    tee(f"\n  Total pipeline time: {total:.3f}s  "
        f"(parse {timings['parse']:.3f}s, ground {timings['ground']:.3f}s, "
        f"compile {timings['compile']:.3f}s, evaluate {timings['evaluate']:.3f}s)")


if __name__ == "__main__":
    main()
