"""
pipeline_stages.py

Shared ProbLog pipeline plumbing used by both main.py and
diagnose_pipeline.py: a STAGED, LOGGED, PER-PHASE-TIMEOUT-BOUNDED
replacement for the single opaque
    get_evaluatable().create_from(model).evaluate()
call every earlier version of main.py used, plus the small
problem_data.pl bootstrap-writer both scripts need (factored out here
so the two callers can't drift out of sync on that file's shape).

Why staged: ProbLog's own pipeline internally does four genuinely
separate things --
  1. PARSE   the .pl source (following its own :- consult(...) chain)
             into a LogicProgram.
  2. GROUND  it -- SLD-resolution proof search over every query,
             which is exactly where do_node's forward walk over
             plan(Node) happens, interleaved with each fluent's
             Reiter-style regression back to s0 (at/4, battery/3,
             ... recursing inward through do(A,S) to the s0 base
             clause, then computing back outward). This is "the plan
             unrolled, regressed to the initial situation" in one
             step -- ProbLog doesn't expose those as two separate
             calls, because regression IS how grounding resolves each
             fluent along the way. Produces a ground LogicFormula:
             one Boolean variable per resolved random choice (z/2,
             zt/2, zbatt/1, ...), one node per combination of proofs
             found.
  3. COMPILE that Boolean formula into a tractable circuit (SDD by
             default) for EXACT weighted model counting.
  4. EVALUATE that circuit against the annotated-disjunction weights
             -- the ones that ultimately bottom out at s0's own facts
             -- to produce each query's final probability. This is
             "verification against the initial database" made
             concrete: the weights being summed over here are s0's.

Collapsing all four into one call gives zero information about which
one is actually stuck when a run hangs. This module makes each one an
explicit, timed, logged step -- confirmed useful directly: pointing
this same staged pipeline at problem3's real Bug0 tree showed the
stall is in stage 2 (grounding), not stage 3 (compilation) as the
project's own FUTUREWORK.md investigation -- done at the granularity
of "whole evaluate_plan query, timed end-to-end" -- had left open;
`problog ground` alone (no compilation at all) already failed to
finish within 90s at the committed 5-value noise tables, while it
finished in 1.7s once every noise table was collapsed to one
deterministic value.

Uses the SAME signal.alarm/SIGALRM + timeout-exception mechanism
ProbLog's own CLI --timeout/--compile-timeout flags use internally
(problog/util.py's start_timer/stop_timer, which raises
KeyboardInterrupt("Timeout") on the alarm) -- not a separate, weaker
mechanism of our own. Same caveat that mechanism carries: a
signal-based timeout can only be delivered at a Python bytecode
boundary. If a stage is blocked deep inside a C extension call
(pysdd's SDD compilation in particular) that never returns control to
the interpreter, the alarm fires but isn't actually delivered until
that call returns -- so a stage timeout here is a lower bound on
wall-clock time genuinely spent in that stage, not always a hard kill.
For a guaranteed hard kill, run diagnose_pipeline.py under the shell's
own `timeout` command (see that script's own header for the exact
invocation) -- that kills the whole OS process regardless of what it's
blocked inside.
"""

import os
import signal
import time

from problog.program import PrologFile
from problog.formula import LogicFormula
from problog import get_evaluatable


class StageTimeout(Exception):
    """Raised when a single pipeline stage exceeds its own phase_timeout."""


def _stage_alarm(signum, frame):
    raise StageTimeout()


def run_stage(tee, label, func, timeout_seconds):
    """Run one pipeline stage: log a start line, call func() under a
    SIGALRM-based timeout, log a done/TIMED OUT/FAILED line with
    elapsed time, and return (result, elapsed_seconds) -- or re-raise
    (after logging) on timeout/failure, so the caller's own try/except
    decides whether to abort the whole run.
    """
    tee(f"  [STAGE] {label} ...")
    old_handler = signal.signal(signal.SIGALRM, _stage_alarm)
    signal.alarm(int(timeout_seconds))
    t0 = time.perf_counter()
    try:
        result = func()
    except StageTimeout:
        elapsed = time.perf_counter() - t0
        tee(f"  [STAGE] {label}: TIMED OUT after {elapsed:.1f}s "
            f"(phase timeout = {timeout_seconds}s)")
        raise
    except Exception as e:
        elapsed = time.perf_counter() - t0
        tee(f"  [STAGE] {label}: FAILED after {elapsed:.3f}s -- {e}")
        raise
    else:
        elapsed = time.perf_counter() - t0
        tee(f"  [STAGE] {label}: done in {elapsed:.3f}s")
        return result, elapsed
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old_handler)


def run_staged_inference(plan_file, tee, phase_timeout=300):
    """
    Runs the ProbLog resolution pipeline against plan_file as FOUR
    separate, logged, individually-timed-out stages (parse / ground /
    compile / evaluate -- see this module's own header for what each
    one actually is).

    Returns (results, timings):
      - results: {str(query_term): float(probability)} -- EXACTLY the
        same shape the old single-call run_problog_api always
        returned, so nothing downstream needs to change.
      - timings: {"parse"|"ground"|"compile"|"evaluate": elapsed_seconds}

    Raises StageTimeout if any single stage exceeds phase_timeout
    seconds (default 300s = 5 minutes) -- which stage is already named
    in the [STAGE] line logged just before the exception propagates,
    so a hang's location is known even though the run itself has to be
    aborted.
    """
    timings = {}

    model, t = run_stage(tee, "Parse (plan.pl formed)",
                          lambda: PrologFile(plan_file), phase_timeout)
    timings["parse"] = t

    lf, t = run_stage(tee, "Ground (plan unrolled, regressed to s0)",
                       lambda: LogicFormula.create_from(model, label_all=True),
                       phase_timeout)
    timings["ground"] = t
    tee(f"    -> {len(lf)} ground node(s)")

    compiled, t = run_stage(tee, "Compile (knowledge compilation)",
                             lambda: get_evaluatable().create_from(lf),
                             phase_timeout)
    timings["compile"] = t

    raw_result, t = run_stage(
        tee, "Evaluate (weighted model count against the initial database)",
        lambda: compiled.evaluate(), phase_timeout)
    timings["evaluate"] = t

    results = {str(term): float(prob) for term, prob in raw_result.items()}
    return results, timings


def write_problem_data_pl(problem_data_path, problem_dir, goal_formula_path,
                           run_label, tee=None):
    """Write module/theory/problem_data.pl's bootstrap consult chain,
    pointing (via absolute paths) at one problem directory's own
    obstacles_generated.pl / config_generated.pl / plan_generated.pl /
    queries_generated.pl, plus goal_formula_path -- the SAME shape
    main.py always wrote inline, factored out here so main.py and
    diagnose_pipeline.py (which deliberately does NOT translate
    behavior_tree.xml first -- see that script's own header) can't
    drift apart on this file's format. Every CALLER is responsible for
    having already written queries_generated.pl (via module/contracts/
    goal_formula_check.py's generate_safety_queries) before calling
    this -- the consult below fails outright if it doesn't exist yet.
    """
    with open(problem_data_path, "w") as f:
        f.write(
            "% AUTO-GENERATED. Points, via absolute paths, at whichever\n"
            "% problem was last selected -- see basic_action_theory.pl's\n"
            "% own Section 0 for how this file is used and regenerated.\n"
            f"% ({run_label})\n\n"
            f":- consult('{os.path.join(problem_dir, 'obstacles_generated.pl')}').\n"
            f":- consult('{os.path.join(problem_dir, 'config_generated.pl')}').\n"
            f":- consult('{os.path.join(problem_dir, 'plan_generated.pl')}').\n"
            f":- consult('{goal_formula_path}').\n"
            f":- consult('{os.path.join(problem_dir, 'queries_generated.pl')}').\n"
        )
    if tee:
        tee(f"  Bootstrap   : {problem_data_path} (points basic_action_theory.pl "
            f"at {problem_dir})")
