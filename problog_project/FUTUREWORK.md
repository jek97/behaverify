# FUTUREWORK.md

## Adopting Reiter progression to fix the reactive-redescend blowup

This document records a diagnosed performance problem in the current
ProbLog action theory, the empirical evidence gathered for it, and a
concrete design for fixing it by adopting **progression** (Reiter,
*Knowledge in Action*, Ch. 4) at each reactive redescend point instead
of always regressing all the way back to `s0`. It is written to be
picked up cold, on a fresh branch, without needing to re-derive any of
this from scratch.

---

## 1. The problem

`evaluate_plan/4` (`module/theory/basic_action_theory.pl:1577-1589`)
drives `plan/1`'s whole tree to a `true`/`false`/`world_too_large`
conclusion, **re-descending the ENTIRE tree from its own root** every
time a leaf comes back `reactive`:

```prolog
evaluate_plan(S0, S, Outcome, Budget) :-
    Budget > 0,
    plan(Node),
    do_node(Node, S0, S1, reactive),
    Budget1 is Budget - 1,
    evaluate_plan(S1, S, Outcome, Budget1).
evaluate_plan(S0, S1, Outcome, _Budget) :-
    plan(Node),
    do_node(Node, S0, S1, Outcome),
    Outcome \== reactive.
evaluate_plan(S0, S0, world_too_large, 0) :-
    plan(Node),
    do_node(Node, S0, _, reactive).
```

Each redescend does **not** start reasoning from a fresh situation —
`S1` is `do(startMoveto(...), do(..., ... do(..., s0)))`, i.e. the
*entire* action history so far, with the just-halted leg's own
`startMoveto`/`haltMoveto` appended. `do_node`'s leaf clauses
(`moveto_leg/2` at line 1163, `planWith/3` at line 1331) then compute
their own preconditions and effects via fluents (`at/4`, `battery/3`,
`holds/2`) that are themselves classic Reiter-style **regressed**
functional fluents:

```prolog
battery(Level, T, do(startMoveto(CP,Triggers,T0), S)) :-
    battery(B0, T0, S),
    ...
battery(Level, T, do(haltMoveto(T1,Reason,Status), S)) :-
    ...
    battery(B1, T1, S),
    ...
```

(`basic_action_theory.pl:410-484` for `battery/3`, `:1034-1069` for
`at/4`; both are pure recursive unwindings back to `s0`, one clause
per action type, in the textbook Reiter successor-state-axiom style —
see the book's Ch. 4 general form `F(do(a,s)) ≡ Φ_F(a,s)`.)

So on hop *k*+1, answering "what's the battery level / position at
time T" requires regressing all the way back through hop *k*'s
**entire** noisy walk again — every one of hop *k*'s own random
choices is still symbolically alive in the query. Under ProbLog's
exact-inference backends (knowledge compilation / weighted model
counting, via DSharp or SDD/`pysdd` — both were tested and both
exhibit this), the compiled Boolean formula's size is driven by how
many distinct random variables are simultaneously "live" in it. Since
nothing is ever forgotten across a redescend, that count only grows
with the number of hops.

### Which random variables actually accumulate

From `config.yaml`'s noise section and its generated Prolog
(`config_generated.pl`):

- **`z/2` (position/lateral) and `zt/2` (tangential)** are indexed by
  the *situation at the start of each leg's own walk* —
  `z(do(startMoveto(CP,Triggers,T0),SPrev), Z)` — so **every leg of
  every hop draws its own fresh `z`/`zt`**. Two hops means two
  independent `z` variables and two independent `zt` variables live
  in the compiled formula simultaneously once hop 2 is reasoned about,
  and they're not just added but *entangled*: `walk_noisy_point/8`
  (`:777-810`) combines `Z` and `Zt` together into the actual (X,Y)
  used both for goal-tolerance checking (`leg_status/8`) and for
  exact crossing-time detection (`earliest_halt/10` →
  `all_trigger_candidates`), so the two axes interact non-trivially
  inside the same case-split logic per leg, per hop.
- **`zbatt/1`** is a single **mission-wide** global variable (no
  situation argument at all) — drawn once, reused by every
  `battery/3` clause everywhere in the history. This is why it's
  comparatively cheap: it contributes a small constant factor
  regardless of hop count, not a per-hop multiplier.

This is consistent with the empirical results below: activating
`zbatt` alongside one other axis stays cheap, but activating *three*
per-leg-entangled axes together (implicitly compounding across two
hops) is what blows up — it isn't which axis, it's how many are
simultaneously live across an ever-growing, never-forgotten history.

### Empirical evidence (problem4, the minimal reproduction)

`problem4` (`problems/problem4/`) is a deliberately minimal
translator-verified BT: one `Fallback` of two `Sequence`s, one
reactive trigger (`battery_below(70)`), no obstacle geometry. All
tests below use `replan_budget(1)` (i.e. exactly one redescend, two
hops total) via genuine `evaluate_plan`/`plan_outcome` queries, no
hand-rolled diagnostic predicates.

| Profile (position / tangential / battery value-counts) | Active stochastic axes | Result |
|---|---|---|
| 3, 1, 1 | 1 | Fast |
| 5, 1, 1 | 1 | Fast |
| 3, 3, 1 | 2 (pos+tang) | Fast |
| 3, 1, 3 | 2 (pos+batt) | Fast |
| 3, 3, 3 | 3 | **Timed out (>10 min)** |

(Profile notation: a discretized-Gaussian `noise.<axis>.discretized_gaussian`
table in `config.yaml` with that many `value`/`weight` entries; `1`
means a single deterministic `value: 0.0, weight: 1.0` entry.) The
same qualitative blowup was seen, at production scale, on `problem3`'s
real Bug0 obstacle-avoidance tree — this was independently confirmed
across **both** the DSharp and SDD compilation backends (an earlier
apparent DSharp-vs-SDD correlation was traced to a stale
`module/theory/problem_data.pl` pointing at the wrong problem, not a
real backend difference — see git history around
`9887a56`/`1211b61`/`911d7c8`/`0c3c12d` for the full diagnostic
trail).

**Conclusion:** the driver is the *count of simultaneously-active
stochastic axes carried across the accumulated history*, not tree
depth/branching by itself (already ruled out earlier in the same
investigation) and not any single axis being special.

---

## 2. Why progression is the right fix

Reiter's *Knowledge in Action*, Ch. 4, draws exactly this distinction:

- **Regression** answers a query about `do(a_n,...,do(a_1,S_0)...)`
  by mechanically reducing it, one action at a time, to an equivalent
  query about `S_0` alone. It never needs a database at any
  intermediate situation, but the regressed formula's size can grow
  with the length of the history being regressed through — exactly
  what `evaluate_plan/4` is doing, once per redescend, forever
  carrying the whole walk.
- **Progression** goes the other way: given a database `D_S`
  (the current truth values of the fluents at `S`) and an action `a`,
  it computes a new database `D_{do(a,S)}` that is *provably
  equivalent*, for the purposes of reasoning about the future, to
  continuing to regress through the full history — but the history
  itself can then be **discarded**. Future queries are asked relative
  to the new, compact `D_{do(a,S)}` as if it were a fresh `S_0`.

Progression is not unconditionally first-order computable in general
(Lin & Reiter 1997 show the general case needs second-order
circumscription), but it *is* guaranteed first-order (finitely
representable) for **local-effect, context-free** successor-state
axioms — where an action's effect on a fluent depends only on that
action's own arguments and the *immediately preceding* value of that
*same* fluent, not on quantifying over other objects or other
fluents.

**This domain's SSAs are exactly that class, and simpler still**:
`battery/3` and `at/4` are not general relational SSAs needing
circumscription at all — they are closed-form *scalar chain
recurrences* (`battery(Level,T,do(A,S)) :- battery(B0,T0,S), <closed
form arithmetic>`). The theory already exploits this: battery's
crossing time is solved *algebraically*
(`first_battery_depletion_time`, referenced at `:486-494`) rather than
by bracket-scan+bisection, precisely because it's linear in elapsed
time within one leg. Progressing these fluents at a redescend point is
therefore not the hard, second-order general case from the book — it
is "evaluate the closed form at the halt instant and keep the
resulting number(s) instead of the formula that produced them."

---

## 3. The probabilistic subtlety (the actual design work)

Classical progression replaces a ground database with a new ground
database. Here, what must be progressed is not a single value but a
**distribution** — a fresh, small discretized table over
`(X, Y, battery)` at the halt situation, playing exactly the role
`start_x`/`start_y`/`battery_start` plus the `z`/`zt`/`zbatt` tables
already play at `s0` (see `config.yaml`'s own `noise:` section and
`module/translators/config_to_prolog.py`). This is not a new kind of
object for this codebase — it's the same shape of thing already used
everywhere, just re-centered on a new situation instead of `s0`.

Two things make this non-trivial, both worth stating precisely before
implementing:

1. **Conditioning on the trigger, not just "the state at some T".**
   The halt situation is defined by *which* trigger fired
   (`leg_status/8`'s `reactive` case, `:977-980`) and *when*
   (`earliest_halt/10`). For a threshold trigger like
   `battery_below(70)`, this means the progressed **battery** value
   is close to *pinned* by construction — the halt is, by definition,
   the instant battery crosses 70%, and because `battery/3` is exactly
   linear in elapsed time within a leg (fixed `Zb`), that crossing
   instant is an algebraic function of `Zb` alone, already computed by
   `first_battery_depletion_time`. What's genuinely *uncertain* at the
   halt point is the **position**, since it depends on `Z` and `Zt`
   sampled up to that (now `Zb`-determined) elapsed time. So the
   progression work per hop is smaller than "progress three
   variables" — it's closer to "solve for the pinned variable
   algebraically (already done), discretize the position distribution
   at that solved instant." For a non-battery trigger (e.g. a future
   `collision`/`obstacle_in_bound` reactive redescend), the roles
   would differ — that would need its own case-by-case analysis
   before generalizing this beyond the battery-only `problem4` case.

2. **Position's progressed distribution is joint, not two independent
   marginals.** `walk_noisy_point/8` combines `Z` and `Zt` together
   (perpendicular vs. tangential offsets from the *same* nominal
   spline point) to produce `(X,Y)`. The progressed table for the next
   hop's starting position is therefore properly a **joint**
   discretized distribution over `(dx,dy)` (or equivalently over
   `(Z,Zt)` pairs weighted by their joint probability), not two
   separately-discretized axes multiplied together after the fact —
   doing the latter would silently reintroduce an independence
   assumption between position and tangential error that the current
   *single-leg* model doesn't make. This is a modeling decision to
   make deliberately, not by accident: either (a) build the true joint
   table (more faithful, more table entries — cost is bounded by the
   product of the per-axis discretization counts of *one* leg, not
   accumulated across hops, which is exactly the point), or (b)
   accept a documented independence approximation at the progression
   boundary if the joint table proves too large in practice.

Discretizing a continuous progressed distribution is not a new kind of
approximation for this project — the existing `z`/`zt`/`zbatt` tables
in `config.yaml` are already discretized-Gaussian approximations of
continuous noise. Progression reuses that same idea, just re-centered.

---

## 4. Proposed architecture

Restructure `evaluate_plan`'s single monolithic ProbLog query into a
**chain of small, independent ProbLog sub-inferences**, one per hop,
glued together by Python (the same layer that already regenerates
`config_generated.pl`/`problem_data.pl` per run — `main.py`,
`module/translators/config_to_prolog.py`):

1. **Hop *k* runs as its own ProbLog inference**, starting from a
   compact situation (for hop 0, literally today's `s0`; for hop
   *k*>0, a *progressed* situation — see step 3) and querying for:
   - the outcome (`true`/`false`/`reactive`) and its probability, and
   - *if reactive*: the joint discretized distribution over the halt
     position (and, for non-battery triggers, whatever else is
     uncertain at that halt) — likely producing this by adding a
     query per discretization bin, or by post-processing the sample
     weights ProbLog already reports (compare to how `first_hit(I)`/
     `on_track(I)` already turn continuous time into a discrete PMF
     over samples — the same technique applies here).
   - This bounds the sub-problem's variable count to *that hop's own*
     `z`/`zt` draws (plus the single shared `zbatt`), never the union
     across hops — directly addressing the measured driver from
     Section 1.

2. **Discretize the reported halt-state distribution** into a small
   table, the same shape as `config.yaml`'s existing
   `noise.position.discretized_gaussian` tables (or a 2-D joint
   variant per Section 3's point 2) — this step lives in Python,
   parallel to `config_to_prolog.py`.

3. **Feed that table into hop *k*+1 as a fresh "start state"**: write
   it out as a new small set of probabilistic facts (structurally like
   `config_generated.pl`'s `z/2`/`zbatt/1` clauses, but keyed to the
   progressed situation rather than `s0`) and re-run `problog` for hop
   *k*+1 against *that*, with **no `do(...)` term referencing hop
   *k*'s actions at all** — the whole point of progression is that hop
   *k*+1 never needs to mention hop *k*'s history again.

4. **Combine hop outcomes in the Python glue**, not in ProbLog:
   `P(final outcome) = Σ_states P(hop k halts in state) × P(hop k+1
   outcome | progressed state)`, iterating up to `replan_budget`
   times, exactly mirroring `evaluate_plan/4`'s existing recursive
   structure but with the recursion happening in Python across
   *separate* ProbLog processes instead of inside one Prolog query
   whose situation term keeps growing.

This is a genuine architecture change — splitting one Prolog-level
recursive predicate into a Python-orchestrated chain of ProbLog
sub-calls — not a small patch to `evaluate_plan/4`'s clauses. It
should be **prototyped against `problem4` first** (already the
minimal, translator-verified reproduction case built for exactly this
investigation) before touching `problem3`'s real Bug0 tree, and
validated by comparing its output probabilities against the current
monolithic `evaluate_plan/4` result on cases still small enough for
the latter to finish (e.g. the 3,1,1 / 5,1,1 / 3,3,1 / 3,1,3 profiles
above, all of which currently succeed) before trusting it on the 3,3,3
case the current approach can't finish at all.

---

## 5. Open questions / risks

- **Generalizing beyond battery triggers.** The "pinned variable
  solved algebraically, only position needs discretizing" simplification
  in Section 3 is specific to threshold-on-a-linear-fluent triggers
  like `battery_below`/`battery_over`. A `collision`/
  `obstacle_in_bound` reactive redescend would need its own analysis
  of what's pinned vs. uncertain at the halt point before this
  generalizes to `problem3`'s real tree.
- **Discretization granularity vs. compounding approximation error.**
  Each progression step introduces a new discretization; over many
  redescends (`replan_budget` large) this error could compound. Worth
  an explicit sensitivity check (compare progressed-chain output
  against monolithic `evaluate_plan/4` output at matching small
  parameters) before relying on it for anything the current approach
  can't already validate directly.
- **Joint vs. independent position table (Section 3, point 2)** is a
  real accuracy/tractability tradeoff to make deliberately, with a
  documented choice, not silently.
- **Where the chain lives.** Whether the per-hop orchestration belongs
  in `main.py` directly, a new sibling script, or a new Python module
  alongside `module/theory/planners.py`/`collision_geometry.py` is an
  open implementation decision, not resolved by this document.

---

## 6. References

- R. Reiter, *Knowledge in Action: Logical Foundations for Specifying
  and Implementing Dynamical Systems*, MIT Press, 2001 — Chapter 4
  ("Progression"), in particular the local-effect/context-free
  first-order-definability result (Lin & Reiter, "How to Progress a
  Database", *Artificial Intelligence* 92 (1997), 131–167).
- This repo's own diagnostic trail: commits `0c3c12d` ("Add
  reproducible two-hop performance diagnostic for problem3"),
  `911d7c8` ("Add 3x3x3 noise variant..."), `1211b61` ("Add 5x1x1 and
  3x3x1 noise variants, pinning down the real driver"), `9887a56`
  ("Add problem4: minimal battery-only reactive-redescend test case").
