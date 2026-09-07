% ============================================================
% ProbLog -- CONTINUOUS-TIME / CONTINUOUS-SPACE moveto()
%
%   - obstacles: POLYGONS (auto-generated from a nav_msgs/
%     OccupancyGrid by occgrid_to_problog.py, see companion file
%     obstacles_generated.pl)
%   - nominal trajectory: a single cubic-Bezier SPLINE, given as
%     a control-point list, evaluated by CLOSED-FORM arithmetic
%     (no discretized time-steps in the action theory itself)
%   - noise: ONE Gaussian lateral-drift draw per walk instance
%     (Brownian-bridge-consistent: zero at the start, growing with
%     elapsed time), resolved as a situation-indexed probabilistic
%     fact -- NOT resampled per query
%   - policy: an EXPLICIT start/end ACTION PAIR
%         startMoveto(ControlPoints, Triggers, T0)  ...  haltMoveto(T,Reason)
%     marking the durative interval, with a BUSY fluent tracking
%     "walk currently in progress" between them. A generic
%     interrupt(T) action can end the walk EARLY (before natural
%     completion), so the walk can be preempted by another action
%     if needed. This is the genuine Reiter-style start/end
%     interval-action pattern (Ch.7), used here specifically
%     because we now want (a) other actions to be able to gate on
%     "is the robot currently walking", and (b) the ability to cut
%     a walk short. Position freezes at whatever point the walk
%     was interrupted -- it does NOT keep evolving toward the
%     original target after the walk has ended.
%   - startMoveto is a genuine TEMPLATE action: Triggers is the
%     COMPLETE list of halting conditions this leg reacts to --
%     collision, battery depletion, obstacle sighting, and any future
%     condition are ALL ordinary entries here, on identical footing.
%     NOTHING is hardcoded: Triggers=[] means the walk halts ONLY on
%     natural completion of its nominal duration, passing straight
%     through an obstacle's margin or running the battery dry without
%     ever noticing, if collision/battery aren't in this leg's own
%     Triggers list. There is NO default Triggers list anywhere in this
%     theory -- every moveto_leg(CP,Triggers) call states its own
%     Triggers explicitly, by design, so a plan's protection level is
%     always visible at the call site rather than inherited from
%     configuration. Different occurrences of moveto in a policy can
%     react to different conditions this way -- see plan/1 near the end
%     of this file for the shipped plan's own explicit choice
%     ([collision,battery]). Whichever entry in Triggers occurs
%     EARLIEST in a given resolved world (or natural completion, if
%     none does) determines Reason -- see the earliest-wins machinery
%     below Poss(haltMoveto(...)).
%   - fluents (at/4, moving/1) are PURE Reiter-style regressions over
%     the situation term S -- given (T,S) and a resolved possible
%     world, they are deterministic functions, never cached, never
%     re-sampled.
%   - safety queries are indexed by "which SAMPLED INSTANT along the
%     walk's ACTUAL executed span" (from T0 to whenever the walk
%     ended, naturally or via interrupt). Sampling is a
%     VERIFICATION-TIME choice (num_samples/1) completely decoupled
%     from the action theory and from the noise model's world count.
%   - FUTURE EXTENSION (not implemented yet, flagged where relevant):
%     time-varying speed. Currently speed/1 is a constant; a varying
%     speed would replace walk_duration's arc-length/speed division
%     with a numeric integral of dt = ds/speed(t) along the spline.
% ============================================================

% ---------------------------------------------------------------
% 0. PROBLEM-SPECIFIC DATA -- obstacles, tunable config, the BT policy,
%    and the goal formula ALL come from problem_data.pl, a small
%    AUTO-GENERATED bootstrap file living next to this one
%    (module/theory/problem_data.pl), rewritten by main.py on every
%    run to point (via absolute paths) at whichever problem was
%    selected (--problem NAME, default problem0; see
%    problems/<NAME>/ for that problem's own config.yaml,
%    behavior_tree.xml, goal_formula.pl, and map.yaml). Consulting ONE
%    fixed file here -- rather than hardcoding four problem-specific
%    paths into THIS file -- is what lets the same theory serve any
%    problem folder without ever being hand-edited itself.
%
%    problem_data.pl provides:
%      - obstacle_polygon/2   from obstacles_generated.pl, itself
%                              generated from map.yaml by
%                              module/translators/occgrid_to_problog.py.
%      - start/1, robot_radius/1, safety_buffer/1, speed/1, sigma/1,
%        sigma_tangential/1, sigma_battery/1, battery_start/1,
%        idle_drain_rate/1, moving_drain_rate/1,
%        tolerance/1, num_samples/1, bracket_samples/1, crossing_eps/1,
%        z/2, zt/2, zbatt/1, disc_step_position/1, disc_step_battery/1,
%        disc_step_time/1 (see the MERGE-GRID QUANTIZATION note above
%        dist/5's own section)
%                              from config_generated.pl, itself
%                              generated from config.yaml by
%                              module/translators/config_to_prolog.py.
%      - plan/1                from plan_generated.pl, itself
%                              translated + validated from
%                              behavior_tree.xml against
%                              module/contracts/schema.yaml by
%                              module/translators/bt_to_prolog.py.
%      - goal_formula/1        the problem's own hand-authored
%                              goal_formula.pl, validated against
%                              module/contracts/vocabulary.yaml by
%                              module/contracts/goal_formula_check.py.
%    main.py regenerates/validates all four automatically before every
%    run -- you never need to run a generator by hand for a normal run.
% ---------------------------------------------------------------
% Fallback clause so obstacle_polygon/2 is always a KNOWN predicate to
% ProbLog even if problem_data.pl's own obstacles_generated.pl defines
% zero real obstacles (its body always fails, so it never contributes
% an actual obstacle).
obstacle_polygon(no_obstacles_placeholder, []) :- fail.

:- consult('./problem_data.pl').
% Must exist relative to wherever this file itself lives (module/theory/),
% not CWD -- same resolution rule every consult/use_module directive in
% this file follows.

% planners.py provides plan_astar/5, plan_straight/5, plan_voronoi/5,
% and follow_boarder/5 as BLACK-BOX (Python-implemented) predicates --
% see that file's own header for the full explanation. ProbLog imports
% and executes it directly the moment this directive loads
% (problog.clausedb's load_external_module), registering both
% predicates before anything below that calls them is ever evaluated.
% Path is resolved relative to THIS file's own directory (not CWD) --
% planners.py lives right next to this file, in module/theory/.
:- use_module('./planners.py').

% collision_geometry.py provides first_threshold_crossing_time/8 as a
% BLACK-BOX (Python-implemented) predicate -- the obstacle-clearance
% geometry and bracket-scan/bisection crossing-time search that used to
% be plain Prolog in sections 1 and the TRIGGERS section below (see
% their own notes for why this moved). Lives right next to this file
% too, resolved relative to THIS file's own directory, same as
% planners.py.
:- use_module('./collision_geometry.py').

% ---------------------------------------------------------------
% 1. GEOMETRY HELPERS -- general-purpose arithmetic used throughout
%    the theory (distance, arc-length summation). The OBSTACLE-SPECIFIC
%    geometry that used to live here (point/segment/polygon distance,
%    ray-casting point-in-polygon, signed clearance, min-clearance-to-
%    any-obstacle) has been MOVED to collision_geometry.py, a Python
%    black box exactly like planners.py's planners -- see the
%    TRIGGERS section further down (first_threshold_crossing_time/8)
%    for why: that geometry is pure deterministic arithmetic once Z is
%    resolved, with no probabilistic content of its own, so ProbLog was
%    paying full SLD-grounding cost (a materialized proof node per
%    bracket sample, per bisection step, per obstacle vertex) for a
%    computation that has no bearing on the weighted model count beyond
%    its single Tcross answer. Moving it to native Python collapses
%    that whole grounding subtree into one black-box call per resolved
%    world -- see collision_geometry.py's own header for the full
%    rationale and for why its results are IDENTICAL to this file's
%    former Prolog implementation (same bracket-sample count, same
%    bisection epsilon, ported line-for-line, not re-derived).
% ---------------------------------------------------------------
dist(X1,Y1,X2,Y2,D) :- D is sqrt((X2-X1)**2 + (Y2-Y1)**2).

% ---------------------------------------------------------------
% MERGE-GRID QUANTIZATION -- three rounding primitives used ONLY at
% the specific "seam" where a NEW leg (a fresh startMoveto) reads its
% own starting condition from wherever the PREVIOUS leg's actual halt
% left things. See disc_step_position/1, disc_step_battery/1, and
% disc_step_time/1 (config facts, from the problem's own config.yaml
% -- see that file's own "grounding:" section) for what actually
% enables this, and leg_start_battery/3, poss(startMoveto(...)), and
% do_node(planWith(...)) further down for where each is applied.
%
% WHY: this project's own investigation (see FUTUREWORK.md and the
% conversation that produced this feature) found that ProbLog's
% grounder DOES merge multiple proofs of an identical ground atom into
% one shared formula node for free -- but a NEW leg's own control
% points (via planWith), starting battery, and starting time are each
% CONTINUOUS functions of the noise (Z,Zt,Zb) an EARLIER leg resolved,
% so they are essentially NEVER bit-identical across different worlds,
% and merging never actually happens on its own. Rounding these three
% quantities to a config-chosen grid, EXACTLY at the point a new leg
% reads them, makes different worlds that land close enough together
% produce the SAME ground term -- letting ProbLog's existing, exact
% sharing machinery do the compression, with no change to how anything
% is combined/weighted.
%
% WITHIN one leg, this changes NOTHING: Tcross (bracket-scan or
% closed-form battery algebra), position (walk_noisy_point), and
% battery drain (noisy_drain) are all still computed at FULL FLOAT
% PRECISION exactly as before this feature existed. Quantization
% happens EXACTLY ONCE per leg boundary -- on the value a leg reads as
% its OWN starting condition -- never mid-computation, and never on
% anything used for REPORTING (first_hit/on_track/verify_safe/
% halted_with/2 all still read the exact, un-rounded Tcross/position/
% battery of whichever leg actually produced them; only what SEEDS the
% NEXT leg is coarsened). cond() checks (holds(battery_over(...)),
% holds(distance_below(...)), etc.) also stay exact, on purpose -- a
% decision boundary like "is battery actually over 70%" should not be
% shifted by a floor/ceiling/round choice made for an unrelated reason.
%
% All three grids default to 0 in config.yaml (disabled): every
% quantize* predicate below passes its Value straight through
% UNCHANGED at Grid =< 0, so a problem that never sets these stays
% byte-for-byte identical to this feature not existing at all.
%
% Direction matters for two of the three, not the first:
%   quantize/3       -- round-to-NEAREST. Used for position: rounding
%                        either way is an equally valid approximation,
%                        no physical-consistency constraint.
%   quantize_down/3  -- FLOOR. Used for a new leg's own starting
%                        battery: never OVERESTIMATE remaining charge,
%                        the same "approximation must never look safer
%                        than reality" convention noisy_drain/3 above
%                        already follows (it clamps drain at 0 for
%                        exactly this reason).
%   quantize_up/3    -- CEILING. Used for a new leg's own start time:
%                        a leg can never plausibly begin BEFORE the
%                        previous one actually ended, so rounding down
%                        (or to nearest) could produce a non-causal
%                        T0 earlier than the real halt instant.
% ---------------------------------------------------------------
quantize(Value, Grid, Value) :- Grid =< 0.
quantize(Value, Grid, Quantized) :-
    Grid > 0,
    Quantized is round(Value / Grid) * Grid.

quantize_down(Value, Grid, Value) :- Grid =< 0.
quantize_down(Value, Grid, Quantized) :-
    Grid > 0,
    Quantized is floor(Value / Grid) * Grid.

quantize_up(Value, Grid, Value) :- Grid =< 0.
quantize_up(Value, Grid, Quantized) :-
    Grid > 0,
    Quantized is ceiling(Value / Grid) * Grid.

sum_list([], 0.0).
sum_list([H|T], Sum) :- sum_list(T, SumT), Sum is H + SumT.

% ---------------------------------------------------------------
% 2. ROBOT / SAFETY PARAMETERS
% ---------------------------------------------------------------
% robot_radius/1 and safety_buffer/1 are now config facts
% (the problem's own config.yaml -> config_generated.pl, consulted above)
% -- see that file for the tunable values themselves. safety_margin/1
% below is a DERIVED value, not a raw constant -- always recomputed
% from the two config facts, never itself config data.
%
% sight_threshold/1 and sight_threshold_valid are GONE: obstacle
% proximity used to be checked against ONE global sight_threshold
% constant (the old obstacle_sighted trigger); it is now
% obstacle_in_bound(Threshold), a genuinely per-call parameter (see the
% TRIGGERS section and holds(obstacle_in_bound(...)) below) -- there is
% no longer one global value to validate against safety_margin. NOT YET
% BUILT: a per-instance check (e.g. in module/translators/bt_to_prolog.py)
% that a given obstacle_in_bound(Threshold)'s Threshold is itself
% sensible (> safety_margin) -- currently unchecked, same as any other
% trigger-list entry's argument.
safety_margin(M) :- robot_radius(R), safety_buffer(B), M is R + B.

% within_obstacle_threshold/3 (the generalized "is (PX,PY) within
% Threshold of the nearest obstacle" test, parametrized by threshold so
% the SAME primitive serves collision, obstacle_in_bound, and any
% future distance-based trigger) now lives inside collision_geometry.py's
% within_obstacle_threshold helper, alongside the rest of the
% obstacle-clearance geometry it was
% moved with -- see the note above dist/5 in section 1.

% ---------------------------------------------------------------
% 3. SPLINE -- chained cubic Bezier segments. ControlPoints =
%    [point(X0,Y0), point(X1,Y1), point(X2,Y2), point(X3,Y3),
%    point(X4,Y4), ...], length must be 3k+1 for k segments (segment i
%    uses control points 3i..3i+3). A straight line is the degenerate
%    case where the interior control points are collinear with the
%    endpoints.
%
%    NO Bezier/spline arithmetic is implemented in Prolog any more --
%    it all lives in exactly ONE place, collision_geometry.py's own
%    _walk_noisy_point (and the walk_noisy_point/8 black-box predicate
%    it backs, registered by the :- use_module('./collision_geometry.py')
%    directive in Section 0, same as first_threshold_crossing_time/8
%    and friends). This file used to carry its OWN, separate
%    reimplementation (bezier_point/tangent, spline_point/tangent,
%    perp_unit, tangent_unit) that had to be kept "identical" to
%    collision_geometry.py's copy by discipline, not by construction --
%    a real duplication/drift risk, now removed: walk_noisy_point/8
%    below IS the foreign predicate (no Prolog clause of that name
%    exists here), and spline_point/4 is a two-line wrapper calling
%    the SAME predicate with zero deviation.
% ---------------------------------------------------------------

% spline_point(+ControlPoints, +U, -X, -Y): U in [0,1] spans the WHOLE
% spline -- the deterministic/nominal point, no noise. Delegates to
% walk_noisy_point/8 with Z=0.0, Zt=0.0 (no deviation) and T0=0.0,
% Duration=1.0, T=U (so Frac=U exactly) -- the SAME underlying
% arithmetic walk_noisy_point/8 itself uses, evaluated at zero
% deviation rather than a second, independent implementation.
spline_point(ControlPoints, U, X, Y) :-
    walk_noisy_point(ControlPoints, 0.0, 1.0, 0.0, 0.0, U, X, Y).

% arc length via one-time numeric integration (deterministic,
% computed ONCE per distinct ControlPoints -- not a random draw,
% not re-simulated per query; this is exactly the "approximate"
% arc-length option discussed: U is treated as advancing linearly
% with elapsed-time fraction, i.e. speed is only approximately
% constant along strongly-curved segments)
arc_length(ControlPoints, Length) :-
    ArcSamples = 50,
    ArcSamplesHi is ArcSamples - 1,
    findall(D,
        ( between(0, ArcSamplesHi, I),
          U0 is I / ArcSamples, U1 is (I+1) / ArcSamples,
          spline_point(ControlPoints, U0, X0,Y0),
          spline_point(ControlPoints, U1, X1,Y1),
          dist(X0,Y0,X1,Y1,D)
        ), Ds),
    sum_list(Ds, Length).

% speed/1 is now a config fact -- see the problem's own config.yaml's motion.speed.

walk_duration(ControlPoints, Duration) :-
    arc_length(ControlPoints, Length),
    speed(Speed),
    Duration is Length / Speed.

% ---------------------------------------------------------------
% 4. STOCHASTIC LATERAL DRIFT -- ONE discretized-Gaussian draw
%    per WALK INSTANCE (i.e. per startMoveto occurrence), resolved
%    as a situation-indexed probabilistic fact (never resampled).
%    Deviation grows with elapsed-time fraction of the ORIGINAL
%    (full) walk duration, Brownian-bridge style: 0 at the start,
%    sigma*sqrt(Duration) (in std-dev units) at natural completion.
%    If the walk is cut short by an interrupt, the deviation simply
%    stops growing wherever it was at the moment of interruption --
%    it does not "reset" or get resampled.
% ---------------------------------------------------------------
% z/2's annotated-disjunction table and sigma/1 (its scale) are now
% config facts -- see the problem's own config.yaml's
% position.lateral.discretized_gaussian and position.lateral.sigma,
% generated into config_generated.pl (consulted at the top of this
% file) by module/translators/config_to_prolog.py. NOTE kept here as a
% standing reminder even though the table itself moved: these weights
% MUST sum to EXACTLY 1.0 -- an earlier version of this table used
% 0.3854 for the centre weight where 0.3954 was required, which summed
% to only 0.99, and ProbLog silently treats missing mass as an
% implicit "none of these" failure branch, which caps EVERY downstream
% probability at 0.99 in every world. The generator checks each
% table's own sum and warns if it's off; always verify it after
% editing config.yaml regardless.

% ---------------------------------------------------------------
% 5. THE MOVING FLUENT -- true while a walk is in progress (between
%    startMoveto and whatever action ends it: haltMoveto OR
%    interrupt). Other actions can gate their Poss axioms on
%    \+ moving(S) (can't start a new walk while moving) or on moving(S)
%    (an interrupt can only fire while a walk IS in progress).
%    Pure regression, exactly on the same footing as at/4.
% ---------------------------------------------------------------
moving(do(startMoveto(_,_,_,_), _)).
moving(do(A,S)) :-
    A \= haltMoveto(_,_,_), A \= interrupt(_),
    moving(S).
% (s0 is not moving: no clause covers it, so moving(s0) correctly fails)

% current_walk(+S, -ControlPoints, -T0): the control points and
% start time of the MOST RECENT startMoveto in S's history. Used
% whenever moving(S) holds (i.e. there is exactly one open walk to
% find), and also to recover the just-closed walk's parameters
% right after an haltMoveto/interrupt action.
current_walk(S, CP, T0) :- current_walk(S, CP, _Triggers, T0, _SPrev).

% current_walk/4 additionally exposes SPrev, the situation
% IMMEDIATELY BEFORE the startMoveto occurred -- needed to look up
% that walk's resolved noise draw Z via
% z(do(startMoveto(CP,Triggers,T0),SPrev),Z), since z/2's key is the
% exact ground startMoveto action term, which includes the situation
% it was added to (and, now, the Triggers list it was called with).
current_walk(S, CP, T0, SPrev) :- current_walk(S, CP, _Triggers, T0, SPrev).

% current_walk/5 additionally exposes Triggers -- the leg's own list
% of EXTRA halting conditions -- needed wherever the earliest-wins
% computation over Triggers has to run (Poss(haltMoveto(...)),
% interrupt's Poss). UNCHANGED signature/behaviour for every one of
% its own (many) existing callers -- now a thin wrapper dropping
% ActionCode from current_walk/6 below, exactly the same "/3 and /4
% are thin wrappers" pattern as when SPrev was added to /3 earlier.
current_walk(S, CP, Triggers, T0, SPrev) :-
    current_walk(S, CP, Triggers, _ActionCode, T0, SPrev).

% current_walk/6 -- the REAL base fact/recursion, over startMoveto/4
% (CP,Triggers,ActionCode,T0) now instead of startMoveto/3. ActionCode
% is the per-MoveTo-OCCURRENCE code bt_to_prolog.py assigns (mirrors
% next_reactive_code()'s own per-reactive-composite code -- see that
% file's own _VarPool note), embedded into the action term by
% poss(startMoveto(...)) below and carried straight through by do_node
% (moveto_leg(...)) -- ONLY poss(haltMoveto(...)) actually needs the
% real value (to tag the final halt Reason with it -- see tag_reason/3
% further down); every other current_walk/5 caller is unaffected.
current_walk(do(startMoveto(CP,Triggers,ActionCode,T0),SPrev), CP, Triggers, ActionCode, T0, SPrev).
current_walk(do(A,S), CP, Triggers, ActionCode, T0, SPrev) :-
    A \= startMoveto(_,_,_,_),
    current_walk(S, CP, Triggers, ActionCode, T0, SPrev).

% ---------------------------------------------------------------
% 4b. THE BATTERY FLUENT -- a second clock fluent, on the exact same
%     footing as at/4: battery(Level,T,S) is the charge level (0..100,
%     percent) at real clock-time T in situation S.
%
%     - starts at 100 in s0
%     - drains LINEARLY in time, at a rate that depends on whether the
%       robot is moving or idle (idle_drain_rate/1 while \+ moving(S),
%       moving_drain_rate/1 while moving(S))
%     - UNLIKE position, battery does NOT freeze after a halt/interrupt
%       -- it keeps draining at the idle rate for as long as the robot
%       sits still afterwards. Position freezing (at/4) and battery's
%       continued idle drain are genuinely different physical
%       behaviours, so they need different regression clauses here,
%       even though both are "clock fluents" in the same formal sense.
%     - stochastic: ONE extra discretized-Gaussian draw per walk
%       instance (zbatt/2, mirroring z/2's role for position), added
%       to the moving drain rate. Because the resulting rate is still
%       CONSTANT over a given walk (the noise is a single per-walk
%       draw, not a fresh draw per instant), battery level stays
%       EXACTLY LINEAR in elapsed time within one walk -- so, unlike
%       collision (which needs bracket-scan + bisection because it's a
%       cubic-spline-vs-polygon test with no closed form), the exact
%       depletion time is solvable by plain algebra. See
%       first_battery_depletion_time/6 below.
% ---------------------------------------------------------------
% battery_start/1, idle_drain_rate/1, moving_drain_rate/1, and
% sigma_battery/1 (the SAME value used in every phase -- moving, idle,
% and s0 -- see zbatt/1 below) are now config facts -- see
% the problem's own config.yaml's battery.* (battery.sigma for
% sigma_battery/1 specifically -- everything battery-related, noise
% included, lives in that one section now).

% zbatt/1: ONE noise draw for the WHOLE MISSION -- deliberately
% decoupled from any specific startMoveto occurrence. This is an
% annotated disjunction with NO ARGUMENTS at all, which ProbLog
% grounds EXACTLY ONCE for the entire program: every reference to
% zbatt(Zb), anywhere in the theory, in every world, refers to the
% SAME single resolved value. This treats "how much this particular
% battery underperforms" as a persistent property of the battery
% itself -- present from the very start, not a fresh, independent
% draw manufactured at each walk. Its annotated-disjunction table is
% now ALSO a config fact -- the SAME noise.discretized_gaussian entries
% as z/2 (see config.yaml), instantiated a second time with zbatt/1's
% own zero-argument functor by module/translators/config_to_prolog.py, so the
% two tables can never drift apart.

% s0's idle phase now ALSO carries genuine stochasticity, using the
% SAME global Zb -- consistent with every other phase, no more
% special-cased determinism here. Uses sqrt(Elapsed) scaling (true
% Wiener-consistent growth, Var(Deviation)=sigma^2*Elapsed) rather
% than the Duration-normalized form used below: at s0 there is no
% walk yet (past OR upcoming) to borrow a reference Duration from,
% and idle time here is genuinely UNBOUNDED (T can be arbitrarily
% large before the first action ever fires -- you noted the first
% action can be scheduled to start after some delay, not just at
% T=0), so there is nothing else to normalize against.
% noisy_drain(+NominalDrain, +Deviation, -TotalDrain): the SHARED,
% MONOTONICITY-SAFE combination used by every phase below. Deviation
% > 0 means "drained LESS than nominal" (a well-performing battery
% this episode); Deviation < 0 means "drained MORE". Clamped at 0 so
% TotalDrain can NEVER be negative -- guaranteeing Level = B_start -
% TotalDrain never EXCEEDS B_start, i.e. battery level is
% monotonically NON-INCREASING over time, BY CONSTRUCTION, regardless
% of how large Deviation gets or which parameters are chosen. In the
% extreme case (noise would suggest giving back more than the full
% nominal drain), the physically sensible floor is "the battery
% simply doesn't drain during this stretch" -- not "the battery
% gains charge", which unclamped additive-to-LEVEL noise could
% otherwise produce for small Elapsed (sqrt(Elapsed) growing faster
% than the linear IdleRate*Elapsed term near Elapsed=0).
noisy_drain(NominalDrain, Deviation, TotalDrain) :-
    TotalDrain is max(0, NominalDrain - Deviation).

% ALL THREE idle-phase clauses (s0, after-halt, after-interrupt) now
% share the IDENTICAL structure: sqrt(Elapsed) scaling, no Duration
% reference at all -- idle time is genuinely UNBOUNDED in every one
% of these cases (T can be arbitrarily large before the first action
% ever fires, or arbitrarily long after any halt/interrupt), so
% there is nothing walk-specific to normalize against, and using the
% SAME formula everywhere removes the earlier inconsistency where
% walk-adjacent idle borrowed the preceding walk's Duration while s0
% did not, despite both being the same kind of unbounded stretch.
battery(Level, T, s0) :-
    battery_start(B0),
    idle_drain_rate(IdleRate),
    sigma_battery(SigmaB),
    zbatt(Zb),
    Elapsed is max(0.0, T),
    Deviation is Zb * SigmaB * sqrt(Elapsed),
    NominalDrain is IdleRate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B0 - TotalDrain)).

% leg_start_battery(+T0, +SPrev, -B0): a NEW leg's own starting
% battery level, rounded DOWN to disc_step_battery/1's own
% granularity (see the MERGE-GRID QUANTIZATION note above dist/5's own
% section) -- the SINGLE shared definition used both here (the MOVING-
% phase clause just below) and by poss(haltMoveto(...))/poss(interrupt
% (...)) further down, so a leg's own crossing-time computation and its
% own battery/3 regression can never read a different B0 for the same
% leg. Quantizing HERE -- exactly where a NEW leg's own physics starts
% -- is what keeps every OTHER battery/3 read (idle-phase reporting,
% mid-walk queries, first_hit/on_track/verify_safe) at full float
% precision, unaffected: only the value that seeds a brand new leg is
% coarsened, once, at the seam.
leg_start_battery(T0, SPrev, B0) :-
    battery(B0Exact, T0, SPrev),
    disc_step_battery(Grid),
    quantize_down(B0Exact, Grid, B0).

% MOVING phase keeps the Duration-normalized scaling (Elapsed/sqrt(D),
% not sqrt(Elapsed)) -- a walk DOES have a genuine, known, fixed
% Duration, and that normalization is what keeps Level EXACTLY LINEAR
% in elapsed time (needed for first_battery_depletion_time's
% closed-form algebraic solve, rather than bracket-scan+bisection).
% Structurally this is now the SAME "nominal drain minus a signed
% deviation, clamped at zero" pattern as the idle phases -- only the
% Deviation formula's normalization differs, for the reason above.
battery(Level, T, do(startMoveto(CP,_Triggers,_ActionCode,T0), S)) :-
    leg_start_battery(T0, S, B0),
    walk_duration(CP, Duration),
    Elapsed0 is T - T0,
    Elapsed is max(0.0, min(Elapsed0, Duration)),
    moving_drain_rate(MovingRate),
    sigma_battery(SigmaB),
    zbatt(Zb),
    Deviation is Zb * SigmaB * Elapsed / sqrt(Duration),
    NominalDrain is MovingRate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B0 - TotalDrain)).

% after a halt/interrupt: for T at or before the halt, delegate
% straight through (same value as during the walk); for T after it,
% the walk's own value at the halt instant becomes a new anchor and
% IDLE drain resumes from there -- this is the "does not freeze"
% behaviour that distinguishes battery from at/4. No current_walk/
% Duration lookup is needed here anymore -- see the note above.
battery(Level, T, do(haltMoveto(T1,Reason,Status), S)) :-
    T =< T1,
    battery(Level, T, S).
battery(Level, T, do(haltMoveto(T1,Reason,Status), S)) :-
    T > T1,
    battery(B1, T1, S),
    idle_drain_rate(IdleRate),
    sigma_battery(SigmaB),
    zbatt(Zb),
    Elapsed is T - T1,
    Deviation is Zb * SigmaB * sqrt(Elapsed),
    NominalDrain is IdleRate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B1 - TotalDrain)).

battery(Level, T, do(interrupt(T1), S)) :-
    T =< T1,
    battery(Level, T, S).
battery(Level, T, do(interrupt(T1), S)) :-
    T > T1,
    battery(B1, T1, S),
    idle_drain_rate(IdleRate),
    sigma_battery(SigmaB),
    zbatt(Zb),
    Elapsed is T - T1,
    Deviation is Zb * SigmaB * sqrt(Elapsed),
    NominalDrain is IdleRate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B1 - TotalDrain)).

% pass-through: any future non-movement action doesn't change how
% battery is computed -- it's a pure function of T and of whichever
% startMoveto/haltMoveto/interrupt anchors exist in the history, same
% principle as at/4's own pass-through clause.
battery(Level, T, do(A,S)) :-
    A \= startMoveto(_,_,_,_), A \= haltMoveto(_,_,_), A \= interrupt(_),
    battery(Level, T, S).

% first_battery_depletion_time(+CP,+T0,+Duration,+B0,+Zb,-Tcross):
% CLOSED-FORM (not bracket/bisect) -- battery is exactly LINEAR in
% elapsed time within one walk (fixed Zb => fixed EffectiveRate), so
% the crossing at Level=0 is a direct algebraic solve. FAILS (no
% clause matches) if the effective rate is non-positive (battery
% isn't actually decreasing -- noise happened to push it flat/up) or
% the algebraic crossing falls beyond this walk's own span -- both
% correctly represent "no depletion in this walk," exactly mirroring
% how first_collision_time fails when there's no crossing.
first_battery_depletion_time(CP,T0,Duration,B0,Zb,Tcross) :-
    moving_drain_rate(MovingRate),
    sigma_battery(SigmaB),
    EffectiveRate is MovingRate - Zb*SigmaB/sqrt(Duration),
    EffectiveRate > 0,
    Tcross0 is T0 + B0/EffectiveRate,
    Tcross0 =< T0 + Duration,
    Tcross = Tcross0.

% first_battery_below_time(+CP,+T0,+Duration,+B0,+Zb,+Threshold,-Tcross):
% the SAME closed-form algebra as first_battery_depletion_time above,
% generalized to an arbitrary Threshold instead of hardcoded Level=0 --
% this is what battery_below(Threshold) (see TRIGGERS section and
% holds(battery_below(...)) further down) uses, kept as a genuinely
% SEPARATE predicate from first_battery_depletion_time/6 (not a
% generalize-in-place rename) since battery_depleted (Level=0 exactly)
% stays its own distinct trigger/Reason, on request. UNLIKE
% first_battery_depletion_time, B0 can legitimately already be AT OR
% BELOW an arbitrary Threshold at T0 (e.g. an earlier leg already
% drained below a later leg's chosen warning level without itself
% using battery_below) -- so this needs the explicit "already true at
% T0" graceful clause every other threshold-crossing predicate in this
% theory already has, rather than relying on the algebra to degrade
% gracefully on its own (it only does that at exactly Threshold=0).
first_battery_below_time(CP,T0,Duration,B0,Zb,Threshold,T0) :-
    B0 =< Threshold.
first_battery_below_time(CP,T0,Duration,B0,Zb,Threshold,Tcross) :-
    B0 > Threshold,
    moving_drain_rate(MovingRate),
    sigma_battery(SigmaB),
    EffectiveRate is MovingRate - Zb*SigmaB/sqrt(Duration),
    EffectiveRate > 0,
    Tcross0 is T0 + (B0-Threshold)/EffectiveRate,
    Tcross0 =< T0 + Duration,
    Tcross = Tcross0.

% first_battery_equal_time(+CP,+T0,+Duration,+B0,+Zb,+Threshold,-Tcross):
% the SAME closed-form algebra again, this time for Level(T) = Threshold
% EXACTLY. Battery is a continuous, monotonically non-increasing
% function of T within one walk (see noisy_drain/3's own note), so for
% any Threshold strictly between the walk's start and end level there
% is EXACTLY ONE instant it passes through it -- the same Tcross0
% formula as first_battery_below_time above, not a separate derivation.
% UNLIKE first_battery_below_time, there is NO "already true" grace
% clause for B0 < Threshold: if the battery is already BELOW Threshold
% at T0, it was EQUAL to it at some earlier, already-elapsed instant
% not covered by this walk -- it can only keep draining further away
% from Threshold from here, so this correctly FAILS (no clause matches)
% rather than firing at T0. B0 = Threshold exactly is its own graceful
% "already true" case (Tcross=T0), same convention as everywhere else.
first_battery_equal_time(CP,T0,Duration,B0,Zb,Threshold,T0) :-
    B0 =:= Threshold.
first_battery_equal_time(CP,T0,Duration,B0,Zb,Threshold,Tcross) :-
    B0 > Threshold,
    moving_drain_rate(MovingRate),
    sigma_battery(SigmaB),
    EffectiveRate is MovingRate - Zb*SigmaB/sqrt(Duration),
    EffectiveRate > 0,
    Tcross0 is T0 + (B0-Threshold)/EffectiveRate,
    Tcross0 =< T0 + Duration,
    Tcross = Tcross0.

% first_battery_over_time(+CP,+T0,+Duration,+B0,+Zb,+Threshold,-Tcross):
% Same TWO-CLAUSE shape as first_battery_below_time/first_battery_equal_time
% (an "already true at T0" clause plus a general future-crossing search),
% not a bespoke single-fact shortcut, so this stays a genuinely VALID,
% extensible check even though battery/3 only ever DRAINS today.
%
% Clause 1 (below): B0 > Threshold already at the walk's own start --
% fires immediately, exactly like the other two predicates' own
% "already true" clauses.
%
% Clause 2 (a genuine future-crossing search) is NOT YET WRITTEN, and
% deliberately so, rather than filled in with a guessed formula: under
% the CURRENT battery/3 regression, noisy_drain/3 clamps TotalDrain at
% max(0, ...), so Level is PROVABLY non-increasing for the whole walk
% (Level(T) <= B0 for all T) -- there is no future instant where
% Level(T) > Threshold unless it was already true at T0, so a second
% clause could only ever correctly FAIL today; there is nothing for it
% to compute. Writing a "symmetric to first_battery_below_time" formula
% now (dividing by a negative EffectiveRate to predict a future rise)
% would NOT be a safe no-op -- EffectiveRate < 0 under the CURRENT
% clamp means "flat, no drain this whole walk", not "charging", so that
% formula would predict a Tcross the actual battery/3 fluent
% contradicts (still flat, never risen) -- a genuine correctness bug,
% not just unreachable code, the moment a Zb draw makes EffectiveRate
% negative. If/when battery/3's regression is extended to model real
% recharging, add clause 2 here using WHATEVER rate formula that
% extension introduces (mirroring how clause 2 of
% first_battery_below_time/first_battery_equal_time use
% moving_drain_rate/sigma_battery today) -- CP/Duration/Zb are already
% threaded through, genuinely used (not wildcarded away), specifically
% so that clause can be added without changing this predicate's
% signature or any of its callers.
first_battery_over_time(CP,T0,Duration,B0,Zb,Threshold,T0) :-
    B0 > Threshold.

% battery_at_leg(+T0,+Duration,+Zb,+B0,+T,-Level): Level(T) during ONE
% moveto leg, as a function of T alone -- the EXACT SAME formula as
% battery/3's own do(startMoveto(...),S) clause above (leg_start_
% battery(T0,S,B0) resolved to a plain value, not re-derived from S),
% just without S, for use by holds_leg/9 further down (which only ever
% has CP/T0/Duration/Z/Zt/Zb/B0 on hand -- the same flat signature
% every trigger_crossing_time/11 clause already receives, not a full
% situation term). MUST be kept in sync BY HAND with battery/3's own
% do(startMoveto(...),S) clause -- there is no single shared definition
% because that clause additionally resolves B0 from S via
% leg_start_battery/3, which this one takes already-resolved.
battery_at_leg(T0,Duration,Zb,B0,T,Level) :-
    Elapsed0 is T - T0,
    Elapsed is max(0.0, min(Elapsed0, Duration)),
    moving_drain_rate(MovingRate),
    sigma_battery(SigmaB),
    Deviation is Zb * SigmaB * Elapsed / sqrt(Duration),
    NominalDrain is MovingRate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B0 - TotalDrain)).

% ---------------------------------------------------------------
% TRIGGERS -- the TEMPLATE mechanism. A leg's Triggers argument is
% the COMPLETE list of halting conditions this leg reacts to --
% collision, battery depletion, obstacle sighting, and any future
% condition are ALL ordinary entries here, on identical footing.
% There is NO hardcoded always-on cause anymore: Triggers=[] means
% the walk halts ONLY on natural completion of its nominal duration
% -- it will pass straight through an obstacle's safety margin, or
% run the battery to empty and beyond, without halting for either,
% if neither `collision` nor `battery` is in its own Triggers list.
% Each recognized trigger name has ONE clause of
% trigger_crossing_time/9 below, giving its own crossing-time AND
% Reason -- the "standard interface" every trigger type must supply:
% given the walk's parameters and the resolved noise, produce a
% (Reason, crossing time) pair, or fail if it never fires. Adding a
% new trigger type later means adding ONE more clause here; nothing
% else in this file needs to change.
%
% Reason is a SEPARATE output from the trigger name itself for
% collision/battery specifically, so the Reason atoms every other
% part of the theory already keys on (crashed(ObstacleId),
% battery_depleted -- via halted_with/2, crashed_in/1,
% battery_depleted_in/1, first_hit/1, hit_by/1, etc.) stay STRUCTURALLY
% as they were; only how those Reasons get triggered changed, not what
% they're called. crashed carries WHICH obstacle (crashed(ObstacleId))
% rather than being a bare atom, since first_collision_time/6 now
% returns the crossed obstacle's own obstacle_polygon/2 Id alongside
% Tcross -- see collision_geometry.py's own header for where that
% argmin actually happens. battery_depleted stays a bare atom: draining
% isn't tied to any specific obstacle. Matching "any crash regardless
% of which obstacle" now needs crashed(_), not bare crashed -- see the
% TODO note near holds(halted_with_cond(...)) for the one place this is
% user-facing.
%
% collision (fixed threshold=safety_margin) and battery (fixed
% threshold=exactly-0, Reason=battery_depleted) are the ORIGINAL,
% UNPARAMETRIZED trigger names -- kept exactly as they were, on
% request, rather than folded into the generic versions below.
% obstacle_in_bound(Threshold) and battery_below(Threshold) are
% GENUINELY SEPARATE, ADDITIONAL trigger names -- a leg can react to
% EITHER or BOTH of a fixed floor and an arbitrary per-call threshold
% at once, e.g. Triggers=[collision,battery,battery_below(20)] halts on
% whichever of "hits an obstacle", "hits exactly empty", or "drops
% under 20%" happens earliest. obstacle_in_bound is what "obstacle
% sighted" was renamed to (see the note above first_threshold_crossing_
% time below for why sight_threshold/1 is gone): it reuses
% first_threshold_crossing_time DIRECTLY, with Threshold now the
% CALLER'S OWN argument instead of a fixed config constant -- the exact
% same black box collision already used, no new machinery. Reason
% carries Threshold too, not just ObstacleId (unlike collision's
% crashed(ObstacleId)) -- so two obstacle_in_bound(...) triggers at
% different thresholds in the same Triggers list stay distinguishable
% by which one actually fired. battery_below(Threshold) is the SAME
% relationship to battery: reuses first_battery_below_time (the
% Threshold-generalized twin of first_battery_depletion_time, see that
% predicate's own note), Reason battery_under(Threshold) -- a
% DIFFERENT word from the trigger name, by request, mirroring
% collision/crashed's own asymmetry.
%
% battery_equal(Threshold) and battery_over(Threshold) are two more
% additional, SEPARATE battery triggers, same family as battery_below.
% battery_equal(Threshold) fires at the exact instant Level(T)=Threshold
% (no "already below" grace -- see first_battery_equal_time's own
% note); Reason keeps the SAME functor as the trigger (battery_equal),
% not a distinct word, since no distinct Reason was requested for these
% two (unlike battery_below/battery_under). battery_over(Threshold)
% currently only ever fires "already true at T0" or never, since
% battery never increases within a walk TODAY -- but its predicate is
% structured with the SAME two-clause shape as battery_below/
% battery_equal, ready for a genuine future-crossing search once
% battery/3 models real recharging -- see first_battery_over_time's
% own note for exactly why a shortcut formula can't safely be
% written in ahead of that.
%
% obstacle_on_path(Threshold) is a DIFFERENT geometric test from
% obstacle_in_bound(Threshold), not another threshold value for the
% same one: obstacle_in_bound asks "is the CURRENT position close to
% ANY obstacle's BOUNDARY", regardless of whether the trajectory ever
% actually enters that obstacle (a path can graze within safety_margin
% of an obstacle's edge -- e.g. because safety_margin already includes
% the robot's own radius -- without the robot's own center ever being
% geometrically INSIDE the polygon). obstacle_on_path instead asks "is
% the CURRENT position close to an obstacle the trajectory ACTUALLY
% ENTERS (goes geometrically inside, not just near) somewhere across
% this WHOLE walk" -- confirmed by direct test to genuinely differ:
% at Z=-1.0 and Z=+2.0 in the real 12-obstacle map, the trajectory
% comes within safety_margin of an obstacle's boundary (obstacle_in_bound
% WOULD fire) but never actually enters it (obstacle_on_path does NOT).
% Reuses first_threshold_crossing_time/obstacle_within_threshold
% UNCHANGED, just called (inside collision_geometry.py) against a
% obstacle set FILTERED to "obstacles this trajectory enters somewhere"
% -- see collision_geometry.py's own header, "ON PATH" section.
%
% obstacle_in_bound(Threshold), obstacle_on_path(Threshold),
% battery_below(Threshold), battery_equal(Threshold), and
% battery_over(Threshold) are ALSO directly usable as cond() leaves,
% checking the CURRENT situation instead of searching a future walk --
% see holds(obstacle_in_bound(...)), holds(obstacle_on_path(...)),
% holds(battery_below(...)), holds(battery_equal(...)), and
% holds(battery_over(...)) further down, which reuse the exact same
% underlying primitives (obstacle_within_threshold/
% obstacle_on_path_within_threshold/battery/3) a single time instead of
% across a bracket-scanned trajectory.
%
% line_of_sight_clear(ObstacleId,GX,GY) and crosses_segment(SX,SY,GX,GY)
% are the Bug-algorithm boundary-LEAVE triggers, for use on a MoveTo leg
% whose ControlPoints came from planners.py's follow_boarder
% (ObstacleId,Offset) planner (a full clockwise loop around the
% obstacle's offset boundary, no stopping logic of its own -- see that
% predicate's own header). Which bug variant a leg implements is
% entirely a matter of WHICH of these two names its own Triggers list
% carries -- line_of_sight_clear for Bug0 (fires as soon as ObstacleId
% stops occluding a straight line to (GX,GY)), crosses_segment for
% Bug2 (fires when the boundary walk re-crosses the straight segment
% from (SX,SY) -- wherever the leg's own circling began -- to (GX,GY),
% at a point strictly closer to goal than (SX,SY) was; see
% collision_geometry.py's own "BUG-ALGORITHM BOUNDARY-LEAVE PRIMITIVES"
% section for exactly why that distance condition is part of the
% definition). line_of_sight_clear(ObstacleId,GX,GY) is ALSO usable as
% a cond() leaf (holds(line_of_sight_clear(...)) further down);
% crosses_segment deliberately is NOT -- see collision_geometry.py's
% own note on why "has my trajectory crossed this segment" doesn't
% have a meaningful point-in-time reading the way the others do.
% ---------------------------------------------------------------
% trigger_crossing_time/11's LAST argument, Code, is the REDESCEND-
% TARGET CODE this specific trigger OCCURRENCE was tagged with by
% bt_to_prolog.py at translation time -- see the CONTROL-FLOW
% REDESCEND TARGETS note above do_node(reactivesequence(...)) further
% down for the full mechanism. collision/battery (never reactive --
% they classify straight to false in leg_status, never to a
% reactive(Code) status) always produce Code=none, since nothing ever
% reads it for them. Every genuinely reactive-classified trigger name
% below (obstacle_in_bound, obstacle_on_path, battery_below,
% battery_equal, battery_over, line_of_sight_clear, crosses_segment)
% now takes Code as an EXTRA trailing argument in the Triggers list
% ITSELF (e.g. battery_below(70,rc3), not battery_below(70)) -- bt_to_
% prolog.py appends it automatically to every reactive trigger token,
% based on which ReactiveSequence/ReactiveFallback (if any) currently
% encloses that leg, so nothing about how a BT.xml author writes
% triggers="..." needs to change; Code just rides along as a plain
% INPUT here, unify-passed straight through to the OUTPUT Reason-Time
% pair's own third slot (Reason's own shape, e.g. battery_under
% (Threshold), is UNCHANGED -- Code is carried ALONGSIDE it, not
% embedded inside it, so halted_with/2 and everything built on it
% keeps working exactly as before).
trigger_crossing_time(collision, CP,T0,Duration,Z,Zt,_Zb,_B0, crashed(ObstacleId), Tcross, none) :-
    first_collision_time(CP,T0,Duration,Z,Zt,Tcross,ObstacleId).

trigger_crossing_time(battery, CP,T0,Duration,_Z,_Zt,Zb,B0, battery_depleted, Tcross, none) :-
    first_battery_depletion_time(CP,T0,Duration,B0,Zb,Tcross).

trigger_crossing_time(obstacle_in_bound(Threshold,Code), CP,T0,Duration,Z,Zt,_Zb,_B0, obstacle_in_bound(Threshold,ObstacleId), Tcross, Code) :-
    first_threshold_crossing_time(CP,T0,Duration,Z,Zt,Threshold,Tcross,ObstacleId).

trigger_crossing_time(obstacle_on_path(Threshold,Code), CP,T0,Duration,Z,Zt,_Zb,_B0, obstacle_on_path(Threshold,ObstacleId), Tcross, Code) :-
    first_on_path_crossing_time(CP,T0,Duration,Z,Zt,Threshold,Tcross,ObstacleId).

trigger_crossing_time(battery_below(Threshold,Code), CP,T0,Duration,_Z,_Zt,Zb,B0, battery_under(Threshold), Tcross, Code) :-
    first_battery_below_time(CP,T0,Duration,B0,Zb,Threshold,Tcross).

trigger_crossing_time(battery_equal(Threshold,Code), CP,T0,Duration,_Z,_Zt,Zb,B0, battery_equal(Threshold), Tcross, Code) :-
    first_battery_equal_time(CP,T0,Duration,B0,Zb,Threshold,Tcross).

trigger_crossing_time(battery_over(Threshold,Code), CP,T0,Duration,_Z,_Zt,Zb,B0, battery_over(Threshold), Tcross, Code) :-
    first_battery_over_time(CP,T0,Duration,B0,Zb,Threshold,Tcross).

trigger_crossing_time(line_of_sight_clear(ObstacleId,GX,GY,Code), CP,T0,Duration,Z,Zt,_Zb,_B0, line_of_sight_clear(ObstacleId,GX,GY), Tcross, Code) :-
    first_line_of_sight_clear_time(CP,T0,Duration,Z,Zt,ObstacleId,GX,GY,Tcross).

trigger_crossing_time(crosses_segment(SX,SY,GX,GY,Code), CP,T0,Duration,Z,Zt,_Zb,_B0, crosses_segment(SX,SY,GX,GY), Tcross, Code) :-
    first_segment_crossing_time(CP,T0,Duration,Z,Zt,SX,SY,GX,GY,Tcross).

% guard_break(Cond,Code): the GENERIC, AUTOMATICALLY-DERIVED trigger
% bt_to_prolog.py's guard-derivation pass emits for a MoveTo sitting
% under a ReactiveSequence/ReactiveFallback ancestor with a Condition
% left sibling (see that file's own CONTROL-FLOW GUARD DERIVATION
% note) -- Cond is ALREADY the exact term that must stay TRUE for the
% guard to keep holding: the bare condition itself for a
% ReactiveSequence-style "left siblings must all SUCCEED" guard, or
% neg(Condition) for a ReactiveFallback-style "left siblings must all
% FAIL" guard (bt_to_prolog.py builds this by NEGATING the actual
% condition per the required polarity -- and per any <Inverter> in the
% chain -- rather than by looking up a pre-built "opposite" trigger
% name, so this works uniformly for every condition in schema.yaml's
% conditions: list with no per-condition crossing-direction table to
% keep in sync or leave a gap in). Fires at the first instant Cond
% stops holding -- see first_becomes_false_time/9 and holds_leg/9
% further down (right after holds/2's own condition clauses, which
% holds_leg/9 mirrors).
trigger_crossing_time(guard_break(Cond,Code), CP,T0,Duration,Z,Zt,Zb,B0, guard_break(Cond), Tcross, Code) :-
    first_becomes_false_time(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Tcross).

% all_trigger_candidates(+Triggers,...,-Candidates): Candidates is a
% list of Reason-Time-Code TRIPLES now (was Reason-Time pairs), one
% per trigger in Triggers that ACTUALLY fires in this resolved world
% (triggers that don't fire contribute nothing -- same "absence, not
% sentinel" convention as everywhere else). Unrecognized trigger names
% (no matching trigger_crossing_time/11 clause) are silently skipped,
% same as "never fires" -- lenient by design, so a typo in a Triggers
% list doesn't halt the whole theory, just means that trigger never
% contributes.
all_trigger_candidates([], _,_,_,_,_,_,_, []).
all_trigger_candidates([Trig|Rest], CP,T0,Duration,Z,Zt,Zb,B0, [Reason-Tcross-Code|RestCands]) :-
    trigger_crossing_time(Trig, CP,T0,Duration,Z,Zt,Zb,B0, Reason, Tcross, Code),
    all_trigger_candidates(Rest, CP,T0,Duration,Z,Zt,Zb,B0, RestCands).
all_trigger_candidates([Trig|Rest], CP,T0,Duration,Z,Zt,Zb,B0, RestCands) :-
    \+ trigger_crossing_time(Trig, CP,T0,Duration,Z,Zt,Zb,B0, _, _, _),
    all_trigger_candidates(Rest, CP,T0,Duration,Z,Zt,Zb,B0, RestCands).

% earliest_of(+TriplesList, -ReasonTimeCodeTriple): generic "minimum by
% SECOND element (Time)" over a non-empty list of Reason-Time-Code
% triples. Used to combine natural completion with however many
% Triggers-derived candidates happen to apply, into ONE earliest-wins
% choice -- this is the mechanism that makes moveto a genuine
% template: it works identically whether Triggers is empty, or
% contains one, or many conditions (including collision/battery
% themselves now), with no change needed here.
earliest_of([Triple], Triple).
earliest_of([R1-T1-C1|Rest], Result) :-
    earliest_of(Rest, _-T2-_),
    T1 =< T2,
    Result = R1-T1-C1.
earliest_of([R1-T1-C1|Rest], Result) :-
    earliest_of(Rest, R2-T2-C2),
    T1 > T2,
    Result = R2-T2-C2.

% walk_noisy_point(+CP,+T0,+Duration,+Z,+Zt,+T,-X,-Y): position along
% the spline at time T, given TWO ALREADY-RESOLVED, INDEPENDENT noise
% draws (rather than looking them up via z/2 or zt/2 itself): Z (lateral
% /normal drift) and Zt (tangential/along-path drift -- a straight
% metric push along the spline's own tangent direction at each point,
% NOT a reparametrization of Frac; see collision_geometry.py's own
% _walk_noisy_point for the reasoning behind Option B, a metric offset,
% over shifting Frac itself). NO Prolog clause of this name exists in
% this file -- this call resolves DIRECTLY against the black-box
% walk_noisy_point predicate collision_geometry.py registers (see
% Section 0's own :- use_module directive, and Section 3 above for the
% full "why this moved" rationale), the SAME way
% first_threshold_crossing_time/8 below already does for the
% obstacle-clearance search. Used here so the same formula is reused by
% first_collision_time's bracket/bisection search below and by at/4,
% without re-deriving Z/Zt through a different situation, and reused
% (at Z=Zt=0) by spline_point/4 above -- one formula, one place, both
% the noisy and the nominal case.

% ---------------------------------------------------------------
% FIRST-THRESHOLD-CROSSING-TIME -- a NATURAL (not chosen) event: the
% earliest time, within a given resolved world (fixed Z), at which
% the noisy trajectory comes within a given distance THRESHOLD of an
% obstacle. GENERALIZED over the threshold (rather than hardcoded to
% collision's safety_margin) so the SAME machinery serves collision
% (threshold=safety_margin, via first_collision_time/6 below),
% obstacle_in_bound(Threshold) (called DIRECTLY with the caller's own
% Threshold -- see trigger_crossing_time/9 above and
% holds(obstacle_in_bound(...)) below, no separate wrapper predicate
% needed since this black box was already threshold-generic), and any
% future distance-based trigger.
%
% first_threshold_crossing_time(+ControlPoints,+T0,+Duration,+Z,+Zt,
% +Threshold,-Tcross,-ObstacleId) is now a BLACK-BOX Python predicate,
% registered by collision_geometry.py's own :- use_module(...)
% directive (see below) -- exactly the same "deliberately NOT part of
% the situation-calculus machinery" reasoning already used for
% planWith/plan_call: this is a deterministic, stateless computation
% over an ALREADY-RESOLVED noise value Z, not a probabilistic choice in
% itself, so there is no frame problem here to justify keeping it in
% Prolog. ObstacleId is the crossed obstacle's own obstacle_polygon/2
% Id (an argmin over obstacles at the exact crossing point, not just
% the crossing time itself). bracket_samples/1 and crossing_eps/1 --
% the bracket-scan count and bisection tolerance -- are now config
% facts too (see the problem's own config.yaml's
% verification.bracket_samples/crossing_eps). collision_geometry.py
% reads them (and sigma/1's value) directly out of the problem's own config.yaml
% itself, not out of this generated fact, since config.yaml is the
% actual single source of truth both sides are driven from -- see
% collision_geometry.py's own header. FAILS (0 ProbLog solutions) if
% the trajectory never comes within Threshold of an obstacle in this
% resolved world -- correctly representing "never happens" via
% absence, not a sentinel value, same convention as everywhere else in
% this theory.

% first_collision_time/6 kept as a thin, name-preserving wrapper over
% the generalized machinery, at threshold=safety_margin -- every
% EXISTING caller (crashed_in, verify_safe, etc.) is unaffected beyond
% the new ObstacleId output.
first_collision_time(CP,T0,Duration,Z,Zt,Tcross,ObstacleId) :-
    safety_margin(M),
    first_threshold_crossing_time(CP,T0,Duration,Z,Zt,M,Tcross,ObstacleId).

% ---------------------------------------------------------------
% Poss AXIOMS for the primitive actions.
% ---------------------------------------------------------------
% NOTE: startMoveto deliberately does NOT check battery > 0 (or
% "not already colliding", or "not already within some obstacle_in_bound Threshold")
% as a precondition. Doing so would make do_action FAIL ENTIRELY when
% the battery is already empty (or the robot already unsafe) --
% classical Golog non-derivability, i.e. "no situation exists" -- the
% wrong semantics for a BT-style outcome (see the do_node/outcome
% discussion earlier). Instead, ALL of collision, battery depletion,
% obstacle_in_bound, and battery_below already have a graceful "already
% true at T0" case built into first_threshold_crossing_time /
% first_battery_depletion_time / first_battery_below_time (Tcross = T0
% exactly), so starting a walk with an
% empty battery -- or already inside an obstacle's margin -- still
% produces a well-formed, immediately-halted situation with the
% correct Reason, consistent with every other halting cause, rather
% than a special-cased blocking precondition for battery alone.
% T0 is rounded UP to disc_step_time/1's own granularity (see the
% MERGE-GRID QUANTIZATION note above dist/5's own section) -- the
% previous leg's own recorded halt instant (embedded in ITS OWN
% haltMoveto/interrupt term, read by REPORTING queries) stays exact;
% only the value THIS new leg treats as its own start time is
% coarsened, and CEILING (never floor/round) guarantees a new leg can
% never appear to start before the previous one actually ended.
poss(startMoveto(_,_Triggers,_ActionCode,T0), S) :-
    \+ moving(S),
    now(T0Exact, S),
    disc_step_time(Grid),
    quantize_up(T0Exact, Grid, T0).

% now(-T,+S): current wall-clock time -- needed above only to know
% WHEN to check the battery level (battery/3 needs a query time).
% Situation argument S is LAST, per Reiter's own convention (see
% module/contracts/vocabulary.yaml's own note -- this used to be
% now(S,T), the one remaining exception flagged when the other six
% out-of-convention accessors were fixed; fixed here too, along with
% every one of its own call sites throughout this file).
now(0, s0).
now(T, do(startMoveto(_,_,_,T),_)).
now(T, do(haltMoveto(T,_,_),_)).
now(T, do(interrupt(T),_)).
now(T, do(A,S)) :-
    A \= startMoveto(_,_,_,_), A \= haltMoveto(_,_,_), A \= interrupt(_),
    now(T, S).

% haltMoveto(T,Reason): the ways a walk stops other than an interrupt.
% ALL are NATURAL events, not choices -- T/Reason are DERIVED, never
% chosen by the plan. Whichever candidate cause -- natural completion,
% or any condition in this leg's own Triggers list (collision, battery,
% obstacle_in_bound(Threshold), battery_below(Threshold), or a future
% one) -- occurs EARLIEST in
% this resolved world wins. This is the genuine TEMPLATE mechanism:
% NOTHING is hardcoded here -- a leg with Triggers=[] halts ONLY on
% natural completion, passing straight through an obstacle's margin
% or running the battery dry without ever noticing, if collision/
% battery aren't in its own Triggers list.
%
% earliest_halt/11 is the SINGLE SHARED definition of "what happens
% first" -- used here, by Poss(interrupt(...)) below, AND by
% verify_safe further down (called there with Z=0.0,Zt=0.0,Zb=0.0
% instead of the resolved noise). Having exactly ONE definition, rather
% than the same computation duplicated at each call site, is what
% guarantees every query stays consistent with what Poss(haltMoveto(...))
% itself actually derives -- see the "SAFETY QUERIES READ THE ACTUAL
% OUTCOME" note further down for why this matters. Code is the winning
% candidate's own redescend-target code (none for natural completion/
% collision/battery, since those never classify to reactive(_) --
% see leg_status/9 further down and the CONTROL-FLOW REDESCEND TARGETS
% note above do_node(reactivesequence(...)) further down).
earliest_halt(CP,Triggers,T0,Duration,Z,Zt,Zb,B0, Reason,T,Code) :-
    all_trigger_candidates(Triggers, CP,T0,Duration,Z,Zt,Zb,B0, ExtraCandidates),
    NaturalEnd is T0 + Duration,
    earliest_of([completed-NaturalEnd-none], ExtraCandidates, Reason-T-Code).

% poss(haltMoveto(...)) is the ONE place a leg's own ActionCode
% (read off the SAME startMoveto term current_walk/6 already resolves
% CP/Triggers/T0/SPrev from -- see that predicate's own note) gets
% baked into the RECORDED Reason, via tag_reason/3 further down --
% leg_status/9 itself still decides Status from the UNTAGGED Reason0
% (it needs to recognize completed/crashed(_)/battery_depleted in
% their ORIGINAL shapes), so tagging happens strictly AFTER Status is
% already settled, on the value that actually gets written into S1's
% own haltMoveto term.
poss(haltMoveto(T, Reason, Status), S) :-
    moving(S),
    current_walk(S, CP, Triggers, ActionCode, T0, SPrev),
    walk_duration(CP, Duration),
    z(do(startMoveto(CP,Triggers,ActionCode,T0),SPrev), Z),
    zt(do(startMoveto(CP,Triggers,ActionCode,T0),SPrev), Zt),
    zbatt(Zb),
    leg_start_battery(T0, SPrev, B0),
    earliest_halt(CP,Triggers,T0,Duration,Z,Zt,Zb,B0, Reason0,T,Code),
    leg_status(Reason0, CP, T0, Duration, Z, Zt, T, Code, Status),
    tag_reason(Reason0, ActionCode, Reason).

% tag_reason(+Reason0, +ActionCode, -Reason): appends ActionCode as
% Reason0's own new TRAILING argument -- crashed(ObstacleId) becomes
% crashed(ObstacleId,ActionCode), completed becomes completed
% (ActionCode), guard_break(Cond) becomes guard_break(Cond,ActionCode),
% and so on for every Reason shape trigger_crossing_time/11 or
% earliest_halt/11 can ever produce -- GENERICALLY, via univ (=..),
% rather than one clause per Reason functor (the same "one generic
% mechanism instead of a per-case table" choice as holds_leg/9's own
% design). This is what lets a safety query check
% halted_with_cond(crashed(Obst1,ActionCode)) -- Obst1 unbound to match
% ANY obstacle, or bound to ask about a specific one, and ActionCode
% likewise -- to distinguish WHICH MoveTo occurrence in the tree
% produced a given halt, using the ordinary cond()/halted_with_cond
% machinery, no new query predicate needed.
tag_reason(Reason0, ActionCode, Reason) :-
    Reason0 =.. [Functor|Args0],
    append(Args0, [ActionCode], Args),
    Reason =.. [Functor|Args].

% leg_target(+ControlPoints, -GX,-GY): a leg's own intended endpoint
% is the LAST point in its OWN control_points list -- NOT necessarily
% the same as the global goal/2 fact, since a leg inside a future
% multi-leg sequence is trying to reach ITS OWN waypoint, not
% necessarily the overall plan's final destination. This is what lets
% status/leg_status generalize correctly to multi-leg plans without
% change: each leg's own success is about achieving its own target.
leg_target(ControlPoints, GX,GY) :-
    last_element(ControlPoints, point(GX,GY)).

last_element([X], X).
last_element([_|T], X) :- T \= [], last_element(T, X).

% leg_status(+Reason,+CP,+T0,+Duration,+Z,+Zt,+T,+Code,-Status): THREE
% possible SHAPES of output now -- true/false are plain Prolog ATOMS
% (there is no built-in boolean type restricting this to two values),
% and the third is now the COMPOUND term reactive(Code), not the bare
% atom reactive -- Code (threaded straight through from earliest_halt/
% 11's own Code output, itself read off the WINNING trigger's own
% Triggers-list occurrence -- see trigger_crossing_time/11's own note)
% is what lets do_node(reactivesequence(...))/do_node(reactivefallback
% (...)) further down decide whether THEY are the redescend target for
% THIS particular reactive halt, instead of every enclosing composite
% unconditionally redescending the way evaluate_plan/4 used to.
%
%   true         -- Reason=completed: the walk ran its full nominal
%                duration without any Trigger cutting it short. Says
%                NOTHING about whether the actual (noisy) final
%                position also landed close enough to the leg's own
%                endpoint -- that used to be baked in here via
%                goal_tolerance/1, but is now its OWN explicit,
%                inspectable BT condition (distance_below/3), hand-
%                placed by the tree author right after a MoveTo in a
%                Sequence wherever that check is wanted, same as any
%                other cond() leaf, rather than something silently
%                folded into MoveTo's own Status.
%   false        -- a genuine, unrecoverable failure: crashed
%                (ObstacleId) or battery_depleted. Nothing downstream
%                can react to these and continue; the leg (and, per
%                Sequence/Fallback's own do_node rules, quite possibly
%                the whole plan) is simply done.
%   reactive(Code) -- every OTHER trigger (obstacle_in_bound(...),
%                obstacle_on_path(...), battery_under/equal/over(...),
%                line_of_sight_clear(...), crosses_segment(...), and
%                any future trigger name not explicitly listed as a
%                hard failure above): the walk was cut short by a
%                condition that's meant to be REACTED to, not treated
%                as outright success or failure -- see the CONTROL-
%                FLOW REDESCEND TARGETS note above do_node(reactive
%                sequence(...)) further down for what happens when a
%                do_node/4 call anywhere in the tree returns this.
leg_status(completed, _CP, _T0, _Duration, _Z, _Zt, _T, _Code, true).
leg_status(crashed(_), _,_,_,_,_,_, _Code, false).
leg_status(battery_depleted, _,_,_,_,_,_, _Code, false).
leg_status(Reason, _,_,_,_,_,_, Code, reactive(Code)) :-
    Reason \= completed,
    Reason \= crashed(_),
    Reason \= battery_depleted.

% earliest_of/3: like earliest_of/2, but takes a fixed head list
% (currently just [completed-NaturalEnd-none]) and a (possibly empty)
% list of Triggers-derived candidates separately, and combines them --
% kept as a distinct small wrapper so call sites read as "natural
% completion ++ whatever this leg's Triggers produced," the intent,
% at a glance.
earliest_of(BuiltIns, ExtraCandidates, Result) :-
    append(BuiltIns, ExtraCandidates, All),
    earliest_of(All, Result).

% append/3 (ProbLog has no built-in append/3 -- see the note on
% drop_n/3 earlier in this file for why hand-rolled list utilities
% appear throughout).
append([], L, L).
append([H|T], L, [H|R]) :- append(T, L, R).

% interrupt(T): the one genuinely CHOSEN action -- the plan decides
% when to cut the walk short. Only possible while a walk is in
% progress, at or after its start, and STRICTLY BEFORE whichever
% happens first in this resolved world among ALL candidate causes in
% this leg's own Triggers list -- you can't "interrupt" a walk that,
% in this world, has already halted on its own for any reason. Uses
% the SAME earliest_halt/10 as Poss(haltMoveto(...)) above, so this
% bound can never drift out of sync with what haltMoveto itself would
% derive.
poss(interrupt(T), S) :-
    moving(S),
    current_walk(S, CP, Triggers, T0, SPrev),
    walk_duration(CP, Duration),
    z(do(startMoveto(CP,Triggers,_ActionCode,T0),SPrev), Z),
    zt(do(startMoveto(CP,Triggers,_ActionCode,T0),SPrev), Zt),
    zbatt(Zb),
    leg_start_battery(T0, SPrev, B0),
    earliest_halt(CP,Triggers,T0,Duration,Z,Zt,Zb,B0, _Reason,Tend,_Code),
    T >= T0, T < Tend.

% ---------------------------------------------------------------
% 6. THE POSITION FLUENT -- pure situation-calculus regression.
%    at(X,Y,T,S): position of the robot at time T in situation S.
%
%    Three cases:
%      (a) base case: before any walk, position is the fixed start
%      (b) INSIDE an open walk (do(startMoveto(...),S)): interpolate
%          along the spline + noise, exactly as before
%      (c) AFTER the walk has ended (do(haltMoveto(T1,_),S) -- for
%          EITHER reason, completed or crashed -- or do(interrupt(T1),S)):
%          position FREEZES at whatever it was at time T1. This is
%          what makes both natural halting and interruption actually
%          stop the robot rather than letting it keep gliding toward
%          the original target -- and in particular, a crash freezes
%          the robot exactly at the point of collision, not beyond it.
% ---------------------------------------------------------------
at(X,Y,_,s0) :- start(X,Y).

at(X,Y,T, do(startMoveto(ControlPoints,Triggers,ActionCode,T0), S)) :-
    walk_duration(ControlPoints, Duration),
    z(do(startMoveto(ControlPoints,Triggers,ActionCode,T0),S), Z),
    zt(do(startMoveto(ControlPoints,Triggers,ActionCode,T0),S), Zt),
    walk_noisy_point(ControlPoints, T0, Duration, Z, Zt, T, X, Y).

at(X,Y,T, do(haltMoveto(T1,_Reason,_Status), S)) :-
    Tc is min(T,T1),
    at(X,Y,Tc,S).

at(X,Y,T, do(interrupt(T1), S)) :-
    Tc is min(T,T1),
    at(X,Y,Tc,S).

% pass-through clause: kept for extensibility, so that additional
% actions that DON'T affect position (e.g. a future sensing action)
% can be appended without breaking the regression.
at(X,Y,T, do(A,S)) :-
    A \= startMoveto(_,_,_,_), A \= haltMoveto(_,_,_), A \= interrupt(_),
    at(X,Y,T,S).

% nominal (zero-noise) position -- for Feature-1-style deterministic
% checks and for the on_track discrepancy fluent. Frac here is
% ALWAYS relative to the walk's own full nominal duration (not to
% however much of it actually got executed before an interrupt).
nominal_at(X,Y,Frac,ControlPoints) :- spline_point(ControlPoints, Frac, X, Y).

% ============================================================
% 7. PRIMITIVE ACTION EXECUTION + BEHAVIOR-TREE INTERFACE.
%
%     do_node(Node, S, S1, Outcome)
%
% Outcome is 'true' or 'false' -- the two-valued signal every node,
% leaf or composite, reports through the SAME predicate. This is the
% "standard interface" every action/condition must supply: given the
% current situation, produce a resulting situation and an outcome.
% Sequence and Fallback below are written PURELY in terms of this
% interface -- they never inspect what KIND of thing a child is,
% which is what makes them genuine reusable templates over a LIST of
% arbitrarily many, arbitrarily nested children.
%
% A single true/false vocabulary is used everywhere -- not
% success/failure -- so every leaf's own natural output (a
% moveto_leg's Status is already true/false; a cond(C)'s holds/2 is
% already a true/false question) can flow straight through as Outcome
% with no translation step in between.
%
% Node is one of:
%     cond(C,Code)              -- CONDITION leaf: tests C against the
%                                  CURRENT situation via holds/2. UNLIKE
%                                  an earlier version of this file
%                                  (cond(C), no Code, S1=S, a genuine
%                                  no-op), this now DOES extend the
%                                  situation, with a checked(Code,C,
%                                  Status) marker -- the direct
%                                  analogue of planWith's own planned
%                                  (Algorithm,Reason) marker just below,
%                                  for the SAME reason: otherwise a
%                                  condition's own outcome leaves no
%                                  trace once execution moves past it,
%                                  making it unqueryable after the fact
%                                  (an action's own Reason survives in
%                                  haltMoveto(...)/planned(...); a bare
%                                  cond(C) with S1=S never did). Code is
%                                  a per-CONDITION-OCCURRENCE identifier
%                                  bt_to_prolog.py assigns (same
%                                  per-occurrence-counter idiom as
%                                  MoveTo/PlanWith's own ActionCode),
%                                  letting checked_with/4 further down
%                                  (the direct analogue of planned_with/
%                                  3) distinguish WHICH cond() leaf in
%                                  the tree a given checked(...) marker
%                                  came from -- there can be many, e.g.
%                                  a DistanceBelow after EVERY MoveTo.
%     moveto_leg(CP,Triggers)   -- ACTION leaf. Triggers is ALWAYS
%                                  given explicitly at the call site --
%                                  there is no sugar/default form, by
%                                  design (see the note above Triggers
%                                  in this file's own header). Runs one
%                                  startMoveto/haltMoveto pair to its
%                                  halt; T0 auto-derived via now/2.
%                                  Outcome IS the Status output,
%                                  unchanged.
%     planWith(Algorithm,Goal,CP) -- PLANNING leaf: ONE TEMPLATE covering
%                                  every planner (plan_astar,
%                                  plan_straight, and any future one --
%                                  see plan_call/8's own dispatch on
%                                  Algorithm) -- not a separate do_node
%                                  clause per planner. A stateless
%                                  black-box call into
%                                  planners.py, binding CP for a
%                                  subsequent moveto_leg(CP,...) to
%                                  use. Goal is EXPLICIT (point(GX,GY)),
%                                  an argument of planWith itself, not
%                                  the global goal/2 fact -- lets two
%                                  planWith calls in one plan target
%                                  genuinely different destinations.
%                                  Interface analogous to haltMoveto's
%                                  own (Reason,Status) formalization
%                                  via plan_call/8 -- Reason is
%                                  completed/no_path -- without any of
%                                  its situation-calculus machinery
%                                  (no primitive_action, no Poss).
%                                  UNLIKE plan_call/8 itself, this DOES
%                                  extend the situation, with a bare
%                                  planned(Algorithm,Reason) MARKER (no
%                                  precondition -- see planned_with/3),
%                                  so a plan calling planning more than
%                                  once stays traceable.
%     seq_node(ChildList)       -- BT SEQUENCE: run children in
%                                  order; stop and fail as soon as one
%                                  fails; succeed if all do.
%     fallback_node(ChildList)  -- BT FALLBACK/SELECTOR: try children
%                                  in order; stop and succeed as soon
%                                  as one succeeds; fail if all do.
% ============================================================
primitive_action(startMoveto(_,_,_,_)).
primitive_action(haltMoveto(_,_,_)).
primitive_action(interrupt(_)).

do_action(A, S, do(A,S)) :- primitive_action(A), poss(A, S).

% -- CONDITION leaf ---------------------------------------------------
% checked(Code,C,Status) is a bare MARKER, exactly like planned(Algorithm,
% Reason) below -- no primitive_action entry, no Poss, just recorded via
% do(...) the same way do_action itself would. Status is fully
% DETERMINED by holds(C,S) at an ALREADY-RESOLVED situation -- recording
% it introduces no new probabilistic choice (unlike z/2's own per-
% startMoveto annotated disjunction), so this adds no branching to the
% underlying inference, only one more do(...) layer for the generic
% pass-through fluents (at/4, battery/3, moving/1, now/2, current_walk/6)
% to skip over -- exactly the same, already-proven-cheap shape planned
% (Algorithm,Reason) already added for PlanWith.
do_node(cond(C,Code), S, do(checked(Code,C,true), S), true)  :- holds(C, S).
do_node(cond(C,Code), S, do(checked(Code,C,false),S), false) :- \+ holds(C, S).

% -- ACTION leaf --------------------------------------------------------
% moveto_leg(CP,Triggers) -- Triggers is ALWAYS given explicitly here;
% there is deliberately NO sugar/default form (no moveto_leg/1, no
% config-driven fallback) -- every call site states its own protection
% level, e.g. moveto_leg(CP,[collision,battery]) or moveto_leg(CP,[])
% for a genuinely unprotected leg. Status flows straight through as
% Outcome -- no translation predicate needed, since both already speak
% true/false.
do_node(moveto_leg(CP,Triggers,ActionCode), S, S1, Status) :-
    do_action(startMoveto(CP,Triggers,ActionCode,_T0), S, S2),
    do_action(haltMoveto(_T,_Reason,Status), S2, S1).

% -- PLANNING actions: deliberately NOT part of the full action theory
%    -- no primitive_action/1 entry, no Poss axiom, no do_action call
%    with its precondition check (these are stateless, purely-
%    computational black-box calls into planners.py, not
%    physical processes -- Reiter's machinery exists to solve the
%    frame problem for things that CHANGE THE WORLD over time; a
%    lookup that returns instantly has no frame problem to solve, so
%    building out that whole apparatus for it would be unnecessary
%    weight, not faithfulness).
%
%    The interface is FORMALIZED THE SAME WAY AS haltMoveto's own
%    (T,Reason,Status): plan_call/8 binds Reason AND Status TOGETHER,
%    directly, within each clause -- exactly as leg_status/7 binds
%    Status alongside a given Reason for moveto, rather than via a
%    SEPARATE Reason->Status lookup table. Status is a genuine second
%    output, not a name recomputed from Reason after the fact; it just
%    happens (for these two planners) to always agree with Reason one-
%    for-one, since there's no extra condition to check beyond "was a
%    path found" (moveto's leg_status has an EXTRA condition --
%    within-tolerance position -- planning does not). plan_call/8 is a
%    genuine standalone predicate, callable directly (not only through
%    do_node), so Reason is truly accessible, not swallowed on the way
%    to Status.
%
%    plan_call/8 is ALREADY a genuine TEMPLATE over Algorithm (its own
%    first argument) -- astar and straight are just two clauses of the
%    SAME predicate, dispatched by ordinary clause selection. Adding a
%    future THIRD planner (e.g. an RRT, or a different black-box
%    module entirely) means adding one more plan_astar-style function
%    to planners.py plus one more pair of plan_call/8 clauses
%    here; nothing about do_node, planned_with/3, or anything
%    downstream needs to change. Because Reason/Status are bound
%    DIRECTLY per clause rather than through a shared central lookup
%    table, a future planner is also free to introduce its OWN
%    distinct Reason atoms (e.g. a timeout-capable planner might use
%    `timeout` alongside `no_path`, both mapping to Status=false) --
%    it declares that mapping in its own clauses, with no need to
%    extend a shared table every other planner also depends on.
%
%    Common Reason vocabulary, shared by EVERY planner (only one
%    possible failure cause: no path exists): completed on success
%    (matching moveto's own success-Reason naming), no_path on
%    failure. CP is [] in the no_path case.
plan_call(astar, SX,SY,GX,GY, CP, completed, true) :-
    plan_astar(SX,SY,GX,GY, CP).
plan_call(astar, SX,SY,GX,GY, [], no_path, false) :-
    \+ plan_astar(SX,SY,GX,GY, _).

plan_call(straight, SX,SY,GX,GY, CP, completed, true) :-
    plan_straight(SX,SY,GX,GY, CP).
plan_call(straight, SX,SY,GX,GY, [], no_path, false) :-
    \+ plan_straight(SX,SY,GX,GY, _).

% voronoi -- a THIRD planner, SAME bare-atom Algorithm shape as astar/
% straight (no compound term needed -- this planner takes no extra
% parameters beyond the shared SX,SY,GX,GY every planWith call already
% carries), exactly the "add one more plan_astar-style function plus
% one more pair of plan_call/8 clauses" recipe this section's own
% header comment anticipated -- needing NO new dispatch machinery
% anywhere downstream (do_node/4, schema.yaml, bt_to_prolog.py, and
% bt_actions.py all reuse their EXISTING astar/straight branches
% verbatim for this one). Builds a roadmap from a generalized Voronoi
% diagram of the obstacle map (planners.py's plan_voronoi/5),
% connecting SX,SY and GX,GY to the closest POINT on the closest EDGE
% of that roadmap -- see that predicate's own header for the geometry.
% Degrades to a straight line (never fails) when there are no
% obstacles to route around at all; only fails (no_path) if a roadmap
% exists but start/goal are genuinely disconnected within it.
plan_call(voronoi, SX,SY,GX,GY, CP, completed, true) :-
    plan_voronoi(SX,SY,GX,GY, CP).
plan_call(voronoi, SX,SY,GX,GY, [], no_path, false) :-
    \+ plan_voronoi(SX,SY,GX,GY, _).

% follow_boarder(ObstacleId,Offset) -- a FOURTH planner, exactly the
% "add one more plan_astar-style function plus one more pair of
% plan_call/8 clauses" recipe this section's own header comment
% already anticipated. Algorithm here is a COMPOUND term, not a bare
% atom like astar/straight -- ObstacleId (which obstacle_polygon/2 to
% circle) and Offset (how far out to stay from its boundary) are
% carried INSIDE Algorithm itself, so planWith/do_node/planned_with
% need NO interface change at all: they already treat Algorithm as
% opaque. Offset is typically unified with the SAME Threshold as
% whichever obstacle_on_path(Threshold)/obstacle_in_bound(Threshold)
% trigger or condition supplied ObstacleId in the first place.
%
% UNLIKE an earlier version of this planner, follow_boarder does NOT
% decide when to stop circling -- it plans a FULL clockwise loop around
% ObstacleId's offset boundary (planners.py's follow_boarder/5),
% always succeeding for a known obstacle (Reason=completed regardless
% of Goal, since there is no Goal-relative stopping decision to make
% here at all -- notice the /5 arity below has no GX,GY). WHEN to
% actually leave the boundary and hand off to a straight-line planner
% is entirely the job of whichever trigger halts the SUBSEQUENT
% moveto_leg(CP,[...]) using this CP -- line_of_sight_clear(ObstacleId,
% GX,GY) for Bug0, crosses_segment(SX,SY,GX,GY) for Bug2 (see the
% TRIGGERS section above) -- so the bug-variant choice is a matter of
% that Triggers list, not a different planner call. If the attached
% trigger never fires, the leg just completes the whole loop naturally
% and Status comes out true via the ordinary leg_status mechanism --
% leg_status/9 no longer judges whether the final position is close to
% the leg's own endpoint (see that predicate's own note: that check is
% now the separate, explicit distance_below/3 BT condition), so a fully
% -circled loop with nothing meaningful downstream of it simply reads
% as a completed leg, same as any other MoveTo. No special no_path case
% needed here for that.
plan_call(follow_boarder(ObstacleId,Offset), SX,SY,_GX,_GY, CP, completed, true) :-
    follow_boarder(SX,SY,ObstacleId,Offset, CP).
plan_call(follow_boarder(ObstacleId,Offset), SX,SY,_GX,_GY, [], no_path, false) :-
    \+ follow_boarder(SX,SY,ObstacleId,Offset, _).

% -- PLANNING leaf: ONE template, planWith(Algorithm,Goal,CP), covering
%    every planner via plan_call/8's own dispatch on Algorithm --
%    NOT two (or more) separate hand-written do_node clauses. Called
%    right before a moveto_leg sharing the SAME CP variable, e.g.:
%        seq_node([planWith(astar,point(17.0,17.0),CP), moveto_leg(CP,[collision,battery])])
%    -- exactly the same "leave a variable free, let it get bound by
%    whichever step derives it" pattern already used for auto-timing
%    T0 across chained legs (see now/2's role in Poss(startMoveto...)).
%    Current position comes from now/2 + at/4 (i.e. "plan from HERE,
%    right now"); Goal is EXPLICIT, an argument of planWith itself
%    (point(GX,GY)), NOT read from the global goal/2 fact -- this is
%    what makes it possible for TWO planWith calls in the SAME plan to
%    target genuinely DIFFERENT destinations (e.g. a "go to P1, then
%    go to P2" multi-leg plan: seq_node([planWith(astar,point(P1x,P1y),CP1),
%    moveto_leg(CP1,...), planWith(astar,point(P2x,P2y),CP2),
%    moveto_leg(CP2,...)])) -- the direct Prolog analogue of
%    instantiating a parametrized "GoTo(target)" BT.cpp subtree twice
%    with two different port bindings, rather than both calls silently
%    aiming at one shared destination (there is no longer a global
%    goal/2 fact at all -- see distance_below/3's own note, and plan_generation
%    /plan/goal_formula.pl for where a plan's own goal information
%    lives now). Status flows
%    straight through as Outcome, exactly like moveto_leg's own Status
%    -- no translation predicate, since plan_call/8 already produces
%    it directly.
%
%    UNLIKE the pure-lookup plan_call/8 it wraps, this DOES extend the
%    situation -- with a bare planned(Algorithm,Reason) MARKER, not a
%    real primitive action (no Poss, no primitive_action entry; the
%    marker is simply CONSTRUCTED via do(...) the same way do_action
%    itself would, minus the precondition gate there's nothing to
%    gate). This is the lightweight piece needed for TRACEABILITY once
%    a plan calls planning MORE THAN ONCE (e.g. a fallback_node trying
%    astar then straight, or several legs each replanning from wherever
%    the previous one ended) -- otherwise there would be nothing in a
%    no-path failure's situation to distinguish it from any other, or
%    to tell WHICH of several planning attempts is which. The marker
%    is transparent to every existing fluent: at/4, battery/3,
%    moving/1, now/2, current_walk/5, and last_action_time/4 ALL
%    already pass through any action other than startMoveto/
%    haltMoveto/interrupt unchanged, so recording it changes nothing
%    about elapsed time, position, or battery -- still genuinely
%    instantaneous, just no longer INVISIBLE to later inspection.
%
%    IMPORTANT GOTCHA when using planWith inside a fallback_node with
%    SEPARATE algorithms per branch: give EACH branch its OWN CP
%    variable, e.g.
%        fallback_node([seq_node([planWith(astar,point(GX,GY),CP1,a1), moveto_leg(CP1,...)]),
%                        seq_node([planWith(straight,point(GX,GY),CP2,a2), moveto_leg(CP2,...)])])
%    NOT a single CP variable shared across both branches. Reusing one
%    CP across fallback alternatives silently breaks: a FAILING
%    planWith still SUCCEEDS as a do_node call (with Outcome=false,
%    CP=[]) -- do_node calls don't get "undone" on Outcome=false the
%    way a genuine Prolog failure would -- so CP is left bound to []
%    by the first branch, and the SECOND branch's own attempt to bind
%    CP to its own (non-empty) result then fails to unify, breaking
%    the fallback in a confusing way that looks unrelated to variable
%    scoping. This was hit directly while testing this exact feature.
% SX,SY are rounded to disc_step_position/1's own granularity (see
% the MERGE-GRID QUANTIZATION note above dist/5's own section) before
% being handed to plan_call/plan straight/plan_astar/... -- this is
% what actually makes CP (hence the NEW leg's own startMoveto(CP,...)
% term) merge-friendly: since a planner's own path always begins
% EXACTLY at the point it's given, quantizing the point handed in here
% is enough on its own to make every downstream CP identical across
% worlds that round to the same grid cell -- no separate CP-level
% rounding needed. Every OTHER read of at/4 (collision detection,
% goal-tolerance checking in leg_status, first_hit/on_track/
% verify_safe reporting) stays exact, unaffected -- only the position
% a NEW leg gets planned FROM is coarsened.
%
% ActionCode (a FOURTH argument now, one per planWith OCCURRENCE in
% the tree, from the SAME var_pool.next_action_code() counter MoveTo's
% own ActionCode already comes from -- codes stay unique tree-wide
% regardless of node kind) is baked into the RECORDED Reason via
% tag_reason/3, the EXACT same generic mechanism poss(haltMoveto(...))
% already uses -- see that predicate's own note. Reason0 out of
% plan_call/8 is always the bare atom completed/no_path; before
% tagging, it's first rebuilt (via =..) to also carry Algorithm and
% the point this call was actually working toward, so the FINAL
% recorded Reason is completed(Algorithm,Goal,ActionCode) or
% no_path(Algorithm,Goal,ActionCode) -- ActionCode LAST is not
% arbitrary, it's what lets halted_with_pattern/3 (built generically
% on "the recorded Reason's own trailing argument is ActionCode",
% see that predicate's own note) work for planning outcomes with NO
% change of its own. TWO mutually exclusive clauses below, split on
% Algorithm's own shape, ONLY because follow_boarder has no real Goal
% point to report (see follow_boarder(ObstacleId,Offset)'s own note
% above plan_call/8) --
% embedding the harmless point(0.0,0.0) placeholder threaded through
% planWith's own second argument for template-sharing purposes would
% be misleading if surfaced in a Reason meant to be read by a human/
% query, so that case reports the honest atom `none` instead. The
% explicit Algorithm \= follow_boarder(_,_) guard on the second clause
% is NOT redundant with the first clause's own head shape: without it,
% a follow_boarder call's own point(0.0,0.0) placeholder Goal argument
% would ALSO unify against the second clause's point(GX,GY) head
% pattern, giving ProbLog TWO derivations of the same fact instead of
% one -- exactly the kind of silent solution-doubling this project has
% already hit once (see match_wild/2's own history note) and now
% checks for on purpose.
do_node(planWith(follow_boarder(ObstacleId,Offset), _Goal, CP, ActionCode), S,
        do(planned(follow_boarder(ObstacleId,Offset),Reason), S), Status) :-
    now(T, S), at(SXExact,SYExact,T,S),
    disc_step_position(Grid),
    quantize(SXExact, Grid, SX), quantize(SYExact, Grid, SY),
    plan_call(follow_boarder(ObstacleId,Offset), SX,SY,_GX,_GY, CP, Reason0, Status),
    Reason0 =.. [Functor],
    Reason1 =.. [Functor, follow_boarder(ObstacleId,Offset), none],
    tag_reason(Reason1, ActionCode, Reason).
do_node(planWith(Algorithm, point(GX,GY), CP, ActionCode), S,
        do(planned(Algorithm,Reason), S), Status) :-
    Algorithm \= follow_boarder(_,_),
    now(T, S), at(SXExact,SYExact,T,S),
    disc_step_position(Grid),
    quantize(SXExact, Grid, SX), quantize(SYExact, Grid, SY),
    plan_call(Algorithm, SX,SY,GX,GY, CP, Reason0, Status),
    Reason0 =.. [Functor],
    Reason1 =.. [Functor, Algorithm, point(GX,GY)],
    tag_reason(Reason1, ActionCode, Reason).

% planned_with(+Algorithm, +Reason, +S): the direct parallel to
% halted_with/2, for the (now-recorded) planning marker. Searches the
% WHOLE history, so it can distinguish which of SEVERAL planning
% attempts (across different legs, or different fallback branches)
% produced a given Reason, and with which algorithm -- though Reason
% ITSELF now already carries Algorithm and ActionCode too (see
% do_node(planWith(...))'s own note), so this predicate's own
% Algorithm argument is redundant with Reason's own first argument in
% practice; kept as-is since dropping it would be a needless interface
% change for something already unambiguous.
planned_with(Algorithm, Reason, do(planned(Algorithm,Reason), _)).
planned_with(Algorithm, Reason, do(_A, S)) :- planned_with(Algorithm, Reason, S).

% checked_with(+Code, -C, -Status, +S): the direct analogue of
% planned_with/3 above, for cond(C,Code)'s own checked(Code,C,Status)
% marker -- searches the WHOLE history for Code (same "search
% everything, one solution per match" shape halted_with/2 and
% planned_with/3 both already use), NOT just the most recent one: a
% cond() leaf sitting inside a reactive_children/2 list can be
% re-checked on every redescend, so a given Code can legitimately carry
% more than one (possibly different) Status across a single resolved
% world's own history -- each is reported as its own solution, exactly
% mirroring how halted_with/2 already handles a Reason that could, in
% principle, recur. Fails outright (no solution) for a Code whose own
% cond() leaf was never reached in a given resolved world at all (e.g.
% it sits in a Fallback branch that never got tried) -- same "absence,
% not sentinel" convention as everywhere else in this file.
checked_with(Code, C, Status, do(checked(Code,C,Status), _)).
checked_with(Code, C, Status, do(_A, S)) :- checked_with(Code, C, Status, S).

% -- SEQUENCE composite: stop and FAIL at the first failing child; --
%    succeed only if every child succeeds, in order. A REACTIVE child
%    (see leg_status/9's own note on the three-valued Status) stops
%    the sequence too, same as false -- but is NOT the same as false:
%    it propagates straight through, UNCHANGED (Code and all -- this
%    clause never inspects WHICH code it is, unlike reactivesequence/
%    reactivefallback below), to whatever node contains THIS seq_node
%    -- see the CONTROL-FLOW REDESCEND TARGETS note below for the full
%    picture of why and where this eventually gets caught. A plain
%    seq_node NEVER catches/redescends on its own, by design -- exactly
%    matching real BT.cpp's own plain Sequence, which a ReactiveSequence
%    is a genuinely DIFFERENT node type from, not a special case of.
do_node(seq_node([]), S, S, true).
do_node(seq_node([Child|Rest]), S, S1, Outcome) :-
    do_node(Child, S, S2, true),
    do_node(seq_node(Rest), S2, S1, Outcome).
do_node(seq_node([Child|_]), S, S1, false) :-
    do_node(Child, S, S1, false).
do_node(seq_node([Child|_]), S, S1, reactive(Code)) :-
    do_node(Child, S, S1, reactive(Code)).

% -- FALLBACK (Selector) composite: stop and SUCCEED at the first ---
%    succeeding child; fail only if every child fails, in order.
%    NOTE: on a failing child, the NEXT child starts from THAT
%    child's resulting situation, not from the original S -- a failed
%    PHYSICAL action (e.g. a crashed moveto_leg) still consumed real
%    time and moved the robot; unlike a classical BT's usual
%    assumption that failed leaves are side-effect-free, a fallback
%    over durative ACTIONS here means "try the next option from
%    wherever the failed attempt left us," not "rewind and try the
%    next option from the start." A REACTIVE child does NOT try the
%    next sibling this way -- same as seq_node above, it propagates
%    straight through unchanged instead (see the CONTROL-FLOW
%    REDESCEND TARGETS note below).
do_node(fallback_node([]), S, S, false).
do_node(fallback_node([Child|_]), S, S1, true) :-
    do_node(Child, S, S1, true).
do_node(fallback_node([Child|Rest]), S, S1, Outcome) :-
    do_node(Child, S, S2, false),
    do_node(fallback_node(Rest), S2, S1, Outcome).
do_node(fallback_node([Child|_]), S, S1, reactive(Code)) :-
    do_node(Child, S, S1, reactive(Code)).

% -- INVERTER decorator: BT.cpp's built-in single-child negation ------
%    node (<Inverter>C</Inverter>). Flips true<->false; a REACTIVE
%    child's reactive(Code) status passes straight through UNCHANGED,
%    same "never inspects, never catches" passthrough seq_node/
%    fallback_node's own reactive clauses already do -- an interrupt
%    signal isn't a true/false outcome to negate, it's a request to be
%    caught by a MATCHING reactivesequence(Code)/reactivefallback(Code)
%    further up, wherever that is; an Inverter never IS one (it takes
%    no Code of its own -- see _REACTIVE_CONTROL_FLOW's own note in
%    bt_to_prolog.py for why the guard-derivation machinery already
%    treats Inverter as fully transparent for that purpose too, not
%    just for do_node's own control flow).
do_node(inverter(Child), S, S1, false) :-
    do_node(Child, S, S1, true).
do_node(inverter(Child), S, S1, true) :-
    do_node(Child, S, S1, false).
do_node(inverter(Child), S, S1, reactive(Code)) :-
    do_node(Child, S, S1, reactive(Code)).

% ---------------------------------------------------------------
% CONTROL-FLOW REDESCEND TARGETS -- reactivesequence(Code) and
% reactivefallback(Code) are the ONLY two node types that ever catch a
% reactive(_) status and act on it; plain seq_node/fallback_node above
% NEVER do (they just pass it straight up, unconditionally, forever).
% This is the direct Prolog analogue of BT.cpp's own real distinction
% between Sequence/Fallback (checked once, never re-monitored) and
% ReactiveSequence/ReactiveFallback (continuously re-ticked) -- see
% this project's own conversation log for the fuller discussion this
% design came out of.
%
% Code is a plain atom (bt_to_prolog.py assigns one, e.g. rc1, rc2,
% ..., to each ReactiveSequence/ReactiveFallback it translates,
% uniquely across the whole tree) IDENTIFYING one specific reactive
% composite. Every reactive-classified trigger name in the Triggers
% list of every MoveTo leg underneath it (obstacle_in_bound(Threshold,
% Code), battery_below(Threshold,Code), etc. -- see trigger_crossing_
% time/11's own note) is tagged, at translation time, with THIS SAME
% Code -- the code of its own NEAREST enclosing ReactiveSequence/
% ReactiveFallback (an intervening plain seq_node/fallback_node
% doesn't matter, since it never inspects Code at all -- reactive(Code)
% passes straight through it unchanged either way). So when a leg
% halts reactively, reactive(Code) bubbles up through zero or more
% plain composites, completely inert, until it reaches the ONE
% composite whose own Code matches -- which is, by construction,
% always its own nearest enclosing reactive composite -- and THAT one
% catches it.
%
% "Catching" means: instead of propagating further, re-run this same
% composite's own children FRESH -- see reactive_children/2 (a
% SEPARATE fact per reactive composite, one row per Code, generated by
% bt_to_prolog.py) and why it has to be a separately-resolved fact,
% not the SAME children term reused across restarts: exactly the
% "planWith inside a fallback_node" gotcha documented above (a second
% pass's own planWith would try to unify a genuinely different control-
% point list against an ALREADY-BOUND CP left over from the first
% pass, and fail outright) -- reactive_children/2 resolved as a FRESH
% goal each time gives genuinely unbound variables on every pass,
% exactly mirroring how plan(Node) itself already works for the
% (now removed) whole-tree redescend evaluate_plan/4 used to do.
%
% Each reactive composite carries its OWN budget, read fresh from
% replan_budget/1 on first entry (see reactivesequence_budgeted/5
% below) and decremented on every restart -- exhausting it resolves to
% world_too_large right there rather than continuing to restart or
% propagating upward (mirrors evaluate_plan/4's own former clause 3,
% just localized). This budget is what stands between a degenerate
% zero-duration leg and an infinite restart loop -- see this project's
% own conversation log for why a PER-composite budget is needed now
% that redescend can happen below the root, not just at it.
%
% A reactive(_) that reaches THE ROOT of the whole tree without any
% composite's own Code ever matching means bt_to_prolog.py tagged a
% leg's own reactive trigger with a Code that no enclosing
% ReactiveSequence/ReactiveFallback actually carries -- a TRANSLATOR
% BUG (that generator also validates this at translation time, as a
% hard failure, so this should never actually happen at runtime) --
% see plan_outcome/1's own reactive_escaped clause further down for
% how this is reported if it somehow does.
do_node(reactivesequence(Code), S, S1, Outcome) :-
    replan_budget(Budget),
    reactivesequence_budgeted(Code, Budget, S, S1, Outcome).

reactivesequence_budgeted(Code, Budget, S, S1, Outcome) :-
    Budget > 0,
    reactive_children(Code, Children),
    do_node(seq_node(Children), S, S2, reactive(Code)),
    Budget1 is Budget - 1,
    reactivesequence_budgeted(Code, Budget1, S2, S1, Outcome).
reactivesequence_budgeted(Code, 0, S, S, world_too_large) :-
    reactive_children(Code, Children),
    do_node(seq_node(Children), S, _, reactive(Code)).
reactivesequence_budgeted(Code, Budget, S, S1, reactive(OtherCode)) :-
    Budget > 0,
    reactive_children(Code, Children),
    do_node(seq_node(Children), S, S1, reactive(OtherCode)),
    OtherCode \= Code.
reactivesequence_budgeted(Code, Budget, S, S1, Outcome) :-
    Budget > 0,
    reactive_children(Code, Children),
    do_node(seq_node(Children), S, S1, Outcome),
    Outcome \= reactive(_).

do_node(reactivefallback(Code), S, S1, Outcome) :-
    replan_budget(Budget),
    reactivefallback_budgeted(Code, Budget, S, S1, Outcome).

reactivefallback_budgeted(Code, Budget, S, S1, Outcome) :-
    Budget > 0,
    reactive_children(Code, Children),
    do_node(fallback_node(Children), S, S2, reactive(Code)),
    Budget1 is Budget - 1,
    reactivefallback_budgeted(Code, Budget1, S2, S1, Outcome).
reactivefallback_budgeted(Code, 0, S, S, world_too_large) :-
    reactive_children(Code, Children),
    do_node(fallback_node(Children), S, _, reactive(Code)).
reactivefallback_budgeted(Code, Budget, S, S1, reactive(OtherCode)) :-
    Budget > 0,
    reactive_children(Code, Children),
    do_node(fallback_node(Children), S, S1, reactive(OtherCode)),
    OtherCode \= Code.
reactivefallback_budgeted(Code, Budget, S, S1, Outcome) :-
    Budget > 0,
    reactive_children(Code, Children),
    do_node(fallback_node(Children), S, S1, Outcome),
    Outcome \= reactive(_).

% ---------------------------------------------------------------
% holds/2: minimal condition language for cond(C) leaves -- standard
% logical combinators plus domain-specific atomic conditions. Extend
% with more atomic conditions as new leaf/condition types are needed;
% nothing about do_node's cond(C) clause above needs to change.
% ---------------------------------------------------------------
holds(and(P,Q), S) :- holds(P,S), holds(Q,S).
holds(or(P,Q),  S) :- holds(P,S) ; holds(Q,S).
holds(neg(P),   S) :- \+ holds(P,S).

% halted_with_cond(Reason): reads the LAST halt's Reason via
% halted_with/2 -- lets a cond() leaf branch on how the PREVIOUS leg
% ended, e.g. cond(halted_with_cond(battery_depleted)). ACTION-
% INDEPENDENT (see halted_with/2's own note): Reason can just as well
% be a planning call's own completed(Algorithm,Goal,Code)/no_path
% (Algorithm,Goal,Code), e.g. cond(halted_with_cond(no_path(_,_,_)))
% to branch on "did the last planning attempt fail, regardless of
% which algorithm/goal" -- same wildcard convention as every other
% Reason shape below.
%
% TODO / KNOWN INTERFACE CHANGE: crashed/obstacle_in_bound/battery_under
% Reasons are compound terms carrying extra info (crashed(ObstacleId),
% obstacle_in_bound(Threshold,ObstacleId), battery_under(Threshold) --
% see trigger_crossing_time/9's own note), not bare atoms -- battery_depleted
% stays a bare atom, unaffected. To match ANY crash regardless of
% obstacle, write cond(halted_with_cond(crashed(_))), NOT
% cond(halted_with_cond(crashed)) (which no longer unifies against
% anything: a bare atom never matches a compound term of the same
% name); similarly obstacle_in_bound(_,_) / battery_under(_) for "any
% threshold/obstacle". To match SPECIFIC values, write e.g.
% cond(halted_with_cond(crashed(obs5))) or
% cond(halted_with_cond(battery_under(20))). In a BT.cpp XML tree (see
% module/translators/bt_to_prolog.py), this is HaltedWith's reason port,
% written the same way: reason="crashed(_)" or reason="crashed(obs5)".
holds(halted_with_cond(Reason), S) :- halted_with(Reason, S).

% distance_below(GX,GY,Threshold) / distance_equal(GX,GY,Threshold) /
% distance_over(GX,GY,Threshold): true iff the CURRENT position (at
% the current time, via now/2) is, respectively, below/exactly-equal-
% to/above Threshold distance from the EXPLICIT point (GX,GY) --
% PARAMETRIZED, same as obstacle_in_bound(Threshold)/battery_below
% (Threshold)/etc., not a lookup against any global "the goal" fact
% (there is no such fact anymore -- see the problem's own
% goal_formula.pl for where a plan's own goal information now lives
% entirely; distance_below is a DIFFERENT, complementary thing: a
% REACTIVE in-tree check, evaluated possibly many times at different
% situations as the policy runs, not a one-time post-hoc verification
% query). This is also what used to be baked directly into
% moveto_leg's own Status output (see leg_status/9's own note) --
% factored out into its own explicit, inspectable condition node
% instead, same as any other cond() leaf.
% Typical use: a fallback child that skips moveto entirely if already
% there, or a check placed right after a MoveTo to confirm it actually
% landed close enough to its own intended target --
%   fallback_node([cond(distance_below(11.675,11.525,0.3)), moveto_leg(CP,[collision,battery])])
% Pass the SAME point as whichever PlanWith node's own
% goal port targets, if that's the intent -- being explicit here means
% there is no longer a global/local goal-point mismatch to drift out
% of sync (the risk a single shared goal/2 fact used to carry).
% distance_equal/distance_over exist for the SAME reason
% battery_equal/battery_over exist alongside battery_below: a
% caller-chosen threshold at a different comparison, not a
% replacement.
holds(distance_below(GX,GY,Threshold), S) :-
    now(T, S), at(X,Y,T,S), dist(X,Y,GX,GY,D), D < Threshold.
holds(distance_equal(GX,GY,Threshold), S) :-
    now(T, S), at(X,Y,T,S), dist(X,Y,GX,GY,D), D =:= Threshold.
holds(distance_over(GX,GY,Threshold), S) :-
    now(T, S), at(X,Y,T,S), dist(X,Y,GX,GY,D), D > Threshold.

% obstacle_in_bound(Threshold): true iff the CURRENT position (at the
% current time, via now/2) is within Threshold of ANY obstacle. Same
% parameter, same underlying geometry as the obstacle_in_bound(Threshold)
% TRIGGER (see trigger_crossing_time/9) -- but this checks ONE point
% (the current situation) via a single call to
% obstacle_within_threshold/3, a plain boolean ProbLog predicate
% collision_geometry.py registers directly over
% within_obstacle_threshold/_min_clearance_all (the SAME primitive the
% trigger's bracket-scan calls repeatedly across a whole future
% trajectory) -- see that module's own header. No bracket-scan/
% bisection here: a condition only ever asks "is it true RIGHT NOW",
% not "will it ever become true during this walk".
holds(obstacle_in_bound(Threshold), S) :-
    now(T, S), at(X,Y,T,S),
    obstacle_within_threshold(X,Y,Threshold).

% line_of_sight_clear(ObstacleId,GX,GY): true iff the CURRENT position
% is NOT occluded from (GX,GY) by ObstacleId's own boundary. Same
% underlying primitive as the line_of_sight_clear(ObstacleId,GX,GY)
% TRIGGER (Bug0's own leave rule -- see trigger_crossing_time/10) --
% but this checks ONE point (the current situation) via a single call
% to line_of_sight_clear/5, exactly the same "single check, no
% bracket-scan" relationship obstacle_in_bound has to its own trigger.
holds(line_of_sight_clear(ObstacleId,GX,GY), S) :-
    now(T, S), at(X,Y,T,S),
    line_of_sight_clear(X,Y,ObstacleId,GX,GY).

% obstacle_on_path(Threshold): true iff the CURRENT position is within
% Threshold of an obstacle THIS WALK'S OWN TRAJECTORY actually enters
% somewhere across its full span -- see trigger_crossing_time/9's own
% note on how this differs from obstacle_in_bound. Needs the CURRENT
% walk's own (ControlPoints,Triggers,T0,SPrev) and its resolved Z --
% current_walk/5 + the z(do(startMoveto(...),SPrev),Z) lookup is the
% EXACT SAME pattern poss(haltMoveto(...)) already uses (works whether
% the walk is still in progress or has just ended, per current_walk/5's
% own doc comment); fails outright if no walk has started yet (nothing
% to check a trajectory against, at s0).
holds(obstacle_on_path(Threshold), S) :-
    current_walk(S, CP, Triggers, T0, SPrev),
    walk_duration(CP, Duration),
    z(do(startMoveto(CP,Triggers,_ActionCode,T0),SPrev), Z),
    zt(do(startMoveto(CP,Triggers,_ActionCode,T0),SPrev), Zt),
    now(T, S), at(X,Y,T,S),
    obstacle_on_path_within_threshold(CP,T0,Duration,Z,Zt,X,Y,Threshold).

% battery_below(Threshold): true iff the CURRENT battery level (at the
% current time, via now/2) is below Threshold. Same parameter, same
% underlying fluent as the battery_below(Threshold) TRIGGER (see
% trigger_crossing_time/9) -- but this is a single battery/3 lookup at
% the current situation, not first_battery_below_time's forward-looking
% closed-form solve; no black box involved at all, battery/3 is already
% plain Prolog.
holds(battery_below(Threshold), S) :-
    now(T, S), battery(Level, T, S),
    Level < Threshold.

% battery_equal(Threshold) / battery_over(Threshold): the SAME shape as
% battery_below above -- a single battery/3 lookup at the current
% situation, no black box, just a different arithmetic comparison.
% battery_equal uses exact arithmetic equality (=:=) per its own name;
% in a continuous, noise-driven model this is true only at whatever
% instant Level(T) genuinely passes through Threshold (see
% first_battery_equal_time's own note on why that's a well-defined,
% single instant, not a probability-zero non-event) -- checking it at
% an ARBITRARY current time will usually be false, same as asking "is
% it exactly noon" at a random moment.
holds(battery_equal(Threshold), S) :-
    now(T, S), battery(Level, T, S),
    Level =:= Threshold.
holds(battery_over(Threshold), S) :-
    now(T, S), battery(Level, T, S),
    Level > Threshold.

% ---------------------------------------------------------------
% holds_leg/9: the T-PARAMETERIZED twin of holds/2 above, used ONLY by
% first_becomes_false_time/9 below (in turn used ONLY by the
% guard_break(Cond,Code) trigger -- see trigger_crossing_time/11) --
% NOT part of the do_node/cond() interface itself (holds/2 stays the
% ONLY thing cond(C) ever calls). holds/2 always asks "is C true RIGHT
% NOW" via now(T,S) -- which, for an IN-PROGRESS leg's own situation
% term do(startMoveto(...),SPrev), always returns that leg's own START
% time T0, never a later instant WITHIN the leg (see the "no bracket-
% scan/bisection here" note above holds(obstacle_in_bound(...)) further
% up) -- so it cannot be reused as-is to watch a condition CONTINUOUSLY
% across a leg's own future. holds_leg(Cond,CP,T0,Duration,Z,Zt,Zb,B0,T)
% asks the SAME question at an EXPLICIT T instead, using the SAME flat
% (CP,T0,Duration,Z,Zt,Zb,B0) signature every trigger_crossing_time/11
% clause already receives (no situation term needed at all -- X,Y come
% from walk_noisy_point/8, battery Level from battery_at_leg/6 above,
% both already pure functions of T within one leg).
%
% ONE clause per condition in schema.yaml's conditions: list that can
% MEANINGFULLY vary within a single leg (i.e. everything except
% HaltedWith, which is history-based and cannot change mid-leg --
% bt_to_prolog.py's own guard-derivation pass rejects HaltedWith as an
% auto-derived guard BEFORE this predicate is ever reached, precisely
% to avoid a missing clause here silently reading as "already false at
% T0"). Adding a new schema.yaml condition later needs a matching
% clause HERE for it to be usable as an automatically-derived reactive
% guard -- or, if it genuinely can't vary mid-leg either, adding it to
% _NON_CONTINUOUS_CONDITIONS in bt_to_prolog.py instead, same as
% HaltedWith.
holds_leg(and(P,Q), CP,T0,Duration,Z,Zt,Zb,B0,T) :-
    holds_leg(P,CP,T0,Duration,Z,Zt,Zb,B0,T), holds_leg(Q,CP,T0,Duration,Z,Zt,Zb,B0,T).
holds_leg(or(P,Q), CP,T0,Duration,Z,Zt,Zb,B0,T) :-
    holds_leg(P,CP,T0,Duration,Z,Zt,Zb,B0,T) ; holds_leg(Q,CP,T0,Duration,Z,Zt,Zb,B0,T).
holds_leg(neg(P), CP,T0,Duration,Z,Zt,Zb,B0,T) :-
    \+ holds_leg(P,CP,T0,Duration,Z,Zt,Zb,B0,T).

holds_leg(distance_below(GX,GY,Threshold), CP,T0,Duration,Z,Zt,_Zb,_B0,T) :-
    walk_noisy_point(CP,T0,Duration,Z,Zt,T,X,Y),
    dist(X,Y,GX,GY,D), D < Threshold.
holds_leg(distance_equal(GX,GY,Threshold), CP,T0,Duration,Z,Zt,_Zb,_B0,T) :-
    walk_noisy_point(CP,T0,Duration,Z,Zt,T,X,Y),
    dist(X,Y,GX,GY,D), D =:= Threshold.
holds_leg(distance_over(GX,GY,Threshold), CP,T0,Duration,Z,Zt,_Zb,_B0,T) :-
    walk_noisy_point(CP,T0,Duration,Z,Zt,T,X,Y),
    dist(X,Y,GX,GY,D), D > Threshold.

holds_leg(obstacle_in_bound(Threshold), CP,T0,Duration,Z,Zt,_Zb,_B0,T) :-
    walk_noisy_point(CP,T0,Duration,Z,Zt,T,X,Y),
    obstacle_within_threshold(X,Y,Threshold).

holds_leg(obstacle_on_path(Threshold), CP,T0,Duration,Z,Zt,_Zb,_B0,T) :-
    walk_noisy_point(CP,T0,Duration,Z,Zt,T,X,Y),
    obstacle_on_path_within_threshold(CP,T0,Duration,Z,Zt,X,Y,Threshold).

holds_leg(line_of_sight_clear(ObstacleId,GX,GY), CP,T0,Duration,Z,Zt,_Zb,_B0,T) :-
    walk_noisy_point(CP,T0,Duration,Z,Zt,T,X,Y),
    line_of_sight_clear(X,Y,ObstacleId,GX,GY).

holds_leg(battery_below(Threshold), _CP,T0,Duration,_Z,_Zt,Zb,B0,T) :-
    battery_at_leg(T0,Duration,Zb,B0,T,Level), Level < Threshold.
holds_leg(battery_equal(Threshold), _CP,T0,Duration,_Z,_Zt,Zb,B0,T) :-
    battery_at_leg(T0,Duration,Zb,B0,T,Level), Level =:= Threshold.
holds_leg(battery_over(Threshold), _CP,T0,Duration,_Z,_Zt,Zb,B0,T) :-
    battery_at_leg(T0,Duration,Zb,B0,T,Level), Level > Threshold.

% first_becomes_false_time(+Cond,+CP,+T0,+Duration,+Z,+Zt,+Zb,+B0,
% -Tcross): the FIRST instant in (T0,T0+Duration] that Cond (already
% polarity-adjusted by bt_to_prolog.py -- see guard_break/2's own note
% in trigger_crossing_time/11 above) stops holding, given it holds at
% T0 -- the same "already true at T0 / genuine future search" two-shape
% convention every other first_*_time predicate in this file already
% follows (two MUTUALLY EXCLUSIVE clauses, on \+holds_leg(...,T0) vs
% holds_leg(...,T0), no cut needed -- same style as first_battery_
% below_time above), just GENERIC over any holds_leg/9-recognized Cond
% instead of one bespoke formula per condition. FAILS (no crossing) if
% Cond holds for the WHOLE walk -- same "absence, not sentinel"
% convention as everywhere else in this file.
%
% Bracket-scans bracket_samples/1 equal steps (the SAME config knob
% first_threshold_crossing_time's own black-box search already uses --
% see Section 0/verification.bracket_samples), then bisects the found
% bracket down to crossing_eps/1 -- returning the HIGH (verified-FALSE)
% end of the final bracket, never the low end or the midpoint, so a
% merge-grid-quantized restart seeded from Tcross is never seeded
% slightly EARLY (i.e. while Cond might still actually hold) -- same
% "never let approximation look safer than reality" convention as
% quantize_down/3.
first_becomes_false_time(Cond,CP,T0,Duration,Z,Zt,Zb,B0,T0) :-
    \+ holds_leg(Cond,CP,T0,Duration,Z,Zt,Zb,B0,T0).
first_becomes_false_time(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Tcross) :-
    holds_leg(Cond,CP,T0,Duration,Z,Zt,Zb,B0,T0),
    bracket_samples(N),
    TEnd is T0 + Duration,
    guard_bracket_scan(Cond,CP,T0,Duration,Z,Zt,Zb,B0,N,1,TEnd,T0,Tlo,Thi),
    crossing_eps(Eps),
    guard_bisect(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Eps,Tlo,Thi,Tcross).

% guard_bracket_scan(...,+N,+I,+TEnd,+Tprev,-Tlo,-Thi): walk forward
% from I=1 to N, one bracket_samples/1-th of the way from T0 to TEnd
% each step, stopping at the FIRST step where Cond has flipped from
% true (Tprev) to false (the current sample) -- Tlo/Thi bracket the
% crossing. Fails (no crossing anywhere in the scan) once I exceeds N,
% exactly mirroring every other first_*_time predicate's own "fails if
% it never happens" convention -- Cond held at every single sample.
% TWO MUTUALLY EXCLUSIVE clauses (on holds_leg(...,Ti) succeeding or
% failing) rather than if-then-else -- ProbLog's own Prolog dialect
% doesn't support '->'/2, same reason every other multi-case predicate
% in this file (e.g. first_battery_below_time above) is written this
% way instead.
guard_bracket_scan(Cond,CP,T0,Duration,Z,Zt,Zb,B0,N,I,TEnd,Tprev,Tlo,Thi) :-
    I =< N,
    Ti is T0 + (TEnd-T0) * I / N,
    holds_leg(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Ti),
    I1 is I + 1,
    guard_bracket_scan(Cond,CP,T0,Duration,Z,Zt,Zb,B0,N,I1,TEnd,Ti,Tlo,Thi).
guard_bracket_scan(Cond,CP,T0,Duration,Z,Zt,Zb,B0,N,I,TEnd,Tprev,Tprev,Ti) :-
    I =< N,
    Ti is T0 + (TEnd-T0) * I / N,
    \+ holds_leg(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Ti).

% guard_bisect(...,+Eps,+Tlo,+Thi,-Tcross): standard bisection --
% invariant Cond holds at Tlo, doesn't hold at Thi; narrows until the
% bracket is under Eps wide, then reports the FALSE end (see
% first_becomes_false_time's own note on why the high end, not the
% midpoint or the low end). Same mutually-exclusive-clauses style as
% guard_bracket_scan above, no '->'/2.
guard_bisect(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Eps,Tlo,Thi,Thi) :-
    Width is Thi - Tlo,
    Width =< Eps.
guard_bisect(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Eps,Tlo,Thi,Tcross) :-
    Width is Thi - Tlo,
    Width > Eps,
    Tmid is (Tlo + Thi) / 2,
    holds_leg(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Tmid),
    guard_bisect(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Eps,Tmid,Thi,Tcross).
guard_bisect(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Eps,Tlo,Thi,Tcross) :-
    Width is Thi - Tlo,
    Width > Eps,
    Tmid is (Tlo + Thi) / 2,
    \+ holds_leg(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Tmid),
    guard_bisect(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Eps,Tlo,Tmid,Tcross).

% ---------------------------------------------------------------
% 8. VERIFICATION-TIME SAMPLING (NOT part of the action theory) --
%    a purely deterministic choice of how finely to CHECK/REPORT the
%    already-closed-form at/4 fluent for VISUALIZATION purposes.
%    NOTE: collision DETECTION itself is now EXACT (see
%    first_collision_time above) -- num_samples/1 only controls the
%    resolution used for plotting and for on_track's drift reporting,
%    it no longer determines whether a collision is found at all.
%    num_samples/1 is now a config fact -- see
%    the problem's own config.yaml's verification.num_samples.
% ---------------------------------------------------------------

sample_frac(I, Frac) :- num_samples(N), Frac is I / N.

% final_situation(+S)/plan_outcome(-Outcome): drive plan/1's tree to a
% genuine true/false/world_too_large conclusion with a SINGLE do_node
% call, from s0 -- NOT the redescend-from-root loop this predicate
% used to be (see git history for that version, and this project's own
% conversation log for why it was replaced). Redescending on a
% reactive(_) halt is now entirely reactivesequence/reactivefallback's
% own job (see the CONTROL-FLOW REDESCEND TARGETS note above
% do_node(reactivesequence(...))) -- every reactive-classified trigger
% is tagged, at translation time, with the code of its own nearest
% enclosing reactive composite, which catches and locally restarts it,
% so a bare reactive(_) should never reach THIS call at all.
%
% If it somehow does (a translator bug -- bt_to_prolog.py is supposed
% to catch this at translation time as a hard failure, so this is a
% defense-in-depth check, not the primary guard), final_situation/1
% simply has no solution in that world (the first clause's own
% Outcome \= reactive(_) guard fails), and plan_outcome/1's OWN second
% clause reports it explicitly as reactive_escaped rather than silently
% folding it into any of the three legitimate outcomes -- P(plan_
% outcome(reactive_escaped)) > 0 in a report is the signal that
% something is structurally wrong with the translated plan, and is
% never expected to be nonzero for a plan that translated cleanly.
final_situation(S) :-
    plan(Node),
    do_node(Node, s0, S, Outcome),
    Outcome \= reactive(_).

% -- FULL OUTCOME ENUMERATION -----------------------------------------
% outcome_entry(+Action, -Entry): Entry = Code-Value for whichever
% ACTION or CONDITION marker carries a queryable outcome of its own --
% Value is Reason with its own trailing ActionCode STRIPPED back off
% (the exact same generic univ+append technique halted_with_pattern/3
% already uses to do the reverse) for a MoveTo's haltMoveto or a
% PlanWith's planned(...) marker, or simply Status for a cond(C,Code)'s
% own checked(Code,C,Status) marker (see that predicate's own note).
% Fails outright (no Entry, no choicepoint) for every other action
% (startMoveto, interrupt) -- those carry no reportable outcome of
% their own, only advance the clock/position.
outcome_entry(haltMoveto(_,Reason,_), Code-Pattern) :-
    Reason =.. [Functor|Args],
    append(Args0, [Code], Args),
    Pattern =.. [Functor|Args0].
outcome_entry(planned(_,Reason), Code-Pattern) :-
    Reason =.. [Functor|Args],
    append(Args0, [Code], Args),
    Pattern =.. [Functor|Args0].
outcome_entry(checked(Code,_,Status), Code-Status).

% history_outcomes(+S, -Entries): every outcome_entry/2 found ANYWHERE
% in S's own history, oldest-first -- the one GENERIC pass behind
% outcome_signature/1 below. UNLIKE halted_with/2 (finds ONE matching
% Reason, nondeterministically, one solution per match), this COLLECTS
% ALL of them at once, since a full outcome needs every action's/
% condition's own contribution together, not one at a time. No sort/
% dedup step needed: do_node/4 always visits a GIVEN tree's own
% children in the SAME left-to-right structural order regardless of
% which Reason/Status values actually occur, so two resolved worlds
% that reach the same SET of codes always visited them in the same
% relative order already -- Entries is already canonical across
% worlds, for free.
history_outcomes(s0, []).
history_outcomes(do(A,S), [Entry|Rest]) :-
    outcome_entry(A, Entry),
    history_outcomes(S, Rest).
history_outcomes(do(A,S), Rest) :-
    \+ outcome_entry(A, _),
    history_outcomes(S, Rest).

% outcome_signature(-Sig): Sig is the full list of every Code-Value
% pair this resolved world's own final_situation actually produced --
% one entry per action Reason and per condition check reached along
% the way, enclosing the WHOLE outcome of the system in one term rather
% than the separate per-action/per-condition MARGINALS halted_with_
% pattern/3 and any_condition_status/2 already report. Queried
% DELIBERATELY NON-GROUND (query(outcome_signature(_))), reusing the
% SAME "ProbLog reports one result row per distinct grounding instead
% of aggregating" behavior halted_with_pattern_detail/3 already relies
% on (see that predicate's own note) -- here that's exactly the wanted
% behavior: one row per DISTINCT combination of Reason/Condition values
% actually reached, each with its own aggregated probability, a full
% enumeration of every possible outcome instead of one marginal at a
% time. NOTE: a cond() leaf re-checked more than once in the SAME world
% (e.g. one sitting inside a reactive_children/2 list that gets
% redescended) contributes ONE Code-Value entry PER actual check, not
% just its last one -- a redescended condition whose own Status
% genuinely differed between checks shows up as two distinct entries
% for the same Code in Sig, which is correct (both really happened in
% that one world), if unusual to read.
outcome_signature(Sig) :-
    final_situation(S),
    history_outcomes(S, Sig).

% plan_outcome(Outcome): the WHOLE tree's own outcome, a first-class
% query -- P(plan_outcome(true)) is the BT-level analogue of verify_
% goal_formula, but based on Status/Outcome rather than an explicit
% goal formula. Outcome is one of true/false/world_too_large (the
% three legitimate outcomes; world_too_large now comes from a LOCAL
% reactivesequence/reactivefallback exhausting its own budget, see
% that note again) or reactive_escaped (a translator-bug signal, see
% final_situation/1's own note just above -- should never actually be
% nonzero).
plan_outcome(Outcome) :-
    plan(Node),
    do_node(Node, s0, _, Outcome),
    Outcome \= reactive(_).
plan_outcome(reactive_escaped) :-
    plan(Node),
    do_node(Node, s0, _, reactive(_)).

% replan_budget/1: the STARTING budget every reactivesequence/
% reactivefallback occurrence reads (fresh, independently) on its own
% first entry -- see reactivesequence_budgeted/5 and
% reactivefallback_budgeted/5 above. Previously this bounded ONE
% global whole-tree redescend loop; now each reactive composite gets
% its OWN counter seeded from this SAME shared value, decremented
% independently as THAT composite restarts. Still exists for
% ProbLog's own sake, not the robot's -- see the CONTROL-FLOW
% REDESCEND TARGETS note's own discussion of why nothing guarantees
% termination on its own (a degenerate zero-duration leg can restart
% forever without this bound, same reasoning as before, now just
% localized to whichever composite it's under).
replan_budget(1000).

% plan_time_span(+S, -T0, -TEnd): T0 is when the (most recent) walk
% started; TEnd is the wall-clock time the PLAN actually ends at --
% the time argument of the final haltMoveto/interrupt action, or (if
% the plan somehow ends mid-walk with no closing action) the walk's
% own natural completion time, as a safe fallback.
plan_time_span(S, T0, TEnd) :-
    current_walk(S, CP, T0),
    last_action_time(S, CP, T0, TEnd).

last_action_time(do(haltMoveto(T,_,_),_), _, _, T).
last_action_time(do(interrupt(T),_), _, _, T).
last_action_time(do(startMoveto(_,_,_,_),_), CP, T0, TEnd) :-
    walk_duration(CP, Duration),
    TEnd is T0 + Duration.
last_action_time(do(A,S), CP, T0, TEnd) :-
    A \= haltMoveto(_,_,_), A \= interrupt(_), A \= startMoveto(_,_,_,_),
    last_action_time(S, CP, T0, TEnd).

sample_time(I, S, T) :-
    plan_time_span(S, T0, TEnd),
    sample_frac(I, Frac),
    T is T0 + (TEnd-T0)*Frac.

% sample_walk_frac(I,S,Frac): the sampled instant expressed as a
% fraction of the WALK's own full nominal duration (for comparing
% against nominal_at/4, which is parametrized the same way) --
% distinct from sample_frac/2, which is a fraction of however much
% of the plan actually got executed (0..(TEnd-T0)) if interrupted.
sample_walk_frac(I, S, WalkFrac) :-
    plan_time_span(S, T0, TEnd),
    current_walk(S, CP, T0),
    walk_duration(CP, Duration),
    sample_frac(I, Frac),
    T is T0 + (TEnd-T0)*Frac,
    WalkFrac is (T - T0) / Duration.


% ---------------------------------------------------------------
% 7. SAFETY QUERIES.
%    SAFETY QUERIES READ THE ACTUAL OUTCOME, THEY DO NOT RE-DERIVE IT.
%    An earlier version of crashed_in/1 independently recomputed
%    first_collision_time over the walk's FULL nominal duration,
%    regardless of whether some OTHER cause (e.g. obstacle_in_bound)
%    had already halted the walk earlier in this same world. That's a
%    real bug, not a subtlety: it answers "would this trajectory
%    eventually reach the collision margin if nothing else stopped
%    it", which can disagree with "did the executed plan actually
%    crash" the moment more than one halting cause can compete to be
%    first -- exactly what Triggers introduces. The fix: every "did X
%    actually happen" query below reads the Reason ALREADY RECORDED
%    in the resolved situation (via halted_with/2), rather than
%    recomputing anything -- this is correct BY CONSTRUCTION for any
%    number of triggers, present or future, since it never touches
%    the trigger-specific detection machinery at all, only the
%    situation's own history.
% ---------------------------------------------------------------

% halted_with(+Reason, +S): TRUE iff SOMEWHERE in S's action history
% EITHER a haltMoveto OR a planWith occurred with exactly this Reason
% -- ACTION-INDEPENDENT on purpose: a query built on this (or on
% halted_with_pattern/3 further down, which is layered directly on
% top of this) shouldn't have to know or care whether a given Reason
% came from a MoveTo leg finishing or a planning call finishing, only
% that SOME action in the history produced it. The two Reason
% vocabularies never collide by construction: a MoveTo's own Reasons
% (completed(Code), crashed(ObstId,Code), battery_under(Threshold,Code),
% ...) and a planning call's own (completed(Algorithm,Goal,Code),
% no_path(Algorithm,Goal,Code)) differ in ARITY even where they share a
% functor name (completed/1 vs completed/3), so Prolog unification
% already keeps "any MoveTo completion" (completed(_)) and "any
% planning success" (completed(_,_,_)) as two distinct, never-
% overlapping queries with no extra disambiguation needed. Searches the
% WHOLE history (not just the most recent halt/plan), so a future
% multi-leg plan where an earlier leg/planning call had a different
% fate than the final one is still handled correctly.
halted_with(Reason, do(haltMoveto(_,Reason,_), _)).
halted_with(Reason, do(planned(_Algorithm,Reason), _)).
halted_with(Reason, do(_A, S)) :- halted_with(Reason, S).

% visited(+Loc, +Tol, +S): TRUE iff the robot ACTUALLY ARRIVED at
% Loc=point(GX,GY) -- the endpoint of SOME already-completed leg
% (Reason=completed(_ActionCode), Status=true, i.e. the walk wasn't
% cut short by a trigger), AND its own actual (noisy) final position
% is within Tol of Loc. Status=true alone no longer implies this (see
% leg_status/9's own note: that check used to be baked into Status,
% but is now the separate, explicit distance_below/3 condition --
% visited/3 re-derives the SAME dist/5 comparison directly, via at/4,
% rather than depending on any particular BT node having been placed
% in the tree to check it) -- anywhere in S's
% history. SAME "search the whole history" shape as halted_with/2
% above, and for the same reason needs no separate persistence/frame
% axiom: situation histories only ever grow by appending do(...), so
% "did this ever happen in S's past" is already monotonic for free --
% a fluent that starts false and, once made true, stays true in every
% situation built on top of that one, exactly the shape a multi-leg
% "visited(A,Tol,S), visited(B,Tol,S), visited(C,Tol,S)" goal formula
% needs, verified
% the same way verify_goal_formula/any_collision already are (P(...) over
% resolved worlds, since a collision or battery depletion partway
% through a multi-leg plan can genuinely truncate the history before
% a later waypoint is ever reached -- this is NOT something you could
% check by inspecting the plan's own static structure instead).
%
% NOTE: for a plain LINEAR Sequence of legs (no Fallback in between),
% seq_node/1's own definition (do_node(seq_node([Child|Rest]),S,S1,
% Outcome):-do_node(Child,S,S2,true),...) already REQUIRES each
% child's Status=true before the next one even starts -- so checking
% visited/3 on just the LAST waypoint already logically entails every
% earlier one was visited too; only worth checking each individually
% once a Fallback sits somewhere before the waypoint you care about.
visited(point(GX,GY), Tol, do(haltMoveto(T,completed(_ActionCode),true), S)) :-
    current_walk(S, CP, _Triggers, _T0, _SPrev),
    leg_target(CP, GX, GY),
    at(X,Y,T, do(haltMoveto(T,completed(_ActionCode),true), S)),
    dist(X,Y,GX,GY,D),
    D =< Tol.
visited(Loc, Tol, do(_A, S)) :- visited(Loc, Tol, S).

% -- crashed_in(S) / battery_depleted_in(S) / obstacle_in_bound_in(S) /
%    battery_under_in(S): trivial one-liners reading the actual Reason,
%    not re-deriving anything. crashed(_)/obstacle_in_bound(_,_) use
%    wildcards since those Reasons carry WHICH obstacle (and, for
%    obstacle_in_bound, WHICH threshold too -- see trigger_crossing_time/9's
%    own note) -- unbound arguments here correctly mean "regardless of
%    which obstacle/threshold". battery_under(_) similarly wildcards
%    its Threshold. Any FUTURE trigger's own "did it actually fire"
%    diagnostic is exactly this same one-liner pattern -- no
%    trigger-specific re-derivation logic to get wrong.
% Every Reason shape below now carries an extra TRAILING ActionCode
% argument (see tag_reason/3, above poss(haltMoveto(...))) -- one more
% wildcard per functor, same "regardless of which" convention as the
% obstacle/threshold wildcards already here.
crashed_in(S) :- halted_with(crashed(_,_), S).
battery_depleted_in(S) :- halted_with(battery_depleted(_), S).
obstacle_in_bound_in(S) :- halted_with(obstacle_in_bound(_,_,_), S).
obstacle_on_path_in(S) :- halted_with(obstacle_on_path(_,_,_), S).
battery_under_in(S) :- halted_with(battery_under(_,_), S).
battery_equal_in(S) :- halted_with(battery_equal(_,_), S).
battery_over_in(S) :- halted_with(battery_over(_,_), S).

% -- crashed_obstacle(ObstacleId,S) / obstacle_in_bound_obstacle
%    (Threshold,ObstacleId,S) / obstacle_on_path_obstacle(Threshold,
%    ObstacleId,S) / battery_under_threshold(Threshold,S) /
%    battery_equal_threshold(Threshold,S) /
%    battery_over_threshold(Threshold,S): the direct accessors for
%    WHICH obstacle/threshold -- unlike the *_in(S) checks above, the
%    extra argument(s) are left bound, not wildcarded. Fails (no
%    solution) if S didn't halt for that reason, same "absence, not
%    sentinel" convention as everywhere else. Situation argument S is
%    LAST in every one of these, per Reiter's own convention (see
%    module/contracts/vocabulary.yaml's own note on this -- these six
%    used to put S FIRST, an inconsistency with visited/2, halted_with
%    /2, at/4, and battery/3 above, all of which already had S last;
%    fixed here since nothing outside this file's own definitions
%    referenced the old argument order).
% One more trailing wildcard each, same reason as the *_in(S) family
% above -- ActionCode itself isn't exposed as an argument HERE (these
% predicates' own job is "which obstacle/threshold", unchanged); query
% halted_with_cond(crashed(ObstacleId,ActionCode)) directly (see
% tag_reason/3's own note) when ActionCode itself is what's wanted.
crashed_obstacle(ObstacleId, S) :- halted_with(crashed(ObstacleId,_), S).
obstacle_in_bound_obstacle(Threshold, ObstacleId, S) :- halted_with(obstacle_in_bound(Threshold,ObstacleId,_), S).
obstacle_on_path_obstacle(Threshold, ObstacleId, S) :- halted_with(obstacle_on_path(Threshold,ObstacleId,_), S).
battery_under_threshold(Threshold, S) :- halted_with(battery_under(Threshold,_), S).
battery_equal_threshold(Threshold, S) :- halted_with(battery_equal(Threshold,_), S).
battery_over_threshold(Threshold, S) :- halted_with(battery_over(Threshold,_), S).

% -- GENERIC per-action safety-query machinery -----------------------
% match_wild(+PatternArgs, +ActualArgs): PatternArgs unifies against
% ActualArgs position-by-position, where the GROUND ATOM 'wild' in
% PatternArgs matches ANY value at that position (an argmin ObstacleId
% that's only known at RUNTIME, never at translation time) while every
% OTHER PatternArgs element must match EXACTLY (a Threshold/GX/GY/Cond
% already known when the query itself was generated). 'wild' is a
% plain ATOM here, deliberately NOT a genuine unbound Prolog variable
% (e.g. '_') -- see halted_with_pattern/3's own note on why that
% distinction is exactly what keeps a query(...) declaration reporting
% ONE aggregated probability instead of ProbLog silently splitting it
% into one row per distinct grounding.
match_wild([], []).
match_wild([wild|Ws], [_|As]) :-
    match_wild(Ws, As).
match_wild([W|Ws], [W|As]) :-
    W \= wild,
    match_wild(Ws, As).

% halted_with_pattern(+GroundPattern, +ActionCode, +S): true iff S's
% history contains a halt OR a planning call (see halted_with/2's own
% action-independence note) whose Reason, once ActionCode (tag_reason/
% 3's own trailing argument -- see poss(haltMoveto(...)) and
% do_node(planWith(...))'s own notes; ALWAYS the trailing argument,
% regardless of which action produced it) is stripped back off,
% MATCHES GroundPattern -- i.e. GroundPattern is the
% UNTAGGED Reason0 shape (crashed(wild), guard_break(battery_over
% (70.0)), battery_under(20), completed, ...), with any part that's
% only known at RUNTIME (an argmin ObstacleId for crashed/obstacle_
% in_bound/obstacle_on_path) written as the literal atom 'wild' by
% whoever constructs it, and any part that's ALREADY known at
% TRANSLATION time (a guard's own Cond, a trigger's own Threshold/
% GX/GY) kept LITERAL -- this is what correctly distinguishes e.g.
% guard_break(battery_over(70.0)) from guard_break(neg(obstacle_in_
% bound(0.6))) as two separate rows, rather than collapsing every
% guard into one combined "guard_break" total (a bare-functor grouping
% would lose exactly that distinction, since guard_break's own
% semantic identity lives entirely in its first argument, not its
% functor name). GroundPattern MUST be fully ground (every position
% either 'wild' or a literal, never a bare Prolog variable) -- a real
% unbound variable here would make query(any_reason_pattern_by_action
% (Pattern,ActionCode)) itself non-ground, and ProbLog reports ONE
% result row per distinct GROUNDING of a non-ground query rather than
% aggregating them (verified directly against ProbLog's own engine
% before writing this) -- 'wild' avoids that entirely by keeping the
% query term itself ground, while match_wild/2 still supplies the
% "any value here" matching semantics internally, never exposed to the
% query's own outer term. module/contracts/goal_formula_check.py's
% generate_safety_queries builds exactly this GroundPattern per
% (MoveTo, trigger) pair at translation time, mirroring bt_to_prolog.py
% 's own trigger-name -> Reason-functor mapping (e.g. battery_below ->
% battery_under).
halted_with_pattern(GroundPattern, ActionCode, S) :-
    halted_with(Reason, S),
    Reason =.. [Functor|Args],
    append(Args0, [ActionCode], Args),
    GroundPattern =.. [Functor|WildArgs],
    match_wild(WildArgs, Args0).

% any_reason_pattern(+GroundPattern): P(GroundPattern occurred, from
% ANY action) -- the auto-generated replacement for hand-picked
% aggregates like the old any_collision/any_battery_depletion (now
% any_reason_pattern(crashed(wild)) / any_reason_pattern(battery_
% depleted), generated automatically for every reason this problem's
% own tree can actually produce, not just the two someone thought to
% hand-write).
any_reason_pattern(GroundPattern) :-
    final_situation(S), halted_with_pattern(GroundPattern, _ActionCode, S).

% any_reason_pattern_by_action(+GroundPattern, +ActionCode): the SAME
% probability, split by WHICH MoveTo occurrence produced it -- e.g.
% any_reason_pattern_by_action(crashed(wild), a1) vs. (..., a2) is
% exactly "20% on the goto-goal leg, 10% on the goto-home leg" for a
% combined any_reason_pattern(crashed(wild)) of 30%.
any_reason_pattern_by_action(GroundPattern, ActionCode) :-
    final_situation(S), halted_with_pattern(GroundPattern, ActionCode, S).

% halted_with_pattern_detail(+Pattern, +ActionCode, +S): the DELIBERATE
% OPPOSITE of halted_with_pattern/3's own ground-only discipline --
% Pattern here is expected to contain a genuine UNBOUND Prolog variable
% at whichever position halted_with_pattern's own GroundPattern would
% have written as the atom 'wild' (e.g. crashed(_) rather than
% crashed(wild)). Plain unification via =.. does the matching, with NO
% match_wild/2 involved -- this is intentional: a query built on THIS
% predicate is exactly the non-ground case halted_with_pattern/3's own
% note warns about, and that's the whole point here, not a bug to
% avoid -- ProbLog reports one result row PER DISTINCT GROUNDING of a
% non-ground query, so any_reason_pattern_detail_by_action(crashed(_),
% a1) is what actually ENUMERATES every concrete obstacle a1 could
% have crashed into, each with its own probability, as the sub-rows
% underneath any_reason_pattern_by_action(crashed(wild),a1)'s own
% single aggregated total -- see main.py's print_reason_breakdown for
% how the two are nested together in the report.
halted_with_pattern_detail(Pattern, ActionCode, S) :-
    halted_with(Reason, S),
    Reason =.. [Functor|Args],
    append(Args0, [ActionCode], Args),
    Pattern =.. [Functor|Args0].

% any_reason_pattern_detail_by_action(+Pattern, +ActionCode): see
% halted_with_pattern_detail/3's own note -- one query(...) declaration
% here (Pattern containing a genuine variable, e.g. crashed(_)) yields
% MANY result rows, one per obstacle/whatever-was-runtime-only actually
% observed, each already showing its own CONCRETE value substituted in
% (e.g. any_reason_pattern_detail_by_action(crashed(obs5),a1)) --
% module/contracts/goal_formula_check.py's generate_safety_queries only
% emits this companion query for a (Pattern,ActionCode) pair whose own
% GroundPattern actually contains 'wild' somewhere; a Pattern with
% nothing runtime-only in it (battery_under(20), guard_break(Cond),
% completed, ...) has no meaningful detail level beyond its own
% aggregate and gets no companion query at all.
any_reason_pattern_detail_by_action(Pattern, ActionCode) :-
    final_situation(S), halted_with_pattern_detail(Pattern, ActionCode, S).

% any_condition_status(+Code, +Status): P(the cond() leaf identified by
% Code was actually checked, AND its own Status came out this way, in
% the world resolved by final_situation) -- the direct analogue of
% any_reason_pattern_by_action/2 above, but for CONDITIONS instead of
% action Reasons: Code identifies WHICH cond(C,Code) occurrence in the
% tree (same per-occurrence-code idiom as ActionCode), Status is true
% or false (never a Pattern -- a condition's own outcome has no
% runtime-only argument structure to distinguish, unlike a MoveTo's own
% Reason). module/contracts/goal_formula_check.py's generate_safety_
% queries emits one query(any_condition_status(Code,true)) and one
% query(any_condition_status(Code,false)) per condition occurrence
% bt_to_prolog.py assigned a code to, mirroring exactly how it already
% emits one any_reason_pattern_by_action query per (Pattern,ActionCode)
% pair. Fails outright (contributes zero probability) in any world
% where this Code's own cond() leaf was never reached at all -- same
% "absence, not sentinel" convention checked_with/4 itself already
% follows, so P(any_condition_status(Code,true)) + P(any_condition_
% status(Code,false)) need NOT sum to 1.0 (exactly like a MoveTo's own
% Reason probabilities not summing to 100% when the leg was never
% reached in some worlds).
any_condition_status(Code, Status) :-
    final_situation(S), checked_with(Code, _C, Status, S).

% last_halt(-Reason): a cond() leaf that reads off WHY the MOST RECENT
% moveto_leg halted. Works by searching BACKWARD through S, but ONLY
% ever skipping past BOOKKEEPING markers -- checked(_,_,_) from a
% cond(C,Code) leaf (see do_node(cond(...))'s own note) and
% planned(_,_) from a PlanWith call (see do_node(planWith(...))'s own
% note) -- neither of which represents anything PHYSICALLY happening
% (no time elapsed, no position/battery change, nothing that could make
% "the last halt" stale); it genuinely STOPS (fails, no further skip)
% at any OTHER action (startMoveto, interrupt), which DO mean something
% new has happened since the last halt.
%
% UPDATED from an EARLIER version that pattern-matched ONLY S's own
% OUTERMOST layer directly against do(haltMoveto(...),_), reasoning
% that do_node(moveto_leg(...),...) always ends with haltMoveto as its
% very last step and no do_node level (plain or reactive) ever layered
% another action on top of that S1 on the way up -- which was true
% right up until cond(C,Code) started recording its own checked(...)
% marker (see that predicate's own note): a DistanceBelow placed right
% after a MoveTo -- exactly this project's own now-standard idiom --
% appends ONE MORE do(...) layer on top of the halt before a LATER
% branch's own cond(last_halt(...)) (or recover_obstacle/1, built on
% top of it) ever gets to read it, which the old direct-unification
% version could no longer see through. Skipping bookkeeping markers
% restores the original "single deterministic unification, not a
% search" behavior for the REAL halt underneath, without giving up
% cond()'s own traceability: this still can only ever produce the ONE
% most recent halt (unlike halted_with/2, which walks the WHOLE history
% and can match more than one past action), so it stays a single
% ProbLog world instead of branching into one world per historical
% match. Fails outright (no solution) if S isn't shaped like a
% just-halted moveto (skipping bookkeeping markers) at all -- same
% "absence, not sentinel" convention as everywhere else.
%
% NOT YET GENERIC across leaf/action types -- a caveat for whoever adds
% the next reactive-capable leaf (e.g. a robotic-arm action halting via
% its own haltArmMove(...) instead of haltMoveto(...)): the first
% clause below will simply FAIL to match such an S, silently, not with
% an error -- do(haltArmMove(...),_) doesn't unify with do(haltMoveto
% (...),_), so neg(last_halt(...))-based guards elsewhere would
% trivially succeed even though something genuinely just halted. Every
% new reactive-capable leaf type needs its OWN base clause added here
% (or all leaves funneled through one shared halt-action functor with a
% Kind tag, instead of a differently-named action per leaf type).
holds(last_halt(Reason), do(haltMoveto(_T,Reason,_Status),_SPrev)).
holds(last_halt(Reason), do(checked(_,_,_), S)) :- holds(last_halt(Reason), S).
holds(last_halt(Reason), do(planned(_,_), S)) :- holds(last_halt(Reason), S).

% KNOWN LIMITATION, found while adding cond(C,Code)'s own checked(...)
% marker (Option B): cond(neg(last_halt(...))) -- problem3's OWN Bug0
% guard, "retry the direct path unless the last halt was an
% obstacle_on_path" -- makes ProbLog's grounder raise a spurious
% "non-ground probabilistic clause" error against z/2's own template
% clause in config_generated.pl, even though the actual derivation
% needs no such thing (verified directly: replan_budget(1) still fails
% just as fast, so it isn't the known reactive-redescend blowup; a bare
% cond(last_halt(...)) with NO neg() wrapper, or a cond() whose own
% Status doesn't change do_node(cond(...))'s own S1 shape, both work
% fine). The negation itself is the trigger -- ProbLog's own grounding
% of \+ over a goal that (transitively, via this predicate's own
% checked(...)-skipping clauses above) reaches back through the
% situation apparently forces broader exploration than a plain,
% non-negated call needs, tripping over z/2's non-ground template
% clause somewhere in that wider search. Root-caused down to "negation
% over last_halt/1 specifically, once cond() stopped being S1=S" but
% NOT resolved further: every reformulation tried (recursion moved out
% of holds/2 into a separate skip_bookkeeping/2 helper; a single do_node
% (cond(...)) clause with a shared checked(...) term shape regardless
% of Status) still reproduced it, pointing at something in ProbLog's
% own negation-grounding internals rather than a fixable shape in this
% theory's own clauses. Affects ONLY problem3's own hand-written
% plan_generated.pl (the one tree in this project using neg(last_halt
% (...)) at all) -- every translator-generated tree (problems 0/1/2/4)
% neither uses last_halt/recover_obstacle nor is affected. problem3 was
% already unable to complete a run within any practical timeout before
% this (a separate, pre-existing reactive-redescend combinatorial
% blowup, see perf_diag_two_hop/), so this is a new FAILURE MODE on an
% already-nonfunctional tree, not a regression on a working one.

% recover_obstacle(-ObstacleId): a cond() leaf that RETRIEVES which
% obstacle the branch that led here just halted against, wildcarding
% Threshold -- built on last_halt/1 above, NOT on the whole-history
% obstacle_on_path_obstacle/3 (see that predicate's own family comment
% above it): a Fallback whose first branch plans+walks straight
% (watching obstacle_on_path(Threshold) as a trigger) and whose SECOND
% branch needs to know WHICH obstacle to hand planWith(follow_boarder
% (ObstacleId,...),...) -- Golog's own fallback_node semantics threads the SITUATION S
% forward from a failed branch into the next one (see fallback_node's
% own do_node note near the top of this section), but NEVER a raw
% Prolog variable binding a failed branch happened to make, so the
% second branch cannot simply "reuse" a variable the first branch
% bound; it has to look the obstacle back up from S. Built on last_halt
% /1 rather than halted_with/2 SPECIFICALLY so this stays correct with
% more than one obstacle in play (e.g. straight -> hits obstacle A ->
% follow A's boundary -> line of sight clears -> straight again -> hits
% a DIFFERENT obstacle B): halted_with/2 would match BOTH A's and B's
% halt actions once B is also in S's history, grounding two alternative
% (one stale) worlds; last_halt/1 can only ever see the outermost one,
% so recover_obstacle always reports the obstacle THIS branch is
% actually reacting to. Fails (no solution) if the most recent halt
% wasn't an obstacle_on_path one at all -- same "absence, not
% sentinel" convention as everywhere else -- so a Fallback branch
% guarded by cond(recover_obstacle(Obst)) simply doesn't apply unless
% there really is one to recover.
holds(recover_obstacle(ObstacleId), S) :-
    holds(last_halt(obstacle_on_path(_Threshold,ObstacleId,_ActionCode)), S).

% -- overall collision probability (exact) --------------------------
any_collision :- final_situation(S), crashed_in(S).

% -- overall battery-depletion probability (exact) -------------------
any_battery_depletion :- final_situation(S), battery_depleted_in(S).

% -- sample_index_for_time(+T,+T0,+Duration,-I): bucket an exact time
%    into the nearest reporting sample, for continuity with plotting.
sample_index_for_time(T,T0,Duration,I) :-
    num_samples(N),
    FracRaw is (T-T0)/Duration,
    IReal is FracRaw*N,
    IRound is round(IReal),
    I is max(0, min(N, IRound)).

% -- Feature 2b analogue: which reporting sample the (exact) first
%    collision falls nearest to -- a genuine PMF, since crashed_in/1
%    is itself all-or-nothing per world and each crashing world maps
%    to exactly one bucket. T is read DIRECTLY off the actual halted
%    situation (no re-derivation) -- if some OTHER cause preempted
%    collision in this world, S's outermost haltMoveto simply won't
%    match Reason=crashed(_), and this correctly contributes nothing.
%    _ObstacleId is deliberately unbound/ignored here -- first_hit is
%    a PMF over WHEN, not WHICH obstacle; see crashed_obstacle/2 for
%    that question, e.g. crashed_obstacle(ObstacleId,S) alongside
%    final_situation(S) for a specific resolved situation.
first_hit(I) :-
    final_situation(S),
    S = do(haltMoveto(Tcross,crashed(_ObstacleId,_ActionCode),_), _),
    current_walk(S, CP, _Triggers, T0, _SPrev),
    walk_duration(CP, Duration),
    sample_index_for_time(Tcross,T0,Duration,I).

% -- Feature 2a analogue: P(the exact collision, if any, falls at or
%    before reporting sample N) ----------------------------------
hit_by(N) :-
    final_situation(S),
    S = do(haltMoveto(Tcross,crashed(_ObstacleId,_ActionCode),_), _),
    current_walk(S, CP, _Triggers, T0, _SPrev),
    walk_duration(CP, Duration),
    sample_index_for_time(Tcross,T0,Duration,I),
    I =< N.

% ---------------------------------------------------------------
% FUTURE EXTENSION NOTE: "safety" here is meant to cover EVERY cause
% that could prevent the robot from reaching the goal. Currently
% there are seven, ALL expressed as ordinary Triggers entries (see
% the TRIGGERS section and trigger_crossing_time/9 far above):
% collision (first_collision_time / crashed_in / any_collision),
% battery depletion (first_battery_depletion_time /
% battery_depleted_in / any_battery_depletion), obstacle_in_bound
% (obstacle_in_bound_in/2, first_threshold_crossing_time),
% obstacle_on_path (obstacle_on_path_in/2, first_on_path_crossing_time),
% battery_below (battery_under_in/2, first_battery_below_time),
% battery_equal (battery_equal_in/2, first_battery_equal_time), and
% battery_over (battery_over_in/2, first_battery_over_time).
% Adding an EIGHTH cause later -- a mechanical fault, a comms timeout,
% whatever -- means exactly:
%   (a) one more trigger_crossing_time/10 clause, giving its own
%       Reason and crossing-time computation
%   (b) a dedicated *_in(S) exact-detection predicate (one line, via
%       halted_with/2) + a corresponding any_* diagnostic query, if
%       you want it separately reportable
%   (c) including the new trigger's name in whichever leg(s)' own
%       explicit Triggers list should react to it
% Nothing else in the theory needs to change; earliest_halt/10 and
% verify_safe below already handle an arbitrary Triggers list with no
% further edits.
% ---------------------------------------------------------------

% -- Feature 1 analogue: deterministic nominal-path safety w.r.t.
%    EVERY cause in THIS LEG'S OWN Triggers list, using the SAME
%    shared earliest_halt/9 that Poss(haltMoveto(...)) itself uses
%    for real execution -- just with noise fixed at its zero/modal
%    value (Z=0.0, Zb=0.0) instead of resolved per-world values.
%    Because this is the SAME predicate, not a separate
%    reimplementation, it cannot drift out of sync the way the
%    earlier crashed_in/1 did. If collision/battery aren't in this
%    leg's Triggers, verify_safe correctly won't flag them, matching
%    exactly what real execution would (or wouldn't) halt on.
% B0 here MUST be leg_start_battery/3, not a raw battery(B0,T0,SPrev)
% read: T0 (from current_walk/5 on the ALREADY-RESOLVED final
% situation) is whatever quantized start time real resolution actually
% used for this leg (see poss(startMoveto(...))'s own note), so this
% has to read the SAME quantized B0 that leg's own real
% Poss(haltMoveto(...)) used -- otherwise this "was it safe from THIS
% leg's own start" check could disagree with what the leg ACTUALLY
% started with, exactly the kind of drift this predicate's own header
% comment (just above) warns against.
verify_safe :-
    final_situation(S),
    current_walk(S, CP, Triggers, T0, SPrev),
    walk_duration(CP, Duration),
    leg_start_battery(T0, SPrev, B0),
    earliest_halt(CP,Triggers,T0,Duration,0.0,0.0,0.0,B0, completed,_,_).

plan_route_blocked :- \+ verify_safe.

% -- on-track: does the noisy position stay within tolerance of ----
%    the nominal spline position at sample I? (still sampled -- this
%    is a REPORTING diagnostic about drift magnitude, not a safety
%    detector, so fixed-resolution sampling remains appropriate here)
%    tolerance/1 is now a config fact -- see
%    the problem's own config.yaml's tolerances.on_track.

on_track(I) :-
    final_situation(S),
    current_walk(S, ControlPoints, _),
    sample_time(I, S, T),
    at(X,Y,T,S),
    sample_walk_frac(I, S, WalkFrac),
    nominal_at(NX,NY,WalkFrac,ControlPoints),
    dist(X,Y,NX,NY,D),
    tolerance(Tol),
    D =< Tol.

% goal_reached (the old single-point "arrives near THE goal AND the
% walk's actual recorded outcome was completed" query) is GONE --
% superseded by goal_formula.pl's own goal_formula/1 (verified,
% earlier, to compute the EXACT SAME probability as goal_reached did
% for a single-leg plan), and by visited/2's own more general history
% search for anything beyond that. See this file's own
% verify_goal_formula wrapper further down, and
% the problem's own goal_formula.pl for where "did the plan
% succeed" is now formalized -- there is no longer a global goal/2
% fact anywhere in this theory for a query like this to read.

% ============================================================
% 9. THE POLICY -- an explicit start/end ACTION PAIR:
%        startMoveto(ControlPoints, T0)  ...  haltMoveto(T,Reason)
%    ControlPoints is computed at runtime by a planWith leaf (see
%    plan/1's own note further down) -- there is no longer a
%    hand-authored control_points/1 fact anywhere in this theory
%    (plan_generation/plan/current_plan.pl, which used to hold one
%    alongside start/2 and goal/2, is gone entirely; see start/1's own
%    note below and goal_formula.pl for where the two things it used
%    to carry now live instead).
%
%    T and Reason are left as FREE VARIABLES -- they are DERIVED by
%    Poss(haltMoveto(T,Reason),S), never chosen by the plan: Reason
%    comes out `completed` if the walk finishes without ever coming
%    within the safety margin of an obstacle in that resolved world,
%    or `crashed(ObstacleId)` (with T = the exact collision time, and
%    ObstacleId = which obstacle) otherwise.
%
%    Default plan below runs the walk to its natural halt (no
%    interruption). To model an interruption, replace it with
%    something like:
%
%        plan(seq(startMoveto(CP,0), seq(interrupt(5.0), nil))).
%
%    which ends the walk at T=5.0 regardless of the walk's own
%    natural completion or collision time -- poss(interrupt(T),S)
%    requires moving(S) and T strictly before whichever of those two
%    happens first in the resolved world, so this composes with any
%    future action that also wants to interrupt a walk: just give it
%    its own poss/2 clause requiring moving(S) plus its own trigger,
%    same pattern as interrupt/1 above.
% ============================================================

% start/1 is now a config.yaml fact (the problem's own
% initial_situation.start_x/start_y, via config_generated.pl,
% consulted via problem_data.pl near the top of this file) -- it used to live in
% plan_generation/plan/current_plan.pl, alongside a global goal/2 fact
% and a control_points/1 static fallback, both now GONE: goal
% information lives entirely in the problem's own goal_formula.pl
% (see verify_goal_formula further down), and control_points/1 has had
% no consumer since planWith started computing ControlPoints at
% runtime.

% plan/1 is no longer hand-written here -- it comes from the problem's
% own plan_generated.pl (consulted via problem_data.pl, see Section 0
% above), generated by module/translators/bt_to_prolog.py translating
% the REAL BT.cpp v4 XML tree at the problem's own behavior_tree.xml
% against module/contracts/schema.yaml (main.py does this automatically
% before every run, exactly like it already does for config_generated.pl).
% This is now the single source of truth for the POLICY'S SHAPE: to
% change the tree, edit behavior_tree.xml and re-run, don't hand-edit a
% plan/1 clause here.
%
% Every moveto_leg node in the XML states its Triggers EXPLICITLY, in
% place -- there is no default_triggers/1 fact and no sugar moveto_leg/1
% form anywhere in this theory (removed deliberately: a plan's
% protection level should be visible where the leg is written, not
% inherited from configuration or a convenience default elsewhere).
% [collision,battery] (the shipped tree's own choice) is a PLAIN CHOICE,
% exactly like any other Triggers list -- nothing about collision/
% battery is hardcoded into the action theory itself (see the TRIGGERS
% section above trigger_crossing_time/9). An empty triggers="" list
% would give a genuinely unprotected leg that completes its full
% nominal duration even through an obstacle's margin or an empty
% battery; adding obstacle_in_bound(0.6) (triggers="collision;battery;
% obstacle_in_bound(0.6)") would ALSO reactively halt when an obstacle
% first comes within 0.6 metres, and battery_below(20)
% (triggers="collision;battery;battery_below(20)") would ALSO halt the
% first time the battery drops under 20%, alongside (not instead of)
% the fixed collision/battery(=0%) triggers.
%
% Since the translator handles arbitrary Sequence/Fallback nesting and
% every schema.yaml action/condition, sequence/fallback/multi-leg
% policies -- and conditions like DistanceBelow/HaltedWith/ObstacleInBound/
% BatteryBelow -- are already expressible in the XML with no further
% changes here; see
% bt_to_prolog.py's own header for the blackboard-to-Prolog-variable
% translation this relies on (e.g. giving two different PlanWith
% nodes distinct blackboard keys, same as the CP1/CP2
% convention this file already documents for hand-written multi-leg
% plans). Only the AUTOMATIC generation of multi-leg XML trees and
% reacting to a planWith/moveto_leg FAILURE by re-planning are future
% work; the theory and the translator both already support authoring
% either by hand in the XML.

% goal_formula/1 is hand-authored, tied to THIS PARTICULAR plan's own
% waypoints -- see the problem's own goal_formula.pl header for the
% full rationale and the "must be kept in sync with behavior_tree.xml"
% caveat. This is the ONLY place a plan's own goal information lives --
% there is no separate global goal/2 fact anywhere in this theory (see
% distance_below/3's own note). It is a UNIFORM formula (Reiter's sense -- one
% free situation argument, every fluent inside applied to exactly it);
% verify_goal_formula below is what actually applies it AT
% final_situation, same "zero-arg convenience wrapper hardwired to
% final_situation" shape as any_collision/plan_outcome below.
% Both plan/1 and goal_formula/1 are consulted via problem_data.pl --
% see Section 0 above, near the top of this file.

verify_goal_formula :- final_situation(S), goal_formula(S).

% ============================================================
% 10. QUERIES
%
% NONE hardcoded here anymore -- every query(...) this problem needs
% (verify_goal_formula, plan_outcome(true/false/world_too_large),
% plan_outcome(reactive_escaped), and one any_reason_pattern(...)/
% any_reason_pattern_by_action(...,ActionCode) pair per Reason this
% problem's own tree can actually produce) is written into this
% problem's own queries_generated.pl by module/contracts/
% goal_formula_check.py's generate_safety_queries, called right after
% goal_formula.pl's own validation (see main.py) -- a problem-
% independent theory file can't itself vary per-problem, but the
% generated file it consults (via problem_data.pl -- see Section 0)
% can, and now ALL of it does, not just the one any_battery_depletion
% special case this used to carry by hand.
%
% hit_by/1, first_hit/1, on_track/1, verify_safe/0, and
% plan_route_blocked/0 are all still DEFINED above (Section 7/8) --
% simply never queried by the generator, so they stay ungrounded
% (ProbLog only grounds what a query(...) or something it depends on
% actually reaches), not deleted. Add them to generate_safety_queries'
% own output if per-sample hazard/drift reporting is wanted again.
%
% plan_outcome(reactive_escaped) remains the runtime safety net for
% the "a reactive trigger's own code matched no enclosing reactive
% composite" translator-bug case -- see final_situation/1 and
% plan_outcome/1's own notes above. Always expected to be exactly 0
% for a correctly-translated plan; a nonzero value here is a bug
% report, not a legitimate outcome.
% ============================================================