% HAND-WRITTEN, not translator-generated -- module/translators/
% bt_to_prolog.py does not yet support the shape behavior_tree.xml
% describes for this problem (<Inverter>, ObstacleOnPath used as a
% plain Condition leaf, and dynamically binding obstacle_id from a
% trigger's own halt reason are all documented as KNOWN GAPS in that
% file's own header). This is the Golog-level term that tree is meant
% to translate to, written directly so the reactive-evaluation
% mechanism (leg_status/8's three-valued Status, evaluate_plan/4) can
% actually be tested end-to-end before the translator catches up.
%
% CURRENTLY DOES NOT RUN AT ALL (a separate matter from the translator
% gaps above): grounding fails fast with a spurious "non-ground
% probabilistic clause" ProbLog error, traced to cond(neg(last_halt
% (...)))'s own interaction with cond(C,Code)'s checked(...) marker --
% see the KNOWN LIMITATION note directly above holds(recover_obstacle
% (...)) in basic_action_theory.pl for the full investigation. Even
% before that, this tree's own reactive-redescend chaining already
% couldn't complete within any practical timeout (see perf_diag_two_hop/),
% so this replaces one non-functional state with another, not a
% regression from a working one.
%
% Two branches under one Fallback, matching behavior_tree.xml's own
% Bug0 shape, PLUS an explicit distance_below/3 early exit as the
% first branch (same idiom DistanceBelow already uses elsewhere in
% this project -- "a fallback child that skips moveto entirely if
% already there"). A distance_below/3 check is also placed right
% after EACH branch's own moveto_leg -- this used to be baked directly
% into moveto_leg's own Status output (see leg_status/9's own note in
% basic_action_theory.pl), now an explicit, hand-placed condition
% instead, same as any other cond() leaf:
%
%   1. distance_below(11.675,11.525,0.3) -- already there, nothing to do.
%   2. Guarded by cond(neg(last_halt(obstacle_on_path(_,_)))): plan
%      straight to goal (PlanWith code a3); walk it, watching
%      obstacle_on_path(0.6) as a trigger; then confirm it actually
%      landed within tolerance.
%   3. Recover WHICH obstacle branch 2's own trigger fired against
%      (recover_obstacle/1, built on last_halt/1 -- see basic_action_
%      theory.pl's own note there); follow its offset boundary
%      (PlanWith code a4); walk it, watching line_of_sight_clear
%      (Obst1,11.675,11.525) as a trigger; then confirm it actually
%      landed within tolerance.
%
% Both planWith calls now also carry their own ActionCode (a3, a4,
% continuing the a1/a2 numbering already used by the two moveto_leg
% occurrences below -- same shared per-occurrence counter
% next_action_code() would assign if this tree DID translate
% automatically) as their own 4th argument, exactly mirroring MoveTo's
% own ActionCode -- see do_node(planWith(...))'s own note in
% basic_action_theory.pl. So a failed astar/straight/voronoi/
% follow_boarder call now records completed(Algorithm,Goal,ActionCode)
% or no_path(Algorithm,Goal,ActionCode) in the situation history (Goal
% is `none` for follow_boarder, which has no goal point), distinguishing
% WHICH of several planning attempts in the same tree produced it, the
% same way crashed(ObstacleId,ActionCode) already distinguishes WHICH
% MoveTo leg crashed.
%
% Every cond(C) below likewise now carries its own Code (c1..c5, in
% reading order -- same next_condition_code() counter bt_to_prolog.py
% would assign) as a 2nd argument, mirroring ActionCode's own
% "identify which OCCURRENCE this is" role but for condition leaves --
% see do_node(cond(C,Code),...)'s own note in basic_action_theory.pl.
% This is a REQUIRED interface change, not optional: do_node(cond(C),
% S,S,Status) (1-arg, no Code) no longer has a matching clause at all
% now that every cond() leaf records a checked(Code,C,Status) marker
% into the situation on its own.
% No third "resume to goal" leg is hand-chained after branch 3 -- it
% doesn't need one. Every trigger above is classified `reactive` (see
% leg_status/9) and tagged with the SAME code, rc1, identifying the
% outer fallback_node itself (now reactivefallback(rc1)) as the
% nearest enclosing reactive composite for both of them -- matching
% what the translator would assign, since neither trigger has a
% reactive composite any closer than the tree's own root. So EITHER
% branch halting via its own trigger makes reactivefallback(rc1)
% restart its own children (fresh, via reactive_children(rc1,...)
% below) from the halted situation: once line of sight clears, branch
% 2 (straight to goal) gets tried again from the new position, this
% time (assuming the geometry allows it) reaching the goal without
% re-triggering obstacle_on_path at all -- resumption falls out of the
% reactive mechanism itself, for free.
%
% Branch 2's guard, cond(neg(last_halt(obstacle_on_path(_,_)))), is
% what actually makes that resumption (and the initial descent) work,
% and it's worth spelling out why it's phrased this way rather than as
% a live geometric check:
%   - At s0, nothing has halted yet, so last_halt/1 has no solution,
%     neg(...) succeeds, and branch 2 is tried -- covering the "very
%     first attempt" case with no separate "is anything undefined yet"
%     guard needed.
%   - Right after branch 2's own obstacle_on_path trigger halts,
%     last_halt/1 reports EXACTLY that reason, neg(...) fails, so
%     branch 2 (as a whole) reports Status=false instead of looping on
%     `reactive` forever -- letting the enclosing fallback_node fall
%     through to branch 3 on THIS SAME descent.
%   - Right after branch 3's own line_of_sight_clear trigger halts,
%     last_halt/1 reports THAT reason (not obstacle_on_path), so
%     neg(...) succeeds again and branch 2 is retried -- this is why
%     the guard is checked against S's own recorded history rather
%     than current_walk/5's live geometry: current_walk/5 would still
%     be reporting branch 3's OWN boundary-following path (which is,
%     by construction, held within threshold of the obstacle for its
%     whole length), permanently blocking branch 2 from ever being
%     retried after a successful recovery.
plan(reactivefallback(rc1)).

reactive_children(rc1, [
    cond(distance_below(11.675,11.525,0.3),c1),
    seq_node([
        cond(neg(last_halt(obstacle_on_path(_,_,_))),c2),
        planWith(straight, point(11.675,11.525), PathS, a3),
        moveto_leg(PathS, [collision,battery,obstacle_on_path(0.6,rc1)], a1),
        cond(distance_below(11.675,11.525,0.3),c3)
    ]),
    seq_node([
        cond(recover_obstacle(Obst1),c4),
        planWith(follow_boarder(Obst1,0.6), point(0.0,0.0), PathFB, a4),
        moveto_leg(PathFB, [collision,battery,line_of_sight_clear(Obst1,11.675,11.525,rc1)], a2),
        cond(distance_below(11.675,11.525,0.3),c5)
    ])
]).
