% ============================================================
% 10. QUERIES -- REPLACED for this diagnostic run by run.sh, which
% strips basic_action_theory.pl's own default QUERIES section (the
% ~48 report queries) and appends this file in its place. Those
% default queries pull in evaluate_plan/4's full replan_budget(1000)
% recursion, which is expensive for a different reason than what this
% diagnostic isolates -- keeping them here would make the timeout
% results ambiguous about which cost is being measured.
% ============================================================

% diag_full_node/1: the EXACT problem3 Bug0 tree, verbatim from
% problems/problem3/plan_generated.pl (see that file, and module/
% theory/basic_action_theory.pl's own last_halt/1 and recover_obstacle
% comments, for the full design rationale).
diag_full_node(fallback_node([
    cond(at_goal(11.675,11.525,0.3)),
    seq_node([
        cond(neg(last_halt(obstacle_on_path(_,_)))),
        planWith(straight, point(11.675,11.525), PathS),
        moveto_leg(PathS, [collision,battery,obstacle_on_path(0.6)])
    ]),
    seq_node([
        cond(recover_obstacle(Obst1)),
        planWith(follow_boarder(Obst1,0.6), point(0.0,0.0), PathFB),
        moveto_leg(PathFB, [collision,battery,line_of_sight_clear(Obst1,11.675,11.525)])
    ])
])).

% diag_two_hop: chains exactly TWO full evaluations of the tree above.
% The first is FORCED to conclude `reactive` -- this mirrors do_node
% (Node,S0,S1,reactive), the first clause of evaluate_plan/4, at
% replan_budget(1). The second call's own Status is left completely
% open (`_X`) -- this mirrors evaluate_plan/4's own SECOND call at
% Budget=0, which can land on a genuine true/false (the "done" clause)
% OR fall into the world_too_large clause if it's ALSO reactive; one
% wildcarded call covers both alternatives in a single query. No third
% hop is possible either way, exactly matching replan_budget(1)'s own
% shape.
%
% What this isolates: every SINGLE evaluation of diag_full_node/1 (any
% status, from a fixed situation) was independently confirmed fast --
% as was every individual sub-piece (planWith alone, moveto_leg with/
% without the obstacle_on_path trigger, branch 3 alone, branch 2's own
% true/false/reactive split). It's specifically CHAINING two
% evaluations together, where the second starts from the continuous,
% noise-dependent situation the first produced, that blows up ProbLog's
% exact inference at the default 5x5x5 noise table (125 combinations
% per hop) -- run.sh runs this same query at that default noise level
% and again at a reduced 3x1x1 table (3 combinations per hop) to check
% whether the noise-table combinatorics are the actual driver.
diag_two_hop :-
    diag_full_node(Node),
    do_node(Node, s0, S1, reactive),
    do_node(Node, S1, _S2, _X).

query(diag_two_hop).
