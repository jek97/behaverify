% goal_formula.pl (problem4)
%
% Simplified to a single conjunction -- goal_formula_check.py doesn't
% yet handle disjunction (';'/2) in goal_formula.pl, so this checks
% only "reached the goal directly", the same shape as problem3's own
% goal_formula.pl. This problem exists for the performance
% investigation (see behavior_tree.xml's own header), not as a
% meaningful mission goal, so a strict "either goal or home" success
% condition isn't needed here -- the queries that actually matter for
% this diagnostic are plan_outcome/1 and any_battery_depletion.
goal_formula(S) :-
    visited(point(22.275,2.075), S).
