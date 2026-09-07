% goal_formula.pl
%
% Hand-authored, tied to THIS PARTICULAR plan's own behavior_tree.xml
% -- "must be kept in sync with whichever tree it verifies": this file
% is now the SINGLE place mission-goal information lives (there is no
% global goal/2 fact any more); if the tree's own waypoints change,
% update this file too.
%
% goal_formula(S) is a UNIFORM formula in S (Reiter's own sense: S is
% the ONLY free situation term in the body, and every fluent that
% takes a situation argument is applied to exactly this S) -- it is
% NOT hardwired to final_situation itself, so it can in principle be
% checked at any situation; basic_action_theory.pl's own
% verify_goal_formula wrapper is what actually applies it at
% final_situation, matching every other top-level verification query
% (any_collision, plan_outcome, ...).
%
% Written directly in Prolog -- and(P,Q)/or(P,Q) (the mini-language
% cond() leaves use inside a BT tree) is a SEPARATE vocabulary for a
% SEPARATE purpose (serializable as BT.cpp XML attribute text); this
% file has no such constraint, so it just uses Prolog's own ','/2 for
% conjunction directly.
%
% Current formula: the shipped single-leg plan (behavior_tree.xml)
% has exactly one MoveTo, planned toward this planWith leg's own goal point
% (11.675,11.525) -- so "visited the goal" is the only waypoint to
% check today. A multi-leg A->B->C plan would list each leg's own
% endpoint here as its own visited(...) conjunct, in the SAME order
% the tree visits them (see visited/3's own note on why, for a plain
% linear Sequence, checking only the LAST waypoint already implies
% every earlier one -- listing them all here is for clarity/
% robustness against a Fallback being added later, not because it's
% strictly needed today).
goal_formula(S) :-
    visited(point(11.675,11.525), 0.3, S).
