"""
translator: bridges a `example_problem_to_solve/<problem>/` directory
(BT.cpp XML tree + config.yaml + map/obstacles + goal_formula.pl, following
the vocabulary in schema.yaml / vocabulary.yaml) into a single BehaVerify
`.tree` DSL file that `behaverify nuxmv ... --generate --ltl` can consume.

This is a one-way, best-effort SOUND ABSTRACTION, not an exact reproduction:
continuous positions become a bounded grid, continuous time becomes one
tick == one grid cell of travel, and ProbLog's probabilistic noise becomes
nuXmv's nondeterministic environment choice (see README.md in this
directory for the full list of modeling decisions).
"""
