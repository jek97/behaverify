% test_once.pl -- does THIS machine's ProbLog support once/1?
%
% r(X) has two solutions, X=1 and X=2. test_once/1 tries to commit to
% the FIRST one via once/1. If once/1 is supported (and actually
% commits, not just an alias for call/1), only test_once(1) should
% appear in the query results (probability 1.0). If once/1 is
% unsupported, ProbLog will raise problog.engine.UnknownClause naming
% once/1 before any results print. If it's silently treated as a
% no-op passthrough, BOTH test_once(1) and test_once(2) will appear --
% that failure mode is worth distinguishing from an outright error.
r(1).
r(2).

test_once(X) :- once(r(X)).

query(test_once(X)).
