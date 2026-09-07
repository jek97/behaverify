% test_cut.pl -- does THIS machine's ProbLog support the cut (!)?
%
% p(X) has two solutions, X=1 and X=2. first/1 tries to commit to the
% FIRST one via cut. If ! is supported, only first(1) should appear in
% the query results (probability 1.0), because the cut prevents
% backtracking into p(2). If ! is unsupported, ProbLog will raise
% problog.engine.UnknownClause naming '!' before any results print.
p(1).
p(2).

first(X) :- p(X), !.

query(first(X)).
