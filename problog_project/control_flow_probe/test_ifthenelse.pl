% test_ifthenelse.pl -- does THIS machine's ProbLog support if-then-else
% (Cond -> Then ; Else)?
%
% If -> is supported, test(X) should commit to X=yes (q(1) holds), and
% only test(yes) should appear in the query results (probability 1.0).
% If -> is unsupported, ProbLog will raise problog.engine.UnknownClause
% naming '->'/2 or ';'/2 before any results print.
q(1).

test(X) :- (q(1) -> X = yes ; X = no).

query(test(X)).
