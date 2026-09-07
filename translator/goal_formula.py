"""
Parses a problem's goal_formula.pl (a single restricted Prolog clause,
`goal_formula(S) :- Conjunct1, Conjunct2, ... .`, per goal_formula_check.py's
own "exactly one clause, no disjunction, no local helper predicates"
limitation -- see vocabulary.yaml's header) and translates it into a
BehaVerify LTLSPEC code_statement, using vocabulary.yaml's own fluent
vocabulary as the dispatch table.

SCOPE: only the fluents actually needed so far are implemented (visited/3,
and battery_depleted_in/1 built from every MoveTo leaf's own `failure`
status). Anything else raises NotImplementedError naming the exact
unsupported predicate/arity, rather than silently mistranslating -- extend
_DISPATCH as new goal formulas arrive. NOTE: visited/3 (with an explicit
Tol argument) is the CURRENT signature (problog_project/module/theory/
basic_action_theory.pl) -- an older visited/2 (no Tol, using a since-removed
global goal_tolerance constant) is no longer produced by this codebase.

TEMPORAL SHAPE: every fluent in vocabulary.yaml is either a HISTORY fluent
(TRUE iff something happened anywhere up to S -- visited/2, the *_in/1
family, halted_with/2) or a POINT-IN-TIME fluent (battery/3, at/4, moving/1,
paired with now/2). A history fluent's own BehaVerify translation already
means "at any point during the run", so it becomes `(finally, ...)`
directly. A point-in-time fluent checked at goal_formula's own situation
argument S means "in the FINAL reached state" -- expressing "final state"
in plain LTL needs an explicit done-marker this translator does not yet
add, so those raise NotImplementedError until one is designed.
"""
import re

from . import leaf_library

_TOKEN_RE = re.compile(r"""
    \s*(?:
        (?P<comment>%[^\n]*)
      | (?P<clauseop>:-)
      | (?P<punct>[(),.])
      | (?P<number>-?\d+\.\d+|-?\d+)
      | (?P<var>[A-Z_][A-Za-z0-9_]*)
      | (?P<atom>[a-z][A-Za-z0-9_]*)
    )
""", re.VERBOSE)


def _tokenize(text):
    pos = 0
    tokens = []
    while pos < len(text):
        match = _TOKEN_RE.match(text, pos)
        if match is None:
            if text[pos:].strip() == '':
                break
            raise ValueError('Cannot tokenize goal_formula.pl near: {!r}'.format(text[pos:pos + 30]))
        pos = match.end()
        if match.lastgroup == 'comment':
            continue
        tokens.append((match.lastgroup, match.group().strip()))
    return tokens


def _parse_term(tokens, i):
    kind, text = tokens[i]
    if kind == 'number':
        return ('num', float(text)), i + 1
    if kind == 'var':
        return ('var', text), i + 1
    if kind == 'atom':
        name = text
        i += 1
        if i < len(tokens) and tokens[i] == ('punct', '('):
            i += 1
            args = []
            while True:
                arg, i = _parse_term(tokens, i)
                args.append(arg)
                if tokens[i] == ('punct', ','):
                    i += 1
                    continue
                break
            if tokens[i] != ('punct', ')'):
                raise ValueError('Expected ")" in goal_formula.pl at token {}'.format(i))
            i += 1
            return ('compound', name, args), i
        return ('atom', name), i
    raise ValueError('Unexpected token in goal_formula.pl: {}'.format(tokens[i]))


def parse_goal_formula(path):
    """Returns (head_var_name, [body_conjuncts]) -- each conjunct a parsed term."""
    with open(path, 'r', encoding='utf-8') as f:
        text = f.read()
    tokens = _tokenize(text)
    # goal_formula ( S ) :- Conjunct , Conjunct ... .
    if tokens[0] != ('atom', 'goal_formula'):
        raise ValueError('goal_formula.pl must define goal_formula/1, found: {}'.format(tokens[0]))
    head, i = _parse_term(tokens, 0)
    if head[0] != 'compound' or head[1] != 'goal_formula' or len(head[2]) != 1 or head[2][0][0] != 'var':
        raise ValueError('goal_formula.pl head must be goal_formula(S) with a single situation variable.')
    situation_var = head[2][0][1]
    if tokens[i] != ('clauseop', ':-'):
        raise ValueError('goal_formula.pl must be a rule (":-"), not a fact.')
    i += 1
    conjuncts = []
    while True:
        term, i = _parse_term(tokens, i)
        conjuncts.append(term)
        if i < len(tokens) and tokens[i] == ('punct', ','):
            i += 1
            continue
        break
    if tokens[i] != ('punct', '.'):
        raise ValueError('goal_formula.pl clause must end with "."')
    return situation_var, conjuncts


def _require_point(term):
    if term[0] != 'compound' or term[1] != 'point' or len(term[2]) != 2:
        raise NotImplementedError('Expected a literal point(X,Y), found: {}'.format(term))
    return term[2][0][1], term[2][1][1]  # (x, y) floats


def _wrap_finally(condition, bound):
    """
    `(finally, condition)` (unbounded F) if bound is None, else
    `(finally_bounded, [0, bound], condition)` (nuXmv's bounded F[0,bound]).

    WHY: unbounded LTL `F` requires nuXmv's general Buchi-tableau/fair-cycle
    construction, which can blow up in practice even on a small, cheap
    model (confirmed directly: this exact model checks fine as an
    INVARSPEC -- i.e. the model itself is cheap to build -- but blows up
    nuXmv's memory on plain LTLSPEC F(...)). A BOUNDED F[0,k] instead
    reduces to a finite unrolling (much closer to reachability checking),
    which is typically dramatically cheaper. Safe to use here because the
    grid is small and MoveTo advances at most one cell/tick, so any
    genuinely reachable goal is reachable well within a modest tick bound
    -- see translator/main.py's --bound flag.
    """
    if bound is None:
        return '(finally, {})'.format(condition)
    return '(finally_bounded, [0, {}], {})'.format(bound, condition)


def _translate_visited(args, config, _factory, situation_var, bound):
    # visited/3: visited(Loc, Tol, S) -- see problog_project/module/theory/
    # basic_action_theory.pl's own visited/3 clauses. Tol is nominally an
    # explicit distance threshold (metres), but is NOT used to build an
    # abs/max-based tolerance condition here (an earlier version did) --
    # confirmed empirically (bisecting against a real nuXmv run) that
    # carrying abs/max arithmetic through LTL's temporal wrapping
    # (finally/finally_bounded) is dramatically more expensive for nuXmv
    # than a plain equality condition, even though the SAME abs/max
    # condition checks fine and fast as a plain INVARSPEC -- i.e. the
    # cost is specific to the combination of this arithmetic with LTL's
    # own tableau/bounded-unrolling construction, not the condition or
    # the model alone. This isn't a loss of fidelity for THIS dynamics,
    # though: MoveTo's own arrival check (leaf_library.make_move_to) is
    # already exact cell equality (target_x/target_y are always integer
    # cells), so the robot never lands "close but not exactly on" a
    # target -- checking exact equality on the goal cell is equivalent
    # to any Tol>=1 cell here, not an approximation of it. Tol is still
    # validated (must be a literal number) so a genuinely fractional-cell
    # semantics elsewhere would be caught, not silently ignored.
    loc, tol, s = args
    if s != ('var', situation_var):
        raise NotImplementedError('visited/3\'s own situation argument must be goal_formula\'s own S (no nested situations).')
    if tol[0] != 'num':
        raise NotImplementedError('visited/3\'s own Tol argument must be a literal number, found: {}'.format(tol))
    x_m, y_m = _require_point(loc)
    cx, cy = config.to_cell(x_m), config.to_cell(y_m)
    condition = '(and, (eq, x, {}), (eq, y, {}))'.format(cx, cy)
    return _wrap_finally(condition, bound)


def _translate_battery_depleted_in(args, _config, factory, situation_var, bound):
    (s,) = args
    if s != ('var', situation_var):
        raise NotImplementedError('battery_depleted_in/1\'s own situation argument must be goal_formula\'s own S.')
    move_to_aliases = factory.move_to_aliases
    if not move_to_aliases:
        return '(False)'
    failures = ['(failure, {})'.format(alias) for alias in move_to_aliases]
    formula = failures[0]
    for extra in failures[1:]:
        formula = '(or, {}, {})'.format(formula, extra)
    return _wrap_finally(formula, bound)


_DISPATCH = {
    ('visited', 3): _translate_visited,
    ('battery_depleted_in', 1): _translate_battery_depleted_in,
}


def translate(path, config, factory, bound=None):
    """Returns a single code_statement string: the AND of every conjunct's
    own translation. Raises NotImplementedError for any conjunct whose
    predicate/arity isn't in _DISPATCH yet. `bound`, if given, makes every
    `finally` a bounded `finally_bounded [0,bound]` instead -- see
    _wrap_finally's own docstring for why that matters for nuXmv's
    verification cost, not just generation."""
    situation_var, conjuncts = parse_goal_formula(path)
    pieces = []
    for term in conjuncts:
        if term[0] != 'compound':
            raise NotImplementedError('goal_formula.pl conjunct must be a predicate call, found: {}'.format(term))
        key = (term[1], len(term[2]))
        if key not in _DISPATCH:
            raise NotImplementedError(
                'goal_formula.pl uses {}/{}, which this translator does not yet support -- '
                'add a case to translator/goal_formula.py\'s _DISPATCH table.'.format(*key)
            )
        pieces.append(_DISPATCH[key](term[2], config, factory, situation_var, bound))
    formula = pieces[0]
    for extra in pieces[1:]:
        formula = '(and, {}, {})'.format(formula, extra)
    return formula
