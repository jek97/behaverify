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


def _translate_visited(args, config, _factory, situation_var):
    # visited/3: visited(Loc, Tol, S) -- see problog_project/module/theory/
    # basic_action_theory.pl's own visited/3 clauses. Tol is an explicit
    # distance threshold (metres), not baked into a global goal_tolerance
    # constant any more -- reuse the SAME squared-distance test (no sqrt)
    # leaf_library.py's DistanceBelow/Equal/Over checks use, so a "visited"
    # goal formula and an explicit DistanceBelow condition node agree on
    # what "close enough" means.
    loc, tol, s = args
    if s != ('var', situation_var):
        raise NotImplementedError('visited/3\'s own situation argument must be goal_formula\'s own S (no nested situations).')
    if tol[0] != 'num':
        raise NotImplementedError('visited/3\'s own Tol argument must be a literal number, found: {}'.format(tol))
    x_m, y_m = _require_point(loc)
    cx, cy = config.to_cell(x_m), config.to_cell(y_m)
    # ceil, not nearest -- see leaf_library.py's _distance_check for why a
    # small Tol must never round down to 0 (would make even exact arrival fail).
    tol_cells = max(1, config.to_cells_ceil(tol[1]))
    condition = leaf_library.squared_distance_condition('x', 'y', cx, cy, tol_cells, 'lte')
    return '(finally, {})'.format(condition)


def _translate_battery_depleted_in(args, _config, factory, situation_var):
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
    return '(finally, {})'.format(formula)


_DISPATCH = {
    ('visited', 3): _translate_visited,
    ('battery_depleted_in', 1): _translate_battery_depleted_in,
}


def translate(path, config, factory):
    """Returns a single code_statement string: the AND of every conjunct's
    own translation. Raises NotImplementedError for any conjunct whose
    predicate/arity isn't in _DISPATCH yet."""
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
        pieces.append(_DISPATCH[key](term[2], config, factory, situation_var))
    formula = pieces[0]
    for extra in pieces[1:]:
        formula = '(and, {}, {})'.format(formula, extra)
    return formula
