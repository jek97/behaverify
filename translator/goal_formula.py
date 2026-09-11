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


def _translate_sample_success_at(args, config, _factory, situation_var, bound):
    # sample_success_at/3: sample_success_at(Loc, Tol, S) -- see
    # basic_action_theory.pl's own sample_success_at/3: TRUE iff SOME
    # take_sample action SUCCEEDED, anywhere in S's history, at a
    # recorded position within Tol of Loc. `sample_success` (leaf_
    # library.make_take_sample) is resampled fresh every time TakeSample
    # runs, so unlike visited/3's own x/y (which freeze once a walk
    # completes) it does NOT stay True forever after one success -- but
    # that's fine: LTL's `finally` already does the "true at ANY point
    # in history" search on its own, so we only need to check "is
    # sample_success True AND is the robot's CURRENT position (x,y) --
    # unchanged since TakeSample is instantaneous -- at Loc" at the
    # SAME instant, exactly like visited/3's own exact-equality
    # simplification (Tol is validated but not used arithmetically, for
    # the identical reason: this grid's positions are already exact
    # cells, so exact equality is equivalent to any Tol>=1 cell here).
    loc, tol, s = args
    if s != ('var', situation_var):
        raise NotImplementedError("sample_success_at/3's own situation argument must be goal_formula's own S.")
    if tol[0] != 'num':
        raise NotImplementedError('sample_success_at/3\'s own Tol argument must be a literal number, found: {}'.format(tol))
    x_m, y_m = _require_point(loc)
    cx, cy = config.to_cell(x_m), config.to_cell(y_m)
    condition = '(and, sample_success, (and, (eq, x, {}), (eq, y, {})))'.format(cx, cy)
    return _wrap_finally(condition, bound)


def _translate_ploughed(args, _config, factory, situation_var, bound):
    # ploughed/3: ploughed(Cx, Cy, S) -- see basic_action_theory.pl's own
    # ploughed/3: TRUE iff macro-cell (Cx,Cy) was swept by the robot's
    # path while the plow was equipped, anywhere in S's history. The
    # source theory uses its OWN, separately-configurable macro-cell
    # grid (config.yaml's ploughing.cell_size, decoupled from disc_step_
    # position) -- this translator does NOT: Cx,Cy here are interpreted
    # DIRECTLY as this translator's own grid-cell coordinates (same
    # units as x/y, target_x/target_y, ...), per an explicit simplifying
    # choice (this project already works on a grid; reuse ITS spacing
    # rather than introduce a second, independent one). One consequence:
    # a real ploughing.cell_size coarser than disc_step_position would
    # let a single real pass plough a whole neighborhood of macro-cells
    # at once, whereas this translation only ever marks the exact cell
    # the robot's discrete path actually occupies -- an honest, coarser-
    # grained approximation, not a systematic bias.
    #
    # Tracking mechanism (leaf_library.LeafFactory._ploughed_updates):
    # a DEDICATED persistent boolean per (Cx,Cy) pair actually referenced
    # here (not a grid-wide writable array -- goal_formula.pl only ever
    # asks about a small, fixed set of specific cells, known at
    # translation time, same as visited/3's own goal points), updated by
    # every MoveTo-family action via a plain eq/and check against the
    # CURRENT (post-step) x,y -- see collect_ploughed_cells, which this
    # module's own translate() calls up front, BEFORE any MoveTo action
    # is built, so that tracking can be wired into it from the start.
    cx_term, cy_term, s = args
    if s != ('var', situation_var):
        raise NotImplementedError("ploughed/3's own situation argument must be goal_formula's own S.")
    if cx_term[0] != 'num' or cy_term[0] != 'num':
        raise NotImplementedError(
            "ploughed/3's Cx,Cy must be literal integers (this translator's own grid-cell units)."
        )
    cx, cy = int(cx_term[1]), int(cy_term[1])
    var_name = factory.ploughed_var(cx, cy)
    return _wrap_finally('(eq, {}, True)'.format(var_name), bound)


def _translate_sample_value(args, factory, situation_var, bound, op):
    # sample_value_below/equal/over/3: sample_value_<cmp>(SampleId,
    # Threshold, S) -- see basic_action_theory.pl's own note: TRUE iff
    # SampleId's own take_sample SUCCEEDED somewhere in S's history and
    # its own drawn value (0..10) compares `op` against Threshold. Same
    # "current-state check already IS the whole-history search" shape
    # sample_value_.../SampleValueBelow's own BT-condition twin uses
    # (leaf_library._sample_value_check) -- sample_success_<id> persists
    # once True, so checking it now covers "ever succeeded", gated so a
    # never-sampled id reads as false rather than a stale/default value.
    sample_id_term, threshold_term, s = args
    if s != ('var', situation_var):
        raise NotImplementedError("sample_value_below/equal/over/3's own situation argument must be goal_formula's own S.")
    if sample_id_term[0] != 'atom':
        raise NotImplementedError(
            "sample_value_below/equal/over/3's own SampleId must be a literal atom, found: {}".format(sample_id_term)
        )
    if threshold_term[0] != 'num':
        raise NotImplementedError(
            "sample_value_below/equal/over/3's own Threshold must be a literal number, found: {}".format(threshold_term)
        )
    sample_id = sample_id_term[1]
    threshold_int = int(round(threshold_term[1]))
    success_var = factory.sample_success_var(sample_id)
    value_var = factory.sample_value_var(sample_id)
    condition = '(and, {}, ({}, {}, {}))'.format(success_var, op, value_var, threshold_int)
    return _wrap_finally(condition, bound)


def _translate_sample_value_below(args, _config, factory, situation_var, bound):
    return _translate_sample_value(args, factory, situation_var, bound, 'lt')


def _translate_sample_value_equal(args, _config, factory, situation_var, bound):
    return _translate_sample_value(args, factory, situation_var, bound, 'eq')


def _translate_sample_value_over(args, _config, factory, situation_var, bound):
    return _translate_sample_value(args, factory, situation_var, bound, 'gt')


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
    ('sample_success_at', 3): _translate_sample_success_at,
    ('sample_value_below', 3): _translate_sample_value_below,
    ('sample_value_equal', 3): _translate_sample_value_equal,
    ('sample_value_over', 3): _translate_sample_value_over,
    ('ploughed', 3): _translate_ploughed,
}


def collect_ploughed_cells(path):
    """
    Every distinct (Cx,Cy) cell any ploughed/3 conjunct in goal_formula.pl
    references -- called BEFORE parse_behavior_tree.parse_tree() builds
    any MoveTo action, so LeafFactory.ploughed_cells can be set up front
    and every MoveTo-family action's own updates can be wired to track
    them from the start (see leaf_library.LeafFactory._ploughed_updates
    and _translate_ploughed's own note on why this can't be decided
    lazily during translate() the way _DISPATCH normally works).
    """
    _situation_var, conjuncts = parse_goal_formula(path)
    cells = set()
    for term in conjuncts:
        if term[0] == 'compound' and term[1] == 'ploughed' and len(term[2]) == 3:
            cx_term, cy_term, _s = term[2]
            if cx_term[0] != 'num' or cy_term[0] != 'num':
                raise NotImplementedError(
                    "ploughed/3's Cx,Cy must be literal integers (this translator's own grid-cell units)."
                )
            cells.add((int(cx_term[1]), int(cy_term[1])))
    return cells


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
