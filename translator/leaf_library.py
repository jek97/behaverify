"""
Builds BehaVerify Action/Check IR objects for the BT.cpp action/condition
vocabulary described in schema.yaml.

Scope, and why: this schema describes several planners (PlanAstar,
PlanVoronoi, FollowBoarder) whose real implementations do black-box search
over map geometry. Reproducing that inside nuXmv's model-checking language
is out of scope for this translator -- their entries raise NotImplementedError
with a clear message rather than silently mis-modeling a search algorithm.
PlanStraight, MoveTo, and every plain fluent-lookup Condition (AtGoal,
Battery*, ObstacleInBound/OnPath) ARE implemented.

KEY SIMPLIFICATION (approved): motion advances exactly one grid cell per
tick, and each step is clamped to a "king move" (both axes shift by at
most one cell). Consequently a single tick's own path never skips over an
obstacle between two non-adjacent cells, so ObstacleOnPath only needs to
check the cell moved FROM and the cell moved TO -- no runtime line-rasterization
(e.g. Bresenham) is needed for it. (geometry.bresenham_cells is kept for
anyone later choosing a larger per-tick step size, where it WOULD be
needed again.)

Per the same approved design: every MoveTo's `triggers=` XML attribute
and BT.cpp's automatic collision/battery-trigger derivation are IGNORED.
Only explicit sibling Condition nodes actually present in the tree affect
behaviour (matching BehaVerify's own reactive, no-memory composite for a
ReactiveSequence/ReactiveFallback -- see parse_behavior_tree.py). Battery
reaching 0 mid-move is still handled inside MoveTo itself, since running out
of energy is a physical property of the action, not a configurable trigger.
"""
from . import ir


def _fmt(value):
    """Render a literal value as .tree DSL code_statement text."""
    if isinstance(value, bool):
        return 'True' if value else 'False'
    if isinstance(value, (int, float)):
        return str(int(value)) if float(value).is_integer() else str(value)
    return str(value)


class LeafFactory:
    def __init__(self, config, grid):
        self.config = config
        self.grid = grid
        self.checks = {}  # name -> ir.Check
        self.actions = {}  # name -> ir.Action
        self.constants = {}  # name -> value
        self._shared_vars_added = False
        self._obstacle_var_added = False
        self.move_to_aliases = []  # filled in by parse_behavior_tree.collect_move_to_aliases
        self.extra_variables = []  # e.g. obstacle_clearance, appended when first needed

        # bounds as constants, reused by every generated leaf
        self.constants['MIN_X'] = grid.min_x
        self.constants['MAX_X'] = grid.max_x
        self.constants['MIN_Y'] = grid.min_y
        self.constants['MAX_Y'] = grid.max_y
        self.constants['GRID_HEIGHT'] = grid.height
        self.constants['MOVING_DRAIN'] = int(round(config.moving_drain_rate))

    # ------------------------------------------------------------------
    # shared state (x, y, battery, ...) -- added once, referenced by name
    # ------------------------------------------------------------------
    def shared_variables(self):
        if self._shared_vars_added:
            return []
        self._shared_vars_added = True
        sx, sy = self.config.start_cell
        variables = [
            ir.Variable('x', 'bl', 'VAR', '[MIN_X, MAX_X]', initial=_fmt(sx)),
            ir.Variable('y', 'bl', 'VAR', '[MIN_Y, MAX_Y]', initial=_fmt(sy)),
            ir.Variable('prev_x', 'bl', 'VAR', '[MIN_X, MAX_X]', initial=_fmt(sx)),
            ir.Variable('prev_y', 'bl', 'VAR', '[MIN_Y, MAX_Y]', initial=_fmt(sy)),
            ir.Variable('target_x', 'bl', 'VAR', '[MIN_X, MAX_X]', initial=_fmt(sx)),
            ir.Variable('target_y', 'bl', 'VAR', '[MIN_Y, MAX_Y]', initial=_fmt(sy)),
            ir.Variable('battery', 'bl', 'VAR', '[0, 100]', initial=_fmt(self.config.battery_start)),
            # noise: nondeterministic environment choice, replacing the source
            # system's discretized-Gaussian probability draws (see noise.py).
            ir.Variable('noise_x', 'env', 'VAR', '{' + ', '.join(str(int(v)) for v in self.config.position_noise_support) + '}', initial=_fmt(0)),
            ir.Variable('noise_y', 'env', 'VAR', '{' + ', '.join(str(int(v)) for v in self.config.tangential_noise_support) + '}', initial=_fmt(0)),
            ir.Variable('battery_noise', 'env', 'VAR', '{' + ', '.join(str(int(v)) for v in self.config.battery_noise_support) + '}', initial=_fmt(0)),
        ]
        return variables

    def obstacle_variable(self):
        """Lazily builds the static obstacle-clearance lookup array (only if a
        Condition actually needs it -- most problems, like problem4, don't).
        Appends the resulting Variable to self.extra_variables exactly once."""
        if self._obstacle_var_added:
            return
        self._obstacle_var_added = True
        grid = self.grid
        # NOTE: these index_of{...} entries define the static table, so the
        # index must be a plain literal cell number here -- the symbolic
        # formula (referencing runtime x/y) belongs only on the READ side,
        # in _clearance_at() below.
        assigns = [
            (str(grid.flat_index(x, y)), str(clearance))
            for (x, y), clearance in sorted(grid.clearance_by_cell.items())
        ]
        var = ir.Variable(
            'obstacle_clearance', 'bl', 'DEFINE', 'INT',
            is_array=True,
            array_size=str(grid.width * grid.height),
            array_default=str(grid.cap_cells),
            array_assigns=assigns,
            static=True,
        )
        self.extra_variables.append(var)

    def _clearance_at(self, x_name, y_name):
        return '(index, obstacle_clearance, (add, (mult, (sub, {x}, MIN_X), GRID_HEIGHT), (sub, {y}, MIN_Y)))'.format(x=x_name, y=y_name)

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------
    def make_move_to(self):
        name = 'MoveTo'
        if name in self.actions:
            return name
        step_x = '(max, -1, (min, 1, (add, (if, (gt, target_x, x), 1, (if, (lt, target_x, x), -1, 0)), noise_x)))'
        step_y = '(max, -1, (min, 1, (add, (if, (gt, target_y, y), 1, (if, (lt, target_y, y), -1, 0)), noise_y)))'
        next_x = '(max, MIN_X, (min, MAX_X, (add, x, {step})))'.format(step=step_x)
        next_y = '(max, MIN_Y, (min, MAX_Y, (add, y, {step})))'.format(step=step_y)
        next_battery = '(max, 0, (sub, battery, (add, MOVING_DRAIN, battery_noise)))'
        action = ir.Action(
            name=name,
            read_variables=['x', 'y', 'target_x', 'target_y', 'battery', 'noise_x', 'noise_y', 'battery_noise'],
            write_variables=['x', 'y', 'prev_x', 'prev_y', 'battery'],
            updates=[
                ('var', 'prev_x', 'x'),
                ('var', 'prev_y', 'y'),
                # x/y/battery all read env-scope noise variables, which
                # check_grammar.py only allows inside a read_environment block
                # (a bare variable_statement may only read blackboard/local vars).
                ('read_env', 'apply_noise', 'True', [
                    ('x', next_x),
                    ('y', next_y),
                    ('battery', next_battery),
                ]),
            ],
            return_cases=[
                ('(and, (eq, x, target_x), (eq, y, target_y))', 'success'),
                ('(lte, battery, 0)', 'failure'),
                (None, 'running'),
            ],
        )
        self.actions[name] = action
        return name

    def make_plan_straight(self, goal_x_m, goal_y_m):
        gx, gy = self.config.to_cell(goal_x_m), self.config.to_cell(goal_y_m)
        name = 'PlanStraight_{}_{}'.format(gx, gy).replace('-', 'm')
        if name in self.actions:
            return name
        action = ir.Action(
            name=name,
            read_variables=[],
            write_variables=['target_x', 'target_y'],
            updates=[('var', 'target_x', str(gx)), ('var', 'target_y', str(gy))],
            return_cases=[(None, 'success')],
        )
        self.actions[name] = action
        return name

    # ------------------------------------------------------------------
    # Conditions
    # ------------------------------------------------------------------
    def make_battery_over(self, threshold):
        return self._battery_check('BatteryOver', 'gt', threshold)

    def make_battery_below(self, threshold):
        return self._battery_check('BatteryBelow', 'lt', threshold)

    def make_battery_equal(self, threshold):
        return self._battery_check('BatteryEqual', 'eq', threshold)

    def _battery_check(self, prefix, op, threshold):
        threshold_int = int(round(threshold))
        name = '{}_{}'.format(prefix, threshold_int)
        if name in self.checks:
            return name
        self.checks[name] = ir.Check(name, ['battery'], '({}, battery, {})'.format(op, threshold_int))
        return name

    def make_at_goal(self, goal_x_m, goal_y_m, tolerance_m):
        gx, gy = self.config.to_cell(goal_x_m), self.config.to_cell(goal_y_m)
        tol = max(1, self.config.to_cells_nearest(tolerance_m))
        name = 'AtGoal_{}_{}_{}'.format(gx, gy, tol).replace('-', 'm')
        if name in self.checks:
            return name
        condition = '(and, (lte, (abs, (sub, x, {gx})), {tol}), (lte, (abs, (sub, y, {gy})), {tol}))'.format(gx=gx, gy=gy, tol=tol)
        self.checks[name] = ir.Check(name, ['x', 'y'], condition)
        return name

    def make_obstacle_in_bound(self, threshold_m):
        return self._obstacle_check('ObstacleInBound', threshold_m, ('x', 'y'))

    def make_obstacle_on_path(self, threshold_m):
        # both endpoints of this tick's 1-cell step -- see module docstring.
        return self._obstacle_check('ObstacleOnPath', threshold_m, ('x', 'y'), also=('prev_x', 'prev_y'))

    def _obstacle_check(self, prefix, threshold_m, primary, also=None):
        self.obstacle_variable()  # ensure it exists (caller adds it to IR once)
        threshold_cells = self.config.to_cells_ceil(threshold_m)
        name = '{}_{}'.format(prefix, threshold_cells)
        if name in self.checks:
            return name
        clauses = ['(lt, {}, {})'.format(self._clearance_at(*primary), threshold_cells)]
        read_vars = ['obstacle_clearance', primary[0], primary[1]]
        if also is not None:
            clauses.append('(lt, {}, {})'.format(self._clearance_at(*also), threshold_cells))
            read_vars += [also[0], also[1]]
        condition = clauses[0] if len(clauses) == 1 else '(or, {}, {})'.format(*clauses)
        self.checks[name] = ir.Check(name, read_vars, condition)
        return name

    # ------------------------------------------------------------------
    # Out of scope, on purpose (see module docstring)
    # ------------------------------------------------------------------
    def make_plan_astar(self, *_args, **_kwargs):
        raise NotImplementedError('PlanAstar: map-search planning is not modeled by this translator.')

    def make_plan_voronoi(self, *_args, **_kwargs):
        raise NotImplementedError('PlanVoronoi: map-search planning is not modeled by this translator.')

    def make_follow_boarder(self, *_args, **_kwargs):
        raise NotImplementedError('FollowBoarder: boundary-following planning is not modeled by this translator.')

    def make_line_of_sight_clear(self, *_args, **_kwargs):
        raise NotImplementedError('LineOfSightClear: occlusion geometry is not modeled by this translator.')

    def make_halted_with(self, *_args, **_kwargs):
        raise NotImplementedError(
            'HaltedWith: MoveTo does not currently record a per-halt Reason fluent; '
            'add one (an extra bl variable) before translating a tree that branches on it.'
        )
