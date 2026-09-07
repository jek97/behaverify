"""
Builds BehaVerify Action/Check IR objects for the BT.cpp action/condition
vocabulary described in module/contracts/schema.yaml (the CURRENT schema,
in problog_project -- PlanAstar/PlanStraight/PlanVoronoi/FollowBoarder were
since unified into one PlanWith node with an `algorithm` port, and AtGoal
was split into DistanceBelow/DistanceEqual/DistanceOver with a `threshold`
port; see make_plan_with/_distance_check below).

Scope: PlanWith(algorithm=straight) and PlanWith(algorithm=astar) ARE
implemented (astar via a precomputed per-cell navigation policy -- see
astar_policy.py's own header for why a full-grid policy, and why it
doesn't just import planners.py). PlanWith(algorithm=voronoi/follow_boarder)
and LineOfSightClear still raise NotImplementedError: voronoi would need
the same "precomputed policy" treatment astar just got (not yet built),
and follow_boarder has no fixed goal point to build a policy FROM at all
(see planners.py's own header on why that planner takes no goal). MoveTo
and every plain fluent-lookup Condition (DistanceBelow/Equal/Over,
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
and bt_to_prolog.py's automatic collision/battery-trigger/guard_break
derivation are IGNORED. MoveTo is modeled purely as "advance one cell
along whichever trajectory the preceding PlanWith produced, with noise
applied" -- the only thing that can cut it short besides arrival/battery
is a reactive Condition sibling, which BehaVerify's own reactive,
no-memory composite already handles natively (see parse_behavior_tree.py),
with no derived-trigger bookkeeping needed. Battery reaching 0 mid-move is
still handled inside MoveTo itself, since running out of energy is a
physical property of the action, not a configurable trigger.
"""
from . import astar_policy, ir


def squared_distance_condition(x_name, y_name, gx, gy, threshold_cells, op):
    """
    A BehaVerify code_statement testing `op` (one of 'lt'/'eq'/'gt') between
    the true Euclidean distance from (x_name,y_name) to (gx,gy) and
    threshold_cells, WITHOUT using sqrt (not in BehaVerify's function_names
    list) -- compares squared distance to squared threshold instead, valid
    since both sides are non-negative and squaring is monotonic there.
    Shared between LeafFactory's own distance checks and goal_formula.py's
    visited/3 translation, so both use the same faithful (non-Chebyshev)
    distance test.
    """
    dx = '(sub, {}, {})'.format(x_name, gx)
    dy = '(sub, {}, {})'.format(y_name, gy)
    dist_sq = '(add, (mult, {dx}, {dx}), (mult, {dy}, {dy}))'.format(dx=dx, dy=dy)
    return '({}, {}, {})'.format(op, dist_sq, threshold_cells * threshold_cells)


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
        # config.yaml's idle_drain_rate (0.05 in problem4) rounds to 0 at this
        # model's integer-percent precision -- kept symbolic rather than
        # hardcoded 0 so a problem with a coarser/larger idle rate still
        # picks up the right value.
        self.constants['IDLE_DRAIN'] = int(round(config.idle_drain_rate))

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
        # Only pay the full moving_drain_rate (+ its noise) when the position
        # actually changes this tick; an already-arrived MoveTo that keeps
        # getting reticked (the tree runs forever) pays config.yaml's own
        # idle_drain_rate instead -- otherwise battery drains to 0 purely
        # from idling at the target, which nuXmv will (correctly) report as
        # a real failure even though the robot never actually moved.
        #
        # NOTE: this must compare against prev_x/prev_y (captured BEFORE x/y
        # are reassigned below), not by recomputing the next_x/next_y step
        # formula again here -- by the time this statement runs, a bare `x`/`y`
        # reference already resolves to the value x's/y's OWN variable_statement
        # just staged earlier in this same update block, not the pre-tick one,
        # so recomputing the step from `x` here would silently compare the new
        # position against itself.
        is_moving = '(or, (neq, x, prev_x), (neq, y, prev_y))'
        next_battery = '(max, 0, (sub, battery, (if, {moving}, (add, MOVING_DRAIN, battery_noise), IDLE_DRAIN)))'.format(moving=is_moving)
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

    def make_plan_with(self, algorithm, goal_x_m=None, goal_y_m=None, obstacle_id=None, offset=None):
        """
        Dispatches the CURRENT schema's single PlanWith node by its own
        `algorithm` port. Returns (plan_action_name, moveto_action_name) --
        the moveto_action_name is which MoveTo variant the NEXT MoveTo leaf
        in the tree must reference (parse_behavior_tree.py threads this
        through), since astar's own navigation table is baked into a
        DEDICATED MoveTo_Astar_<goal> action, not the generic MoveTo.
        """
        if algorithm == 'straight':
            return self.make_plan_straight(goal_x_m, goal_y_m), self.make_move_to()
        if algorithm == 'astar':
            return self.make_plan_astar(goal_x_m, goal_y_m)
        if algorithm == 'voronoi':
            raise NotImplementedError(
                'PlanWith(algorithm=voronoi): not modeled by this translator yet. '
                'astar is implemented as a precomputed per-cell navigation policy '
                '(see astar_policy.py); voronoi would need the same treatment '
                '(a per-cell policy following the free-space Voronoi roadmap) -- '
                'not yet built.'
            )
        if algorithm == 'follow_boarder':
            raise NotImplementedError(
                'PlanWith(algorithm=follow_boarder): boundary-following planning is not '
                'modeled by this translator -- it has no fixed goal point to build a '
                'per-cell policy from (see planners.py\'s own header for why).'
            )
        raise NotImplementedError('PlanWith(algorithm={!r}): unrecognized algorithm.'.format(algorithm))

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

    def make_plan_astar(self, goal_x_m, goal_y_m):
        """
        Precomputes a per-cell navigation policy toward (goal_x_m,goal_y_m)
        (see astar_policy.py), bakes it into two static DEFINE arrays
        (dx/dy per cell, same array-lookup pattern as obstacle_clearance),
        and builds a DEDICATED MoveTo_Astar_<goal> action that reads its
        own step from that table instead of the generic MoveTo's
        sign-toward-target heuristic. A cell the policy doesn't cover
        (goal unreachable from there -- blocked or disconnected) defaults
        to (0,0): MoveTo_Astar simply won't make progress from such a
        cell (still subject to noise, so not necessarily perfectly frozen),
        which is an honest degeneration given there's no PlanWith-level
        Status/Reason=no_path signal modeled here to react to instead.
        """
        gx, gy = self.config.to_cell(goal_x_m), self.config.to_cell(goal_y_m)
        plan_name = 'PlanAstar_{}_{}'.format(gx, gy).replace('-', 'm')
        moveto_name = 'MoveTo_Astar_{}_{}'.format(gx, gy).replace('-', 'm')
        if plan_name in self.actions:
            return plan_name, moveto_name

        obstacles_m = getattr(self.grid, 'obstacles_m', None)
        if obstacles_m is None:
            raise RuntimeError('LeafFactory.grid must carry .obstacles_m for astar support (set by translator/main.py).')
        bounds = (self.grid.min_x, self.grid.max_x, self.grid.min_y, self.grid.max_y)
        policy = astar_policy.compute_policy(obstacles_m, self.config.disc_step_position, bounds, (gx, gy))

        dx_var = 'astar_dx_{}_{}'.format(gx, gy).replace('-', 'm')
        dy_var = 'astar_dy_{}_{}'.format(gx, gy).replace('-', 'm')
        dx_assigns = [(str(self.grid.flat_index(cx, cy)), str(dx)) for (cx, cy), (dx, _dy) in sorted(policy.items())]
        dy_assigns = [(str(self.grid.flat_index(cx, cy)), str(dy)) for (cx, cy), (_dx, dy) in sorted(policy.items())]
        array_size = str(self.grid.width * self.grid.height)
        self.extra_variables.append(ir.Variable(dx_var, 'bl', 'DEFINE', 'INT', is_array=True, array_size=array_size, array_default='0', array_assigns=dx_assigns, static=True))
        self.extra_variables.append(ir.Variable(dy_var, 'bl', 'DEFINE', 'INT', is_array=True, array_size=array_size, array_default='0', array_assigns=dy_assigns, static=True))

        plan_action = ir.Action(
            name=plan_name,
            read_variables=[],
            write_variables=['target_x', 'target_y'],
            updates=[('var', 'target_x', str(gx)), ('var', 'target_y', str(gy))],
            return_cases=[(None, 'success')],
        )
        self.actions[plan_name] = plan_action

        flat_index_expr = '(add, (mult, (sub, x, MIN_X), GRID_HEIGHT), (sub, y, MIN_Y))'
        dx_lookup = '(index, {}, {})'.format(dx_var, flat_index_expr)
        dy_lookup = '(index, {}, {})'.format(dy_var, flat_index_expr)
        next_x = '(max, MIN_X, (min, MAX_X, (add, x, (max, -1, (min, 1, (add, {dx}, noise_x))))))'.format(dx=dx_lookup)
        next_y = '(max, MIN_Y, (min, MAX_Y, (add, y, (max, -1, (min, 1, (add, {dy}, noise_y))))))'.format(dy=dy_lookup)
        is_moving = '(or, (neq, x, prev_x), (neq, y, prev_y))'
        next_battery = '(max, 0, (sub, battery, (if, {moving}, (add, MOVING_DRAIN, battery_noise), IDLE_DRAIN)))'.format(moving=is_moving)
        moveto_action = ir.Action(
            name=moveto_name,
            read_variables=['x', 'y', 'target_x', 'target_y', 'battery', 'noise_x', 'noise_y', 'battery_noise', dx_var, dy_var],
            write_variables=['x', 'y', 'prev_x', 'prev_y', 'battery'],
            updates=[
                ('var', 'prev_x', 'x'),
                ('var', 'prev_y', 'y'),
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
        self.actions[moveto_name] = moveto_action
        return plan_name, moveto_name

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

    def make_distance_below(self, goal_x_m, goal_y_m, threshold_m):
        return self._distance_check('DistanceBelow', 'lt', goal_x_m, goal_y_m, threshold_m)

    def make_distance_equal(self, goal_x_m, goal_y_m, threshold_m):
        return self._distance_check('DistanceEqual', 'eq', goal_x_m, goal_y_m, threshold_m)

    def make_distance_over(self, goal_x_m, goal_y_m, threshold_m):
        return self._distance_check('DistanceOver', 'gt', goal_x_m, goal_y_m, threshold_m)

    def _distance_check(self, prefix, op, goal_x_m, goal_y_m, threshold_m):
        gx, gy = self.config.to_cell(goal_x_m), self.config.to_cell(goal_y_m)
        # ceil, not nearest/floor: a small threshold (e.g. 0.3m, smaller than
        # one grid cell) must never round down to 0 -- for a 'lt' (DistanceBelow)
        # check that would make even exact arrival (distance 0) fail, since
        # 0 is not < 0. Ceiling guarantees a real arrival always satisfies it.
        tol = max(1, self.config.to_cells_ceil(threshold_m))
        name = '{}_{}_{}_{}'.format(prefix, gx, gy, tol).replace('-', 'm')
        if name in self.checks:
            return name
        self.checks[name] = ir.Check(name, ['x', 'y'], squared_distance_condition('x', 'y', gx, gy, tol, op))
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
    # Out of scope, on purpose (see module docstring and make_plan_with)
    # ------------------------------------------------------------------
    def make_line_of_sight_clear(self, *_args, **_kwargs):
        raise NotImplementedError('LineOfSightClear: occlusion geometry is not modeled by this translator.')

    def make_halted_with(self, *_args, **_kwargs):
        raise NotImplementedError(
            'HaltedWith: MoveTo does not currently record a per-halt Reason fluent; '
            'add one (an extra bl variable) before translating a tree that branches on it.'
        )
