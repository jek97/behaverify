"""
Builds BehaVerify Action/Check IR objects for the BT.cpp action/condition
vocabulary described in module/contracts/schema.yaml (the CURRENT schema,
in problog_project -- PlanAstar/PlanStraight/PlanVoronoi/FollowBoarder were
since unified into one PlanWith node with an `algorithm` port, and AtGoal
was split into DistanceBelow/DistanceEqual/DistanceOver with a `threshold`
port; see make_plan_with/_distance_check below).

Scope: PlanWith(algorithm=straight/astar/voronoi) ARE implemented, both
astar and voronoi via a precomputed per-cell navigation policy -- see
astar_policy.py's own header for why a full-grid policy, why it doesn't
just import planners.py, and (for voronoi) exactly what its
clearance-weighted flood is -- and isn't -- faithful to. PlanWith
(algorithm=follow_boarder) and LineOfSightClear still raise
NotImplementedError: follow_boarder has no fixed goal point to build a
policy FROM at all (see planners.py's own header on why that planner
takes no goal), and LineOfSightClear needs the same occlusion geometry.
MoveTo and every plain fluent-lookup Condition (DistanceBelow/Equal/Over,
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
    (x_name,y_name) and (gx,gy)'s distance and threshold_cells, using
    Chebyshev distance (max(|dx|,|dy|)) -- NOT true Euclidean distance.

    An earlier version compared squared distance ((dx*dx)+(dy*dy)) to
    threshold_cells**2 to avoid sqrt (not in BehaVerify's function_names
    list) while staying geometrically exact. That MULTIPLIES two
    variable-derived expressions together, which is a well-known blowup
    case for BDD-based symbolic model checkers like nuXmv (multiplying two
    range-bounded variables needs far more symbolic state than addition/
    comparison) -- confirmed in practice: nuXmv was OOM-killed (exit -9)
    generating this check for problem4, which plain generation-time
    grammar checking has no way to catch (the blowup only shows up once
    nuXmv actually tries to build/traverse the encoding). Chebyshev
    distance only needs sub/abs/max/lte -- all linear, cheap to encode --
    at the cost of being a squarish rather than circular "closeness"
    region, which is an acceptable approximation at this grid's own
    resolution. Shared between LeafFactory's own distance checks and
    goal_formula.py's visited/3 translation.
    """
    dx = '(abs, (sub, {}, {}))'.format(x_name, gx)
    dy = '(abs, (sub, {}, {}))'.format(y_name, gy)
    chebyshev = '(max, {dx}, {dy})'.format(dx=dx, dy=dy)
    return '({}, {}, {})'.format(op, chebyshev, threshold_cells)


def _fmt(value):
    """Render a literal value as .tree DSL code_statement text."""
    if isinstance(value, bool):
        return 'True' if value else 'False'
    if isinstance(value, (int, float)):
        return str(int(value)) if float(value).is_integer() else str(value)
    return str(value)


class LeafFactory:
    def __init__(self, config, grid, drift_period=None):
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
        # drift_period lets a caller override the physically-derived default
        # (config.drift_resample_period_ticks) -- useful when a problem's own
        # legs are much shorter than that period, since nuXmv still has to
        # encode the full [0, DRIFT_RESAMPLE_PERIOD] state range even though
        # the resample branch is never reached (see translator/main.py's
        # --drift_period flag).
        self.constants['DRIFT_RESAMPLE_PERIOD'] = (
            drift_period if drift_period is not None else config.drift_resample_period_ticks
        )

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
            # Position (x, y) is deterministic -- MoveTo steps straight along
            # its heading, no noise term at all. This was tried the other
            # way (noise_x/noise_y as a per-tick nondeterministic env choice,
            # then a periodically-resampled lateral "drift" nudge on top of
            # that) and both blew up nuXmv's BDD-based LTL model checking,
            # even after confirming it wasn't the drift's specific
            # cross-axis coupling (a same-axis diagnostic variant blew up
            # too) -- so the randomness is confined to battery instead,
            # which only ever needs a single linear add, not a clamped
            # king-move step shared between two coupled state variables.
            #
            # battery_drift/ticks_since_resample: the same periodically-
            # resampled mechanism originally built for position (see
            # _battery_drift_updates below), retargeted at battery drain.
            # `bl` (persistent, held between ticks) rather than a per-tick
            # `env` choice -- the point is that it does NOT change every tick.
            ir.Variable('battery_drift', 'bl', 'VAR', '{' + ', '.join(str(int(v)) for v in self.config.battery_noise_support) + '}', initial=_fmt(0)),
            ir.Variable('ticks_since_resample', 'bl', 'VAR', '[0, DRIFT_RESAMPLE_PERIOD]', initial=_fmt(0)),
        ]
        return variables

    def _battery_drift_updates(self):
        """
        Shared ('case_var', ...) update entries for battery_drift/
        ticks_since_resample, appended to every MoveTo-family action's own
        updates. Position used to carry this same periodic-resample
        mechanism (see shared_variables' note on why it was moved to
        battery instead); the mission-wide persistent pacing (not reset
        per-leg) is unchanged.
        """
        resample_due = '(gte, ticks_since_resample, DRIFT_RESAMPLE_PERIOD)'
        drift_values = [str(int(v)) for v in self.config.battery_noise_support]
        return [
            ('case_var', 'battery_drift', [
                (resample_due, drift_values),
                (None, ['battery_drift']),
            ]),
            ('case_var', 'ticks_since_resample', [
                (resample_due, ['0']),
                (None, ['(add, ticks_since_resample, 1)']),
            ]),
        ]

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
        # Position is deterministic: step exactly one cell toward target_x/
        # target_y each tick, no noise term (see shared_variables' note on
        # why -- randomness lives only in battery_drift now).
        heading_x = '(if, (gt, target_x, x), 1, (if, (lt, target_x, x), -1, 0))'
        heading_y = '(if, (gt, target_y, y), 1, (if, (lt, target_y, y), -1, 0))'
        next_x = '(max, MIN_X, (min, MAX_X, (add, x, {h})))'.format(h=heading_x)
        next_y = '(max, MIN_Y, (min, MAX_Y, (add, y, {h})))'.format(h=heading_y)
        # Only pay the full moving_drain_rate (+ its drift) when the position
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
        next_battery = '(max, 0, (sub, battery, (if, {moving}, (add, MOVING_DRAIN, battery_drift), IDLE_DRAIN)))'.format(moving=is_moving)
        action = ir.Action(
            name=name,
            read_variables=['x', 'y', 'target_x', 'target_y', 'battery', 'battery_drift', 'ticks_since_resample'],
            write_variables=['x', 'y', 'prev_x', 'prev_y', 'battery', 'battery_drift', 'ticks_since_resample'],
            updates=[
                ('var', 'prev_x', 'x'),
                ('var', 'prev_y', 'y'),
                # resample (or hold) the drift BEFORE computing this tick's
                # battery update, so a fresh resample takes effect the same
                # tick -- both read the pre-tick ticks_since_resample
                # (neither is staged yet at this point in the update block).
            ] + self._battery_drift_updates() + [
                ('var', 'x', next_x),
                ('var', 'y', next_y),
                ('var', 'battery', next_battery),
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
            return self.make_plan_voronoi(goal_x_m, goal_y_m)
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
        """Precomputed shortest-path (uniform-cost) navigation policy --
        see _make_policy_based_plan's own docstring for the shared mechanism."""
        return self._make_policy_based_plan('Astar', goal_x_m, goal_y_m, clearance_weight=0.0)

    def make_plan_voronoi(self, goal_x_m, goal_y_m):
        """
        Precomputed clearance-WEIGHTED navigation policy -- same mechanism
        as make_plan_astar, but the underlying flood (astar_policy.compute_policy
        with clearance_weight>0) is biased away from tight passages, toward
        high-clearance corridors, as a discrete stand-in for "route along
        the free-space medial axis". See astar_policy.py's own "VORONOI,
        HONESTLY" note for exactly what this is (and isn't) faithful to.
        """
        return self._make_policy_based_plan('Voronoi', goal_x_m, goal_y_m, clearance_weight=astar_policy.VORONOI_CLEARANCE_WEIGHT)

    def _make_policy_based_plan(self, label, goal_x_m, goal_y_m, clearance_weight):
        """
        Precomputes a per-cell navigation policy toward (goal_x_m,goal_y_m)
        (see astar_policy.py), bakes it into two static DEFINE arrays
        (dx/dy per cell, same array-lookup pattern as obstacle_clearance),
        and builds a DEDICATED MoveTo_<label>_<goal> action that reads its
        own step from that table instead of the generic MoveTo's
        sign-toward-target heuristic. A cell the policy doesn't cover
        (goal unreachable from there -- blocked or disconnected) defaults
        to (0,0): MoveTo_<label> simply won't make progress from such a
        cell (still subject to noise, so not necessarily perfectly frozen),
        which is an honest degeneration given there's no PlanWith-level
        Status/Reason=no_path signal modeled here to react to instead.
        """
        gx, gy = self.config.to_cell(goal_x_m), self.config.to_cell(goal_y_m)
        plan_name = 'Plan{}_{}_{}'.format(label, gx, gy).replace('-', 'm')
        moveto_name = 'MoveTo_{}_{}_{}'.format(label, gx, gy).replace('-', 'm')
        if plan_name in self.actions:
            return plan_name, moveto_name

        obstacles_m = getattr(self.grid, 'obstacles_m', None)
        if obstacles_m is None:
            raise RuntimeError('LeafFactory.grid must carry .obstacles_m for {} support (set by translator/main.py).'.format(label))
        bounds = (self.grid.min_x, self.grid.max_x, self.grid.min_y, self.grid.max_y)
        policy = astar_policy.compute_policy(obstacles_m, self.config.disc_step_position, bounds, (gx, gy), clearance_weight=clearance_weight)

        dx_var = '{}_dx_{}_{}'.format(label.lower(), gx, gy).replace('-', 'm')
        dy_var = '{}_dy_{}_{}'.format(label.lower(), gx, gy).replace('-', 'm')
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
        # local heading = the policy table's own recommended step at the
        # CURRENT cell -- unlike a straight leg's single leg-wide heading,
        # this naturally follows a bending astar/voronoi path, since it's
        # re-read from the table fresh every tick. Deterministic, same as
        # make_move_to -- no noise term (see shared_variables' note).
        next_x = '(max, MIN_X, (min, MAX_X, (add, x, {dx})))'.format(dx=dx_lookup)
        next_y = '(max, MIN_Y, (min, MAX_Y, (add, y, {dy})))'.format(dy=dy_lookup)
        is_moving = '(or, (neq, x, prev_x), (neq, y, prev_y))'
        next_battery = '(max, 0, (sub, battery, (if, {moving}, (add, MOVING_DRAIN, battery_drift), IDLE_DRAIN)))'.format(moving=is_moving)
        moveto_action = ir.Action(
            name=moveto_name,
            read_variables=['x', 'y', 'target_x', 'target_y', 'battery', 'battery_drift', 'ticks_since_resample', dx_var, dy_var],
            write_variables=['x', 'y', 'prev_x', 'prev_y', 'battery', 'battery_drift', 'ticks_since_resample'],
            updates=[
                ('var', 'prev_x', 'x'),
                ('var', 'prev_y', 'y'),
            ] + self._battery_drift_updates() + [
                ('var', 'x', next_x),
                ('var', 'y', next_y),
                ('var', 'battery', next_battery),
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
