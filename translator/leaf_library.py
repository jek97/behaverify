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

TakeSample/InstallTool/UninstallTool/DeployTool/RetractTool ARE implemented
(see make_take_sample/make_install_tool/make_uninstall_tool/
make_deploy_tool/make_retract_tool below) -- InstallTool/UninstallTool/
DeployTool/RetractTool's own `triggers=` port is ignored, same established
precedent as MoveTo's. TakeSample now requires an `id=` port (e.g.
"soil1"); on success it also draws a value 0..10 (config.yaml's sample.
value.mean/sigma, a discretized Normal -- see make_take_sample), readable
back via SampleValueBelow/Equal/Over(id,threshold) (make_sample_value_
below/equal/over) or goal_formula.py's sample_value_below/equal/over/3.
DeployTool/RetractTool (make_deploy_tool/make_retract_tool) are currently
restricted to tool kind=plow, mirroring basic_action_theory.pl's own
poss(start_deploy_tool(...)) kind check -- but caught at TRANSLATION time
here (this translator parses config.yaml, unlike bt_to_prolog.py, so it
doesn't need to defer the check to a runtime Prolog failure).
The one thing NOT modeled is robot VELOCITY changing while a tool is
equipped (tool.equipped.<tool>.speed in config.yaml) -- MoveTo already
ignores config.yaml's own BASE motion.speed entirely (ticks are cells, not
time -- see parse_config.py's module docstring), so respecting a tool-
RELATIVE speed change while still ignoring the base value would be
inconsistent; only the battery-drain-rate side of "battery drain and
velocity change while equipped" is implemented (_moving_drain_for_hitch).

MULTI-INSTANCE TOOLS: a BT's own <InstallTool tool="..."> / <UninstallTool
tool="..."> names a specific tool INSTANCE id (e.g. "cart1"), not a kind --
config.yaml's tool.instances maps each id to a kind and a starting
position (ProblemConfig.tool_instance_kind/tool_instance_start_m).
Installing requires hitch(free) AND proximity to THIS instance's own
CURRENT position (tool_position -- see _ensure_tool_position_vars/
_make_tool_action); uninstalling requires hitch_id(THIS id) specifically
(hitch_id, not hitch, is the fluent that can tell cart1 from cart2 apart).
A successful uninstall drops the tool wherever the robot currently is.

RetryUntilSuccessful(num_attempts="n")/Repeat(num_cycles="n") ARE
implemented in parse_behavior_tree.py, by literal XML unrolling into n
copies of the one child wired via a plain Fallback/Sequence -- the same
sound 1:1 translation the source ProbLog system's own bt_to_prolog.py
uses (see parse_behavior_tree.py's _UNROLL_TAGS for why this isn't an
approximation, and why BehaVerify's own built-in `repeat` decorator means
something different and can't be reused for this).
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
        self.enum_atoms = set()  # quoted atom literals, e.g. "'free'" -- collected into ProblemIR.enumerations
        # Set by translator/main.py, BEFORE shared_variables()/parse_tree(),
        # from parse_behavior_tree.uses_tool_actions -- True iff this
        # problem's behavior_tree.xml uses InstallTool/UninstallTool
        # anywhere, which is also when MoveTo needs to be hitch-aware (see
        # shared_variables/_moving_drain_for_hitch below).
        self.tool_aware = False
        # Set by translator/main.py alongside tool_aware, from
        # parse_behavior_tree.collect_tool_instance_ids -- every distinct
        # tool INSTANCE id (e.g. "cart1") the tree's InstallTool/
        # UninstallTool nodes reference, needed upfront to declare the
        # shared `hitch_id` fluent's own domain (see shared_variables).
        self.tool_instance_ids = set()
        self._tool_position_vars_added = set()  # instance ids whose pos_x/pos_y vars already exist
        # Set by translator/main.py, BEFORE shared_variables()/parse_tree(),
        # from goal_formula.collect_ploughed_cells -- every (Cx,Cy) cell a
        # goal formula's own ploughed/3 conjunct references, needed
        # upfront so every MoveTo-family action can be wired (see
        # _ploughed_updates) to track them from the very first one built.
        self.ploughed_cells = set()

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
        if self.tool_aware:
            # hitch(free/cart/plow): KIND-level -- see basic_action_
            # theory.pl's own hitch/2. A plain persistent fluent (like
            # x/y/battery), not resampled -- only make_install_tool/
            # make_uninstall_tool's own coin-flip success ever changes
            # it. Only added when this problem's tree actually uses
            # InstallTool/UninstallTool, so a problem that doesn't (e.g.
            # problem4) gets byte-identical output to before this
            # feature existed.
            self.enum_atoms.update(["'free'", "'cart'", "'plow'"])
            variables.append(ir.Variable('hitch', 'bl', 'VAR', "{'free', 'cart', 'plow'}", initial="'free'"))
            # hitch_id(free/<every instance id this tree references>):
            # the INSTANCE-level sibling of hitch, needed to tell "cart1
            # attached" from "cart2 attached" apart -- specifically for
            # UninstallTool's own precondition (that SPECIFIC instance
            # must be the one attached, not just "something of its
            # kind") and for tool_position's own hitched-exclusion (see
            # _make_tool_action). Domain is fixed upfront from
            # self.tool_instance_ids (set by translator/main.py before
            # this call), same reason tool_aware itself is precomputed.
            id_atoms = ["'{}'".format(i) for i in sorted(self.tool_instance_ids)]
            self.enum_atoms.update(id_atoms)
            self.enum_atoms.add("'free'")
            variables.append(ir.Variable('hitch_id', 'bl', 'VAR', '{' + ', '.join(["'free'"] + id_atoms) + '}', initial="'free'"))
            # deployed: a boolean fluent mirroring hitch's own "only a
            # SUCCESSFUL halt flips it" discipline -- basic_action_
            # theory.pl's own deployed/1. Currently reachable for plow
            # only (make_deploy_tool/make_retract_tool reject any other
            # kind), but kept as a single GLOBAL boolean, not per-
            # instance -- only one tool can ever be hitched at a time, so
            # only one could ever be deployed at a time either. Always
            # declared whenever tool_aware (not gated on the tree
            # actually using DeployTool/RetractTool): UninstallTool's own
            # precondition now unconditionally reads it too (see
            # _make_tool_action).
            variables.append(ir.Variable('deployed', 'bl', 'VAR', 'BOOLEAN', initial='False'))
            # MoveTo's drain rate WHILE a tool is equipped (tool.equipped.
            # <tool>.moving_drain_rate), and its OWN further switch while
            # deployed(S) also holds (tool.equipped.<tool>.deployed_
            # moving_drain_rate) -- see _moving_drain_for_hitch.
            self.constants['MOVING_DRAIN_CART'] = int(round(self.config.tool_moving_drain_rate['cart']))
            self.constants['MOVING_DRAIN_PLOW'] = int(round(self.config.tool_moving_drain_rate['plow']))
            self.constants['MOVING_DRAIN_PLOW_DEPLOYED'] = int(round(self.config.tool_moving_drain_rate_deployed['plow']))
            # install_tool_range(Range) -- see basic_action_theory.pl's
            # own poss(start_install_tool(...)); ceil to never let a
            # real in-range attempt round down and wrongly fail (same
            # convention _distance_check's own DistanceBelow uses, since
            # this IS a distance_below check, just embedded in an
            # action's precondition instead of a standalone Check node).
            self.constants['INSTALL_RANGE'] = max(1, self.config.to_cells_ceil(self.config.install_range_m))
            # ploughed_<cx>_<cy>: one persistent boolean per (Cx,Cy) cell
            # a goal formula's own ploughed/3 conjunct references (see
            # goal_formula.collect_ploughed_cells/_ploughed_updates) --
            # empty (no vars added) for a problem whose goal formula
            # never mentions ploughed/3, same "only pay for what's used"
            # treatment as everything else in this block.
            for (cx, cy) in sorted(self.ploughed_cells):
                variables.append(ir.Variable(self.ploughed_var(cx, cy), 'bl', 'VAR', 'BOOLEAN', initial='False'))
        return variables

    def ploughed_var(self, cx, cy):
        return 'ploughed_{}_{}'.format(cx, cy).replace('-', 'm')

    def _ploughed_updates(self):
        """
        Shared ('case_var', ...) update entries -- one per (Cx,Cy) cell a
        goal formula's own ploughed/3 conjunct actually references --
        appended to every MoveTo-family action's own updates, AFTER its
        own x/y are already staged to their POST-step value. Deliberately
        NOT a grid-wide writable array: goal_formula.pl only ever asks
        about a small, FIXED set of specific cells (known at translation
        time, same as visited/3's own goal points), so a dedicated
        persistent boolean per referenced cell is both simpler and
        cheaper than a general array write conditioned on an arbitrary
        runtime index would be -- and avoids introducing that untested-
        for-performance pattern into MoveTo's own transition at all. Each
        check is a plain eq/and over the CURRENT (already-staged) x,y
        against a COMPILE-TIME CONSTANT cx,cy -- the same cost shape as
        DistanceBelow's own check, not an array-index lookup.
        """
        updates = []
        for (cx, cy) in sorted(self.ploughed_cells):
            var_name = self.ploughed_var(cx, cy)
            # ploughed now ALSO requires deployed(SPrev), not just
            # hitch(plow,SPrev) -- basic_action_theory.pl's own
            # "DeployTool/RetractTool: ploughing now requires deployed,
            # not just hitched" note: merely having the plow installed
            # and walking around should NOT mark any cells ploughed.
            condition = "(and, (eq, hitch, 'plow'), (and, deployed, (and, (eq, x, {}), (eq, y, {}))))".format(cx, cy)
            updates.append(('case_var', var_name, [
                (condition, ['True']),
                (None, [var_name]),
            ]))
        return updates

    def _moving_drain_for_hitch(self):
        """
        MOVING_DRAIN, but hitch-aware: reads the shared `hitch` fluent
        (only ever 3 values) via explicit if/eq case dispatch -- NOT
        `mult` (see squared_distance_condition's own note on why
        multiplying two variable-derived expressions is the confirmed
        nuXmv blowup pattern this translator avoids everywhere). Only
        called when self.tool_aware (see shared_variables) -- a problem
        that never uses InstallTool/UninstallTool never has `hitch` to
        read at all, so make_move_to/_make_policy_based_plan fall back to
        the plain MOVING_DRAIN constant in that case, unchanged from
        before this feature existed.

        Plow additionally switches to a THIRD rate while deployed(S) also
        holds (basic_action_theory.pl's own effective_tool_moving_drain_
        rate/3) -- currently only plow can ever deploy, so cart's own
        branch has no deployed-specific variant to check.
        """
        return (
            "(if, (eq, hitch, 'cart'), MOVING_DRAIN_CART, "
            "(if, (eq, hitch, 'plow'), (if, deployed, MOVING_DRAIN_PLOW_DEPLOYED, MOVING_DRAIN_PLOW), "
            "MOVING_DRAIN))"
        )

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
        moving_drain = self._moving_drain_for_hitch() if self.tool_aware else 'MOVING_DRAIN'
        next_battery = '(max, 0, (sub, battery, (if, {moving}, (add, {drain}, battery_noise), IDLE_DRAIN)))'.format(moving=is_moving, drain=moving_drain)
        read_variables = ['x', 'y', 'target_x', 'target_y', 'battery', 'noise_x', 'noise_y', 'battery_noise']
        write_variables = ['x', 'y', 'prev_x', 'prev_y', 'battery']
        if self.tool_aware:
            read_variables += ['hitch', 'deployed']
        ploughed_vars = [self.ploughed_var(cx, cy) for (cx, cy) in sorted(self.ploughed_cells)]
        read_variables += ploughed_vars
        write_variables += ploughed_vars
        action = ir.Action(
            name=name,
            read_variables=read_variables,
            write_variables=write_variables,
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
            ] + self._ploughed_updates(),
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
        next_x = '(max, MIN_X, (min, MAX_X, (add, x, (max, -1, (min, 1, (add, {dx}, noise_x))))))'.format(dx=dx_lookup)
        next_y = '(max, MIN_Y, (min, MAX_Y, (add, y, (max, -1, (min, 1, (add, {dy}, noise_y))))))'.format(dy=dy_lookup)
        is_moving = '(or, (neq, x, prev_x), (neq, y, prev_y))'
        moving_drain = self._moving_drain_for_hitch() if self.tool_aware else 'MOVING_DRAIN'
        next_battery = '(max, 0, (sub, battery, (if, {moving}, (add, {drain}, battery_noise), IDLE_DRAIN)))'.format(moving=is_moving, drain=moving_drain)
        read_variables = ['x', 'y', 'target_x', 'target_y', 'battery', 'noise_x', 'noise_y', 'battery_noise', dx_var, dy_var]
        write_variables = ['x', 'y', 'prev_x', 'prev_y', 'battery']
        if self.tool_aware:
            read_variables += ['hitch', 'deployed']
        ploughed_vars = [self.ploughed_var(cx, cy) for (cx, cy) in sorted(self.ploughed_cells)]
        read_variables += ploughed_vars
        write_variables += ploughed_vars
        moveto_action = ir.Action(
            name=moveto_name,
            read_variables=read_variables,
            write_variables=write_variables,
            updates=[
                ('var', 'prev_x', 'x'),
                ('var', 'prev_y', 'y'),
                ('read_env', 'apply_noise', 'True', [
                    ('x', next_x),
                    ('y', next_y),
                    ('battery', next_battery),
                ]),
            ] + self._ploughed_updates(),
            return_cases=[
                ('(and, (eq, x, target_x), (eq, y, target_y))', 'success'),
                ('(lte, battery, 0)', 'failure'),
                (None, 'running'),
            ],
        )
        self.actions[moveto_name] = moveto_action
        return plan_name, moveto_name

    @staticmethod
    def sample_success_var(sample_id):
        return 'sample_success_{}'.format(sample_id)

    @staticmethod
    def sample_value_var(sample_id):
        return 'sample_value_{}'.format(sample_id)

    def make_take_sample(self, sample_id):
        """
        take_sample(ActionCode) in basic_action_theory.pl: instantaneous,
        like PlanWith -- but UNLIKE every other action, its outcome is a
        genuine new probabilistic choice (config.yaml's own sample.
        success_probability), not a deterministic function of the current
        state, and -- ONLY on success -- a SECOND independent draw, a
        value 0..10 (config.yaml's sample.value.mean/sigma, a discretized
        Normal). Both are translated the same way every other
        probabilistic draw in this model is (noise_x/noise_y/
        battery_noise) -- as a full nondeterministic choice, not weighted
        by the actual probability/bell-curve shape, since nuXmv's LTL/CTL
        model checking asks "can this happen" (possibility), not "how
        likely is this" -- so neither config number plays any role here,
        only WHETHER an outcome is possible at all (every one of the 11
        values is, barring a literal 0.0-mass tail -- not specially
        handled, since exploring a branch that never actually gets drawn
        costs nothing extra).

        `sample_id` (schema.yaml's own required `id` port, e.g. "soil1")
        is the tree author's own name for this occurrence -- NOT the
        auto-generated ActionCode BT.cpp's own translator uses -- and is
        what a LATER SampleValueBelow/Equal/Over condition (or a goal
        formula's own sample_value_below/equal/over/3) references to read
        THIS specific sample's own value back out. Each distinct id gets
        its own dedicated action + variables (unlike the old, portless
        TakeSample, which was a single shared action) -- same "keyed by
        the parameters that make different occurrences actually
        different" idiom PlanStraight_<gx>_<gy> already uses.

        Position (x,y) is already global blackboard state and doesn't
        change during this instantaneous action, so "record the robot's
        position at the instant of sampling" (the reason port's own
        sample_success(X,Y,V,SampleId,ActionCode)) needs no bookkeeping
        of its own -- x,y already ARE that position for as long as
        sample_success_<id> stays True (see goal_formula.py's
        sample_success_at/3).
        """
        name = 'TakeSample_{}'.format(sample_id)
        if name in self.actions:
            return name
        success_var = self.sample_success_var(sample_id)
        value_var = self.sample_value_var(sample_id)
        action = ir.Action(
            name=name,
            read_variables=[success_var, value_var],
            write_variables=[success_var, value_var],
            updates=[
                ('case_var', success_var, [(None, ['True', 'False'])]),
                # value is drawn ONLY on success (reads success_var's OWN
                # just-staged new value, from the case_var immediately
                # above, per this module's own staging convention) --
                # held (not resampled) on a failed draw, matching the
                # theory's own "value only exists conditional on
                # success" shape; SampleValueBelow/Equal/Over below
                # additionally gates on success_var, so a stale held
                # value from an EARLIER success can never be misread
                # after a later failure.
                ('case_var', value_var, [
                    (success_var, [str(v) for v in range(11)]),
                    (None, [value_var]),
                ]),
            ],
            return_cases=[
                ('(eq, {}, True)'.format(success_var), 'success'),
                (None, 'failure'),
            ],
        )
        self.actions[name] = action
        self.extra_variables.append(ir.Variable(success_var, 'bl', 'VAR', 'BOOLEAN', initial='False'))
        self.extra_variables.append(ir.Variable(value_var, 'bl', 'VAR', '[0, 10]', initial='0'))
        return name

    def make_install_tool(self, tool_id):
        return self._make_tool_action('Install', tool_id)

    def make_uninstall_tool(self, tool_id):
        return self._make_tool_action('Uninstall', tool_id)

    def make_deploy_tool(self, tool_id):
        return self._make_tool_action('Deploy', tool_id)

    def make_retract_tool(self, tool_id):
        return self._make_tool_action('Retract', tool_id)

    def _ensure_tool_position_vars(self, tool_id):
        """
        Lazily builds this tool INSTANCE's own persistent (pos_x, pos_y)
        -- basic_action_theory.pl's own tool_position/4, seeded from
        config.yaml's tool.instances[].x/y and updated (see
        _make_tool_action) only on a SUCCESSFUL uninstall of THIS id, to
        wherever the robot actually was at that moment. Shared between an
        id's own InstallTool_<id> (reads it, for the proximity
        precondition) and UninstallTool_<id> (reads AND writes it) --
        added once regardless of which one is built first.
        """
        if tool_id in self._tool_position_vars_added:
            return
        self._tool_position_vars_added.add(tool_id)
        if tool_id not in self.config.tool_instance_start_m:
            raise NotImplementedError(
                'InstallTool/UninstallTool tool={!r}: no tool.instances entry in '
                "config.yaml for this id -- every instance id a tree's InstallTool/"
                'UninstallTool nodes reference needs its own {{id,kind,x,y}} entry '
                'under tool.instances.'.format(tool_id)
            )
        start_x_m, start_y_m = self.config.tool_instance_start_m[tool_id]
        sx, sy = self.config.to_cell(start_x_m), self.config.to_cell(start_y_m)
        self.extra_variables.append(ir.Variable(self._tool_pos_x_var(tool_id), 'bl', 'VAR', '[MIN_X, MAX_X]', initial=_fmt(sx)))
        self.extra_variables.append(ir.Variable(self._tool_pos_y_var(tool_id), 'bl', 'VAR', '[MIN_Y, MAX_Y]', initial=_fmt(sy)))

    @staticmethod
    def _tool_pos_x_var(tool_id):
        return '{}_pos_x'.format(tool_id)

    @staticmethod
    def _tool_pos_y_var(tool_id):
        return '{}_pos_y'.format(tool_id)

    # config lookup + precondition shape per action kind -- see
    # _make_tool_action's own docstring for what each precondition means.
    _TOOL_ACTION_KINDS = ('Install', 'Uninstall', 'Deploy', 'Retract')

    def _make_tool_action(self, action_kind, tool_id):
        """
        Shared InstallTool/UninstallTool/DeployTool/RetractTool builder --
        durative, FIXED-Duration actions (config.yaml's tool.<kind_lower>.
        duration_seconds.<tool_kind>, ROUNDED DIRECTLY to ticks -- see
        ProblemConfig.install_duration_ticks's own note on why: this
        translator has no other seconds<->tick conversion anywhere,
        MoveTo's own "one grid cell per tick" being an equally arbitrary,
        explicitly-approved choice rather than a physically-derived one).

        `tool_id` names a specific tool INSTANCE (e.g. "cart1"), not a
        kind -- config.yaml's tool.instances maps each id to a kind and a
        starting position (see _ensure_tool_position_vars/
        ProblemConfig.tool_instance_kind); an id with no such entry is a
        clear translation-time error, not a silent default (mirrors
        config_to_prolog.py's own validation, just done earlier -- this
        translator DOES parse config.yaml, unlike bt_to_prolog.py, so
        there's no reason to defer the check to a runtime Prolog failure
        the way that file has to). DeployTool/RetractTool are additionally
        restricted to kind=plow (basic_action_theory.pl's own poss(
        start_deploy_tool(...)) enforces this at the Prolog level, since
        THAT translator can't see config.yaml at all; this one can, so it
        catches the same mistake at translation time instead).

        Mechanism, mirroring basic_action_theory.pl's own install_tool_leg/
        uninstall_tool_leg/deploy_tool_leg/retract_tool_leg:
          - precondition not holding -> immediate failure, state
            untouched -- this action is structurally IMPOSSIBLE from a
            poss/2 point of view (not a probabilistic failure), and an
            immediate failure is the closest this return-status-only leaf
            shape can capture that (can only actually arise from
            re-entering an already-resolved node, e.g. inside
            RetryUntilSuccessful/Repeat). Preconditions:
              Install:    hitch(free) AND proximity to THIS instance's
                          own CURRENT tool_position (within INSTALL_
                          RANGE) -- reusing the same Chebyshev-distance
                          primitive DistanceBelow's own check uses, just
                          between two VARIABLE pairs (x,y vs this id's
                          own pos_x/pos_y) instead of a compile-time-
                          constant goal; still only sub/abs/max, no
                          `mult` of two variable-derived expressions, so
                          this doesn't reintroduce the confirmed nuXmv
                          blowup pattern (see squared_distance_
                          condition's own note).
              Uninstall:  hitch_id(THIS id) AND NOT deployed -- a
                          deployed tool must be retracted first.
              Deploy:     hitch_id(THIS id) AND NOT deployed.
              Retract:    hitch_id(THIS id) AND deployed.
            hitch_id (not hitch) is what tells "cart1 attached" from
            "cart2 attached" apart.
          - battery already at 0 -> failure (the one ALWAYS-on trigger;
            the `triggers=` port's own EXTRA battery-only halts are out of
            scope, same precedent as MoveTo's own `triggers=`).
          - Duration elapses with nothing halting early -> a genuine coin
            flip (config.yaml's own success_probability, default 0.9),
            translated to nondeterministic choice exactly like
            make_take_sample above -- on success:
              Install:    hitch (KIND) := this id's kind, hitch_id := this id.
              Uninstall:  hitch := free, hitch_id := free, tool_position
                          (this id's own pos_x/pos_y) updates to wherever
                          the robot currently is (dropped there).
              Deploy:     deployed := True.
              Retract:    deployed := False.
            on failure, none of these change, per the theory's own
            hitch/2/hitch_id/2/tool_position/4/deployed/1 clauses.
          - otherwise -> drain battery at THIS action's own rate
            (tool.<kind_lower>.drain_rate, default idle_drain_rate),
            increment the elapsed-ticks counter (self-resetting once
            resolved, same shape as the drift mechanism's own
            ticks_since_resample), return running.
        """
        if action_kind not in self._TOOL_ACTION_KINDS:
            raise ValueError('Unknown tool action kind: {!r}'.format(action_kind))
        name = '{}Tool_{}'.format(action_kind, tool_id)
        if name in self.actions:
            return name
        if tool_id not in self.config.tool_instance_kind:
            raise NotImplementedError(
                '{}Tool tool={!r}: no tool.instances entry in config.yaml for this '
                "id -- every instance id a tree's InstallTool/UninstallTool/"
                'DeployTool/RetractTool nodes reference needs its own '
                '{{id,kind,x,y}} entry under tool.instances.'.format(action_kind, tool_id)
            )
        tool_kind = self.config.tool_instance_kind[tool_id]  # 'cart'/'plow', resolved at translation time
        if action_kind in ('Deploy', 'Retract') and tool_kind != 'plow':
            raise NotImplementedError(
                '{}Tool tool={!r}: kind={!r} -- deploy/retract is currently only '
                "modeled for plow (matching basic_action_theory.pl's own "
                'poss(start_deploy_tool(...)) kind restriction).'.format(action_kind, tool_id, tool_kind)
            )

        duration_ticks_fn = {
            'Install': self.config.install_duration_ticks,
            'Uninstall': self.config.uninstall_duration_ticks,
            'Deploy': self.config.deploy_duration_ticks,
            'Retract': self.config.retract_duration_ticks,
        }[action_kind]
        drain_rate = int(round({
            'Install': self.config.install_drain_rate,
            'Uninstall': self.config.uninstall_drain_rate,
            'Deploy': self.config.deploy_drain_rate,
            'Retract': self.config.retract_drain_rate,
        }[action_kind]))
        duration_ticks = duration_ticks_fn(tool_kind)

        elapsed_var = '{}_elapsed'.format(name.lower())
        outcome_var = '{}_outcome'.format(name.lower())
        # precondition_held/resolved: SNAPSHOTS of the precondition/
        # duration-elapsed check, captured from hitch/hitch_id/deployed/
        # elapsed_var's PRE-tick values as the FIRST two update
        # statements, then reused everywhere else in this SAME tick
        # (including return_cases, which only ever sees POST-update/
        # staged values -- see ir.py's own note on staging). Without
        # this, return_cases re-deriving "is the precondition satisfied"/
        # "has duration elapsed" directly would read the NEWLY updated
        # values instead of the ones this tick's decision was actually
        # based on -- e.g. a successful install flips hitch/hitch_id away
        # from free in THIS SAME tick, which would make a freshly-
        # recomputed precondition check wrongly read as failed. Same idea
        # as MoveTo's own prev_x/prev_y capture, generalized to booleans.
        precondition_held_var = '{}_precondition_held'.format(name.lower())
        resolved_var = '{}_resolved'.format(name.lower())
        self.extra_variables.append(ir.Variable(elapsed_var, 'bl', 'VAR', '[0, {}]'.format(duration_ticks), initial='0'))
        self.extra_variables.append(ir.Variable(outcome_var, 'bl', 'VAR', 'BOOLEAN', initial='False'))
        self.extra_variables.append(ir.Variable(precondition_held_var, 'bl', 'VAR', 'BOOLEAN', initial='False'))
        self.extra_variables.append(ir.Variable(resolved_var, 'bl', 'VAR', 'BOOLEAN', initial='False'))

        read_variables = ['hitch', 'hitch_id', 'deployed', 'battery', elapsed_var, outcome_var, precondition_held_var, resolved_var]
        write_variables = ['battery', elapsed_var, outcome_var, precondition_held_var, resolved_var]
        # success_updates: (var_name, new_value_code_on_success) pairs --
        # turned into uniform case_var entries below. Only the variable(s)
        # THIS action kind can actually change need to appear here (a
        # write_variables entry not listed here would never change, so
        # there's no point declaring it writable).
        if action_kind == 'Install':
            self._ensure_tool_position_vars(tool_id)
            pos_x_var, pos_y_var = self._tool_pos_x_var(tool_id), self._tool_pos_y_var(tool_id)
            proximity_ok = squared_distance_condition('x', 'y', pos_x_var, pos_y_var, 'INSTALL_RANGE', 'lt')
            precondition_ok = "(and, (eq, hitch, 'free'), {})".format(proximity_ok)
            success_updates = [('hitch', "'{}'".format(tool_kind)), ('hitch_id', "'{}'".format(tool_id))]
            read_variables += ['x', 'y', pos_x_var, pos_y_var]
            write_variables += ['hitch', 'hitch_id']
        elif action_kind == 'Uninstall':
            self._ensure_tool_position_vars(tool_id)
            pos_x_var, pos_y_var = self._tool_pos_x_var(tool_id), self._tool_pos_y_var(tool_id)
            precondition_ok = "(and, (eq, hitch_id, '{}'), (not, deployed))".format(tool_id)
            success_updates = [('hitch', "'free'"), ('hitch_id', "'free'"), (pos_x_var, 'x'), (pos_y_var, 'y')]
            read_variables += ['x', 'y', pos_x_var, pos_y_var]
            write_variables += ['hitch', 'hitch_id', pos_x_var, pos_y_var]
        elif action_kind == 'Deploy':
            precondition_ok = "(and, (eq, hitch_id, '{}'), (not, deployed))".format(tool_id)
            success_updates = [('deployed', 'True')]
            write_variables += ['deployed']
        else:  # Retract
            precondition_ok = "(and, (eq, hitch_id, '{}'), deployed)".format(tool_id)
            success_updates = [('deployed', 'False')]
            write_variables += ['deployed']

        duration_reached = '(gte, {}, {})'.format(elapsed_var, duration_ticks)
        resolved_and_succeeded = '(and, {}, {})'.format(resolved_var, outcome_var)

        updates = [
            ('var', precondition_held_var, precondition_ok),
            ('var', resolved_var, '(and, {}, {})'.format(precondition_held_var, duration_reached)),
            ('case_var', outcome_var, [
                (resolved_var, ['True', 'False']),
                (None, [outcome_var]),
            ]),
            ('case_var', elapsed_var, [
                (resolved_var, ['0']),
                (precondition_held_var, ['(add, {}, 1)'.format(elapsed_var)]),
                (None, ['0']),
            ]),
            ('var', 'battery', '(if, {ok}, (max, 0, (sub, battery, {rate})), battery)'.format(ok=precondition_held_var, rate=drain_rate)),
        ]
        for var_name, new_value_on_success in success_updates:
            updates.append(('case_var', var_name, [
                (resolved_and_succeeded, [new_value_on_success]),
                (None, [var_name]),
            ]))

        action = ir.Action(
            name=name,
            read_variables=read_variables,
            write_variables=write_variables,
            updates=updates,
            return_cases=[
                ('(not, {})'.format(precondition_held_var), 'failure'),
                ('(lte, battery, 0)', 'failure'),
                (resolved_and_succeeded, 'success'),
                (resolved_var, 'failure'),
                (None, 'running'),
            ],
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

    def make_sample_value_below(self, sample_id, threshold):
        return self._sample_value_check('SampleValueBelow', 'lt', sample_id, threshold)

    def make_sample_value_equal(self, sample_id, threshold):
        return self._sample_value_check('SampleValueEqual', 'eq', sample_id, threshold)

    def make_sample_value_over(self, sample_id, threshold):
        return self._sample_value_check('SampleValueOver', 'gt', sample_id, threshold)

    def _sample_value_check(self, prefix, op, sample_id, threshold):
        """
        holds(sample_value_below/equal/over(SampleId,Threshold), S) --
        checks the VALUE a SUCCESSFUL <TakeSample id="..."/> came back
        with. A HISTORY lookup, same shape as HaltedWith (the value is
        fixed the instant it's drawn) -- but since this translator's own
        sample_success_<id>/sample_value_<id> already PERSIST as plain
        bl fluents (see make_take_sample), a plain current-state check
        already IS the history lookup: sample_success_<id> stays True
        from the moment of a real success onward (until the SAME id's
        TakeSample runs again), so checking it now is equivalent to
        "did id ever succeed, and is its value still the one from that
        success" -- gated on sample_success_<id> so a never-sampled (or
        since-resampled-to-failure) id reads as false, not a stale value.
        """
        threshold_int = int(round(threshold))
        name = '{}_{}_{}'.format(prefix, sample_id, threshold_int).replace('-', 'm')
        if name in self.checks:
            return name
        success_var = self.sample_success_var(sample_id)
        value_var = self.sample_value_var(sample_id)
        condition = '(and, {}, ({}, {}, {}))'.format(success_var, op, value_var, threshold_int)
        self.checks[name] = ir.Check(name, [success_var, value_var], condition)
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
