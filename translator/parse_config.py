"""
Parses a problem's config.yaml into the numeric parameters the rest of the
translator needs (discretization steps, battery/robot/tolerance constants,
and the noise support tables used by noise.py).

NOTE on config.yaml's own disc_step_* values: in the source ProbLog system
they control noise-draw MERGING for proof-sharing performance, not movement
granularity. We reuse disc_step_position as our own tick-to-distance ratio
(one grid cell of travel per tick) per an explicit user decision, and
disc_step_position again (not disc_step_time) as the length unit every
other spatial quantity (goal coordinates, distance thresholds) gets
discretized into -- see round_distance_to_cells()/round_position_to_cell().
"""
import math

import yaml


def load_config(config_path):
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def round_distance_to_cells(value_metres, step, mode='ceil'):
    """
    Turn a continuous distance/threshold into a whole number of grid cells.
    mode='ceil' (default): never UNDER-report a safety-relevant distance
    (a threshold rounded down could let the abstraction miss a real
    hazard). mode='nearest': for quantities where over- and under-shoot are
    equally acceptable (e.g. the goal tolerance, which the grid resolution
    already dominates).
    """
    ratio = value_metres / step
    if mode == 'ceil':
        return int(math.ceil(ratio - 1e-9))
    if mode == 'floor':
        return int(math.floor(ratio + 1e-9))
    return int(round(ratio))


def round_position_to_cell(value_metres, step):
    """A coordinate (goal point, start point, obstacle vertex) -> nearest cell index."""
    return int(round(value_metres / step))


class ProblemConfig:
    def __init__(self, config_path):
        raw = load_config(config_path)

        self.start_x_m = raw['initial_situation']['start_x']
        self.start_y_m = raw['initial_situation']['start_y']

        self.robot_radius = raw['robot']['radius']
        self.safety_buffer = raw['robot']['safety_buffer']
        self.safety_margin = self.robot_radius + self.safety_buffer

        self.speed = raw['motion']['speed']

        self.disc_step_position = raw['position']['disc_step_position']
        if self.disc_step_position in (0, None):
            raise NotImplementedError(
                'disc_step_position: 0 (exact/no discretization) is not supported by this '
                'translator -- nuXmv needs a finite grid; pick a nonzero step in config.yaml.'
            )

        self.position_noise_support = [pt['value'] for pt in raw['position']['lateral']['discretized_gaussian']]
        self.tangential_noise_support = [pt['value'] for pt in raw['position']['tangential']['discretized_gaussian']]

        battery = raw['battery']
        self.battery_enabled = bool(battery.get('enabled', True))
        self.battery_start = battery['start']
        self.idle_drain_rate = battery['idle_drain_rate']
        self.moving_drain_rate = battery['moving_drain_rate']
        self.disc_step_battery = battery.get('disc_step_battery', 0)
        self.battery_noise_support = [pt['value'] for pt in battery['discretized_gaussian']]

        # NOTE: tolerances.goal was removed from config.yaml -- goal
        # tolerance is no longer a single global config value, it's now
        # per-node (DistanceBelow/DistanceEqual/DistanceOver's own
        # `threshold` port, and visited/3's own explicit Tol argument).
        self.on_track_tolerance_m = raw['tolerances']['on_track']

        self.disc_step_time = raw['grounding']['disc_step_time']

        # ------------------------------------------------------------
        # TakeSample / InstallTool / UninstallTool (new actions in the
        # source theory -- see basic_action_theory.pl's own
        # do_node(take_sample(...))/install_tool_leg/uninstall_tool_leg).
        # Same key shape and defaults as config_to_prolog.py's own
        # _binary_result_block/_tool_duration_facts/_tool_moveto_param_facts,
        # so a config.yaml with none of these sections behaves exactly
        # like the source system's own defaults.
        # ------------------------------------------------------------
        self.sample_success_probability = float(raw.get('sample', {}).get('success_probability', 0.5))

        tool_cfg = raw.get('tool', {})
        install_cfg = tool_cfg.get('install', {})
        uninstall_cfg = tool_cfg.get('uninstall', {})
        equipped_cfg = tool_cfg.get('equipped', {})

        self.install_success_probability = float(install_cfg.get('success_probability', 0.9))
        self.uninstall_success_probability = float(uninstall_cfg.get('success_probability', 0.9))
        # ONE drain rate per action TYPE (not per tool) -- matches
        # config_to_prolog.py's own install_tool_drain_rate/1/
        # uninstall_tool_drain_rate/1 shape.
        self.install_drain_rate = float(install_cfg.get('drain_rate', self.idle_drain_rate))
        self.uninstall_drain_rate = float(uninstall_cfg.get('drain_rate', self.idle_drain_rate))
        self._install_duration_s = install_cfg.get('duration_seconds', {})
        self._uninstall_duration_s = uninstall_cfg.get('duration_seconds', {})

        # tool.equipped.<cart|plow>.moving_drain_rate: the MoveTo drain
        # rate used WHILE that tool is equipped (hitch(Tool,S) --
        # basic_action_theory.pl's own tool_moving_drain_rate/2), reused
        # by leaf_library.py's _moving_drain_for_hitch. Defaults to the
        # SAME moving_drain_rate as no tool equipped, matching
        # tool_moving_drain_rate(free,_)'s own reuse of battery.moving_
        # drain_rate. NOTE: tool.equipped.<tool>.speed is intentionally
        # NOT read here -- see leaf_library.py's _moving_drain_for_hitch
        # docstring for why velocity-while-equipped is out of scope.
        self.tool_moving_drain_rate = {
            tool: float(equipped_cfg.get(tool, {}).get('moving_drain_rate', self.moving_drain_rate))
            for tool in ('cart', 'plow')
        }

    def install_duration_ticks(self, tool):
        """
        config.yaml's tool.install.duration_seconds.<tool> (default 10s,
        matching config_to_prolog.py's own _DEFAULT_TOOL_DURATION_S),
        rounded DIRECTLY to ticks -- this translator has no other
        seconds<->tick conversion anywhere (MoveTo's own "one grid cell
        per tick" is ALSO an arbitrary, explicitly-approved granularity
        choice, not derived from real time/speed -- see this module's own
        header), so "1 second of Duration = 1 tick" here is the same kind
        of simplest-possible choice, not a physically-derived one.
        """
        return max(1, round(float(self._install_duration_s.get(tool, 10.0))))

    def uninstall_duration_ticks(self, tool):
        """See install_duration_ticks's own note -- same shape, uninstall's own config key."""
        return max(1, round(float(self._uninstall_duration_s.get(tool, 10.0))))

    def to_cell(self, value_metres):
        return round_position_to_cell(value_metres, self.disc_step_position)

    def to_cells_ceil(self, value_metres):
        return round_distance_to_cells(value_metres, self.disc_step_position, mode='ceil')

    def to_cells_nearest(self, value_metres):
        return round_distance_to_cells(value_metres, self.disc_step_position, mode='nearest')

    @property
    def start_cell(self):
        return (self.to_cell(self.start_x_m), self.to_cell(self.start_y_m))
