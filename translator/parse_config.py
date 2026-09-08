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

        # Position is modeled deterministically (see leaf_library.py's
        # MoveTo) -- the discretized-Gaussian lateral/tangential position
        # noise tables aren't used; lateral_sigma_m is kept only to pace
        # the battery drift's resample period below (drift_resample_period_ticks).
        self.lateral_sigma_m = raw['position']['lateral']['sigma']

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

    def to_cell(self, value_metres):
        return round_position_to_cell(value_metres, self.disc_step_position)

    def to_cells_ceil(self, value_metres):
        return round_distance_to_cells(value_metres, self.disc_step_position, mode='ceil')

    def to_cells_nearest(self, value_metres):
        return round_distance_to_cells(value_metres, self.disc_step_position, mode='nearest')

    @property
    def start_cell(self):
        return (self.to_cell(self.start_x_m), self.to_cell(self.start_y_m))

    @property
    def drift_resample_period_ticks(self):
        """
        How many ticks of accumulated real Brownian drift it takes before a
        full grid cell of lateral deviation becomes plausible -- reused (not
        for position, which is modeled deterministically -- see
        leaf_library.py's MoveTo) to pace how often the battery's
        nondeterministic drift term gets resampled (make_move_to/
        _make_policy_based_plan's shared _battery_drift_updates). Derived
        directly from config.yaml's own physics: lateral position variance
        grows as
        sigma^2 * elapsed (sigma is "metres per sqrt(time-unit)"), and one
        MoveTo tick == one time-unit (speed=1 m/time-unit, disc_step_position
        m/cell -- see this module's own header). Solving
        sigma^2 * t = disc_step_position^2 for t gives the tick count at
        which the STANDARD DEVIATION of drift first reaches one cell width.
        """
        return max(1, round((self.disc_step_position / self.lateral_sigma_m) ** 2))
