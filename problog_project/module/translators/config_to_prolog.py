#!/usr/bin/env python3
"""
module/translators/config_to_prolog.py

Turns a problem's config.yaml into config_generated.pl -- plain Prolog
facts, same predicate names/arities basic_action_theory.pl used to
define inline (robot_radius/1, sigma/1, battery_start/1, the z/2 and
zbatt/1 annotated disjunctions, etc.) -- see config.yaml's own header
for the full rationale.

This mirrors an ALREADY-ESTABLISHED project pattern: occgrid_to_problog.py
generates obstacles_generated.pl from a map, and bt_to_prolog.py
generates plan_generated.pl from a BT.cpp XML tree -- both siblings of
this file in module/translators/, both writing into the SAME problem
directory this file does. config_generated.pl is a third instance of
the same "source data -> generated Prolog facts, consulted separately"
shape, just with config.yaml as the source instead of a map or a plan.

config.yaml's own top level is organized by PHYSICAL QUANTITY
(position:, battery:, ...) rather than by "noise vs. drain vs.
grounding" -- see that file's own header for why -- and the Prolog FACT
NAMES this emits mirror config.yaml's own key names directly (sigma/1,
sigma_tangential/1, battery_start/1, disc_step_position/1, ...) -- the
three discretization-step knobs (disc_step_position/1, disc_step_
battery/1, disc_step_time/1) are named identically to their own
config.yaml keys (position.disc_step_position, battery.disc_step_
battery, grounding.disc_step_time) specifically so the two stay
trivially greppable as the same knob.

battery.enabled (config.yaml) drives TWO things here:
  - battery_enabled/1: a plain fact basic_action_theory.pl doesn't
    itself read (nothing there branches on it) -- it exists so other
    generators/tooling can inspect it without re-parsing config.yaml.
  - whether query(any_battery_depletion) is emitted at all (see
    render_prolog's own note on this below) -- basic_action_theory.pl's
    own Section 10 no longer declares this query as a hardcoded fact,
    specifically so it can be conditional on a PER-PROBLEM config
    value instead of being the same for every problem.
Every OTHER battery-related fact (battery_start/1, idle_drain_rate/1,
moving_drain_rate/1, sigma_battery/1, disc_step_battery/1, zbatt/1's
own table) is ALWAYS emitted regardless of battery.enabled -- battery/3
(the fluent itself) is unconditional theory code, still tracking charge
level either way; only whether battery can ever be a HALTING cause
(handled in module/translators/bt_to_prolog.py, which strips battery-
related trigger names out of every leg's own Triggers list when
disabled) and whether depletion is queried change.

Usage:
    python3 module/translators/config_to_prolog.py
        (regenerates config_generated.pl from config.yaml, both in
        problems/problem0/ by default)

main.py calls generate() itself before every run, so you don't
normally need to run this by hand -- it's here mainly so
config_generated.pl can be regenerated/inspected on its own, and so the
generation logic has exactly one implementation.
"""
import os
import sys

import yaml

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))
_DEFAULT_PROBLEM_DIR = os.path.join(_PROJECT_ROOT, "problems", "problem0")
DEFAULT_CONFIG_PATH = os.path.join(_DEFAULT_PROBLEM_DIR, "config.yaml")
DEFAULT_OUTPUT_PATH = os.path.join(_DEFAULT_PROBLEM_DIR, "config_generated.pl")


def load_config(config_path=DEFAULT_CONFIG_PATH):
    with open(config_path) as f:
        return yaml.safe_load(f)


def _check_gaussian_weights(label, discretized_gaussian):
    total = sum(entry["weight"] for entry in discretized_gaussian)
    if abs(total - 1.0) > 1e-9:
        print(
            f"[warn] config.yaml's {label}.discretized_gaussian weights "
            f"sum to {total!r}, not 1.0 -- ProbLog silently treats the missing "
            f"mass as an implicit failure branch, which will cap every "
            f"downstream probability. Fix the weights in config.yaml.",
            file=sys.stderr,
        )


def _format_number(x):
    """Preserve int vs. float formatting from the YAML source (e.g.
    battery_start(100). stays an integer fact, sigma(0.15). stays a
    float) -- Prolog's own arithmetic treats the two interchangeably,
    this is purely for the generated file to read naturally."""
    return repr(x)


def _gaussian_disjunction(functor, args_prefix, discretized_gaussian):
    """Build one annotated-disjunction block, e.g.:
        0.0606::z(do(startMoveto(CP,Triggers,ActionCode,T0),S), -2.0) ;
        ...
        0.0606::z(do(startMoveto(CP,Triggers,ActionCode,T0),S),  2.0).
    or, for a zero-argument functor like zbatt/1:
        0.0606::zbatt(-2.0) ;
        ...
        0.0606::zbatt( 2.0).
    """
    lines = []
    n = len(discretized_gaussian)
    for i, entry in enumerate(discretized_gaussian):
        weight = _format_number(entry["weight"])
        value = _format_number(entry["value"])
        head = f"{functor}({args_prefix}{value})" if args_prefix else f"{functor}({value})"
        terminator = " ;" if i < n - 1 else "."
        lines.append(f"{weight}::{head}{terminator}")
    return "\n".join(lines)


def render_prolog(config):
    position_cfg = config["position"]
    battery_cfg = config["battery"]

    _check_gaussian_weights("position.lateral", position_cfg["lateral"]["discretized_gaussian"])
    _check_gaussian_weights("position.tangential", position_cfg["tangential"]["discretized_gaussian"])
    _check_gaussian_weights("battery", battery_cfg["discretized_gaussian"])

    # position.disc_step_position/battery.disc_step_battery default to 0
    # if omitted; grounding.disc_step_time the same, via its own (much
    # smaller, single-purpose) grounding: section -- all three
    # "disabled, exact" at 0, per basic_action_theory.pl's own quantize/
    # quantize_down/quantize_up.
    disc_step_position = position_cfg.get("disc_step_position", 0.0)
    disc_step_battery = battery_cfg.get("disc_step_battery", 0.0)
    disc_step_time = config.get("grounding", {}).get("disc_step_time", 0.0)

    battery_enabled = battery_cfg.get("enabled", True)

    # ActionCode is a free variable here, same as CP/Triggers/T0 --
    # z/zt's own key doesn't need its ACTUAL value (CP+T0+S already
    # uniquely pin down "this leg"), it just has to be PRESENT so the
    # key's arity matches startMoveto/4 (see basic_action_theory.pl's
    # own poss(haltMoveto(...))/tag_reason note for why startMoveto
    # gained this 4th argument).
    z_block = _gaussian_disjunction(
        "z", "do(startMoveto(CP,Triggers,ActionCode,T0),S), ",
        position_cfg["lateral"]["discretized_gaussian"])
    zt_block = _gaussian_disjunction(
        "zt", "do(startMoveto(CP,Triggers,ActionCode,T0),S), ",
        position_cfg["tangential"]["discretized_gaussian"])
    zbatt_block = _gaussian_disjunction(
        "zbatt", "", battery_cfg["discretized_gaussian"])

    lines = [
        "% AUTO-GENERATED by module/translators/config_to_prolog.py from",
        "% this problem's own config.yaml -- DO NOT HAND-EDIT, edit config.yaml",
        "% instead and regenerate (main.py does this automatically before",
        "% every run).",
        "",
        f"start({_format_number(config['initial_situation']['start_x'])},"
        f"{_format_number(config['initial_situation']['start_y'])}).",
        f"robot_radius({_format_number(config['robot']['radius'])}).",
        f"safety_buffer({_format_number(config['robot']['safety_buffer'])}).",
        f"speed({_format_number(config['motion']['speed'])}).",
        f"sigma({_format_number(position_cfg['lateral']['sigma'])}).",
        f"sigma_tangential({_format_number(position_cfg['tangential']['sigma'])}).",
        f"disc_step_position({_format_number(disc_step_position)}).",
        f"battery_enabled({str(bool(battery_enabled)).lower()}).",
        f"sigma_battery({_format_number(battery_cfg['sigma'])}).",
        f"battery_start({_format_number(battery_cfg['start'])}).",
        f"idle_drain_rate({_format_number(battery_cfg['idle_drain_rate'])}).",
        f"moving_drain_rate({_format_number(battery_cfg['moving_drain_rate'])}).",
        f"disc_step_battery({_format_number(disc_step_battery)}).",
        f"tolerance({_format_number(config['tolerances']['on_track'])}).",
        f"num_samples({_format_number(config['verification']['num_samples'])}).",
        f"bracket_samples({_format_number(config['verification']['bracket_samples'])}).",
        f"crossing_eps({_format_number(config['verification']['crossing_eps'])}).",
        f"disc_step_time({_format_number(disc_step_time)}).",
        "",
        z_block,
        "",
        zt_block,
        "",
        zbatt_block,
        "",
    ]

    # any_battery_depletion is basic_action_theory.pl's own ONLY query
    # about "finishing the battery" (see main.py's SUMMARY_QUERIES) --
    # emitted here, conditionally, rather than as a hardcoded fact in
    # Section 10 of basic_action_theory.pl, specifically so a problem
    # with battery.enabled: false can drop it without touching the
    # (problem-independent) theory file at all.
    if battery_enabled:
        lines += ["query(any_battery_depletion).", ""]

    return "\n".join(lines)


def generate(config_path=DEFAULT_CONFIG_PATH, output_path=DEFAULT_OUTPUT_PATH):
    config = load_config(config_path)
    text = render_prolog(config)
    with open(output_path, "w") as f:
        f.write(text)
    return output_path


if __name__ == "__main__":
    out = generate()
    print(f"Wrote {out}")
