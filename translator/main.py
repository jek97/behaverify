"""
CLI: translate one example_problem_to_solve/<problem>/ directory into a
BehaVerify `.tree` file.

Usage:
    python3 -m translator.main example_problem_to_solve/problem4 problem4.tree
"""
import argparse
import os
import sys

from . import emit_tree_dsl, goal_formula, ir, parse_behavior_tree, parse_config, parse_map
from .leaf_library import LeafFactory


def translate_problem(problem_dir, spec_type='LTLSPEC'):
    config = parse_config.ProblemConfig(os.path.join(problem_dir, 'config.yaml'))

    obstacles_m = parse_map.parse_obstacles(os.path.join(problem_dir, 'obstacles_generated.pl'))
    xml_path = os.path.join(problem_dir, 'behavior_tree.xml')
    goal_points_m = parse_behavior_tree.collect_goal_points_m(xml_path)
    goal_points_m.append((config.start_x_m, config.start_y_m))
    bounds = parse_map.compute_grid_bounds(obstacles_m, goal_points_m, config)
    grid = parse_map.build_clearance_grid(obstacles_m, config, bounds)
    grid.obstacles_m = obstacles_m  # needed by leaf_library.make_plan_astar's policy precomputation

    factory = LeafFactory(config, grid)
    shared_vars = factory.shared_variables()

    tree_root = parse_behavior_tree.parse_tree(xml_path, factory)
    factory.move_to_aliases = parse_behavior_tree.collect_move_to_aliases(tree_root)

    goal_code = goal_formula.translate(os.path.join(problem_dir, 'goal_formula.pl'), config, factory)

    problem_ir = ir.ProblemIR()
    problem_ir.constants = [ir.Constant(name, value) for name, value in factory.constants.items()]
    problem_ir.variables = shared_vars + factory.extra_variables
    problem_ir.environment_update = _noise_environment_update(shared_vars)
    problem_ir.checks = list(factory.checks.values())
    problem_ir.actions = list(factory.actions.values())
    problem_ir.tree_root = tree_root
    problem_ir.specifications = [ir.Specification(spec_type, goal_code)]
    problem_ir.tick_prerequisite = 'True'

    return problem_ir


def _noise_environment_update(shared_vars):
    """Every `env` variable gets an unconditional free choice each tick --
    replacing the source system's probabilistic noise draw with nuXmv's
    nondeterministic choice over the same discrete support (see
    leaf_library.LeafFactory.shared_variables and the module docstring in
    translator/__init__.py for why)."""
    updates = []
    for var in shared_vars:
        if var.scope != 'env':
            continue
        domain_values = var.domain.strip('{}').split(',')
        domain_values = [v.strip() for v in domain_values]
        loop_domain = '{' + ', '.join(domain_values) + '}'
        code = '(loop, __i, {} such_that True, __i)'.format(loop_domain)
        updates.append((var.name, code))
    return updates


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('problem_dir', help='e.g. example_problem_to_solve/problem4')
    parser.add_argument('output_tree', help='output .tree file path')
    parser.add_argument('--spec_type', default='LTLSPEC', choices=['LTLSPEC', 'CTLSPEC', 'INVARSPEC'],
                         help='LTLSPEC = universal ("for all paths"); use CTLSPEC for an existential (EF-style) reading.')
    args = parser.parse_args(argv)

    problem_ir = translate_problem(args.problem_dir, spec_type=args.spec_type)
    text = emit_tree_dsl.render(problem_ir)
    with open(args.output_tree, 'w', encoding='utf-8') as f:
        f.write(text)
    print('Wrote {}'.format(args.output_tree))


if __name__ == '__main__':
    sys.exit(main())
