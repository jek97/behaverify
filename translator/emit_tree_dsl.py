"""
Renders a fully-populated ir.ProblemIR into BehaVerify `.tree` DSL text,
following the exact section order behaverify.tx's grammar requires:
configuration, enumerations, constants, variables, environment_update,
checks, environment_checks, actions, sub_trees, tree, tick_prerequisite,
specifications.
"""
from . import ir


def _indent(level):
    return '    ' * level


def _render_enumerations(enumerations):
    body = ', '.join(enumerations)
    return 'enumerations {\n    ' + body + '\n}\n' if enumerations else 'enumerations {}\n'


def _render_constants(constants):
    body = ', '.join('{} := {}'.format(c.name, c.value) for c in constants)
    return 'constants {' + body + '}\n'


def _render_variable(var):
    header = 'variable { ' + var.scope + ' ' + var.name + ' ' + var.model_as + ' ' + var.domain
    if var.model_as == 'DEFINE' and var.static:
        header += ' static'
    if not var.is_array:
        return header + ' assign{result{' + var.initial + '}}}\n'
    lines = [header + ' array ' + var.array_size]
    lines.append('default{assign{result{' + var.array_default + '}}}')
    lines.append('constant_index')
    for index_code, value_code in var.array_assigns:
        lines.append('index_of{' + index_code + '}assign{result{' + value_code + '}}')
    lines.append('}')
    return ' '.join(lines) + '\n'


def _render_variables(variables):
    body = ''.join(_render_variable(v) for v in variables)
    return 'variables {\n' + body + '}\n'


def _render_environment_update(env_updates):
    body = ''.join(
        'variable_statement {{ {} assign{{result{{{}}}}}}}\n'.format(name, code)
        for name, code in env_updates
    )
    return 'environment_update {\n' + body + '}\n'


def _render_checks(checks):
    body = ''
    for c in checks:
        body += (
            'check { ' + c.name + '\n'
            + '    arguments {} read_variables {' + ', '.join(c.read_variables) + '}\n'
            + '    condition {' + c.condition + '}\n'
            + '}\n'
        )
    return 'checks {\n' + body + '}\n'


def _render_action(a):
    lines = ['action { ' + a.name]
    lines.append(
        '    arguments {} local_variables {' + ', '.join(a.local_variables) + '}'
        + ' read_variables {' + ', '.join(a.read_variables) + '}'
        + ' write_variables {' + ', '.join(a.write_variables) + '}'
    )
    lines.append('    initial_values {}')
    lines.append('    update {')
    for entry in a.updates:
        if entry[0] == 'var':
            _, var_name, value_code = entry
            lines.append('        variable_statement {' + var_name + ' assign{result{' + value_code + '}}}')
        elif entry[0] == 'read_env':
            _, label, condition_code, var_assigns = entry
            lines.append('        read_environment {' + label)
            lines.append('            condition {' + condition_code + '}')
            for var_name, value_code in var_assigns:
                lines.append('            variable_statement {' + var_name + ' assign{result{' + value_code + '}}}')
            lines.append('        }')
        elif entry[0] == 'case_var':
            _, var_name, cases = entry
            lines.append('        variable_statement {' + var_name + ' assign{')
            for condition_code, values in cases:
                values_text = ', '.join(values)
                if condition_code is None:
                    lines.append('            result {' + values_text + '}')
                else:
                    lines.append('            case {' + condition_code + '} result {' + values_text + '}')
            lines.append('        }}')
        else:
            raise ValueError('Unknown action update entry kind: {}'.format(entry[0]))
    lines.append('        return_statement {')
    for condition, status in a.return_cases:
        if condition is None:
            lines.append('            result {' + status + '}')
        else:
            lines.append('            case {' + condition + '} result {' + status + '}')
    lines.append('        }')
    lines.append('    }')
    lines.append('}')
    return '\n'.join(lines) + '\n'


def _render_actions(actions):
    return 'actions {\n' + ''.join(_render_action(a) for a in actions) + '}\n'


def _render_tree_node(node, level):
    pad = _indent(level)
    if node.kind == 'leaf':
        return pad + node.name + ': ' + node.leaf_ref + ' {}\n'
    memory_text = (' ' + node.memory) if node.memory else ''
    lines = [pad + 'composite {' + node.name]
    lines.append(pad + '    ' + node.node_type + memory_text)
    lines.append(pad + '    children {')
    for child in node.children:
        lines.append(_render_tree_node(child, level + 2))
    lines.append(pad + '    }')
    lines.append(pad + '}')
    return '\n'.join(lines) + '\n'


def render(problem_ir):
    out = []
    out.append('configuration {}\n')
    out.append(_render_enumerations(problem_ir.enumerations))
    out.append(_render_constants(problem_ir.constants))
    out.append(_render_variables(problem_ir.variables))
    out.append(_render_environment_update(problem_ir.environment_update))
    out.append(_render_checks(problem_ir.checks))
    out.append('environment_checks {}\n')
    out.append(_render_actions(problem_ir.actions))
    out.append('sub_trees {}\n')
    out.append('tree {\n' + _render_tree_node(problem_ir.tree_root, 1) + '}\n')
    out.append('tick_prerequisite {' + problem_ir.tick_prerequisite + '}\n')
    spec_body = ''.join(
        '    {} {{{}}}\n'.format(s.spec_type, s.code)
        for s in problem_ir.specifications
    )
    out.append('specifications {\n' + spec_body + '}\n')
    return ''.join(out)
