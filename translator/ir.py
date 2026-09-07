"""
Shared intermediate representation for the translator pipeline.

Every parse_*.py module fills in part of a single ProblemIR instance;
emit_tree_dsl.py is the only module that turns it into `.tree` DSL text.
Keeping this separate from the DSL grammar itself (rather than building a
textX AST directly) keeps every stage independently testable and keeps the
final rendering (section order, syntax) in exactly one place.
"""
from dataclasses import dataclass, field


@dataclass
class Constant:
    name: str
    value: object  # int, float, or bool -- rendered verbatim


@dataclass
class Variable:
    name: str
    scope: str  # 'bl' | 'env' | 'local'
    model_as: str  # 'VAR' | 'FROZENVAR' | 'DEFINE'
    domain: str  # e.g. '[0, 10]', 'BOOLEAN', 'INT', "{'a','b'}"
    initial: str = None  # code_statement text for plain (non-array) assign{result{...}}
    is_array: bool = False
    array_size: str = None  # code_statement text
    array_default: str = None  # code_statement text
    array_assigns: list = field(default_factory=list)  # list of (index_code, value_code)
    static: bool = False  # only meaningful for DEFINE


@dataclass
class Check:
    name: str
    read_variables: list  # list of variable names
    condition: str  # code_statement text


@dataclass
class Action:
    name: str
    read_variables: list
    write_variables: list
    local_variables: list = field(default_factory=list)
    # ordered list of update statements, executed before the case-based return.
    # each entry is either:
    #   ('var', var_name, value_code)                          -- plain variable_statement
    #   ('read_env', label, condition_code, [(var_name, code)]) -- read_environment block
    #       (REQUIRED to reference any `env`-scope variable inside value_code --
    #       check_grammar.py rejects a bare variable_statement that does, see
    #       leaf_library.py's make_move_to)
    updates: list = field(default_factory=list)
    # return_statement: list of (condition_code_or_None, status) with the last one being default (condition None)
    return_cases: list = field(default_factory=list)


@dataclass
class TreeNode:
    """One node in the tree{} block."""
    kind: str  # 'composite' | 'leaf'
    # composite fields
    node_type: str = None  # 'sequence' | 'selector'
    memory: str = None  # 'with_true_memory' | '' (reactive/no-memory)
    name: str = None  # composite's own name, or leaf's local alias
    children: list = field(default_factory=list)
    # leaf fields
    leaf_kind: str = None  # 'check' | 'action'
    leaf_ref: str = None  # name of the check_node/action_node this leaf instantiates


@dataclass
class Specification:
    spec_type: str  # 'LTLSPEC' | 'CTLSPEC' | 'INVARSPEC'
    code: str  # code_statement text


@dataclass
class ProblemIR:
    constants: list = field(default_factory=list)
    variables: list = field(default_factory=list)
    environment_update: list = field(default_factory=list)  # list of (var_name, value_code)
    checks: list = field(default_factory=list)
    actions: list = field(default_factory=list)
    tree_root: TreeNode = None
    specifications: list = field(default_factory=list)
    tick_prerequisite: str = 'True'
