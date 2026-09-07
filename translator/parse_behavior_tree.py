"""
Parses a problem's behavior_tree.xml (BT.cpp XML, <TreeNodesModel> section
ignored -- it's redundant with schema.yaml) into a BehaVerify tree{} IR
node, and populates a LeafFactory with every action/check the tree
actually needs.

Composite mapping:
  <Fallback>          -> selector, with_partial_memory (plain BT.cpp Fallback)
  <ReactiveFallback>  -> selector, no memory ('')      (re-check every child from
  <Sequence>          -> sequence, with_partial_memory  the start each tick --
  <ReactiveSequence>  -> sequence, no memory ('')        see leaf_library.py's
                                                          module docstring and
                                                          node_creator.py's
                                                          create_composite_*_without_memory)
This reproduces BT.cpp's ReactiveSequence/ReactiveFallback restart-on-tick
semantics NATIVELY, so a Condition left-sibling of a running action aborts
it the moment the condition goes false -- no derived `triggers=`/guard_break
bookkeeping needed (dropped entirely, per the approved design).

Leaf mapping is the CURRENT schema.yaml's action/condition vocabulary
(PlanWith with an `algorithm` port replaces PlanAstar/PlanStraight/
PlanVoronoi/FollowBoarder; DistanceBelow/DistanceEqual/DistanceOver with a
`threshold` port replace AtGoal), dispatched by XML tag name to a
LeafFactory method (see leaf_library.py for what's implemented vs.
explicitly out of scope).

PlanWith/MoveTo PAIRING: PlanWith writes its output to a `control_points`
port a SUBSEQUENT MoveTo reads (wired via a shared blackboard key in the
real XML, e.g. control_points="{cp}" on both). This translator doesn't
track blackboard ports individually -- it assumes the same adjacency the
real trees always use in practice: a MoveTo leaf is preceded, among its
own composite's children, by the PlanWith that feeds it. _walk's composite
loop below tracks "which MoveTo action the most recently seen PlanWith
sibling selected" and hands it to the next MoveTo leaf it encounters.
"""
import xml.etree.ElementTree as ET

from . import ir

_COMPOSITE_TAGS = {
    # 'with_true_memory' exists in the grammar but is NOT implemented by this
    # BehaVerify build (check_grammar.py raises "True memory not yet
    # implemented"); 'with_partial_memory' (resume from the last-running
    # child, per node_creator.py's create_composite_*_with_partial_memory)
    # is the one that actually matches BT.cpp's plain, non-reactive
    # Sequence/Fallback semantics.
    'Fallback': ('selector', 'with_partial_memory'),
    'ReactiveFallback': ('selector', ''),
    'Sequence': ('sequence', 'with_partial_memory'),
    'ReactiveSequence': ('sequence', ''),
}


def _parse_point(text):
    x_str, y_str = text.split(';')
    return float(x_str), float(y_str)


def _build_non_moveto_leaf(tag, attrib, factory):
    """
    Returns (leaf_kind, leaf_ref_name) for one XML leaf element, OR, for a
    PlanWith, (leaf_kind, leaf_ref_name, next_moveto_name) -- the extra
    element tells the caller which MoveTo action the following MoveTo leaf
    must use. Never handles the 'MoveTo' tag itself -- see _walk, which
    needs the pending-moveto state threaded from the composite loop.
    """
    if tag == 'PlanWith':
        algorithm = attrib['algorithm']
        goal_x_m = goal_y_m = None
        if 'goal' in attrib:
            goal_x_m, goal_y_m = _parse_point(attrib['goal'])
        obstacle_id = attrib.get('obstacle_id')
        offset = float(attrib['offset']) if 'offset' in attrib else None
        plan_name, moveto_name = factory.make_plan_with(algorithm, goal_x_m, goal_y_m, obstacle_id, offset)
        return 'action', plan_name, moveto_name
    if tag == 'BatteryOver':
        return 'check', factory.make_battery_over(float(attrib['threshold']))
    if tag == 'BatteryBelow':
        return 'check', factory.make_battery_below(float(attrib['threshold']))
    if tag == 'BatteryEqual':
        return 'check', factory.make_battery_equal(float(attrib['threshold']))
    if tag == 'DistanceBelow':
        gx, gy = _parse_point(attrib['goal'])
        return 'check', factory.make_distance_below(gx, gy, float(attrib['threshold']))
    if tag == 'DistanceEqual':
        gx, gy = _parse_point(attrib['goal'])
        return 'check', factory.make_distance_equal(gx, gy, float(attrib['threshold']))
    if tag == 'DistanceOver':
        gx, gy = _parse_point(attrib['goal'])
        return 'check', factory.make_distance_over(gx, gy, float(attrib['threshold']))
    if tag == 'ObstacleInBound':
        return 'check', factory.make_obstacle_in_bound(float(attrib['threshold']))
    if tag == 'ObstacleOnPath':
        return 'check', factory.make_obstacle_on_path(float(attrib['threshold']))
    if tag == 'LineOfSightClear':
        return 'check', factory.make_line_of_sight_clear()
    if tag == 'HaltedWith':
        return 'check', factory.make_halted_with()
    raise NotImplementedError('Unrecognized behavior_tree.xml tag: <{}> -- not in schema.yaml\'s vocabulary.'.format(tag))


def _walk(element, factory, path):
    tag = element.tag
    if tag in _COMPOSITE_TAGS:
        node_type, memory = _COMPOSITE_TAGS[tag]
        name = element.attrib.get('name', tag)
        children = []
        pending_moveto = None  # set by the most recent PlanWith sibling, consumed by the next MoveTo
        for child in element:
            if child.tag == 'MoveTo':
                moveto_name = pending_moveto if pending_moveto is not None else factory.make_move_to()
                pending_moveto = None
                alias = '_'.join(path + [name, 'MoveTo'])
                children.append(ir.TreeNode(kind='leaf', leaf_kind='action', leaf_ref=moveto_name, name=alias))
                continue
            if child.tag in _COMPOSITE_TAGS or child.tag == 'Inverter':
                children.append(_walk(child, factory, path + [name]))
                continue
            result = _build_non_moveto_leaf(child.tag, child.attrib, factory)
            if len(result) == 3:
                leaf_kind, leaf_ref, next_moveto = result
                pending_moveto = next_moveto
            else:
                leaf_kind, leaf_ref = result
            alias = '_'.join(path + [name, child.tag])
            children.append(ir.TreeNode(kind='leaf', leaf_kind=leaf_kind, leaf_ref=leaf_ref, name=alias))
        return ir.TreeNode(kind='composite', node_type=node_type, memory=memory, name=name, children=children)
    if tag == 'Inverter':
        raise NotImplementedError('<Inverter> decorator translation is not implemented yet.')
    if tag == 'MoveTo':
        # a MoveTo at the TREE ROOT (no enclosing composite to have carried a
        # preceding PlanWith) -- always the generic, straight-line mover.
        alias = '_'.join(path + ['MoveTo'])
        return ir.TreeNode(kind='leaf', leaf_kind='action', leaf_ref=factory.make_move_to(), name=alias)
    result = _build_non_moveto_leaf(tag, element.attrib, factory)
    leaf_kind, leaf_ref = result[0], result[1]
    alias = '_'.join(path + [tag])
    return ir.TreeNode(kind='leaf', leaf_kind=leaf_kind, leaf_ref=leaf_ref, name=alias)


def collect_goal_points_m(xml_path):
    """All literal `goal="X;Y"` points anywhere in the tree, in metres --
    used by parse_map.py to make sure the grid covers every planning target,
    not just the obstacles."""
    tree = ET.parse(xml_path)
    points = []
    for element in tree.getroot().iter():
        if 'goal' in element.attrib:
            points.append(_parse_point(element.attrib['goal']))
    return points


def collect_move_to_aliases(tree_node):
    """All leaf aliases in the tree that instantiate ANY MoveTo variant
    (the generic 'MoveTo', or a goal-specific 'MoveTo_Astar_<goal>') --
    used by goal_formula.py to build e.g. battery_depleted_in/1's
    `(or, (failure, alias1), (failure, alias2), ...)`."""
    aliases = []
    if tree_node.kind == 'leaf':
        if tree_node.leaf_kind == 'action' and tree_node.leaf_ref.startswith('MoveTo'):
            aliases.append(tree_node.name)
    else:
        for child in tree_node.children:
            aliases.extend(collect_move_to_aliases(child))
    return aliases


def parse_tree(xml_path, factory):
    """
    Returns the IR.TreeNode root for <BehaviorTree ID="main_tree_to_execute">,
    with every referenced action/check added to `factory` along the way.
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()
    main_id = root.attrib.get('main_tree_to_execute')
    bt_elements = root.findall('BehaviorTree')
    chosen = None
    for bt in bt_elements:
        if main_id is None or bt.attrib.get('ID') == main_id:
            chosen = bt
            break
    if chosen is None:
        raise ValueError('No <BehaviorTree> matching main_tree_to_execute="{}" found in {}'.format(main_id, xml_path))
    # a <BehaviorTree> has exactly one real child (the root composite/leaf)
    (root_node,) = list(chosen)
    return _walk(root_node, factory, [])
