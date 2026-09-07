"""
Parses a problem's behavior_tree.xml (BT.cpp XML, <TreeNodesModel> section
ignored -- it's redundant with schema.yaml) into a BehaVerify tree{} IR
node, and populates a LeafFactory with every action/check the tree
actually needs.

Composite mapping:
  <Fallback>          -> selector, with_true_memory  (plain BT.cpp Fallback)
  <ReactiveFallback>  -> selector, no memory ('')    (re-check every child from
  <Sequence>          -> sequence, with_true_memory   the start each tick --
  <ReactiveSequence>  -> sequence, no memory ('')      see leaf_library.py's
                                                        module docstring and
                                                        node_creator.py's
                                                        create_composite_*_without_memory)
This reproduces BT.cpp's ReactiveSequence/ReactiveFallback restart-on-tick
semantics NATIVELY, so a Condition left-sibling of a running action aborts
it the moment the condition goes false -- no derived `triggers=`/guard_break
bookkeeping needed (dropped entirely, per the approved design).

Leaf mapping is schema.yaml's action/condition vocabulary, dispatched by
XML tag name to a LeafFactory method (see leaf_library.py for what's
implemented vs. explicitly out of scope).
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


def _build_leaf(tag, attrib, factory):
    """Returns (leaf_kind, leaf_ref_name) for one XML leaf element."""
    if tag == 'MoveTo':
        return 'action', factory.make_move_to()
    if tag == 'PlanStraight':
        gx, gy = _parse_point(attrib['goal'])
        return 'action', factory.make_plan_straight(gx, gy)
    if tag == 'PlanAstar':
        return 'action', factory.make_plan_astar()
    if tag == 'PlanVoronoi':
        return 'action', factory.make_plan_voronoi()
    if tag == 'FollowBoarder':
        return 'action', factory.make_follow_boarder()
    if tag == 'BatteryOver':
        return 'check', factory.make_battery_over(float(attrib['threshold']))
    if tag == 'BatteryBelow':
        return 'check', factory.make_battery_below(float(attrib['threshold']))
    if tag == 'BatteryEqual':
        return 'check', factory.make_battery_equal(float(attrib['threshold']))
    if tag == 'AtGoal':
        gx, gy = _parse_point(attrib['goal'])
        return 'check', factory.make_at_goal(gx, gy, float(attrib['tolerance']))
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
        children = [_walk(child, factory, path + [name]) for child in element]
        return ir.TreeNode(kind='composite', node_type=node_type, memory=memory, name=name, children=children)
    if tag == 'Inverter':
        raise NotImplementedError('<Inverter> decorator translation is not implemented yet.')
    # leaf
    leaf_kind, leaf_ref = _build_leaf(tag, element.attrib, factory)
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


def collect_move_to_aliases(tree_node, move_to_action_name):
    """All leaf aliases in the tree that instantiate the shared MoveTo action
    -- used by goal_formula.py to build e.g. battery_depleted_in/1's
    `(or, (failure, alias1), (failure, alias2), ...)`."""
    aliases = []
    if tree_node.kind == 'leaf':
        if tree_node.leaf_kind == 'action' and tree_node.leaf_ref == move_to_action_name:
            aliases.append(tree_node.name)
    else:
        for child in tree_node.children:
            aliases.extend(collect_move_to_aliases(child, move_to_action_name))
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
