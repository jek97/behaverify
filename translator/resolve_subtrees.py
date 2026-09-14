"""
Standalone preprocessing script -- NOT wired into translate_problem's own
runtime parsing at all (parse_behavior_tree.py still has no <include>/
<SubTree>/{blackboard_key} support whatsoever). Run this FIRST, by hand,
against a problem directory whose own behavior_tree.xml uses <include>/
<SubTree>; it writes out a single flattened, self-contained
behavior_tree_resolved.xml the existing translator/ package can then
consume completely unchanged (point translator/main.py at that file
instead of the original).

Mirrors, as closely as this translator's own much narrower vocabulary
needs, the SAME two mechanisms problog_project_guideline/module/
translators/bt_to_prolog.py already implements for the real ProbLog
pipeline (see that file's own _collect_tree_registry/_apply_subtree_
remap and its module docstring's "SubTree"/"<include>" notes) -- this
script is deliberately not a generalization of that logic, just a
second, independent implementation of the same two ideas against plain
ElementTree, kept dead simple:

  1. SUBTREE FILE DISCOVERY: <include path="foo.xml"/> is BT.cpp v4
     syntax naming an bare filename living ALONGSIDE the file that
     includes it (same restriction bt_to_prolog.py enforces -- no `/`,
     no `..`). This repo's own subtree library lives in
     problog_project_guideline/problems/subtrees/ (a SIBLING directory
     of every individual problem directory, e.g. .../problems/
     problem2L/), not inside each problem directory itself -- so any
     problem that actually references one needs its own LOCAL copy
     first (this is also exactly what the real problem directories
     already do -- problem2L/plowing.xml etc. are themselves such
     copies, committed alongside that problem's own behavior_tree.xml).
     ensure_subtree_files() copies over whichever referenced files
     the problem directory doesn't already have, from that same-
     directory-as-problems/ "subtrees" sibling folder.

  2. STRUCTURAL ASSEMBLY: once every <include>d file is available
     locally, resolve_tree() builds the same {tree_id: <BehaviorTree>}
     registry bt_to_prolog.py's own _collect_tree_registry does, then
     recursively expands every <SubTree ID="..." port="..."/> into a
     deep-copied, port-substituted clone of that tree's own body,
     spliced in exactly where the <SubTree> sat -- see
     _apply_blackboard_remap's own docstring for the substitution rule
     itself (identical to bt_to_prolog.py's _apply_subtree_remap: a
     REMAPPED "{key}" becomes the caller's own text verbatim; an
     UNREMAPPED one is renamed "{key__subN}", unique per call site, so
     two instantiations of the same subtree never collide).

WHAT THIS SCRIPT DELIBERATELY DOES NOT DO: resolve a "{key}" placeholder
down to a LITERAL value when nothing upstream ever bound it to one.
Every "{key}" left over after assembly (see collect_unresolved_
blackboard_refs) is EITHER a genuinely dynamic value -- e.g. this
project's own plowing.xml/install_closest_plow.xml subtrees feed
NearestToolOfKind's own `position` OUTPUT port straight into a later
PlanWith's `goal` INPUT port; which tool is nearest, and therefore which
point that even is, is runtime/state-dependent, not a translation-time
constant -- OR a port the CALLING tree simply never remapped at all (a
genuine caller bug). Resolving the first case would mean this script
doing actual planning/reasoning about the model, which is squarely
translator/leaf_library.py's own job (query actions like
NearestToolOfKind/ToolPosition/HitchedId are themselves a separate,
not-yet-implemented Tier of work -- this script does not add them).
So: this script's OWN output is directly usable right now for any
problem whose subtrees only ever remap LITERAL ports (a plain "X;Y"
Point, a literal tool id, ...); a problem like problem2L (below), whose
subtrees route a query action's own output into a later port, still
gets a fully STRUCTURALLY flattened single tree out of this script --
just with the honest {key} placeholders `main()` prints a warning
about, still left for a human (or a future query-action-aware
translator change) to actually resolve.

ONE unresolved-ref case is entirely EXPECTED and harmless, though, even
for a tree that never used <SubTree>/<include> at all: every PlanWith/
PlanWithWaypoints/MoveTo `control_points="{cp}"` port -- see this
repo's own translator/parse_behavior_tree.py module docstring's own
"PlanWith/MoveTo PAIRING" note: this translator's leaf_library.py never
reads control_points at all, it infers the pairing purely from XML
sibling ADJACENCY, so a leftover "{cp}" there is simply never consumed,
by design, whether or not this script ran.

Usage:
    python3 -m translator.resolve_subtrees <problem_dir>

Writes <problem_dir>/behavior_tree_resolved.xml (never overwrites the
original behavior_tree.xml).
"""
import copy
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET

_BLACKBOARD_RE = re.compile(r'^\{(\w+)\}$')


def _is_blackboard_ref(value):
    return _BLACKBOARD_RE.match(value) is not None


def _blackboard_key(value):
    return _BLACKBOARD_RE.match(value).group(1)


def default_subtrees_source(problem_dir):
    """The 'subtrees' folder SIBLING to the problem directory itself
    (problem_dir's own parent/subtrees) -- matches this repo's own
    layout (problog_project_guideline/problems/<problem>/ alongside
    problog_project_guideline/problems/subtrees/)."""
    return os.path.join(os.path.dirname(os.path.abspath(problem_dir)), 'subtrees')


def collect_include_filenames(xml_path):
    """Every bare filename any <include path="..."/> in xml_path names
    (BT.cpp v4 syntax: direct children of <root>, siblings of
    <BehaviorTree>) -- does NOT recurse into other files, since at this
    point they may not exist locally yet (that's exactly what
    ensure_subtree_files is for)."""
    root = ET.parse(xml_path).getroot()
    names = []
    for inc in root.findall('include'):
        path = inc.attrib.get('path')
        if not path:
            raise ValueError('<include> in {} is missing its required path="..." attribute.'.format(xml_path))
        if '/' in path or '\\' in path or os.path.isabs(path) or '..' in path.replace('\\', '/').split('/'):
            raise ValueError(
                '<include path="{}"> in {} must be a bare filename in the SAME directory '
                '(no path separators or ".." segments).'.format(path, xml_path)
            )
        names.append(path)
    return names


def ensure_subtree_files(problem_dir, subtrees_source=None):
    """
    Copies every subtree file behavior_tree.xml (transitively, through
    each newly-copied file's OWN <include>s too -- a subtree file can
    itself <include> another) references, but doesn't already have a
    local copy of, from `subtrees_source` (default:
    default_subtrees_source(problem_dir)) into problem_dir itself --
    making the problem directory fully self-contained, the same way
    this repo's own real problem directories already are (see this
    module's own docstring). A no-op (returns []) for a problem whose
    behavior_tree.xml has no <include> at all.

    Returns the list of filenames actually copied (for main()'s own
    reporting) -- a filename the problem directory already had a local
    copy of is left untouched (never overwritten -- a problem's own
    local copy, once made, is that problem's to hand-edit if it ever
    needs to diverge from the shared library).
    """
    if subtrees_source is None:
        subtrees_source = default_subtrees_source(problem_dir)
    main_xml = os.path.join(problem_dir, 'behavior_tree.xml')
    copied = []
    pending = collect_include_filenames(main_xml)
    seen = set()
    while pending:
        name = pending.pop()
        if name in seen:
            continue
        seen.add(name)
        local_path = os.path.join(problem_dir, name)
        if not os.path.isfile(local_path):
            source_path = os.path.join(subtrees_source, name)
            if not os.path.isfile(source_path):
                raise FileNotFoundError(
                    '{} references <include path="{}"/>, which is not already present in {} '
                    'and was not found in the subtrees source folder {} either.'.format(
                        main_xml, name, problem_dir, subtrees_source
                    )
                )
            shutil.copy(source_path, local_path)
            copied.append(name)
        # recurse into this (now-local) file's own <include>s, so a
        # subtree-of-a-subtree gets copied too, before assembly needs it.
        pending.extend(collect_include_filenames(local_path))
    return copied


def _collect_tree_registry(xml_path, registry, visited_files):
    """Populates registry ({tree_id: <BehaviorTree> element}) from
    xml_path's own <BehaviorTree ID="..."> elements plus every
    <include path="..."/> it names (resolved in xml_path's own
    directory -- see ensure_subtree_files, which must already have made
    every one of these locally available before this is called).
    visited_files (realpaths) makes re-including an already-visited
    file a silent no-op (a diamond -- two files both including a third)
    rather than a duplicate-ID error or infinite recursion, same
    tolerance bt_to_prolog.py's own _collect_tree_registry has."""
    real = os.path.realpath(xml_path)
    if real in visited_files:
        return
    visited_files.add(real)
    root = ET.parse(xml_path).getroot()
    for bt in root.findall('BehaviorTree'):
        tree_id = bt.attrib.get('ID')
        if not tree_id:
            continue
        if tree_id in registry:
            raise ValueError('Duplicate <BehaviorTree ID="{}"> -- already defined elsewhere.'.format(tree_id))
        registry[tree_id] = bt
    file_dir = os.path.dirname(os.path.abspath(xml_path))
    for name in collect_include_filenames(xml_path):
        _collect_tree_registry(os.path.join(file_dir, name), registry, visited_files)


def _apply_blackboard_remap(root_elem, remap, suffix):
    """
    Rewrites every "{key}" attribute value throughout root_elem's own
    subtree (itself plus every descendant) IN PLACE -- identical
    substitution rule to bt_to_prolog.py's own _apply_subtree_remap:

      - a "{key}" whose key IS in `remap` (the calling <SubTree>
        element's own attributes, minus ID/name) becomes remap[key]
        VERBATIM -- itself either a literal or another "{parent_key}"
        blackboard reference, so nested SubTrees compose correctly:
        this substitution runs, then the (already-substituted) result
        is what a doubly-nested SubTree's own remap sees.
      - a "{key}" NOT in `remap` is PRIVATE to this one SubTree
        INSTANCE (BT.cpp's own default for an un-remapped port) --
        renamed to a fresh, call-site-unique "{key__suffix}" so two
        instantiations of the same subtree (or two unrelated subtrees
        that happen to reuse a common local port name) never collide
        into the same blackboard key once flattened.
    """
    for elem in root_elem.iter():
        for attr_name, value in list(elem.attrib.items()):
            if not _is_blackboard_ref(value):
                continue
            key = _blackboard_key(value)
            if key in remap:
                elem.attrib[attr_name] = remap[key]
            else:
                elem.attrib[attr_name] = '{{{}__{}}}'.format(key, suffix)


def _expand_subtrees(elem, registry, expanding, suffix_counter):
    """
    Recursively replaces every <SubTree> element reachable from `elem`
    (elem itself included) with the (deep-copied, port-remapped,
    recursively-expanded) body of the tree it references -- returns the
    (possibly replacED) element the caller should use in `elem`'s own
    place. `expanding` (a list of tree IDs currently being expanded,
    innermost last) is this function's own self-inclusion guard,
    exactly like bt_to_prolog.py's var_pool.expanding.
    """
    if elem.tag == 'SubTree':
        subtree_id = elem.attrib.get('ID')
        if not subtree_id:
            raise ValueError('<SubTree> requires an ID="..." attribute.')
        if subtree_id not in registry:
            raise ValueError(
                '<SubTree ID="{}"> references an unknown tree -- no <BehaviorTree ID="{}"> '
                'was found in this file or any <include>d file.'.format(subtree_id, subtree_id)
            )
        if subtree_id in expanding:
            raise ValueError(
                '<SubTree ID="{}"> recursion detected ({} -> {}) -- a SubTree can never '
                '(directly or indirectly) include itself.'.format(subtree_id, ' -> '.join(expanding), subtree_id)
            )
        body_children = list(registry[subtree_id])
        if len(body_children) != 1:
            raise ValueError('<BehaviorTree ID="{}"> must have exactly one child.'.format(subtree_id))
        remap = {k: v for k, v in elem.attrib.items() if k not in ('ID', 'name')}
        suffix_counter[0] += 1
        clone = copy.deepcopy(body_children[0])
        _apply_blackboard_remap(clone, remap, 'sub{}'.format(suffix_counter[0]))
        expanding.append(subtree_id)
        try:
            return _expand_subtrees(clone, registry, expanding, suffix_counter)
        finally:
            expanding.pop()
    # not a SubTree itself -- recurse into (and possibly replace) each child in place.
    for i, child in enumerate(list(elem)):
        elem[i] = _expand_subtrees(child, registry, expanding, suffix_counter)
    return elem


def collect_unresolved_blackboard_refs(elem):
    """Every distinct "{key}" attribute value left anywhere in the
    (already fully SubTree-expanded) tree rooted at `elem` -- see this
    module's own docstring on why some of these are expected to remain
    (a query action's own dynamic output, or a genuine caller mistake)
    rather than something this script can resolve itself."""
    refs = set()
    for e in elem.iter():
        for value in e.attrib.values():
            if _is_blackboard_ref(value):
                refs.add(value)
    return refs


def resolve_tree(problem_dir, output_name='behavior_tree_resolved.xml'):
    """
    Assumes ensure_subtree_files(problem_dir) has already been run (see
    main() below, which always does both in order). Builds the full
    tree registry from behavior_tree.xml and every (now-local)
    <include>d file, expands every <SubTree> in the MAIN tree's own
    root composite/leaf, and writes the result -- a <root> with the
    SAME main_tree_to_execute-selected <BehaviorTree>, no <include>/
    <SubTree> elements anywhere in it -- to problem_dir/output_name.
    Returns (output_path, unresolved_refs) -- see
    collect_unresolved_blackboard_refs.
    """
    main_xml = os.path.join(problem_dir, 'behavior_tree.xml')
    tree = ET.parse(main_xml)
    root = tree.getroot()

    registry = {}
    _collect_tree_registry(main_xml, registry, set())

    main_id = root.attrib.get('main_tree_to_execute')
    bt_elements = root.findall('BehaviorTree')
    chosen = None
    for bt in bt_elements:
        if main_id is None or bt.attrib.get('ID') == main_id:
            chosen = bt
            break
    if chosen is None:
        raise ValueError('No <BehaviorTree> matching main_tree_to_execute="{}" found in {}'.format(main_id, main_xml))
    (root_node,) = list(chosen)

    suffix_counter = [0]
    expanded_root_node = _expand_subtrees(root_node, registry, [], suffix_counter)
    chosen[0] = expanded_root_node

    # Drop every <include> (no longer meaningful once flattened) and
    # every OTHER <BehaviorTree> (only the chosen, now fully-expanded
    # one is kept -- a resolved tree has nothing left to reference them).
    out_root = ET.Element('root', {k: v for k, v in root.attrib.items()})
    out_root.append(chosen)
    output_path = os.path.join(problem_dir, output_name)
    ET.ElementTree(out_root).write(output_path, encoding='unicode', xml_declaration=False)

    unresolved = collect_unresolved_blackboard_refs(chosen)
    return output_path, unresolved


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) not in (1, 2):
        print('Usage: python3 -m translator.resolve_subtrees <problem_dir> [subtrees_source_dir]', file=sys.stderr)
        return 2
    problem_dir = argv[0]
    subtrees_source = argv[1] if len(argv) == 2 else None
    copied = ensure_subtree_files(problem_dir, subtrees_source)
    if copied:
        print('Copied subtree file(s) into {}: {}'.format(problem_dir, ', '.join(sorted(copied))))
    output_path, unresolved = resolve_tree(problem_dir)
    print('Wrote {}'.format(output_path))
    if unresolved:
        print(
            'WARNING: {} unresolved blackboard reference(s) remain -- {} -- see this '
            'module\'s own docstring ("WHAT THIS SCRIPT DELIBERATELY DOES NOT DO") for '
            'why: either a query action\'s own dynamic output with nothing to substitute '
            'at translation time, or a port the calling tree never remapped at all.'.format(
                len(unresolved), ', '.join(sorted(unresolved))
            )
        )
    return 0


if __name__ == '__main__':
    sys.exit(main())
