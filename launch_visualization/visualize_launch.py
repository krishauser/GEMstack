#!/usr/bin/env python3
"""
visualize_graph.py

Static visualization of a GEMstack launch’s active components (with implementations & args)
against the ground-truth computation graph as Graphviz PNGs.

Features:
  • Annotates description, mode, vehicle_interface, mission_execution/run
  • Green = pure source (no inputs)
  • Red   = pure sink   (no outputs)
  • Clusters for drive→perception, drive→planning, visualization
  • Pseudo-node “all” for unmatched inputs/outputs
  • If no variant specified, outputs one PNG per variant into an output folder
  • Filenames prefixed by the launch YAML basename
  • Safe lookups to avoid KeyError on unexpected keys
"""

import argparse
import os
import yaml
import json
from graphviz import Digraph

def load_yaml(path):
    base = os.path.dirname(os.path.abspath(path))
    class Loader(yaml.SafeLoader): pass
    def include(loader, node):
        return load_yaml(os.path.join(base, loader.construct_scalar(node)))
    def relpath(loader, node):
        return os.path.join(base, loader.construct_scalar(node))
    Loader.add_constructor('!include', include)
    Loader.add_constructor('!relative_path', relpath)
    with open(path, 'r') as f:
        return yaml.load(f, Loader)

def normalize_list(x):
    if x is None: return []
    if isinstance(x, list): return x
    return [x]

def collect_components(gt_graph):
    comps = {}
    for entry in gt_graph.get('components', []):
        for name, io in entry.items():
            comps[name] = {
                'inputs':  normalize_list(io.get('inputs')),
                'outputs': normalize_list(io.get('outputs')),
            }
    return comps

def deep_merge(a, b):
    for k, v in b.items():
        if k in a and isinstance(a[k], dict) and isinstance(v, dict):
            deep_merge(a[k], v)
        else:
            a[k] = v

def resolve_variant(launch, variant):
    if not variant:
        return launch
    vs = launch.get('variants', {})
    if variant not in vs:
        raise KeyError(f"Variant '{variant}' not found")
    merged = yaml.safe_load(yaml.dump(launch))
    deep_merge(merged, vs[variant])
    return merged

def apply_run_overrides(spec):
    run = spec.get('run', {})
    if 'description' in run:
        spec['description'] = run['description']
    if 'mode' in run:
        spec['mode'] = run['mode']
    if 'vehicle_interface' in run:
        spec['vehicle_interface'] = run['vehicle_interface']
    if 'mission_execution' in run:
        spec['mission_execution'] = run['mission_execution']
    if 'drive' in run:
        deep_merge(spec.setdefault('drive', {}), run['drive'])
    if 'visualization' in run:
        spec['visualization'] = run['visualization']

def gather_active(spec, comps_def):
    raw = set()
    def recurse(d):
        for k, v in d.items():
            if isinstance(v, str) or (isinstance(v, dict) and 'type' in v):
                raw.add(k)
            elif isinstance(v, dict):
                recurse(v)
    recurse(spec.get('drive', {}))
    recurse(spec.get('visualization', {}))
    return {c for c in raw if c in comps_def}

def collect_impls_and_args(spec, comps_def):
    impls = {}
    def recurse(d):
        for k, v in d.items():
            if isinstance(v, str) and k in comps_def:
                impls[k] = {'impl': v, 'args': None}
            elif isinstance(v, dict) and 'type' in v and k in comps_def:
                impls[k] = {'impl': v['type'], 'args': v.get('args')}
            elif isinstance(v, dict):
                recurse(v)
    recurse(spec.get('drive', {}))
    recurse(spec.get('visualization', {}))
    return impls

def get_drive_clusters(spec, active):
    clusters = {}
    for grp, cfg in spec.get('drive', {}).items():
        if isinstance(cfg, dict):
            comps = [c for c in cfg if c in active]
            if comps:
                clusters[grp] = comps
    return clusters

def _add_node(dot, name, comps_def, impls):
    inputs  = comps_def.get(name, {}).get('inputs', [])
    outputs = comps_def.get(name, {}).get('outputs', [])
    style = {}
    if not inputs:
        style = {'style': 'filled', 'fillcolor': 'lightgreen'}
    elif not outputs:
        style = {'style': 'filled', 'fillcolor': 'lightcoral'}
    impl = impls.get(name, {}).get('impl', '<none>')
    args = impls.get(name, {}).get('args')
    label = f"{name}\\n[{impl}]"
    if args is not None:
        label += "\\n" + json.dumps(args)
    dot.node(name, label, **style)

def build_static(comps_def, active, impls, spec):
    dot = Digraph(comment='Computation Graph')
    dot.attr(
        rankdir='LR',
        margin='1.0,0.5',
        nodesep='1.0',
        ranksep='1.0'
    )

    # Top annotation
    meta = []
    if 'description' in spec:
        meta.append(spec['description'])
    if 'mode' in spec:
        meta.append(f"mode: {spec['mode']}")
    vi = spec.get('vehicle_interface')
    if isinstance(vi, str):
        meta.append(f"vehicle_interface: {vi}")
    elif isinstance(vi, dict):
        meta.append(
            f"vehicle_interface: {vi.get('type')} "
            f"{json.dumps(vi.get('args')) if vi.get('args') else ''}"
        )
    if 'mission_execution' in spec:
        meta.append(f"mission_execution: {spec['mission_execution']}")
    dot.attr(label='\n'.join(meta) + '\n\n', labelloc='t', fontsize='14')

    # Pseudo-node 'all'
    unmatched_in = {
        inp for c in active
        for inp in comps_def.get(c, {}).get('inputs', [])
        if inp != 'all' and not any(
            inp in comps_def.get(p, {}).get('outputs', []) for p in active
        )
    }
    unmatched_out = {
        outp for c in active
        for outp in comps_def.get(c, {}).get('outputs', [])
        if not any(
            outp in comps_def.get(c2, {}).get('inputs', []) for c2 in active
        )
    }
    needs_all = bool(
        unmatched_in or unmatched_out or
        any('all' in comps_def.get(c, {}).get('inputs', []) for c in active)
    )
    if needs_all:
        dot.node('all', 'all', style='filled', fillcolor='lightblue')

    clustered = set()
    # drive clusters
    for grp, nodes in get_drive_clusters(spec, active).items():
        with dot.subgraph(name=f'cluster_{grp}') as c:
            c.attr(
                label=grp.capitalize(),
                style='rounded,filled',
                color='lightgrey',
                margin='0.5'
            )
            for comp in nodes:
                clustered.add(comp)
                _add_node(c, comp, comps_def, impls)

    # visualization cluster
    if 'visualization' in spec:
        with dot.subgraph(name='cluster_visualization') as c:
            c.attr(
                label='Visualization',
                style='rounded,filled',
                color='lightgrey',
                margin='0.5'
            )
            for comp in spec.get('visualization', {}):
                if comp in active:
                    clustered.add(comp)
                    _add_node(c, comp, comps_def, impls)

    # remaining nodes
    for comp in sorted(active):
        if comp in clustered:
            continue
        _add_node(dot, comp, comps_def, impls)

    # edges inputs→component
    for comp in active:
        for inp in comps_def.get(comp, {}).get('inputs', []):
            if inp == 'all' or inp in unmatched_in:
                dot.edge(inp, comp, label=inp)
            else:
                for prod in (
                    p for p in active
                    if inp in comps_def.get(p, {}).get('outputs', [])
                ):
                    dot.edge(prod, comp, label=inp)

    # edges unmatched outputs→all
    for comp in active:
        for outp in comps_def.get(comp, {}).get('outputs', []):
            if outp in unmatched_out:
                dot.edge(comp, 'all', label=outp)

    return dot

def main():
    parser = argparse.ArgumentParser(
        description="Static visualization of GEMstack computation graph"
    )
    parser.add_argument('launch', help="Path to launch YAML")
    parser.add_argument(
        '-g','--graph',
        default=os.path.expanduser(
            '../GEMstack/knowledge/defaults/computation_graph.yaml'
        )
    )
    parser.add_argument(
        '-v','--variant',
        help="Variant to render (omit to render all)"
    )
    parser.add_argument(
        '-o','--output',
        default='graph',
        help="Output file or folder"
    )
    args = parser.parse_args()

    gt = load_yaml(args.graph)
    launch = load_yaml(args.launch)
    comps_def = collect_components(gt)

    launch_prefix = os.path.splitext(os.path.basename(args.launch))[0]
    variants = [v for v in launch.get('variants', {}) if v != 'log_ros']
    to_render = [args.variant] if args.variant else ['base'] + variants

    multiple = os.path.isdir(args.output) or len(to_render) > 1
    if multiple:
        os.makedirs(args.output, exist_ok=True)

    for vn in to_render:
        spec = resolve_variant(launch, None if vn == 'base' else vn)
        apply_run_overrides(spec)
        active = gather_active(spec, comps_def)
        impls = collect_impls_and_args(spec, comps_def)
        dot = build_static(comps_def, active, impls, spec)

        filename = f"{launch_prefix}_{vn}_vis"
        if multiple:
            root = os.path.join(args.output, filename)
            ext = 'png'
        else:
            root = filename
            ext = args.output.split('.')[-1] if '.' in args.output else 'png'

        dot.format = ext
        out_path = dot.render(root, cleanup=True, view=not multiple)
        print(f"✔ Wrote {out_path}")

if __name__ == '__main__':
    main()
