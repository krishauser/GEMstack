# GEMstack Launch File Visualizer

## Usage


```python visualize_graph.py <launch_file.yaml> [OPTIONS]```

## Options

| Flag                  | Description                                                                                                   |
|-----------------------|---------------------------------------------------------------------------------------------------------------|
| `-g, --graph <PATH>`  | Path to `computation_graph.yaml` (default: `~/GEMstack/knowledge/defaults/computation_graph.yaml`)             |
| `-v, --variant <NAME>`| Specific variant to visualize (e.g. `sim`, `fake_sim`). Omit to render **all** variants.                     |
| `-o, --output <PATH>` | Output file or directory (default: `graph`). If a directory, generates one PNG per variant inside it.         |

## Examples

### Render all variants to a folder

```python visualize_graph.py fixed_route.yaml -o out/```

This produces:

-> out/fixed_route_base_vis.png

-> out/fixed_route_sim_vis.png







