# du-blueprint
# (this version is currently not working)

A tool for generating Dual Universe blueprint files from models.

Warning: The larger core sizes (above XXL) require a pretty beefy machine to generate. An XXXL
version of 'suzanne` (the Blender monkey) peaks at ~16GB memory usage and generates a 750MB
blueprint file. Expect an 8x bigger number for each size above that.

Quick start:
```
du-blueprint generate --auto --type=dynamic --size=l my_model.obj my_blueprint.blueprint
```

The only supported format at the moment is `.obj`. For good results, use a manifold mesh.
For best results, take into account in game voxel limitations when making your model.

This tool is very much in the "make it work" stage of development. There are a lot of
easy improvements that can be made, so PRs are welcome. Just let me know if you are working
on something beforehand.

Right now the voxelization process is pretty naive and unoptimized, and just throws threads
at the problem.

## Materials

Use `--material=<id>` to assign DU honeycomb material IDs to meshes in the input `.obj`.

- Input type: numeric DU honeycomb IDs (e.g., `1971262921`).
- Repeat or comma-separate the flag to pass multiple IDs (use `=`):
  - Repeated flags: `--material=111 --material=222 --material=333`
  - Comma list: `--material=111,222,333`
- Assignment rule: materials apply sequentially to the meshes as they appear in the OBJ
  (i.e., first ID -> first mesh, second ID -> second mesh, etc.).
- Fewer IDs than meshes: the last provided ID is reused for all remaining meshes.
- More IDs than meshes: extra IDs are ignored.
- Default: if not provided, a single default material is used (`1971262921`).

Notes
- The exporter adds DU’s debug material in slot 1; per‑mesh materials begin at slot 2 internally.
- OBJ mesh order depends on how your DCC splits/exports mesh objects and groups.

Examples
```
# Single material for all meshes
du-blueprint generate ... --material=1971262921 model.obj out.blueprint

# Per-mesh materials in order (first->third meshes)
du-blueprint generate ... --material=111,222,333 model.obj out.blueprint

# Equivalent using repeated flags
du-blueprint generate ... --material=111 --material=222 --material=333 model.obj out.blueprint

# If the OBJ has 5 meshes and you pass only 2 materials, mesh 3–5 use 222
du-blueprint generate ... --material=111,222 model.obj out.blueprint
```

## Straight-Line Controls

To favor straighter, more continuous voxel lines, the `generate` command supports several flags. These tune how boundary vertices are chosen and how neighboring offsets are snapped.

- `--straightness-bias=<float>`
  - Adds a small penalty to off-axis candidates so axis-aligned runs are preferred.
  - Default: `0.0` (off). Try `0.5–2.0` for noticeable effect.
- `--angle-straightness-bias=<float>`
  - Prefers candidates aligned with local mesh edge directions (keeps lines straight along angled panels).
  - Default: `0.0` (off). Try `0.5–2.0` for a noticeable effect.
- `--edge-first=<bool>` (or just `--edge-first` to enable)
  - Prefer snapping to triangle edges before vertices. Often yields longer straight runs on panels.
- `--snap-threshold=<float>`
  - Discrete-space distance threshold for accepting vertex/edge snaps.
  - Default: `84.0`. Increase slightly (e.g., `90`) to be more permissive.
- `--search-span=<float>`
  - Length of the probe along the surface direction when seeking a good offset (in voxel units).
  - Default: `5.0`.
- `--min-segment-len=<int>`
  - Minimum segment length considered during neighbor snapping; `2` enables adjacent-pair smoothing.
  - Default: `2`.
- `--snap-gap=<u8>`
  - Maximum per-axis difference for two neighboring offsets to be merged by snapping.
  - Default: `2`.
- `--direction-bins=<6|18|26>`
  - Controls which directions the continuity pass snaps along:
    - `6`: axis-aligned only (±X, ±Y, ±Z)
    - `18`: axes + face diagonals (e.g., X±Y, Y±Z, X±Z)
    - `26`: axes + face diagonals + space diagonals (±X±Y±Z)
  - Default: `6`.
- `--edge-preserve-deg=<float>`
  - Preserves mesh-defined edges: smoothing is only applied when neighboring face normals differ by at most this angle (degrees).
  - Higher values smooth more aggressively across subtle edges; lower values keep edges crisper. Default: `25`.
- `--inflate=<float>`
  - Inflates the collision AABB used during voxelization (robustness for thin shells). Lower it (e.g., `1.01`) if thin panels over-smooth, increase slightly if gaps appear.
  - Default: `1.05`.
- `--panel-thin-factor=<float>`
  - Multiplier of voxel size used as the surface distance tolerance to detect thin panels (default `0.35`). Smaller values make detection stricter.

### Resolution Control

- `--min-voxel-size=<float>`
  - Clamps the voxelization LOD so that the effective world‑space voxel size is at least this value. Helps eliminate sub‑voxel noise and tiny features by coarsening resolution as needed. Optional; when omitted, resolution follows core size.

## UV Material Mapping

You can assign DU honeycomb materials from an image texture via the OBJ’s UVs. This enables multiple materials on a single OBJ mesh (per‑voxel labeling along boundaries), not just per‑mesh fills.

### Requirements

- The OBJ must contain UVs (`vt`).
- Provide a texture via `--uv-texture=<path>` or export an MTL with a `map_Kd` entry (auto‑detected). If neither is present, the tool falls back to Kd colors (see below).

### Flags

- `--uv-texture=<path>`
  - PNG/JPEG sampled in UV space to pick material colors.
  - If omitted, the tool tries to auto‑detect a diffuse texture from the OBJ/MTL (`map_Kd`).
- `--uv-prompt`
  - Interactively maps each detected color to a DU honeycomb ID.
  - Prompts show the color as hex plus an approximate human‑readable name (nearest in a CSS‑like palette) to speed up selection.

### How it works

1) A small palette of dominant colors is detected (capped at ~32).
2) When `--uv-prompt` is set, you’re asked for a DU honeycomb ID for each color (press Enter to skip a color).
3) During voxelization:
   - Interior leaves get a per‑mesh default fill.
   - Significant boundary cells are projected to triangle UVs, the texture is sampled, the color is mapped to a DU ID, and the cell’s material label is overridden.
4) Each chunk’s material mapper includes Debug1, per‑mesh default fills (set to the majority DU seen for that mesh), and all UV colors used (up to the per‑chunk label limit of 254).

### Kd (solid‑color) fallback

If no texture is available, but the MTL has diffuse colors (`Kd`) and you pass `--uv-prompt`, the tool:

- Collects Kd colors for the materials actually referenced by your submeshes.
- Prompts for DU IDs for those colors.
- Applies the result as per‑mesh materials (no per‑voxel UV detail).

### Examples

- Use a provided texture (recommended for multi‑material within one mesh):
```
du-blueprint generate \
  --uv-texture=albedo_or_mask.png --uv-prompt \
  --type=dynamic --size=l \
  input.obj output.blueprint
```

- Auto‑detect texture from OBJ/MTL:
```
du-blueprint generate --uv-prompt --type=dynamic --size=l input.obj output.blueprint
```

- Kd fallback (no texture):
```
du-blueprint generate --uv-prompt --type=dynamic --size=l input.obj output.blueprint
```

### Tips & Limits

- Keep the palette clean and small (we cap detection at ~32 colors). Very busy textures can exhaust per‑chunk label slots (max 254 + Debug1).
- The UV method assigns materials only on significant boundaries; interiors remain with the mesh default. This preserves solid regions and performance.
- Smoothing/edge‑guard settings still apply at seams. If you see “teeth” or blocky seams, narrow the edge guard and increase interior face smoothing (see Straight‑Line Controls), or use a tighter seam gap.
- If nothing prompts with `--uv-prompt`, make sure a texture is provided (or auto‑detected) and that the OBJ has UVs.

### Fine Edge Controls

- `--edge-line-smoothing=<bool>`
  - Enables an extra pass that smooths offsets along detected edge tangents (the line defined by adjacent face normals). Keeps fine edges angular but removes local jaggies.
  - Default: `true` (use `--edge-line-smoothing=false` to disable).
- `--edge-snap-gap=<u8>`
  - Snap tolerance used specifically for edge-line smoothing; typically stricter than `--snap-gap` to avoid bloating edges. Default: `1`.
- `--edge-min-segment-len=<int>`
  - Minimum run length to apply edge-line smoothing. Default: `3`.
- `--offset-scale=<float>`
  - Controls sub-voxel discretization of offsets. Higher values (e.g., `126`) can reduce quantization jaggies on very fine edges, at the cost of slightly more aggressive snapping. Default: `84.0`.
- `--edge-protect-radius=<int>`
  - Number of voxel cells to protect around detected edges where face smoothing is disabled. Helps keep edges pristine while smoothing the interior. Default: `1`.
- `--face-snap-gap=<u8>`
  - Interior face smoothing snap tolerance. Default: `3`.
- `--face-min-segment-len=<int>`
  - Interior face smoothing minimum run length. Default: `3`.
- `--face-smoothing-passes=<u8>`
  - Number of interior face smoothing passes. Default: `1`.

Notes:
- Higher straightening values may remove small features or introduce slight faceting on curves.
- If you see over-snapping on curved areas, lower `--straightness-bias` or `--snap-gap`.

### Example Presets

- Subtle Nudge (keep detail, reduce micro-wiggles)
  - `--straightness-bias 0.5 --snap-gap 1`

- Crisp Panels (favor long straight runs on flat surfaces)
  - `--edge-first --straightness-bias 1.5 --snap-gap 2 --min-segment-len 2`

- Strong Straightening (aggressive straight lines, may lose tiny features)
  - `--edge-first --straightness-bias 2.0 --snap-threshold 90 --snap-gap 3 --min-segment-len 3`

### Command Examples

Generate with subtle straightening:
```
du-blueprint generate \
  --auto --type=dynamic --size=l \
  --straightness-bias=0.5 --snap-gap=1 \
  my_model.obj my_blueprint.blueprint
```

Generate with strong straightening for panel-like models:
```
du-blueprint generate \
  --auto --type=dynamic --size=l \
  --edge-first --straightness-bias=1.0 --angle-straightness-bias=1.5 \
  --snap-threshold=90 --snap-gap=3 --min-segment-len=3 --direction-bins=18 \
  --edge-preserve-deg=20 --inflate=1.01 \
  my_model.obj my_blueprint.blueprint
```

## FAQ

### Q. Why does my construct have a weird orientation?

Make sure the model orientation matches DU expections. DU is Z-up and Y-forward; many models
are Y-up.

### Q. Why does my construct have weird floating boxes?

You tried to import a non-manifold mesh. The voxelizer tries it's best to account
for this, but it isn't perfect.

### Q. How can I make my mesh manifold?

Blender. Search for a tutorial on the "3D-Print Toolbox" addon; this is a common problem
with 3D printing.
