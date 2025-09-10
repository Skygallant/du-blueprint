# du-blueprint

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

Use `--material` to assign DU honeycomb material IDs to meshes in the input `.obj`.

- Input type: numeric DU honeycomb IDs (e.g., `1971262921`).
- Repeat or comma-separate the flag to pass multiple IDs:
  - Repeated flags: `--material 111 --material 222 --material 333`
  - Comma list: `--material 111,222,333`
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
du-blueprint generate ... --material 1971262921 model.obj out.blueprint

# Per-mesh materials in order (first->third meshes)
du-blueprint generate ... --material 111,222,333 model.obj out.blueprint

# Equivalent using repeated flags
du-blueprint generate ... --material 111 --material 222 --material 333 model.obj out.blueprint

# If the OBJ has 5 meshes and you pass only 2 materials, mesh 3–5 use 222
du-blueprint generate ... --material 111,222 model.obj out.blueprint
```

## Straight-Line Controls

To favor straighter, more continuous voxel lines, the `generate` command supports several flags. These tune how boundary vertices are chosen and how neighboring offsets are snapped.

- `--straightness-bias <float>`
  - Adds a small penalty to off-axis candidates so axis-aligned runs are preferred.
  - Default: `0.0` (off). Try `0.5–2.0` for noticeable effect.
- `--edge-first`
  - Prefer snapping to triangle edges before vertices. Often yields longer straight runs on panels.
- `--snap-threshold <float>`
  - Discrete-space distance threshold for accepting vertex/edge snaps.
  - Default: `84.0`. Increase slightly (e.g., `90`) to be more permissive.
- `--search-span <float>`
  - Length of the probe along the surface direction when seeking a good offset (in voxel units).
  - Default: `5.0`.
- `--min-segment-len <int>`
  - Minimum segment length considered during neighbor snapping; `2` enables adjacent-pair smoothing.
  - Default: `2`.
- `--snap-gap <u8>`
  - Maximum per-axis difference for two neighboring offsets to be merged by snapping.
  - Default: `2`.

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
  --straightness-bias 0.5 --snap-gap 1 \
  my_model.obj my_blueprint.blueprint
```

Generate with strong straightening for panel-like models:
```
du-blueprint generate \
  --auto --type=dynamic --size=l \
  --edge-first --straightness-bias 2.0 --snap-threshold 90 --snap-gap 3 --min-segment-len 3 \
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
