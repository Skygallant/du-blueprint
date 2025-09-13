use std::collections::HashMap;
use std::fs::File;
use std::io::{self, Write};
use std::path::{Path, PathBuf};

use base64::Engine;
use parry3d_f64::bounding_volume::Aabb;
use parry3d_f64::math::{Isometry, Point, Vector};
use parry3d_f64::shape::{TriMesh, TriMeshFlags};
use squarion::{AggregateMetadata, Deserialize, VoxelCellData};
use tobj::LoadOptions;

mod blueprint;
mod squarion;
mod svo;
mod voxelization;

use crate::blueprint::*;

use clap::{Args, Parser, Subcommand};
use palette::{IntoColor, Lab, Srgb};

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Args)]
#[group(required = true, multiple = false)]
struct ScaleInfo {
    /// Automatically scale model to fill core
    #[arg(short, long)]
    auto: bool,

    #[arg(long, default_value_t = 1.0)]
    scale: f64,
}

#[derive(Args, Clone)]
struct StraightnessArgs {
    /// Bias towards straight, axis-aligned runs (0 = off)
    #[arg(long, default_value_t = 0.0)]
    straightness_bias: f64,

    /// Bias towards straight runs along local edge directions (0 = off)
    #[arg(long, default_value_t = 0.0)]
    angle_straightness_bias: f64,

    /// Snap threshold for vertex/edge selection in discrete space
    #[arg(long, default_value_t = 84.0)]
    snap_threshold: f64,

    /// Search span when probing along surface normals (in voxel units)
    #[arg(long, default_value_t = 5.0)]
    search_span: f64,

    /// Prefer edges before vertices when snapping
    /// Accepts `--edge-first`, `--edge-first=true`, or `--edge-first=false`.
    #[arg(
        long,
        default_value_t = false,
        num_args = 0..=1,
        default_missing_value = "true",
        value_parser = clap::builder::BoolishValueParser::new()
    )]
    edge_first: bool,

    /// Minimum segment length for neighbor snapping (2 = adjacent pairs)
    #[arg(long, default_value_t = 2)]
    min_segment_len: i32,

    /// Neighbor snap gap (maximum per-axis difference to merge)
    #[arg(long, default_value_t = 2)]
    snap_gap: u8,

    /// Direction bins for snapping continuity: 6, 18, or 26
    #[arg(long, default_value_t = 6)]
    direction_bins: u8,

    /// Preserve mesh feature edges: max face-normal difference (degrees) to allow smoothing
    #[arg(long, default_value_t = 25.0)]
    edge_preserve_deg: f64,

    /// Inflate collision AABBs slightly during voxelization (robustness for thin shells)
    #[arg(long, default_value_t = 1.05)]
    inflate: f64,

    /// Panel mode: always place materials on boundary voxels (better for thin panels)
    #[arg(
        long,
        default_value_t = false,
        num_args = 0..=1,
        default_missing_value = "true",
        value_parser = clap::builder::BoolishValueParser::new()
    )]
    panel_mode: bool,

    /// Enable edge-line smoothing along detected edge tangents
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    edge_line_smoothing: bool,

    /// Snap gap for edge-line smoothing (stricter than normal smoothing)
    #[arg(long, default_value_t = 1)]
    edge_snap_gap: u8,

    /// Minimum run length for edge-line smoothing
    #[arg(long, default_value_t = 3)]
    edge_min_segment_len: i32,

    /// Offset discretization scale (higher gives finer sub-voxel resolution)
    #[arg(long, default_value_t = 84.0)]
    offset_scale: f64,

    /// Edge protection radius (cells) where face smoothing is disabled around detected edges
    #[arg(long, default_value_t = 1)]
    edge_protect_radius: i32,

    /// Face-only smoothing snap gap (interior faces). Higher smooths more; avoid crossing edges.
    #[arg(long, default_value_t = 3)]
    face_snap_gap: u8,

    /// Face-only smoothing minimum segment length
    #[arg(long, default_value_t = 3)]
    face_min_segment_len: i32,

    /// Number of face-only smoothing passes
    #[arg(long, default_value_t = 1)]
    face_smoothing_passes: u8,

    /// Minimum world-space voxel size; clamps LOD to avoid sub-voxel features
    #[arg(long)]
    min_voxel_size: Option<f64>,
}

#[derive(Subcommand)]
enum Commands {
    /// Generate a blueprint file from an obj file.
    Generate {
        /// Input obj file name
        input: PathBuf,

        /// Output blueprint file name
        output: PathBuf,

        #[arg(short, long, value_enum)]
        r#type: CoreType,

        #[arg(short, long, value_enum)]
        size: CoreSize,

        /// Voxel material ID(s). Repeat or comma-separate to assign per OBJ mesh.
        /// First applies to first mesh, second to second mesh, etc.
        #[arg(short, long, value_delimiter = ',', default_values_t = vec![1971262921])]
        material: Vec<u64>,

        #[command(flatten)]
        scale: ScaleInfo,

        #[command(flatten)]
        straight: StraightnessArgs,

        /// Optional UV texture (albedo or mask) to drive per-voxel materials
        #[arg(long)]
        uv_texture: Option<PathBuf>,

        /// Prompt to map detected texture colors to DU honeycomb IDs
        #[arg(long, default_value_t = false)]
        uv_prompt: bool,
    },
    /// Parse a base64 voxel chunk and dump the result to stdout
    ParseVoxel {
        // Input base64
        b64: String,
    },
    /// Parse a base64 meta chunk and dump the result to stdout
    ParseMeta {
        // Input base64
        b64: String,
    },
}

fn main() {
    let cli = Cli::parse();
    match cli.command {
        Commands::Generate {
            input,
            output,
            size,
            r#type,
            material,
            scale,
            straight,
            uv_texture,
            uv_prompt,
        } => {
            // Helper: try to detect a diffuse texture from the referenced MTL
            fn detect_mtl_diffuse(obj_path: &Path) -> Option<PathBuf> {
                let obj_dir = obj_path.parent().unwrap_or(Path::new("."));
                let content = std::fs::read_to_string(obj_path).ok()?;
                let mut mtl_rel: Option<PathBuf> = None;
                for line in content.lines() {
                    let l = line.trim();
                    if l.starts_with("map_Kd ") {
                        let p = l.split_whitespace().skip(1).collect::<Vec<_>>().join(" ");
                        if !p.is_empty() { return Some(obj_dir.join(p)); }
                    }
                    if l.starts_with("mtllib ") && mtl_rel.is_none() {
                        let p = l.split_whitespace().skip(1).collect::<Vec<_>>().join(" ");
                        if !p.is_empty() { mtl_rel = Some(PathBuf::from(p)); }
                    }
                }
                if let Some(mtl_rel) = mtl_rel {
                    let mtl_path = obj_dir.join(mtl_rel);
                    if let Ok(mtl) = std::fs::read_to_string(&mtl_path) {
                        for line in mtl.lines() {
                            let l = line.trim();
                            if l.starts_with("map_Kd ") {
                                let p = l.split_whitespace().skip(1).collect::<Vec<_>>().join(" ");
                                if !p.is_empty() { return Some(obj_dir.join(p)); }
                            }
                        }
                    }
                }
                None
            }
            let (models, materials_res) = tobj::load_obj(
                &input,
                &LoadOptions {
                    merge_identical_points: true,
                    triangulate: true,
                    ..Default::default()
                },
            )
            .unwrap();
            let materials = materials_res.unwrap_or_default();

            // Build per-mesh TriMeshes and also a merged mesh for AABB/scaling.
            let mut submeshes: Vec<TriMesh> = Vec::new();
            let mut submesh_uv_faces: Vec<Vec<[[f32; 2]; 3]>> = Vec::new();
            let mut submesh_mat_ids: Vec<Option<usize>> = Vec::new();
            let mut merged: Option<TriMesh> = None;
            for model in models {
                let vertices = Vec::from_iter(
                    model
                        .mesh
                        .positions
                        .chunks_exact(3)
                        .map(|x| Point::from_slice(&[x[0] as f64, x[1] as f64, x[2] as f64])),
                );
                let indices = Vec::from_iter(
                    model
                        .mesh
                        .indices
                        .chunks_exact(3)
                        .map(|c| [c[0], c[1], c[2]]),
                );
                // Per-face UVs aligned with the triangle indices
                let texcoords = &model.mesh.texcoords;
                // tobj::Mesh::texcoord_indices may be Option or empty Vec depending on loader; normalize to Option<&[u32]>
                let tindices: Option<&[u32]> = if model.mesh.texcoord_indices.len() > 0 {
                    Some(&model.mesh.texcoord_indices)
                } else {
                    None
                };
                let mut uv_faces: Vec<[[f32; 2]; 3]> = Vec::new();
                for (fi, tri) in model.mesh.indices.chunks_exact(3).enumerate() {
                    let (u0, v0, u1, v1, u2, v2) = if let Some(ti) = tindices {
                        let t0 = ti[3 * fi] as usize;
                        let t1 = ti[3 * fi + 1] as usize;
                        let t2 = ti[3 * fi + 2] as usize;
                        (
                            *texcoords.get(2 * t0).unwrap_or(&0.0),
                            *texcoords.get(2 * t0 + 1).unwrap_or(&0.0),
                            *texcoords.get(2 * t1).unwrap_or(&0.0),
                            *texcoords.get(2 * t1 + 1).unwrap_or(&0.0),
                            *texcoords.get(2 * t2).unwrap_or(&0.0),
                            *texcoords.get(2 * t2 + 1).unwrap_or(&0.0),
                        )
                    } else {
                        let vi0 = tri[0] as usize;
                        let vi1 = tri[1] as usize;
                        let vi2 = tri[2] as usize;
                        (
                            *texcoords.get(2 * vi0).unwrap_or(&0.0),
                            *texcoords.get(2 * vi0 + 1).unwrap_or(&0.0),
                            *texcoords.get(2 * vi1).unwrap_or(&0.0),
                            *texcoords.get(2 * vi1 + 1).unwrap_or(&0.0),
                            *texcoords.get(2 * vi2).unwrap_or(&0.0),
                            *texcoords.get(2 * vi2 + 1).unwrap_or(&0.0),
                        )
                    };
                    uv_faces.push([[u0, v0], [u1, v1], [u2, v2]]);
                }
                let mut sub_mesh = TriMesh::new(vertices, indices);
                sub_mesh
                    .set_flags(
                        TriMeshFlags::ORIENTED
                            | TriMeshFlags::FIX_INTERNAL_EDGES
                            | TriMeshFlags::DELETE_DEGENERATE_TRIANGLES,
                    )
                    .unwrap();
                if let Some(m) = &mut merged {
                    m.append(&sub_mesh);
                } else {
                    merged = Some(sub_mesh.clone());
                }
                submeshes.push(sub_mesh);
                submesh_uv_faces.push(uv_faces);
                submesh_mat_ids.push(model.mesh.material_id);
            }
            let merged = merged.expect("OBJ contained no meshes");

            // TODO: allow translations and rotations
            let isometry = Isometry::default();

            let mut height = size.height() - 3;
            let aabb = merged.aabb(&isometry);
            let svo_aabb = if scale.auto {
                let scale = Vector::repeat(aabb.extents().max()).component_div(&aabb.extents());
                aabb.scaled_wrt_center(&scale)
                    .scaled_wrt_center(&Vector::repeat(2.0))
            } else {
                let extents = Vector::repeat(4.0 * (1 << height) as f64);
                Aabb::from_half_extents(aabb.center(), extents / scale.scale)
            };

            // If requested, clamp the SVO height so that the effective voxel size is not smaller
            // than the given world-space minimum. Approx: voxel_size ~= svo_aabb.extents().x / (2^height * 32)
            if let Some(min_vs) = straight.min_voxel_size {
                let ext = svo_aabb.extents().x.max(1e-9);
                let max_leaves = (ext / (min_vs * 32.0)).floor();
                if max_leaves.is_finite() && max_leaves >= 1.0 {
                    let cap = max_leaves.log2().floor().max(0.0) as usize;
                    if height > cap { height = cap; }
                } else {
                    // If min_vs is very large, force coarsest height 0
                    height = 0;
                }
            }

            // Create a single multi-mesh SVO with per-mesh materials.
            let params = voxelization::VoxelizationParams {
                straightness_bias: straight.straightness_bias,
                angle_straightness_bias: straight.angle_straightness_bias,
                snap_threshold: straight.snap_threshold,
                search_span: straight.search_span,
                inflate: straight.inflate,
                degenerate_area: 1e-6,
                edge_first: straight.edge_first,
                min_segment_len: straight.min_segment_len,
                snap_gap: straight.snap_gap,
                direction_bins: straight.direction_bins,
                edge_preserve_deg: straight.edge_preserve_deg,
                panel_mode: straight.panel_mode,
                edge_line_smoothing: straight.edge_line_smoothing,
                edge_snap_gap: straight.edge_snap_gap,
                edge_min_segment_len: straight.edge_min_segment_len,
                offset_scale: straight.offset_scale,
                edge_protect_radius: straight.edge_protect_radius,
                face_snap_gap: straight.face_snap_gap,
                face_min_segment_len: straight.face_min_segment_len,
                face_smoothing_passes: straight.face_smoothing_passes,
                default_offset_surface: true,
                seam_harmonize_gap: 1,
            };
            // Use provided --uv-texture if any; otherwise try MTL detection
            let uv_tex_path_opt = uv_texture.clone().or_else(|| detect_mtl_diffuse(&input));

            let (svo, fill_material_id) = if let Some(tex_path) = &uv_tex_path_opt {
                // Load texture and optionally prompt for color→DU mapping
                let img = image::open(tex_path).expect("Failed to open --uv-texture").to_rgba8();
                let (w, h) = img.dimensions();
                // Build a small palette of prominent colors
                let mut hist: HashMap<[u8; 3], u64> = HashMap::new();
                for p in img.pixels() {
                    let rgb = [p[0], p[1], p[2]];
                    *hist.entry(rgb).or_insert(0) += 1;
                }
                let mut palette: Vec<([u8; 3], u64)> = hist.into_iter().collect();
                palette.sort_by_key(|(_, c)| std::cmp::Reverse(*c));
                if palette.len() > 32 {
                    palette.truncate(32);
                }
                let mut color_to_du: HashMap<[u8; 3], u64> = HashMap::new();
                if uv_prompt {
                        println!("Map detected colors to DU honeycomb IDs (Enter to skip):");
                        for (rgb, cnt) in &palette {
                            let (name, _) = nearest_color_name(*rgb);
                            println!("  #{:02x}{:02x}{:02x} ≈ {} ({} px)", rgb[0], rgb[1], rgb[2], name, cnt);
                            print!("    DU ID: ");
                            let _ = io::stdout().flush();
                            let mut line = String::new();
                            io::stdin().read_line(&mut line).ok();
                            if let Ok(id) = line.trim().parse::<u64>() {
                                color_to_du.insert(*rgb, id);
                            }
                        }
                    }
                let fill = color_to_du.values().next().copied().unwrap_or(1971262921u64);
                let sampler = voxelization::UvSampler::new(img, w as usize, h as usize, color_to_du);
                let svo = voxelization::create_lods_multi_uv(
                    &isometry,
                    &submeshes,
                    &submesh_uv_faces,
                    &svo_aabb,
                    Point::origin(),
                    height,
                    &params,
                    sampler,
                );
                (svo, fill)
            } else {
                // Fallback to per-mesh materials; if --uv-prompt is set and MTL has Kd colors,
                // prompt to map each detected Kd color to a DU honeycomb id.
                let mut mats: Vec<u64> = Vec::new();
                if uv_prompt && !materials.is_empty() {
                    // Collect Kd colors actually referenced by submeshes
                    let mut kd_set: HashMap<[u8; 3], Vec<usize>> = HashMap::new(); // color -> list of mat_ids
                    for mid_opt in &submesh_mat_ids {
                        if let Some(mid) = *mid_opt {
                            if let Some(mat) = materials.get(mid) {
                                let kd_arr = mat.diffuse.unwrap_or([0.5, 0.5, 0.5]);
                                let rgb = [
                                    (kd_arr[0].clamp(0.0, 1.0) * 255.0).round() as u8,
                                    (kd_arr[1].clamp(0.0, 1.0) * 255.0).round() as u8,
                                    (kd_arr[2].clamp(0.0, 1.0) * 255.0).round() as u8,
                                ];
                                kd_set.entry(rgb).or_default().push(mid);
                            }
                        }
                    }
                    // Prompt mapping color -> DU id
                    let mut color_to_du: HashMap<[u8; 3], u64> = HashMap::new();
                    if !kd_set.is_empty() {
                        println!("Map detected MTL Kd colors to DU honeycomb IDs (Enter to skip):");
                        for (rgb, _) in kd_set.iter() {
                            let (name, _) = nearest_color_name(*rgb);
                            println!("  #{:02x}{:02x}{:02x} ≈ {}", rgb[0], rgb[1], rgb[2], name);
                            print!("    DU ID: ");
                            let _ = io::stdout().flush();
                            let mut line = String::new();
                            io::stdin().read_line(&mut line).ok();
                            if let Ok(id) = line.trim().parse::<u64>() {
                                color_to_du.insert(*rgb, id);
                            }
                        }
                    }
                    // Build mats per submesh from mapping (fallback to first provided or default)
                    let fallback = material.first().copied().unwrap_or(1971262921);
                    for mid_opt in &submesh_mat_ids {
                        let du = if let Some(mid) = *mid_opt {
                            if let Some(mat) = materials.get(mid) {
                                let kd_arr = mat.diffuse.unwrap_or([0.5, 0.5, 0.5]);
                                let rgb = [
                                    (kd_arr[0].clamp(0.0, 1.0) * 255.0).round() as u8,
                                    (kd_arr[1].clamp(0.0, 1.0) * 255.0).round() as u8,
                                    (kd_arr[2].clamp(0.0, 1.0) * 255.0).round() as u8,
                                ];
                                color_to_du.get(&rgb).copied().unwrap_or(fallback)
                            } else { fallback }
                        } else { fallback };
                        mats.push(du);
                    }
                    if mats.is_empty() {
                        mats = material.clone();
                    }
                } else {
                    mats = material.clone();
                }
                if mats.is_empty() { mats.push(1971262921); }
                if mats.len() < submeshes.len() {
                    let last = *mats.last().unwrap();
                    mats.resize(submeshes.len(), last);
                }
                let svo = voxelization::create_lods_multi(
                    &isometry,
                    &submeshes,
                    &svo_aabb,
                    Point::origin(),
                    height,
                    &mats,
                    &params,
                );
                (svo, mats[0])
            };
            let bp = Blueprint::new(
                input
                    .clone()
                    .file_stem()
                    .unwrap()
                    .to_str()
                    .unwrap()
                    .to_string(),
                CoreInfo::from(size, r#type),
                fill_material_id,
                svo,
            );
            File::create(output)
                .unwrap()
                .write(
                    bp.to_construct_json_result()
                        .unwrap()
                        .to_string()
                        .as_bytes(),
                )
                .unwrap();
        }
        Commands::ParseVoxel { b64 } => {
            let bytes = base64::prelude::BASE64_STANDARD.decode(b64).unwrap();
            let voxel = VoxelCellData::decompress(&bytes);
            match voxel {
                Ok(voxel) => println!("{:#?}", serde_json::to_string(&voxel)),
                Err(_) => println!("{:#?}", voxel),
            }
        }
        Commands::ParseMeta { b64 } => {
            let bytes = base64::prelude::BASE64_STANDARD.decode(b64).unwrap();
            let meta = AggregateMetadata::decompress(&bytes);
            match meta {
                Ok(meta) => println!("{:#?}", serde_json::to_string(&meta)),
                Err(_) => println!("{:#?}", meta),
            }
        }
    }
}
// Minimal CSS-like palette for nearest-name lookup
static CSS_PALETTE: &[(&str, [u8; 3])] = &[
    ("black", [0, 0, 0]),
    ("white", [255, 255, 255]),
    ("gray", [128, 128, 128]),
    ("silver", [192, 192, 192]),
    ("red", [255, 0, 0]),
    ("maroon", [128, 0, 0]),
    ("yellow", [255, 255, 0]),
    ("olive", [128, 128, 0]),
    ("lime", [0, 255, 0]),
    ("green", [0, 128, 0]),
    ("aqua", [0, 255, 255]),
    ("teal", [0, 128, 128]),
    ("blue", [0, 0, 255]),
    ("navy", [0, 0, 128]),
    ("fuchsia", [255, 0, 255]),
    ("purple", [128, 0, 128]),
    ("orange", [255, 165, 0]),
    ("brown", [165, 42, 42]),
    ("tan", [210, 180, 140]),
    ("lightslategray", [119, 136, 153]),
    ("darkslategray", [47, 79, 79]),
    ("lightgray", [211, 211, 211]),
    ("darkgray", [169, 169, 169]),
    ("crimson", [220, 20, 60]),
    ("indianred", [205, 92, 92]),
    ("tomato", [255, 99, 71]),
    ("salmon", [250, 128, 114]),
    ("gold", [255, 215, 0]),
    ("khaki", [240, 230, 140]),
    ("greenyellow", [173, 255, 47]),
    ("seagreen", [46, 139, 87]),
    ("cyan", [0, 255, 255]),
    ("deepskyblue", [0, 191, 255]),
    ("royalblue", [65, 105, 225]),
    ("slateblue", [106, 90, 205]),
    ("magenta", [255, 0, 255]),
    ("orchid", [218, 112, 214]),
    ("sienna", [160, 82, 45]),
];

fn nearest_color_name(rgb: [u8; 3]) -> (&'static str, f32) {
    let base_lab: Lab = Srgb::new(
        rgb[0] as f32 / 255.0,
        rgb[1] as f32 / 255.0,
        rgb[2] as f32 / 255.0,
    )
    .into_color();
    let mut best_name = "unknown";
    let mut best_d = f32::MAX;
    for (name, c) in CSS_PALETTE.iter() {
        let named_lab: Lab = Srgb::new(c[0] as f32 / 255.0, c[1] as f32 / 255.0, c[2] as f32 / 255.0).into_color();
        let dl = base_lab.l - named_lab.l;
        let da = base_lab.a - named_lab.a;
        let db = base_lab.b - named_lab.b;
        let d = (dl * dl + da * da + db * db).sqrt();
        if d < best_d {
            best_d = d;
            best_name = name;
        }
    }
    (best_name, best_d)
}
