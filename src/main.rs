use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

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

    /// Snap threshold for vertex/edge selection in discrete space
    #[arg(long, default_value_t = 84.0)]
    snap_threshold: f64,

    /// Search span when probing along surface normals (in voxel units)
    #[arg(long, default_value_t = 5.0)]
    search_span: f64,

    /// Prefer edges before vertices when snapping
    #[arg(long, default_value_t = false)]
    edge_first: bool,

    /// Minimum segment length for neighbor snapping (2 = adjacent pairs)
    #[arg(long, default_value_t = 2)]
    min_segment_len: i32,

    /// Neighbor snap gap (maximum per-axis difference to merge)
    #[arg(long, default_value_t = 2)]
    snap_gap: u8,
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
        } => {
            let (models, _) = tobj::load_obj(
                &input,
                &LoadOptions {
                    merge_identical_points: true,
                    triangulate: true,
                    ..Default::default()
                },
            )
            .unwrap();

            // Build per-mesh TriMeshes and also a merged mesh for AABB/scaling.
            let mut submeshes: Vec<TriMesh> = Vec::new();
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
            }
            let merged = merged.expect("OBJ contained no meshes");

            // TODO: allow translations and rotations
            let isometry = Isometry::default();

            let height = size.height() - 3;
            let aabb = merged.aabb(&isometry);
            let svo_aabb = if scale.auto {
                let scale = Vector::repeat(aabb.extents().max()).component_div(&aabb.extents());
                aabb.scaled_wrt_center(&scale)
                    .scaled_wrt_center(&Vector::repeat(2.0))
            } else {
                let extents = Vector::repeat(4.0 * (1 << height) as f64);
                Aabb::from_half_extents(aabb.center(), extents / scale.scale)
            };

            // Prepare material list per mesh; pad with last provided material if fewer than meshes.
            let mut mats: Vec<u64> = material.clone();
            if mats.is_empty() {
                mats.push(1971262921);
            }
            if mats.len() < submeshes.len() {
                let last = *mats.last().unwrap();
                mats.resize(submeshes.len(), last);
            }

            // Create a single multi-mesh SVO with per-mesh materials.
            let params = voxelization::VoxelizationParams {
                straightness_bias: straight.straightness_bias,
                snap_threshold: straight.snap_threshold,
                search_span: straight.search_span,
                inflate: 1.05,
                degenerate_area: 1e-6,
                edge_first: straight.edge_first,
                min_segment_len: straight.min_segment_len,
                snap_gap: straight.snap_gap,
            };
            let svo = voxelization::create_lods_multi(
                &isometry,
                &submeshes,
                &svo_aabb,
                Point::origin(),
                height,
                &mats,
                &params,
            );
            let bp = Blueprint::new(
                input
                    .clone()
                    .file_stem()
                    .unwrap()
                    .to_str()
                    .unwrap()
                    .to_string(),
                CoreInfo::from(size, r#type),
                mats[0],
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
