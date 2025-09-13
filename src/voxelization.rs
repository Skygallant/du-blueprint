use async_std::task::{self, block_on};
use std::collections::{HashMap, HashSet};
use std::fmt::Debug;
use std::sync::Arc;

use line_drawing::{VoxelOrigin, WalkVoxels};
use ordered_float::NotNan;
use parry3d_f64::bounding_volume::Aabb;
use parry3d_f64::math::{Isometry, Point, Vector};
use parry3d_f64::query::{intersection_test, PointQuery};
use parry3d_f64::shape::{Cuboid, Shape, TriMesh, Triangle};

use crate::squarion::*;
use crate::svo::*;
use std::f64::consts::PI;
use image::RgbaImage;

#[derive(Clone, Debug)]
pub struct VoxelizationParams {
    pub straightness_bias: f64,
    pub angle_straightness_bias: f64,
    pub snap_threshold: f64,
    pub search_span: f64,
    pub inflate: f64,
    pub degenerate_area: f64,
    pub edge_first: bool,
    pub min_segment_len: i32,
    #[allow(unused)]
    pub snap_gap: u8,
    pub direction_bins: u8,
    pub edge_preserve_deg: f64,
    pub panel_mode: bool,
    pub edge_line_smoothing: bool,
    pub edge_snap_gap: u8,
    pub edge_min_segment_len: i32,
    pub offset_scale: f64,
    pub edge_protect_radius: i32,
    pub face_snap_gap: u8,
    pub face_min_segment_len: i32,
    pub face_smoothing_passes: u8,
    // Use surface-projected default offsets instead of [126,126,126]
    pub default_offset_surface: bool,
    // Harmonize offsets across sharp seams with a tight per-axis gap
    pub seam_harmonize_gap: u8,
}

impl Default for VoxelizationParams {
    fn default() -> Self {
        Self {
            straightness_bias: 0.0,
            angle_straightness_bias: 0.0,
            snap_threshold: 84.0,
            search_span: 5.0,
            inflate: 1.05,
            degenerate_area: 1e-6,
            edge_first: false,
            min_segment_len: 2,
            snap_gap: 2,
            direction_bins: 6,
            edge_preserve_deg: 25.0,
            panel_mode: false,
            edge_line_smoothing: true,
            edge_snap_gap: 1,
            edge_min_segment_len: 3,
            offset_scale: 84.0,
            edge_protect_radius: 1,
            face_snap_gap: 3,
            face_min_segment_len: 3,
            face_smoothing_passes: 1,
            default_offset_surface: false,
            seam_harmonize_gap: 1,
        }
    }
}

#[derive(Clone)]
pub struct UvSampler {
    img: RgbaImage,
    w: usize,
    h: usize,
    // Exact color → DU id mapping
    color_to_du: std::collections::HashMap<[u8; 3], u64>,
}

impl UvSampler {
    pub fn new(img: RgbaImage, w: usize, h: usize, color_to_du: std::collections::HashMap<[u8; 3], u64>) -> Self {
        Self { img, w, h, color_to_du }
    }

    fn sample_rgb(&self, mut u: f32, mut v: f32) -> [u8; 3] {
        // Repeat wrap and nearest sampling
        u = u.fract(); if u < 0.0 { u += 1.0; }
        v = v.fract(); if v < 0.0 { v += 1.0; }
        let x = (u * self.w as f32).floor().clamp(0.0, self.w as f32 - 1.0) as u32;
        let y = ((1.0 - v) * self.h as f32).floor().clamp(0.0, self.h as f32 - 1.0) as u32;
        let p = self.img.get_pixel(x, y);
        [p[0], p[1], p[2]]
    }

    pub fn color_to_du_id(&self, rgb: [u8; 3]) -> Option<u64> {
        if let Some(id) = self.color_to_du.get(&rgb) { return Some(*id); }
        // Nearest in palette if provided
        if self.color_to_du.is_empty() { return None; }
        let mut best: Option<(u64, u32)> = None;
        for (c, id) in self.color_to_du.iter() {
            let dr = c[0] as i32 - rgb[0] as i32;
            let dg = c[1] as i32 - rgb[1] as i32;
            let db = c[2] as i32 - rgb[2] as i32;
            let d = (dr*dr + dg*dg + db*db) as u32;
            if best.map(|(_, bd)| d < bd).unwrap_or(true) { best = Some((*id, d)); }
        }
        best.map(|(id, _)| id)
    }
}

#[derive(Debug, Clone, PartialEq)]
enum Voxel {
    Internal,
    External,
    Boundary(bool),
}

fn voxelize(
    isometry: &Isometry<f64>,
    mesh: &TriMesh,
    aabb: &Aabb,
    origin: Point<i32>,
    extent: usize,
    clip_range: &RangeZYX,
    params: &VoxelizationParams,
) -> Svo<Voxel> {
    let voxel_size = aabb.extents().x / extent as f64;
    Svo::from_fn(origin, extent, &|range| {
        if range.intersection(clip_range).volume() == 0 {
            return SvoReturn::Leaf(Voxel::External);
        }

        let mins = aabb.mins + (range.origin - origin).map(|v| v as f64) * voxel_size;
        let maxs = mins + range.size.map(|v| v as f64) * voxel_size;
        let aabb = Aabb::new(mins, maxs);

        // Scale up the region slightly. Makes intersection detection more robust.
        let cuboid = Cuboid::new(aabb.half_extents() * params.inflate);
        let cuboid_pos = Isometry::from(aabb.center());
        if !intersection_test(isometry, mesh, &cuboid_pos, &cuboid).unwrap() {
            // Vote on if the voxel is inside or outside. We need to do this because some people won't
            // read the FAQ, and try to import non-manifold meshes. This makes the process more reliable.
            let mut inside_count = mesh.contains_point(isometry, &aabb.center()) as u32;
            for point in aabb.vertices() {
                inside_count += mesh.contains_point(isometry, &point) as u32
            }
            // Bias towards assuming outside, since it's better to have empty internals than random
            // floating cubes.
            if inside_count >= 7 {
                SvoReturn::Leaf(Voxel::Internal)
            } else {
                SvoReturn::Leaf(Voxel::External)
            }
        } else if range.volume() == 1 {
            // We do a quick check to see if the voxel is "significant", i.e. the center is in the mesh.
            //
            // This helps remove artifacts from internal angles in the model.
            let mut significant = mesh.contains_point(isometry, &aabb.center());
            // Panel mode: treat boundary voxels near the surface as significant even if the center is outside.
            if params.panel_mode && !significant {
                let dist = mesh.distance_to_local_point(&aabb.center(), false);
                let surf_tol = voxel_size * 0.35;
                if dist < surf_tol {
                    significant = true;
                }
            }
            SvoReturn::Leaf(Voxel::Boundary(significant))
        } else {
            SvoReturn::Internal(Voxel::Boundary(false))
        }
    })
}

fn discretize(point: Point<f64>, voxel_size: f64, scale: f64) -> Point<f64> {
    (scale * point / voxel_size).map(|v| v.round())
}

// In game voxels operate on a discrete grid, so the best solutions are ones that
// minimize error when going from the model surface to the in game voxel surface.
fn lowest_error_point_on_surface(
    starts: &[Point<f64>],
    end: &Point<f64>,
    shape: &impl Shape,
    params: &VoxelizationParams,
    align_dir: Option<Vector<f64>>,
) -> Point<f64> {
    let mut lowest_error = f64::MAX;
    let mut best = *end;

    for start in starts {
        if (end - start).magnitude() < 0.1 {
            continue;
        }
        let dir = (end - start).normalize();
        let start_probe = *end - params.search_span * dir;
        let end_probe = *end + params.search_span * dir;
        for (x, y, z) in WalkVoxels::<f64, i64>::new(
            (start_probe.x, start_probe.y, start_probe.z),
            (end_probe.x, end_probe.y, end_probe.z),
            &VoxelOrigin::Corner,
        ) {
            let point = Point::new(x as f64, y as f64, z as f64);
            let geom_error = shape.distance_to_local_point(&point, false);
            let v = point - *end;
            // Axis-aligned penalty
            let ax = v.x.abs();
            let ay = v.y.abs();
            let az = v.z.abs();
            let l1 = ax + ay + az;
            let l_inf = ax.max(ay).max(az);
            let axis_penalty = (l1 - l_inf) * params.straightness_bias;
            // Angle-aligned penalty if a direction is provided
            let angle_penalty = if let Some(dir) = align_dir {
                let n = dir.normalize();
                let dot = v.dot(&n);
                let proj = n * dot;
                let perp = v - proj;
                perp.magnitude() * params.angle_straightness_bias
            } else {
                0.0
            };
            let straight_penalty = axis_penalty + angle_penalty;
            let total = geom_error + straight_penalty;
            if total < lowest_error {
                lowest_error = total;
                best = point;
            }
        }
    }
    best
}

fn to_voxel_offset(offset: Vector<f64>) -> Vector<u8> {
    offset.map(|v| (126.0 + v).clamp(0.0, 252.0) as u8)
}

// Tries to snap to the nearest vertex, then edge, then face.
//
// Results in some artifacts in game where too many voxels snap to a vertex/edge and as a result
// you get some weird shadows on flat surfaces, but this otherwise preserves the model features.
fn calculate_vertex_offset(
    isometry: &Isometry<f64>,
    mesh: &TriMesh,
    aabb: &Aabb,
    anchor: Point<f64>,
    voxel_size: f64,
    params: &VoxelizationParams,
) -> Vector<u8> {
    let discrete_anchor = discretize(anchor, voxel_size, params.offset_scale);
    let discrete_pos = discretize(aabb.center(), voxel_size, params.offset_scale);
    let (_, feature) = mesh.project_point_and_get_feature(isometry, &anchor);
    let face = feature.unwrap_face();
    let original_triangle = mesh.triangle(face).transformed(isometry);
    let a = discretize(original_triangle.a, voxel_size, params.offset_scale);
    let b = discretize(original_triangle.b, voxel_size, params.offset_scale);
    let c = discretize(original_triangle.c, voxel_size, params.offset_scale);
    let triangle = Triangle::new(a, b, c);

    let try_edge_first = params.edge_first;

    let choose_vertex = || -> Option<Vector<u8>> {
        let closest_vertex = triangle
            .vertices()
            .iter()
            .min_by_key(|v| NotNan::new((*v - discrete_anchor).magnitude()).unwrap())
            .unwrap();
        let offset = closest_vertex - discrete_pos;
        if offset.magnitude() < params.snap_threshold {
            Some(to_voxel_offset(closest_vertex - discrete_pos))
        } else {
            None
        }
    };

    let choose_edge = || -> Option<Vector<u8>> {
        let (segment, closest_edge) = triangle
            .edges()
            .map(|s| (s, s.project_local_point(&discrete_anchor, false).point))
            .into_iter()
            .min_by_key(|(_, v)| NotNan::new((*v - discrete_anchor).magnitude()).unwrap())
            .unwrap();
        let offset = closest_edge - discrete_pos;
        if offset.magnitude() < params.snap_threshold {
            // Direction along the chosen edge (in discrete space)
            let seg_vec = segment.b - segment.a;
            let seg_dir = Vector::new(seg_vec.x, seg_vec.y, seg_vec.z);
            let best = lowest_error_point_on_surface(
                &[segment.a],
                &closest_edge,
                &segment,
                params,
                Some(seg_dir),
            );
            Some(to_voxel_offset(best - discrete_pos))
        } else {
            None
        }
    };

    let pick = if try_edge_first {
        choose_edge().or_else(|| choose_vertex())
    } else {
        choose_vertex().or_else(|| choose_edge())
    };

    if let Some(p) = pick {
        return p;
    }

    if triangle.area() <= params.degenerate_area {
        let fallback = original_triangle
            .project_local_point(&discrete_anchor, false)
            .point;
        to_voxel_offset(fallback - discrete_pos)
    } else {
        let point = triangle.project_local_point(&discrete_anchor, false).point;
        let best = lowest_error_point_on_surface(&[a, b, c], &point, &triangle, params, None);
        to_voxel_offset(best - discrete_pos)
    }
}

fn extract_vertices(
    voxels: &Svo<Voxel>,
    isometry: &Isometry<f64>,
    mesh: &TriMesh,
    aabb: &Aabb,
    origin: Point<i32>,
    params: &VoxelizationParams,
) -> HashMap<Point<i32>, Point<u8>> {
    let voxel_size = aabb.extents().x / voxels.range.size.x as f64;
    let mut significant_points = HashMap::new();
    voxels.cata(|range, v, cs| {
        if cs.is_some() {
            return;
        }
        match v {
            Voxel::Boundary(significant) => {
                assert_eq!(range.volume(), 1);
                let center =
                    aabb.mins + voxel_size * (range.origin - origin).map(|v| v as f64 + 0.5);
                for offset in &RangeZYX::OFFSETS {
                    let offset = Vector::from_row_slice(offset);
                    let point = range.origin + offset;

                    let pos = aabb.mins + voxel_size * (point - origin).map(|v| v as f64);
                    if mesh.contains_point(isometry, &pos) != *significant {
                        let entry = significant_points
                            .entry(point)
                            .or_insert_with(|| Vec::new());
                        entry.push(center.coords)
                    }
                }
            }
            _ => (),
        };
    });

    let mut result = HashMap::new();
    for (point, anchors) in significant_points {
        let anchor = anchors.iter().fold(Point::origin(), |a, v| a + v) / anchors.len() as f64;
        let pos = aabb.mins + voxel_size * (point - origin).map(|v| v as f64);
        let aabb = Aabb::from_half_extents(pos, Vector::repeat(voxel_size * 1.5));

        let best = calculate_vertex_offset(isometry, mesh, &aabb, anchor, voxel_size, params);
        result.insert(point, Point::origin() + best);
    }
    // Continuity pass to encourage straight lines by snapping neighbors
    if params.min_segment_len >= 2 {
        let mut updated = result.clone();
        // Helper: compute face normal near a grid point; None if not on a face (edge/vertex) so we preserve sharp features.
        let face_normal_at = |grid_p: &Point<i32>| -> Option<Vector<f64>> {
            let world = aabb.mins + voxel_size * (*grid_p - origin).map(|v| v as f64);
            let (_, feat) = mesh.project_point_and_get_feature(isometry, &world);
            let face_idx = feat.unwrap_face();
            let tri = mesh.triangle(face_idx).transformed(isometry);
            let ab = tri.b - tri.a;
            let ac = tri.c - tri.a;
            let n = ab.cross(&ac);
            let norm = n.normalize();
            Some(Vector::new(norm.x, norm.y, norm.z))
        };
        let preserve_angle = params.edge_preserve_deg * PI / 180.0;
        // Build direction set based on bins: 6 (axes), 18 (axes + face diagonals), 26 (axes + face + space diagonals)
        let mut dirs: Vec<Vector<i32>> = vec![
            Vector::new(1, 0, 0),
            Vector::new(-1, 0, 0),
            Vector::new(0, 1, 0),
            Vector::new(0, -1, 0),
            Vector::new(0, 0, 1),
            Vector::new(0, 0, -1),
        ];
        if params.direction_bins >= 18 {
            let face_diagonals = [
                Vector::new(1, 1, 0),
                Vector::new(1, -1, 0),
                Vector::new(-1, 1, 0),
                Vector::new(-1, -1, 0),
                Vector::new(1, 0, 1),
                Vector::new(1, 0, -1),
                Vector::new(-1, 0, 1),
                Vector::new(-1, 0, -1),
                Vector::new(0, 1, 1),
                Vector::new(0, 1, -1),
                Vector::new(0, -1, 1),
                Vector::new(0, -1, -1),
            ];
            dirs.extend_from_slice(&face_diagonals);
        }
        if params.direction_bins >= 26 {
            let space_diagonals = [
                Vector::new(1, 1, 1),
                Vector::new(1, 1, -1),
                Vector::new(1, -1, 1),
                Vector::new(1, -1, -1),
                Vector::new(-1, 1, 1),
                Vector::new(-1, 1, -1),
                Vector::new(-1, -1, 1),
                Vector::new(-1, -1, -1),
            ];
            dirs.extend_from_slice(&space_diagonals);
        }
        // Build an edge protection mask by detecting large normal discontinuities, then dilate
        let neighbor6 = [
            Vector::new(1, 0, 0),
            Vector::new(-1, 0, 0),
            Vector::new(0, 1, 0),
            Vector::new(0, -1, 0),
            Vector::new(0, 0, 1),
            Vector::new(0, 0, -1),
        ];
        let mut edge_mask: HashSet<Point<i32>> = HashSet::new();
        for &p in result.keys() {
            if let Some(n0) = face_normal_at(&p) {
                for d in neighbor6.iter() {
                    let pn = p + *d;
                    if let Some(_) = result.get(&pn) {
                        if let Some(n1) = face_normal_at(&pn) {
                            let ang = n0.dot(&n1).clamp(-1.0, 1.0).acos();
                            if ang > preserve_angle {
                                edge_mask.insert(p);
                                break;
                            }
                        }
                    }
                }
            }
        }
        if params.edge_protect_radius > 0 {
            let mut dilated = edge_mask.clone();
            for _ in 0..params.edge_protect_radius {
                let mut next = dilated.clone();
                for &p in dilated.iter() {
                    for d in neighbor6.iter() {
                        next.insert(p + *d);
                    }
                }
                dilated = next;
            }
            edge_mask = dilated;
        }

        for dir in dirs.iter() {
            for (&p, off) in result.iter() {
                let p_next = p + *dir;
                if let Some(off2) = result.get(&p_next) {
                    // Do not smooth across or at protected edge points
                    if edge_mask.contains(&p) || edge_mask.contains(&p_next) { continue; }
                    // Preserve sharp mesh features: require similar face normals
                    let n1 = face_normal_at(&p);
                    let n2 = face_normal_at(&p_next);
                    let ok_normals = match (n1, n2) {
                        (Some(a), Some(b)) => {
                            let dot = a.dot(&b).clamp(-1.0, 1.0);
                            dot.acos() <= preserve_angle
                        }
                        _ => false, // if we can't determine, be conservative (don't smooth)
                    };
                    if !ok_normals { continue; }
                    let dx = (off.x as i32 - off2.x as i32).abs() as u8;
                    let dy = (off.y as i32 - off2.y as i32).abs() as u8;
                    let dz = (off.z as i32 - off2.z as i32).abs() as u8;
                    if dx <= params.face_snap_gap && dy <= params.face_snap_gap && dz <= params.face_snap_gap {
                        let avg = Point::new(
                            ((off.x as u16 + off2.x as u16) / 2) as u8,
                            ((off.y as u16 + off2.y as u16) / 2) as u8,
                            ((off.z as u16 + off2.z as u16) / 2) as u8,
                        );
                        updated.insert(p, avg);
                        updated.insert(p_next, avg);
                    }
                }
            }
        }
        // Replace result with the averaged map
        result = updated;
        // Optional: edge-line smoothing to keep fine edges angular but not jagged
        if params.edge_line_smoothing {
            let mut edge_smoothed = result.clone();
            let face_normal_at = |grid_p: &Point<i32>| -> Option<Vector<f64>> {
                let world = aabb.mins + voxel_size * (*grid_p - origin).map(|v| v as f64);
                let (_, feat) = mesh.project_point_and_get_feature(isometry, &world);
                let face_idx = feat.unwrap_face();
                let tri = mesh.triangle(face_idx).transformed(isometry);
                let ab = tri.b - tri.a;
                let ac = tri.c - tri.a;
                let n = ab.cross(&ac);
                let norm = n.normalize();
                Some(Vector::new(norm.x, norm.y, norm.z))
            };
            let preserve_angle = params.edge_preserve_deg * PI / 180.0;
            // build normalized float directions for bin matching
            let dirs_f: Vec<Vector<f64>> = dirs
                .iter()
                .map(|d| Vector::new(d.x as f64, d.y as f64, d.z as f64).normalize())
                .collect();
            let choose_dir_bin = |t: Vector<f64>| -> usize {
                let tn = t.normalize();
                let mut best = 0usize;
                let mut best_dot = -1.0f64;
                for (i, df) in dirs_f.iter().enumerate() {
                    let dot = tn.dot(df);
                    if dot > best_dot {
                        best_dot = dot;
                        best = i;
                    }
                }
                best
            };
            let mut visited: HashSet<Point<i32>> = HashSet::new();
            for &start in result.keys() {
                if visited.contains(&start) { continue; }
                // detect an edge by checking any neighbor with a sufficiently different normal
                let n0 = match face_normal_at(&start) { Some(n) => n, None => continue };
                let mut picked_dir: Option<Vector<i32>> = None;
                for d in dirs.iter() {
                    let nxt = start + *d;
                    if let Some(_) = result.get(&nxt) {
                        if let Some(n1) = face_normal_at(&nxt) {
                            let ang = n0.dot(&n1).clamp(-1.0, 1.0).acos();
                            if ang > preserve_angle { // edge detected
                                // edge tangent is cross of normals
                                let t = n0.cross(&n1);
                                if t.magnitude() > 1e-6 {
                                    let idx = choose_dir_bin(t);
                                    picked_dir = Some(dirs[idx]);
                                    break;
                                }
                            }
                        }
                    }
                }
                if picked_dir.is_none() { continue; }
                let edir = picked_dir.unwrap();
                if visited.contains(&start) { continue; }
                // build run along edge direction
                let mut run: Vec<Point<i32>> = Vec::new();
                let mut curr = start;
                loop {
                    if let Some(off_curr) = result.get(&curr) {
                        run.push(curr);
                        visited.insert(curr);
                        let next = curr + edir;
                        if let Some(off_next) = result.get(&next) {
                            // ensure still on edge
                            if let (Some(nc), Some(nn)) = (face_normal_at(&curr), face_normal_at(&next)) {
                                let ang = nc.dot(&nn).clamp(-1.0, 1.0).acos();
                                if ang <= preserve_angle { break; }
                            } else { break; }
                            let dx = (off_curr.x as i32 - off_next.x as i32).abs() as u8;
                            let dy = (off_curr.y as i32 - off_next.y as i32).abs() as u8;
                            let dz = (off_curr.z as i32 - off_next.z as i32).abs() as u8;
                            if dx <= params.edge_snap_gap && dy <= params.edge_snap_gap && dz <= params.edge_snap_gap {
                                curr = next;
                                continue;
                            }
                        }
                    }
                    break;
                }
                if run.len() as i32 >= params.edge_min_segment_len && run.len() >= 3 {
                    let s = result.get(&run[0]).unwrap();
                    let e = result.get(run.last().unwrap()).unwrap();
                    let sx = s.x as i32; let sy = s.y as i32; let sz = s.z as i32;
                    let ex = e.x as i32; let ey = e.y as i32; let ez = e.z as i32;
                    let n = run.len();
                    for (i, pt) in run.iter().enumerate() {
                        let tnum = i as f64; let tden = (n - 1) as f64;
                        let ix = (sx as f64 + (ex - sx) as f64 * tnum / tden).round().clamp(0.0, 255.0) as u8;
                        let iy = (sy as f64 + (ey - sy) as f64 * tnum / tden).round().clamp(0.0, 255.0) as u8;
                        let iz = (sz as f64 + (ez - sz) as f64 * tnum / tden).round().clamp(0.0, 255.0) as u8;
                        edge_smoothed.insert(*pt, Point::new(ix, iy, iz));
                    }
                }
            }
            result = edge_smoothed;
        }
        // Optional: seam harmonization across sharp edges with tight gap
        if params.seam_harmonize_gap > 0 {
            let mut harmonized = result.clone();
            for dir in dirs.iter() {
                for (&p, off) in result.iter() {
                    let p_next = p + *dir;
                    if let Some(off2) = result.get(&p_next) {
                        // only at sharp feature boundaries
                        let n1 = face_normal_at(&p);
                        let n2 = face_normal_at(&p_next);
                        let sharp = match (n1, n2) {
                            (Some(a), Some(b)) => {
                                let dot = a.dot(&b).clamp(-1.0, 1.0);
                                dot.acos() > preserve_angle
                            }
                            _ => false,
                        };
                        if !sharp { continue; }
                        let dx = (off.x as i32 - off2.x as i32).abs() as u8;
                        let dy = (off.y as i32 - off2.y as i32).abs() as u8;
                        let dz = (off.z as i32 - off2.z as i32).abs() as u8;
                        if dx <= params.seam_harmonize_gap && dy <= params.seam_harmonize_gap && dz <= params.seam_harmonize_gap {
                            let avg = Point::new(
                                ((off.x as u16 + off2.x as u16) / 2) as u8,
                                ((off.y as u16 + off2.y as u16) / 2) as u8,
                                ((off.z as u16 + off2.z as u16) / 2) as u8,
                            );
                            harmonized.insert(p, avg);
                            harmonized.insert(p_next, avg);
                        }
                    }
                }
            }
            result = harmonized;
        }
        // Face-only smoothing passes (interior runs only, away from edges)
        let mut smoothed = result.clone();
        for _pass in 0..params.face_smoothing_passes {
            let mut pass_map = smoothed.clone();
            for dir in dirs.iter() {
                let mut visited: HashSet<Point<i32>> = HashSet::new();
                for &start in smoothed.keys() {
                    if visited.contains(&start) { continue; }
                    let prev = start - *dir;
                    if smoothed.contains_key(&prev) { continue; }
                    // build run forward
                    let mut run: Vec<Point<i32>> = Vec::new();
                    let mut curr = start;
                    while let Some(off_curr) = smoothed.get(&curr) {
                        if edge_mask.contains(&curr) { break; }
                        run.push(curr);
                        visited.insert(curr);
                        let next = curr + *dir;
                        if let Some(off_next) = smoothed.get(&next) {
                            if edge_mask.contains(&next) { break; }
                            // Preserve sharp features along the run as well
                            let n1 = face_normal_at(&curr);
                            let n2 = face_normal_at(&next);
                            let ok_normals = match (n1, n2) {
                                (Some(a), Some(b)) => {
                                    let dot = a.dot(&b).clamp(-1.0, 1.0);
                                    dot.acos() <= preserve_angle
                                }
                                _ => false,
                            };
                            if !ok_normals { break; }
                            let dx = (off_curr.x as i32 - off_next.x as i32).abs() as u8;
                            let dy = (off_curr.y as i32 - off_next.y as i32).abs() as u8;
                            let dz = (off_curr.z as i32 - off_next.z as i32).abs() as u8;
                            if dx <= params.face_snap_gap && dy <= params.face_snap_gap && dz <= params.face_snap_gap {
                                curr = next;
                                continue;
                            }
                        }
                        break;
                    }
                    if run.len() as i32 >= params.face_min_segment_len && run.len() >= 3 {
                        let s = smoothed.get(&run[0]).unwrap();
                        let e = smoothed.get(run.last().unwrap()).unwrap();
                        let sx = s.x as i32; let sy = s.y as i32; let sz = s.z as i32;
                        let ex = e.x as i32; let ey = e.y as i32; let ez = e.z as i32;
                        let n = run.len();
                        for (i, pt) in run.iter().enumerate() {
                            let t_num = i as f64; let t_den = (n - 1) as f64;
                            let ix = (sx as f64 + (ex - sx) as f64 * t_num / t_den).round().clamp(0.0, 255.0) as u8;
                            let iy = (sy as f64 + (ey - sy) as f64 * t_num / t_den).round().clamp(0.0, 255.0) as u8;
                            let iz = (sz as f64 + (ez - sz) as f64 * t_num / t_den).round().clamp(0.0, 255.0) as u8;
                            pass_map.insert(*pt, Point::new(ix, iy, iz));
                        }
                    }
                }
            }
            smoothed = pass_map;
        }
        result = smoothed;
    }
    result
}

// This is by far the most expensive part, mostly due to Trimesh being kinda slow and the algorithm itself
// being pretty naive. For now we just throw threads at it, but it can definitely be improved.
#[allow(dead_code)]

// Multi-mesh variant: builds a single grid that overlays contributions from each mesh
// and maps each mesh to a distinct material index and id.
fn voxelize_chunk_multi(
    isometry: &Isometry<f64>,
    meshes: &[(Arc<TriMesh>, u8, u64)],
    aabb: &Aabb,
    voxel_origin: &Point<i32>,
    is_lod: bool,
    params: &VoxelizationParams,
) -> Option<VoxelCellData> {
    // Use the same over-voxelization strategy as single-mesh.
    let voxel_size = aabb.extents().x / 32.0;
    let voxel_size_offset = Vector::repeat(voxel_size);
    let origin = aabb.mins - voxel_size_offset * 2.0;

    let range = RangeZYX::with_extent(*voxel_origin - Vector::repeat(1), 35);

    // Note that this large aabb could result in a lot of wasted computation, so we clip the range.
    let svo_aabb = Aabb::new(origin, origin + voxel_size_offset * 64.0);

    let inner_range = RangeZYX::with_extent(*voxel_origin, 32);
    let mut grid = VertexGrid::new(range, inner_range);

    // Track if anything was written to materials across all meshes
    let mut any_materials = false;

    for (mesh, mat_index, _mat_id) in meshes.iter() {
        let voxels = voxelize(
            isometry,
            mesh.as_ref(),
            &svo_aabb,
            *voxel_origin - Vector::repeat(2),
            64,
            &RangeZYX::with_extent(*voxel_origin - Vector::repeat(1), 35),
            params,
        );

        voxels.cata(|subrange, value, cs| {
            if cs.is_some() {
                return;
            }
            let (place_materials, place_positions) = match value {
                Voxel::External => (false, false),
                Voxel::Internal => (true, true),
                // For fully solid shape: always assign materials on boundary leaves too; positions always set.
                Voxel::Boundary(_significant) => (true, true),
            };
            if place_materials {
                any_materials = true;
                // Materials are placed on the +[1, 1, 1] vertex.
                let material_range = RangeZYX {
                    origin: subrange.origin + Vector::repeat(1),
                    size: subrange.size,
                };
                grid.set_materials(&material_range, VertexMaterial::new(*mat_index));
            }
            if place_positions {
                // Set the default positions for all voxels in this subrange.
                let voxel_range = RangeZYX {
                    origin: subrange.origin,
                    size: subrange.size + Vector::repeat(1),
                };
                if params.default_offset_surface {
                    let center = svo_aabb.mins
                        + (subrange.origin - (*voxel_origin - Vector::repeat(2))).map(|v| v as f64 + 0.5)
                            * (aabb.extents().x / 64.0);
                    let proj = mesh.project_point(isometry, &center, false);
                    let off = to_voxel_offset(proj.point - center);
                    grid.set_voxels(&voxel_range, VertexVoxel::new([off.x, off.y, off.z]));
                } else {
                    grid.set_voxels(&voxel_range, VertexVoxel::new([126, 126, 126]));
                }
            }
        });

        // Extract and set significant vertex offsets for this mesh.
        let vertices = extract_vertices(
            &voxels,
            isometry,
            mesh.as_ref(),
            &svo_aabb,
            *voxel_origin - Vector::repeat(2),
            params,
        );
        for (point, offset) in vertices {
            grid.set_voxel(&point, VertexVoxel::new([offset.x, offset.y, offset.z]));
        }
    }

    // Fallback for thin single-material shells: if positions exist but no materials were placed,
    // fill the inner_range with the first mesh's material so the chunk is not discarded.
    if !is_lod && !any_materials && !grid.is_empty() {
        if let Some((_, first_idx, _)) = meshes.first() {
            let material_range = RangeZYX {
                origin: inner_range.origin + Vector::repeat(1),
                size: inner_range.size,
            };
            grid.set_materials(&material_range, VertexMaterial::new(*first_idx));
            any_materials = true;
        }
    }

    if !is_lod && (!any_materials || grid.is_empty()) {
        return None;
    }

    // Build a combined material mapping: debug + each mesh material
    let mut mapping = MaterialMapper::default();
    mapping.insert(
        1,
        MaterialId {
            id: 157903047,
            short_name: "Debug1\0\0".into(),
        },
    );
    for (_, idx, mat_id) in meshes.iter() {
        mapping.insert(
            *idx,
            MaterialId {
                id: *mat_id,
                short_name: "Material".into(),
            },
        );
    }

    Some(VoxelCellData::new(grid, mapping))
}


// (Single-mesh Voxelizer removed as dead code)

// Create LODs for multiple meshes and materials into a single SVO with a combined mapping.
pub fn create_lods_multi(
    isometry: &Isometry<f64>,
    meshes: &[TriMesh],
    aabb: &Aabb,
    origin: Point<i32>,
    height: usize,
    materials: &[u64],
    params: &VoxelizationParams,
) -> Svo<Option<VoxelCellData>> {
    let extent = 1 << height;
    let chunk_size = aabb.extents().x / extent as f64;

    // Pre-wrap meshes and assign stable material indices starting at 2
    let wrapped: Vec<(Arc<TriMesh>, u8, u64)> = meshes
        .iter()
        .enumerate()
        .map(|(i, m)| (Arc::new(m.clone()), (2 + i) as u8, materials[i]))
        .collect();

    let isometry = Arc::new(*isometry);
    let params = Arc::new(params.clone());
    let chunk_futures = Svo::from_fn(origin, extent, &|range| {
        let mins = aabb.mins + (range.origin - origin).map(|v| v as f64) * chunk_size;
        let maxs = mins + range.size.map(|v| v as f64) * chunk_size;
        let aabb = Aabb::new(mins.into(), maxs.into());

        // If no mesh intersects this chunk, skip entirely.
        let cuboid = Cuboid::new(aabb.half_extents() * 1.05);
        let cuboid_pos = Isometry::from(aabb.center());
        let mut intersects_any = false;
        for (mesh, _, _) in wrapped.iter() {
            if intersection_test(&isometry, mesh.as_ref(), &cuboid_pos, &cuboid).unwrap() {
                intersects_any = true;
                break;
            }
        }
        if !intersects_any {
            return SvoReturn::Leaf(None);
        }

        let is_lod = range.size.x > 1;
        let voxel_origin = range.origin * 32 / range.size.x;
        let isometry = isometry.clone();
        let chunk_meshes = wrapped.clone();
        let params = params.clone();
        let task = task::spawn(async move {
            voxelize_chunk_multi(&isometry, &chunk_meshes, &aabb, &voxel_origin, is_lod, params.as_ref())
        });
        if range.size.x == 1 {
            SvoReturn::Leaf(Some(task))
        } else {
            SvoReturn::Internal(Some(task))
        }
    });

    chunk_futures.into_map(|f| f.map(|f| block_on(f)).flatten())
}

// Create LODs for multiple meshes with UV-driven per-voxel materials
pub fn create_lods_multi_uv(
    isometry: &Isometry<f64>,
    meshes: &[TriMesh],
    uv_faces: &Vec<Vec<[[f32; 2]; 3]>>, // per-mesh, per-face UV coords
    aabb: &Aabb,
    origin: Point<i32>,
    height: usize,
    params: &VoxelizationParams,
    sampler: UvSampler,
) -> Svo<Option<VoxelCellData>> {
    let extent = 1 << height;
    let chunk_size = aabb.extents().x / extent as f64;

    // Wrap meshes and uv faces
    let wrapped: Vec<(Arc<TriMesh>, Vec<[[f32; 2]; 3]>)> = meshes
        .iter()
        .cloned()
        .zip(uv_faces.clone().into_iter())
        .map(|(m, u)| (Arc::new(m), u))
        .collect();

    let isometry = Arc::new(*isometry);
    let params = Arc::new(params.clone());
    let sampler = Arc::new(sampler);
    let chunk_futures = Svo::from_fn(origin, extent, &|range| {
        let mins = aabb.mins + (range.origin - origin).map(|v| v as f64) * chunk_size;
        let maxs = mins + range.size.map(|v| v as f64) * chunk_size;
        let aabb = Aabb::new(mins.into(), maxs.into());

        // Skip if no mesh intersects
        let cuboid = Cuboid::new(aabb.half_extents() * 1.05);
        let cuboid_pos = Isometry::from(aabb.center());
        let mut intersects_any = false;
        for (mesh, _) in wrapped.iter() {
            if intersection_test(&isometry, mesh.as_ref(), &cuboid_pos, &cuboid).unwrap() {
                intersects_any = true;
                break;
            }
        }
        if !intersects_any { return SvoReturn::Leaf(None); }

        let is_lod = range.size.x > 1;
        let voxel_origin = range.origin * 32 / range.size.x;
        let isometry = isometry.clone();
        let chunk_meshes = wrapped.clone();
        let params = params.clone();
        let sampler = sampler.clone();
        let task = task::spawn(async move {
            voxelize_chunk_multi_uv(&isometry, &chunk_meshes, &aabb, &voxel_origin, is_lod, params.as_ref(), sampler.as_ref())
        });
        if range.size.x == 1 { SvoReturn::Leaf(Some(task)) } else { SvoReturn::Internal(Some(task)) }
    });

    chunk_futures.into_map(|f| f.map(|f| block_on(f)).flatten())
}

fn barycentric(p: Point<f64>, a: Point<f64>, b: Point<f64>, c: Point<f64>) -> (f64, f64, f64) {
    let v0 = b - a; let v1 = c - a; let v2 = p - a;
    let d00 = v0.dot(&v0); let d01 = v0.dot(&v1); let d11 = v1.dot(&v1);
    let d20 = v2.dot(&v0); let d21 = v2.dot(&v1);
    let denom = (d00 * d11 - d01 * d01).max(1e-12);
    let v = (d11 * d20 - d01 * d21) / denom;
    let w = (d00 * d21 - d01 * d20) / denom;
    let u = 1.0 - v - w;
    (u, v, w)
}

fn voxelize_chunk_multi_uv(
    isometry: &Isometry<f64>,
    meshes: &[(Arc<TriMesh>, Vec<[[f32; 2]; 3]>)],
    aabb: &Aabb,
    voxel_origin: &Point<i32>,
    is_lod: bool,
    params: &VoxelizationParams,
    sampler: &UvSampler,
) -> Option<VoxelCellData> {
    let voxel_size = aabb.extents().x / 32.0;
    let voxel_size_offset = Vector::repeat(voxel_size);
    let origin = aabb.mins - voxel_size_offset * 2.0;

    let range = RangeZYX::with_extent(*voxel_origin - Vector::repeat(1), 35);
    let svo_aabb = Aabb::new(origin, origin + voxel_size_offset * 64.0);

    let inner_range = RangeZYX::with_extent(*voxel_origin, 32);
    let mut grid = VertexGrid::new(range, inner_range);
    let mut any_materials = false;

    // Prepare dynamic mapping for UV materials (start after mesh default slots)
    let mut du_to_idx: std::collections::HashMap<u64, u8> = std::collections::HashMap::new();
    let mut next_idx: u8 = 2 + meshes.len() as u8; // 1 is debug, 2.. per-mesh defaults
    // Track majority DU id per mesh (used to set default fill per mesh)
    let mut mesh_counts: Vec<std::collections::HashMap<u64, u32>> = vec![Default::default(); meshes.len()];

    for (mesh_i, (mesh, uvf)) in meshes.iter().enumerate() {
        let voxels = voxelize(
            isometry,
            mesh.as_ref(),
            &svo_aabb,
            *voxel_origin - Vector::repeat(2),
            64,
            &RangeZYX::with_extent(*voxel_origin - Vector::repeat(1), 35),
            params,
        );

        voxels.cata(|subrange, value, cs| {
            if cs.is_some() { return; }
            // For UV mode: place a mesh-default material on both Internal and Boundary leaves.
            // Per-voxel UV overrides will replace it where significant.
            let (place_materials, place_positions) = match value {
                Voxel::External => (false, false),
                Voxel::Internal => (true, true),
                Voxel::Boundary(_sig) => (true, true),
            };
            if place_materials {
                any_materials = true;
                let material_range = RangeZYX { origin: subrange.origin + Vector::repeat(1), size: subrange.size };
                // Assign per-mesh default material index (2 + mesh_i)
                grid.set_materials(&material_range, VertexMaterial::new((2 + mesh_i) as u8));
            }
            if place_positions {
                let voxel_range = RangeZYX { origin: subrange.origin, size: subrange.size + Vector::repeat(1) };
                grid.set_voxels(&voxel_range, VertexVoxel::new([126, 126, 126]));
            }
        });

        // For boundary leaves, sample UV and override material per cell
        voxels.cata(|subrange, value, cs| {
            if cs.is_some() { return; }
            if let Voxel::Boundary(sig) = value {
                if !*sig { return; }
                // center point in world
                let center = svo_aabb.mins + voxel_size * (subrange.origin - (*voxel_origin - Vector::repeat(2))).map(|v| v as f64 + 0.5);
                let (proj, feat) = mesh.project_point_and_get_feature(isometry, &center);
                let face_idx = feat.unwrap_face();
                let tri = mesh.triangle(face_idx).transformed(isometry);
                let (u_b, v_b, w_b) = barycentric(proj.point, tri.a, tri.b, tri.c);
                let uv = uvf.get(face_idx as usize).cloned().unwrap_or([[0.0,0.0],[0.0,0.0],[0.0,0.0]]);
                let u = u_b * uv[0][0] as f64 + v_b * uv[1][0] as f64 + w_b * uv[2][0] as f64;
                let v = u_b * uv[0][1] as f64 + v_b * uv[1][1] as f64 + w_b * uv[2][1] as f64;
                let rgb = sampler.sample_rgb(u as f32, v as f32);
                if let Some(du) = sampler.color_to_du_id(rgb) {
                    // Guard against running out of indices; reuse mesh default if saturated.
                    let idx = if let Some(existing) = du_to_idx.get(&du) {
                        *existing
                    } else if next_idx == u8::MAX {
                        (2 + mesh_i) as u8
                    } else {
                        let cur = next_idx;
                        next_idx = next_idx.saturating_add(1);
                        du_to_idx.insert(du, cur);
                        cur
                    };
                    let material_range = RangeZYX { origin: subrange.origin + Vector::repeat(1), size: Vector::repeat(1) };
                    grid.set_materials(&material_range, VertexMaterial::new(idx));
                    *mesh_counts[mesh_i].entry(du).or_insert(0) += 1;
                    any_materials = true;
                }
            }
        });
    }

    if !is_lod && (!any_materials || grid.is_empty()) { return None; }

    // Build mapping: debug, per-mesh defaults (majority color if available), then UV colors
    let mut mapping = MaterialMapper::default();
    mapping.insert(1, MaterialId { id: 157903047, short_name: "Debug1\0\0".into() });

    for (i, counts) in mesh_counts.iter().enumerate() {
        let mut best: Option<(u64, u32)> = None;
        for (k, c) in counts.iter() {
            if best.map(|(_, bc)| c > &bc).unwrap_or(true) { best = Some((*k, *c)); }
        }
        let du = best.map(|(k, _)| k).unwrap_or(1971262921);
        mapping.insert((2 + i) as u8, MaterialId { id: du, short_name: "Material".into() });
    }
    for (du, idx) in du_to_idx.iter() {
        mapping.insert(*idx, MaterialId { id: *du, short_name: "Material".into() });
    }

    Some(VoxelCellData::new(grid, mapping))
}

