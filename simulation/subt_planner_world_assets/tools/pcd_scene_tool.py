#!/usr/bin/env python3
"""Inspect a PCD scene and convert it into Gazebo-friendly mesh assets.

This helper is aimed at the subt_planner + Gazebo workflow in this workspace:

1. Read a .pcd point cloud with Open3D.
2. Estimate scene bounds and print suggested subt_planner parameters.
3. Optionally clean, reconstruct, simplify, and export a mesh.
4. Export a geometry-only Collada (.dae) file without requiring Blender.

The DAE exporter intentionally stays minimal: vertices, normals, triangles,
and one plain material. That is usually enough for Gazebo model loading.
"""

from __future__ import annotations

import argparse
import datetime as dt
import math
import pathlib
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import open3d as o3d


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be > 0")
    return parsed


def nonnegative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError("value must be >= 0")
    return parsed


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be > 0")
    return parsed


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def format_vec(values: np.ndarray, digits: int = 3) -> str:
    rounded = [f"{float(v):.{digits}f}" for v in values]
    return "[" + ", ".join(rounded) + "]"


def load_point_cloud(path: pathlib.Path) -> o3d.geometry.PointCloud:
    if not path.exists():
        raise FileNotFoundError(f"input file does not exist: {path}")

    point_cloud = o3d.io.read_point_cloud(str(path))
    if point_cloud.is_empty():
        raise ValueError(f"failed to load points from: {path}")

    points = np.asarray(point_cloud.points)
    finite_mask = np.isfinite(points).all(axis=1)
    if not np.any(finite_mask):
        raise ValueError(f"point cloud only contains non-finite values: {path}")
    if not np.all(finite_mask):
        indices = np.flatnonzero(finite_mask)
        point_cloud = point_cloud.select_by_index(indices.tolist())

    if point_cloud.is_empty():
        raise ValueError(f"point cloud is empty after filtering: {path}")
    return point_cloud


def point_cloud_stats(point_cloud: o3d.geometry.PointCloud) -> dict[str, np.ndarray | float | int]:
    points = np.asarray(point_cloud.points)
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    extents = maxs - mins
    center = (mins + maxs) / 2.0
    diagonal = float(np.linalg.norm(extents))
    return {
        "count": len(points),
        "mins": mins,
        "maxs": maxs,
        "extents": extents,
        "center": center,
        "diagonal": diagonal,
    }


def trimmed_bounds(points: np.ndarray, trim_percent: float) -> tuple[np.ndarray, np.ndarray]:
    if trim_percent <= 0.0:
        return points.min(axis=0), points.max(axis=0)
    if trim_percent >= 50.0:
        raise ValueError("trim percentile must be < 50")

    lower = np.percentile(points, trim_percent, axis=0)
    upper = np.percentile(points, 100.0 - trim_percent, axis=0)
    return lower, upper


def round_bounds(mins: np.ndarray, maxs: np.ndarray, resolution: float) -> tuple[np.ndarray, np.ndarray]:
    rounded_min = np.floor(mins / resolution) * resolution
    rounded_max = np.ceil(maxs / resolution) * resolution
    return rounded_min, rounded_max


def recommend_global_bounds(
    mins: np.ndarray,
    maxs: np.ndarray,
    margin_abs: float,
    margin_ratio: float,
    round_resolution: float,
) -> tuple[np.ndarray, np.ndarray]:
    extents = maxs - mins
    margin = np.maximum(np.full(3, margin_abs), extents * margin_ratio)
    return round_bounds(mins - margin, maxs + margin, round_resolution)


def recommend_local_search(extents: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    horizontal_extent = max(float(extents[0]), float(extents[1]))
    vertical_extent = float(extents[2])

    xy_half = clamp(horizontal_extent * 0.15, 10.0, 50.0)
    z_half = clamp(max(2.0, vertical_extent * 0.35), 2.0, 20.0)
    return np.array([-xy_half, -xy_half, -z_half]), np.array([xy_half, xy_half, z_half])


def recommend_traverse_length(extents: np.ndarray) -> float:
    horizontal_extent = max(float(extents[0]), float(extents[1]))
    return clamp(horizontal_extent * 0.12, 5.0, 15.0)


def estimate_spacing(point_cloud: o3d.geometry.PointCloud) -> float:
    distances = point_cloud.compute_nearest_neighbor_distance()
    if not distances:
        return 0.0
    return float(np.median(np.asarray(distances)))


def print_inspection_report(
    point_cloud: o3d.geometry.PointCloud,
    trim_percent: float,
    margin_abs: float,
    margin_ratio: float,
    round_resolution: float,
) -> None:
    stats = point_cloud_stats(point_cloud)
    points = np.asarray(point_cloud.points)
    robust_min, robust_max = trimmed_bounds(points, trim_percent)
    robust_extents = robust_max - robust_min
    robust_center = (robust_min + robust_max) / 2.0
    recommended_min, recommended_max = recommend_global_bounds(
        robust_min, robust_max, margin_abs, margin_ratio, round_resolution
    )
    local_search_min, local_search_max = recommend_local_search(robust_extents)
    traverse_length = recommend_traverse_length(robust_extents)
    median_spacing = estimate_spacing(point_cloud)

    print(f"Input point count: {stats['count']}")
    print(f"AABB min:          {format_vec(stats['mins'])}")
    print(f"AABB max:          {format_vec(stats['maxs'])}")
    print(f"AABB extents:      {format_vec(stats['extents'])}")
    print(f"AABB center:       {format_vec(stats['center'])}")
    print(f"AABB diagonal:     {stats['diagonal']:.3f} m")
    print()
    print(f"Trimmed bounds ({trim_percent:.2f}%):")
    print(f"  min:             {format_vec(robust_min)}")
    print(f"  max:             {format_vec(robust_max)}")
    print(f"  extents:         {format_vec(robust_extents)}")
    print(f"  center:          {format_vec(robust_center)}")
    print()
    if median_spacing > 0.0:
        print(f"Median nearest-neighbor spacing: {median_spacing:.4f} m")
        print()
    print("Suggested subt_planner params (heuristic):")
    print("BoundedSpaceParams:")
    print("  Global:")
    print("    type:           kCuboid")
    print(f"    min_val:        {format_vec(recommended_min)}")
    print(f"    max_val:        {format_vec(recommended_max)}")
    print("  LocalSearch:")
    print("    type:           kCuboid")
    print(f"    min_val:        {format_vec(local_search_min)}")
    print(f"    max_val:        {format_vec(local_search_max)}")
    print("PlanningParams:")
    print(f"  traverse_length_max: {traverse_length:.2f}")


def prepare_point_cloud(
    point_cloud: o3d.geometry.PointCloud,
    voxel_size: float,
    nb_neighbors: int,
    std_ratio: float,
) -> o3d.geometry.PointCloud:
    processed = point_cloud
    if voxel_size > 0.0:
        processed = processed.voxel_down_sample(voxel_size)

    if processed.is_empty():
        raise ValueError("point cloud became empty after voxel downsample")

    filtered, _ = processed.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio
    )
    if filtered.is_empty():
        raise ValueError("point cloud became empty after outlier removal")
    return filtered


def estimate_normals(point_cloud: o3d.geometry.PointCloud, search_radius: float) -> None:
    point_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=search_radius, max_nn=60)
    )
    if len(point_cloud.points) >= 20:
        try:
            point_cloud.orient_normals_consistent_tangent_plane(20)
        except RuntimeError:
            pass


def reconstruct_mesh_poisson(
    point_cloud: o3d.geometry.PointCloud,
    depth: int,
    density_trim_quantile: float,
    crop_margin: float,
) -> o3d.geometry.TriangleMesh:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        point_cloud, depth=depth
    )
    densities_np = np.asarray(densities)
    if density_trim_quantile > 0.0 and len(densities_np):
        threshold = np.quantile(densities_np, density_trim_quantile)
        mesh.remove_vertices_by_mask(densities_np < threshold)

    stats = point_cloud_stats(point_cloud)
    crop_box = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=stats["mins"] - crop_margin,
        max_bound=stats["maxs"] + crop_margin,
    )
    mesh = mesh.crop(crop_box)
    return mesh


def reconstruct_mesh_bpa(
    point_cloud: o3d.geometry.PointCloud,
    radius_scale: float,
) -> o3d.geometry.TriangleMesh:
    spacing = estimate_spacing(point_cloud)
    if spacing <= 0.0:
        raise ValueError("failed to estimate spacing for ball pivoting")
    radii = o3d.utility.DoubleVector(
        [spacing * radius_scale, spacing * radius_scale * 2.0, spacing * radius_scale * 4.0]
    )
    return o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(point_cloud, radii)


def postprocess_mesh(mesh: o3d.geometry.TriangleMesh, target_triangles: int) -> o3d.geometry.TriangleMesh:
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_non_manifold_edges()
    if target_triangles > 0 and len(mesh.triangles) > target_triangles:
        mesh = mesh.simplify_quadric_decimation(target_triangles)
        mesh.remove_duplicated_vertices()
        mesh.remove_duplicated_triangles()
        mesh.remove_degenerate_triangles()
        mesh.remove_non_manifold_edges()
    mesh.compute_vertex_normals()
    return mesh


def write_collada(mesh: o3d.geometry.TriangleMesh, output_path: pathlib.Path) -> None:
    if mesh.is_empty():
        raise ValueError("mesh is empty")

    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    triangles = np.asarray(mesh.triangles, dtype=np.int32)
    if len(vertices) == 0 or len(triangles) == 0:
        raise ValueError("mesh has no triangles")

    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    normals = np.asarray(mesh.vertex_normals, dtype=np.float64)
    if len(normals) != len(vertices):
        normals = np.zeros_like(vertices)

    namespace = "http://www.collada.org/2005/11/COLLADASchema"
    ET.register_namespace("", namespace)

    def qname(tag: str) -> str:
        return f"{{{namespace}}}{tag}"

    created_at = dt.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"
    geometry_id = "scene-geometry"
    material_id = "scene-material"
    effect_id = "scene-effect"

    root = ET.Element(qname("COLLADA"), {"version": "1.4.1"})
    asset = ET.SubElement(root, qname("asset"))
    contributor = ET.SubElement(asset, qname("contributor"))
    ET.SubElement(contributor, qname("authoring_tool")).text = "pcd_scene_tool.py"
    ET.SubElement(asset, qname("created")).text = created_at
    ET.SubElement(asset, qname("modified")).text = created_at
    ET.SubElement(asset, qname("unit"), {"name": "meter", "meter": "1"})
    ET.SubElement(asset, qname("up_axis")).text = "Z_UP"

    library_effects = ET.SubElement(root, qname("library_effects"))
    effect = ET.SubElement(library_effects, qname("effect"), {"id": effect_id})
    profile_common = ET.SubElement(effect, qname("profile_COMMON"))
    technique = ET.SubElement(profile_common, qname("technique"), {"sid": "common"})
    lambert = ET.SubElement(technique, qname("lambert"))
    diffuse = ET.SubElement(lambert, qname("diffuse"))
    ET.SubElement(diffuse, qname("color")).text = "0.7 0.7 0.7 1.0"

    library_materials = ET.SubElement(root, qname("library_materials"))
    material = ET.SubElement(
        library_materials, qname("material"), {"id": material_id, "name": material_id}
    )
    ET.SubElement(material, qname("instance_effect"), {"url": f"#{effect_id}"})

    library_geometries = ET.SubElement(root, qname("library_geometries"))
    geometry = ET.SubElement(
        library_geometries, qname("geometry"), {"id": geometry_id, "name": geometry_id}
    )
    mesh_node = ET.SubElement(geometry, qname("mesh"))

    positions_id = "scene-positions"
    positions_array_id = "scene-positions-array"
    positions_source = ET.SubElement(mesh_node, qname("source"), {"id": positions_id})
    position_text = " ".join(f"{value:.9g}" for value in vertices.reshape(-1))
    ET.SubElement(
        positions_source,
        qname("float_array"),
        {"id": positions_array_id, "count": str(vertices.size)},
    ).text = position_text
    positions_technique = ET.SubElement(positions_source, qname("technique_common"))
    positions_accessor = ET.SubElement(
        positions_technique,
        qname("accessor"),
        {"source": f"#{positions_array_id}", "count": str(len(vertices)), "stride": "3"},
    )
    for axis in ("X", "Y", "Z"):
        ET.SubElement(positions_accessor, qname("param"), {"name": axis, "type": "float"})

    normals_id = "scene-normals"
    normals_array_id = "scene-normals-array"
    normals_source = ET.SubElement(mesh_node, qname("source"), {"id": normals_id})
    normal_text = " ".join(f"{value:.9g}" for value in normals.reshape(-1))
    ET.SubElement(
        normals_source,
        qname("float_array"),
        {"id": normals_array_id, "count": str(normals.size)},
    ).text = normal_text
    normals_technique = ET.SubElement(normals_source, qname("technique_common"))
    normals_accessor = ET.SubElement(
        normals_technique,
        qname("accessor"),
        {"source": f"#{normals_array_id}", "count": str(len(normals)), "stride": "3"},
    )
    for axis in ("X", "Y", "Z"):
        ET.SubElement(normals_accessor, qname("param"), {"name": axis, "type": "float"})

    vertices_id = "scene-vertices"
    vertices_node = ET.SubElement(mesh_node, qname("vertices"), {"id": vertices_id})
    ET.SubElement(vertices_node, qname("input"), {"semantic": "POSITION", "source": f"#{positions_id}"})

    triangles_node = ET.SubElement(
        mesh_node,
        qname("triangles"),
        {"count": str(len(triangles)), "material": material_id},
    )
    ET.SubElement(
        triangles_node, qname("input"), {"semantic": "VERTEX", "source": f"#{vertices_id}", "offset": "0"}
    )
    ET.SubElement(
        triangles_node, qname("input"), {"semantic": "NORMAL", "source": f"#{normals_id}", "offset": "1"}
    )
    triangle_indices: list[str] = []
    for triangle in triangles:
        for vertex_index in triangle:
            idx = int(vertex_index)
            triangle_indices.extend((str(idx), str(idx)))
    ET.SubElement(triangles_node, qname("p")).text = " ".join(triangle_indices)

    library_visual_scenes = ET.SubElement(root, qname("library_visual_scenes"))
    visual_scene = ET.SubElement(
        library_visual_scenes, qname("visual_scene"), {"id": "Scene", "name": "Scene"}
    )
    node = ET.SubElement(visual_scene, qname("node"), {"id": "scene-node", "name": "scene-node"})
    instance_geometry = ET.SubElement(node, qname("instance_geometry"), {"url": f"#{geometry_id}"})
    bind_material = ET.SubElement(instance_geometry, qname("bind_material"))
    technique_common = ET.SubElement(bind_material, qname("technique_common"))
    ET.SubElement(
        technique_common,
        qname("instance_material"),
        {"symbol": material_id, "target": f"#{material_id}"},
    )

    scene = ET.SubElement(root, qname("scene"))
    ET.SubElement(scene, qname("instance_visual_scene"), {"url": "#Scene"})

    tree = ET.ElementTree(root)
    try:
        ET.indent(tree, space="  ")
    except AttributeError:
        pass
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(output_path), encoding="utf-8", xml_declaration=True)


def write_binary_stl(
    mesh: o3d.geometry.TriangleMesh,
    output_path: pathlib.Path,
    *,
    double_sided: bool = False,
) -> None:
    if mesh.is_empty():
        raise ValueError("mesh is empty")

    vertices = np.asarray(mesh.vertices, dtype=np.float32)
    triangles = np.asarray(mesh.triangles, dtype=np.int32)
    if len(vertices) == 0 or len(triangles) == 0:
        raise ValueError("mesh has no triangles")

    triangle_count = len(triangles) * (2 if double_sided else 1)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    header = b"pcd_scene_tool.py".ljust(80, b" ")
    with output_path.open("wb") as stl_file:
        stl_file.write(header)
        stl_file.write(struct.pack("<I", triangle_count))
        for triangle in triangles:
            p1, p2, p3 = vertices[triangle[0]], vertices[triangle[1]], vertices[triangle[2]]
            normal = np.cross(p2 - p1, p3 - p1)
            norm = float(np.linalg.norm(normal))
            if norm > 1e-12:
                normal = normal / norm
            else:
                normal = np.zeros(3, dtype=np.float32)
            stl_file.write(
                struct.pack(
                    "<12fH",
                    float(normal[0]),
                    float(normal[1]),
                    float(normal[2]),
                    float(p1[0]),
                    float(p1[1]),
                    float(p1[2]),
                    float(p2[0]),
                    float(p2[1]),
                    float(p2[2]),
                    float(p3[0]),
                    float(p3[1]),
                    float(p3[2]),
                    0,
                )
            )
            if double_sided:
                reversed_normal = -normal
                stl_file.write(
                    struct.pack(
                        "<12fH",
                        float(reversed_normal[0]),
                        float(reversed_normal[1]),
                        float(reversed_normal[2]),
                        float(p1[0]),
                        float(p1[1]),
                        float(p1[2]),
                        float(p3[0]),
                        float(p3[1]),
                        float(p3[2]),
                        float(p2[0]),
                        float(p2[1]),
                        float(p2[2]),
                        0,
                    )
                )


def mesh_report(mesh: o3d.geometry.TriangleMesh, label: str) -> None:
    print(f"{label}:")
    print(f"  vertices:  {len(mesh.vertices)}")
    print(f"  triangles: {len(mesh.triangles)}")


def run_mesh_pipeline(args: argparse.Namespace) -> None:
    input_path = pathlib.Path(args.input).expanduser().resolve()
    output_dir = pathlib.Path(args.out_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    point_cloud = load_point_cloud(input_path)
    processed = prepare_point_cloud(
        point_cloud, voxel_size=args.voxel_size, nb_neighbors=args.nb_neighbors, std_ratio=args.std_ratio
    )

    spacing = estimate_spacing(processed)
    normal_radius = args.normal_radius
    if normal_radius <= 0.0:
        normal_radius = max(0.1, spacing * 6.0 if spacing > 0.0 else 0.25)
    estimate_normals(processed, normal_radius)

    if args.method == "poisson":
        mesh = reconstruct_mesh_poisson(
            processed,
            depth=args.poisson_depth,
            density_trim_quantile=args.density_trim_quantile,
            crop_margin=args.crop_margin,
        )
    else:
        mesh = reconstruct_mesh_bpa(processed, radius_scale=args.bpa_radius_scale)

    if mesh.is_empty():
        raise ValueError("reconstruction produced an empty mesh")

    mesh = postprocess_mesh(mesh, target_triangles=args.target_triangles)
    if mesh.is_empty():
        raise ValueError("mesh is empty after post-processing")

    basename = args.basename or input_path.stem
    clean_pcd_path = output_dir / f"{basename}_clean.pcd"
    ply_path = output_dir / f"{basename}.ply"
    obj_path = output_dir / f"{basename}.obj"
    dae_path = output_dir / f"{basename}.dae"
    stl_path = output_dir / f"{basename}.stl"
    visual_stl_path = output_dir / f"{basename}_visual.stl"

    if not o3d.io.write_point_cloud(str(clean_pcd_path), processed):
        raise RuntimeError(f"failed to write cleaned point cloud: {clean_pcd_path}")
    if not o3d.io.write_triangle_mesh(str(ply_path), mesh, write_ascii=False):
        raise RuntimeError(f"failed to write mesh: {ply_path}")
    if not o3d.io.write_triangle_mesh(str(obj_path), mesh, write_ascii=False):
        raise RuntimeError(f"failed to write mesh: {obj_path}")
    write_collada(mesh, dae_path)
    write_binary_stl(mesh, stl_path)
    write_binary_stl(mesh, visual_stl_path, double_sided=True)

    print(f"Input:             {input_path}")
    print(f"Output directory:  {output_dir}")
    print(f"Method:            {args.method}")
    print(f"Normal radius:     {normal_radius:.4f} m")
    if spacing > 0.0:
        print(f"Median spacing:    {spacing:.4f} m")
    print(f"Clean point cloud: {clean_pcd_path}")
    print(f"PLY mesh:          {ply_path}")
    print(f"OBJ mesh:          {obj_path}")
    print(f"DAE mesh:          {dae_path}")
    print(f"STL mesh:          {stl_path}")
    print(f"Visual STL mesh:   {visual_stl_path}")
    mesh_report(mesh, "Mesh stats")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect a PCD scene and build Gazebo mesh assets."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    inspect_parser = subparsers.add_parser(
        "inspect", help="print scene bounds and subt_planner parameter suggestions"
    )
    inspect_parser.add_argument("input", help="absolute or relative path to the .pcd file")
    inspect_parser.add_argument(
        "--trim-percent",
        type=nonnegative_float,
        default=0.5,
        help="discard this percentile on each axis before producing heuristic bounds",
    )
    inspect_parser.add_argument(
        "--margin-abs",
        type=nonnegative_float,
        default=2.0,
        help="minimum absolute padding added to suggested global bounds",
    )
    inspect_parser.add_argument(
        "--margin-ratio",
        type=nonnegative_float,
        default=0.05,
        help="relative padding added to suggested global bounds",
    )
    inspect_parser.add_argument(
        "--round-resolution",
        type=positive_float,
        default=0.5,
        help="round suggested bounds to this grid resolution",
    )

    mesh_parser = subparsers.add_parser(
        "mesh", help="clean the point cloud and export a mesh as PLY/OBJ/DAE"
    )
    mesh_parser.add_argument("input", help="absolute or relative path to the .pcd file")
    mesh_parser.add_argument(
        "--out-dir",
        default=".",
        help="directory for output assets",
    )
    mesh_parser.add_argument(
        "--basename",
        default="",
        help="basename for exported files; defaults to the input stem",
    )
    mesh_parser.add_argument(
        "--method",
        choices=("poisson", "bpa"),
        default="poisson",
        help="mesh reconstruction method",
    )
    mesh_parser.add_argument(
        "--voxel-size",
        type=nonnegative_float,
        default=0.1,
        help="downsample voxel size before reconstruction; 0 disables downsampling",
    )
    mesh_parser.add_argument(
        "--nb-neighbors",
        type=positive_int,
        default=20,
        help="neighbors for statistical outlier removal",
    )
    mesh_parser.add_argument(
        "--std-ratio",
        type=positive_float,
        default=2.0,
        help="standard-deviation ratio for statistical outlier removal",
    )
    mesh_parser.add_argument(
        "--normal-radius",
        type=nonnegative_float,
        default=0.0,
        help="normal estimation radius; 0 auto-selects from point spacing",
    )
    mesh_parser.add_argument(
        "--poisson-depth",
        type=positive_int,
        default=9,
        help="octree depth for Poisson reconstruction",
    )
    mesh_parser.add_argument(
        "--density-trim-quantile",
        type=nonnegative_float,
        default=0.02,
        help="drop the lowest-density Poisson vertices below this quantile",
    )
    mesh_parser.add_argument(
        "--crop-margin",
        type=nonnegative_float,
        default=1.0,
        help="extra crop margin around the cleaned point cloud",
    )
    mesh_parser.add_argument(
        "--bpa-radius-scale",
        type=positive_float,
        default=3.0,
        help="ball pivoting base radius multiplier against median spacing",
    )
    mesh_parser.add_argument(
        "--target-triangles",
        type=positive_int,
        default=250000,
        help="simplify down to this triangle budget if the mesh is denser",
    )

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        if args.command == "inspect":
            point_cloud = load_point_cloud(pathlib.Path(args.input).expanduser().resolve())
            print_inspection_report(
                point_cloud,
                trim_percent=args.trim_percent,
                margin_abs=args.margin_abs,
                margin_ratio=args.margin_ratio,
                round_resolution=args.round_resolution,
            )
        elif args.command == "mesh":
            run_mesh_pipeline(args)
        else:
            parser.error(f"unknown command: {args.command}")
    except Exception as exc:  # pragma: no cover - CLI error path
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
