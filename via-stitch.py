#!/usr/bin/env python3

import argparse
import math
import sys
from typing import List

import kipy
from kipy.board_types import BoardPolygon, Pad, Track, Via
from kipy.errors import ApiError
from kipy.geometry import Box2, Vector2
from kipy.util import from_mm
from kipy.util.board_layer import BoardLayer


def get_polygon_area(polygon: BoardPolygon) -> float:
    """Calculates the area of a BoardPolygon using the shoelace formula."""
    if not polygon.polygons or not polygon.polygons[0].outline.nodes:
        return 0.0
    points = [node.point for node in polygon.polygons[0].outline if node.has_point]
    if len(points) < 3:
        return 0.0
    n = len(points)
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += points[i].x * points[j].y
        area -= points[j].x * points[i].y
    return abs(area) / 2.0


def point_segment_distance(p: Vector2, a: Vector2, b: Vector2) -> float:
    """Calculates the minimum distance from point p to the line segment [a, b]."""
    ab = b - a
    ap = p - a
    l2 = ab.x * ab.x + ab.y * ab.y
    if l2 == 0.0:
        return ap.length()
    t = max(0, min(1, (ap.x * ab.x + ap.y * ab.y) / l2))
    projection = a + ab * t
    return (p - projection).length()


def main():
    """Main function to run the via stitching script."""
    parser = argparse.ArgumentParser(
        description="Perform via stitching with collision avoidance."
    )
    # Required arguments
    parser.add_argument("--via-size", type=float, required=True, help="Diameter of the via pad in mm.")
    parser.add_argument("--drill-size", type=float, required=True, help="Diameter of the via drill hole in mm.")
    parser.add_argument("--spacing", type=float, required=True, help="Spacing between vias in mm.")
    parser.add_argument("--clearance", type=float, required=True, help="Minimum clearance from the new via pad edge to other copper objects, in mm.")
    # Optional arguments
    parser.add_argument("--net-class", type=str, help="The net class to assign to the vias. If not found, vias will have no net.")
    parser.add_argument("--edge-clearance", type=float, default=0.5, help="Minimum clearance from the board edge to the via pad edge, in mm. Default is 0.5mm.")

    args = parser.parse_args()

    via_size_nm = from_mm(args.via_size)
    drill_size_nm = from_mm(args.drill_size)
    spacing_nm = from_mm(args.spacing)
    edge_clearance_nm = from_mm(args.edge_clearance)
    clearance_nm = from_mm(args.clearance)
    via_radius_nm = via_size_nm / 2.0

    try:
        kicad = kipy.KiCad()
        board = kicad.get_board()
    except (ApiError, FileNotFoundError) as e:
        print(f"Error: Could not connect to KiCad. {e}", file=sys.stderr)
        sys.exit(1)

    outline_poly = max(
        (s for s in board.get_shapes() if isinstance(s, BoardPolygon) and s.layer == BoardLayer.BL_Edge_Cuts),
        key=get_polygon_area,
        default=None,
    )
    if not outline_poly:
        print("Error: No BoardPolygon found on Edge.Cuts layer.", file=sys.stderr)
        sys.exit(1)
    print("INFO: Selected the largest polygon as the main outline.")

    target_net = None
    if args.net_class:
        nets_in_class = board.get_nets(netclass_filter=args.net_class)
        if nets_in_class:
            target_net = nets_in_class[0]
            print(f"INFO: Found target net: {target_net.name}")
        else:
            print(f"Warning: No nets in class '{args.net_class}'. Vias will have no net.", file=sys.stderr)

    bbox = board.get_item_bounding_box(outline_poly)
    if not bbox:
        print("Error: Could not determine bounding box of the outline polygon.", file=sys.stderr)
        sys.exit(1)
    print(f"INFO: Calculated tiling bounding box: {bbox}")

    # --- Cache Obstacles for Collision Detection ---
    print("INFO: Caching board objects for collision detection...")
    obstacles = { 'pads': [], 'tracks': [], 'vias': [] }
    for fp in board.get_footprints():
        obstacles['pads'].extend(fp.definition.pads)
    obstacles['tracks'].extend(board.get_tracks())
    obstacles['vias'].extend(board.get_vias())
    print(f"INFO: Cached {len(obstacles['pads'])} pads, {len(obstacles['tracks'])} tracks, and {len(obstacles['vias'])} vias.")

    # --- Generate Via Grid ---
    vias_to_create = []
    center = bbox.center()
    
    safe_bbox_width = bbox.size.x - 2 * edge_clearance_nm
    safe_bbox_height = bbox.size.y - 2 * edge_clearance_nm

    if safe_bbox_width < via_size_nm or safe_bbox_height < via_size_nm:
        print("Warning: Board area is too small for via size and edge clearance.", file=sys.stderr)
    else:
        half_cols = int(safe_bbox_width / (2.0 * spacing_nm))
        half_rows = int(safe_bbox_height / (2.0 * spacing_nm))
        clearance_radius = via_radius_nm + clearance_nm

        for i in range(-half_rows, half_rows + 1):
            for j in range(-half_cols, half_cols + 1):
                pos = Vector2.from_xy(int(center.x + j * spacing_nm), int(center.y + i * spacing_nm))
                is_valid = True

                # 1. Check Edge Clearance
                if not (pos.x - via_radius_nm >= bbox.pos.x + edge_clearance_nm and
                        pos.x + via_radius_nm <= bbox.pos.x + bbox.size.x - edge_clearance_nm and
                        pos.y - via_radius_nm >= bbox.pos.y + edge_clearance_nm and
                        pos.y + via_radius_nm <= bbox.pos.y + bbox.size.y - edge_clearance_nm):
                    continue

                # 2. Check Collision with other objects
                # Vias (distance check)
                for v in obstacles['vias']:
                    if (target_net and v.net == target_net): continue
                    if (pos - v.position).length() < via_radius_nm + (v.diameter / 2.0) + clearance_nm:
                        is_valid = False; break
                if not is_valid: continue

                # Tracks (distance check)
                for t in obstacles['tracks']:
                    if (target_net and t.net == target_net): continue
                    if point_segment_distance(pos, t.start, t.end) < via_radius_nm + (t.width / 2.0) + clearance_nm:
                        is_valid = False; break
                if not is_valid: continue

                # Pads (hit_test check)
                clearance_radius_int = int(clearance_radius)
                test_points = [
                    pos,
                    pos + Vector2.from_xy(0, clearance_radius_int),
                    pos + Vector2.from_xy(0, -clearance_radius_int),
                    pos + Vector2.from_xy(-clearance_radius_int, 0),
                    pos + Vector2.from_xy(clearance_radius_int, 0)
                ]
                for p in obstacles['pads']:
                    if (target_net and p.net == target_net): continue
                    for test_point in test_points:
                        if board.hit_test(p, test_point):
                            is_valid = False; break
                    if not is_valid: break
                if not is_valid: continue

                # If all checks pass, create the via
                via = Via()
                via.position = pos
                if target_net: via.net = target_net
                via.diameter = via_size_nm
                via.drill_diameter = drill_size_nm
                vias_to_create.append(via)

    if not vias_to_create:
        print("ERROR: No valid positions found for vias. Nothing to do.", file=sys.stderr)
        return

    # Create vias in a single transaction
    commit = board.begin_commit()
    board.create_items(vias_to_create)
    commit_msg = f"Add {len(vias_to_create)} stitching vias"
    if target_net: commit_msg += f" to {args.net_class}"
    board.push_commit(commit, commit_msg)

    print(f"✅ Successfully created {len(vias_to_create)} stitching vias.")

if __name__ == "__main__":
    main()
