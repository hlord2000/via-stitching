#!/usr/bin/env python3

import argparse
import math
import sys
from typing import List, Tuple

import kipy
from kipy.board import Board
from kipy.board_types import (BoardCircle, BoardPolygon, BoardRectangle,
                             BoardSegment, BoardShape, Pad, Track, Via)
from kipy.errors import ApiError
from kipy.geometry import Box2, Vector2
from kipy.util import from_mm
from kipy.util.board_layer import BoardLayer


def get_outline_segments(board: Board, shapes: List[BoardShape]) -> List[Tuple[Vector2, Vector2]]:
    """
    Extracts all line segments from a list of board shapes.
    Uses get_item_bounding_box for reliable geometry of rectangles and circles.
    """
    segments = []
    for shape in shapes:
        if isinstance(shape, BoardPolygon):
            if not shape.polygons or not shape.polygons[0].outline.nodes:
                continue
            points = [node.point for node in shape.polygons[0].outline if node.has_point]
            if len(points) >= 2:
                for i in range(len(points)):
                    segments.append((points[i], points[(i + 1) % len(points)]))
        
        elif isinstance(shape, BoardRectangle):
            # Use the item's bounding box, which is the most reliable way to get its geometry.
            bbox = board.get_item_bounding_box(shape)
            if bbox:
                p1 = bbox.pos
                p3 = Vector2.from_xy(bbox.pos.x + bbox.size.x, bbox.pos.y + bbox.size.y)
                p2 = Vector2.from_xy(p3.x, p1.y)
                p4 = Vector2.from_xy(p1.x, p3.y)
                segments.extend([(p1, p2), (p2, p3), (p3, p4), (p4, p1)])
            else:
                 print(f"Warning: Could not get bounding box for a BoardRectangle on Edge.Cuts. Skipping.", file=sys.stderr)

        elif isinstance(shape, BoardSegment):
            segments.append((shape.start, shape.end))

        elif isinstance(shape, BoardCircle):
            # Approximate circle by getting its bounding box to find center and radius.
            bbox = board.get_item_bounding_box(shape)
            if bbox:
                center = bbox.center()
                radius = bbox.size.x / 2.0
                num_segments = 64
                points = []
                for i in range(num_segments):
                    angle = 2 * math.pi * i / num_segments
                    x = center.x + radius * math.cos(angle)
                    y = center.y + radius * math.sin(angle)
                    points.append(Vector2.from_xy(int(x), int(y)))
                
                for i in range(num_segments):
                    segments.append((points[i], points[(i + 1) % num_segments]))
            else:
                print(f"Warning: Could not get bounding box for a BoardCircle on Edge.Cuts. Skipping.", file=sys.stderr)

    return segments


def get_enclosing_bbox(board: Board, shapes: List[BoardShape]) -> Box2 | None:
    """Calculates the single bounding box that encloses a list of shapes."""
    if not shapes:
        return None
    
    bboxes = board.get_item_bounding_box(shapes)
    valid_bboxes = [b for b in bboxes if b]
    if not valid_bboxes:
        return None

    min_x = min(b.pos.x for b in valid_bboxes)
    min_y = min(b.pos.y for b in valid_bboxes)
    max_x = max(b.pos.x + b.size.x for b in valid_bboxes)
    max_y = max(b.pos.y + b.size.y for b in valid_bboxes)
    
    # Create the kipy wrapper objects
    position_wrapper = Vector2.from_xy(min_x, min_y)
    size_wrapper = Vector2.from_xy(max_x - min_x, max_y - min_y)
    
    # FIX: Pass the underlying .proto attribute to the Box2 constructor
    return Box2(position_wrapper.proto, size_wrapper.proto)


def is_point_inside_outline(point: Vector2, segments: List[Tuple[Vector2, Vector2]]) -> bool:
    """
    Determines if a point is inside a shape defined by segments using the Ray Casting algorithm.
    """
    intersections = 0
    for start, end in segments:
        if start.y == end.y: continue
        if min(start.y, end.y) < point.y <= max(start.y, end.y):
            x_intersect = (point.y - start.y) * (end.x - start.x) / (end.y - start.y) + start.x
            if x_intersect > point.x:
                intersections += 1
    return intersections % 2 == 1


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

    # --- Identify Board Outline from all relevant shapes on Edge.Cuts ---
    outline_shapes = [
        s for s in board.get_shapes()
        if s.layer == BoardLayer.BL_Edge_Cuts and isinstance(s, (BoardPolygon, BoardRectangle, BoardSegment, BoardCircle))
    ]
    if not outline_shapes:
        print("Error: No Polygons, Rectangles, Segments, or Circles found on Edge.Cuts layer.", file=sys.stderr)
        sys.exit(1)
    print(f"INFO: Found {len(outline_shapes)} shapes on Edge.Cuts to define board outline.")

    outline_segments = get_outline_segments(board, outline_shapes)
    if not outline_segments:
        print("Error: Could not extract any line segments from the Edge.Cuts shapes.", file=sys.stderr)
        sys.exit(1)

    bbox = get_enclosing_bbox(board, outline_shapes)
    if not bbox:
        print("Error: Could not determine bounding box of the outline shapes.", file=sys.stderr)
        sys.exit(1)
    print(f"INFO: Calculated tiling bounding box: {bbox}")

    target_net = None
    if args.net_class:
        nets_in_class = board.get_nets(netclass_filter=args.net_class)
        if nets_in_class:
            target_net = nets_in_class[0]
            print(f"INFO: Found target net: {target_net.name}")
        else:
            print(f"Warning: No nets in class '{args.net_class}'. Vias will have no net.", file=sys.stderr)
            
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
    
    safe_bbox_width = bbox.size.x
    safe_bbox_height = bbox.size.y

    if safe_bbox_width < via_size_nm or safe_bbox_height < via_size_nm:
        print("Warning: Board area is too small for via size.", file=sys.stderr)
    else:
        half_cols = int(safe_bbox_width / (2.0 * spacing_nm))
        half_rows = int(safe_bbox_height / (2.0 * spacing_nm))
        clearance_radius = via_radius_nm + clearance_nm

        for i in range(-half_rows, half_rows + 1):
            for j in range(-half_cols, half_cols + 1):
                pos = Vector2.from_xy(int(center.x + j * spacing_nm), int(center.y + i * spacing_nm))
                
                if not is_point_inside_outline(pos, outline_segments):
                    continue

                min_dist_to_edge = min(point_segment_distance(pos, start, end) for start, end in outline_segments)
                if min_dist_to_edge < via_radius_nm + edge_clearance_nm:
                    continue
                
                is_valid = True
                for v in obstacles['vias']:
                    if (target_net and v.net == target_net): continue
                    if (pos - v.position).length() < via_radius_nm + (v.diameter / 2.0) + clearance_nm:
                        is_valid = False; break
                if not is_valid: continue

                for t in obstacles['tracks']:
                    if (target_net and t.net == target_net): continue
                    if point_segment_distance(pos, t.start, t.end) < via_radius_nm + (t.width / 2.0) + clearance_nm:
                        is_valid = False; break
                if not is_valid: continue

                clearance_radius_int = int(clearance_radius)
                test_points = [
                    pos,
                    pos + Vector2.from_xy(0, clearance_radius_int), pos + Vector2.from_xy(0, -clearance_radius_int),
                    pos + Vector2.from_xy(-clearance_radius_int, 0), pos + Vector2.from_xy(clearance_radius_int, 0)
                ]
                for p in obstacles['pads']:
                    if (target_net and p.net == target_net): continue
                    for test_point in test_points:
                        if board.hit_test(p, test_point):
                            is_valid = False; break
                    if not is_valid: break
                if not is_valid: continue

                via = Via()
                via.position = pos
                if target_net: via.net = target_net
                via.diameter = via_size_nm
                via.drill_diameter = drill_size_nm
                vias_to_create.append(via)

    if not vias_to_create:
        print("ERROR: No valid positions found for vias. Nothing to do.", file=sys.stderr)
        return

    commit = board.begin_commit()
    board.create_items(vias_to_create)
    commit_msg = f"Add {len(vias_to_create)} stitching vias"
    if target_net: commit_msg += f" to {args.net_class}"
    board.push_commit(commit, commit_msg)

    print(f"✅ Successfully created {len(vias_to_create)} stitching vias.")


if __name__ == "__main__":
    main()
