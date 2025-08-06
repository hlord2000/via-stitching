#!/usr/bin/env python3
#
# Utility functions for Via Stitching and Fencing for KiCad.

import math
import kipy
from kipy.board import Board
from kipy.board_types import (BoardCircle, BoardPolygon, BoardRectangle,
                             BoardSegment, BoardShape, Pad, Track, Via, ArcTrack, Zone)
from kipy.geometry import Box2, Vector2, PolyLine
from kipy.geometry import normalize_angle_radians

def get_polygon_segments(poly: PolyLine) -> list[tuple[Vector2, Vector2]]:
    """Converts a PolyLine (with points and arcs) into a list of straight line segments."""
    segments = []
    if not poly.nodes:
        return segments

    points = []
    for node in poly.nodes:
        if node.has_point:
            points.append(node.point)
        elif node.has_arc:
            arc = node.arc
            center = arc.center()
            if not center:
                points.append(arc.start)
                continue

            radius = arc.radius()
            start_angle = arc.start_angle()
            arc_angle_val = arc.angle()

            if start_angle is None or arc_angle_val is None:
                points.append(arc.start)
                continue

            # Tessellate arc into segments
            num_steps = max(2, int(abs(arc_angle_val) / (math.pi / 18))) # ~10 deg steps
            for i in range(num_steps + 1):
                angle = start_angle + (arc_angle_val * i) / num_steps
                points.append(Vector2.from_xy(
                    int(center.x + radius * math.cos(angle)),
                    int(center.y + radius * math.sin(angle))
                ))

    if len(points) >= 2:
        for i in range(len(points) - 1):
            segments.append((points[i], points[i+1]))
        if poly.closed:
            segments.append((points[-1], points[0]))
            
    return segments

def get_outline_segments(board: Board, shapes: list[BoardShape]) -> list[tuple[Vector2, Vector2]]:
    """Get all segments from a list of shapes, tessellating curves."""
    segments = []
    for shape in shapes:
        if isinstance(shape, (Zone, BoardPolygon)):
            poly_with_holes = shape.outline if isinstance(shape, Zone) else shape.polygons[0]
            segments.extend(get_polygon_segments(poly_with_holes.outline))
            for hole in poly_with_holes.holes:
                segments.extend(get_polygon_segments(hole))
        elif isinstance(shape, BoardRectangle):
            bbox = board.get_item_bounding_box(shape)
            if bbox:
                p1, p3 = bbox.pos, Vector2.from_xy(bbox.pos.x + bbox.size.x, bbox.pos.y + bbox.size.y)
                p2, p4 = Vector2.from_xy(p3.x, p1.y), Vector2.from_xy(p1.x, p3.y)
                segments.extend([(p1, p2), (p2, p3), (p3, p4), (p4, p1)])
        elif isinstance(shape, BoardSegment):
            segments.append((shape.start, shape.end))
        elif isinstance(shape, BoardCircle):
            bbox = board.get_item_bounding_box(shape)
            if bbox:
                center, radius = bbox.center(), bbox.size.x / 2.0
                points = [Vector2.from_xy(int(center.x + radius * math.cos(2*math.pi*i/64)), int(center.y + radius * math.sin(2*math.pi*i/64))) for i in range(64)]
                for i in range(len(points)):
                    segments.append((points[i], points[(i + 1) % len(points)]))
    return segments

def get_enclosing_bbox(board: Board, shapes: list[BoardShape]) -> Box2 | None:
    """Get the enclosing bounding box for a list of shapes."""
    if not shapes: return None
    
    bboxes = []
    for shape in shapes:
        if isinstance(shape, Zone):
             # Use filled polygons for bbox calculation if available and filled
            if shape.filled and shape.filled_polygons:
                for layer_polys in shape.filled_polygons.values():
                    for poly in layer_polys:
                        bboxes.append(poly.bounding_box())
            else: # Fallback to outline
                bboxes.append(shape.outline.bounding_box())
        elif isinstance(shape, BoardPolygon):
            if shape.polygons:
                bboxes.append(shape.polygons[0].bounding_box())
        else:
            item_bbox = board.get_item_bounding_box(shape)
            if item_bbox:
                bboxes.append(item_bbox)

    if not bboxes: return None

    final_bbox = bboxes[0]
    for bbox in bboxes[1:]:
        final_bbox.merge(bbox)
        
    return final_bbox

def is_point_inside_polygon_with_holes(point: Vector2, poly_with_holes: 'PolygonWithHoles') -> bool:
    """Checks if a point is inside a polygon with holes."""
    outline_segments = get_polygon_segments(poly_with_holes.outline)
    if not is_point_inside_segments(point, outline_segments):
        return False
    for hole in poly_with_holes.holes:
        hole_segments = get_polygon_segments(hole)
        if is_point_inside_segments(point, hole_segments):
            return False
    return True

def is_point_inside_segments(point: Vector2, segments: list[tuple[Vector2, Vector2]]) -> bool:
    """Checks if a point is inside a shape defined by segments using the ray casting algorithm."""
    intersections = 0
    for start, end in segments:
        if start.y == end.y: continue
        if min(start.y, end.y) < point.y <= max(start.y, end.y):
            if (point.x < (end.x - start.x) * (point.y - start.y) / (end.y - start.y) + start.x):
                intersections += 1
    return intersections % 2 == 1

def point_segment_distance(p: Vector2, a: Vector2, b: Vector2) -> float:
    """Calculates the shortest distance from a point to a line segment."""
    ab, ap = b - a, p - a
    l2 = ab.x**2 + ab.y**2
    if l2 == 0.0: return ap.length()
    t = max(0, min(1, (ap.x * ab.x + ap.y * ab.y) / l2))
    projection = a + ab * t
    return (p - projection).length()

def point_arc_distance(p: Vector2, arc: ArcTrack) -> float:
    """Calculates the shortest distance from a point to an arc track."""
    center = arc.center()
    if not center:
        return point_segment_distance(p, arc.start, arc.end)

    radius = arc.radius()
    cp = p - center
    dist_to_center = cp.length()

    start_angle = normalize_angle_radians((arc.start - center).angle())
    p_angle = normalize_angle_radians(cp.angle())
    arc_total_angle = arc.angle()

    if arc_total_angle is None:
        return point_segment_distance(p, arc.start, arc.end)

    delta_angle = normalize_angle_radians(p_angle - start_angle)

    if 0 <= delta_angle <= arc_total_angle:
        return abs(dist_to_center - radius)
    else:
        return min((p - arc.start).length(), (p - arc.end).length())

def point_polygon_distance(p: Vector2, poly: 'PolygonWithHoles') -> float:
    """Calculates the shortest distance from a point to a polygon's boundary."""
    outline_segments = get_polygon_segments(poly.outline)

    min_dist = min((point_segment_distance(p, s, e) for s, e in outline_segments), default=float('inf'))

    for hole in poly.holes:
        hole_segments = get_polygon_segments(hole)
        min_dist = min(min_dist, min((point_segment_distance(p, s, e) for s, e in hole_segments), default=float('inf')))

    return min_dist
