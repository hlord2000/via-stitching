#!/usr/bin/env python3

import argparse
import math
import sys

import kipy
from kipy.board_types import BoardPolygon, Via
from kipy.errors import ApiError
from kipy.geometry import Box2, Vector2
from kipy.util import from_mm
from kipy.util.board_layer import BoardLayer


def get_polygon_area(polygon: BoardPolygon) -> float:
    """Calculates the area of a BoardPolygon using the shoelace formula."""
    # This assumes a simple polygon and ignores holes and arcs for area calculation.
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


def main():
    """Main function to run the via stitching script."""
    parser = argparse.ArgumentParser(
        description="Perform via stitching within the board outline's bounding box."
    )
    parser.add_argument(
        "--via-size", type=float, required=True, help="Diameter of the via pad in mm."
    )
    parser.add_argument(
        "--drill-size",
        type=float,
        required=True,
        help="Diameter of the via drill hole in mm.",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        required=True,
        help="Spacing between vias in mm.",
    )
    parser.add_argument(
        "--net-class",
        type=str,
        help="The net class to assign to the vias. If not found, vias will have no net.",
    )

    args = parser.parse_args()

    via_size_nm = from_mm(args.via_size)
    drill_size_nm = from_mm(args.drill_size)
    spacing_nm = from_mm(args.spacing)

    try:
        kicad = kipy.KiCad()
        board = kicad.get_board()
    except (ApiError, FileNotFoundError) as e:
        print(f"Error: Could not connect to KiCad. {e}", file=sys.stderr)
        sys.exit(1)

    # Find all shapes on Edge.Cuts layer
    edge_cuts_shapes = [
        shape for shape in board.get_shapes() if shape.layer == BoardLayer.BL_Edge_Cuts
    ]

    if not edge_cuts_shapes:
        print("Error: No shapes found on Edge.Cuts layer.", file=sys.stderr)
        sys.exit(1)

    # Find the largest BoardPolygon to use for the bounding box
    board_polygons = [
        shape for shape in edge_cuts_shapes if isinstance(shape, BoardPolygon)
    ]

    print(f"INFO: Found {len(edge_cuts_shapes)} shapes on Edge.Cuts layer.")
    print(f"INFO: Found {len(board_polygons)} BoardPolygon shapes.")

    if not board_polygons:
        print(
            "Error: No BoardPolygon found on Edge.Cuts layer. "
            "This script requires a closed polygon to determine the board outline.",
            file=sys.stderr,
        )
        sys.exit(1)

    outline_poly = max(board_polygons, key=get_polygon_area)
    print("INFO: Selected the largest polygon as the main outline.")

    # Find the target net
    target_net = None
    if args.net_class:
        nets_in_class = board.get_nets(netclass_filter=args.net_class)
        if not nets_in_class:
            print(
                f"Warning: No nets found in net class '{args.net_class}'. Vias will be created with no net.",
                file=sys.stderr,
            )
        else:
            target_net = nets_in_class[0]
            print(f"INFO: Found target net: {target_net.name}")

    # Calculate the bounding box of the main outline polygon. Vias will be placed
    # in a grid covering this entire box.
    bbox = board.get_item_bounding_box(outline_poly)
    if not bbox:
        print(
            "Error: Could not determine bounding box of the main outline polygon.",
            file=sys.stderr,
        )
        sys.exit(1)
    print(f"INFO: Calculated tiling bounding box: {bbox}")

    vias_to_create = []

    # Tile the bounding box with a staggered "checkerboard" grid
    y_coords = []
    current_y = bbox.pos.y
    while current_y <= bbox.pos.y + bbox.size.y:
        y_coords.append(current_y)
        current_y += spacing_nm

    x_coords = []
    current_x = bbox.pos.x
    while current_x <= bbox.pos.x + bbox.size.x:
        x_coords.append(current_x)
        current_x += spacing_nm

    total_grid_points = 0
    is_odd_row = False
    for y in y_coords:
        x_offset = spacing_nm / 2.0 if is_odd_row else 0.0
        for x in x_coords:
            pos = Vector2.from_xy(int(x + x_offset), int(y))
            total_grid_points += 1
            
            # The grid generation is already constrained by the bounding box (bbox).
            # By adding a via at every grid point, we are effectively using the
            # bounding box as the area for via stitching. For non-rectangular
            # boards, this will place vias outside the intended outline.
            via = Via()
            via.position = pos
            if target_net:
                via.net = target_net
            via.diameter = via_size_nm
            via.drill_diameter = drill_size_nm
            vias_to_create.append(via)
        is_odd_row = not is_odd_row

    print(f"INFO: Generated a grid of {total_grid_points} potential via locations over the bounding box.")


    if not vias_to_create:
        print("ERROR: No valid positions found for vias. Nothing to do.")
        print("DEBUG: This could be because the --spacing is too large for the board size.")
        return

    # Create vias in a single transaction
    commit = board.begin_commit()
    board.create_items(vias_to_create)
    commit_msg = f"Add {len(vias_to_create)} stitching vias"
    if target_net:
        commit_msg += f" to {args.net_class}"
    board.push_commit(commit, commit_msg)

    print(f"Successfully created {len(vias_to_create)} stitching vias.")


if __name__ == "__main__":
    main()
