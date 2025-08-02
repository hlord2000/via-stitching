#!/usr/bin/env python3

# Copyright The KiCad Developers
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the “Software”), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import re
from kipy import KiCad
from kipy.board_types import Via, BoardLayer
from kipy.geometry import Vector2, Box2
from kipy.util import from_mm
from kipy.errors import ApiError

if __name__=='__main__':
    try:
        kicad = KiCad()
        print(f"kicad-python API version: {kicad.get_api_version()}")
        print(f"KiCad version: {kicad.get_version()}")
        kicad.check_version()

        board = kicad.get_board()
        project = board.get_project()

        # Get default netclass for via sizes
        netclasses = project.get_net_classes()
        default_netclass = next((nc for nc in netclasses if nc.name == 'Default'), None)

        if default_netclass:
            via_diameter = default_netclass.via_diameter or from_mm(0.8)
            via_drill = default_netclass.via_drill or from_mm(0.4)
        else:
            print("Default netclass not found, using default via sizes.")
            via_diameter = from_mm(0.8)
            via_drill = from_mm(0.4)

        # Find Edge.Cuts bounding box by parsing the board file
        board_string = board.get_as_string()
        all_points = []

        # Find points from lines and arcs on Edge.Cuts
        line_arc_items = re.findall(r'\((?:gr_line|gr_arc).*?\(layer Edge\.Cuts\).*?\)', board_string, re.DOTALL)
        for item in line_arc_items:
            points = re.findall(r'\((?:start|end|mid)\s+([-\d\.]+)\s+([-\d\.]+)\)', item)
            for x_str, y_str in points:
                all_points.append(Vector2.from_xy(from_mm(float(x_str)), from_mm(float(y_str))))

        # Find points from polygons on Edge.Cuts
        poly_items = re.findall(r'\((?:gr_poly).*?\(layer Edge\.Cuts\).*?\)', board_string, re.DOTALL)
        for item in poly_items:
            points = re.findall(r'\(xy\s+([-\d\.]+)\s+([-\d\.]+)\)', item)
            for x_str, y_str in points:
                all_points.append(Vector2.from_xy(from_mm(float(x_str)), from_mm(float(y_str))))

        if not all_points:
            raise Exception("Could not find any points on Edge.Cuts layer. Board outline may be composed of unsupported shapes (e.g. circles).")

        # Calculate bounding box from all found points
        min_x = min(p.x for p in all_points)
        min_y = min(p.y for p in all_points)
        max_x = max(p.x for p in all_points)
        max_y = max(p.y for p in all_points)

        total_bbox = Box2.from_pos_size(
            Vector2.from_xy(min_x, min_y),
            Vector2.from_xy(max_x - min_x, max_y - min_y)
        )

        # Create a grid of vias
        vias_to_create = []
        grid_spacing = from_mm(2)  # 2 mm spacing

        start_x = total_bbox.pos.x
        start_y = total_bbox.pos.y
        end_x = total_bbox.pos.x + total_bbox.size.x
        end_y = total_bbox.pos.y + total_bbox.size.y

        for x in range(start_x, end_x, grid_spacing):
            for y in range(start_y, end_y, grid_spacing):
                via = Via()
                via.position = Vector2.from_xy(x, y)
                via.drill_diameter = via_drill
                via.diameter = via_diameter
                vias_to_create.append(via)

        created_vias = board.create_items(vias_to_create)
        print(f"Created {len(created_vias)} vias.")

    except Exception as e:
        print(f"An error occurred: {e}")

