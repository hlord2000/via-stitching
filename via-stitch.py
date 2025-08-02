#!/usr/bin/env python3
#
# Via Stitching GUI for KiCad

import wx
import sys
import math
import logging
import random
import kipy
from kipy.board import Board
from kipy.board_types import (BoardCircle, BoardPolygon, BoardRectangle,
                             BoardSegment, BoardShape, Pad, Track, Via, ArcTrack, Zone, BoardText, BoardTextBox)
from kipy.errors import ApiError
from kipy.geometry import Box2, Vector2
from kipy.util import from_mm
from kipy.util.board_layer import BoardLayer, is_copper_layer
from kipy.geometry import normalize_angle_radians
from kipy.proto.common.types import KiCadObjectType

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# --- Backend Logic (Helper Functions) ---

def get_outline_segments(board: Board, shapes: list[BoardShape]) -> list[tuple[Vector2, Vector2]]:
    segments = []
    for shape in shapes:
        if isinstance(shape, BoardPolygon):
            if shape.polygons and shape.polygons[0].outline.nodes:
                points = [node.point for node in shape.polygons[0].outline if node.has_point]
                if len(points) >= 2:
                    for i in range(len(points)):
                        segments.append((points[i], points[(i + 1) % len(points)]))
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
    if not shapes: return None
    bboxes = [b for b in board.get_item_bounding_box(shapes) if b]
    if not bboxes: return None
    min_x, min_y = min(b.pos.x for b in bboxes), min(b.pos.y for b in bboxes)
    max_x, max_y = max(b.pos.x + b.size.x for b in bboxes), max(b.pos.y + b.size.y for b in bboxes)
    pos = Vector2.from_xy(min_x, min_y)
    size = Vector2.from_xy(max_x - min_x, max_y - min_y)
    return Box2(pos.proto, size.proto)

def is_point_inside_outline(point: Vector2, segments: list[tuple[Vector2, Vector2]]) -> bool:
    intersections = 0
    for start, end in segments:
        if start.y == end.y: continue
        if min(start.y, end.y) < point.y <= max(start.y, end.y):
            if (point.x < (end.x - start.x) * (point.y - start.y) / (end.y - start.y) + start.x):
                intersections += 1
    return intersections % 2 == 1

def point_segment_distance(p: Vector2, a: Vector2, b: Vector2) -> float:
    ab, ap = b - a, p - a
    l2 = ab.x**2 + ab.y**2
    if l2 == 0.0: return ap.length()
    t = max(0, min(1, (ap.x * ab.x + ap.y * ab.y) / l2))
    projection = a + ab * t
    return (p - projection).length()

def point_arc_distance(p: Vector2, arc: ArcTrack) -> float:
    center = arc.center()
    if not center:
        return point_segment_distance(p, arc.start, arc.end)

    radius = arc.radius()
    cp = p - center
    dist_to_center = cp.length()

    start_angle = normalize_angle_radians((arc.start - center).angle())
    p_angle = normalize_angle_radians(cp.angle())
    arc_total_angle = arc.angle()

    if arc_total_angle is None: # Should not happen if center is valid
        return point_segment_distance(p, arc.start, arc.end)

    delta_angle = normalize_angle_radians(p_angle - start_angle)

    if 0 <= delta_angle <= arc_total_angle:
        return abs(dist_to_center - radius)
    else:
        return min((p - arc.start).length(), (p - arc.end).length())

def point_polygon_distance(p: Vector2, poly: BoardPolygon) -> float:
    segments = []
    if poly.polygons and poly.polygons[0].outline.nodes:
        points = [node.point for node in poly.polygons[0].outline if node.has_point]
        if len(points) >= 2:
            for i in range(len(points)):
                segments.append((points[i], points[(i + 1) % len(points)]))

    if not segments:
        return float('inf')

    # Simple check for holes is omitted for performance.
    if is_point_inside_outline(p, segments):
        return 0.0

    return min(point_segment_distance(p, s, e) for s, e in segments)


# --- GUI Dialog Definition ---

class StitcherDialog(wx.Dialog):
    def __init__(self, parent):
        super(StitcherDialog, self).__init__(parent, title="Via Stitching Tool", style=wx.DEFAULT_DIALOG_STYLE | wx.STAY_ON_TOP)
        self.kicad, self.board = None, None
        top_sizer = wx.BoxSizer(wx.VERTICAL)
        panel = wx.Panel(self)
        grid_sizer = wx.GridBagSizer(5, 5)
        self.m_sizing_mode = wx.RadioBox(panel, label="Sizing Mode", choices=["Manual", "From Netclass"])
        grid_sizer.Add(self.m_sizing_mode, pos=(0, 0), span=(1, 2), flag=wx.EXPAND | wx.BOTTOM, border=5)
        self.m_via_size_text = wx.TextCtrl(panel, value="0.8")
        grid_sizer.Add(wx.StaticText(panel, label="Via Size (mm)"), pos=(1, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_via_size_text, pos=(1, 1), flag=wx.EXPAND)
        self.m_drill_size_text = wx.TextCtrl(panel, value="0.4")
        grid_sizer.Add(wx.StaticText(panel, label="Drill Size (mm)"), pos=(2, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_drill_size_text, pos=(2, 1), flag=wx.EXPAND)
        self.m_netclass_choice = wx.Choice(panel)
        grid_sizer.Add(wx.StaticText(panel, label="Source Netclass"), pos=(3, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_netclass_choice, pos=(3, 1), flag=wx.EXPAND)
        grid_sizer.Add(wx.StaticLine(panel), pos=(4, 0), span=(1, 2), flag=wx.EXPAND | wx.ALL, border=5)
        self.m_spacing_text = wx.TextCtrl(panel, value="5")
        grid_sizer.Add(wx.StaticText(panel, label="Spacing (mm)"), pos=(5, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_spacing_text, pos=(5, 1), flag=wx.EXPAND)
        self.m_clearance_text = wx.TextCtrl(panel, value="5")
        grid_sizer.Add(wx.StaticText(panel, label="Copper Clearance (mm)"), pos=(6, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_clearance_text, pos=(6, 1), flag=wx.EXPAND)
        self.m_edge_clearance_text = wx.TextCtrl(panel, value="5")
        grid_sizer.Add(wx.StaticText(panel, label="Edge Clearance (mm)"), pos=(7, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_edge_clearance_text, pos=(7, 1), flag=wx.EXPAND)
        
        self.m_row_offset_text = wx.TextCtrl(panel, value="2.5")
        grid_sizer.Add(wx.StaticText(panel, label="Row Offset (mm)"), pos=(8, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_row_offset_text, pos=(8, 1), flag=wx.EXPAND)

        self.m_random_variance_checkbox = wx.CheckBox(panel, label="Add Random Variance")
        grid_sizer.Add(self.m_random_variance_checkbox, pos=(9, 0), span=(1, 2), flag=wx.EXPAND | wx.TOP, border=5)

        self.m_max_x_variance_text = wx.TextCtrl(panel, value="0.5")
        grid_sizer.Add(wx.StaticText(panel, label="Max X Variance (mm)"), pos=(10, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_max_x_variance_text, pos=(10, 1), flag=wx.EXPAND)

        self.m_max_y_variance_text = wx.TextCtrl(panel, value="0.5")
        grid_sizer.Add(wx.StaticText(panel, label="Max Y Variance (mm)"), pos=(11, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_max_y_variance_text, pos=(11, 1), flag=wx.EXPAND)

        grid_sizer.Add(wx.StaticLine(panel), pos=(12, 0), span=(1, 2), flag=wx.EXPAND | wx.ALL, border=5)
        self.m_net_choice = wx.Choice(panel)
        grid_sizer.Add(wx.StaticText(panel, label="Assign to Net"), pos=(13, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_net_choice, pos=(13, 1), flag=wx.EXPAND)
        grid_sizer.AddGrowableCol(1)
        panel.SetSizer(grid_sizer)
        top_sizer.Add(panel, 1, wx.EXPAND | wx.ALL, border=5)
        self.m_stitch_button = wx.Button(self, id=wx.ID_OK, label="Stitch Vias")
        self.m_close_button = wx.Button(self, id=wx.ID_CANCEL, label="Close")
        self.m_stitch_button.SetDefault()
        button_sizer = wx.StdDialogButtonSizer()
        button_sizer.AddButton(self.m_stitch_button)
        button_sizer.AddButton(self.m_close_button)
        button_sizer.Realize()
        top_sizer.Add(button_sizer, 0, wx.ALIGN_RIGHT | wx.ALL, border=10)
        self.SetSizerAndFit(top_sizer)
        self.Bind(wx.EVT_RADIOBOX, self.on_sizing_mode_change, self.m_sizing_mode)
        self.Bind(wx.EVT_CHECKBOX, self.on_random_variance_change, self.m_random_variance_checkbox)
        self.Bind(wx.EVT_BUTTON, self.on_stitch, self.m_stitch_button)
        self.m_stitch_button.Disable()
        self.populate_data()
        self.on_sizing_mode_change(None)
        self.on_random_variance_change(None)
        self.Center()

    def on_random_variance_change(self, event):
        is_random = self.m_random_variance_checkbox.GetValue()
        self.m_max_x_variance_text.Enable(is_random)
        self.m_max_y_variance_text.Enable(is_random)

    def on_sizing_mode_change(self, event):
        is_manual = self.m_sizing_mode.GetSelection() == 0
        self.m_via_size_text.Enable(is_manual)
        self.m_drill_size_text.Enable(is_manual)
        self.m_netclass_choice.Enable(not is_manual)

    def populate_data(self):
        try:
            self.kicad, self.board = kipy.KiCad(), kipy.KiCad().get_board()
            all_nets = sorted(self.board.get_nets(), key=lambda n: n.code)
            net_names = [n.name for n in all_nets if "unconnected" not in n.name.lower()]
            self.m_net_choice.Append("[No Net]"), self.m_net_choice.Append(net_names), self.m_net_choice.SetSelection(0)
            netclasses = []
            if all_nets:
                netclass_map = self.board.get_netclass_for_nets(all_nets)
                netclasses = sorted(list(set(nc.name for nc in netclass_map.values())))
            self.m_netclass_choice.Append(netclasses)
            if "Default" in netclasses: self.m_netclass_choice.SetStringSelection("Default")
            elif len(netclasses) > 0: self.m_netclass_choice.SetSelection(0)
            self.m_stitch_button.Enable()
        except (ApiError, FileNotFoundError) as e:
            wx.MessageBox(f"Could not connect to KiCad PCB Editor: {e}", "Connection Error", wx.OK | wx.ICON_ERROR, parent=self)
        except Exception as e:
            wx.MessageBox(f"An unexpected error occurred during startup: {e}", "Error", wx.OK | wx.ICON_ERROR, parent=self)

    def on_stitch(self, event):
        wx.BeginBusyCursor()
        self.m_stitch_button.Disable()
        try:
            params = {
                'spacing': float(self.m_spacing_text.GetValue()), 'clearance': float(self.m_clearance_text.GetValue()),
                'edge_clearance': float(self.m_edge_clearance_text.GetValue()),
                'net_name': self.m_net_choice.GetStringSelection() if self.m_net_choice.GetSelection() > 0 else None,
                'row_offset': float(self.m_row_offset_text.GetValue()),
                'random_variance': self.m_random_variance_checkbox.GetValue(),
                'max_x_variance': float(self.m_max_x_variance_text.GetValue()),
                'max_y_variance': float(self.m_max_y_variance_text.GetValue())
            }
            if self.m_sizing_mode.GetSelection() == 0:
                params.update({'via_size': float(self.m_via_size_text.GetValue()), 'drill_size': float(self.m_drill_size_text.GetValue()), 'netclass_name': None})
            else:
                params.update({'via_size': None, 'drill_size': None, 'netclass_name': self.m_netclass_choice.GetStringSelection()})
            
            vias_kept_count = self.perform_stitching(params)
            
            if vias_kept_count is not None:
                wx.MessageBox(f"✅ Successfully created {vias_kept_count} stitching vias.", "Success", wx.OK | wx.ICON_INFORMATION, parent=self)
                self.EndModal(wx.ID_OK)
        except ValueError:
            wx.MessageBox("Invalid input. Please ensure all number fields contain valid numbers.", "Input Error", wx.OK | wx.ICON_ERROR, parent=self)
        except Exception as e:
            logging.error("Stitching process failed.", exc_info=True)
            wx.MessageBox(f"An error occurred during stitching:\n{e}\n\nSee console for detailed log.", "Processing Error", wx.OK | wx.ICON_ERROR, parent=self)
        finally:
            self.m_stitch_button.Enable()
            wx.EndBusyCursor()

    def perform_stitching(self, params: dict) -> int | None:
        if params['via_size'] is not None:
            via_size_nm, drill_size_nm = from_mm(params['via_size']), from_mm(params['drill_size'])
        else:
            target_netclass_name = params['netclass_name']
            if not target_netclass_name: raise Exception("No netclass selected.")
            nets_in_class = self.board.get_nets(netclass_filter=target_netclass_name)
            if not nets_in_class: raise Exception(f"Netclass '{target_netclass_name}' not found.")
            netclass_obj = list(self.board.get_netclass_for_nets(nets_in_class[0]).values())[0]
            via_size_nm, drill_size_nm = netclass_obj.via_diameter, netclass_obj.via_drill
            if not via_size_nm or not drill_size_nm: raise Exception(f"Netclass '{target_netclass_name}' has invalid via dimensions.")

        spacing_nm = from_mm(params['spacing'])
        edge_clearance_nm = from_mm(params['edge_clearance'])
        clearance_nm = from_mm(params['clearance'])
        row_offset_nm = from_mm(params['row_offset'])
        random_variance = params['random_variance']
        max_x_variance_nm = from_mm(params['max_x_variance'])
        max_y_variance_nm = from_mm(params['max_y_variance'])
        via_radius_nm = via_size_nm / 2.0
        
        outline_shapes = [s for s in self.board.get_shapes() if s.layer == BoardLayer.BL_Edge_Cuts]
        if not outline_shapes: raise Exception("No valid board outline found.")
        outline_segments, bbox = get_outline_segments(self.board, outline_shapes), get_enclosing_bbox(self.board, outline_shapes)
        if not bbox: raise Exception("Could not determine board bounding box.")
        
        target_net = next((n for n in self.board.get_nets() if n.name == params['net_name']), None) if params['net_name'] else None
        
        # --- Collect all obstacles once ---
        obstacles = []
        all_items = self.board.get_items([
            KiCadObjectType.KOT_PCB_PAD,
            KiCadObjectType.KOT_PCB_VIA,
            KiCadObjectType.KOT_PCB_TRACE,
            KiCadObjectType.KOT_PCB_ARC,
            KiCadObjectType.KOT_PCB_ZONE,
            KiCadObjectType.KOT_PCB_SHAPE,
            KiCadObjectType.KOT_PCB_TEXT,
            KiCadObjectType.KOT_PCB_TEXTBOX,
        ])

        for item in all_items:
            if isinstance(item, (BoardShape, BoardText, BoardTextBox)) and not is_copper_layer(item.layer):
                continue
            
            net = getattr(item, 'net', None)

            if isinstance(item, Zone):
                if item.is_rule_area() or not item.filled: continue
                for layer, polygons in item.filled_polygons.items():
                    for poly in polygons:
                        obstacles.append({'item': poly, 'net': item.net, 'type': 'zone_poly'})
            else:
                obstacles.append({'item': item, 'net': net, 'type': item.__class__.__name__})

        # --- Generate candidate positions ---
        candidate_positions = []
        center = bbox.center()
        half_cols = int(bbox.size.x / (2.0 * spacing_nm)) if spacing_nm > 0 else 0
        half_rows = int(bbox.size.y / (2.0 * spacing_nm)) if spacing_nm > 0 else 0
        for i in range(-half_rows, half_rows + 1):
            row_x_offset = row_offset_nm if i % 2 != 0 else 0
            for j in range(-half_cols, half_cols + 1):
                x_pos = center.x + j * spacing_nm + row_x_offset
                y_pos = center.y + i * spacing_nm

                if random_variance:
                    x_pos += random.uniform(-max_x_variance_nm, max_x_variance_nm)
                    y_pos += random.uniform(-max_y_variance_nm, max_y_variance_nm)

                pos = Vector2.from_xy(int(x_pos), int(y_pos))
                if not is_point_inside_outline(pos, outline_segments): continue
                if min(point_segment_distance(pos, s, e) for s, e in outline_segments) < via_radius_nm + edge_clearance_nm: continue
                candidate_positions.append(pos)

        if not candidate_positions:
            raise Exception("No valid positions for vias found within board outline and edge clearance.")

        # --- Filter candidates and create valid vias ---
        valid_vias = []
        for pos in candidate_positions:
            is_valid = True
            # Check against spacing of already added valid vias
            for v in valid_vias:
                if (pos - v.position).length() < spacing_nm:
                    is_valid = False; break
            if not is_valid: continue

            # Check against all board obstacles
            for obs in obstacles:
                if target_net and obs['net'] and obs['net'].name == target_net.name:
                    continue

                item = obs['item']
                dist = float('inf')
                obs_clearance = 0

                if isinstance(item, Via):
                    dist = (pos - item.position).length()
                    obs_clearance = item.diameter / 2.0
                elif isinstance(item, Pad):
                    dist = (pos - item.position).length()
                    obs_clearance = max(item.padstack.copper_layer(BoardLayer.BL_F_Cu).size.x, item.padstack.copper_layer(BoardLayer.BL_F_Cu).size.y) / 2.0 if item.padstack.copper_layer(BoardLayer.BL_F_Cu) else 0
                elif isinstance(item, Track):
                    dist = point_segment_distance(pos, item.start, item.end)
                    obs_clearance = item.width / 2.0
                elif isinstance(item, ArcTrack):
                    dist = point_arc_distance(pos, item)
                    obs_clearance = item.width / 2.0
                elif isinstance(item, BoardShape):
                    # Simplified check for board shapes
                    bbox = self.board.get_item_bounding_box(item)
                    if bbox and bbox.center().length() > 0: # check if valid bbox
                        dist = (pos - bbox.center()).length()
                        obs_clearance = max(bbox.size.x, bbox.size.y) / 2.0
                elif obs['type'] == 'zone_poly':
                    # This is slow, only check if bbox overlaps
                    bbox = item.bounding_box()
                    if (pos - bbox.center()).length() < (via_radius_nm + max(bbox.size.x, bbox.size.y)/2.0 + clearance_nm):
                        dist = point_polygon_distance(pos, item)
                        obs_clearance = 0 # clearance is from edge

                if dist < via_radius_nm + obs_clearance + clearance_nm:
                    is_valid = False; break
            
            if is_valid:
                via = Via()
                via.position, via.diameter, via.drill_diameter = pos, via_size_nm, drill_size_nm
                if target_net: via.net = target_net
                valid_vias.append(via)

        if not valid_vias:
            return 0

        commit = self.board.begin_commit()
        self.board.create_items(valid_vias)
        commit_msg = f"Add {len(valid_vias)} stitching vias"
        if target_net: commit_msg += f" to net {params['net_name']}"
        self.board.push_commit(commit, commit_msg)
        
        return len(valid_vias)

if __name__ == "__main__":
    app = wx.App(False)
    dialog = StitcherDialog(None)
    dialog.ShowModal()
    dialog.Destroy()