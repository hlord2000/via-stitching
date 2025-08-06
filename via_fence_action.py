#!/usr/bin/env python3
#
# Via Fencing GUI for KiCad

import wx
import sys
import math
import logging
import random
import kipy
import argparse
from kipy.board import Board
from kipy.board_types import (Track, Via, ArcTrack, BoardSegment, Zone, Pad, ViaType, BoardShape, BoardText, BoardTextBox)
from kipy.errors import ApiError
from kipy.geometry import Vector2
from kipy.util import from_mm
from kipy.util.board_layer import BoardLayer, is_copper_layer
from kipy.proto.common.types import KiCadObjectType
from stitching_utils import *

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class FenceDialog(wx.Dialog):
    def __init__(self, parent):
        super(FenceDialog, self).__init__(parent, title="Via Fencing Tool", style=wx.DEFAULT_DIALOG_STYLE | wx.STAY_ON_TOP)
        self.debug_mode = False
        self.kicad, self.board = None, None
        top_sizer = wx.BoxSizer(wx.VERTICAL)
        panel = wx.Panel(self)
        grid_sizer = wx.GridBagSizer(5, 5)

        self.selection_info_text = wx.StaticText(panel, label="Select a trace or line to begin.")
        grid_sizer.Add(self.selection_info_text, pos=(0, 0), span=(1, 2), flag=wx.EXPAND | wx.BOTTOM, border=5)

        self.m_sizing_mode = wx.RadioBox(panel, label="Sizing Mode", choices=["Manual", "From Netclass"])
        grid_sizer.Add(self.m_sizing_mode, pos=(1, 0), span=(1, 2), flag=wx.EXPAND | wx.BOTTOM, border=5)
        self.m_via_size_text = wx.TextCtrl(panel, value="0.8")
        grid_sizer.Add(wx.StaticText(panel, label="Via Size (mm)"), pos=(2, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_via_size_text, pos=(2, 1), flag=wx.EXPAND)
        self.m_drill_size_text = wx.TextCtrl(panel, value="0.4")
        grid_sizer.Add(wx.StaticText(panel, label="Drill Size (mm)"), pos=(3, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_drill_size_text, pos=(3, 1), flag=wx.EXPAND)
        self.m_netclass_choice = wx.Choice(panel)
        grid_sizer.Add(wx.StaticText(panel, label="Source Netclass"), pos=(4, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_netclass_choice, pos=(4, 1), flag=wx.EXPAND)
        grid_sizer.Add(wx.StaticLine(panel), pos=(5, 0), span=(1, 2), flag=wx.EXPAND | wx.ALL, border=5)
        
        self.m_num_rows_text = wx.TextCtrl(panel, value="1")
        grid_sizer.Add(wx.StaticText(panel, label="Number of Rows"), pos=(6, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_num_rows_text, pos=(6, 1), flag=wx.EXPAND)
        self.m_distance_text = wx.TextCtrl(panel, value="1.0")
        grid_sizer.Add(wx.StaticText(panel, label="Distance from Trace (mm)"), pos=(7, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_distance_text, pos=(7, 1), flag=wx.EXPAND)
        self.m_spacing_text = wx.TextCtrl(panel, value="2.0")
        grid_sizer.Add(wx.StaticText(panel, label="Via Spacing (mm)"), pos=(8, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_spacing_text, pos=(8, 1), flag=wx.EXPAND)
        self.m_row_offset_text = wx.TextCtrl(panel, value="1.0")
        grid_sizer.Add(wx.StaticText(panel, label="Row Offset (mm)"), pos=(9, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_row_offset_text, pos=(9, 1), flag=wx.EXPAND)
        self.m_clearance_text = wx.TextCtrl(panel, value="0.2")
        grid_sizer.Add(wx.StaticText(panel, label="Clearance (mm)"), pos=(10, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_clearance_text, pos=(10, 1), flag=wx.EXPAND)

        self.m_diff_pair_checkbox = wx.CheckBox(panel, label="Fence as Differential Pair")
        grid_sizer.Add(self.m_diff_pair_checkbox, pos=(11, 0), span=(1, 2), flag=wx.EXPAND | wx.TOP, border=5)

        self.m_random_variance_checkbox = wx.CheckBox(panel, label="Add Random Variance")
        grid_sizer.Add(self.m_random_variance_checkbox, pos=(12, 0), span=(1, 2), flag=wx.EXPAND | wx.TOP, border=5)
        self.m_max_variance_text = wx.TextCtrl(panel, value="0.5")
        grid_sizer.Add(wx.StaticText(panel, label="Max Variance (mm)"), pos=(13, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_max_variance_text, pos=(13, 1), flag=wx.EXPAND)

        grid_sizer.Add(wx.StaticLine(panel), pos=(14, 0), span=(1, 2), flag=wx.EXPAND | wx.ALL, border=5)
        self.m_net_choice = wx.Choice(panel)
        grid_sizer.Add(wx.StaticText(panel, label="Assign to Net"), pos=(15, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_net_choice, pos=(15, 1), flag=wx.EXPAND)
        
        grid_sizer.AddGrowableCol(1)
        panel.SetSizer(grid_sizer)
        top_sizer.Add(panel, 1, wx.EXPAND | wx.ALL, border=5)
        self.m_fence_button = wx.Button(self, id=wx.ID_OK, label="Create Fence")
        self.m_close_button = wx.Button(self, id=wx.ID_CANCEL, label="Close")
        self.m_fence_button.SetDefault()
        button_sizer = wx.StdDialogButtonSizer()
        button_sizer.AddButton(self.m_fence_button)
        button_sizer.AddButton(self.m_close_button)
        button_sizer.Realize()
        top_sizer.Add(button_sizer, 0, wx.ALIGN_RIGHT | wx.ALL, border=10)
        self.SetSizerAndFit(top_sizer)
        
        self.Bind(wx.EVT_BUTTON, self.on_fence, self.m_fence_button)
        self.m_fence_button.Disable()
        self.populate_data()
        self.Center()

    def populate_data(self):
        try:
            self.kicad = kipy.KiCad()
            self.board = self.kicad.get_board()
            
            self.selection = self.board.get_selection()
            self.selected_items = [s for s in self.selection if isinstance(s, (Track, ArcTrack, BoardSegment))]
            
            all_nets = sorted(self.board.get_nets(), key=lambda n: n.name)
            net_names = [n.name for n in all_nets if "unconnected" not in n.name.lower()]
            self.m_net_choice.Append("[No Net]"), self.m_net_choice.Append(net_names), self.m_net_choice.SetSelection(0)

            if self.selected_items:
                self.selection_info_text.SetLabel(f"Found {len(self.selected_items)} selected item(s).")
                self.m_fence_button.Enable()
                
                default_net_name = None
                first_item = self.selected_items[0]
                item_layer = getattr(first_item, 'layer', None)

                if item_layer is not None:
                    all_zones = self.board.get_zones()
                    zones_on_layer = [z for z in all_zones if item_layer in z.layers and not z.is_rule_area() and z.net is not None]
                    
                    closest_zone = None
                    min_dist = float('inf')

                    item_midpoint = None
                    if isinstance(first_item, (Track, BoardSegment)):
                        item_midpoint = (first_item.start + first_item.end) * 0.5
                    elif isinstance(first_item, ArcTrack):
                        item_midpoint = first_item.mid

                    if item_midpoint:
                        for zone in zones_on_layer:
                            if not zone.filled: continue
                            for layer, polys in zone.filled_polygons.items():
                                if layer == item_layer:
                                    for poly in polys:
                                        dist = point_polygon_distance(item_midpoint, poly)
                                        if dist < min_dist:
                                            min_dist = dist
                                            closest_zone = zone
                    
                    if closest_zone:
                        default_net_name = closest_zone.net.name

                if default_net_name:
                    try:
                        idx = net_names.index(default_net_name)
                        self.m_net_choice.SetSelection(idx + 1)
                    except ValueError:
                        pass
                elif hasattr(first_item, 'net'):
                    item_net = first_item.net
                    if item_net:
                        try:
                            idx = net_names.index(item_net.name)
                            self.m_net_choice.SetSelection(idx + 1)
                        except ValueError:
                            pass
            
            netclasses = []
            if all_nets:
                netclass_map = self.board.get_netclass_for_nets(all_nets)
                netclasses = sorted(list(set(nc.name for nc in netclass_map.values())))
            self.m_netclass_choice.Append(netclasses)
            if "Default" in netclasses: self.m_netclass_choice.SetStringSelection("Default")
            elif len(netclasses) > 0: self.m_netclass_choice.SetSelection(0)

        except (ApiError, FileNotFoundError) as e:
            wx.MessageBox(f"Could not connect to KiCad PCB Editor: {e}", "Connection Error", wx.OK | wx.ICON_ERROR, parent=self)
        except Exception as e:
            wx.MessageBox(f"An unexpected error occurred during startup: {e}", "Error", wx.OK | wx.ICON_ERROR, parent=self)

    def on_fence(self, event):
        wx.BeginBusyCursor()
        self.m_fence_button.Disable()
        try:
            params = {
                'num_rows': int(self.m_num_rows_text.GetValue()),
                'distance': float(self.m_distance_text.GetValue()),
                'spacing': float(self.m_spacing_text.GetValue()),
                'row_offset': float(self.m_row_offset_text.GetValue()),
                'clearance': float(self.m_clearance_text.GetValue()),
                'random_variance': self.m_random_variance_checkbox.GetValue(),
                'max_variance': float(self.m_max_variance_text.GetValue()),
                'net_name': self.m_net_choice.GetStringSelection() if self.m_net_choice.GetSelection() > 0 else None,
                'is_diff_pair': self.m_diff_pair_checkbox.GetValue(),
            }
            if self.m_sizing_mode.GetSelection() == 0:
                params.update({'via_size': float(self.m_via_size_text.GetValue()), 'drill_size': float(self.m_drill_size_text.GetValue()), 'netclass_name': None})
            else:
                params.update({'via_size': None, 'drill_size': None, 'netclass_name': self.m_netclass_choice.GetStringSelection()})

            vias_created = self.perform_fencing(params)
            
            if vias_created is not None:
                wx.MessageBox(f"✅ Successfully created {vias_created} fencing vias.", "Success", wx.OK | wx.ICON_INFORMATION, parent=self)
                self.EndModal(wx.ID_OK)
        except ValueError as e:
            wx.MessageBox(f"Invalid input: {e}", "Input Error", wx.OK | wx.ICON_ERROR, parent=self)
        except Exception as e:
            logging.error("Fencing process failed.", exc_info=True)
            wx.MessageBox(f"An error occurred during fencing:\n{e}\n\nSee console for detailed log.", "Processing Error", wx.OK | wx.ICON_ERROR, parent=self)
        finally:
            self.m_fence_button.Enable()
            wx.EndBusyCursor()

    def _generate_candidates(self, item, spacing_nm, distance_nm, via_radius_nm, num_rows, row_offset_nm, sides):
        points = []
        item_width = getattr(item, 'width', 0)

        if distance_nm <= item_width / 2 + via_radius_nm:
            logging.warning("Distance from trace is too small, vias may overlap the trace.")

        for row in range(1, num_rows + 1):
            current_dist = distance_nm + (row - 1) * row_offset_nm
            for side in sides:
                offset_dist = current_dist + item_width / 2
                if isinstance(item, (Track, BoardSegment)):
                    p1, p2 = item.start, item.end
                    length = (p2 - p1).length()
                    if length == 0: continue

                    num_vias = round(length / spacing_nm) + 1
                    if num_vias > 1:
                        actual_spacing = length / (num_vias - 1)
                        direction = (p2 - p1) * (1.0 / length)
                        perp_dir = Vector2.from_xy(-direction.y, direction.x)
                        offset = perp_dir * offset_dist * side
                        for i in range(num_vias):
                            points.append(p1 + direction * (actual_spacing * i) + offset)
                elif isinstance(item, ArcTrack):
                    center = item.center()
                    if not center: continue
                    
                    radius = item.radius()
                    new_radius = radius + offset_dist * side
                    if new_radius <= 0: continue

                    start_angle = item.start_angle()
                    arc_angle_val = item.angle()
                    if start_angle is None or arc_angle_val is None: continue
                    
                    length = abs(arc_angle_val) * new_radius
                    num_vias = round(length / spacing_nm) + 1
                    if num_vias > 1:
                        angle_step = arc_angle_val / (num_vias - 1)
                        for i in range(num_vias):
                            angle = start_angle + angle_step * i
                            points.append(Vector2.from_xy(
                                int(center.x + new_radius * math.cos(angle)),
                                int(center.y + new_radius * math.sin(angle))
                            ))
        return points

    def perform_fencing(self, params: dict) -> int | None:
        logging.info(f"Starting fencing with params: {params}")
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
        if spacing_nm <= 0:
            raise ValueError("Spacing must be a positive value.")

        distance_nm = from_mm(params['distance'])
        via_radius_nm = via_size_nm / 2.0
        
        row_offset_nm = from_mm(params['row_offset'])
        clearance_nm = from_mm(params['clearance'])
        
        target_net = next((n for n in self.board.get_nets() if n.name == params['net_name']), None) if params['net_name'] else None

        obstacles = []
        all_items = self.board.get_items([KiCadObjectType.KOT_PCB_TRACE, KiCadObjectType.KOT_PCB_ARC])
        selected_ids = [item.id for item in self.selected_items]
        for item in all_items:
            if item.id in selected_ids: continue
            obstacles.append({'item': item, 'net': getattr(item, 'net', None), 'type': item.__class__.__name__})
        logging.info(f"Collected {len(obstacles)} trace obstacles.")

        candidate_points = []
        if params['is_diff_pair']:
            if len(self.selected_items) != 2:
                raise ValueError("Differential pair fencing requires exactly two tracks to be selected.")
            
            item1, item2 = self.selected_items
            sides = [-1, 1]
            points1 = self._generate_candidates(item1, spacing_nm, distance_nm, via_radius_nm, params['num_rows'], row_offset_nm, sides)
            points2 = self._generate_candidates(item2, spacing_nm, distance_nm, via_radius_nm, params['num_rows'], row_offset_nm, sides)

            midpoints = [(p1+p2)*0.5 for p1, p2 in zip(points1, points2)] if len(points1) == len(points2) else []

            for p in points1 + points2:
                is_outer = True
                for mid in midpoints:
                    if (p-mid).length() < (item1.position - mid).length() or (p-mid).length() < (item2.position - mid).length():
                        is_outer = False
                        break
                if is_outer:
                    candidate_points.append(p)
        else:
            for item in self.selected_items:
                sides = [-1, 1]
                if not is_copper_layer(item.layer):
                    sides = [1]
                candidate_points.extend(self._generate_candidates(item, spacing_nm, distance_nm, via_radius_nm, params['num_rows'], row_offset_nm, sides))

        logging.info(f"Generated {len(candidate_points)} initial candidate points.")

        items_to_create = []
        validated_points = []
        for pos in candidate_points:
            error_reason = ""

            for vp in validated_points:
                if (pos - vp).length() < spacing_nm:
                    error_reason = "Spacing conflict with new via"
                    break
            if error_reason:
                logging.debug(f"Candidate at ({pos.x}, {pos.y}) rejected: {error_reason}")
                continue

            for obs in obstacles:
                if target_net and obs['net'] and obs['net'].name == target_net.name:
                    continue

                item = obs['item']
                dist = float('inf')
                obs_clearance = 0
                
                if isinstance(item, Track):
                    dist, obs_clearance = point_segment_distance(pos, item.start, item.end), item.width / 2.0
                elif isinstance(item, ArcTrack):
                    dist, obs_clearance = point_arc_distance(pos, item), item.width / 2.0

                if dist < via_radius_nm + obs_clearance + clearance_nm:
                    error_reason = f"Conflict: {obs['type']}"
                    break
            
            if not error_reason:
                validated_points.append(pos)
                via = Via()
                via.position, via.diameter, via.drill_diameter = pos, via_size_nm, drill_size_nm
                if target_net: via.net = target_net
                items_to_create.append(via)
            
            if self.debug_mode and error_reason:
                via = Via()
                via.position, via.diameter, via.drill_diameter = pos, via_size_nm, drill_size_nm
                if target_net: via.net = target_net
                items_to_create.append(via)
                text = BoardText()
                text.position = pos
                text.value = error_reason
                text.layer = BoardLayer.BL_Cmts_User
                text.attributes.size = Vector2.from_xy(from_mm(0.2), from_mm(0.2))
                items_to_create.append(text)

        vias_created_count = len([i for i in items_to_create if isinstance(i, Via)])
        logging.info(f"Found {vias_created_count} valid via positions.")

        if not items_to_create:
            return 0

        commit = self.board.begin_commit()
        self.board.create_items(items_to_create)
        self.board.push_commit(commit, f"Add {vias_created_count} fencing vias")
        
        return vias_created_count

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Via Fencing for KiCad PCB Editor.")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode.")
    args, _ = parser.parse_known_args()

    app = wx.App(False)
    dialog = FenceDialog(None)
    dialog.debug_mode = args.debug
    dialog.ShowModal()
    dialog.Destroy()
