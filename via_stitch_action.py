#!/usr/bin/env python3
#
# Via Stitching GUI for KiCad

import wx
import sys
import math
import logging
import random
import kipy
import argparse
from kipy.board import Board
from kipy.board_types import (BoardCircle, BoardPolygon, BoardRectangle,
                             BoardSegment, BoardShape, Pad, Track, Via, ArcTrack, Zone, BoardText, BoardTextBox, ViaType)
from kipy.errors import ApiError
from kipy.geometry import Box2, Vector2
from kipy.util import from_mm
from kipy.util.board_layer import BoardLayer, is_copper_layer
from kipy.proto.common.types import KiCadObjectType
from stitching_utils import *

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class StitcherDialog(wx.Dialog):
    def __init__(self, parent):
        super(StitcherDialog, self).__init__(parent, title="Via Stitching Tool", style=wx.DEFAULT_DIALOG_STYLE | wx.STAY_ON_TOP)
        self.debug_mode = False
        self.kicad, self.board = None, None
        top_sizer = wx.BoxSizer(wx.VERTICAL)
        panel = wx.Panel(self)
        grid_sizer = wx.GridBagSizer(5, 5)

        self.selection_info_text = wx.StaticText(panel, label="No selection. Stitching entire board.")
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
        self.m_spacing_text = wx.TextCtrl(panel, value="5")
        grid_sizer.Add(wx.StaticText(panel, label="Spacing (mm)"), pos=(6, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_spacing_text, pos=(6, 1), flag=wx.EXPAND)
        self.m_clearance_text = wx.TextCtrl(panel, value="0.2")
        grid_sizer.Add(wx.StaticText(panel, label="Copper Clearance (mm)"), pos=(7, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_clearance_text, pos=(7, 1), flag=wx.EXPAND)
        self.m_edge_clearance_text = wx.TextCtrl(panel, value="0.5")
        grid_sizer.Add(wx.StaticText(panel, label="Edge Clearance (mm)"), pos=(8, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_edge_clearance_text, pos=(8, 1), flag=wx.EXPAND)
        
        self.m_row_offset_text = wx.TextCtrl(panel, value="2.5")
        grid_sizer.Add(wx.StaticText(panel, label="Row Offset (mm)"), pos=(9, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_row_offset_text, pos=(9, 1), flag=wx.EXPAND)

        self.m_random_variance_checkbox = wx.CheckBox(panel, label="Add Random Variance")
        grid_sizer.Add(self.m_random_variance_checkbox, pos=(10, 0), span=(1, 2), flag=wx.EXPAND | wx.TOP, border=5)

        self.m_max_x_variance_text = wx.TextCtrl(panel, value="0.5")
        grid_sizer.Add(wx.StaticText(panel, label="Max X Variance (mm)"), pos=(11, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_max_x_variance_text, pos=(11, 1), flag=wx.EXPAND)

        self.m_max_y_variance_text = wx.TextCtrl(panel, value="0.5")
        grid_sizer.Add(wx.StaticText(panel, label="Max Y Variance (mm)"), pos=(12, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_max_y_variance_text, pos=(12, 1), flag=wx.EXPAND)

        grid_sizer.Add(wx.StaticLine(panel), pos=(13, 0), span=(1, 2), flag=wx.EXPAND | wx.ALL, border=5)
        
        self.m_stitch_selection_only_checkbox = wx.CheckBox(panel, label="Stitch only selected area")
        grid_sizer.Add(self.m_stitch_selection_only_checkbox, pos=(14, 0), span=(1, 1), flag=wx.EXPAND | wx.TOP, border=5)

        self.m_stitch_layers_only_checkbox = wx.CheckBox(panel, label="Stitch only selected zone layers")
        grid_sizer.Add(self.m_stitch_layers_only_checkbox, pos=(14, 1), span=(1, 1), flag=wx.EXPAND | wx.TOP, border=5)

        self.m_net_choice = wx.Choice(panel)
        grid_sizer.Add(wx.StaticText(panel, label="Assign to Net"), pos=(15, 0), flag=wx.ALIGN_CENTER_VERTICAL|wx.LEFT, border=5)
        grid_sizer.Add(self.m_net_choice, pos=(15, 1), flag=wx.EXPAND)
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
            self.kicad = kipy.KiCad()
            self.board = self.kicad.get_board()
            
            selection = self.board.get_selection()
            self.selected_zones = [s for s in selection if isinstance(s, (Zone))]
            if self.selected_zones:
                self.selection_info_text.SetLabel(f"Found {len(self.selected_zones)} selected zone(s).")
                self.m_stitch_selection_only_checkbox.SetValue(True)
            else:
                self.m_stitch_selection_only_checkbox.SetValue(False)
                self.m_stitch_selection_only_checkbox.Disable()
                self.m_stitch_layers_only_checkbox.Disable()

            all_nets = sorted(self.board.get_nets(), key=lambda n: n.name)
            net_names = [n.name for n in all_nets if "unconnected" not in n.name.lower()]
            self.m_net_choice.Append("[No Net]"), self.m_net_choice.Append(net_names), self.m_net_choice.SetSelection(0)
            
            if self.selected_zones:
                zone_net = self.selected_zones[0].net
                if zone_net:
                    try:
                        idx = net_names.index(zone_net.name)
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
                'max_y_variance': float(self.m_max_y_variance_text.GetValue()),
                'stitch_selection_only': self.m_stitch_selection_only_checkbox.GetValue(),
                'stitch_layers_only': self.m_stitch_layers_only_checkbox.GetValue(),
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
        logging.info(f"Starting stitching with params: {params}")
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

        edge_clearance_nm = from_mm(params['edge_clearance'])
        clearance_nm = from_mm(params['clearance'])
        row_offset_nm = from_mm(params['row_offset'])
        random_variance = params['random_variance']
        max_x_variance_nm = from_mm(params['max_x_variance'])
        max_y_variance_nm = from_mm(params['max_y_variance'])
        via_radius_nm = via_size_nm / 2.0
        
        stitch_areas = []
        if params['stitch_selection_only'] and self.selected_zones:
            for zone in self.selected_zones:
                if not zone.filled:
                    wx.MessageBox(f"Warning: Selected zone '{zone.name}' is not filled. Skipping.", "Warning", wx.OK | wx.ICON_WARNING)
                    continue
                for layer, polys in zone.filled_polygons.items():
                    for poly in polys:
                        stitch_areas.append({'poly': poly, 'zone': zone})
            if not stitch_areas: raise Exception("Selected zones have no filled areas to stitch.")
            bbox = get_enclosing_bbox(self.board, self.selected_zones)
        else:
            outline_shapes = [s for s in self.board.get_shapes() if s.layer == BoardLayer.BL_Edge_Cuts]
            if not outline_shapes: raise Exception("No valid board outline found.")
            bbox = get_enclosing_bbox(self.board, outline_shapes)
            if bbox:
                outline_poly = BoardPolygon()
                outline_poly.polygons.append(kipy.geometry.PolygonWithHoles())
                outline_poly.polygons[0].outline.nodes.extend([
                    kipy.geometry.PolyLineNode.from_point(bbox.pos),
                    kipy.geometry.PolyLineNode.from_point(Vector2.from_xy(bbox.pos.x + bbox.size.x, bbox.pos.y)),
                    kipy.geometry.PolyLineNode.from_point(Vector2.from_xy(bbox.pos.x + bbox.size.x, bbox.pos.y + bbox.size.y)),
                    kipy.geometry.PolyLineNode.from_point(Vector2.from_xy(bbox.pos.x, bbox.pos.y + bbox.size.y)),
                ])
                outline_poly.polygons[0].outline.closed = True
                stitch_areas.append({'poly': outline_poly.polygons[0], 'zone': None})

        if not bbox: raise Exception("Could not determine board bounding box.")
        logging.info(f"Found {len(stitch_areas)} stitch areas. Bounding box: {bbox.pos.x}, {bbox.pos.y} -> {bbox.pos.x+bbox.size.x}, {bbox.pos.y+bbox.size.y}")
        
        target_net = next((n for n in self.board.get_nets() if n.name == params['net_name']), None) if params['net_name'] else None
        
        obstacles = []
        all_items = self.board.get_items([KiCadObjectType.KOT_PCB_PAD, KiCadObjectType.KOT_PCB_VIA, KiCadObjectType.KOT_PCB_TRACE, KiCadObjectType.KOT_PCB_ARC])
        for item in all_items:
            obstacles.append({'item': item, 'net': getattr(item, 'net', None), 'type': item.__class__.__name__})
        logging.info(f"Collected {len(obstacles)} obstacles.")

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
                
                for area in stitch_areas:
                    if is_point_inside_polygon_with_holes(pos, area['poly']):
                        if point_polygon_distance(pos, area['poly']) >= via_radius_nm + edge_clearance_nm:
                            candidate_positions.append({'pos': pos, 'zone': area['zone']})
                            break
        
        logging.info(f"Generated {len(candidate_positions)} candidate positions inside stitch areas.")
        if not candidate_positions:
            raise Exception("No valid positions for vias found within the specified area and clearance.")

        items_to_create = []
        validated_points = []
        for cand in candidate_positions:
            pos, zone = cand['pos'], cand['zone']
            
            error_reason = ""

            for vp in validated_points:
                if (pos - vp).length() < spacing_nm:
                    error_reason = "Spacing conflict with new via"
                    break
            if error_reason:
                logging.debug(f"Candidate at ({pos.x}, {pos.y}) rejected: {error_reason}")
                continue

            if not error_reason:
                via_layers = None
                if zone and params['stitch_layers_only']:
                    via_layers = zone.layers

                for obs in obstacles:
                    if target_net and obs['net'] and obs['net'].name == target_net.name:
                        continue
                    
                    obs_layer = getattr(obs['item'], 'layer', None)
                    if obs_layer is None and hasattr(obs['item'], 'padstack'):
                        obs_layer = obs['item'].padstack.layers

                    if via_layers and obs_layer:
                        if isinstance(obs_layer, list):
                            if not any(l in via_layers for l in obs_layer):
                                continue
                        elif obs_layer not in via_layers:
                            continue

                    item = obs['item']
                    dist = float('inf')
                    obs_clearance = 0
                    if isinstance(item, Via):
                        dist, obs_clearance = (pos - item.position).length(), item.diameter / 2.0
                    elif isinstance(item, Pad):
                        dist = (pos - item.position).length()
                        pad_layer = item.padstack.copper_layer(BoardLayer.BL_F_Cu) or item.padstack.copper_layer(BoardLayer.BL_B_Cu)
                        obs_clearance = max(pad_layer.size.x, pad_layer.size.y) / 2.0 if pad_layer else 0
                    elif isinstance(item, Track):
                        dist, obs_clearance = point_segment_distance(pos, item.start, item.end), item.width / 2.0
                    elif isinstance(item, ArcTrack):
                        dist, obs_clearance = point_arc_distance(pos, item), item.width / 2.0

                    if dist < via_radius_nm + obs_clearance + clearance_nm:
                        error_reason = f"Conflict: {obs['type']}"
                        break
            
            is_valid = not error_reason

            if not is_valid and not self.debug_mode:
                continue

            validated_points.append(pos)
            via = Via()
            via.position, via.diameter, via.drill_diameter = pos, via_size_nm, drill_size_nm
            if target_net: via.net = target_net
            items_to_create.append(via)

            if not is_valid and self.debug_mode:
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
        commit_msg = f"Add {vias_created_count} stitching vias"
        if target_net: commit_msg += f" to net {params['net_name']}"
        self.board.push_commit(commit, commit_msg)
        
        return vias_created_count

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Via Stitching for KiCad PCB Editor.")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode.")
    args, _ = parser.parse_known_args()

    app = wx.App(False)
    dialog = StitcherDialog(None)
    dialog.debug_mode = args.debug
    dialog.ShowModal()
    dialog.Destroy()
