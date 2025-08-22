#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Map (Occupancy PNG/PGM) -> Gazebo SDF (accurate segments)
- Extracts contours of occupied cells
- Simplifies in METERS (Douglas–Peucker)
- Splits into straight segments, merges colinear runs
- Emits one model, one link, many rotated box segments (correct length & pose)

ROS 2 node parameters:
  map_path         : path to <map>.yaml (with image/resolution/origin)
  output_world     : output SDF world file
  wall_height      : height of walls in meters (Z size)
  wall_thickness   : wall thickness in meters (Y size of each box)
  max_segments     : cap segment count for performance
  wall_simplify_m  : polyline simplification tolerance in meters (0.02–0.08 good)
  min_seg_len_m    : drop very short segments (noise)
  merge_angle_deg  : merge edges whose directions differ less than this angle
  denoise_cm       : morphological denoise kernel in centimeters (0 to disable)
  debug_output     : write *_segments.txt with details

Validate result:
  gz sdf -k generated_world.world
"""

import os
import math
import yaml
import numpy as np
from PIL import Image

from skimage import measure
from scipy import ndimage

import rclpy
from rclpy.node import Node


class MapToGazeboSegments(Node):
    def __init__(self):
        super().__init__('map_to_gazebo_segments')

        # ---- Parameters (with sensible defaults)
        self.declare_parameter('map_path', '/home/divya/project/src/navigation_demo/maps/office_new.yaml')
        self.declare_parameter('output_world', 'generated_world.world')
        self.declare_parameter('wall_height', 2.20)
        self.declare_parameter('wall_thickness', 0.15)
        self.declare_parameter('max_segments', 6000)
        self.declare_parameter('wall_simplify_m', 0.05)
        self.declare_parameter('min_seg_len_m', 0.10)
        self.declare_parameter('merge_angle_deg', 5.0)
        self.declare_parameter('denoise_cm', 0.0)       # 0 = no denoise
        self.declare_parameter('debug_output', True)

        # ---- Read params
        p = lambda k: self.get_parameter(k).value
        self.map_yaml       = p('map_path')
        self.output_world   = p('output_world')
        self.wall_height    = float(p('wall_height'))
        self.wall_thickness = float(p('wall_thickness'))
        self.max_segments   = int(p('max_segments'))
        self.wall_simplify_m= float(p('wall_simplify_m'))
        self.min_seg_len_m  = float(p('min_seg_len_m'))
        self.merge_angle_deg= float(p('merge_angle_deg'))
        self.denoise_cm     = float(p('denoise_cm'))
        self.debug_output   = bool(p('debug_output'))

        try:
            self.convert()
        finally:
            # end quickly; no spin needed
            pass

    # ---------------- Core pipeline ----------------

    def convert(self):
        # 1) Load map metadata + image
        img, resolution, origin, meta = self.load_map(self.map_yaml)
        self.resolution = float(resolution)
        self.origin_xy  = (float(origin[0]), float(origin[1]))
        H, W = img.shape
        self.get_logger().info(f"Map: {W}x{H}px   res={self.resolution} m/px   origin={origin}")

        # 2) Binarize -> occupied mask (True=wall)
        wall_mask = self.preprocess_map(img, self.resolution, self.denoise_cm, debug=True)

        # 3) Extract accurate contour segments in WORLD coords
        segments = self.extract_wall_segments(
            wall_mask,
            origin=self.origin_xy,
            resolution=self.resolution,
            wall_simplify_m=self.wall_simplify_m,
            merge_angle_deg=self.merge_angle_deg,
            min_seg_len_m=self.min_seg_len_m
        )

        self.get_logger().info(f"Segments (pre-cap): {len(segments)}")

        # 4) Cap for performance (keep longest first)
        if len(segments) > self.max_segments:
            segments.sort(key=lambda s: s['length'], reverse=True)
            segments = segments[:self.max_segments]
            self.get_logger().info(f"Capped segments to: {len(segments)}")

        # 5) Generate SDF (single model + link; each segment = rotated box)
        world_xml = self.generate_world_sdf(segments, self.wall_height, self.wall_thickness)

        # 6) Save SDF
        with open(self.output_world, 'w') as f:
            f.write(world_xml)
        self.get_logger().info(f"✅ Generated world: {self.output_world}")

        # 7) Optional debug dump
        if self.debug_output:
            dbg = self.output_world.replace('.world', '_segments.txt')
            self.dump_segments_debug(dbg, segments, meta, (H, W))
            self.get_logger().info(f"📝 Segment debug: {dbg}")

    # ---------------- Utilities ----------------

    def load_map(self, map_yaml):
        if not os.path.exists(map_yaml):
            raise FileNotFoundError(f"Map YAML not found: {map_yaml}")
        with open(map_yaml, 'r') as f:
            data = yaml.safe_load(f)

        image_path = data['image']
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(map_yaml), image_path)
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"Map image not found: {image_path}")

        # Load as grayscale
        img = Image.open(image_path).convert('L')
        arr = np.array(img)

        res = float(data['resolution'])
        origin = data.get('origin', [0.0, 0.0, 0.0])
        if len(origin) < 2:
            origin = [0.0, 0.0, 0.0]

        return arr, res, origin, data

    def preprocess_map(self, map_array, resolution, denoise_cm, debug=False):
        """
        Produce occupied mask: True where walls/obstacles.
        Conventional maps: 0=occupied (black), 255=free, ~205=unknown.
        """
        unique_vals = np.unique(map_array)
        if debug:
            self.get_logger().info(f"Pixel values (sampled): {unique_vals[:10]} ...")

        if 0 in unique_vals:
            mask = (map_array == 0)
        else:
            # Fallback: dark quantile
            thr = np.percentile(map_array, 15.0)
            mask = (map_array <= thr)

        # Optional small denoise (open/close) using physical kernel size
        if denoise_cm and denoise_cm > 0:
            k_m = float(denoise_cm) / 100.0
            k_px = max(1, int(round(k_m / resolution)))
            se = np.ones((k_px, k_px), dtype=np.uint8)
            # Close (fill tiny gaps) then open (remove tiny specks)
            mask = ndimage.binary_closing(mask, structure=se)
            mask = ndimage.binary_opening(mask, structure=se)

        mask = mask.astype(np.uint8)
        if debug:
            pct = 100.0 * mask.sum() / mask.size
            self.get_logger().info(f"Occupied pixels: {mask.sum()} ({pct:.2f}%)")
        return mask

    def px_to_world(self, r, c, H, origin, res):
        # Image Y+ downward -> world Y+ upward
        wx = origin[0] + c * res
        wy = origin[1] + (H - r) * res
        return (float(wx), float(wy))

    def simplify_contour_px(self, contour_px, resolution, tol_m):
        if len(contour_px) < 3:
            return contour_px
        tol_px = max(1.0, float(tol_m) / float(resolution))
        return measure.approximate_polygon(contour_px, tolerance=tol_px)

    def segmentize_polyline_world(self, poly_w, merge_angle_deg=5.0, min_seg_len=0.10):
        """
        Given world polyline [(x,y),...], return merged segments as tuples:
        (cx, cy, length, yaw)
        """
        if len(poly_w) < 2:
            return []

        def seg_info(p0, p1):
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            L = math.hypot(dx, dy)
            yaw = math.atan2(dy, dx)
            cx = 0.5 * (p0[0] + p1[0])
            cy = 0.5 * (p0[1] + p1[1])
            return [cx, cy, L, yaw]

        edges = [seg_info(poly_w[i], poly_w[i + 1]) for i in range(len(poly_w) - 1)]
        if not edges:
            return []

        def angle_close(a, b, tol_deg):
            d = (a - b + math.pi) % (2 * math.pi) - math.pi
            return abs(math.degrees(d)) <= tol_deg

        merged = []
        cur = edges[0]
        for e in edges[1:]:
            _, _, L, yaw = e
            if angle_close(cur[3], yaw, merge_angle_deg):
                # extend: shift center forward along heading by L/2; increase length
                cur[0] += math.cos(cur[3]) * (L / 2.0)
                cur[1] += math.sin(cur[3]) * (L / 2.0)
                cur[2] += L
            else:
                if cur[2] >= min_seg_len:
                    merged.append(tuple(cur))
                cur = e
        if cur[2] >= min_seg_len:
            merged.append(tuple(cur))
        return merged

    def extract_wall_segments(self, wall_mask, origin, resolution,
                              wall_simplify_m=0.05,
                              merge_angle_deg=5.0,
                              min_seg_len_m=0.10):
        """
        1) find_contours on binary mask
        2) simplify each contour (tolerance in meters)
        3) convert to world coords
        4) segmentize & merge colinear runs
        """
        H, W = wall_mask.shape
        # find_contours expects values in [0,1]; level=0.5 follows the boundary
        contours = measure.find_contours(wall_mask, level=0.5, fully_connected='high')
        self.get_logger().info(f"Contours found: {len(contours)}")

        segments = []
        for idx, cnt in enumerate(contours):
            # cnt: Nx2 floats (row, col)
            cnt_s = self.simplify_contour_px(cnt, resolution, tol_m=wall_simplify_m)
            if len(cnt_s) < 2:
                continue

            # Convert to world
            poly_w = [self.px_to_world(r=float(p[0]), c=float(p[1]), H=H, origin=origin, res=resolution)
                      for p in cnt_s]

            # Break into straight merged segments
            segs = self.segmentize_polyline_world(poly_w,
                                                  merge_angle_deg=merge_angle_deg,
                                                  min_seg_len=min_seg_len_m)
            for (cx, cy, L, yaw) in segs:
                if not (np.isfinite(cx) and np.isfinite(cy) and np.isfinite(L) and np.isfinite(yaw)):
                    continue
                if L <= 0:
                    continue
                segments.append({
                    'center_world': (cx, cy),
                    'length': float(L),
                    'yaw': float(yaw)
                })
        return segments

    def generate_world_sdf(self, segments, wall_height, wall_thickness):
        """SDF 1.10, one model with one link; each segment is a rotated box."""
        def clamp_pos(x, mn=0.05):
            return float(x) if (np.isfinite(x) and x > 0.0) else mn

        wall_h = clamp_pos(wall_height, 0.05)
        wall_t = clamp_pos(wall_thickness, 0.05)

        header = f"""<?xml version="1.0"?>
<sdf version="1.10">
  <world name="generated_world">
    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
          <material><diffuse>0.85 0.85 0.85 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Walls -->
    <model name="map_walls">
      <static>true</static>
      <link name="walls_link">
"""
        pieces = [header]
        z = wall_h / 2.0
        for i, s in enumerate(segments):
            cx, cy = s['center_world']
            L = clamp_pos(s['length'])
            yaw = float(s['yaw'])
            pieces.append(f"""
        <!-- segment {i} -->
        <collision name="c_{i}">
          <pose>{cx:.6f} {cy:.6f} {z:.6f} 0 0 {yaw:.6f}</pose>
          <geometry><box><size>{L:.6f} {wall_t:.6f} {wall_h:.6f}</size></box></geometry>
          <surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface>
        </collision>
        <visual name="v_{i}">
          <pose>{cx:.6f} {cy:.6f} {z:.6f} 0 0 {yaw:.6f}</pose>
          <geometry><box><size>{L:.6f} {wall_t:.6f} {wall_h:.6f}</size></box></geometry>
<material>
  <ambient>0.4 0.1 0.1 1</ambient>
  <diffuse>0.8 0.2 0.2 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <emissive>0 0 0 1</emissive>
</material>
          <cast_shadows>false</cast_shadows>
        </visual>
""")
        footer = """
      </link>
    </model>
  </world>
</sdf>
"""
        pieces.append(footer)
        return "".join(pieces)

    def dump_segments_debug(self, path, segments, meta, shape_px):
        H, W = shape_px
        with open(path, 'w') as f:
            f.write("Map->SDF Segment Debug\n")
            f.write("======================\n")
            f.write(f"image: {meta.get('image')}\n")
            f.write(f"resolution: {meta.get('resolution')} m/px\n")
            f.write(f"origin: {meta.get('origin')}\n")
            f.write(f"size_px: {W} x {H}\n")
            f.write(f"segments: {len(segments)}\n\n")
            for i, s in enumerate(segments[:10000]):  # guard
                cx, cy = s['center_world']
                L = s['length']
                yaw_deg = math.degrees(s['yaw'])
                f.write(f"{i:05d}: pos=({cx:.3f},{cy:.3f}) L={L:.3f} yaw={yaw_deg:.1f}°\n")


def main(args=None):
    rclpy.init(args=args)
    node = MapToGazeboSegments()
    # no spin; all work is done in __init__
    rclpy.shutdown()


if __name__ == '__main__':
    main()
