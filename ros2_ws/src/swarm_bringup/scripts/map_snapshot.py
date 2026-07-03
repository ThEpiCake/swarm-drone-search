#!/usr/bin/env python3
"""
map_snapshot.py — dumps a readable crash-debug snapshot to /tmp/map_snapshot.txt
Captures: 2D occupancy grid (ASCII), drone position, frontier goal, RPLIDAR scan.

Usage (while simulation is running):
    python3 map_snapshot.py              # saves to /tmp/map_snapshot.txt
    python3 map_snapshot.py /tmp/snap.txt
"""

import sys, math, time, json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped

NAMESPACE = "/px4_0"

TRANSIENT = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
BEST_EFFORT = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class Snapshot(Node):
    def __init__(self):
        super().__init__("map_snapshot")
        self.grid = None
        self.odom = None
        self.goal = None
        self.scan = None
        self.drone_path = None

        self.create_subscription(OccupancyGrid,
            f"{NAMESPACE}/voxel_slice_map", self._grid, TRANSIENT)
        self.create_subscription(Odometry,
            f"{NAMESPACE}/local_position/odom", self._odom, BEST_EFFORT)
        self.create_subscription(PointStamped,
            f"{NAMESPACE}/frontier_goal", self._goal, 10)
        self.create_subscription(LaserScan,
            f"{NAMESPACE}/scan", self._scan, BEST_EFFORT)
        self.create_subscription(Path,
            f"{NAMESPACE}/drone_path", self._path, BEST_EFFORT)

    def _grid(self, m):  self.grid = m
    def _odom(self, m):  self.odom = m
    def _goal(self, m):  self.goal = m
    def _scan(self, m):  self.scan = m
    def _path(self, m):  self.drone_path = m

    def ready(self):
        return self.grid is not None and self.odom is not None

    # ------------------------------------------------------------------ #
    def build_report(self):
        lines = []

        # ── Drone position ──────────────────────────────────────────────
        if self.odom:
            p = self.odom.pose.pose.position
            q = self.odom.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                              1 - 2*(q.y**2 + q.z**2))
            lines.append(f"DRONE  N={p.x:.2f}  E={p.y:.2f}  D={p.z:.2f}  "
                         f"yaw={math.degrees(yaw):.0f}°")
        else:
            lines.append("DRONE  (no odom)")

        # ── Frontier goal ───────────────────────────────────────────────
        if self.goal:
            g = self.goal.point
            lines.append(f"GOAL   N={g.x:.2f}  E={g.y:.2f}  D={g.z:.2f}")
        else:
            lines.append("GOAL   (none)")

        # ── RPLIDAR: forward arc ±90° and full min ──────────────────────
        if self.scan:
            r = self.scan.ranges
            n = len(r)
            ai = self.scan.angle_increment
            amin = self.scan.angle_min
            # forward = index where angle ≈ 0
            fwd_idx = int(round(-amin / ai))
            arc45 = int(round(math.radians(45) / ai))

            def minrange(lo, hi):
                vals = [r[i] for i in range(lo, hi)
                        if 0 < r[i] < self.scan.range_max]
                return min(vals) if vals else float('inf')

            fwd   = minrange(max(0,fwd_idx-arc45), min(n,fwd_idx+arc45))
            left  = minrange(max(0,fwd_idx+arc45), min(n,fwd_idx+arc45*2))
            right = minrange(max(0,fwd_idx-arc45*2), max(0,fwd_idx-arc45))
            gmin  = minrange(0, n)

            lines.append(f"LIDAR  fwd(±45°)={fwd:.2f}m  "
                         f"left={left:.2f}m  right={right:.2f}m  "
                         f"global_min={gmin:.2f}m")

            # Sector summary (72 sectors × 5°)
            sectors = []
            for s in range(72):
                lo = int(round(s * 5 / math.degrees(ai)))
                hi = int(round((s+1) * 5 / math.degrees(ai)))
                v = minrange(lo, min(n, hi))
                sectors.append(f"s{s*5}={v:.1f}" if v < float('inf') else f"s{s*5}=∞")
            lines.append("SECTORS: " + "  ".join(sectors))
        else:
            lines.append("LIDAR  (no scan)")

        # ── ASCII occupancy grid ────────────────────────────────────────
        if self.grid:
            g = self.grid
            w, h = g.info.width, g.info.height
            res = g.info.resolution
            ox  = g.info.origin.position.x
            oy  = g.info.origin.position.y

            lines.append(f"\nMAP  {w}×{h} cells @ {res:.2f}m/cell  "
                         f"origin=({ox:.2f},{oy:.2f})  "
                         f"frame={g.header.frame_id}")

            # Find drone cell
            if self.odom:
                p = self.odom.pose.pose.position
                dc = int((p.y - oy) / res)   # east  → col
                dr = int((p.x - ox) / res)   # north → row
            else:
                dc = dr = -1

            # Find goal cell
            if self.goal:
                gc_col = int((self.goal.point.y - oy) / res)
                gc_row = int((self.goal.point.x - ox) / res)
            else:
                gc_col = gc_row = -1

            # Render (flip rows so North is up)
            CHARS = {-1: "·", 0: " ", 100: "█"}
            ascii_rows = []
            for row in range(h - 1, -1, -1):
                line_chars = []
                for col in range(w):
                    idx = row * w + col
                    val = g.data[idx]
                    if row == dr and col == dc:
                        ch = "D"
                    elif row == gc_row and col == gc_col:
                        ch = "G"
                    else:
                        ch = CHARS.get(val, "?")
                    line_chars.append(ch)
                ascii_rows.append("".join(line_chars))
            lines.append("\n".join(ascii_rows))

            # Stats
            total = w * h
            occ   = sum(1 for v in g.data if v == 100)
            free  = sum(1 for v in g.data if v == 0)
            unk   = sum(1 for v in g.data if v == -1)
            lines.append(f"\nSTATS  occ={occ} ({100*occ/total:.1f}%)  "
                         f"free={free} ({100*free/total:.1f}%)  "
                         f"unknown={unk} ({100*unk/total:.1f}%)")
            lines.append("LEGEND: D=drone  G=goal  █=occupied  · =unknown  (space)=free")
        else:
            lines.append("\nMAP  (no grid received)")

        # ── Drone travel path ───────────────────────────────────────────
        if self.drone_path and self.drone_path.poses:
            pts = [(p.pose.position.x, p.pose.position.y)
                   for p in self.drone_path.poses]
            lines.append(f"\nPATH  {len(pts)} poses  "
                         f"start=({pts[0][0]:.1f},{pts[0][1]:.1f})  "
                         f"end=({pts[-1][0]:.1f},{pts[-1][1]:.1f})")

        return "\n".join(lines)


def main():
    out_file = sys.argv[1] if len(sys.argv) > 1 else "/tmp/map_snapshot.txt"
    rclpy.init()
    node = Snapshot()

    print("Collecting data (up to 8s)...")
    deadline = time.time() + 8.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.ready():
            break

    if not node.ready():
        print("WARNING: did not receive grid+odom within 8s — partial snapshot")

    report = node.build_report()
    with open(out_file, "w") as f:
        f.write(report)

    print(f"\nSaved → {out_file}")
    print("=" * 60)
    print(report[:2000])   # preview first 2000 chars in terminal
    if len(report) > 2000:
        print(f"... ({len(report) - 2000} more chars in file)")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
