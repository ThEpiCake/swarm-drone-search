#!/usr/bin/env python3
"""
Isometric 3-D layer visualizer — TAKEOFF mode validation tool.

Subscribes to /px4_0/occupancy_map (nav_msgs/OccupancyGrid).  Each message
published by lidar_mapper carries one horizontal slice of the map; this tool
accumulates every slice received and renders them as stacked coloured grids in
an isometric 3-D matplotlib figure.  The view updates automatically as the
drone ascends and new layers are populated.

Usage (run alongside the simulation):
    python3 visualize_layers.py [--topic /px4_0/occupancy_map]

Tip: if the window does not open, change the BACKEND constant below.
"""

import argparse
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid

import matplotlib
BACKEND = "TkAgg"          # change to "Qt5Agg" or "wxAgg" if TkAgg is absent
matplotlib.use(BACKEND)
import matplotlib.pyplot as plt                           # noqa: E402
import matplotlib.patches as mpatches                    # noqa: E402
from mpl_toolkits.mplot3d.art3d import PolyCollection   # noqa: E402

# ── Visual constants ──────────────────────────────────────────────────────────
_C_OCC  = (0.85, 0.15, 0.10, 0.88)   # deep red   — obstacle / inflated zone
_C_FREE = (0.18, 0.72, 0.22, 0.35)   # mid green  — confirmed free space
_C_UNK  = (0.70, 0.70, 0.70, 0.07)   # pale grey  — unvisited cells


# ── ROS2 subscriber node ──────────────────────────────────────────────────────

class LayerCollector(Node):
    """Accumulates all occupancy layers broadcast by lidar_mapper."""

    def __init__(self, topic: str):
        super().__init__("layer_visualizer")
        self._lock: threading.Lock = threading.Lock()
        self._layers: dict[float, dict] = {}   # z_m → layer dict
        self._dirty: bool = False

        # lidar_mapper publishes with transient_local/reliable so late
        # subscribers receive the most recent message for each layer.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, topic, self._cb, qos)
        self.get_logger().info(f"Subscribed to {topic}")

    def _cb(self, msg: OccupancyGrid) -> None:
        z_key = round(msg.info.origin.position.z, 3)     # layer altitude (m)
        w   = msg.info.width                              # East  cells
        h   = msg.info.height                             # North cells
        res = msg.info.resolution
        oe  = msg.info.origin.position.x                  # East  origin (m)
        on  = msg.info.origin.position.y                  # North origin (m)
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        with self._lock:
            self._layers[z_key] = dict(data=data, res=res, w=w, h=h, oe=oe, on=on)
            self._dirty = True

    def snapshot(self) -> dict[float, dict]:
        """Return a thread-safe copy of all layers; clears dirty flag."""
        with self._lock:
            self._dirty = False
            return {z: dict(d) for z, d in self._layers.items()}

    @property
    def dirty(self) -> bool:
        with self._lock:
            return self._dirty


# ── Rendering helpers ─────────────────────────────────────────────────────────

def _add_polys(ax, z: float, layer: dict, val: int,
               fc, ec="none", lw: float = 0.0) -> None:
    """Build a PolyCollection for all cells == val and add it at altitude z."""
    data = layer["data"]
    res, oe, on = layer["res"], layer["oe"], layer["on"]

    mask = (data == val) if val != -1 else (data < 0)
    rows, cols = np.where(mask)
    if rows.size == 0:
        return

    e0 = cols * res + oe
    n0 = rows * res + on
    e1, n1 = e0 + res, n0 + res

    # PolyCollection expects a (N, 4, 2) array of (East, North) vertices per quad
    polys = np.stack([
        np.column_stack([e0, n0]),
        np.column_stack([e1, n0]),
        np.column_stack([e1, n1]),
        np.column_stack([e0, n1]),
    ], axis=1)  # shape (N, 4, 2)

    pc = PolyCollection(polys, facecolors=[fc], edgecolors=ec, linewidths=lw)
    ax.add_collection3d(pc, zs=z, zdir="z")


def redraw(fig: plt.Figure, ax, collector: LayerCollector) -> None:
    layers = collector.snapshot()
    ax.cla()

    ax.set_xlabel("East  (m)", labelpad=6)
    ax.set_ylabel("North (m)", labelpad=6)
    ax.set_zlabel("Altitude (m)", labelpad=6)

    n_layers = len(layers)
    total_occ = sum(
        int(np.sum(v["data"] == 100)) for v in layers.values()
    )
    ax.set_title(
        f"Layered scan  —  {n_layers} layer(s)  |  {total_occ} occupied cells",
        fontsize=10, pad=10,
    )

    # Classic isometric view angle
    ax.view_init(elev=28, azim=-48)

    if not layers:
        ax.text(0, 0, 1.5, "Waiting for /px4_0/occupancy_map …",
                ha="center", va="center", fontsize=10, color="gray")
        fig.canvas.draw_idle()
        return

    all_z = sorted(layers.keys())

    for z in all_z:
        layer = layers[z]
        # Skip unknown cells (most numerous, slow to render, add little information)
        _add_polys(ax, z, layer,  0,  _C_FREE)             # green: free
        _add_polys(ax, z, layer, 100, _C_OCC,              # red: occupied
                   ec=(0.55, 0.05, 0.05, 1.0), lw=0.25)

    # ── axis limits ───────────────────────────────────────────────────────────
    sample = next(iter(layers.values()))
    res, oe, on, h, w = (
        sample["res"], sample["oe"], sample["on"],
        sample["h"], sample["w"],
    )
    ax.set_xlim(oe, oe + w * res)
    ax.set_ylim(on, on + h * res)
    ax.set_zlim(all_z[0] - 0.05, all_z[-1] + 0.55)

    # Z ticks at each layer floor
    ax.set_zticks(all_z)
    ax.set_zticklabels([f"{z:.2f}" for z in all_z], fontsize=7)

    # ── legend ────────────────────────────────────────────────────────────────
    ax.legend(handles=[
        mpatches.Patch(color=_C_OCC[:3],  alpha=0.9, label="Occupied / inflated"),
        mpatches.Patch(color=_C_FREE[:3], alpha=0.6, label="Free space"),
    ], loc="upper left", fontsize=8, framealpha=0.6)

    fig.canvas.draw_idle()
    fig.canvas.flush_events()


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Isometric layer visualizer")
    parser.add_argument("--topic", default="/occupancy_map",
                        help="OccupancyGrid topic (default: /occupancy_map)")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=[sys.argv[0]] + ros_args)
    collector = LayerCollector(args.topic)

    # ROS2 spin runs in a daemon thread; matplotlib owns the main thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(collector,), daemon=True)
    ros_thread.start()

    plt.ion()
    fig = plt.figure("Drone Scan Layers — Isometric", figsize=(11, 7))
    ax  = fig.add_subplot(111, projection="3d")
    plt.tight_layout()

    # Show "waiting" state immediately so the user sees the window is alive
    redraw(fig, ax, collector)
    plt.show(block=False)

    print(f"Visualizer running — subscribed to {args.topic}")
    print("Press Ctrl-C to exit.")
    try:
        while rclpy.ok():
            if collector.dirty:
                redraw(fig, ax, collector)
            # plt.pause() runs the GUI event loop for the given duration.
            # Do NOT use time.sleep() here — it freezes the window.
            plt.pause(0.5)   # 500 ms: responsive enough to see layers appear one by one
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        collector.destroy_node()
        rclpy.shutdown()
        plt.close("all")


if __name__ == "__main__":
    main()
