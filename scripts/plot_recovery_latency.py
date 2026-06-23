#!/usr/bin/env python3
"""
Plot fault detection and recovery latency from results/metrics.csv.

Usage:
    python3 scripts/plot_recovery_latency.py [--csv PATH] [--out-dir PATH]

Outputs:
    results/recovery_latency.png    — bar chart: median latency per fault type
    results/bus_health_timeline.png — frame rate + invalid ratio over time

Also prints a 90-second interview summary to stdout.
"""

import argparse
import sys
import os
import csv
from collections import defaultdict

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--csv',     default='results/metrics.csv')
    p.add_argument('--out-dir', default='results')
    return p.parse_args()


def load_metrics(path):
    if not os.path.exists(path):
        print(f"[ERROR] metrics file not found: {path}")
        print("  Run the demo first:")
        print("    ros2 launch can_fault_monitor can_demo.launch.py")
        print("  Then inject faults:")
        print("    ros2 service call /fault_injector/inject_fault \\")
        print("      can_fault_monitor/srv/InjectFault \\")
        print("      '{fault_type: \"NODE_DROPOUT\", target_can_id: 256, duration_sec: 2.0}'")
        sys.exit(1)

    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                'ts':       float(row['timestamp_sec']),
                'fault':    row['fault_type'],
                'det_ms':   float(row['detection_latency_ms']),
                'rec_ms':   float(row['recovery_latency_ms']),
                'fps':      float(row['bus_frame_rate_hz']),
                'ratio':    float(row['invalid_frame_ratio']),
            })
    return rows


def print_summary(rows):
    by_fault = defaultdict(list)
    for r in rows:
        by_fault[r['fault']].append(r)

    print("\n" + "="*60)
    print("  CAN FAULT MONITOR — RESULTS SUMMARY")
    print("="*60)
    for fault, events in sorted(by_fault.items()):
        det_vals = [e['det_ms'] for e in events if e['det_ms'] > 0]
        rec_vals = [e['rec_ms'] for e in events if e['rec_ms'] > 0]
        print(f"\n  {fault}")
        print(f"    Events: {len(events)}")
        if det_vals:
            print(f"    Detection latency:  median={median(det_vals):.1f} ms  max={max(det_vals):.1f} ms")
        if rec_vals:
            print(f"    Recovery latency:   median={median(rec_vals):.1f} ms  max={max(rec_vals):.1f} ms")
    print("\n" + "="*60)

    print("""
  INTERVIEW TALKING POINTS:
  ─────────────────────────
  • NODE_DROPOUT: detected within ~100–120 ms (5 missed frames at 50 Hz +
    20 ms eval loop jitter). Exponential backoff recovery avoids thundering herd.

  • FRAME_CORRUPTION: 500 ms sliding window ratio > 20% triggers DEGRADED.
    DLC=9 is the illegal-frame marker — caught at SocketCAN layer before the
    CRC or ACK mechanism runs, so it's detected at the application layer.

  • BUS_FLOOD: 100 ms window catches a 2,000 fps runaway within 1–3 timer ticks.
    On physical CAN (1 Mbit/s), the bandwidth ceiling of ~7,700 fps means BUS_FLOOD
    at 2,000 fps would measurably delay 0x100 heartbeats before detection.

  • Hardware Delta: vcan0 has no TEC/REC counters, no bus-off state machine,
    no error frames, and no timing jitter. To run on real CAN: change
    interface: "can0" in config/params.yaml. Zero code changes.
""")


def median(vals):
    s = sorted(vals)
    n = len(s)
    return (s[n//2] + s[(n-1)//2]) / 2


def plot(rows, out_dir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        import numpy as np
    except ImportError:
        print("[WARN] matplotlib not installed. Skipping plots.")
        print("  pip install matplotlib --break-system-packages")
        return

    os.makedirs(out_dir, exist_ok=True)

    # ── Plot 1: Recovery latency bar chart ──────────────────────────────────
    fault_types = ['NODE_DROPOUT', 'FRAME_CORRUPTION', 'BUS_FLOOD']
    colors      = ['#E74C3C', '#F39C12', '#2980B9']

    det_medians = []
    rec_medians = []
    labels_present = []
    colors_present = []

    by_fault = defaultdict(list)
    for r in rows:
        by_fault[r['fault']].append(r)

    for ft, color in zip(fault_types, colors):
        events = by_fault.get(ft, [])
        if not events:
            continue
        det_vals = [e['det_ms'] for e in events if e['det_ms'] > 0]
        rec_vals = [e['rec_ms'] for e in events if e['rec_ms'] > 0]
        det_medians.append(median(det_vals) if det_vals else 0)
        rec_medians.append(median(rec_vals) if rec_vals else 0)
        labels_present.append(ft.replace('_', '\n'))
        colors_present.append(color)

    if labels_present:
        x = np.arange(len(labels_present))
        width = 0.35
        fig, ax = plt.subplots(figsize=(9, 5))
        bars1 = ax.bar(x - width/2, det_medians, width, label='Detection latency',  color=colors_present, alpha=0.85)
        bars2 = ax.bar(x + width/2, rec_medians, width, label='Recovery latency',   color=colors_present, alpha=0.45, hatch='//')
        ax.set_title('CAN Fault Detection & Recovery Latency (median)', fontsize=13, fontweight='bold')
        ax.set_ylabel('Latency (ms)')
        ax.set_xticks(x); ax.set_xticklabels(labels_present, fontsize=9)
        ax.axhline(100, color='gray', linestyle='--', linewidth=0.8, label='50Hz deadline (100 ms)')
        ax.legend()
        for bar in list(bars1) + list(bars2):
            h = bar.get_height()
            if h > 0:
                ax.annotate(f'{h:.0f}', xy=(bar.get_x() + bar.get_width()/2, h),
                            xytext=(0, 3), textcoords='offset points', ha='center', fontsize=8)
        fig.tight_layout()
        path = os.path.join(out_dir, 'recovery_latency.png')
        fig.savefig(path, dpi=150)
        print(f"  [OK] {path}")
        plt.close(fig)

    # ── Plot 2: Bus health timeline ──────────────────────────────────────────
    if rows:
        ts     = [r['ts'] - rows[0]['ts'] for r in rows]
        fps    = [r['fps']   for r in rows]
        ratios = [r['ratio'] for r in rows]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        ax1.plot(ts, fps, color='#2980B9', linewidth=1.2)
        ax1.axhline(500, color='#E74C3C', linestyle='--', linewidth=0.8, label='Flood threshold (500 fps)')
        ax1.set_ylabel('Frame rate (fps)'); ax1.set_title('Bus Health Timeline')
        ax1.legend(fontsize=8)
        ax2.plot(ts, ratios, color='#E67E22', linewidth=1.2)
        ax2.axhline(0.20, color='#E74C3C', linestyle='--', linewidth=0.8, label='Corruption threshold (20%)')
        ax2.set_ylabel('Invalid frame ratio'); ax2.set_xlabel('Time (s)')
        ax2.legend(fontsize=8)
        fig.tight_layout()
        path = os.path.join(out_dir, 'bus_health_timeline.png')
        fig.savefig(path, dpi=150)
        print(f"  [OK] {path}")
        plt.close(fig)


def main():
    args   = parse_args()
    rows   = load_metrics(args.csv)
    print_summary(rows)
    plot(rows, args.out_dir)


if __name__ == '__main__':
    main()
