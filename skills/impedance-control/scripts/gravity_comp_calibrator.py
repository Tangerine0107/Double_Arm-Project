#!/usr/bin/env python3
"""
Gravity Compensation Calibration Helper

Interactive script to calibrate tau_ff_scale per joint by measuring drift
in teach mode. Helps the user systematically adjust gravity compensation.

Usage:
    python gravity_comp_calibrator.py --joint 0 --target 0.0 --duration 5.0

Requirements:
    - ROS 2 with JointTrajectoryController or custom impedance controller
    - Joint state publisher running
    - Arm in teach mode (low Kp)
"""

import argparse
import time
import math
import sys
from typing import Optional, List, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print("[WARN] ROS 2 not available. Running in simulation/mock mode.")


class DriftMeasurer:
    """Measure joint position drift over time to assess gravity compensation."""

    def __init__(self, joint_names: List[str]):
        self.joint_names = joint_names
        self.positions = {}  # joint_name -> [pos_samples]
        self.velocities = {}  # joint_name -> [vel_samples]
        self.start_time = None
        self.end_time = None

    def add_sample(self, joint_positions: dict, joint_velocities: dict):
        """Add a joint state sample."""
        for name in self.joint_names:
            if name in joint_positions:
                if name not in self.positions:
                    self.positions[name] = []
                self.positions[name].append(joint_positions[name])
            if name in joint_velocities:
                if name not in self.velocities:
                    self.velocities[name] = []
                self.velocities[name].append(joint_velocities[name])

    def compute_drift(self, joint_name: str) -> Tuple[float, float, float]:
        """
        Compute drift statistics for a joint.

        Returns: (total_drift_rad, drift_rate_rad_per_s, avg_velocity_rad_per_s)
        """
        if joint_name not in self.positions or len(self.positions[joint_name]) < 2:
            return 0.0, 0.0, 0.0

        positions = self.positions[joint_name]
        total_drift = positions[-1] - positions[0]
        duration = (self.end_time - self.start_time) if self.end_time and self.start_time else 1.0
        drift_rate = total_drift / duration if duration > 0 else 0.0

        avg_vel = 0.0
        if joint_name in self.velocities and len(self.velocities[joint_name]) > 0:
            avg_vel = sum(self.velocities[joint_name]) / len(self.velocities[joint_name])

        return total_drift, drift_rate, avg_vel

    def suggest_scale_adjustment(self, joint_name: str, current_scale: float) -> float:
        """
        Suggest tau_ff_scale adjustment based on drift direction.

        Positive drift (arm moving up/forward) = too much compensation → decrease scale
        Negative drift (arm drifting down) = too little compensation → increase scale
        """
        total_drift, drift_rate, _ = self.compute_drift(joint_name)

        # Convert drift rate to scale adjustment (heuristic)
        # Typical drift rate of 0.05 rad/s → ~0.02 scale adjustment
        adjustment = -drift_rate * 0.4  # negative feedback
        adjustment = max(-0.05, min(0.05, adjustment))  # clamp to ±0.05 per step

        return current_scale + adjustment


def calibrate_joint(joint_idx: int, joint_name: str, current_scale: float,
                    duration: float = 5.0) -> dict:
    """
    Run a single calibration measurement for one joint.

    Returns dict with drift stats and suggested new scale.
    """
    print(f"\n{'='*60}")
    print(f"Calibrating joint {joint_idx} ({joint_name})")
    print(f"Current tau_ff_scale: {current_scale:.4f}")
    print(f"{'='*60}")
    print(f"\nInstructions:")
    print(f"  1. Put arm in teach mode (Kp = kp_min)")
    print(f"  2. Move joint {joint_idx} to a representative position")
    print(f"  3. Hold the arm steady")
    print(f"  4. Press ENTER to start measurement ({duration}s)")
    input("\nPress ENTER when ready...")

    measurer = DriftMeasurer([joint_name])
    measurer.start_time = time.time()

    # Simulate measurement (replace with ROS 2 subscription in real use)
    print(f"\nMeasuring drift for {duration}s...", end="", flush=True)
    for i in range(int(duration * 20)):  # 20 Hz sample rate
        # In real use: read from /joint_states topic
        # Here we simulate a small drift
        t = i / 20.0
        simulated_pos = 0.0 + 0.02 * t  # 0.02 rad/s drift (example)
        simulated_vel = 0.02
        measurer.add_sample({joint_name: simulated_pos}, {joint_name: simulated_vel})
        time.sleep(0.05)
        if i % 20 == 0:
            print(".", end="", flush=True)

    print(" done!")
    measurer.end_time = time.time()

    total_drift, drift_rate, avg_vel = measurer.compute_drift(joint_name)
    suggested_scale = measurer.suggest_scale_adjustment(joint_name, current_scale)

    result = {
        "joint_idx": joint_idx,
        "joint_name": joint_name,
        "current_scale": current_scale,
        "total_drift_rad": total_drift,
        "drift_rate_rad_per_s": drift_rate,
        "avg_velocity_rad_per_s": avg_vel,
        "suggested_scale": suggested_scale,
        "scale_delta": suggested_scale - current_scale,
    }

    print(f"\n  Results:")
    print(f"    Total drift:     {total_drift*1000:.1f} mrad ({total_drift*180/math.pi:.2f} deg)")
    print(f"    Drift rate:      {drift_rate*1000:.1f} mrad/s")
    print(f"    Avg velocity:    {avg_vel*1000:.1f} mrad/s")
    print(f"    Suggested scale: {suggested_scale:.4f} (Δ={suggested_scale - current_scale:+.4f})")

    drift_direction = "UP (too much compensation)" if total_drift > 0 else "DOWN (too little compensation)"
    print(f"    Drift direction: {drift_direction}")

    return result


def main():
    parser = argparse.ArgumentParser(description="Gravity Compensation Calibration Helper")
    parser.add_argument("--joint", type=int, default=None,
                        help="Joint index to calibrate (0-6). If omitted, calibrate all joints.")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Measurement duration in seconds (default: 5.0)")
    parser.add_argument("--iterations", type=int, default=1,
                        help="Number of calibration iterations per joint (default: 1)")
    parser.add_argument("--joint-names", type=str, nargs="+",
                        default=["L1", "L2", "L3", "L4", "L5", "L6", "L7"],
                        help="Joint names (default: L1 L2 ... L7)")
    parser.add_argument("--initial-scale", type=float, default=1.0,
                        help="Initial tau_ff_scale for all joints (default: 1.0)")
    args = parser.parse_args()

    print("=" * 60)
    print("  Gravity Compensation Calibration Helper")
    print("=" * 60)
    print(f"  Measurement duration: {args.duration}s")
    print(f"  Iterations per joint: {args.iterations}")
    print(f"  Initial scale: {args.initial_scale}")

    # Determine which joints to calibrate
    if args.joint is not None:
        joint_indices = [args.joint]
    else:
        joint_indices = list(range(len(args.joint_names)))

    scales = [args.initial_scale] * len(args.joint_names)
    all_results = []

    for iteration in range(args.iterations):
        print(f"\n{'#'*60}")
        print(f"# Iteration {iteration + 1}/{args.iterations}")
        print(f"{'#'*60}")

        for j in joint_indices:
            result = calibrate_joint(j, args.joint_names[j], scales[j], args.duration)
            all_results.append(result)
            scales[j] = result["suggested_scale"]

    # Final summary
    print(f"\n\n{'='*60}")
    print("  CALIBRATION SUMMARY")
    print(f"{'='*60}")
    print(f"\n  {'Joint':<8} {'Final Scale':>12} {'Δ from start':>14}")
    print(f"  {'-'*8} {'-'*12} {'-'*14}")
    for j in joint_indices:
        delta = scales[j] - args.initial_scale
        print(f"  {args.joint_names[j]:<8} {scales[j]:>12.4f} {delta:>+14.4f}")

    print(f"\n  Final tau_ff_scale array (copy to YAML config):")
    scale_str = ", ".join([f"{s:.4f}" for s in scales])
    print(f"    [{scale_str}]")

    print(f"\n  Next steps:")
    print(f"    1. Apply the new tau_ff_scale values")
    print(f"    2. Verify across multiple workspace positions")
    print(f"    3. If drift still visible, run another iteration")
    print(f"    4. Fine-tune friction parameters for better zero-velocity behavior")

    return 0


if __name__ == "__main__":
    sys.exit(main())
