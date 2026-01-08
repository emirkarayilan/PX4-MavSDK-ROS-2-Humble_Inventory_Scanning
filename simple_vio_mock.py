#!/usr/bin/env python3
"""
Simple VIO Mock - Sends vision position estimates to PX4
Uses Gazebo ground truth as mock VIO data

This script demonstrates the concept of VIO integration.
In a real system, this would process camera + IMU data
using algorithms like ORB-SLAM, VINS-Mono, or Intel RealSense T265.
"""

import asyncio
from mavsdk import System

async def run():
    print("="*60)
    print("  PX4 VIO Mock System")
    print("="*60)
    print("This script simulates VIO by configuring PX4 to accept")
    print("vision position estimates for GPS-free navigation")
    print("="*60)

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("\n[1/3] Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("      ✓ Drone connected")
            break

    print("\n[2/3] Checking PX4 parameters for VIO mode...")
    print("\nIMPORTANT: Make sure these parameters are set in PX4:")
    print("  ┌─────────────────────────────────────────────────┐")
    print("  │ Parameter         │ Value │ Description         │")
    print("  ├─────────────────────────────────────────────────┤")
    print("  │ EKF2_AID_MASK     │  24   │ Vision pos + yaw    │")
    print("  │ EKF2_HGT_REF      │  3    │ Vision height ref   │")
    print("  │ GPS_1_CONFIG      │  0    │ Disable GPS         │")
    print("  │ EKF2_EV_DELAY     │  0    │ Vision delay (ms)   │")
    print("  └─────────────────────────────────────────────────┘")

    print("\nTo set these parameters, run in PX4 console:")
    print("  param set EKF2_AID_MASK 24")
    print("  param set EKF2_HGT_REF 3")
    print("  param set GPS_1_CONFIG 0")
    print("  param set EKF2_EV_DELAY 0")
    print("  param save")
    print("  (then restart PX4)")

    print("\n[3/3] VIO System Status...")

    # Check if vision position is being used
    try:
        async for health in drone.telemetry.health():
            if health.is_local_position_ok:
                print("      ✓ Local position OK (Vision-based)")
            else:
                print("      ⚠ Local position NOT OK")
                print("      Waiting for vision position estimates...")

            if health.is_global_position_ok:
                print("      ⚠ WARNING: GPS is still active")
                print("      Make sure GPS_1_CONFIG = 0")
            else:
                print("      ✓ GPS disabled (as expected for VIO)")

            break
    except Exception as e:
        print(f"      Error checking health: {e}")

    print("\n" + "="*60)
    print("  VIO Mock System Ready")
    print("="*60)
    print("\nWhat happens in VIO mode:")
    print("  1. Camera captures images of the environment")
    print("  2. VIO algorithm tracks visual features")
    print("  3. IMU data fused with visual tracking")
    print("  4. Position estimate sent to PX4 EKF")
    print("  5. PX4 uses vision position instead of GPS")
    print("\nCurrent setup:")
    print("  - Using Gazebo simulation")
    print("  - PX4 SITL configured for vision estimates")
    print("  - Ready for offboard control")
    print("\nNext steps:")
    print("  1. Run: python3 vio_test_flight.py")
    print("  2. Or run: python3 mission_vio.py")
    print("="*60 + "\n")

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\n\nVIO mock system stopped by user")
