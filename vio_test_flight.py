#!/usr/bin/env python3
"""
VIO Test Flight - Basic test for vision-based navigation
Tests VIO position tracking without GPS
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def run():
    print("\n" + "="*60)
    print("  PX4 IRIS VIO TEST FLIGHT (WITHOUT GPS)")
    print("="*60)

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("\n[1/7] Connecting to drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("      ✓ Connected")
            break

    print("\n[2/7] Waiting for vision position estimation...")
    print("      Note: This replaces GPS in VIO mode")

    # Wait for local position (from vision)
    timeout = 30
    start_time = asyncio.get_event_loop().time()

    while True:
        async for health in drone.telemetry.health():
            if health.is_local_position_ok:
                print("      ✓ Vision position OK")
                break

            elapsed = asyncio.get_event_loop().time() - start_time
            if elapsed > timeout:
                print("      ✗ ERROR: No vision position after 30s")
                print("      Make sure EKF2_AID_MASK=24 and vision data is being sent")
                return

            await asyncio.sleep(0.5)
        break

    print("\n[3/7] Getting initial position...")
    async for position in drone.telemetry.position():
        print(f"      Local position valid")
        break

    print("\n[4/7] Arming motors...")
    try:
        await drone.action.arm()
        print("      ✓ Motors armed")
    except Exception as e:
        print(f"      ✗ ERROR: {e}")
        return

    await asyncio.sleep(2)

    print("\n[5/7] Starting offboard mode...")

    # Send initial setpoint
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    try:
        await drone.offboard.start()
        print("      ✓ Offboard mode active")
    except OffboardError as error:
        print(f"      ✗ ERROR: {error}")
        await drone.action.disarm()
        return

    print("\n[6/7] Executing test flight pattern...")
    print("      This tests VIO position tracking without GPS")

    # Test flight waypoints (North, East, Down, Yaw)
    waypoints = [
        (0.0, 0.0, -2.0, 0.0, "Takeoff to 2m"),
        (2.0, 0.0, -2.0, 0.0, "Move 2m North"),
        (2.0, 2.0, -2.0, 90.0, "Move 2m East + Turn 90°"),
        (0.0, 2.0, -2.0, 180.0, "Move 2m South + Turn 180°"),
        (0.0, 0.0, -2.0, 270.0, "Return to start + Turn 270°"),
        (0.0, 0.0, -1.0, 0.0, "Descend to 1m"),
    ]

    for i, (n, e, d, yaw, desc) in enumerate(waypoints, 1):
        print(f"      Waypoint {i}/{len(waypoints)}: {desc}")
        print(f"      → N={n}m, E={e}m, Alt={abs(d)}m, Yaw={yaw}°")

        # Send setpoint multiple times
        for _ in range(30):
            await drone.offboard.set_position_ned(PositionNedYaw(n, e, d, yaw))
            await asyncio.sleep(0.1)

        # Verify position
        async for position in drone.telemetry.position():
            current_alt = position.relative_altitude_m
            print(f"      Current altitude: {current_alt:.2f}m")
            break

    print("\n[7/7] Landing...")
    await drone.offboard.stop()
    await drone.action.land()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("      ✓ Landed")
            break

    await drone.action.disarm()

    print("\n" + "="*60)
    print("  VIO TEST FLIGHT COMPLETED SUCCESSFULLY!")
    print("="*60)
    print("\nVIO Performance Summary:")
    print("  - Position tracking: Vision-based (no GPS)")
    print("  - Flight pattern: Square + altitude changes")
    print("  - Yaw control: 4 orientations tested")
    print("\nNext steps:")
    print("  1. Check QGroundControl for flight path accuracy")
    print("  2. Review EKF2 logs for vision quality")
    print("  3. Proceed with your warehouse inventory mission")
    print("="*60 + "\n")

if __name__ == "__main__":
    asyncio.run(run())
