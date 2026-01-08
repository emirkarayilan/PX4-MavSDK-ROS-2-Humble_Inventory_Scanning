#!/usr/bin/env python3
"""
Warehouse Inventory Mission - VIO Version (GPS-Free)
Based on original mission.py but adapted for vision-based navigation
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def run():
    drone = System()

    print("\n🏭 Warehouse Inventory Scanning Mission (VIO Mode)")
    print("="*60)
    print("Navigation: Visual Inertial Odometry (No GPS Required)")
    print("="*60)

    print("\n[1/6] Connecting to drone...")
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("      ✓ Connection established")
            break

    print("\n[2/6] Waiting for VIO position lock...")
    print("      (Replacing GPS with vision-based localization)")

    vio_timeout = 30
    vio_ready = False

    async def check_vio():
        """Check for vision position instead of GPS"""
        nonlocal vio_ready
        async for health in drone.telemetry.health():
            # In VIO mode, we check local position instead of global
            if health.is_local_position_ok and health.is_home_position_ok:
                vio_ready = True
                return
            await asyncio.sleep(0.5)

    try:
        await asyncio.wait_for(check_vio(), timeout=vio_timeout)
        print("      ✓ VIO position lock acquired")
    except asyncio.TimeoutError:
        print("      ✗ ERROR: VIO position not available")
        print("      Make sure:")
        print("        - EKF2_AID_MASK = 24")
        print("        - EKF2_HGT_REF = 3")
        print("        - Vision position data is being sent")
        return

    print("\n[3/6] Getting initial position (from VIO)...")
    async for position in drone.telemetry.position():
        print(f"      Local position: {position.relative_altitude_m:.2f}m altitude")
        break

    print("\n[4/6] Arming motors...")
    try:
        await drone.action.arm()
        print("      ✓ Motors armed")
    except Exception as e:
        print(f"      ✗ ERROR: Cannot arm - {e}")
        return

    await asyncio.sleep(2)

    print("\n[5/6] Activating offboard mode...")

    # Send initial setpoint
    for _ in range(50):
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
        await asyncio.sleep(0.05)

    try:
        await drone.offboard.start()
        print("      ✓ Offboard mode active")
    except OffboardError as error:
        print(f"      ✗ ERROR: {error}")
        await drone.action.disarm()
        return

    print("\n[6/6] Taking off to 2m altitude...")

    for i in range(40):
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))

        if i % 10 == 0:
            async for position in drone.telemetry.position():
                current_alt = position.relative_altitude_m
                print(f"      Current altitude: {current_alt:.2f}m")
                break

        await asyncio.sleep(0.2)

    print("\n✓ Takeoff complete, starting shelf scanning...")
    print("="*60)

    # Warehouse scanning coordinates (same as original mission)
    coordinates = [
        # Front shelf - lower level
        (2.0, 0.8, -0.35, 0),
        (2.0, 0.8, -1.25, 0),
        (2.0, 0.8, -2.05, 0),

        # Front shelf - middle level
        (2.0, 1.4, -1.95, 0),
        (2.0, 1.4, -1.25, 0),
        (2.0, 1.4, -0.15, 0),

        # Front shelf - upper level
        (2.0, 2.0, -0.35, 0),
        (2.0, 2.0, -1.25, 0),
        (2.0, 2.0, -2.05, 0),
        (2.0, 2.0, -3.85, 0),

        # Corridor entry
        (4.4, 2.05, -3.85, 0),
        (4.4, 2.05, -1.90, 180),
        (4.4, 2.05, -1.25, 180),
        (4.4, 2.05, -0.35, 180),

        # Back of front shelf - upper level
        (4.4, 1.4, -0.35, 180),
        (4.4, 1.4, -1.25, 180),
        (4.4, 1.4, -2.05, 180),

        # Back of front shelf - middle level
        (4.4, 0.8, -2.05, 180),
        (4.4, 0.8, -1.25, 180),
        (4.4, 0.8, -0.35, 180),

        # Back of front shelf - lower level
        (4.4, 0.8, -0.35, 180),
        (4.4, 0.8, -1.25, 180),
        (4.4, 0.8, -2.05, 180),

        # Return mission and land
        (4.4, 0.8, -2.05, 0),
        (4.4, 0.8, -1.25, 0),
        (4.4, 0.8, -0.35, 0),
        (0.0, 0.0, -2.0, 0),
        (0.0, 0.0, 0.0, 0),
    ]

    total_points = len(coordinates)

    for i, (x, y, z, yaw) in enumerate(coordinates, start=1):
        if i > 1 and yaw != coordinates[i-2][3]:
            print(f"\n>>> Changing orientation: {yaw}°")

        print(f"Point {i}/{total_points}: N={x}m, E={y}m, Alt={abs(z):.2f}m, Yaw={yaw}°")

        for _ in range(30):
            await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, yaw))
            await asyncio.sleep(0.2)

    print("\n" + "="*60)
    print("✓ All shelf points scanned")
    print("="*60)

    print("\n🛬 Landing...")
    await drone.offboard.stop()
    await drone.action.land()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("      ✓ Landed successfully")
            break

    await drone.action.disarm()

    print("\n" + "="*60)
    print("🎯 MISSION COMPLETED SUCCESSFULLY (VIO MODE)")
    print("="*60)
    print("\nMission Statistics:")
    print(f"  - Total waypoints: {total_points}")
    print("  - Navigation mode: VIO (GPS-free)")
    print("  - Altitude levels: Multiple")
    print("  - Orientation changes: Front/Back views")
    print("="*60 + "\n")

if __name__ == "__main__":
    asyncio.run(run())
