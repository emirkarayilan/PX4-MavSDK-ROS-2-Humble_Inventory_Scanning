#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def run():
    
    drone = System()
    
    print("\nDepo Raf Tarama Gorevi Basladi")
    print("-" * 50)
    
    print("\n[1/6] Drone baglantisi kuruluyor...")
    await drone.connect(system_address="udp://:14540")
    
    connection_timeout = 10
    connected = False
    
    async def check_connection():
        """bağlantı kontrolü"""
        nonlocal connected
        async for state in drone.core.connection_state():
            if state.is_connected:
                connected = True
                return
    
    try:
        await asyncio.wait_for(check_connection(), timeout=connection_timeout)
        print("      Baglanti basarili")
    except asyncio.TimeoutError:
        print("      HATA: Baglanti zaman asimi")
        return
    
    print("\n[2/6] GPS sinyali bekleniyor...")
    gps_timeout = 30
    
    async def check_gps():
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                return True
            await asyncio.sleep(1)
    
    try:
        await asyncio.wait_for(check_gps(), timeout=gps_timeout)
        print("      GPS hazir")
    except asyncio.TimeoutError:
        print("      HATA: GPS sinyali alinamadi")
        print("      Gazebo'da drone spawn edilmis olabilir")
        return
    
    print("\n[3/6] Baslangic pozisyonu aliniyor...")
    async for position in drone.telemetry.position():
        print(f"      Enlem: {position.latitude_deg:.6f}")
        print(f"      Boylam: {position.longitude_deg:.6f}")
        print(f"      Yukseklik: {position.absolute_altitude_m:.2f}m")
        break
    
    print("\n[4/6] Motorlar calistiriliyor...")
    try:
        await drone.action.arm()
        print("      Motorlar calisiyor")
    except Exception as e:
        print(f"      HATA: Motorlar calistirilamadi - {e}")
        return
    
    await asyncio.sleep(2)
    async for armed in drone.telemetry.armed():
        if not armed:
            print("      HATA: Motorlar kapandi")
            return
        break

    print("\n[5/6] Offboard modu aktiflestirilyor...")
    
    for _ in range(50):
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
        await asyncio.sleep(0.05)
    
    try:
        await drone.offboard.start()
        print("      Offboard modu aktif")
    except OffboardError as error:
        print(f"      HATA: Offboard baslatilmadi - {error}")
        await drone.action.disarm()
        return
    

    print("\n[6/6] Kalkis yapiliyor (2m yukseklige)...")
    
    
    for i in range(40):
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
        
        
        if i % 10 == 0:
            async for position in drone.telemetry.position():
                current_alt = position.relative_altitude_m
                print(f"      Guncel yukseklik: {current_alt:.2f}m")
                break
        
        await asyncio.sleep(0.2)
    
    print("\nKalkis tamamlandi, raf taramasina baslanıyor...")
    print("-" * 50)
    
    coordinates = [
        # Ondeki raf - alt kat (on tarafa bakiyor, yaw=0)
        (2.0, 0.8, -0.35, 0),
        (2.0, 0.8, -1.25, 0),
        (2.0, 0.8, -2.05, 0),
        
        # Ondeki raf - orta kat (on tarafa bakiyor, yaw=0)
        (2.0, 1.4, -1.95, 0),
        (2.0, 1.4, -1.25, 0),
        (2.0, 1.4, -0.15, 0),
        
        # Ondeki raf - ust kat (on tarafa bakiyor, yaw=0)
        (2.0, 2.0, -0.35, 0),
        (2.0, 2.0, -1.25, 0),
        (2.0, 2.0, -2.05, 0),
        (2.0, 2.0, -3.85, 0),
        
        # Koridor giriş
        (4.4, 2.05, -3.85, 0), 
        (4.4, 2.05, -1.90, 180), #180 dönüş        
        (4.4, 2.05, -1.25, 180),
        (4.4, 2.05, -0.35, 180),
        
        
        # Ondeki rafin ARKASI - ust kat (arkaya bakiyor, yaw=180)
        (4.4, 1.4, -0.35, 180),
        (4.4, 1.4, -1.25, 180),
        (4.4, 1.4, -2.05, 180),
        
        # Ondeki rafin ARKASI - orta kat (arkaya bakiyor, yaw=180)
        (4.4, 0.8, -2.05, 180),
        (4.4, 0.8, -1.25, 180),
        (4.4, 0.8, -0.35, 180),
        
        # Ondeki rafin ARKASI - alt kat (arkaya bakiyor, yaw=180)
        (4.4, 0.8, -0.35, 180),
        (4.4, 0.8, -1.25, 180),
        (4.4, 0.8, -2.05, 180),
        
        # TEKRAR DÖNÜŞ: Normal yöne don (arka rafa gecis icin)
        (4.4, 0.8, -2.05, 0),
        
        # Arka raf - birinci bolum (one bakiyor, yaw=0)
        (4.4, 0.8, -1.25, 0),
        (4.4, 0.8, -0.35, 0),
        
        # Arka raf - ikinci bolum
        (4.4, 1.4, -0.35, 0),
        (4.4, 1.4, -1.25, 0),
        (4.4, 1.4, -2.05, 0),
        
        # Arka raf - ucuncu bolum
        (4.4, 0.9, -2.05, 0),
        (4.4, 0.9, -1.25, 0),
        (4.4, 0.9, -0.35, 0),
        
        # Arka raf - dorduncu bolum
        (4.4, 0.3, -0.35, 0),
        (4.4, 0.3, -1.25, 0),
        (4.4, 0.3, -1.95, 0),
        
        # Arka raf - besinci bolum
        (4.4, -0.4, -1.90, 0),
        (4.4, -0.4, -1.25, 0),
        (4.4, -0.9, -0.35, 0),
        
        # Gorev bitirme adimlari
        (4.4, -0.9, -3.75, 0),
        (0.0, 0.0, -3.75, 0),
        (0.0, 0.0, 0.0, 0),
    ]
    
    toplam_nokta = len(coordinates)
    
    for i, (x, y, z, yaw) in enumerate(coordinates, start=1):
        if i > 1 and yaw != coordinates[i-2][3]:
            print(f"\n>>> Drone yonu degistiriliyor: {yaw} derece")
        
        print(f"Nokta {i}/{toplam_nokta}: N={x}m, E={y}m, H={abs(z):.2f}m, Yon={yaw}°")
        
        for _ in range(30):
            await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, yaw))
            await asyncio.sleep(0.2)
    
    print("\n" + "-" * 50)
    print("Tum raf noktalari tarandı")
    print("-" * 50)
    
    print("\nInis yapiliyor...")
    
    await drone.offboard.stop()
    
    await drone.action.land()
    
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Inis tamamlandi")
            break
    
    await drone.action.disarm()
    
    print("\nGorev basariyla tamamlandi")
    print("-" * 50)

if __name__ == "__main__":
    asyncio.run(run())
