#!/usr/bin/env python3
# Drone ile depo raflarındaki kutuları tarama görevi

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def run():
    """
    Drone'u depo rafları arasında gezdirir ve kutuları tarar
    """
    
    # Drone nesnesi olustur
    drone = System()
    
    print("\nDepo Raf Tarama Gorevi Basladi")
    print("-" * 50)
    
    # ========================================
    # 1. BAGLANTI KURMA
    # ========================================
    print("\n[1/6] Drone baglantisi kuruluyor...")
    await drone.connect(system_address="udp://:14540")
    
    # Baglanti kurmak icin 10 saniye bekle
    connection_timeout = 10
    connected = False
    
    async def check_connection():
        """Drone baglantisini kontrol et"""
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
        print("      PX4 SITL veya Gazebo calismiyor olabilir")
        return
    
    # ========================================
    # 2. GPS HAZIR BEKLEME
    # ========================================
    print("\n[2/6] GPS sinyali bekleniyor...")
    gps_timeout = 30
    
    async def check_gps():
        """GPS hazir olana kadar bekle"""
        async for health in drone.telemetry.health():
            # GPS durumunu kontrol et
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
    
    # ========================================
    # 3. BASLANGIC POZISYONU
    # ========================================
    print("\n[3/6] Baslangic pozisyonu aliniyor...")
    async for position in drone.telemetry.position():
        print(f"      Enlem: {position.latitude_deg:.6f}")
        print(f"      Boylam: {position.longitude_deg:.6f}")
        print(f"      Yukseklik: {position.absolute_altitude_m:.2f}m")
        break
    
    # ========================================
    # 4. MOTORLARI CALISTIR (ARM)
    # ========================================
    print("\n[4/6] Motorlar calistiriliyor...")
    try:
        await drone.action.arm()
        print("      Motorlar calisiyor")
    except Exception as e:
        print(f"      HATA: Motorlar calistirilamadi - {e}")
        return
    
    # Motorlarin gercekten calistigini dogrula
    await asyncio.sleep(2)
    async for armed in drone.telemetry.armed():
        if not armed:
            print("      HATA: Motorlar kapandi")
            return
        break
    
    # ========================================
    # 5. OFFBOARD MODU AKTIF ET
    # ========================================
    print("\n[5/6] Offboard modu aktiflestirilyor...")
    
    # Offboard baslatmadan once pozisyon akisi gonder
    # PX4, offboard moduna gecmeden once pozisyon almak ister
    for _ in range(50):
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
        await asyncio.sleep(0.05)
    
    # Offboard modunu baslat
    try:
        await drone.offboard.start()
        print("      Offboard modu aktif")
    except OffboardError as error:
        print(f"      HATA: Offboard baslatilmadi - {error}")
        await drone.action.disarm()
        return
    
    # ========================================
    # 6. KALKIS
    # ========================================
    print("\n[6/6] Kalkis yapiliyor (2m yukseklige)...")
    
    # 8 saniye boyunca 2m yukseklige cik
    for i in range(40):
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
        
        # Her 2 saniyede bir yukseklik bilgisi goster
        if i % 10 == 0:
            async for position in drone.telemetry.position():
                current_alt = position.relative_altitude_m
                print(f"      Guncel yukseklik: {current_alt:.2f}m")
                break
        
        await asyncio.sleep(0.2)
    
    print("\nKalkis tamamlandi, raf taramasina baslanıyor...")
    print("-" * 50)
    
    # ========================================
    # RAF KOORDINATLARI
    # ========================================
    # NED koordinat sistemi kullaniliyor:
    # - North (X): Ileri/Geri (pozitif = ileri)
    # - East (Y): Sag/Sol (pozitif = saga)
    # - Down (Z): Asagi/Yukari (negatif = yukari)
    # - Yaw: Donus acisi (derece cinsinden, saat yonu tersine pozitif)
    
    # Raf yapisi onunde gezinmek icin koordinatlar
    # Her nokta: (North, East, Down, Yaw) seklinde
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
    
    # Her noktaya sirasiyla git
    for i, (x, y, z, yaw) in enumerate(coordinates, start=1):
        # Eger yaw degistiyse bilgi ver
        if i > 1 and yaw != coordinates[i-2][3]:
            print(f"\n>>> Drone yonu degistiriliyor: {yaw} derece")
        
        print(f"Nokta {i}/{toplam_nokta}: N={x}m, E={y}m, H={abs(z):.2f}m, Yon={yaw}°")
        
        # Her noktada 6 saniye bekle (kamera gorutusu icin)
        # Pozisyonu ve yonu 0.2 saniyede bir gonder
        for _ in range(30):
            await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, yaw))
            await asyncio.sleep(0.2)
    
    print("\n" + "-" * 50)
    print("Tum raf noktalari tarandı")
    print("-" * 50)
    
    # ========================================
    # INIS
    # ========================================
    print("\nInis yapiliyor...")
    
    # Offboard modunu durdur
    await drone.offboard.stop()
    
    # Otomatik inis komutu
    await drone.action.land()
    
    # Havada miyiz diye kontrol et, yere inene kadar bekle
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Inis tamamlandi")
            break
    
    # Motorlari durdur
    await drone.action.disarm()
    
    print("\nGorev basariyla tamamlandi")
    print("-" * 50)

if __name__ == "__main__":
    # Programi calistir
    asyncio.run(run())
