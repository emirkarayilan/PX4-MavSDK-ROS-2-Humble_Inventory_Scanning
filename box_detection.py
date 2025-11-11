#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class KutuAlgilama(Node):
    
    def __init__(self):
        super().__init__('kutu_algilama')
        
        self.bridge = CvBridge()
        self.kutu_var = False
        self.son_algilama_sayaci = 0
        self.goruntu_sayisi = 0
        self.tanınan_kutular = []
        self.aktif_kutu_sayaci = 0
        self.onay_esigi = 15
        self.mevcut_konum = None
        
        self.min_kutu_boyutu = 8000
        self.max_kutu_boyutu = 100000
        self.min_merkez_x = 0.25
        self.max_merkez_x = 0.75
        
        kamera_topic = '/front_camera/image_raw'
        
        print("\nKutu Algilama Sistemi")
        print(f"Kamera topic: {kamera_topic}")
        
        self.get_logger().info('Mevcut image topicler kontrol ediliyor...')
        import time
        time.sleep(1)
        
        topic_names_and_types = self.get_topic_names_and_types()
        
        image_topics = [name for name, types in topic_names_and_types 
                       if 'sensor_msgs/msg/Image' in types]
        
        if image_topics:
            print("\nBulunan image topicler:")
            for i, topic in enumerate(image_topics, 1):
                print(f"  {i}. {topic}")
        
        self.subscription = self.create_subscription(
            Image,
            kamera_topic,
            self.goruntu_callback,
            10)
        
        self.timer = self.create_timer(3.0, self.topic_kontrol)
        
        self.get_logger().info(f'{kamera_topic} topicine abone olundu')
        print("Sadece 1 kutu algilanacak ve bir kere kaydedilecek")
        print("Durdurmak icin: Ctrl+C\n")
    
    def topic_kontrol(self):
        if self.goruntu_sayisi == 0:
            self.get_logger().warning('Henuz goruntu alinamadi')
            print("Kamera goruntusu gelmiyor. Gazebo ve drone kontrolu yapin.")
        else:
            self.get_logger().info(f'Kamera calisiyor ({self.goruntu_sayisi} goruntu)')
            self.timer.cancel()
    
    def mesafe_ve_aci_uygun_mu(self, x, y, w, h, alan, goruntu_genisligi):
        mesafe_uygun = alan >= self.min_kutu_boyutu and alan <= self.max_kutu_boyutu
        
        if alan < self.min_kutu_boyutu:
            mesafe_durumu = "Uzak"
        elif alan > self.max_kutu_boyutu:
            mesafe_durumu = "Cok yakin"
        else:
            mesafe_durumu = "Yakin"
        
        merkez_x = x + w // 2
        normalize_pozisyon = merkez_x / goruntu_genisligi
        aci_uygun = self.min_merkez_x <= normalize_pozisyon <= self.max_merkez_x
        
        if normalize_pozisyon < self.min_merkez_x:
            aci_durumu = "Sol"
        elif normalize_pozisyon > self.max_merkez_x:
            aci_durumu = "Sag"
        else:
            aci_durumu = "Uygun"
        
        uygun_mu = mesafe_uygun and aci_uygun
        
        return uygun_mu, mesafe_durumu, aci_durumu
    
    def kutu_zaten_tanindi_mi(self, x, y, w, h):
        for tanınan in self.tanınan_kutular:
            tx, ty, tw, th = tanınan
            
            merkez1_x = x + w // 2
            merkez1_y = y + h // 2
            merkez2_x = tx + tw // 2
            merkez2_y = ty + th // 2
            
            mesafe = np.sqrt((merkez1_x - merkez2_x)**2 + (merkez1_y - merkez2_y)**2)
            boyut_farki = abs(w - tw) + abs(h - th)
            
            if mesafe < 100 and boyut_farki < 50:
                return True
        
        return False
    
    def kutu_algilama(self, goruntu):
        hsv = cv2.cvtColor(goruntu, cv2.COLOR_BGR2HSV)
        
        alt_kahve1 = np.array([5, 30, 30])
        ust_kahve1 = np.array([30, 255, 255])
        alt_kahve2 = np.array([0, 30, 30])
        ust_kahve2 = np.array([10, 255, 255])
        alt_sari = np.array([20, 40, 80])
        ust_sari = np.array([35, 255, 255])
        
        maske1 = cv2.inRange(hsv, alt_kahve1, ust_kahve1)
        maske2 = cv2.inRange(hsv, alt_kahve2, ust_kahve2)
        maske3 = cv2.inRange(hsv, alt_sari, ust_sari)
        
        maske = cv2.bitwise_or(cv2.bitwise_or(maske1, maske2), maske3)
        
        kernel_kucuk = np.ones((3, 3), np.uint8)
        kernel_buyuk = np.ones((7, 7), np.uint8)
        
        maske = cv2.morphologyEx(maske, cv2.MORPH_CLOSE, kernel_buyuk)
        maske = cv2.morphologyEx(maske, cv2.MORPH_OPEN, kernel_kucuk)
        maske = cv2.GaussianBlur(maske, (5, 5), 0)
        
        konturlar, _ = cv2.findContours(maske, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        kutu_bulundu = False
        sonuc_goruntu = goruntu.copy()
        en_buyuk_alan = 0
        en_buyuk_kutu = None
        
        for kontur in konturlar:
            alan = cv2.contourArea(kontur)
            
            if alan > 500:
                x, y, w, h = cv2.boundingRect(kontur)
                en_boy_orani = float(w) / h
                
                if 0.3 < en_boy_orani < 4.0:
                    if alan > en_buyuk_alan:
                        en_buyuk_alan = alan
                        en_buyuk_kutu = (x, y, w, h, alan)
        
        if en_buyuk_kutu:
            x, y, w, h, alan = en_buyuk_kutu
            zaten_tanindi = self.kutu_zaten_tanindi_mi(x, y, w, h)
            
            if zaten_tanindi:
                cv2.rectangle(sonuc_goruntu, (x-5, y-5), (x+w+5, y+h+5), (128, 128, 128), 4)
                cv2.rectangle(sonuc_goruntu, (x, y), (x+w, y+h), (100, 100, 100), 2)
                
                etiket_w = 280
                etiket_h = 40
                etiket_x = max(x, 10)
                etiket_y = max(y - 50, 10)
                
                cv2.rectangle(sonuc_goruntu, 
                             (etiket_x, etiket_y), 
                             (etiket_x + etiket_w, etiket_y + etiket_h), 
                             (100, 100, 100), -1)
                
                cv2.rectangle(sonuc_goruntu, 
                             (etiket_x, etiket_y), 
                             (etiket_x + etiket_w, etiket_y + etiket_h), 
                             (200, 200, 200), 2)
                
                cv2.putText(sonuc_goruntu, 'Zaten kaydedildi', 
                           (etiket_x + 10, etiket_y + 28),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                
                kutu_bulundu = False
                
            else:
                cv2.rectangle(sonuc_goruntu, (x-5, y-5), (x+w+5, y+h+5), (0, 255, 0), 4)
                cv2.rectangle(sonuc_goruntu, (x, y), (x+w, y+h), (0, 200, 0), 2)
                
                etiket_w = 280
                etiket_h = 40
                etiket_x = max(x, 10)
                etiket_y = max(y - 50, 10)
                
                cv2.rectangle(sonuc_goruntu, 
                             (etiket_x, etiket_y), 
                             (etiket_x + etiket_w, etiket_y + etiket_h), 
                             (0, 200, 0), -1)
                
                cv2.rectangle(sonuc_goruntu, 
                             (etiket_x, etiket_y), 
                             (etiket_x + etiket_w, etiket_y + etiket_h), 
                             (255, 255, 255), 2)
                
                cv2.putText(sonuc_goruntu, 'Kutu algilandi', 
                           (etiket_x + 10, etiket_y + 28),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                
                cv2.putText(sonuc_goruntu, f'Alan: {int(alan)} | {w}x{h} px', 
                           (x, y+h+20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                merkez_x = x + w // 2
                merkez_y = y + h // 2
                cv2.circle(sonuc_goruntu, (merkez_x, merkez_y), 6, (0, 255, 0), -1)
                cv2.circle(sonuc_goruntu, (merkez_x, merkez_y), 10, (255, 255, 255), 2)
                
                kutu_bulundu = True
                self.mevcut_konum = (x, y, w, h)
        
        yukseklik, genislik = goruntu.shape[:2]
        
        info_text = f'Kayitli: {len(self.tanınan_kutular)} kutu'
        cv2.putText(sonuc_goruntu, info_text, 
                   (10, yukseklik - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(sonuc_goruntu, info_text, 
                   (10, yukseklik - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
 
        if kutu_bulundu:
            cv2.rectangle(sonuc_goruntu, (0, 0), (genislik, 50), (0, 180, 0), -1)
            cv2.putText(sonuc_goruntu, 'Yeni kutu bulundu', 
                       (20, 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
        else:
            cv2.rectangle(sonuc_goruntu, (0, 0), (genislik, 50), (0, 0, 180), -1)
            cv2.putText(sonuc_goruntu, 'Araniyor', 
                       (20, 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
            
            nokta_sayisi = (self.son_algilama_sayaci // 10) % 4
            cv2.putText(sonuc_goruntu, '.' * nokta_sayisi, 
                       (genislik - 80, 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        
        return sonuc_goruntu, kutu_bulundu
    
    def goruntu_callback(self, msg):
        try:
            self.goruntu_sayisi += 1
            
            if self.goruntu_sayisi == 1:
                self.get_logger().info('Ilk goruntu alindi')
                print(f"\nKamera baglantisi basarili")
                print(f"Goruntu boyutu: {msg.width}x{msg.height}")
                print(f"Encoding: {msg.encoding}\n")
            
            try:
                cv_goruntu = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e1:
                self.get_logger().warning(f'bgr8 basarisiz, rgb8 deneniyor: {e1}')
                try:
                    cv_goruntu = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                    cv_goruntu = cv2.cvtColor(cv_goruntu, cv2.COLOR_RGB2BGR)
                except Exception as e2:
                    self.get_logger().error(f'rgb8 de basarisiz: {e2}')
                    cv_goruntu = self.bridge.imgmsg_to_cv2(msg)
            
            if cv_goruntu is None or cv_goruntu.size == 0:
                self.get_logger().error('Goruntu bos')
                return
            
            islenmiş_goruntu, kutu_var = self.kutu_algilama(cv_goruntu)
            self.son_algilama_sayaci += 1
            
            if kutu_var and self.mevcut_konum:
                self.aktif_kutu_sayaci += 1
                
                if self.aktif_kutu_sayaci >= self.onay_esigi:
                    x, y, w, h = self.mevcut_konum
                    
                    self.tanınan_kutular.append((x, y, w, h))
                    self.get_logger().info(f'Yeni kutu kaydedildi! Toplam: {len(self.tanınan_kutular)}')
                    print(f">>> Kutu #{len(self.tanınan_kutular)} envantere eklendi")
                    
                    self.aktif_kutu_sayaci = 0
                    self.mevcut_konum = None
            else:
                self.aktif_kutu_sayaci = 0
                self.mevcut_konum = None
            
            if kutu_var and not self.kutu_var:
                self.get_logger().info('Yeni kutu algilandi')
            elif not kutu_var and self.kutu_var:
                self.get_logger().info('Kutu gorus alanından cikti')
            
            self.kutu_var = kutu_var
            
            pencere_adi = 'Drone Kamera - Kutu Algilama'
            cv2.namedWindow(pencere_adi, cv2.WINDOW_NORMAL)
            cv2.imshow(pencere_adi, islenmiş_goruntu)
            
            key = cv2.waitKey(10)
            if key == 27:
                self.get_logger().info('ESC tusuna basildi, kapatiliyor')
                raise KeyboardInterrupt
            
        except KeyboardInterrupt:
            raise
        except Exception as e:
            self.get_logger().error(f'Hata: {e}')
            import traceback
            traceback.print_exc()

def main(args=None):
    import warnings
    warnings.filterwarnings('ignore')
    
    rclpy.init(args=args)
    algilayici = KutuAlgilama()
    
    try:
        rclpy.spin(algilayici)
    except KeyboardInterrupt:
        print("\nProgram durduruldu")
    finally:
        cv2.destroyAllWindows()
        algilayici.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()