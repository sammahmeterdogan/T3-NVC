<img width="1885" height="1199" alt="image" src="https://github.com/user-attachments/assets/f17b500b-6e03-422a-82b6-7e4c58791600" /><img width="1909" height="1187" alt="image" src="https://github.com/user-attachments/assets/f304e7fe-7e03-4e5e-82ff-e457887d23a2" />
<img width="1838" height="710" alt="image" src="https://github.com/user-attachments/assets/25c5c558-917c-4cf5-add3-a16fe8f1c14a" />


# 🤖 TurtleBot3 Web Simülatörü

**Terminal kullanmadan, tamamen web üzerinden TurtleBot3 robot simülasyonu ve kontrolü platformu**

## 📖 İçindekiler

- [Proje Hakkında](#-proje-hakkında)
- [Özellikler](#-özellikler)
- [Sistem Mimarisi](#-sistem-mimarisi)
- [Kurulum](#-kurulum)
- [Kullanım](#-kullanım)
- [API Dokümantasyonu](#-api-dokümantasyonu)
- [Proje Yapısı](#-proje-yapısı)
- [Konfigürasyon](#-konfigürasyon)
- [Sorun Giderme](#-sorun-giderme)
- [Katkıda Bulunma](#-katkıda-bulunma)
- [Lisans](#-lisans)

## 🎯 Proje Hakkında

TurtleBot3 Web Simülatörü, robotik geliştiricilerin ve araştırmacıların terminal komutları ile uğraşmadan, tamamen web tarayıcısı üzerinden TurtleBot3 robotlarını simüle edebilmelerini ve kontrol edebilmelerini sağlayan profesyonel bir platformdur.

### Temel Amaç

- **Kullanım Kolaylığı**: Karmaşık ROS komutlarını bilmeden robot simülasyonu
- **Erişilebilirlik**: Herhangi bir cihazdan web tarayıcısı ile erişim
- **Eğitim Odaklı**: Robotik öğrenenler için ideal başlangıç platformu
- **Gerçek Zamanlı**: Canlı 3D görselleştirme ve kontrol
- **Kapsamlı**: TurtleBot3'ün tüm örneklerini içerir

## ✨ Özellikler

### 🎮 Simülasyon Kontrolü
- **Model Seçimi**: Burger, Waffle ve Waffle Pi modelleri arasında seçim
- **Tek Tıkla Başlatma**: Start/Stop butonları ile simülasyon kontrolü
- **Senaryo Yönetimi**: Hazır senaryolar (SLAM, Navigation, Teleoperation vb.)
- **Gerçek Zamanlı Durum**: Simülasyon durumu ve sistem sağlığı takibi

### 🗺️ Haritalama ve Navigasyon
- **SLAM**: Gerçek zamanlı haritalama (slam_toolbox)
- **Harita Kaydetme**: Oluşturulan haritaları PGM/YAML formatında kaydetme
- **Harita Yönetimi**: Kayıtlı haritaları listeleme, yükleme, silme
- **Otonom Navigasyon**: Nav2 ile hedef nokta belirleme ve navigasyon
- **Waypoint Navigasyonu**: Çoklu hedef noktaları ile rota planlama

### 🕹️ Robot Kontrolü
- **Teleoperation**: Klavye ve sanal joystick ile manuel kontrol
- **Hız Kontrolü**: Linear ve angular hız ayarları
- **Hassas Kontrol**: 8 yönlü hareket desteği
- **Acil Durdurma**: Anında durdurma butonu
- **Pozisyon Kontrolü**: Hassas pozisyon komutları

### 📊 Görselleştirme
- **3D Simülasyon**: Three.js tabanlı RViz benzeri görselleştirme
- **Sensör Verileri**: LaserScan, Odometry, IMU verileri
- **Kamera Akışı**: MJPEG formatında canlı kamera görüntüsü
- **Harita Görüntüleme**: OccupancyGrid harita gösterimi
- **Robot Modeli**: URDF tabanlı robot modeli görselleştirme
- **Path Görselleştirme**: Robot yolu ve planlanan rota gösterimi

### 📡 Telemetri ve Monitoring
- **Poz Takibi**: X, Y, Theta pozisyon bilgisi
- **Hız Göstergesi**: Linear ve angular hız değerleri
- **Batarya Durumu**: Batarya seviyesi takibi
- **CPU/Memory Kullanımı**: Sistem kaynak monitörü
- **Bağlantı Durumu**: ROS Bridge ve WebSocket durumu

### 💾 Veri Yönetimi
- **PostgreSQL Entegrasyonu**: Tüm veriler veritabanında saklanır
- **Session Yönetimi**: Simülasyon oturumları kaydı
- **Konfigürasyon Kayıtları**: Robot ayarları veritabanında
- **Flyway Migration**: Otomatik veritabanı şema yönetimi

## 🏗️ Sistem Mimarisi

### Backend (Spring Boot 3.2.0)

#### Teknoloji Stack
- **Java 17** - Temel programlama dili
- **Spring Boot 3.2.0** - Microservice framework
- **Spring Data JPA** - ORM ve veritabanı işlemleri
- **Spring WebSocket** - STOMP protokolü ile gerçek zamanlı iletişim
- **Spring Security** - Güvenlik katmanı (permit all konfigürasyonu)
- **PostgreSQL 15** - Ana veritabanı
- **Flyway 9.22.3** - Veritabanı migration yönetimi
- **MapStruct 1.5.5** - DTO-Entity mapping
- **Lombok** - Boilerplate kod azaltma
- **Docker Java Client 3.3.4** - Docker container yönetimi
- **Java-WebSocket 1.5.4** - ROS Bridge bağlantısı
- **SpringDoc OpenAPI 2.3.0** - Swagger UI ve API dokümantasyonu

#### Çok Katmanlı Mimari

```
backend/
├── rcp-base/                    # Temel modül
│   └── src/main/java/com/samma/rcp/base/
│       ├── controller/          # BaseController abstract sınıfı
│       ├── dto/                 # RequestDTO, ResponseDTO base sınıfları
│       ├── mapper/              # BaseMapper interface
│       └── service/             # BaseService interface ve implementasyonu
│
└── rcp-app/                     # Ana uygulama modülü
    └── src/main/java/com/samma/rcp/app/
        ├── config/              # Konfigürasyon sınıfları
        │   ├── WebConfig        # CORS ve Web ayarları
        │   ├── SecurityConfig   # Spring Security (permit all)
        │   ├── OpenApiConfig    # Swagger/OpenAPI
        │   └── RosDockerProps   # ROS ve Docker properties
        │
        ├── controller/          # REST API Controller'ları
        │   ├── SimulationController    # Simülasyon yönetimi
        │   ├── MapController           # Harita işlemleri
        │   ├── ExampleController       # Hazır senaryolar
        │   └── TeleopController        # Robot kontrolü
        │
        ├── domain/              # Domain katmanı
        │   ├── entity/          # JPA Entity'leri
        │   │   ├── RobotConfig         # Robot konfigürasyonu
        │   │   ├── SimulationSession   # Simülasyon oturumu
        │   │   ├── SavedMap            # Kaydedilmiş haritalar
        │   │   └── ExampleScenario     # Örnek senaryolar
        │   │
        │   ├── model/           # Enum ve model sınıfları
        │   │   ├── RobotModel          # BURGER, WAFFLE, WAFFLE_PI
        │   │   ├── ScenarioType        # TELEOP, SLAM, NAVIGATION vb.
        │   │   └── SimulationStatus    # RUNNING, STOPPED, ERROR vb.
        │   │
        │   └── repo/            # Spring Data JPA Repository'leri
        │       ├── RobotConfigRepository
        │       ├── SimulationSessionRepository
        │       └── SavedMapRepository
        │
        ├── dto/                 # Data Transfer Object'leri
        │   ├── RobotConfigDTO
        │   ├── SimulationStartRequest
        │   ├── SimulationStatusDTO
        │   ├── SavedMapDTO
        │   ├── GoalPoseDTO              # Navigasyon hedefi
        │   └── TwistDTO                 # Hız komutları
        │
        ├── mapper/              # MapStruct Mapper'ları
        │   ├── MapStructConfig
        │   ├── RobotConfigMapper
        │   └── SavedMapMapper
        │
        ├── orchestration/       # Docker ve ROS yönetimi
        │   ├── DockerService            # Docker compose up/down
        │   ├── RosBridgeClient          # ROS WebSocket bağlantısı
        │   ├── SimulationOrchestrator   # Simülasyon lifecycle
        │   └── RosCommandGateway        # ROS komut gönderimi
        │
        ├── service/             # İş mantığı servisleri
        │   ├── SimulationService        # Simülasyon işlemleri
        │   ├── MapService               # Harita kaydetme/yükleme
        │   ├── RobotConfigService       # Robot konfigürasyonu
        │   └── TeleopService            # Teleoperation servisi
        │
        └── ws/                  # WebSocket handler
            └── RobotSocketHandler       # STOMP WebSocket

```

### Frontend (React 18.3.1 + Vite)

#### Teknoloji Stack
- **React 18.3.1** - UI framework
- **Vite 5.4.10** - Build tool ve dev server
- **Three.js 0.169.0** - 3D görselleştirme
- **@react-three/fiber 8.17.10** - React Three.js entegrasyonu
- **@react-three/drei 9.114.3** - Three.js yardımcı bileşenleri
- **ROSLIB 1.3.0** - ROS Bridge JavaScript client
- **@stomp/stompjs 7.0.0** - WebSocket STOMP client
- **React Router 6.26.0** - Sayfa yönlendirme
- **@tanstack/react-query 5.59.0** - Sunucu state yönetimi
- **Zustand 4.5.5** - Client state yönetimi
- **Tailwind CSS 3.4.14** - Utility-first CSS framework
- **Framer Motion 11.11.11** - Animasyon kütüphanesi
- **Lucide React 0.453.0** - İkon kütüphanesi
- **React Hook Form 7.53.0** - Form yönetimi
- **React Hot Toast 2.4.1** - Bildirim sistemi
- **Recharts 2.12.7** - Grafik kütüphanesi

#### Dosya Yapısı

```
frontend/
├── src/
│   ├── services/                # API ve WebSocket servisleri
│   │   ├── api.js               # REST API çağrıları (axios)
│   │   ├── rosClient.js         # ROS Bridge bağlantısı (ROSLIB)
│   │   └── ws.js                # STOMP WebSocket yönetimi
│   │
│   ├── components/
│   │   ├── layout/              # Sayfa düzeni bileşenleri
│   │   │   ├── Layout.jsx       # Ana layout wrapper
│   │   │   ├── Sidebar.jsx      # Sol menü (navigasyon, bağlantı durumu)
│   │   │   ├── Topbar.jsx       # Üst bar (arama, bildirimler)
│   │   │   └── PageContainer.jsx # Sayfa container wrapper
│   │   │
│   │   ├── simulation/          # Simülasyon bileşenleri
│   │   │   ├── RvizPanel.jsx    # 3D görselleştirme (Three.js)
│   │   │   ├── TeleopPad.jsx    # Robot kontrol pad'i
│   │   │   ├── ModelSelector.jsx # Robot model seçici
│   │   │   ├── ScenarioSelector.jsx # Senaryo seçici
│   │   │   └── StatusPanel.jsx  # Durum gösterge paneli
│   │   │
│   │   └── ui/                  # Genel UI bileşenleri
│   │       └── LoadingSpinner.jsx
│   │
│   ├── pages/                   # Sayfa bileşenleri
│   │   ├── Dashboard.jsx        # Ana sayfa (istatistikler, hızlı erişim)
│   │   ├── Simulator.jsx        # Simülasyon kontrol sayfası
│   │   ├── Examples.jsx         # Hazır senaryolar
│   │   ├── Maps.jsx             # Harita yönetimi
│   │   ├── Settings.jsx         # Ayarlar sayfası
│   │   └── NotFound.jsx         # 404 sayfası
│   │
│   ├── styles/
│   │   └── index.css            # Tailwind CSS ve özel stiller
│   │
│   ├── App.jsx                  # Ana uygulama bileşeni ve routing
│   └── main.jsx                 # Uygulama entry point
│
├── public/                      # Statik dosyalar
├── index.html                   # HTML template
├── vite.config.js              # Vite konfigürasyonu
├── tailwind.config.js          # Tailwind CSS ayarları
├── postcss.config.js           # PostCSS ayarları
└── package.json                # Bağımlılıklar ve scriptler
```

## 🚀 Kurulum

### Adım 1: Sistem Gereksinimlerinin Kontrol Edilmesi ve Kurulumu

#### 1.1 Node.js 18+ Kurulumu (Ubuntu/Linux)
```bash
# Mevcut Node.js versiyonunu kontrol edin
node --version  # 18+ olması gerekli

# Eğer Node.js yüklü değilse veya eski versiyon varsa:
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs

# Alternatif: nvm ile kurulum (önerilen)
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
source ~/.bashrc
nvm install 18
nvm use 18
nvm alias default 18
```

#### 1.2 Java 17+ Kurulumu
```bash
# Mevcut Java versiyonunu kontrol edin
java --version  # 17+ olması gerekli

# Ubuntu'da Java 17 kurulumu
sudo apt update
sudo apt install openjdk-17-jdk

# JAVA_HOME ortam değişkenini ayarlayın
echo 'export JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64' >> ~/.bashrc
source ~/.bashrc

# Java kurulumunu doğrulayın
java --version
javac --version
```

#### 1.3 Docker ve Docker Compose Kurulumu
```bash
# Docker kurulumu
sudo apt update
sudo apt install docker.io docker-compose-plugin

# Docker servisini başlatın
sudo systemctl start docker
sudo systemctl enable docker

# Kullanıcıyı docker grubuna ekleyin (root olmadan çalıştırmak için)
sudo usermod -aG docker $USER

# Yeniden giriş yapın veya aşağıdaki komutu çalıştırın
newgrp docker

# Docker kurulumunu doğrulayın
docker --version
docker compose version
```

#### 1.4 Git Kurulumu (eğer yoksa)
```bash
# Git kurulumu
sudo apt install git

# Git konfigürasyonu
git config --global user.name "İsminiz"
git config --global user.email "email@example.com"
```

### Adım 2: Projeyi Clone Etme

```bash
# Projeyi bilgisayarınıza indirin
git clone <repository-url>
cd turtlebot3-web-simulator

# Proje yapısını kontrol edin
ls -la
# Şu klasörler görünmelidir: frontend/, backend/, ros-stack/
```

### Adım 3: PostgreSQL Kurulumu (Opsiyonel - Production için)

#### 3.1 PostgreSQL Kurulumu
```bash
# PostgreSQL kurulumu
sudo apt update
sudo apt install postgresql postgresql-contrib

# PostgreSQL servisini başlatın
sudo systemctl start postgresql
sudo systemctl enable postgresql
```

#### 3.2 Veritabanı Oluşturma
```bash
# PostgreSQL kullanıcısına geçin
sudo -u postgres psql

# Veritabanı ve kullanıcı oluşturun
CREATE DATABASE turtlebot3_db;
CREATE USER turtlebot3_user WITH PASSWORD 'password123';
GRANT ALL PRIVILEGES ON DATABASE turtlebot3_db TO turtlebot3_user;
\q
```

### Adım 4: Backend Kurulumu ve Çalıştırma

#### 4.1 Development Mode (Önerilen - H2 Database)
```bash
# Backend dizinine gidin
cd backend

# Gradle wrapper'ın çalıştırılabilir olduğundan emin olun
chmod +x gradlew

# Projeyi build edin (ilk kez çalıştırma uzun sürebilir)
./gradlew build

# Development modunda başlatın (H2 in-memory database kullanır)
./gradlew bootRun --args='--spring.profiles.active=dev'
```

#### 4.2 Production Mode (PostgreSQL ile)
```bash
# PostgreSQL ile production modunda başlatma
./gradlew bootRun --args='--spring.profiles.active=prod'
```

#### 4.3 Backend Çalışma Kontrolü
```bash
# Başka bir terminalde API'yi test edin
curl http://localhost:8080/api/sim/status

# Swagger UI'ya erişim
# Tarayıcıda: http://localhost:8080/swagger-ui.html
```

### Adım 5: Frontend Kurulumu ve Çalıştırma

```bash
# Yeni bir terminal açın
cd turtlebot3-web-simulator/frontend

# Node modüllerini yükleyin
npm install

# Development sunucusunu başlatın
npm run dev

# Alternatif olarak:
npm start
```

#### 5.1 Frontend Çalışma Kontrolü
```bash
# Tarayıcıda aşağıdaki URL'yi açın:
# http://localhost:5173

# Terminal çıktısında şu mesajları görmelisiniz:
# "Local:   http://localhost:5173/"
# "Network: http://192.168.x.x:5173/"
```

### Adım 6: ROS Stack (Docker ile TurtleBot3 Simülasyonu)

> ℹ️ **Not — Docker ile Tek Komutta Çalıştırma**
> 
> Bu Compose altyapısı **TurtleBot3 simülasyonu (Gazebo)**, **SLAM (slam_toolbox)**, **Nav2 (navigasyon)** ve **rosbridge (WebSocket köprüsü)** servislerini tek seferde ayağa kaldırır. Gerçek robota ihtiyaç duymadan web tabanlı geliştirme ve test yapabilirsiniz.

#### 6.1 ROS Stack Başlatma
```bash
# Ana proje dizinine gidin
cd turtlebot3-web-simulator

# ROS stack'i başlatın (tüm ROS servisleri)
docker compose -f ros-stack/docker-compose.yml up -d --build

# Servislerin durumunu kontrol edin
docker compose -f ros-stack/docker-compose.yml ps

# Logları takip edin
docker compose -f ros-stack/docker-compose.yml logs -f
```

#### 6.2 ROS Services Kontrolü
```bash
# ROS Bridge bağlantısını test edin
# Tarayıcıda: ws://localhost:9090

# Gazebo simülasyon penceresinin açıldığını kontrol edin
# Docker logs ile Gazebo'nun başladığını doğrulayın:
docker compose -f ros-stack/docker-compose.yml logs gazebo
```

### Adım 7: Sistem Entegrasyonu Testi

#### 7.1 Servis Portları Kontrolü
```bash
# Tüm servislerin çalıştığını kontrol edin
netstat -tulpn | grep -E ":(3000|8080|5173|9090|5432)"

# Veya:
ss -tulpn | grep -E ":(3000|8080|5173|9090|5432)"
```

| Servis | Port | URL | Durum |
|--------|------|-----|--------|
| Frontend (Dev) | 5173 | http://localhost:5173 | ✅ |
| Backend API | 8080 | http://localhost:8080/api | ✅ |
| ROS Bridge | 9090 | ws://localhost:9090 | ✅ |
| PostgreSQL | 5432 | localhost:5432 | ✅ (opsiyonel) |
| Swagger UI | 8080 | http://localhost:8080/swagger-ui.html | ✅ |

#### 7.2 Entegre Test
```bash
# 1. Frontend erişim testi
curl -s http://localhost:5173 > /dev/null && echo "Frontend: OK" || echo "Frontend: FAIL"

# 2. Backend API testi
curl -s http://localhost:8080/api/sim/status && echo "Backend: OK" || echo "Backend: FAIL"

# 3. ROS Bridge testi (basit WebSocket kontrolü)
nc -zv localhost 9090 && echo "ROS Bridge: OK" || echo "ROS Bridge: FAIL"
```

### Adım 8: İlk Simülasyon Testi

#### 8.1 Web Arayüzü Üzerinden Test
```bash
# 1. Tarayıcıda http://localhost:5173 adresini açın
# 2. Dashboard sayfasında sistem durumlarını kontrol edin
# 3. "Simulator" menüsüne gidin
# 4. Robot modelini seçin (Burger önerilir)
# 5. "Start Simulation" butonuna tıklayın
# 6. 3D görselleştirmede robotun göründüğünü doğrulayın
# 7. Teleoperation pad ile robotun hareket ettiğini test edin
```

#### 8.2 Terminal Üzerinden Durum Kontrolü
```bash
# Backend loglarını izleyin
cd backend && ./gradlew bootRun --info

# ROS Bridge loglarını izleyin
docker compose -f ros-stack/docker-compose.yml logs -f rosbridge

# Gazebo simülasyon loglarını izleyin
docker compose -f ros-stack/docker-compose.yml logs -f gazebo
```

## 🚀 Alternatif Kurulum Yöntemleri

### Yöntem 1: Full Docker (Tüm Stack)
```bash
# Tüm sistemi Docker ile çalıştırma
docker compose up --build -d

# Erişim URL'leri:
# Frontend: http://localhost:3000
# Backend: http://localhost:8080
# ROS Bridge: ws://localhost:9090
```

### Yöntem 2: Development Mode (Önerilen)
```bash
# Terminal 1: Backend (H2 database ile)
cd backend && ./gradlew bootRun --args='--spring.profiles.active=dev'

# Terminal 2: Frontend (hot reload ile)
cd frontend && npm run dev

# Terminal 3: ROS Stack (sadece ROS servisleri)
docker compose -f ros-stack/docker-compose.yml up -d
```

### Yöntem 3: Production Mode
```bash
# PostgreSQL kurulumu gerekli (Adım 3)
# Frontend build
cd frontend && npm run build

# Backend production mode
cd backend && ./gradlew bootRun --args='--spring.profiles.active=prod'

# ROS Stack
docker compose -f ros-stack/docker-compose.yml up -d
```

## 🔧 Kurulum Sonrası Yapılacaklar

### İlk Konfigürasyon
1. **Robot Model Ayarı**: Dashboard'da varsayılan robot modelini seçin
2. **Harita Klasörü**: Haritaların kaydedileceği klasörü belirleyin
3. **ROS Bridge URL**: Varsayılan ws://localhost:9090 ayarını doğrulayın

### Geliştirme Ortamı Hazırlığı
```bash
# IDE'ler için önerilen eklentiler:
# - VS Code: Spring Boot Extension Pack, ES7+ React snippets
# - IntelliJ IDEA: Spring Boot, JavaScript/TypeScript
```

## ✨ Yeni Özellikler

### Turtlesim Web Kontrolü (v1.1.0)
- **Web Tabanlı Turtle Kontrolü**: Tarayıcı üzerinden turtlesim kontrolü
- **Gerçek Zamanlı VNC Görüntüsü**: noVNC ile canlı turtle görüntüleme
- **Joystick Kontrolü**: Sanal joystick ile turtle hareketi
- **ROS Bridge Entegrasyonu**: Tam WebSocket desteği

#### Turtlesim Kullanımı
```bash
# 1. Turtlesim servisini başlatın
docker-compose up -d turtlesim-novnc

# 2. Frontend'te Turtlesim sayfasına gidin
# http://localhost:5173/turtlesim

# 3. VNC görüntüsünü görüntüleyin
# http://localhost:6081/vnc.html?autoconnect=1&resize=scale

# 4. Joystick veya ok tuşları ile turtle'ı kontrol edin
```

## 🔧 Sorun Giderme

### Sık Karşılaşılan Sorunlar

#### 0. ROS Bridge Bağlantı Hatası (500 Internal Server Error)
```bash
# Hata: "WebsocketNotConnectedException" veya cmd_vel 500 hatası
# Çözüm: docker-compose.yml'de rosbridge network alias'ını kontrol edin

# Rosbridge servisinde network yapılandırması şu şekilde olmalı:
networks:
  turtlebot-network:
    aliases:
      - rosbridge

# Servisleri yeniden başlatın
docker-compose down rosbridge backend
docker-compose up -d rosbridge backend
```

#### 1. Node.js Versiyon Hatası
```bash
# Hata: "Unexpected token '.'" veya ES6 syntax hataları
# Çözüm: Node.js versiyonunu 18+ güncelleyin
node --version  # 18.x.x veya üstü olmalı
```

#### 2. Java Versiyon Hatası
```bash
# Hata: "Unsupported class file major version" 
# Çözüm: Java 17+ yükleyin ve JAVA_HOME'u ayarlayın
java --version  # 17.x.x veya üstü olmalı
echo $JAVA_HOME  # Boş olmamalı
```

#### 3. Frontend Bağlantı Sorunu
```bash
# Hata: "WebSocket connection failed" veya API çağrıları başarısız
# Çözüm: Backend'in çalıştığından emin olun
curl -v http://localhost:8080/health
# Beklenilen: 200 OK yanıtı
```

#### 4. ROS Bridge Bağlantı Sorunu
```bash
# Hata: "ROS Bridge disconnected" veya WebSocket hatası
# Çözüm: ROS stack konteynerlerini kontrol edin
docker compose -f ros-stack/docker-compose.yml ps
# Beklenilen: Tüm servisler "running" durumunda
```

#### 5. Database Bağlantı Sorunu
```bash
# Hata: "Connection to database failed"
# Çözüm 1: Development mode kullanın (H2 database)
./gradlew bootRun --args='--spring.profiles.active=dev'

# Çözüm 2: PostgreSQL servisini kontrol edin
sudo systemctl status postgresql
sudo -u postgres psql -c "\l"  # Veritabanı listesi
```

#### 6. Port Kullanım Çakışması
```bash
# Hata: "Port already in use" veya "EADDRINUSE"
# Çözüm: Kullanılan portları kontrol edin
sudo netstat -tulpn | grep -E ":(3000|8080|5173|9090|5432)"

# Çakışan süreci sonlandırın
sudo lsof -ti:8080 | xargs kill -9  # 8080 portu için örnek
```

### Log Kontrolü ve Debugging

#### Backend Logs
```bash
# Development modunda detaylı loglar
./gradlew bootRun --args='--spring.profiles.active=dev --logging.level.com.samma.rcp=DEBUG'

# Production modunda loglar
./gradlew bootRun --info

# Docker ile backend çalışıyorsa
docker logs turtlebot3-backend -f
```

#### Frontend Logs
```bash
# Terminal logs
cd frontend && npm run dev

# Browser console (F12)
# Network tab'ında API çağrılarını kontrol edin
# Console tab'ında JavaScript hatalarını kontrol edin
```

#### ROS Stack Logs
```bash
# Tüm ROS servislerin logları
docker compose -f ros-stack/docker-compose.yml logs -f

# Belirli bir servisin logları
docker compose -f ros-stack/docker-compose.yml logs -f rosbridge
docker compose -f ros-stack/docker-compose.yml logs -f gazebo
docker compose -f ros-stack/docker-compose.yml logs -f navigation
```

### Performans Optimizasyonu

#### Memory ve CPU Kullanımı
```bash
# Docker konteyner kaynak kullanımı
docker stats

# Java heap boyutu ayarı (eğer memory sorunu varsa)
./gradlew bootRun --args='--spring.profiles.active=dev -Xmx2g -Xms1g'
```

#### Gazebo Performansı
```bash
# GPU acceleration için (NVIDIA kartı varsa)
docker compose -f ros-stack/docker-compose.yml --profile gpu up
```

### Sistem Sağlık Kontrolü

#### Otomatik Health Check Script
```bash
#!/bin/bash
# health_check.sh dosyası oluşturun

echo "=== TurtleBot3 Web Simulator Health Check ==="

# Frontend check
if curl -s http://localhost:5173 > /dev/null; then
    echo "✅ Frontend: OK"
else
    echo "❌ Frontend: FAIL"
fi

# Backend check
if curl -s http://localhost:8080/api/sim/status > /dev/null; then
    echo "✅ Backend: OK"
else
    echo "❌ Backend: FAIL"
fi

# ROS Bridge check
if nc -zv localhost 9090 2>/dev/null; then
    echo "✅ ROS Bridge: OK"
else
    echo "❌ ROS Bridge: FAIL"
fi

# Docker containers check
echo "=== Docker Container Status ==="
docker compose -f ros-stack/docker-compose.yml ps
```

#### Scriptı Çalıştırma
```bash
chmod +x health_check.sh
./health_check.sh
```

### Recovery Procedures

#### Tam Sistem Restart
```bash
# 1. Tüm servisleri durdur
pkill -f "java.*spring-boot"  # Backend
pkill -f "vite"              # Frontend
docker compose -f ros-stack/docker-compose.yml down

# 2. Docker volumes temizle (dikkat: veri kaybı olabilir)
docker system prune -f
docker volume prune -f

# 3. Yeniden başlat
./gradlew bootRun --args='--spring.profiles.active=dev' &
npm run dev &
docker compose -f ros-stack/docker-compose.yml up -d
```

#### Veritabanı Reset (H2 Development)
```bash
# H2 database dosyalarını sil
rm -rf backend/data/
./gradlew bootRun --args='--spring.profiles.active=dev'
```

## 📚 Geliştirici Kılavuzu

### Hot Reload ve Development
```bash
# Frontend: Otomatik hot reload aktif
cd frontend && npm run dev

# Backend: Code değişikliklerinde restart gerekli
# IntelliJ IDEA kullanıyorsanız: Spring Boot DevTools aktif
# VS Code kullanıyorsanız: Manuel restart gerekli
```

### Testing
```bash
# Backend testleri
cd backend && ./gradlew test

# Frontend testleri  
cd frontend && npm test

# Integration testleri
cd backend && ./gradlew integrationTest
```

### Build ve Deployment
```bash
# Frontend production build
cd frontend && npm run build

# Backend production jar
cd backend && ./gradlew bootJar

# Docker images
docker compose -f docker-compose.prod.yml build
```

Bu adımları takip ederek TurtleBot3 Web Simülatörü sistemi 10-15 dakikada tam olarak kurulup çalışır hale gelecektir. Herhangi bir sorun yaşarsanız, yukarıdaki sorun giderme bölümünü kontrol edin.

---

## API Dokümantasyonu

### API Endpoint'leri

#### Simülasyon Yönetimi
```
POST   /api/sim/start              # Simülasyon başlatma
POST   /api/sim/stop               # Simülasyon durdurma
GET    /api/sim/status             # Durum sorgulama
```

#### Harita İşlemleri
```
POST   /api/map/save               # Harita kaydetme
GET    /api/map/list               # Harita listesi
POST   /api/map/load/{id}          # Harita yükleme
DELETE /api/map/{id}               # Harita silme
```

#### Robot Kontrolü
```
POST   /api/teleop/twist           # Hız komutu gönderme
POST   /api/nav/goal               # Navigasyon hedefi
POST   /api/nav/waypoints          # Çoklu hedef noktası
GET    /api/nav/status             # Navigasyon durumu
```

#### SLAM İşlemleri
```
POST   /api/slam/start             # SLAM başlatma
POST   /api/slam/stop              # SLAM durdurma
GET    /api/slam/status            # SLAM durumu
```

#### Örnek Senaryolar
```
GET    /api/examples               # Tüm senaryolar
POST   /api/examples/{id}/launch   # Senaryo başlatma
```

#### Konfigürasyon
```
GET    /api/config/models          # Robot modelleri
GET    /api/config                 # Konfigürasyon listesi
POST   /api/config                 # Yeni konfigürasyon
PUT    /api/config/{id}            # Konfigürasyon güncelleme
```

### WebSocket Topics

```
/topic/status                      # Simülasyon durumu
/topic/pose                        # Robot pozisyonu
/topic/telemetry                   # Telemetri verileri
/topic/battery                     # Batarya durumu
/topic/map-meta                    # Harita metadata
```

### Veritabanı Şeması (PostgreSQL)

```sql
-- Robot konfigürasyonları
robot_config (
    id BIGSERIAL PRIMARY KEY,
    model VARCHAR(50),
    created_at TIMESTAMP,
    updated_at TIMESTAMP
)

-- Simülasyon oturumları
simulation_session (
    id BIGSERIAL PRIMARY KEY,
    model VARCHAR(50),
    scenario VARCHAR(100),
    status VARCHAR(50),
    started_at TIMESTAMP,
    ended_at TIMESTAMP,
    ros_bridge_url VARCHAR(255),
    video_url VARCHAR(255)
)

-- Kaydedilmiş haritalar
saved_map (
    id BIGSERIAL PRIMARY KEY,
    name VARCHAR(255),
    file_path VARCHAR(500),
    size_mb DOUBLE,
    resolution DOUBLE,
    width INTEGER,
    height INTEGER,
    created_at TIMESTAMP
)

-- Örnek senaryolar
example_scenario (
    id BIGSERIAL PRIMARY KEY,
    key VARCHAR(100) UNIQUE,
    title VARCHAR(255),
    description TEXT,
    launch_args TEXT,
    category VARCHAR(100),
    difficulty VARCHAR(50),
    enabled BOOLEAN
)
```

## 🚀 Kullanım

### İlk Başlatma
1. **Frontend**: http://localhost:5173
2. **Dashboard** sayfasında sistem durumunu kontrol edin
3. **Simulator** menüsüne gidin
4. Robot modelini seçin (Burger önerilir)
5. "Start Simulation" butonuna tıklayın

### Robot Kontrolü
- **Klavye**: W,A,S,D tuşları ile hareket
- **Joystick**: Sanal joystick ile kontrol
- **Navigasyon**: Harita üzerinde hedef belirleme

### Harita Oluşturma
1. SLAM modunu başlatın
2. Robotu manuel olarak hareket ettirin
3. Harita oluşturulmasını bekleyin
4. "Save Map" ile haritayı kaydedin

---

