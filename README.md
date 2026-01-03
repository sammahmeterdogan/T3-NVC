# TurtleBot3 Web Simülatörü (Production-Ready)

Tarayıcı üzerinden TurtleBot3 simülasyonu, teleop, SLAM, navigasyon ve Turtlesim demo. Stack: React (Vite), Spring Boot, ROS2 Humble, rosbridge, noVNC, Docker Compose.

## Özellikler (Özet)
- Dashboard: tek sağlık kaynağı (`/api/health/summary`), duruma duyarlı Start/Stop/Restart/Recover.
- Simulator: model/senaryo seçimi, 5 aşamalı bağlantı pipeline, basic/advanced kontrol modu.
- Maps: tam CRUD (liste, kaydet, yükle, indir ZIP, önizle, sil, yükle).
- Examples: 13 resmi ROS örneği, resmi doküman linkleri, launch feedback (spinner/başarılı/hata).
- Turtlesim: Fit/Fill/1:1, fullscreen, klavye odağı yönetimi, düğme tabanlı kontrol, anlık cmd_vel geri bildirimi.
- Sağlık: rosbridge, STOMP, noVNC (rviz/turtlesim), DB durumları tek endpoint’te.

## Mimari (Kısa)
```
Frontend (Vite/React 5173)
  ├─ REST: /api/* -> Backend (8082->8080)
  ├─ STOMP SockJS: /ws/robot
  └─ roslib: ws://localhost:9091 (rosbridge)

Backend (Spring Boot)
  ├─ /api/sim/* (compose up/down)
  ├─ /api/health/summary (tek sağlık kaynağı)
  ├─ /api/map/* (CRUD + upload/download/preview)
  ├─ /api/examples/* (manifest bazlı)
  └─ /api/turtlesim/* (cmd_vel)

Docker (root compose)
  - backend:8082, postgres:5432, rosbridge:9091->9090, rviz-novnc:6082->6080, turtlesim-novnc:6081

ROS Stack (ros-stack/docker-compose.yml, orchestrator)
  - rosbridge:9091->9090, tb3-sim, tb3-slam, tb3-nav, rviz:6080
```

## Portlar
- Frontend: 5173  
- Backend API: 8082 (container 8080)  
- STOMP SockJS: `/ws/robot` (8082)  
- ROSBridge: 9091 (container 9090)  **Önemli:** roslib URL’i 9091 kullanın.  
- RViz noVNC: 6082  
- Turtlesim noVNC: 6081  
- Postgres: 5432  

## Hızlı Başlangıç (Docker)
```bash
# Tüm stack
docker compose up -d --build

# (Opsiyonel) ROS stack'i ayrı ayaklandırmak isterseniz
docker compose -f ros-stack/docker-compose.yml up -d --build
```
Erişim:  
- Frontend: http://localhost:5173  
- Health:   http://localhost:8082/api/health/summary  
- RViz:     http://localhost:6082/vnc.html  
- Turtlesim: http://localhost:6081/vnc.html  

## Geliştirme
```bash
# Backend
cd backend && ./gradlew bootRun --args='--spring.profiles.active=docker'

# Frontend
cd frontend && npm install && npm run dev
```

## Temel API’ler
- Health: `GET /api/health/summary`
- Simulation: `POST /api/sim/start`, `POST /api/sim/stop`, `GET /api/sim/status`
- Maps: `GET /api/map/list`, `POST /api/map/save`, `POST /api/map/upload`, `GET /api/map/download/{id}`, `GET /api/map/preview/{id}`, `DELETE /api/map/{id}`, `POST /api/map/load/{id}`
- Examples: `GET /api/examples`, `POST /api/examples/{id}/launch`
- Turtlesim: `POST /api/turtlesim/cmd_vel`, `GET /api/turtlesim/status`

## Konfig (önemli env’ler)
- Backend: `ROSBRIDGE_URL` (varsayılan `ws://rosbridge:9090`), `BACKEND_HOST_PORT=8082`
- Frontend: `VITE_API_URL=/api`, `VITE_WS_URL=/ws/robot`, `VITE_ROSBRIDGE_URL=ws://localhost:9091`
- ROS Stack: `TURTLEBOT3_MODEL`, `GAZEBO_WORLD`, `MAP_FILE`

## Sayfa Özeti
- Dashboard: sağlık kartları, start/stop, son hata kartı.
- Simulator: model/senaryo seç, pipeline ilerlemesi, basic/advanced kontrol.
- Maps: listele/kaydet/yükle/indir/önizle/sil.
- Examples: resmi ROS linkleri, launch feedback.
- Turtlesim: Fit/Fill/1:1, fullscreen, klavye odağı uyarısı, buton kontrolü, cmd_vel geri bildirimi.

## Sağlık ve Test
```bash
curl http://localhost:8082/api/health/summary
curl -X POST http://localhost:8082/api/sim/start -H "Content-Type: application/json" \
  -d '{"model":"BURGER","scenario":"TELEOP"}'
curl http://localhost:8082/api/map/list
```
UI: `/dashboard`, `/simulator`, `/maps`, `/examples`, `/turtlesim`

## Notlar
- ROSBridge host portu 9091; roslib URL’inizi buna göre ayarlayın.
- Orchestrator `ros-stack/docker-compose.yml` kullanır; root compose’taki rosbridge ile port çakışması olmamalı.
- WebSocket/STOMP ve ROS bağlantıları idempotent, backoff’lu ve yalnızca CONNECTED sonrası subscribe olur.

## Lisans
MIT (aksi belirtilmediyse).

