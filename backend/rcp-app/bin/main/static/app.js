/* roslibjs bağlantısı + teleop + /chatter + /odom
 * NOT(ekipler): WS portu hostta compose ile map edilir (varsayılan 9090).
 * Backend /api/sim/status JSON içinde wsUrl döndürüyorsa onu kullanırız;
 * yoksa otomatik "ws://<host>:9090" deneriz.
 */

const els = {
    btnStart:  document.getElementById('btnStart'),
    btnStop:   document.getElementById('btnStop'),
    rosStatus: document.getElementById('rosStatus'),
    wsStatus:  document.getElementById('wsStatus'),
    chatter:   document.getElementById('chatterLog'),
    posX:      document.getElementById('posX'),
    posY:      document.getElementById('posY'),
    posT:      document.getElementById('posT'),
    velLin:    document.getElementById('velLin'),
    velAng:    document.getElementById('velAng'),
    maxLinear: document.getElementById('maxLinear'),
    maxAngular:document.getElementById('maxAngular')
};

let ros = null;
let chatterSub = null;
let odomSub = null;
let cmdVelPub = null;

let reconnectTimer = null;
let reconnectDelayMs = 1000; // exponential backoff başlangıcı
let running = false;         // backend "start" sonrası true

// Teleop durumu
const teleop = {
    linear: 0.0,
    angular: 0.0,
    interval: null,
    rateHz: 15,          // 10–20Hz arası güvenli
    boost: 1.0
};

// Backend API yardımcıları
async function api(method, url, body) {
    const res = await fetch(url, {
        method, headers: { 'Content-Type': 'application/json' },
        body: body ? JSON.stringify(body) : undefined
    });
    if (!res.ok) throw new Error(`${method} ${url} -> ${res.status}`);
    return res.json().catch(() => ({}));
}

// UI durum rozetleri
function setRos(ok)  { els.rosStatus.textContent = `ROS: ${ok ? 'Connected' : 'Disconnected'}`; }
function setWs(txt)  { els.wsStatus.textContent  = `WebSocket: ${txt}`; }

// Bağlanma akışı
async function getWsUrl() {
    // Tercih: backend status { wsUrl } döndürür
    try {
        const s = await api('GET', '/api/sim/status');
        if (s && s.wsUrl) return s.wsUrl;
    } catch { /* düşersek fallback */ }

    // Fallback: 9090 varsay
    const host = window.location.hostname || 'localhost';
    const port = window.WS_PORT || 9090; // ekipler override edebilir
    return `ws://${host}:${port}`;
}

async function connectRos() {
    const url = await getWsUrl();
    setWs(`connecting (${url})...`);

    ros = new ROSLIB.Ros({ url });

    ros.on('connection', () => {
        setWs('connected');
        setRos(true);
        reconnectDelayMs = 1000; // başarıda backoff sıfırla
        setupTopics();
    });

    ros.on('close', () => {
        setWs('closed');
        setRos(false);
        teardownTopics();
        if (running) scheduleReconnect(); // sim çalışıyorsa yeniden dene
    });

    ros.on('error', (e) => {
        setWs('error');
        console.error('ROS error', e);
        setRos(false);
    });
}

function scheduleReconnect() {
    if (reconnectTimer) return;
    reconnectTimer = setTimeout(async () => {
        reconnectTimer = null;
        reconnectDelayMs = Math.min(reconnectDelayMs * 2, 15000); // max 15sn
        await connectRos();
    }, reconnectDelayMs);
}

// Topic kurulumları
function setupTopics() {
    // /chatter (std_msgs/msg/String) — demo data
    chatterSub = new ROSLIB.Topic({
        ros, name: '/chatter', messageType: 'std_msgs/msg/String'
    });
    const maxLines = 200;
    chatterSub.subscribe(msg => {
        const line = `[${new Date().toLocaleTimeString()}] ${msg.data}\n`;
        els.chatter.textContent += line;
        // log’u sınırlı tut
        const lines = els.chatter.textContent.split('\n');
        if (lines.length > maxLines) {
            els.chatter.textContent = lines.slice(lines.length - maxLines).join('\n');
        }
        els.chatter.scrollTop = els.chatter.scrollHeight;
    });

    // /odom (nav_msgs/msg/Odometry) — sim/robot yayınlamıyorsa 0 kalır (normal)
    odomSub = new ROSLIB.Topic({
        ros, name: '/odom', messageType: 'nav_msgs/msg/Odometry'
    });
    odomSub.subscribe(msg => {
        const p = msg.pose.pose.position || { x:0,y:0,z:0 };
        const v = msg.twist.twist || { linear:{x:0}, angular:{z:0} };
        els.posX.textContent  = p.x.toFixed(2);
        els.posY.textContent  = p.y.toFixed(2);
        // Yaw hesaplamasını basit bırakıyoruz; quaternion → yaw dönüşümü istenirse eklenir
        els.posT.textContent  = '0.00';
        els.velLin.textContent= (v.linear?.x ?? 0).toFixed(2);
        els.velAng.textContent= (v.angular?.z ?? 0).toFixed(2);
    });

    // /cmd_vel (geometry_msgs/msg/Twist) — teleop publisher
    cmdVelPub = new ROSLIB.Topic({
        ros, name: '/cmd_vel', messageType: 'geometry_msgs/msg/Twist'
    });
}

function teardownTopics() {
    try { chatterSub && chatterSub.unsubscribe(); } catch {}
    try { odomSub && odomSub.unsubscribe(); } catch {}
    chatterSub = odomSub = null;
    cmdVelPub = null;
}

// Teleop yayın döngüsü
function startTeleopLoop() {
    if (teleop.interval || !cmdVelPub) return;
    const periodMs = Math.max(10, Math.floor(1000 / teleop.rateHz));
    teleop.interval = setInterval(() => {
        const twist = new ROSLIB.Message({
            linear:  { x: teleop.linear * teleop.boost, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: teleop.angular * teleop.boost }
        });
        cmdVelPub.publish(twist);
    }, periodMs);
}

function stopTeleopLoop() {
    if (teleop.interval) {
        clearInterval(teleop.interval);
        teleop.interval = null;
    }
    // yayınları temiz sonlandır
    if (cmdVelPub) cmdVelPub.publish(new ROSLIB.Message({
        linear:{x:0,y:0,z:0}, angular:{x:0,y:0,z:0}
    }));
}

// Klavye kontrolü
function handleKey(isDown, code) {
    const maxLin = parseFloat(els.maxLinear.value || '0.26');
    const maxAng = parseFloat(els.maxAngular.value || '1.82');

    switch (code) {
        case 'ShiftLeft':
        case 'ShiftRight':
            teleop.boost = isDown ? 1.5 : 1.0; // küçük boost
            break;
        case 'KeyW':
        case 'ArrowUp':
            teleop.linear = isDown ?  maxLin : 0.0;
            break;
        case 'KeyS':
        case 'ArrowDown':
            teleop.linear = isDown ? -maxLin : 0.0;
            break;
        case 'KeyA':
        case 'ArrowLeft':
            teleop.angular = isDown ?  maxAng : 0.0;
            break;
        case 'KeyD':
        case 'ArrowRight':
            teleop.angular = isDown ? -maxAng : 0.0;
            break;
        case 'Space':
            teleop.linear = 0.0; teleop.angular = 0.0;
            stopTeleopLoop(); // space’te yayın döngüsünü durdur
            return;
        default:
            return;
    }
    // bir tuşa basıldıysa yayın döngüsünü başlat
    if (isDown) startTeleopLoop();
}

window.addEventListener('keydown', (e) => handleKey(true, e.code));
window.addEventListener('keyup',   (e) => handleKey(false, e.code));

// Start/Stop butonları
els.btnStart.addEventListener('click', async () => {
    try {
        setWs('starting...');
        const res = await api('POST', '/api/sim/start');
        running = true;
        // backend wsUrl döndürüyorsa direkt bağlanırız
        await connectRos();
    } catch (e) {
        console.error(e);
        setWs('start failed');
    }
});

els.btnStop.addEventListener('click', async () => {
    try {
        setWs('stopping...');
        running = false;
        stopTeleopLoop();
        teardownTopics();
        if (ros) { try { ros.close(); } catch {} ros = null; }
        await api('POST', '/api/sim/stop');
        setWs('stopped');
        setRos(false);
    } catch (e) {
        console.error(e);
        setWs('stop failed');
    }
});

// Sayfa açıldığında mevcut duruma bak; çalışıyorsa bağlan
(async function boot() {
    try {
        const s = await api('GET', '/api/sim/status');
        running = !!s?.running;
        if (running) await connectRos();
    } catch {
        /* status yoksa sessiz geç */
    }
})();
