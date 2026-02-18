// AVROS Console - Joystick WebSocket Client

const WS_URL = `wss://${location.host}/ws`;

// Cache DOM elements
const statusEl = document.getElementById('status');
const valuesEl = document.getElementById('values');
const estopBtn = document.getElementById('estop');
const modeButtons = document.querySelectorAll('.modes button');

let ws = null;
let estop = false;
let mode = 'D';
let joystickX = 0;
let joystickY = 0;
let sendInterval = null;

// ===== WEBSOCKET =====
function connect() {
    ws = new WebSocket(WS_URL);

    ws.onopen = () => {
        statusEl.textContent = 'Connected';
        statusEl.className = '';
        startSending();
    };

    ws.onclose = () => {
        statusEl.textContent = 'Disconnected';
        statusEl.className = 'disconnected';
        stopSending();
        setTimeout(connect, 1000);
    };

    ws.onmessage = (e) => {
        const data = JSON.parse(e.data);
        if (data.error) {
            statusEl.textContent = data.error;
            statusEl.className = 'disconnected';
            stopSending();
            return;
        }
        valuesEl.textContent = `T: ${data.t.toFixed(2)} | S: ${data.s.toFixed(2)} | B: ${data.b.toFixed(2)}`;
    };
}

function send(data) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(data));
    }
}

function startSending() {
    if (sendInterval) return;
    sendInterval = setInterval(() => {
        send({ type: 'control', x: joystickX, y: joystickY });
    }, 50);
}

function stopSending() {
    if (sendInterval) {
        clearInterval(sendInterval);
        sendInterval = null;
    }
}

// ===== JOYSTICK =====
const joystick = nipplejs.create({
    zone: document.getElementById('joystick-zone'),
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: '#1976d2',
    size: 150,
    restOpacity: 0.7
});

joystick.on('move', (evt, data) => {
    const maxDist = 75;
    joystickX = Math.max(-1, Math.min(1, data.vector.x * data.distance / maxDist));
    joystickY = Math.max(-1, Math.min(1, data.vector.y * data.distance / maxDist));
});

joystick.on('end', () => {
    joystickX = 0;
    joystickY = 0;
});

// ===== E-STOP =====
estopBtn.addEventListener('click', () => {
    estop = !estop;
    estopBtn.classList.toggle('active', estop);
    estopBtn.textContent = estop ? 'E-STOP ACTIVE' : 'E-STOP';
    send({ type: 'estop', value: estop });
});

// ===== MODE BUTTONS =====
modeButtons.forEach(btn => {
    btn.addEventListener('click', () => {
        modeButtons.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        mode = btn.dataset.mode;
        send({ type: 'mode', value: mode });
    });
});

// ===== START =====
connect();
