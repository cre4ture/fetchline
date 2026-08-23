const STORAGE_KEY = "fetchline-servo-control-v1";
const POSITION_MIN = 0;
const POSITION_MAX = 4095;
const CONTROL_INTERVAL_MS = 45;

const defaults = {
  endpoint: { host: "192.168.1.123", port: 3333 },
  motor: { id: 1, speedPercent: 25, acceleration: 20 },
  joints: Array.from({ length: 6 }, (_, index) => ({
    id: index + 2,
    acceleration: 20,
    torquePercent: 100,
  })),
};

let config = loadConfig();
let socket;
let connectedToMcu = false;
const currentPositions = new Map();
const queuedMoves = new Map();

function loadConfig() {
  try {
    const stored = JSON.parse(localStorage.getItem(STORAGE_KEY));
    if (!stored) return structuredClone(defaults);
    return {
      endpoint: {
        host: typeof stored.endpoint?.host === "string" ? stored.endpoint.host : defaults.endpoint.host,
        port: clampNumber(stored.endpoint?.port, 1, 65535, defaults.endpoint.port),
      },
      motor: {
        id: clampNumber(stored.motor?.id, 1, 253, defaults.motor.id),
        speedPercent: clampNumber(stored.motor?.speedPercent, 0, 100, defaults.motor.speedPercent),
        acceleration: clampNumber(stored.motor?.acceleration, 0, 254, defaults.motor.acceleration),
      },
      joints: Array.from({ length: 6 }, (_, index) => ({
        id: clampNumber(stored.joints?.[index]?.id, 1, 253, index + 2),
        acceleration: clampNumber(stored.joints?.[index]?.acceleration, 0, 254, 20),
        torquePercent: clampNumber(stored.joints?.[index]?.torquePercent, 0, 100, 100),
      })),
    };
  } catch {
    return structuredClone(defaults);
  }
}

function clampNumber(value, minimum, maximum, fallback) {
  const number = Number(value);
  return Number.isFinite(number) ? Math.min(maximum, Math.max(minimum, Math.round(number))) : fallback;
}

function saveConfig() {
  localStorage.setItem(STORAGE_KEY, JSON.stringify(config));
}

function setStatus(text, kind = "") {
  const status = document.querySelector("#connection-status");
  status.textContent = text;
  status.className = `status ${kind}`;
}

function motorSpeed() {
  return Math.round((config.motor.speedPercent / 100) * 4095);
}

function torqueLimit(percent) {
  return Math.round((percent / 100) * 1000);
}

function render() {
  document.querySelector("#mcu-host").value = config.endpoint.host;
  document.querySelector("#mcu-port").value = config.endpoint.port;
  renderMotor();
  renderJoints();
}

function renderMotor() {
  const motor = config.motor;
  document.querySelector("#motor-panel").innerHTML = `
    <div class="card-heading">
      <div><p class="eyebrow">SERVO 1</p><h2 id="motor-title">Continuous motor</h2></div>
      <span class="joint-number">RUN UNTIL STOP</span>
    </div>
    <div class="motor-layout">
      <label>Servo ID
        <input id="motor-id" type="number" min="1" max="253" value="${motor.id}">
      </label>
      <div class="control">
        <label class="range-label" for="motor-speed">Maximum speed <output id="motor-speed-value">${motor.speedPercent}%</output></label>
        <input id="motor-speed" type="range" min="0" max="100" value="${motor.speedPercent}">
        <p class="hint">${motorSpeed()} / 4095 STS speed units</p>
      </div>
      <div class="control">
        <label class="range-label" for="motor-acceleration">Acceleration profile <output id="motor-acceleration-value">${motor.acceleration}</output></label>
        <input id="motor-acceleration" type="range" min="0" max="254" value="${motor.acceleration}">
        <p class="hint">0 is immediate; higher values ramp more gently.</p>
      </div>
    </div>
    <div class="motor-actions">
      <button id="motor-clockwise" type="button">Run clockwise</button>
      <button id="motor-counterclockwise" type="button" class="quiet">Run counter-clockwise</button>
      <button id="motor-stop" type="button" class="stop">Stop</button>
    </div>
    <p class="hint">Run selects STS continuous mode for this servo (a persistent servo setting), enables torque, and then starts it at the selected speed.</p>`;

  document.querySelector("#motor-id").addEventListener("change", (event) => {
    config.motor.id = clampNumber(event.target.value, 1, 253, config.motor.id);
    event.target.value = config.motor.id;
    saveConfig();
  });
  document.querySelector("#motor-speed").addEventListener("input", (event) => {
    config.motor.speedPercent = Number(event.target.value);
    document.querySelector("#motor-speed-value").value = `${config.motor.speedPercent}%`;
    document.querySelector("#motor-speed").nextElementSibling.textContent = `${motorSpeed()} / 4095 STS speed units`;
    saveConfig();
  });
  document.querySelector("#motor-acceleration").addEventListener("input", (event) => {
    config.motor.acceleration = Number(event.target.value);
    document.querySelector("#motor-acceleration-value").value = config.motor.acceleration;
    saveConfig();
  });
  document.querySelector("#motor-clockwise").addEventListener("click", () => startMotor("clockwise"));
  document.querySelector("#motor-counterclockwise").addEventListener("click", () => startMotor("counterclockwise"));
  document.querySelector("#motor-stop").addEventListener("click", () => send({ type: "stop_motor", id: config.motor.id }));
}

function renderJoints() {
  document.querySelector("#position-panels").innerHTML = config.joints.map((joint, index) => {
    const position = currentPositions.get(index) ?? 2048;
    return `
      <article class="card joint-card" data-joint-index="${index}">
        <div class="joint-title">
          <div><p class="eyebrow">SERVO ${index + 2}</p><h2>Position hold</h2></div>
          <span class="joint-number">ID ${joint.id}</span>
        </div>
        <p class="position-readout"><output id="position-value-${index}">${position}</output> <span>/ 4095 steps</span></p>
        <input class="position-slider" data-position-index="${index}" type="range" min="${POSITION_MIN}" max="${POSITION_MAX}" value="${Math.max(POSITION_MIN, Math.min(POSITION_MAX, position))}" aria-label="Servo ${index + 2} target position">
        <div class="joint-settings">
          <label>Servo ID
            <input data-id-index="${index}" type="number" min="1" max="253" value="${joint.id}">
          </label>
          <label>Max. acceleration
            <input data-acceleration-index="${index}" type="number" min="0" max="254" value="${joint.acceleration}">
          </label>
          <label>Holding torque
            <input data-torque-index="${index}" type="number" min="0" max="100" value="${joint.torquePercent}">
          </label>
        </div>
        <p class="hint">Acceleration: 0–254. Holding torque: ${torqueLimit(joint.torquePercent)} / 1000 STS units.</p>
      </article>`;
  }).join("");

  document.querySelectorAll("[data-id-index]").forEach((element) => element.addEventListener("change", (event) => {
    const index = Number(event.target.dataset.idIndex);
    config.joints[index].id = clampNumber(event.target.value, 1, 253, config.joints[index].id);
    event.target.value = config.joints[index].id;
    saveConfig();
    renderJoints();
  }));
  document.querySelectorAll("[data-acceleration-index]").forEach((element) => element.addEventListener("change", (event) => {
    const index = Number(event.target.dataset.accelerationIndex);
    config.joints[index].acceleration = clampNumber(event.target.value, 0, 254, config.joints[index].acceleration);
    event.target.value = config.joints[index].acceleration;
    saveConfig();
  }));
  document.querySelectorAll("[data-torque-index]").forEach((element) => element.addEventListener("change", (event) => {
    const index = Number(event.target.dataset.torqueIndex);
    config.joints[index].torquePercent = clampNumber(event.target.value, 0, 100, config.joints[index].torquePercent);
    event.target.value = config.joints[index].torquePercent;
    saveConfig();
    renderJoints();
  }));
  document.querySelectorAll("[data-position-index]").forEach((element) => {
    element.addEventListener("input", (event) => queuePositionMove(Number(event.target.dataset.positionIndex), Number(event.target.value)));
    element.addEventListener("change", (event) => queuePositionMove(Number(event.target.dataset.positionIndex), Number(event.target.value), true));
  });
}

function startMotor(direction) {
  send({
    type: "start_motor",
    id: config.motor.id,
    speed: motorSpeed(),
    acceleration: config.motor.acceleration,
    direction,
  });
}

function queuePositionMove(index, position, flush = false) {
  currentPositions.set(index, position);
  document.querySelector(`#position-value-${index}`).value = position;
  const pending = queuedMoves.get(index);
  if (pending) clearTimeout(pending);
  const sendMove = () => {
    queuedMoves.delete(index);
    const joint = config.joints[index];
    send({
      type: "move_position",
      id: joint.id,
      position: currentPositions.get(index),
      acceleration: joint.acceleration,
      torque_limit: torqueLimit(joint.torquePercent),
    });
  };
  if (flush) sendMove();
  else queuedMoves.set(index, setTimeout(sendMove, CONTROL_INTERVAL_MS));
}

function connectMcu() {
  connectedToMcu = false;
  setStatus("Connecting to MCU…");
  send({ type: "connect", host: config.endpoint.host, port: config.endpoint.port }, true);
}

function refreshPositions() {
  send({ type: "read_positions", ids: config.joints.map((joint) => joint.id) });
}

function send(message, allowedBeforeMcuConnect = false) {
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    setStatus("Local host is not connected", "error");
    return;
  }
  if (!allowedBeforeMcuConnect && !connectedToMcu) {
    setStatus("Connect to the MCU first", "error");
    return;
  }
  socket.send(JSON.stringify(message));
}

function openSocket() {
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  socket = new WebSocket(`${protocol}//${location.host}/ws`);
  socket.addEventListener("open", connectMcu);
  socket.addEventListener("close", () => {
    connectedToMcu = false;
    setStatus("Local host disconnected — retrying…", "error");
    setTimeout(openSocket, 1000);
  });
  socket.addEventListener("error", () => setStatus("Could not reach the local host", "error"));
  socket.addEventListener("message", (event) => handleServerMessage(JSON.parse(event.data)));
}

function handleServerMessage(message) {
  if (message.type === "connected") {
    connectedToMcu = true;
    setStatus(`Connected to ${message.address}`, "good");
    refreshPositions();
  } else if (message.type === "position") {
    updatePosition(message.id, message.position);
  } else if (message.type === "positions") {
    message.positions.forEach(({ id, position }) => updatePosition(id, position));
  } else if (message.type === "error") {
    connectedToMcu = false;
    setStatus(message.message, "error");
  }
}

function updatePosition(id, position) {
  const index = config.joints.findIndex((joint) => joint.id === id);
  if (index === -1) return;
  currentPositions.set(index, position);
  const output = document.querySelector(`#position-value-${index}`);
  const slider = document.querySelector(`[data-position-index="${index}"]`);
  if (output) output.value = position;
  if (slider && position >= POSITION_MIN && position <= POSITION_MAX) slider.value = position;
}

document.addEventListener("DOMContentLoaded", () => {
  render();
  document.querySelector("#connection-form").addEventListener("submit", (event) => {
    event.preventDefault();
    config.endpoint.host = document.querySelector("#mcu-host").value.trim();
    config.endpoint.port = clampNumber(document.querySelector("#mcu-port").value, 1, 65535, config.endpoint.port);
    document.querySelector("#mcu-port").value = config.endpoint.port;
    saveConfig();
    connectMcu();
  });
  document.querySelector("#refresh-positions").addEventListener("click", refreshPositions);
  openSocket();
});
