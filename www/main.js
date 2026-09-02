// QuadTree Collision Sandbox — frontend driver.
//
// Loads the Rust-built WebAssembly module, owns the Canvas2D render loop,
// and wires the control panel. The hot path (one rAF tick) reads a single
// Float32Array exposed by the WASM module and blits it to the canvas as
// `fillRect` per particle.

const wasm = await import("../pkg/quadtree_collision.js");
// The default export is the async init function. After it resolves, the
// module's WebAssembly.Memory is reachable through `wasm.memory`.
const wasmInstance = await wasm.default();

const VIEW = { SPLIT: 0, BOIDS: 1, COLLISIONS: 2 };
const STATE = { paused: false, showQt: false };

const canvasLeft = document.getElementById("canvas-left");
const canvasRight = document.getElementById("canvas-right");
const fpsValue = document.getElementById("fps-value");
const leftLabel = document.getElementById("left-label");
const rightLabel = document.getElementById("right-label");
const totalValue = document.getElementById("total-value");
const stage = document.getElementById("stage");
const totalHint = document.getElementById("total-hint");

const ctxLeft = canvasLeft.getContext("2d", { alpha: false });
const ctxRight = canvasRight.getContext("2d", { alpha: false });

// One sandbox handles both sides; the WASM module manages the memory
// layout internally. The world is the viewport (clamped to a minimum so
// the simulation has room to work on very small windows). Compute the
// initial world size from the stage rect *before* constructing the
// sandbox so the first frame already has the right scale.
const MIN_WORLD_W = 640;
const MIN_WORLD_H = 360;

const stageRect0 = stage.getBoundingClientRect();
const dpr0 = Math.min(window.devicePixelRatio || 1, 2);
const initW = Math.max(MIN_WORLD_W, Math.floor(stageRect0.width * dpr0));
const initH = Math.max(MIN_WORLD_H, Math.floor(stageRect0.height * dpr0));
const sandbox = new wasm.Sandbox(500, initW, initH);
sandbox.set_view_split();

/**
 * Clamps the world to a minimum size. The world always matches the
 * viewport (so the simulation fills the available space); on viewports
 * smaller than the minimum we hold the world at 640x360 and the canvas
 * clips via CSS `overflow: hidden`.
 */
function clampWorld(fullW, fullH) {
  return {
    w: Math.max(MIN_WORLD_W, fullW),
    h: Math.max(MIN_WORLD_H, fullH),
  };
}

let currentView = "split";

function setView(view) {
  currentView = view;
  if (view === "split") {
    sandbox.set_view_split();
    canvasLeft.classList.remove("full");
    canvasRight.classList.remove("full");
    canvasLeft.style.display = "block";
    canvasRight.style.display = "block";
  } else {
    const mode = view === "boids" ? 0 : 1;
    sandbox.set_view_single(mode);
    canvasLeft.classList.add("full");
    canvasRight.style.display = "none";
  }
  refreshSize();
  updateLabels();
}

function updateLabels() {
  if (currentView === "split") {
    leftLabel.textContent = "boids";
    rightLabel.textContent = "elastic";
    totalHint.textContent = `Total: ${(
      sandbox.particle_count_per_side() * 2
    ).toLocaleString()} across both sides.`;
  } else if (currentView === "boids") {
    leftLabel.textContent = "boids";
    rightLabel.textContent = "—";
    totalHint.textContent = `Total: ${sandbox.particle_count_per_side().toLocaleString()} boids.`;
  } else {
    leftLabel.textContent = "elastic";
    rightLabel.textContent = "—";
    totalHint.textContent = `Total: ${sandbox.particle_count_per_side().toLocaleString()} particles.`;
  }
  totalValue.textContent = currentView === "split"
    ? (sandbox.particle_count_per_side() * 2).toLocaleString()
    : sandbox.particle_count_per_side().toLocaleString();
}

// --- Size handling ---

let stageRect = null;

function refreshSize() {
  stageRect = stage.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const fullW = Math.floor(stageRect.width * dpr);
  const fullH = Math.floor(stageRect.height * dpr);
  const { w: worldW, h: worldH } = clampWorld(fullW, fullH);

  if (currentView === "split") {
    const halfW = Math.floor(fullW / 2);
    canvasLeft.width = halfW;
    canvasLeft.height = fullH;
    canvasRight.width = fullW - halfW;
    canvasRight.height = fullH;
  } else {
    canvasLeft.width = fullW;
    canvasLeft.height = fullH;
  }
  sandbox.set_world(worldW, worldH);
}

const ro = new ResizeObserver(refreshSize);
ro.observe(stage);
refreshSize();

// --- UI wiring ---

document.querySelectorAll(".seg-btn").forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll(".seg-btn").forEach((b) => b.classList.remove("active"));
    btn.classList.add("active");
    setView(btn.dataset.view);
  });
});

const countInput = document.getElementById("count");
const countValue = document.getElementById("count-value");
let countDebounce = null;
countInput.addEventListener("input", () => {
  const v = parseInt(countInput.value, 10);
  countValue.textContent = v.toLocaleString();
  if (countDebounce) clearTimeout(countDebounce);
  countDebounce = setTimeout(() => {
    sandbox.set_count(v);
    updateLabels();
  }, 150);
});

const boidSliders = [
  { id: "boid-perception", param: "perception" },
  { id: "boid-sep", param: "separation_radius" },
  { id: "boid-speed", param: "max_speed" },
  { id: "boid-force", param: "max_force" },
];
boidSliders.forEach(({ id, param }) => {
  const el = document.getElementById(id);
  const out = document.getElementById(`${id}-value`);
  const handler = () => {
    const v = parseFloat(el.value);
    out.textContent = v.toFixed(0);
    sandbox.set_param(param, v);
  };
  el.addEventListener("input", handler);
  handler();
});

const collSliders = [
  { id: "coll-damping", param: "damping", digits: 3 },
  { id: "coll-restitution", param: "restitution", digits: 2 },
];
collSliders.forEach(({ id, param, digits }) => {
  const el = document.getElementById(id);
  const out = document.getElementById(`${id}-value`);
  const handler = () => {
    const v = parseFloat(el.value);
    out.textContent = v.toFixed(digits);
    sandbox.set_param(param, v);
  };
  el.addEventListener("input", handler);
  handler();
});

document.getElementById("show-qt").addEventListener("change", (e) => {
  STATE.showQt = e.target.checked;
});

const pauseBtn = document.getElementById("pause-btn");
pauseBtn.addEventListener("click", () => {
  STATE.paused = !STATE.paused;
  pauseBtn.textContent = STATE.paused ? "Resume" : "Pause";
});

document.getElementById("reset-btn").addEventListener("click", () => {
  if (currentView === "split") {
    sandbox.set_view_split();
  } else {
    sandbox.set_view_single(currentView === "boids" ? 0 : 1);
  }
  // Refresh the HUD so the FPS ring / totals are consistent with the
  // freshly seeded particles.
  updateLabels();
  lastFps = 0;
  fpsCounter = 0;
  lastFpsTime = performance.now();
});

// --- Render ---

function drawSide(ctx, w, h, positions, count, offset, color) {
  // Background clear with subtle trail.
  ctx.fillStyle = "rgba(6, 8, 16, 1)";
  ctx.fillRect(0, 0, w, h);

  ctx.fillStyle = color;
  for (let i = 0; i < count; i++) {
    const base = (i + offset) * 4;
    const x = positions[base];
    const y = positions[base + 1];
    ctx.fillRect(x - 1, y - 1, 2, 2);
  }
}

function drawQtOverlay(ctx, w, h, nodesPtr, nodesLen) {
  if (!nodesPtr || nodesLen === 0) return;
  const mem = wasmInstance.memory.buffer;
  const view = new Float32Array(mem, nodesPtr, nodesLen);
  ctx.strokeStyle = "rgba(94, 234, 212, 0.25)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let i = 0; i < nodesLen; i += 5) {
    const x = view[i];
    const y = view[i + 1];
    const ww = view[i + 2];
    const hh = view[i + 3];
    ctx.rect(x, y, ww, hh);
  }
  ctx.stroke();
}

function drawDivider(ctx, x, h) {
  ctx.strokeStyle = "rgba(255, 255, 255, 0.08)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(x, 0);
  ctx.lineTo(x, h);
  ctx.stroke();
}

let lastFps = 0;
let fpsCounter = 0;
let lastFpsTime = performance.now();

function tick() {
  if (!STATE.paused) {
    sandbox.step(1 / 60);
  }
  fpsCounter++;

  const now = performance.now();
  if (now - lastFpsTime > 500) {
    lastFps = (fpsCounter * 1000) / (now - lastFpsTime);
    fpsValue.textContent = lastFps.toFixed(0);
    fpsCounter = 0;
    lastFpsTime = now;
  }

  const posPtr = sandbox.positions_ptr();
  const posLen = sandbox.positions_len();
  const mem = wasmInstance.memory.buffer;
  const positions = new Float32Array(mem, posPtr, posLen);

  if (currentView === "split") {
    const halfW = canvasLeft.width;
    const fullH = canvasLeft.height;
    const count = sandbox.particle_count_per_side();
    drawSide(ctxLeft, halfW, fullH, positions, count, 0, "#5eead4");
    drawSide(ctxRight, canvasRight.width, fullH, positions, count, count, "#fb923c");
    if (STATE.showQt) {
      drawQtOverlay(ctxLeft, halfW, fullH, sandbox.left_qt_nodes_ptr(), sandbox.left_qt_nodes_len());
      drawQtOverlay(ctxRight, canvasRight.width, fullH, sandbox.right_qt_nodes_ptr(), sandbox.right_qt_nodes_len());
    }
    drawDivider(ctxLeft, halfW - 0.5, fullH);
  } else {
    const w = canvasLeft.width;
    const h = canvasLeft.height;
    const count = sandbox.particle_count_per_side();
    const color = currentView === "boids" ? "#5eead4" : "#fb923c";
    drawSide(ctxLeft, w, h, positions, count, 0, color);
    if (STATE.showQt) {
      drawQtOverlay(ctxLeft, w, h, sandbox.left_qt_nodes_ptr(), sandbox.left_qt_nodes_len());
    }
  }

  requestAnimationFrame(tick);
}

updateLabels();
requestAnimationFrame(tick);
