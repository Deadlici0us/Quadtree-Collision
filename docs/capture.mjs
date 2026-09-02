// Captures the three demo views as short animated GIFs for the README.
//
// Pipeline per view:
//   1. Open the page in a fresh Playwright context with recordVideo enabled.
//   2. Let the simulation run for CAPTURE_SECONDS so the boids/collisions
//      have visible motion in the loop.
//   3. Close the context (Playwright finalises the .webm).
//   4. Pipe the .webm through ffmpeg's two-pass palette pipeline:
//        ffmpeg -i in.webm -vf "fps=FPS,scale=W:-1:flags=lanczos,palettegen" palette.png
//        ffmpeg -i in.webm -i palette.png -lavfi "fps=FPS,scale=W:-1:flags=lanczos [x]; [x][1:v] paletteuse" -loop 0 out.gif
//
// No system packages beyond Chromium (Playwright) and ffmpeg are required.

import { chromium } from "/tmp/opencode/pw/node_modules/playwright/index.mjs";
import { createServer } from "node:http";
import { readFile, mkdir, rm, rename } from "node:fs/promises";
import { spawn } from "node:child_process";
import { extname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = "/workspace/Quadtree-Collision";
const OUT = `${ROOT}/docs/screenshots`;
const VID = `${ROOT}/docs/.videos`;
const PORT = 8765;
const FPS = 8;
const WIDTH = 480;
const CAPTURE_SECONDS = 4;
const SETTLE_SECONDS = 2.5;
const MAX_COLORS = 16;

await mkdir(OUT, { recursive: true });
await mkdir(VID, { recursive: true });

const MIME = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".svg": "image/svg+xml",
  ".wasm": "application/wasm",
  ".json": "application/json",
};

const server = createServer(async (req, res) => {
  try {
    const url = new URL(req.url, `http://localhost:${PORT}`);
    let path = decodeURIComponent(url.pathname);
    if (path === "/") path = "/index.html";
    const full = resolve(join(ROOT, path));
    if (!full.startsWith(ROOT)) {
      res.writeHead(403).end();
      return;
    }
    const data = await readFile(full);
    res.writeHead(200, {
      "Content-Type": MIME[extname(full)] ?? "application/octet-stream",
      "Cross-Origin-Embedder-Policy": "require-corp",
      "Cross-Origin-Opener-Policy": "same-origin",
    });
    res.end(data);
  } catch (e) {
    res.writeHead(404).end(String(e));
  }
});

function ffmpeg(args) {
  return new Promise((resolve, reject) => {
    const p = spawn("ffmpeg", args, { stdio: ["ignore", "inherit", "inherit"] });
    p.on("error", reject);
    p.on("exit", (code) =>
      code === 0 ? resolve() : reject(new Error(`ffmpeg exited ${code}`))
    );
  });
}

async function recordAndEncode({ url, clickSelector, gifName }) {
  const browser = await chromium.launch();
  const context = await browser.newContext({
    viewport: { width: 1280, height: 720 },
    deviceScaleFactor: 1,
    recordVideo: { dir: VID, size: { width: 1280, height: 720 } },
  });
  const page = await context.newPage();
  page.on("pageerror", (e) => console.error("PAGEERROR:", e.message));
  page.on("console", (m) => {
    if (m.type() === "error") console.error("CONSOLE.error:", m.text());
  });

  await page.goto(url, { waitUntil: "networkidle" });
  if (clickSelector) {
    await page.click(clickSelector);
  }
  // Give the simulation time to settle into a steady state, then record.
  await page.waitForTimeout(1500);

  const videoPath = await page.video().path();
  // Hold the recording open for CAPTURE_SECONDS.
  await page.waitForTimeout(CAPTURE_SECONDS * 1000);
  await context.close();
  await browser.close();

  const palettePath = join(VID, `${gifName}.palette.png`);
  const gifPath = join(OUT, `${gifName}.gif`);

  // Two-pass palette for best quality at small size.
  // The recording starts at page load and includes the settle period; trim
  // the first SETTLE_SECONDS (-ss) and keep CAPTURE_SECONDS of motion (-t).
  const vfScale = `scale=${WIDTH}:-1:flags=lanczos`;

  await ffmpeg([
    "-y",
    "-ss", String(SETTLE_SECONDS),
    "-t", String(CAPTURE_SECONDS),
    "-i", videoPath,
    "-vf", `fps=${FPS},${vfScale},palettegen=stats_mode=diff:max_colors=${MAX_COLORS}`,
    palettePath,
  ]);

  await ffmpeg([
    "-y",
    "-ss", String(SETTLE_SECONDS),
    "-t", String(CAPTURE_SECONDS),
    "-i", videoPath,
    "-i", palettePath,
    "-lavfi", `fps=${FPS},${vfScale} [x]; [x][1:v] paletteuse=dither=none`,
    "-loop", "0",
    gifPath,
  ]);

  await rm(palettePath, { force: true });
  console.log(`captured ${gifName}.gif`);
}

await new Promise((r) => server.listen(PORT, r));
console.log(`server up on :${PORT}`);

try {
  // Wipe stale videos from prior runs.
  await rm(VID, { recursive: true, force: true });
  await mkdir(VID, { recursive: true });

  await recordAndEncode({
    url: `http://localhost:${PORT}/index.html`,
    clickSelector: null,
    gifName: "split",
  });

  await recordAndEncode({
    url: `http://localhost:${PORT}/index.html`,
    clickSelector: 'button[data-view="boids"]',
    gifName: "boids",
  });

  await recordAndEncode({
    url: `http://localhost:${PORT}/index.html`,
    clickSelector: 'button[data-view="collisions"]',
    gifName: "collisions",
  });
} finally {
  await rm(VID, { recursive: true, force: true });
  server.close();
}
console.log("done");