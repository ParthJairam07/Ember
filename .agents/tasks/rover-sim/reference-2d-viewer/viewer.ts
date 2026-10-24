import { RoverSim, type SimFrame, type SimWorld } from '/Users/donaldlee/Documents/GitHub-Repos/soon2026/apps/expo-app/src/lib/rover-brain';

const pile = { center: [0, 0, 1] as [number, number, number], radiusM: 1 };
const WORLDS: Record<string, SimWorld> = {
  'Cluttered yard': {
    leaves: [[0, -3], [1.2, -2.6], [-1.5, -3.5], [2.5, -0.5], [-2.2, 0.5], [0.8, -4.2], [-0.6, -1.4], [1.9, -3.8]],
    obstacles: [
      { center: [0, -1.9], radiusM: 0.25, heightM: 1 },
      { center: [1.6, -1.5], radiusM: 0.3, heightM: 0.4 },
      { center: [-1.6, -2.4], radiusM: 0.2, heightM: 1 },
    ],
    pile,
  },
  'Leaves behind a trunk': {
    leaves: [[0, -3], [0.3, -3.4], [-0.4, -3.2]],
    obstacles: [{ center: [0, -1.6], radiusM: 0.25, heightM: 1 }],
    pile,
  },
  'Open patch': { leaves: [[0.5, -2], [-1, -1.5], [1.5, -2.5], [-0.3, -3], [2, -1], [-2, -2.5]], pile },
};

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;
const canvas = $<HTMLCanvasElement>('view');
const ctx = canvas.getContext('2d')!;
const worldSel = $<HTMLSelectElement>('world');
const speedSel = $<HTMLSelectElement>('speed');
const seedIn = $<HTMLInputElement>('seed');
const noisy = $<HTMLInputElement>('noisy');
const playBtn = $<HTMLButtonElement>('play');
for (const name of Object.keys(WORLDS)) worldSel.add(new Option(name, name));

let sim: RoverSim;
let frame: SimFrame;
let trail: [number, number][] = [];
let playing = true;

function reset() {
  const opts = noisy.checked ? { falsePerFrame: 0.3, noisePerM: 0.04, scoopSuccess: 0.7 } : {};
  sim = new RoverSim(structuredClone(WORLDS[worldSel.value]), { seed: Number(seedIn.value) || 1, ...opts });
  sim.start();
  frame = sim.frame();
  trail = [];
  draw();
}

// World metres → canvas pixels. x right, z down the screen, so the rover's start heading (−z) is up.
const VIEW = { x0: -3.5, x1: 3.5, z0: -5, z1: 2.3 };
let scale = 1;
let ox = 0;
let oy = 0;
function layout() {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  canvas.width = w * dpr;
  canvas.height = h * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  scale = Math.min(w / (VIEW.x1 - VIEW.x0), h / (VIEW.z1 - VIEW.z0));
  ox = (w - (VIEW.x1 - VIEW.x0) * scale) / 2 - VIEW.x0 * scale;
  oy = (h - (VIEW.z1 - VIEW.z0) * scale) / 2 - VIEW.z0 * scale;
}
const px = (x: number) => ox + x * scale;
const py = (z: number) => oy + z * scale;

const css = (name: string) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

function circle(x: number, z: number, r: number, fill?: string, stroke?: string, width = 1) {
  ctx.beginPath();
  ctx.arc(px(x), py(z), r * scale, 0, Math.PI * 2);
  if (fill) {
    ctx.fillStyle = fill;
    ctx.fill();
  }
  if (stroke) {
    ctx.strokeStyle = stroke;
    ctx.lineWidth = width;
    ctx.stroke();
  }
}

function draw() {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  ctx.clearRect(0, 0, w, h);

  // 1 m grid.
  ctx.strokeStyle = css('--grid');
  ctx.lineWidth = 1;
  for (let x = Math.ceil(VIEW.x0); x <= VIEW.x1; x++) {
    ctx.beginPath();
    ctx.moveTo(px(x), py(VIEW.z0));
    ctx.lineTo(px(x), py(VIEW.z1));
    ctx.stroke();
  }
  for (let z = Math.ceil(VIEW.z0); z <= VIEW.z1; z++) {
    ctx.beginPath();
    ctx.moveTo(px(VIEW.x0), py(z));
    ctx.lineTo(px(VIEW.x1), py(z));
    ctx.stroke();
  }

  // Pile.
  if (frame.pile) {
    const c = frame.pile.center;
    circle(c[0], c[2], frame.pile.radiusM, css('--pile-fill'), css('--pile'), 1.5);
    ctx.fillStyle = css('--pile');
    ctx.font = '600 12px system-ui';
    ctx.textAlign = 'center';
    ctx.fillText('PILE', px(c[0]), py(c[2] + frame.pile.radiusM) - 8);
  }

  // Obstacles.
  for (const o of sim.world.obstacles ?? []) circle(o.center[0], o.center[1], o.radiusM, css('--obstacle'));

  // Path so far.
  ctx.strokeStyle = css('--trail');
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  trail.forEach(([x, z], i) => (i ? ctx.lineTo(px(x), py(z)) : ctx.moveTo(px(x), py(z))));
  ctx.stroke();

  const yaw = (frame.rover.yawDeg * Math.PI) / 180;
  const cam = frame.rover.position;

  // Camera view (about ±27°, to 5 m) and the corridor fan.
  ctx.fillStyle = css('--fov');
  ctx.beginPath();
  ctx.moveTo(px(cam[0]), py(cam[2]));
  for (let a = -27; a <= 27; a += 3) {
    const b = yaw + (a * Math.PI) / 180;
    ctx.lineTo(px(cam[0] + Math.sin(b) * 5), py(cam[2] - Math.cos(b) * 5));
  }
  ctx.closePath();
  ctx.fill();
  for (const c of frame.tick.corridors) {
    const b = yaw + (c.bearingDeg * Math.PI) / 180;
    const d = Math.min(c.clearanceM, 2.5);
    ctx.strokeStyle = c.blockedBy ? css('--blocked') : css('--open');
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(px(cam[0]), py(cam[2]));
    ctx.lineTo(px(cam[0] + Math.sin(b) * d), py(cam[2] - Math.cos(b) * d));
    ctx.stroke();
  }

  // True leaves (filled) and what the brain believes (rings).
  for (const l of frame.leaves) {
    if (l.where === 'scoop') continue;
    circle(l.position[0], l.position[2], 0.06, l.where === 'pile' ? css('--leaf-piled') : css('--leaf'));
  }
  const target = frame.tick.step?.state.targetId;
  for (const t of frame.tracked) {
    if (t.state === 'collected') continue;
    const color = t.state === 'failed' ? css('--blocked') : t.state === 'tentative' ? css('--muted') : css('--belief');
    circle(t.world[0], t.world[2], t.id === target ? 0.16 : 0.11, undefined, color, t.id === target ? 2.5 : 1.25);
  }

  // Rover: body round the axle, scoop in front, heading.
  const fwd = [Math.sin(yaw), -Math.cos(yaw)];
  const ax = cam[0] - fwd[0] * sim.options.axleBackM;
  const az = cam[2] - fwd[1] * sim.options.axleBackM;
  circle(ax, az, sim.options.bodyRadiusM, css('--rover-fill'), css('--rover'), 2);
  const lip = sim.brain.options.autopilot.scoopLipM;
  const half = sim.brain.options.autopilot.scoopWidthM / 2;
  const right = [-fwd[1], fwd[0]];
  const scoopColor = frame.scoop === 'down' ? css('--scoop-down') : frame.scoop === 'dump' ? css('--pile') : css('--rover');
  ctx.strokeStyle = scoopColor;
  ctx.lineWidth = frame.scoop === 'down' ? 5 : 3;
  ctx.beginPath();
  ctx.moveTo(px(cam[0] + fwd[0] * lip - right[0] * half), py(cam[2] + fwd[1] * lip - right[1] * half));
  ctx.lineTo(px(cam[0] + fwd[0] * lip + right[0] * half), py(cam[2] + fwd[1] * lip + right[1] * half));
  ctx.stroke();
  if (frame.stats.inScoop > 0) circle(cam[0] + fwd[0] * 0.05, cam[2] + fwd[1] * 0.05, 0.06, css('--leaf'));
  circle(cam[0], cam[2], 0.035, css('--rover'));

  // Line to the current goal.
  const step = frame.tick.step;
  const goalLeaf = frame.tracked.find((t) => t.id === target);
  const goal =
    step && ['returning', 'dumping'].includes(step.state.mode) && frame.pile
      ? frame.pile.center
      : goalLeaf && ['approaching', 'aligning', 'scooping'].includes(step!.state.mode)
        ? goalLeaf.world
        : null;
  if (goal) {
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = css('--belief');
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(px(cam[0]), py(cam[2]));
    ctx.lineTo(px(goal[0]), py(goal[2]));
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Readouts.
  const s = frame.stats;
  const total = frame.leaves.length;
  $('mode').textContent = step ? step.state.mode : '—';
  $('message').textContent = step?.message ?? 'Ready';
  $('time').textContent = `${Math.floor(frame.timeMs / 60000)}:${String(Math.floor((frame.timeMs / 1000) % 60)).padStart(2, '0')}`;
  $('piled').textContent = `${s.inPile} / ${total}`;
  $('ground').textContent = String(s.onGround);
  $('scoop').textContent = `${frame.scoop}${s.inScoop ? ` · ${s.inScoop} leaf` : ''}`;
  $('trips').textContent = String(s.dumped);
  $('hits').textContent = String(s.collisions);
  $('wheels').textContent = step ? `${step.left.toFixed(0)} / ${step.right.toFixed(0)}` : '0 / 0';
  const done = s.onGround === 0 && s.inScoop === 0;
  $('status').textContent = done ? `Done: every leaf is in the pile in ${$('time').textContent}` : '';
}

let last = performance.now();
let carry = 0;
function loop(now: number) {
  const dt = Math.min(100, now - last);
  last = now;
  const done = frame.stats.onGround === 0 && frame.stats.inScoop === 0;
  if (playing && !done) {
    carry += dt * Number(speedSel.value);
    while (carry >= sim.options.tickMs) {
      carry -= sim.options.tickMs;
      frame = sim.step();
      trail.push([frame.rover.position[0], frame.rover.position[2]]);
      if (frame.stats.onGround === 0 && frame.stats.inScoop === 0) break;
    }
  }
  draw();
  requestAnimationFrame(loop);
}

playBtn.onclick = () => {
  playing = !playing;
  playBtn.textContent = playing ? 'Pause' : 'Play';
};
$('reset').onclick = reset;
worldSel.onchange = reset;
seedIn.onchange = reset;
noisy.onchange = reset;
window.addEventListener('resize', () => {
  layout();
  draw();
});
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', draw);

layout();
reset();
requestAnimationFrame(loop);
