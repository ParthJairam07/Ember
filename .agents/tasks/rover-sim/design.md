# Rover simulated environment: requirements and design

Status: first draft (no design review yet), revised for the user's requirement update (default sample scene first, pile-up job, movable leaves). See section 24 for the update's mapping.
Scope owner: the sim, inside `frontend/` (Parth's web app). The brain (`apps/expo-app/src/lib/rover-brain`) belongs to another agent and is read-only to us.

---

# Part 1: Requirements

## 1. Summary

Build a 3D simulated environment in the frontend's scene area where the real rover brain can drive a virtual leaf-scooping rover whose job is to gather leaves into a pile: scoop, carry, drive to the pile spot, dump. The page opens on a good-looking default sample (flat grass, scattered 3D dry leaves, rover with a phone camera, brain autopilot driving through the simulated detector and LiDAR, naive controller as automatic fallback); that sample is the first milestone, delivered end to end before the other templates and the realism knobs. Every leaf is an individual movable object: the user can drag leaves and small clusters in the 3D view at any time, the rover's wheels and scoop push leaves they touch, and scooped leaves ride in the scoop and fall out at the dump. The sim produces the same sensor data the iPhone produces (ARKit pose, sensor intrinsics, LiDAR depth plus confidence, OWL-ViT-style detection boxes in the upright portrait frame), feeds it through the brain's own `perceive` / `obstacles` / autopilot code via a thin adapter, and applies the brain's wheel and scoop commands to a physically limited differential-drive rover. Scene templates (seeded, deterministic) cover the terrain types the rover will meet, including a best-effort recreation of the reference video `backend/data/raw/IMG_5056.MOV`. A headless core lets the same sim run under vitest without a browser, with correctness tests that must pass and brain scenario runs that are reported, not hard-failed. Later a real Gaussian splat can underlay or replace the procedural ground.

## 2. Assumptions (explicit)

- A1. The brain is the code in `apps/expo-app/src/lib/rover-brain` at run time. Today it exports a `Brain` class (`brain.ts`) plus the lower-level functions; the phone hook `use-rover-brain.ts` still uses the older raw-function API. The adapter supports both (decision DP1).
- A2. Rover top speed is not documented anywhere; we use 0.6 m/s at 100 % (the brain's own `sim.ts` value), configurable.
- A3. The phone is in portrait with the ARKit camera rolled 90° relative to the brain test helper `makePose` (camera +x points down in the world, +y points right). The brain is roll-agnostic (it only uses −z for heading and consistent pose/intrinsics for projection), so this is what the sim emits (decision DP3).
- A4. Sensor image 1920×1440 (landscape, sensor orientation), detection frame 576×768 (upright, `FRAME_SIZE = 768` longest side, as `use-detections.ts`), depth 256×192 sensor orientation. fx = fy = 1545 px gives a 50.0° horizontal (portrait) FOV and 63.7° vertical.
- A5. "Pile" means two things. The brain's `Pile` (`{ center, radiusM }`) is the spot leaves are gathered to; the sim calls it the pile spot (`DropZone` in code). A pile of leaves, whether pre-placed by a template or grown by dumping, is just many individual leaves stacked in one place (no separate pile object). Default pile spot: radius 0.6 m, centred 1.2 m behind the rover's start (it starts facing away from it), shown as a tarp marker; the user can move it.
- A8. The current brain already has carry/dump (`capacity` scoop loads, `returning` → `dumping` → `backing`, `Pile` via `setPile`). The sim/adapter only needs its own dump trip for the fallback raw-function path or a brain that drops the feature (section 17.1).
- A9. Simple 2D ground-plane leaf physics (push, slide with friction, stack height from a grid) is enough; no physics engine.
- A6. The detector is simulated by projection of ground-truth objects with a noise model, not by running a model on rendered pixels.
- A7. Physical climbable step is 0.05 m (wheel radius 0.06 m), lower than the brain's 0.08 m `maxStepM`. That mismatch is deliberate realism; the sim reports it, it does not hide it.

## 3. Functional requirements

FR-0 Default sample (first milestone). On page load the scene area shows the `sample` template (flat grass, ~60 scattered 3D dry leaves, pile spot behind the rover), controller `rover-brain`, preset `realistic`, playing at 1×. The phone inset shows the simulated camera with detection boxes; the rover finds leaves, scoops them, carries them to the pile spot and dumps them so a visible pile grows. If the brain fails to load or goes fatal, the controller switches to `naive` automatically with a visible notice, and the sample still gathers leaves.

FR-1 Scene source dropdown. The Scene panel gets a "Source" dropdown: `Sample: flat ground + leaves` (default), `Splat (file)` (existing behaviour, unchanged), and the other templates: Reference site (IMG_5056), Flat scatter, Leaf piles, Obstacles + leaves, Drop-off / curb edge, Slope / uneven, Distractors, Dense forest floor, Empty baseline. Templates show a seed field (integer) and a reroll button.

FR-2 Deterministic world generation. `generateWorld(templateId, seed)` is a pure function; the same inputs give an identical world.

FR-3 World content. Ground heightfield with surface types (grass, asphalt, soil/mulch), instanced 3D curled dry leaves (tan, brown, orange, yellow) that are individual movable objects, piles made of stacked leaves, solid obstacles (rocks, logs, trunks, posts, fence, wall), drop edges (curb, retaining wall), distractors (pine cones, bark chips, brown rocks, sticks, roots), a drop zone, and a start pose.

FR-4 Rover model and physics. Recognisable 4-wheel rover with front scoop and phone mast. Differential drive with max speed, acceleration limit, motor deadband, per-wheel noise and slip, command latency, and a 500 ms watchdog. Collisions with solid obstacles stop the rover and count a failure. Rolling off a drop counts a failure and disables the rover. Leaving the site bounds counts the same way.

FR-5 Scoop rule. A leaf is collected only when it enters the scoop footprint through its front edge while the scoop is down and the rover moves forward, up to the scoop capacity (default 25 leaves). Side entry or reversing never collects; the scoop's sides push the leaf instead. Piles lose the leaves scooped from them and so shrink. The scoop animates up / down / dump.

FR-5a Pile up. Scooped leaves ride in the scoop and are relocated (never deleted) at a dump: they fall out in front of the lip and settle on the ground or on the leaves already there, so a pile grows visibly. Gathered-into-pile = leaves on the ground inside the pile spot, reported live and in the harness. The pile spot is configurable: template default, "Set pile spot" then click the ground, or drag its marker; changes reach the brain via `setPile` immediately. If the controller has no carry/dump behaviour, the adapter's dump-trip shim takes the rover to the pile spot when the scoop is full or no targets remain, and that gap is reported in the UI and the report.

FR-5b Movable leaves. Each leaf has its own position, yaw, height and velocity. The user can click-drag a single leaf, or Shift-drag a cluster (all leaves within 0.35 m of the grabbed one), anywhere on the ground in the 3D view, paused or running. The rover's wheels, the scoop (sides when down; the whole scoop when its underside is lower than the local leaf layer) and the body front (where the leaf layer is taller than the 5 cm chassis clearance) push leaves they touch. Sensors and collection always use current leaf positions.

FR-6 Sensors (phone-equivalent shapes).
- Camera snapshot: `{ pose: number[16] col-major camera-to-world (AR world), intrinsics (sensor image), timestamp (s), trackingNormal, depth? }`.
- Detection result: `{ detections: { box: {xmin,ymin,xmax,ymax}, label, confidence }[], frameSize: {576,768}, camera }`, boxes in upright-frame pixels.
- LiDAR depth 256×192, float32 little-endian metres along the optical axis, row-major sensor orientation, confidence bytes 0/1/2, at 5 Hz, ~5 m range.
- AR pose with optional drift, noise, arbitrary AR origin, and tracking-loss events.

FR-7 Simulated detector: latency (~300 ms), low and spread scores (0.05–0.35), distance/size/contrast-dependent misses, false positives on distractors and background, box jitter, occlusion, grass occlusion at grazing angles.

FR-8 Realism knobs panel with presets `ideal` / `realistic` / `harsh` and per-knob editing (`custom`).

FR-9 Controllers: `rover-brain` (via adapter), `naive` (go to nearest detection, built in), `manual` (WASD keyboard, built in). Brain import or runtime errors are caught and shown in the UI; the sim never crashes because of the brain.

FR-10 Controls: play, pause, step (one 200 ms controller tick), reset, speed 0.25×–8×, controller selector, follow vs orbit camera.

FR-11 Overlays (toggleable): FOV cone, phone-camera inset with detection boxes and captions, LiDAR points, corridors, path trail, target line, brain leaf map (tentative / confirmed / collected / failed).

FR-12 Readout: controller status, brain mode and message, scoop load (n / capacity), collected / total leaves (%), gathered into pile (n / total, %), dumps, collisions, drop-offs, stuck, sim time, distance, actual speed factor.

FR-13 Headless harness. Core sim (world, physics, sensors, depth by raycasting) imports nothing from DOM or three.js and runs in Node under vitest. `npm run test` = `vitest --run` runs environment-correctness tests that must pass. `npm run sim:report` runs every template with the brain and writes a metrics report without failing on brain performance.

FR-14 Splat-ready world layer. The world model and renderer accept an optional splat underlay with a world transform; the raycaster has a pluggable surface interface so a splat-derived surface can replace the procedural ground later.

## 4. Non-functional requirements

- NFR-1 Determinism: no `Math.random`, `Date.now` or `performance.now` inside `src/sim/core`; fixed 10 ms physics step; seeded RNG streams per subsystem.
- NFR-1a Determinism scope: runs without user edits are deterministic per seed; a drag is a user event applied at a step boundary and is outside that guarantee.
- NFR-2 Performance: at 1× the browser sim holds real time on a recent laptop with the reference template (≤ 12 ms sim work per animation frame, leaf physics included for up to 2500 leaves); dragging stays at display frame rate; a 256×192 depth scan costs ≤ 20 ms on flat or heightfield terrain. When the sim can't keep up, it slows down and shows the actual speed rather than skipping sensor events.
- NFR-3 Isolation: only `src/sim/brainAdapter.ts` imports `@rover-brain`. Nothing under `apps/expo-app` is modified.
- NFR-4 Existing app unaffected: splat viewer, rover remote control, wildfire page and `npm run build` / `npm run lint` keep working.
- NFR-5 Assets: no binary assets required; any optional textures under `frontend/public/sim/` total < 500 KB.
- NFR-6 Accessibility: all new controls are native `<select>`, `<input>`, `<button>` with labels; keyboard reachable; readout uses text, not colour alone.

## 5. Acceptance criteria

0. Opening the app (`npm run dev`, `/`) shows the sample scene within 2 s: flat grass, scattered 3D dry leaves, the rover with scoop and phone mast, the pile-spot marker, and the phone inset with detection boxes; it starts playing with controller `rover-brain`. Within 180 sim-seconds (realistic) the rover has dumped at least once and the pile spot holds at least 5 leaves (informational for the brain; it must hold with `naive` and is checked by an env test with the naive controller under `ideal`).
0a. With the brain import forced to fail (e.g. temporarily aliasing `@rover-brain` to a missing file), the sample switches to `naive`, shows "Rover brain unavailable: … running naive controller", and still gathers leaves into the pile.
0b. Dragging a leaf moves it with the cursor and drops it on the ground; Shift-drag moves a cluster; this works while playing; the detector boxes and brain leaf map follow the new positions within a few detection frames.
0c. Driving the rover (manual) with the scoop up through scattered leaves pushes leaves touched by the wheels aside; none disappear. After a dump, the dumped leaves are visible on the ground in the pile spot, stacked higher than single leaves.
1. Selecting `Splat (file)` shows the existing ScenePanel controls and SplatViewer; Load sample, Open file, Load URL and Clear behave exactly as before.
2. Selecting any template replaces the viewer with the 3D sim view showing ground, leaves and the rover within 2 s on a laptop.
3. Same template + seed renders the same world after reload; a new seed (reroll) changes leaf and obstacle placement.
4. With controller `rover-brain` and preset `ideal` on Flat scatter seed 1, pressing Play makes the rover drive to and scoop at least one leaf within 120 sim-seconds (reported in `sim:report`; this is a brain metric, so it's informational).
5. Making the brain module throw (or fail to import) shows a red error message in the controller panel, the rover stops, and the rest of the page keeps working.
6. Manual controller: holding W drives forward, A/D pivot, S reverses, Q toggles the scoop, E dumps; releasing all keys stops the rover; the real-rover remote does not receive these keys while manual sim control is active.
7. Speed control 0.25×, 0.5×, 1×, 2×, 4×, 8× changes sim rate; the readout shows the actual factor.
8. Every overlay toggle hides / shows its overlay; the phone inset shows the rendered camera view and the boxes the brain received.
9. Realism presets change detector latency, misses, false positives, depth noise, drift and motor behaviour; editing a knob shows `custom`.
10. `npm run test` passes and includes: (a) projection: a leaf at a known world point, ideal settings, `locateDetections` recovers it within 3 cm using LiDAR and within 3 cm via floor ray; (b) portrait convention: a leaf straight ahead lands at the horizontal centre of the upright frame, a leaf to the right lands right of centre; (c) sim math equals brain `uprightToSensor`, `headingFromPose`, `unproject` on random inputs; (d) depth of a flat floor decodes via `scanDepth` to points within 1 cm of y = 0 and `floorSource: 'lidar'`; a wall 2 m ahead gives a corridor clearance at 0° within 0.1 m of the expected value; a curb gives `blockedBy: 'drop'`; (e) kinematics: straight, pivot direction (+yaw = right), accel ramp, deadband, latency, 500 ms watchdog; (f) scoop rule: front entry collects, scoop up / side entry / reverse doesn't, capacity respected, piles shrink, leaf count conserved (ground + scoop + held = total) every step; dump relocates carried leaves into the pile spot and raises the stack height there; wheels push touched leaves; a moved leaf is seen by the detector at its new position; (f2) pile-up end to end: `sample` seed 1, `ideal`, naive controller, 180 s → at least one dump and ≥ 5 leaves gathered in the pile spot; (g) collision and drop-off counting; (h) determinism per seed for world and a 60 s naive-controller run; (i) adapter: fake brain modules that throw or lack exports produce a stopped rover plus an error status.
11. `npm run sim:report` runs all ten templates with the brain under `realistic` and prints / writes a table (collected %, gathered into pile, dumps, dump-trip shim used (yes/no), collisions, drop-offs, stuck, brain errors, time to first scoop, false confirmations). It exits 0 even if the brain does badly; it exits non-zero only if the sim itself crashes.
12. `npm run build` and `npm run lint` succeed. `git status` shows no changes under `apps/expo-app`.

## 6. Out of scope

- Changing the brain or its tests, or the phone app.
- Running OWL-ViT (or any model) on rendered frames.
- Loading a real splat into the sim (only the hook points for it).
- Generating scenes live while the rover moves.
- Arm hardware, BLE, ESP32 protocol changes.
- Persisting knob / overlay settings or user edits, URL deep links, user-placed obstacles (leaves and the pile spot are movable; obstacles are not).
- Rigid-body leaf physics (tumbling, wind, leaf-leaf collisions); leaves slide and stack in 2D plus a height grid only.
- Web Worker offloading (noted as a later optimisation).

---

# Part 2: Design

## 7. Overview

The sim lives in `frontend/src/sim/` in three layers. `core/` is pure TypeScript (no DOM, no three.js): seeded world generation, terrain and primitive raycasting, rover drive physics, scoop and collisions, and the phone sensors (AR pose, LiDAR depth, detector). It advances on a fixed 10 ms step and emits phone-shaped sensor events on the phone's own schedule (5 Hz camera/depth tick, detection frames with latency). `controllers/` turns those events into wheel and scoop commands: `brainAdapter.ts` runs the real brain exactly the way the phone hook does (`locateDetections` → `observe`, `scanDepth` → `addScan`, `tick` → `left/right/scoop`), while `naive.ts` and `manual.ts` are built-in fallbacks. `render/` and `ui/` draw the world with three.js and hold the React panels. The headless harness is just vitest running `core/` plus the adapter in Node. Coordinates are ARKit's: metres, +y up, gravity-aligned, yaw 0 = facing −z, + clockwise seen from above; three.js uses the same right-handed y-up frame, so no conversion between sim and render.

Technology stack (locked): React 19, Vite 8, TypeScript 6, Tailwind 4 (existing); three 0.186 (existing) with `three/examples/jsm/controls/OrbitControls.js`; vitest 5.0.5 (new, pinned exact, devDependency). No other new dependencies.

## 8. Terrain notes (reference frames from IMG_5056)

Viewed frames 00, 03, 04, 06, 09, 10, 11 (1080×1920 portrait, camera near ground level, slight downward tilt).

- Ground: mown lawn, bright green grass 3–6 cm tall with clover patches and darker soil showing through in places. Not bare forest soil. Grass blades partly hide leaves at grazing angles, especially beyond ~2 m.
- Leaves: maple leaves dominate, flat-ish with curled lobes and edges lifted 1–3 cm, 8–15 cm across. Colours: tan/beige (most common), light brown, golden yellow, orange-brown, a few dark red-brown. Density varies a lot: sparse lawn 2–6 leaves/m² (frames 00, 09, 11); dense carpets 60–150 /m² overlapping leaves near the path (frame 06) and on the slope under a large tree (frame 10); a drift band along the treeline.
- Piles: no raked piles in the video, only carpets and drifts. Pile shapes in the Leaf piles template follow raked garden piles (low domes 0.3–0.6 m radius, 8–20 cm high) as the rover's target use case.
- Obstacles: isolated large trunks (0.3–0.6 m diameter) at the lawn edge and near the path; thin poles / lamp post at the path edge (frames 04, 06); a gazebo, a house, a parked car, people and a conifer / cedar hedge in the background (beyond 8 m, so mostly irrelevant to LiDAR but in the camera view).
- Edges: a grey asphalt path ~2 m wide curves along the lawn's edge with a small lip (a few cm) between grass and asphalt; beyond the path, the deciduous forest edge with dense trunks and understory.
- Unevenness: gently undulating lawn; the site rises towards the treeline; one hillside around 15–20° (frame 10).
- Distractors in the real scene: bark on trunks (OWL-ViT's known false hit), dark soil patches, path edge, dry grass. A conifer suggests pine cones and needles nearby.
- Lighting: clear blue sky, low autumn sun, long hard tree shadows across the lawn (frames 03, 09), strong sunlit / shadow contrast; leaves in shadow are noticeably less distinct.

How templates use this:
- Reference site: lawn + curving asphalt path + forest-edge trunk row + lamp post + 3 large trunks + gentle rise and undulation + sparse leaves with two dense carpets and a treeline drift; low sun with trunk shadows.
- Flat scatter: the sparse-lawn density (4 /m²) on flat grass.
- Dense forest floor: the carpet density (100+ /m²) but on brown soil / mulch, where leaf-to-ground contrast is low (matches "misses on leaves blending into soil").
- Slope: the frame-10 hillside (12° main slope, a 25° bank) with leaves on it.
- Distractors: bark chips, pine cones, brown rocks and sticks on the ground, plus trunks (bark), as in the README's known false hits.
- Lighting everywhere: one directional sun at 25° elevation; static shadow flags per leaf lower detection contrast.

Ground look is procedural (canvas-generated textures from seeded noise: grass green #5f8f3a–#7fb24a with clover speckle, asphalt #8c8c88, soil #5a4632). No frame crops are shipped in v1 (keeps the repo clean; the 500 KB budget stays unused and available).

## 9. File layout

New files (all under `frontend/`):

```
src/sim/
  core/
    rng.ts            seeded streams: mulberry32 + FNV-1a(seed, streamName)
    math.ts           Vec3, mat helpers, portraitPose, intrinsics, project/unproject, upright<->sensor
    types.ts          World, LeafSpec, Obstacle, Distractor, DropZone (pile spot), PhoneCamera, DetectionResult, DriveCommand, Edit
    terrain.ts        HeightGrid (5 cm grid, bilinear), surface lookup, tile max for raymarch
    raycast.ts        Raycaster: terrain + leaf stack grid + primitives (cylinder, OBB, ellipsoid, capsule); Surface interface
    leaves.ts         LeafField: per-leaf state arrays, spatial hash, stack (litter) grid, push/slide/settle, edits
    world/
      generate.ts     generateWorld(templateId, seed) + validation
      templates.ts    TEMPLATE_IDS, labels, descriptions, per-template generators
      scatter.ts      Poisson-disc + density-map placement
    rover/
      config.ts       RoverConfig + DEFAULT_ROVER_CONFIG
      drive.ts        command queue, latency, watchdog, deadband, accel, noise, kinematics
      body.ts         body pose on terrain (pitch/roll), footprint, camera mount transform
      collision.ts    footprint vs obstacles, drop/out-of-bounds detection
      scoop.ts        scoop state machine + collection rule + piles + dump
    sensors/
      arPose.ts       AR origin transform, drift, noise, tracking loss
      lidar.ts        depth map render (raycast), noise, confidence, encoding
      detector.ts     projection detector with latency queue
    realism.ts        RealismKnobs + presets ideal/realistic/harsh
    sim.ts            Simulation class: fixed step, event scheduling, controller calls, metrics
    metrics.ts        counters + trajectory hash
  controllers/
    types.ts          Controller interface, ControllerStatus, ControllerDebug
    naive.ts          go-to-nearest using sim math (no brain import)
    manual.ts         WASD state -> commands
    dumpTrip.ts       drive-to-pile-spot + dump routine (shared by naive and the adapter shim)
  brainAdapter.ts     the only importer of @rover-brain
  render/
    SimRenderer.ts    owns WebGLRenderer, scene, cameras, per-frame sync from Simulation
    leafMeshes.ts     3 curled leaf geometries, InstancedMesh with instanceColor
    worldMeshes.ts    ground mesh, textures, obstacles, distractors, piles, backdrop, drop zone marker
    roverModel.ts     body, wheels, scoop pivot, mast, phone
    overlays.ts       FOV cone, LiDAR points, corridors, trail, target line, leaf-map markers
    phoneView.ts      inset render with intrinsics-derived projection
    interaction.ts    pointer picking + drag of leaves / clusters / pile-spot marker
  ui/
    SimView.tsx           viewer container (canvas + inset + HUD), replaces SplatViewer in template mode
    SimControlsPanel.tsx  play/pause/step/reset/speed/controller/camera/overlays
    RealismPanel.tsx      preset select + knobs
    SimReadout.tsx        HUD text
    PhoneInset.tsx        inset canvas + SVG boxes/captions
    useSimulation.ts      owns Simulation + controller + rAF loop, exposes snapshot via useSyncExternalStore
  **/*.test.ts            env-correctness tests (colocated)
  report/scenarios.report.ts
vitest.report.config.ts
tsconfig.test.json
```

Modified files: `frontend/package.json` (vitest devDep, `test`, `sim:report` scripts), `frontend/vite.config.ts` (alias, fs allow, `test` block), `frontend/tsconfig.app.json` (paths, `erasableSyntaxOnly: false`, exclude tests), `frontend/tsconfig.json` (reference `tsconfig.test.json`), `frontend/.gitignore` (`sim-report/`), `frontend/src/App.tsx`, `frontend/src/components/ScenePanel.tsx`, `frontend/src/hooks/useRover.ts` (keyboard on/off option).

## 10. Build integration and the brain import

Vite alias in `vite.config.ts`:

```ts
resolve: { alias: { '@rover-brain': fileURLToPath(new URL('../apps/expo-app/src/lib/rover-brain/index.ts', import.meta.url)) } },
server: { fs: { allow: [searchForWorkspaceRoot(process.cwd()), fileURLToPath(new URL('../apps/expo-app/src/lib/rover-brain', import.meta.url))] } },
test: { include: ['src/**/*.test.ts'], environment: 'node' },
```

(`/// <reference types="vitest/config" />` at the top so `test` type-checks.) The root `package.json` has `workspaces`, so `searchForWorkspaceRoot` already reaches the repo root; the explicit entry keeps it working if that changes. Vite watches imported files outside its root, so edits by the brain agent are served on the next import (and Reset in the UI creates a fresh brain instance from the current module).

TypeScript: `tsconfig.app.json` adds `"paths": { "@rover-brain": ["../apps/expo-app/src/lib/rover-brain/index.ts"] }` (TS 6: no `baseUrl` needed) and excludes `src/**/*.test.ts`, `src/**/*.report.ts`. New `tsconfig.test.json` (types `node`, `vitest/globals` not used; tests import from `vitest`) includes those files plus `src/sim/**`, same paths; `tsconfig.json` references it so `tsc -b` checks tests.

Verified while designing: compiling the brain under the frontend's current options fails with TS1294 on `constructor(private readonly options ...)` in `leaf-map.ts:80` and `obstacles.ts:256` because the frontend sets `erasableSyntaxOnly: true`. Decision: set `erasableSyntaxOnly: false` in `tsconfig.app.json` and `tsconfig.test.json`. Vite's transform handles parameter properties; nothing in the frontend relies on Node type-stripping. The frontend doesn't set `strict`, so the brain's stricter code type-checks fine otherwise. Risk accepted: if the brain agent later writes code that fails the frontend's checks, `npm run build` breaks until fixed; the build step is where that's visible and the adapter's runtime guard still keeps the dev server usable.

Concurrent edits: a separate task (fix-splat-sample-leaves) is changing `ScenePanel.tsx`, `SplatViewer.tsx` and possibly `vite.config.ts` (new default splat `public/splats/leaf-floor.splat`, generator `frontend/scripts/make-leaf-splat.mjs`). The implementer re-reads those files at build time and applies our changes additively on top of theirs (adding the alias / `server.fs` / `test` keys to whatever `defineConfig` object exists; wrapping the existing splat controls rather than rewriting them). Their "Load sample" behaviour and the new splat file stay as they leave them.

Brain baseline: designed against commit b0faf113 ("Rover brain: scoop leaves and pile them up, with a simulator"), whose `rover-brain` matches what was read above; the phone hook `use-rover-brain.ts` at that commit uses `Brain` (`new Brain()`, `observe`, `addScan`, `start`/`stop`, `tick`, `setPile`), which is exactly the call sequence the adapter mirrors.

Runtime import: the adapter loads the brain with `await import('@rover-brain')` inside `try/catch`, never a static import, so a syntax error in a file the brain agent is mid-editing doesn't break the page; it shows up as a controller error. It reads exports from the namespace object, so a renamed export is a detectable `undefined`, not an ESM link error. Type-only imports (`import type { ... } from '@rover-brain'`) are allowed in the adapter only.

## 11. Coordinate conventions and the phone's data shapes

The sim defines its own structural types in `core/types.ts`, copied from the native contract in `apps/expo-app/modules/splat-capture/src/SplatCapture.types.ts` (more stable than the brain's internals):

```ts
type PinholeIntrinsics = { fx: number; fy: number; cx: number; cy: number; width: number; height: number }
type PhoneDepth = { width: number; height: number; data: Uint8Array /* LE float32 */; confidence?: Uint8Array /* 0,1,2 */ }
type PhoneCamera = { timestamp: number /* s */; pose: number[] /* 16, col-major camera->AR world */; intrinsics: PinholeIntrinsics; trackingNormal: boolean; depth?: PhoneDepth }
type SimBox = { xmin: number; ymin: number; xmax: number; ymax: number } // upright-frame pixels
type SimDetection = { box: SimBox; label: string; confidence: number }
type DetectionResult = { detections: SimDetection[]; frameSize: { width: 576; height: 768 }; camera: PhoneCamera }
type DriveCommand = { left: number; right: number; scoop?: 'up' | 'down' | 'dump' } // -100..100
```

These are structurally identical to the brain's `CameraSnapshot` and `BoxDetection`, so the adapter passes them straight through.

Intrinsics (sensor, landscape): `{ fx: 1545, fy: 1545, cx: 960, cy: 720, width: 1920, height: 1440 }`. Upright frame: 576×768, scale `s = 576 / 1440 = 0.4`. Mapping (inverse of brain `uprightToSensor`): sensor (x, y) → upright (u, v) = ((1440 − y)·s, x·s).

Portrait pose: `portraitPose(position, pitchDeg, yawDeg, rollDeg = 0) = makePoseLike(position, pitchDeg, yawDeg) · Rz(−90°)`, i.e. columns `[−y₀, x₀, z₀]` of the non-rolled rotation. So camera +x = world down, +y = world right, −z = look direction. This matches ARKit with the phone in portrait (camera space defined in landscape-right), and the quarter-turn-clockwise upright frame: rows of the upright frame go down the world, columns go right. On slopes the body's pitch and roll are composed in before the mast pitch (section 13). A debug knob `poseConvention: 'arkit-portrait' | 'landscape'` exists for tests only (the brain's own test helpers use the landscape form); env tests run both to show the brain is roll-agnostic.

Depth semantics: each depth pixel stores the distance along the optical axis (camera-space −z), not the ray length, because brain `unproject` multiplies the normalised ray by depth. Depth intrinsics are the sensor intrinsics scaled to 256×192 (`scaleIntrinsics`), and pixel (x, y) is sampled at its centre (x + 0.5, y + 0.5), matching `depthToWorld`.

AR world vs sim world: `T_ar` is a rigid transform (yaw + translation) from sim world to AR world. Ideal: identity. Realistic/harsh: at reset, AR origin = initial camera position on the ground with a random yaw (as ARKit starts its session wherever the phone is), then drift (section 14.1). Everything given to the brain is in AR coordinates; the drop zone is converted with `T_ar` at reset; brain debug output (leaf map, target, corridors) is mapped back with `T_ar⁻¹` for overlays, so drift shows up visually as brain markers offset from true leaves.

Time: `nowMs` passed to the brain is sim time in ms; `camera.timestamp` is sim time in seconds. The brain uses no wall clock (checked: no `Date.now` / `Math.random` in rover-brain), so runs are deterministic.

## 12. World model and templates

```ts
type SurfaceKind = 'grass' | 'asphalt' | 'soil' | 'mulch'
// Static per-leaf look, generated once:
type LeafSpec = { id: number; x: number; z: number; yawRad: number; sizeM: number; curl: number /*0..1*/; shape: 0|1|2; color: number /*hex*/;
                  baseContrast: number /*0..1, colour factor*/ }
// Dynamic state lives in LeafField (core/leaves.ts) as typed arrays indexed by leaf id (section 15.1):
//   x, z, y, yaw, vx, vz, state ('ground' | 'scoop' | 'held'), lastTouchMs, contrast and inShadow (recomputed when a leaf moves)
type Obstacle =
  | { kind: 'cylinder'; tag: 'trunk' | 'post'; x: number; z: number; radiusM: number; heightM: number }
  | { kind: 'box'; tag: 'wall' | 'fence' | 'curb'; cx: number; cz: number; yawRad: number; halfW: number; halfD: number; heightM: number }
  | { kind: 'ellipsoid'; tag: 'rock'; x: number; z: number; rx: number; ry: number; rz: number }
  | { kind: 'capsule'; tag: 'log' | 'root' | 'stick'; ax: number; az: number; bx: number; bz: number; radiusM: number }
type Distractor = { id: number; kind: 'pinecone' | 'bark' | 'brownRock' | 'stick'; x: number; z: number; sizeM: number; yawRad: number; fpWeight: number }
type DropZone = { x: number; z: number; radiusM: number }
type World = {
  templateId: TemplateId; seed: number; bounds: { minX: number; maxX: number; minZ: number; maxZ: number }
  terrain: TerrainSpec   // heightfield params + steps (curbs/walls as height discontinuities) + surface regions
  leaves: LeafSpec[]   // pre-placed piles are clusters of LeafSpecs; the stack grid gives them height
  obstacles: Obstacle[]; distractors: Distractor[]
  dropZone: DropZone | null; start: { x: number; z: number; yawDeg: number }
  sun: { elevationDeg: number; azimuthDeg: number }
  underlay?: { kind: 'splat'; url: string; toWorld: number[] /*16 col-major*/ }   // future
}
```

Obstacles whose top is below the physical climbable step (roots, small sticks) are traversable: they enter depth but don't collide. Obstacles above it are solid. `solid(o) = topHeight(o) > rover.climbStepM`.

Templates (all sizes in metres; densities are means; leaf size 0.08–0.15 m unless stated; pile spot radius 0.6 m, centred 1.2 m behind the start unless stated):

| id | Label | Area | Terrain | Contents |
|---|---|---|---|---|
| `sample` (default) | Sample: flat ground + leaves | 10×10 | flat grass, low sun, mown-stripe texture | 60 leaves (Poisson-disc min 0.25 m, kept 1.0–4.5 m from the start so the first leaves are in view), palette weighted to the reference video (tan 40 %, light brown 25 %, yellow 15 %, orange 12 %, dark 8 %); start at (0, 0) facing −z; pile spot at (0, +1.2); no obstacles |
| `reference` | Reference site (IMG_5056) | 20×14 | grass, 6° rise to +z treeline, 4 cm undulation (λ 3 m); asphalt path 2 m wide along a sine curve at the −x side with 3 cm lip | 3 trunks r 0.15–0.3 by the path, 1 post r 0.06 h 3, treeline row of 25 trunks along +z edge, backdrop forest; leaves 3 /m² + carpet 2×3 at 80 /m² by path + carpet r 1.8 at 120 /m² under one trunk + 1 m drift band along treeline; 6 bark false-hits (trunk bases) |
| `flat-scatter` | Flat ground, scattered leaves | 12×12 | flat grass | 120 leaves Poisson-disc min 0.15 m |
| `piles` | Leaf piles | 12×12 | flat grass | 5 piles as dense clusters (r 0.3–0.6, 40–120 stacked leaves each, giving 8–20 cm via the stack grid); 30 loose leaves |
| `obstacles` | Obstacles + leaves | 14×14 | flat grass | 6 rocks (0.15–0.4), 1 log 2 m × r 0.12, 4 trunks, fence 3 m (0.05 thick, 1.0 h, posts), wall 4×0.2×0.8; 80 leaves, 25 of them within 0.3 m of an obstacle |
| `drop-edge` | Drop-off / curb edge | 14×12 | grass platform; curb step −0.15 to asphalt road along −z side; retaining wall drop −0.40 along +x side | 70 leaves, 15 within 0.4 m of an edge, 8 on the road below the curb |
| `slope` | Slope / uneven ground | 14×14 | 12° main slope + bumps 8 cm (λ 1.5 m) + one 25° bank region | 90 leaves spread over slope and bank |
| `distractors` | Distractors (false hits) | 12×12 | flat grass with soil patches | 60 leaves; 40 pine cones 0.08–0.12; 25 bark chips 0.10–0.20 flat; 12 brown rocks 0.10–0.25 (some solid); 15 sticks; 3 trunks |
| `forest-floor` | Dense mixed forest floor | 12×12 | soil/mulch, 5 cm unevenness | 600 leaves (low contrast), needle patches (visual), 6 trunks, 10 roots (h 0.05), 20 sticks, 30 pine cones, 2 piles |
| `empty` | Empty baseline | 12×12 | flat grass | nothing; drop zone only |

Placement: scatter uses rejection against obstacle footprints (+0.05 m), the start pose clearance circle (0.8 m), the pile spot, and bounds (0.3 m inset). A leaf's `contrast` = surface factor at its current position (grass 0.8, asphalt 0.9, soil 0.35, mulch 0.25) × `baseContrast` (yellow 1.1, tan 1.0, brown 0.8) clamped to 1; `inShadow` = one ray towards the sun against obstacles. Both are recomputed whenever the leaf moves more than 5 cm. Heights come from the stack grid (section 15.1).

Validation (`validateWorld`, run in `generateWorld`, throws `Error('invalid world: ...')`): start pose footprint is collision-free and on ground with < 10° slope; drop zone lies on traversable ground; no leaf centre inside a solid obstacle; counts ≤ caps (leaves 2500, obstacles 200, distractors 300). A throw here is a programmer bug in a template; the UI catches it and shows "Template failed: <message>" in the Scene panel, leaving the previous world in place. Unknown template id is a type error at compile time; at run time (`generateWorld` called with a string from the select) it throws the same way.

Seeded RNG: `rngStream(seed, name)` = mulberry32(FNV-1a-32(`${seed}:${name}`)). Streams: `world`, `detector`, `lidar`, `pose`, `motor`, `tracking`. Separate streams keep, say, a detector knob change from reshuffling motor noise.

Splat-ready: `Raycaster` takes a list of `Surface` objects (`intersect(ray, tMax): number | null`, `heightAt?(x,z)`), with `HeightfieldSurface` and `PrimitiveSurface` implemented now. A future `SplatSurface` (from a downsampled splat point cloud or a mesh fitted to it) implements the same interface and either replaces the heightfield or adds to it; the renderer adds `SplatUnderlayLayer` (GaussianSplats3D `DropInViewer` into the same three.js scene, transformed by `world.underlay.toWorld`) and hides the procedural ground mesh. Neither is built in v1; the types and interfaces are.

## 13. Rover model and physics

```ts
type RoverConfig = {
  trackWidthM: 0.30; wheelBaseM: 0.28; wheelRadiusM: 0.06; bodyLengthM: 0.42; bodyWidthM: 0.34
  axleBackM: 0.12            // drive axle centre behind the camera (brain sim value)
  cameraHeightM: 0.30; cameraPitchDeg: 20   // UI range 0.20–0.40 m, 10–30°
  topSpeedMps: 0.6; maxAccelMps2: 1.5
  climbStepM: 0.05; maxClimbDeg: 22; fallDropM: 0.08
  scoopLipM: 0.15; scoopWidthM: 0.30; scoopDepthM: 0.20; scoopCapacity: 25   // leaves
  chassisClearanceM: 0.05; wheelWidthM: 0.04
  lowerMs: 500; liftMs: 800; dumpMs: 1500
  watchdogMs: 500; controllerTickMs: 200
}
```

Rover frame: the "rover position" is the camera's horizontal position (as in the brain); the axle centre is `axleBackM` behind it along the heading. Camera height and pitch changes apply on Reset (UI says so). If `cameraHeightM` ≠ the brain's `DEFAULT_ROVER.cameraHeightM` (0.30) by more than 2 cm, or `scoopLipM` / `scoopWidthM` differ from `DEFAULT_AUTOPILOT` by more than 2 cm, the controller panel shows an amber "mismatch with brain config" note listing them (the brain falls back to `cameraHeightM` when it can't fit the floor, and plans scoops with its own lip).

Drive pipeline per 10 ms physics step (`drive.ts`):
1. Command queue: commands from the controller are timestamped and released `cmdLatencyMs` later. With probability `cmdDropP` a command is dropped (harsh only).
2. Watchdog: if the last released command is older than `watchdogMs` (500 ms), target = (0, 0). Counted once per episode in metrics `watchdogStops`.
3. Validation: non-finite → 0 plus a controller warning; clamp to [−100, 100].
4. Deadband: |cmd| < `deadbandPct` → 0.
5. Target wheel speed `= cmd / 100 · topSpeedMps · (1 + N(0, motorNoise))` (noise resampled every 200 ms per wheel, so it's a bias, not jitter).
6. Accel limit: actual wheel speed moves toward target by at most `maxAccelMps2 · dt`.
7. Slip: effective speed `= actual · (1 − slip)`; on slopes, uphill component scales by `max(0, 1 − slopeDeg / maxClimbDeg)` and beyond `maxClimbDeg` the rover slides back down-slope at 0.05 m/s.
8. Kinematics (midpoint integration about the axle centre): `v = (vl + vr)/2`, `ω = (vl − vr)/trackWidthM` (rad/s, + = clockwise from above = + yaw, so left > right turns right, matching the brain's `+turn` is right); `yaw += ω·dt`; axle moves `v·dt` along `yaw` at the midpoint angle; camera = axle + `axleBackM` forward.

Body on terrain (`body.ts`): heights at the four wheel contact points give pitch and roll (least-squares plane); body y = mean contact height + wheel radius. The camera mount transform = body transform · translate(mast) · pitch(−cameraPitchDeg) · Rz(−90°) (portrait). So the brain sees a tilted horizon on slopes, as the phone would.

Collisions (`collision.ts`): footprint = oriented rectangle from (camera forward + `scoopLipM` when the scoop is down, else + 0.05) back to (axle − 0.20), width `bodyWidthM`. Solid obstacles are tested as 2D shapes (circle, OBB, ellipse approximated by its circumscribing ellipse test, capsule). If a step would create or deepen overlap, the pose reverts to the previous step and both actual wheel speeds go to 0 (the rover "bumps"); a move that reduces overlap is allowed so the rover can reverse out. A new contact episode increments `collisions` and emits a `bump` event; the episode ends after 0.5 s without contact.

Drops (`collision.ts`): after each step, for each wheel, `drop = h_prev − h_now` at the wheel's position. If any `drop > fallDropM` (a discontinuity: curb or wall edge; smooth slopes produce millimetres), the rover is `fallen`: `dropOffs += 1`, the rover is disabled (commands ignored, body tilted 30° nose-down at the edge for display), and a banner "Fell off a drop at (x, z)" is shown. Outside `bounds` by more than 0.5 m: `outOfBounds += 1`, rover disabled, banner "Left the site". The harness ends a scenario early on either.

Stuck: commanded |left| + |right| > 20 for 20 s while the camera moved < 0.05 m → `stuck += 1` (once per episode). Informational.

## 14. Sensors

### 14.1 AR pose (`arPose.ts`)

Each controller tick (200 ms) the sim builds `PhoneCamera` from the true camera transform `C_true`:
- `pose = T_ar · D(t) · C_true`, where `D(t)` is accumulated drift: per metre travelled, translation random walk σ = `driftPosPerM` and yaw random walk σ = `driftYawDegPerM`; plus per-sample jitter σ = `poseNoiseM` (position only).
- Tracking loss: Poisson events at `trackingLossPerMin`, duration uniform in [`lossMinMs`, `lossMaxMs`]. During loss `trackingNormal = false` and the pose is frozen at its last value; depth is still attached (ARKit keeps delivering it), detections continue to be generated (the hook skips them when `trackingNormal` is false).
- `timestamp = simTimeMs / 1000`.

### 14.2 LiDAR depth (`lidar.ts`)

At 5 Hz (every controller tick), render 256×192 depth from `C_true` (true geometry relative to the true camera, so AR drift appears only through the pose, like the real phone):
- For pixel (x, y): direction `= R · normalize(unproject(x+0.5, y+0.5, 1, Kd))`; intersect all surfaces; `depth = t · (dir · forward)` (axis depth).
- Range: `t` beyond `lidarRangeM` (5 m) or no hit (sky) → value 0, confidence 0.
- Noise: `depth += N(0, depthNoiseM + depthNoiseRel · depth)`; dropout with probability `depthDropoutP` → value 0, confidence 0.
- Confidence: 2 if depth < 2.5 m, 1 if < 4.0 m, else 0; grazing incidence (|n·dir| < 0.15) lowers by one level.
- Encoding: `Float32Array` → `new Uint8Array(f32.buffer)` (little-endian on all supported platforms; a test asserts `decodeDepth` round-trips), confidence `Uint8Array(w·h)`.
- Leaves enter depth through the stack grid (section 15.1), added to the terrain height: a single leaf is ≤ 2 cm (harmless), a pile is a mound of its real current height, so piles grow and shrink in depth as leaves are dumped and scooped. Grass is not modelled as geometry; its effect is in depth noise.
- Resolution knob: `depthRes: 'full' (256×192) | 'half' (128×96)`; the report harness uses `half` for speed. The brain reads `width`/`height` from the payload, so both are valid shapes.

Raycast performance (`terrain.ts`, `raycast.ts`): heights are precomputed on a 5 cm grid over bounds + 5 m margin with bilinear lookup and a 1 m tile max-height grid. Ray march: when the ray is above the current tile's max, jump to the tile exit; otherwise step `max(0.01 m, 0.5 · (y − h) / |dir.y|)` capped at 0.05 m, then 6 bisection steps on sign change. Flat templates (`terrain.flat === true`) use the analytic plane. Primitives are prefiltered per scan to those intersecting the camera's 6 m frustum bounding box, then tested analytically.

### 14.3 Detector (`detector.ts`)

Runs like the phone's detection loop: one frame in flight at a time. At capture time `t0` it snapshots `PhoneCamera` (with the latest depth), computes detections against the true world at `t0`, and delivers `DetectionResult` at `t0 + latency` where `latency ~ N(latencyMs, latencyJitterMs)` clamped to ≥ 100 ms (ideal: 0 → delivered in the same tick). The next capture starts at delivery. The brain therefore sees stale boxes paired with the camera they were taken with, which is the real situation `locateDetections` is built for.

Candidates per frame:
- Ground leaves at their current `LeafField` position (`state === 'ground'`; `held` leaves under a drag are skipped). Outline: 8 points on the leaf's ellipse at height `y + curl·0.02`, projected through `C_true` and intrinsics → sensor px → upright px; bbox clipped to the frame. Reject if behind camera, < 30 % of bbox inside the frame, or min side < `minBoxPx` (6 px upright). Leaves fully covered by others (stack height above the leaf's own `y` > 2 cm at its centre) are not candidates.
- Clusters: leaves are grouped on the stack grid (connected cells with stack > 4 cm); each cluster yields one box over its outline with a pile label (`"a pile of autumn leaves"`), in addition to its top-layer leaves. Pile boxes larger than 1.5 m are left for the brain to reject (`too_big`).
- Distractors: same projection with `fpWeight` (pine cone 1.0, bark 0.8, brownRock 0.6, stick 0.4).
- Trunk bark: for trunks within 6 m, a box over a random 0.2–0.5 m section between 0.3 and 1.5 m height (LiDAR places these off the ground, so the brain should reject them as `off_ground`).
- Background false positives: `bgFalsePerFrame` (Poisson) random boxes 20–80 px anywhere in the frame (on sky/canopy they'll be unplaceable or too far; on grass they become plausible false leaves).

Visibility: 5 rays (centre + 4 outline points) from the true camera to each candidate, test against primitives and terrain; `visible = unoccludedRays / 5`, require ≥ 0.4. Grass occlusion: with elevation angle α of the line of sight, `grass = clamp((α − 4°) / 10°, 0, 1)` blended by `grassOcclusion` knob (0 = off) — distant leaves at grazing angles hide in grass, as in the frames.

Probability of a hit:
`p = pNear + (pFar − pNear)·min(1, dist / 5) ` × `sizeFactor (clamp(minSidePx / 24, 0.3, 1))` × `contrastFactor (lerp(1 − contrastWeight, 1, contrast))` × `(inShadow ? 1 − shadowPenalty : 1)` × `visible` × `grass`; false positives use `fpRate · fpWeight` instead of `p`.

Score: lognormal with median `scoreMedian` (0.12) and σ 0.5, scaled by `0.7 + 0.6·sizeFactor`, clamped to [0.05, 0.35]; ideal = 0.3 fixed. Emitted only if score ≥ `scoreThreshold` (0.05). Label: uniformly one of the phone's query strings (`"a photo of a dead leaf"`, `"a dry brown leaf"`, `"fallen brown leaves"`, `"a pile of autumn leaves"`; piles prefer the last).

Jitter: each box edge += N(0, `jitterFrac` · box side), centre += N(0, `jitterFrac` · box side); clamp to frame; xmin < xmax enforced.

Each detection internally carries `truth: { kind, id } | null` for metrics and overlays; the adapter strips it before calling the brain (the brain object never sees truth).

Self-view: with `scoopInView` (harsh only) leaves in the raised scoop yield boxes at the bottom of the frame (tests the brain's `selfMaskM`).

### 14.4 Realism presets (`realism.ts`)

| Knob | ideal | realistic | harsh |
|---|---|---|---|
| latencyMs / jitterMs | 0 / 0 | 300 / 80 | 600 / 200 |
| pNear / pFar | 1 / 1 | 0.75 / 0.25 | 0.5 / 0.1 |
| contrastWeight / shadowPenalty / grassOcclusion | 0 / 0 / 0 | 0.6 / 0.3 / 0.7 | 0.8 / 0.5 / 1 |
| fpRate (distractors) / bgFalsePerFrame | 0 / 0 | 0.2 / 0.1 | 0.4 / 0.4 |
| jitterFrac | 0 | 0.04 | 0.08 |
| score | fixed 0.3 | median 0.12 | median 0.09 |
| depthNoiseM / depthNoiseRel / depthDropoutP | 0 / 0 / 0 | 0.005 / 0.01 / 0.02 | 0.01 / 0.02 / 0.08 |
| driftPosPerM / driftYawDegPerM / poseNoiseM | 0 / 0 / 0 | 0.01 / 0.3 / 0.002 | 0.03 / 1.0 / 0.005 |
| AR origin | identity | random yaw at start | random yaw at start |
| trackingLossPerMin / duration ms | 0 | 0.33 / 500–1500 | 1 / 1000–3000 |
| cmdLatencyMs / cmdDropP | 0 / 0 | 80 / 0 | 150 / 0.05 |
| deadbandPct / motorNoise / slip | 0 / 0 / 0 | 15 / 0.05 / 0.03 | 22 / 0.12 / 0.08 |
| scoopInView | false | false | true |

Knob edits apply from the next sensor event. All knobs have slider ranges (0 to 2× the harsh value) enforced in the UI and re-clamped in `realism.ts` (`clampKnobs`), so a bad value from code can't reach the core.

## 15. Scoop and leaf collection (`scoop.ts`)

Scoop state: `up | lowering | down | lifting | dumping`, with angle animated over `lowerMs` / `liftMs` / `dumpMs`. Commands come from `DriveCommand.scoop`; collection only happens in `down` (fully lowered).

Footprint in the rover's horizontal frame (origin = camera ground position, forward = flattened heading): forward ∈ [`scoopLipM − scoopDepthM`, `scoopLipM`], right ∈ [−`scoopWidthM`/2, +`scoopWidthM`/2].

Collection rule, per physics step, for each ground leaf within 1 m:
- Compute the leaf's rover-frame (right, forward) before and after the step.
- Collected iff scoop is `down`, rover forward speed > 0.02 m/s, |right| ≤ width/2 at the crossing, and the leaf crossed the lip line (forward went from > `scoopLipM` to ≤ `scoopLipM`) during this step, and the scoop holds fewer than `scoopCapacity` leaves.
- A leaf that ends up inside the footprint any other way (side entry while pivoting, or reversing) is pushed out by the scoop's side wall (15.2). Not collected.
- Piles need no special rule: a pile is stacked leaves, and every leaf crossing the lip is collected until capacity, so a pass through a pile takes up to 25 leaves and the pile visibly shrinks; several passes clear it. Leaves beyond capacity are pushed ahead of the lip (bulldozed), which is what a full scoop does.

### 15.1 Leaf field (`core/leaves.ts`)

Every leaf is an individual object in a structure-of-arrays `LeafField` (Float32Array `x, z, y, yaw, vx, vz`, Uint8Array `state`, Int32Array `scoopSlot`), sized to the world's leaf count (cap 2500). Two grids sit beside it:
- Spatial hash: 0.25 m cells → leaf ids, updated when a leaf crosses a cell. Used by push, collection, detector culling and picking.
- Stack (litter) grid: 5 cm cells over the bounds, `stack[cell]` = summed leaf thickness. Each ground leaf adds `leafLayerM` (0.02 m: a curled leaf plus air) spread over the cells within its radius, weighted so its total contribution is `leafLayerM × leafArea`. Updated incrementally (subtract at old position, add at new) when a leaf moves more than 1 cm, is scooped or dumped.

Leaf height: `y = terrain(x, z) + max(0, stack(x, z) − leafLayerM) · 0.9 + 0.003`, order-independent, so a stacked pile renders as a mound of individual leaves. The raycaster adds the stack grid on top of the terrain heightfield (tile max-heights updated with it), so LiDAR sees piles at their real current size.

Leaf physics per 10 ms step, only for "awake" leaves (touched in the last 1 s or with speed > 1 mm/s; everything else is skipped, which keeps 2500 leaves cheap):
- Slide: `x += vx·dt`, `z += vz·dt`; velocity decays with ground friction `v *= exp(−dt / 0.12 s)`; stop below 1 mm/s.
- Solid obstacles: a leaf whose centre enters an obstacle footprint is projected out to its boundary and its normal velocity zeroed.
- Drops: a leaf that slides over a drop edge falls to the lower level (its height comes from the terrain), no special casing.
- Bounds: clamped 0.2 m inside the world bounds.
- No leaf-leaf collisions; overlap is what stacking is.

### 15.2 Pushing (`core/leaves.ts`, called by `scoop.ts`)

Pushers are 2D shapes in the rover frame, tested against awake-or-nearby leaves from the spatial hash (within 0.6 m of the rover):
- Wheels: four rectangles `wheelWidthM` × `2·wheelRadiusM`, always.
- Scoop side walls: two thin rectangles along the footprint's sides, when the scoop is lowering, down or lifting.
- Scoop front when up or body front: a rectangle across the front of the body and raised scoop, only where `stack` at the leaf exceeds `chassisClearanceM` (0.05 m), i.e. piles taller than the clearance get bulldozed, single leaves pass under.
- Full scoop: the lip line itself pushes leaves crossing it once capacity is reached.

A leaf inside a pusher is moved out along the shortest exit direction (+1 cm), gets velocity `0.8 ×` the pusher's point velocity along that direction plus a small random yaw kick (seeded `leaves` stream), and is marked awake. Pushing never deletes or collects.

### 15.3 Carry, dump and pile-up

Collected leaves get `state = 'scoop'` and a slot index; while carried, their pose is derived each frame from the scoop's transform (slots on a 5 × 5 grid inside the scoop bowl, stacked by slot), so they visibly ride along. Leaves never leave the `LeafField`.

Dump: when the scoop reaches the `dumping` angle, carried leaves are released one by one over `dumpMs`: each is placed at the lip position plus a Gaussian offset (σ 0.07 m forward, 0.05 m lateral), with a forward velocity of 0.15 m/s, `state = 'ground'`, and added to the stack grid. Dumping repeatedly at the pile spot therefore grows a visible mound; leaves dumped elsewhere just lie where they fall and can be collected again.

Gathered into pile: a ground leaf whose centre is inside the pile spot circle. Computed from current positions each metrics update, so leaves dragged into the spot count too (that's a user edit; the readout shows it). The pile spot is in sim world coordinates; the controller receives it in AR coordinates (`T_ar`) and is updated whenever it changes.

Metrics: `collected` = unique leaf ids that have ever entered the scoop; `gathered` = leaves in the pile spot now; `dumps` = completed dump actions with at least one leaf; `total` = leaf count. The readout shows load (n / 25), `collected / total (%)`, `gathered / total (%)` and dumps.

### 15.4 User edits

`Simulation.queueEdit(edit)` with `edit` one of `{ kind: 'grab', ids: number[] }`, `{ kind: 'move', ids, dx, dz }` (or absolute `{ kind: 'place', id, x, z }`), `{ kind: 'release', ids }`, `{ kind: 'setPileSpot', x, z, radiusM }`. Edits are applied at the start of the next physics step, never mid-step. Grabbed leaves get `state = 'held'`: removed from the stack grid and ignored by pushers, scoop and detector while held (the user's hand is "over" them); on release they get `state = 'ground'` at the drop point (clamped to bounds, pushed out of solid obstacles), are re-added to the stack grid and marked awake. Grabbing a leaf that is in the scoop is not allowed (picking ignores scoop leaves). `setPileSpot` validates: centre inside bounds, radius 0.3–2.0 m, not on a solid obstacle; on failure the edit is rejected and the UI shows "Pile spot must be on open ground". The controller is told the new spot via `controller.setPileSpot(pileAR)` in the same step.

Scoop command fallback (DP4): if the controller's command lacks `scoop`, the sim uses `scoopMode = 'passive'`: the scoop is permanently down (a dustpan plough), so forward entry still collects. The UI shows "scoop: passive (controller sends no scoop command)".

## 16. Simulation loop (`sim.ts`)

```ts
class Simulation {
  constructor(world: World, opts: { rover: RoverConfig; realism: RealismKnobs; seed: number })
  readonly world: World; timeMs: number; state: SimState   // rover pose, wheels, scoop, leaves, metrics, events
  setController(c: Controller): void
  setRealism(k: RealismKnobs): void
  stepPhysics(): void          // advance 10 ms
  advance(ms: number): void    // repeated stepPhysics
  snapshot(): SimSnapshot      // render/UI copy, cheap (typed arrays reused)
}
```

Per 10 ms step, in order: (0) apply queued user edits; (1) if a detection result is due, call `controller.onDetections(result, nowMs)`; (2) at multiples of 200 ms: build `PhoneCamera` (pose + depth), call `controller.onTick(camera, nowMs)` and enqueue its command; (3) start the next detection capture if none is in flight; (4) drive pipeline + kinematics; (5) collisions, drops, bounds; (6) scoop state, collection, pushing; (7) leaf physics for awake leaves, stack-grid updates; (8) dump release; (9) metrics, trail point every 100 ms. Delivery-before-tick ordering matches the phone, where detection effects land between steering ticks.

Controller interface (`controllers/types.ts`):

```ts
type ControllerStatus = { kind: 'ready' } | { kind: 'loading' } | { kind: 'error'; message: string; detail?: string; fatal: boolean }
type ControllerDebug = {
  mode?: string; message?: string; load?: number
  leafMap?: { id: number; world: [number, number, number]; state: 'tentative' | 'confirmed' | 'collected' | 'failed' }[]   // AR coords
  corridors?: { bearingDeg: number; clearanceM: number; blockedBy: 'obstacle' | 'drop' | null }[]
  targetWorld?: [number, number, number]   // AR coords
  captions?: (string | undefined)[]        // per detection of the last result, like the phone HUD
  floorY?: number | null
}
interface Controller {
  readonly id: 'rover-brain' | 'naive' | 'manual'
  reset(ctx: { nowMs: number; pileAR: { center: [number, number, number]; radiusM: number } | null; scoopCapacity: number }): void
  setPileSpot(pileAR: { center: [number, number, number]; radiusM: number } | null): void
  start(nowMs: number): void
  onDetections(result: DetectionResult, nowMs: number): void
  onTick(camera: PhoneCamera, nowMs: number, rover: { scoopLoad: number }): DriveCommand
  status(): ControllerStatus
  debug(): ControllerDebug
}
```

Every controller call from the sim is wrapped in try/catch (second line of defence after the adapter's own guards): a throw yields `{ left: 0, right: 0 }` for that tick and an error status.

Browser loop (`useSimulation.ts`): `requestAnimationFrame`; `acc += realDt · speed`; run `stepPhysics()` while `acc ≥ 10` and work < 12 ms this frame; leftover is dropped (not carried) when over budget, and `actualSpeed` is measured over 1 s and shown. Step button advances exactly 200 ms. The React snapshot is published at most every 100 ms through `useSyncExternalStore`; the renderer reads the live `Simulation` each frame directly, outside React.

## 17. Brain adapter (`brainAdapter.ts`)

Contract. The adapter is the phone app's role: it holds the brain, calls it with phone-shaped data, and returns drive commands. It provides to the brain exactly what `use-rover-brain.ts` and `brain.ts` consume:
- per detection result: `result.detections` (upright px boxes), `result.frameSize` (576×768), `result.camera` (`CameraSnapshot`), `floorY`;
- per 200 ms tick: the current `CameraSnapshot` with depth (`currentCamera(true)` equivalent), `nowMs`;
- once at reset: the drop zone as `Pile { center: Vec3 (AR), radiusM }`.

It calls, preferred path (`Brain` exported):

```ts
reset:        brain = new rb.Brain(); brain.setPile(pileAR)   // the committed autopilot halts with "Scoop full: set a pile" without one
setPileSpot:  brain.setPile(pileAR)                     // live, when the user moves the pile spot
start:        brain.start(nowMs)                        // on first Play after reset
onDetections: if (!isPose(cam.pose) || !cam.trackingNormal) return
              heading = rb.headingFromPose(cam.pose)
              floorY  = brain.obstacles.floorY ?? (heading ? heading.origin[1] - rb.DEFAULT_ROVER.cameraHeightM : null)
              { leaves, rejected } = rb.locateDetections(stripTruth(result.detections), result.frameSize, cam, floorY)
              brain.observe(leaves, heading, nowMs); captions from leaves/rejected (same text as the phone)
onTick:       scan = rb.scanDepth(cam); if (scan) brain.addScan(scan, nowMs)
              heading = cam.trackingNormal && rb.isPose(cam.pose) ? rb.headingFromPose(cam.pose) : null
              t = brain.tick(nowMs, heading)
              return t.step ? { left: t.step.left, right: t.step.right, scoop: t.step.scoop } : { left: 0, right: 0, scoop: 'up' }
debug:        brain.leaves.snapshot(), t.corridors, brain.autopilot.mode/load, t.step?.message, target = leaf with id brain.autopilot.targetId
```

Fallback path (no `Brain` export; mirrors today's `use-rover-brain.ts`): `LeafMap`, `ObstacleMemory`, `corridors`, `initialAutopilotState`, `stepAutopilot(state, { nowMs, heading, leaves, corridors, scanAgeMs, pile })`; apply `collectedIds ?? (collectedId != null ? [collectedId] : [])` via `markCollected(id, nowMs)`, plus `missingId` / `failedId` when present and `markFailed` exists. Scoop from `step.scoop` if present, else omitted (passive scoop, DP4).

Load and validation (`loadBrainModule(): Promise<BrainModuleResult>` and `createBrainController(mod)`, the latter taking the module object so tests can inject fakes):
- Import fails (syntax error, missing file): status `{ kind: 'error', fatal: true, message: 'Could not load rover-brain: <err.message>' }`; commands are (0, 0); `console.error` once with the full error. UI offers "Retry".
- Required exports missing (`locateDetections`, `scanDepth`, `headingFromPose`, `isPose`, `DEFAULT_ROVER`, and either `Brain` or all of `LeafMap`, `ObstacleMemory`, `corridors`, `stepAutopilot`, `initialAutopilotState`): fatal error listing the missing names.
- Runtime throw in any call: caught; that call returns (0, 0) (or is skipped for `onDetections`); status `{ kind: 'error', fatal: false, message, detail: first stack line }`; `console.error` once per distinct message; after 5 consecutive failing ticks the status becomes fatal ("Brain crashed: <message>") and 17.2 applies (swap to naive in the UI; recorded as a failure in the report).
- Output validation: `left`/`right` not finite → 0 with a warning in status; values clamped to [−100, 100] by the drive pipeline; `scoop` not in {up, down, dump} → treated as absent.
- Successful tick after non-fatal errors clears the status to `ready`.

The adapter never passes truth fields and never reads the sim world. Its only knowledge of the sim is the data shapes above, the pile spot (which the phone user sets with "Set pile"), and, for the shim only, the scoop load (17.1).

### 17.1 Carry/dump contract and the dump-trip shim

The committed brain (b0faf113) has the full pile-up cycle: `approaching → aligning → scooping → lifting`, `load` counted in scoop loads, `returning` to the pile once `load ≥ capacity` (default 1 load), `dumping` (scoop `'dump'`), `backing`, then searching again; leaves inside the pile radius are never targets. So with the preferred path the brain drives the whole trip and the sim just executes `scoop` commands. Note the unit difference: the brain counts loads (passes), the sim counts leaves (capacity 25); with capacity 1 the brain dumps after every pass, which the sim handles correctly.

The adapter detects whether the brain does its own dumping: `carryDump = 'brain'` if `step.scoop` is ever present and `'dump'` is a value the brain can return (checked once: the module exports `DEFAULT_AUTOPILOT.capacity` and `stepAutopilot` returns a `scoop` field on the first tick), else `'shim'`. In shim mode (fallback raw-function path, an older or future brain without carry/dump, and the naive controller), `controllers/dumpTrip.ts` takes over driving when the scoop holds ≥ `scoopCapacity` leaves, or holds any and the controller has had no target for 10 s:
1. Turn toward the pile spot (AR coords, from the camera pose), pivot at 30 % until |bearing| < 10°.
2. Drive at 45 % with proportional steering until the camera is `dumpAtM` (0.6 m, the brain's value) from the spot's centre; uses the brain's corridors if available (stops and pivots 30° if bearing 0 is blocked under 0.45 m), otherwise none.
3. Stop, command `scoop: 'dump'` for `dumpMs`, then reverse 0.35 m at 28 %, then hand control back.
While the shim drives, the brain still receives `observe`/`addScan`/`tick` (its output is ignored) so its maps stay current. The UI shows "Dump trip: sim shim (brain has no carry/dump)" and the report column "shim used" is set. This is a contract gap to tell the brain owner about, not a sim feature to keep: the shim is only there so pile-up can be demonstrated with any controller. The scoop load it reads is sim knowledge the phone doesn't have (the real rover has no scoop-load sensor); this is stated in the UI tooltip and in DP11.

### 17.2 Automatic fallback in the default sample

When the selected controller is `rover-brain` and its status becomes fatal (import failure, missing exports, or 5 consecutive failing ticks), `useSimulation` swaps in `naive` (with the dump-trip shim) at the next step boundary, keeps the sim playing, and shows an amber notice "Rover brain unavailable: <message>. Running naive controller." with "Retry brain" (re-imports, and on success switches back after a controller reset). Non-fatal errors don't trigger the swap. The fallback is only automatic; choosing `rover-brain` again manually retries. In the headless report, there is no fallback: brain failures are recorded as such.

Built-in controllers:
- `naive.ts`: uses sim `math.ts` only. On each detection result, places every box's centre by floor ray at the sim's assumed floor (camera height) in AR coords, ignores points inside the pile spot, keeps the highest-score point within 4 m as target; on tick, pivots toward the target if |bearing| > 15°, else arcs forward at 45 % with scoop `down` within 0.6 m; drops the target 1.5 s after the lip passes it; searches by pivoting at 30 % when no target. Carry and dump via `dumpTrip.ts`. Ignores obstacles (it's a baseline). Never throws.
- `manual.ts`: key state from a window listener (ignores text inputs, like `useRover`): W (60, 60), S (−40, −40), A (−35, 35), D (35, −35), W+A (30, 60), W+D (60, 30); Q toggles scoop down / up; E dumps; commands are resent every tick while held, (0, 0) when nothing is held. Window blur clears keys.

## 18. Rendering (`render/`)

`SimRenderer` owns a `WebGLRenderer` (antialias, `shadowMap` PCFSoft, 2048 shadow map for the sun), a `Scene`, a view `PerspectiveCamera` (60°), and `OrbitControls`. Camera modes: `orbit` (OrbitControls around the rover's start or user target) and `follow` (chase cam 1.6 m behind, 1.0 m up, lerped 0.1 per frame, looking 1 m ahead of the rover). Lights: `DirectionalLight` sun at `world.sun`, `HemisphereLight` sky #bcd8ff / ground #4a6b2a.

- Ground: `PlaneGeometry` over bounds + 5 m margin, 10 cm segments, vertices displaced by the height grid; vertex colours from surface + noise; canvas-generated detail textures (256² grass, asphalt, soil) repeated. Curbs and walls get explicit side faces (boxes) so steps look like steps.
- Leaves: `leafMeshes.ts` builds 3 geometries (5-lobe maple outline via `Shape` → `ShapeGeometry` with 24 segments; oval; narrow) and bends them with a curl displacement (`y += curl·0.02·(r/R)²` plus a lobe-tip lift). One `InstancedMesh` per shape, capacity = leaf count, `instanceColor` from the palette (#C9A66B tan, #A0703C light brown, #7A4A26 brown, #C8702A orange, #D9B44A yellow, #6E2E22 dark red-brown), double-sided `MeshStandardMaterial` roughness 0.9. No leaf is ever hidden: in-scoop leaves are drawn in the scoop, dumped ones on the ground.
- Leaf transforms: each frame, `SimRenderer` reads the `LeafField`'s dirty list (leaves that moved, were scooped, ride in the scoop, or were dumped) and rewrites only those instance matrices (`setMatrixAt`, then `instanceMatrix.needsUpdate` with `addUpdateRange` over the touched span); scoop leaves are posed from the scoop slot transform every frame. A tilt from the local stack-grid gradient makes leaves on a pile lie on its slope.
- Piles: no separate mesh; a pile is its leaves. Under dense clusters (stack > 4 cm) a low leaf-brown "litter" mesh, rebuilt from the stack grid at most twice a second, fills gaps so piles read as solid mounds.
- Obstacles: trunks (`CylinderGeometry`, bark colour #5b4a3a with vertical-stripe texture), rocks (`IcosahedronGeometry(1, 2)` with seeded vertex jitter, grey-brown), logs (`CapsuleGeometry`), fence (posts + rails), wall (box, concrete grey). Distractors: pine cone (`ConeGeometry` with jittered scales, #6b4a2b), bark chips (thin boxes #4a3526), brown rocks, sticks (thin capsules). Reference backdrop: a ring of trunks and low-poly canopy blobs (yellow/orange/green) beyond bounds, not in the raycaster beyond 8 m (visual only).
- Pile spot: a flat ring (`RingGeometry`) in tarp blue at ground level plus a small flag; draggable (below).
- Interaction (`interaction.ts`): pointer events on the canvas. `pointerdown` raycasts (three `Raycaster`) against the three leaf `InstancedMesh`es (`instanceId` → leaf id), then the pile-spot marker; leaves in the scoop are skipped. A hit on a leaf disables `OrbitControls`, captures the pointer, and queues `grab` for that leaf, or with Shift for all ground leaves within 0.35 m of it (spatial hash). `pointermove` intersects the pointer ray with the ground (terrain mesh) and queues `move` deltas (throttled to one per animation frame); held leaves render 3 cm above the ground under the cursor. `pointerup` / `pointercancel` / Escape queue `release`. A hit on the marker drags the pile spot (queued `setPileSpot` on release). "Set pile spot" button in the controls arms a one-shot click on the ground. Cursor: `grab` when hovering a leaf, `grabbing` while dragging. A miss passes the event to `OrbitControls`. Keyboard alternative for accessibility is out of scope for dragging (pile spot can also be set by numeric X/Z fields in the controls panel).
- Rover (`roverModel.ts`): body box 0.42×0.10×0.34 dark grey, 4 black wheels (`CylinderGeometry` r 0.06, w 0.04) spun by wheel speed, front scoop = open box shell 0.30 wide hinged at the front of the body, rotation: up +20°, down −35° (lip on the ground at `scoopLipM` ahead of the camera), dump +100°; mast thin cylinder up to `cameraHeightM`; phone box 0.072×0.147×0.008 in portrait, tilted `cameraPitchDeg`, lens dot on the back.
- Overlays (`overlays.ts`): FOV cone = 4 lines from the true camera to the image corners at 5 m; LiDAR points = `Points` from the last depth map lifted with sim math (stride 2), coloured by height above ground (green ground, red > 8 cm, blue < −10 cm); corridors = 13 line segments from the rover at each bearing, length = clearance (cap 5 m), red if blocked, `blockedBy: 'drop'` dashed; trail = `Line` of the last 3000 positions; target line from rover to `debug.targetWorld` (via `T_ar⁻¹`); leaf map = small rings at brain leaf positions (`T_ar⁻¹`) coloured tentative #facc15, confirmed #22c55e, collected #71717a, failed #ef4444. Overlay colours are paired with legend text in the controls panel.
- Phone view (`phoneView.ts`): a second camera rendered into a 288×384 render target each frame (only when the inset is shown). Its world matrix is the true camera pose times Rz(+90°) (undoing the portrait roll so three.js "up" is upright), and its projection matrix is built from the upright intrinsics (`fx_u = fy·0.4`, `fy_u = fx·0.4`, `cx_u = (1440 − cy)·0.4`, `cy_u = cx·0.4` on 576×768) with near 0.05, far 60. `PhoneInset.tsx` draws the target on a canvas and an SVG layer with the last delivered boxes (scaled from 576×768), coloured by the adapter's caption (located = green with "1.2 m · 15° R", rejected = red with the reason, matching the phone HUD).

WebGL failure (context creation throws or `webglcontextlost`): `SimView` shows "3D view unavailable (WebGL): <message>"; the core sim and readout keep running.

## 19. UI integration

`App.tsx` state changes from `scene: SceneSource` to:

```ts
type SceneMode = { kind: 'splat'; scene: SceneSource } | { kind: 'template'; templateId: TemplateId; seed: number }
```

Initial state: `{ kind: 'template', templateId: 'sample', seed: 1 }`, controller `rover-brain`, preset `realistic`, follow camera, overlays on: phone view, FOV cone, target line, brain leaf map, trail; playing at 1× as soon as the world is built and the brain import has settled (ready or fallen back to naive). This replaces today's empty splat viewer as the landing view; `Splat (file)` is one dropdown choice away and keeps whatever default the splat task sets (`leaf-floor.splat`). The main column renders `SplatViewer` (unchanged props) when `kind === 'splat'` and `SimView` otherwise. Under the viewer, the existing row (`ScenePanel` + `CapturePanel`) stays; in template mode a second row shows `SimControlsPanel` and `RealismPanel`. `SimReadout` and `PhoneInset` are overlays inside `SimView` (readout top-left, inset bottom-left, 180×240 CSS px).

`ScenePanel.tsx`: new props `mode: SceneMode`, `onModeChange`. A labelled `<select>` "Source" at the top: "Sample: flat ground + leaves" (first, default), "Splat (file)", then the other nine templates. In splat mode the existing buttons, file input, URL form and help text render exactly as the splat task leaves them (calls go through to `onModeChange({ kind: 'splat', scene })`). In template mode: the template description (one sentence), a labelled number `<input>` "Seed" (integer 0–2147483647; invalid or empty input gets a red border and `aria-invalid`, is not applied, and the last valid seed stays in use; applied on Enter or blur), a "Reroll" button (UI picks a random seed with `Math.random`; the core never does), and a hint line "Drag leaves to move them · Shift-drag moves a cluster · drag the blue ring to move the pile spot". The header label shows the template name and seed.

`SimControlsPanel.tsx`: Play/Pause toggle, Step (200 ms), Reset; speed `<select>` (0.25, 0.5, 1, 2, 4, 8); controller `<select>` (Rover brain, Naive go-to-nearest, Manual WASD) with status line (ready / loading / error text and Retry); camera radio (Follow / Orbit); overlay checkboxes (FOV cone, Phone view, LiDAR points, Corridors, Trail, Target line, Brain leaf map) with colour legend; pile spot: "Set pile spot" (arms a ground click), numeric X / Z / radius fields (validated as in 15.4), "Reset to template default"; scoop capacity (5–50 leaves, applies on Reset); rover mount (camera height, pitch; "applies on Reset"); brain config mismatch note.

`RealismPanel.tsx`: preset `<select>` (ideal, realistic, harsh, custom) and grouped sliders (Detector, LiDAR, Pose, Drive) with numeric value labels. Default preset: realistic.

`SimReadout.tsx`: controller + status (incl. fallback notice and "Dump trip: sim shim" when active); brain mode and message; `Scoop 3 / 25`; `Collected 12 / 60 (20 %) · In pile 9 / 60 (15 %) · Dumps 4`; `Collisions 1 · Drop-offs 0 · Stuck 0`; `t 01:23.4 · 14.2 m · 1.0× (actual 0.9×)`; brain leaf counts (tentative / confirmed / collected / failed); banners for fallen / out of bounds / brain crashed.

Template change or seed change: regenerate world, reset sim, reset controller, paused (the initial load is the one exception: it autoplays). User leaf drags and pile-spot moves are kept until Reset or a template/seed change, which restore the generated layout. Controller change: reset controller only (world and rover stay), paused. Reset: rebuild rover + world state from the same world object (leaves restored), new brain instance (picks up the latest brain code).

`useRover.ts`: `useRover({ keyboard: boolean } = { keyboard: true })`; the keydown/keyup effect is skipped when `keyboard` is false. `App` passes `keyboard: !(mode.kind === 'template' && controller === 'manual')`, so WASD drives the sim rover instead of the real one during manual sim control. The DPad and Stop button still drive the real rover.

## 20. Error handling summary

| Operation | Failure | Recoverable? | Caller gets | Logged |
|---|---|---|---|---|
| `generateWorld` | template bug / validation | yes (keep previous world) | throws `Error`; ScenePanel shows message | `console.error` |
| seed input | non-integer / out of range | yes | not applied, field marked invalid | no |
| brain import | syntax/link/missing file | yes (Retry, Reset) | status error fatal, (0,0) commands; UI swaps to naive (17.2) | `console.error` once |
| brain exports | missing names | yes (after brain fix + Retry) | status error fatal listing names; UI swaps to naive | `console.error` once |
| brain call | throws | yes; fatal after 5 consecutive ticks | (0,0) for that tick; UI swaps to naive when fatal | `console.error` once per distinct message |
| leaf drag / edit | target leaf in scoop, drop inside obstacle, outside bounds | yes | grab ignored / leaf pushed to nearest valid spot | no |
| pile spot edit | outside bounds, radius out of 0.3–2.0, on obstacle | yes | edit rejected, previous spot kept, inline message | no |
| brain output | NaN / bad scoop | yes | sanitised command + warning in status | `console.warn` once |
| controller command loss | latency / drop / no command | n/a | watchdog stops motors after 500 ms | metrics `watchdogStops` |
| WebGL | no context / lost | no for rendering | SimView error card; sim continues | `console.error` |
| sim over budget | too slow for speed | yes | lower actual speed shown | no |
| rover falls / leaves bounds | physics | yes via Reset | rover disabled, banner, metric | no |
| report run | sim throws | no | test fails (non-zero exit) | vitest output |
| report run | brain bad / throws | yes | recorded in report row, run continues | report file |

Invariant ownership:
- The brain never sees ground truth: enforced in the adapter (`stripTruth`, no world access) because it's the only boundary between sim and brain; tested with a spy module.
- Sensor shapes match the phone: owned by `core/sensors` and `core/math`, verified by env tests against brain functions.
- Leaf-count conservation (ground + scoop + held = total; leaves are relocated, never deleted): owned by `LeafField`, the only code that changes leaf `state`; asserted after every step in tests (`assertConserved`).
- Stack grid consistent with leaf positions: owned by `LeafField` (incremental updates on every move); a test rebuilds it from scratch after a run and compares within 1e-6.
- Determinism: owned by `core` (RNG streams, fixed step); tested.
- Knob ranges: UI sliders limit, `clampKnobs` in core re-enforces.

## 21. Testing

Unit (vitest, Node, `src/sim/**/*.test.ts`, must pass under `npm run test`):
- `core/math.test.ts`: portrait pose columns; `toUpright(toSensor(p)) = p`; equality with `rb.uprightToSensor`, `rb.headingFromPose` (yaw within 1e-9), `rb.unproject`, `rb.scaleIntrinsics` for 200 random inputs; both pose conventions.
- `sensors/detector.test.ts` (ideal): leaf at (0.4, 0, −1.5) with camera at (0, 0.3, 0) pitch 20 yaw 30 → one box; `rb.locateDetections` with sim depth → world within 0.03 m, `source: 'lidar'`; without depth, floor ray within 0.03 m; leaf straight ahead → box centre u within 2 px of 288, v > 384; leaf right of centre → u > 288; trunk bark box → `rejected` with `off_ground`; latency: result delivered ≥ latency after capture, camera timestamp = capture time.
- `sensors/lidar.test.ts`: flat floor → `rb.decodeDepth` OK, `rb.scanDepth` points y within 0.01, `floorSource: 'lidar'`, `floorY` within 0.01 of 0; wall box 2 m ahead → `rb.corridors(...)[bearing 0].clearanceM` within 0.1 of 2.0 − (camera-to-wall offset); curb 0.15 m down at 1.5 m → `blockedBy: 'drop'`; beyond 5 m → confidence 0; byte length = 4·w·h.
- `rover/drive.test.ts`: (50, 50) for 2 s → distance ≈ 0.3·2 − ramp loss within 2 %, yaw unchanged; (40, −40) → yaw increases (turns right), camera moves on a circle of radius `axleBackM`; deadband 15 → (10, 10) gives no motion; latency 80 ms → first motion after 80 ms; single command then silence → wheels target 0 at 500 ms, speed 0 by 500 + decel time.
- `rover/scoop.test.ts`: front entry collects; scoop up doesn't; pivot sweep doesn't (leaf pushed out by the side wall, still on the ground); reverse doesn't; capacity respected (26th leaf is pushed ahead, not collected); pile cluster of 60: three passes, leaf count in the cluster strictly decreasing, cluster max stack height decreasing; dump at the pile spot → carried leaves on the ground inside the spot, `gathered` up by that number, stack height there increased; conservation every step.
- `core/leaves.test.ts`: wheel pushes a leaf in its path aside (moved, not collected, velocity decays to 0 within 1 s); a leaf pushed into a rock ends outside it; a pile taller than chassis clearance is bulldozed by the body front, single leaves pass under; stack grid rebuilt from scratch equals the incremental one; `grab`/`move`/`release` edits move a leaf, exclude it from detection while held, and re-add it to the stack grid; Shift-cluster grab selects exactly the leaves within 0.35 m; the detector sees a moved leaf at its new position on the next capture.
- `pileup.test.ts` (milestone-1 end-to-end, must pass): `sample` seed 1, `ideal`, naive controller with the dump-trip shim, 180 sim-s → ≥ 1 dump and ≥ 5 leaves gathered in the pile spot, no leaf lost; `setPileSpot` mid-run moves where the next dump lands.
- `rover/collision.test.ts`: drive into wall → `collisions = 1`, no penetration, reversing frees; drive off curb → `dropOffs = 1`, rover disabled; leave bounds → `outOfBounds = 1`.
- `world/generate.test.ts`: every template validates for seeds 1–20; same seed → identical JSON hash; different seeds → different hash; counts within the table's ranges.
- `sim.test.ts`: 60 s naive run on `obstacles` seed 7 realistic, twice → identical metrics and trajectory hash; leaf conservation every step.
- `brainAdapter.test.ts`: fake module missing `locateDetections` → fatal status, (0, 0); fake `Brain.tick` throwing → (0, 0), non-fatal, fatal after 5; NaN output sanitised; spy confirms detections passed to the brain have no `truth` field; real `@rover-brain` loads and ticks once on `empty` without throwing (if the brain is mid-edit and broken this one fails, which is the signal we want).

Report (`npm run sim:report` = `vitest --run --config vitest.report.config.ts`, which merges `vite.config.ts` and sets `include: ['src/**/*.report.ts']`, `testTimeout: 600_000`): `report/scenarios.report.ts` runs every template × seeds (`SIM_SEEDS` env, default `1`) × preset `realistic` (`SIM_PRESETS` env to add others) for 180 sim-s (`SIM_SECONDS`) with the rover-brain controller and `depthRes: 'half'`. It records, per run: collected %, gathered into pile (count and %), dumps, whether the dump-trip shim was used, collisions, drop-offs, out of bounds, stuck, watchdog stops, brain errors (count + first message), time to first scoop, confirmed brain leaves with no true leaf within 0.25 m (false confirmations), and sim ms per sim-s. It prints a markdown table, writes `frontend/sim-report/latest.json` and `latest.md` (gitignored), and uses no assertions on brain metrics; the test fails only if the sim throws.

Not unit-testable: rendering and UI (manual check: acceptance criteria 1–9 in the browser via `npm run dev`). `npm run build` and `npm run lint` cover type and lint correctness.

## 22. Decision points (brain API may not fit yet)

- DP1 Brain entry point. `Brain` class exists now (`observe`, `addScan`, `setPile`, `start`, `tick`), while the phone hook still uses raw functions. Recommendation: prefer `Brain`, fall back to composing raw functions; feature-detect at load. Our adapter mirrors the phone hook's perceive step (`locateDetections` → `observe`) because `Brain.observe` takes already-located observations.
- DP2 `erasableSyntaxOnly`. The brain uses parameter properties; recommendation: turn the option off in the frontend's app/test tsconfigs (verified this is the only blocker today).
- DP3 Pose roll. The brain's tests and `sim.ts` use landscape-style `makePose` with portrait-shaped intrinsics; the phone emits portrait-rolled poses with landscape sensor intrinsics. Recommendation: the sim emits the phone's form; tests show the brain gives the same answers either way. If the brain later relies on camera x being horizontal, the sim exposes it (that's a real-phone bug).
- DP4 Scoop command. The committed autopilot returns `scoop` (`up`/`down`/`dump`); the phone's firmware ignores it today (no scoop hardware), the sim executes it. A brain without it gets the passive scoop (always down), shown in the UI.
- DP5 Pile spot. The brain's `Pile` is user-set on the phone ("Set pile" puts it 1.5 m ahead); the committed autopilot halts with "Scoop full: set a pile" without one. The sim always sets one (template default, user-movable) via `setPile` in AR coords, updated live.
- DP6 Detection timing. Real detections arrive late with the camera of their capture frame; the sim does the same rather than handing the brain fresh boxes.
- DP7 Pile height vs obstacle threshold. Piles 8–20 cm tall exceed the brain's `maxStepM` (0.08 m) and look like obstacles in depth. Recommendation: keep physical truth; the report shows the effect so the brain agent can decide (e.g. treat detected-pile regions as soft).
- DP8 Mount and scoop geometry. Sim defaults match `DEFAULT_ROVER.cameraHeightM` (0.30) and `DEFAULT_AUTOPILOT.scoopLipM` / `scoopWidthM` (0.15 / 0.30); mismatches are flagged in the UI, never silently synced, because the real rover's geometry is the source of truth.
- DP9 Climbable step. Sim 0.05 m vs brain 0.08 m: 5–8 cm objects (some rocks, logs' ends) collide physically though the brain thinks they're passable. Kept as realism; configurable in `RoverConfig`.
- DP10 The brain's own `RoverSim` (`sim.ts`) feeds ground-truth-derived observations and skips `perceive`/depth. We don't use it; our sim exercises the full sensor path. No attempt to share world types with it.
- DP11 Carry/dump ownership. The committed brain owns the dump trip (returning/dumping/backing). The sim's `dumpTrip.ts` shim exists only for controllers without it (fallback raw path, naive, a future brain that drops it) and is flagged in UI and report as a contract gap. It triggers on the sim's true scoop load, which the real phone can't measure; if the brain owner wants a load signal, the contract would need a new input (e.g. a scoop-full flag from a sensor), and that's their decision.
- DP12 Load units. Brain `capacity` counts scoop passes (default 1); the sim's `scoopCapacity` counts leaves (25). No conversion: the brain decides when to dump, the sim decides how many leaves physically fit.

## 23. Build order (for the planner)

### Milestone 1: the default sample, end to end (ship first)

Goal: acceptance criteria 0, 0a, 0b, 0c, 1, 10 (a, b, c, the flat-floor part of d, f, f2, h, i) and 12 on the `sample` template. Realism uses fixed `ideal` and `realistic` presets in code (no panel yet). Flat terrain only (analytic plane + stack grid), no obstacles, no drops.

1. Tooling: vitest 5.0.5 devDep + `test` / `sim:report` scripts, `vite.config.ts` alias / fs / test (merged into whatever the splat task left), tsconfig changes, `tsconfig.test.json`, `.gitignore`. Smoke test importing `@rover-brain`.
2. `core/rng.ts`, `core/math.ts`, `core/types.ts` + `math.test.ts`.
3. `core/leaves.ts` (LeafField, spatial hash, stack grid, push/slide, edits) + `leaves.test.ts`.
4. `core/terrain.ts` (flat + stack grid), `core/raycast.ts` (plane + stack heightfield), `sensors/lidar.ts` + `lidar.test.ts` (floor, pile mound).
5. `sensors/arPose.ts`, `sensors/detector.ts` (leaves + clusters), `realism.ts` (ideal, realistic) + `detector.test.ts`.
6. `world/generate.ts` + `sample` template + validation.
7. `rover/config.ts`, `drive.ts`, `body.ts` (flat), `scoop.ts` (collect, push, carry, dump) + tests.
8. `sim.ts`, `metrics.ts`, `controllers/dumpTrip.ts`, `controllers/naive.ts` + `pileup.test.ts`, `sim.test.ts` (determinism).
9. `brainAdapter.ts` (Brain path, carry/dump detection, guards) + `brainAdapter.test.ts`; `useSimulation.ts` with auto-fallback (17.2).
10. `render/`: ground, leaf instancing with dirty updates, rover model + scoop animation, pile-spot marker, litter mesh, phone view, FOV cone, target line, leaf-map markers, trail; `interaction.ts` (leaf / cluster / pile-spot drag).
11. `ui/`: `SimView`, `PhoneInset`, `SimReadout`, minimal `SimControlsPanel` (play/pause/step/reset, speed, controller select rover-brain / naive, pile-spot controls); `ScenePanel` Source dropdown with Sample + Splat (file); `App` default = sample, autoplay.
12. Verify M1: `npm run test`, `npm run lint`, `npm run build`, browser check of 0–0c and 1, `git status` clean under `apps/expo-app`.

### Milestone 2: the rest

13. Heightfield terrain + raymarch, primitives, collisions, drops, bounds, slopes (`body.ts` pitch/roll) + `collision.test.ts`, remaining `lidar.test.ts` (wall, curb).
14. Remaining nine templates + `generate.test.ts`.
15. Distractors and bark false positives in the detector, `harsh` preset, `RealismPanel`.
16. `controllers/manual.ts` + `useRover` keyboard option; remaining overlays (LiDAR points, corridors); camera height / pitch / scoop capacity controls; brain-config mismatch note.
17. `report/scenarios.report.ts`, `vitest.report.config.ts`.
18. Verify all: `npm run test`, `npm run sim:report`, `npm run lint`, `npm run build`, browser pass over all acceptance criteria, `git status` clean under `apps/expo-app`.

## 24. Requirement update mapping (folded in this revision)

- Default sample first: FR-0, AC 0 / 0a, template `sample` (section 12), App initial state and autoplay (section 19), auto-fallback to naive (17.2), Milestone 1 (section 23).
- Pile up: FR-5a, carry/dump/pile growth (15.3), brain contract and dump-trip shim (17.1, DP11, DP12), gathered-into-pile metrics in readout and report, pile spot editing (15.4, section 19), `pileup.test.ts`.
- Movable leaves: FR-5b, `LeafField` with per-leaf state, spatial hash and stack grid (15.1), pushing by wheels / scoop / body (15.2), drag of leaves, clusters and the pile spot (15.4, `interaction.ts`), instanced rendering with dirty per-instance updates (section 18), sensors reading current positions (14.2, 14.3), `leaves.test.ts`. No physics engine: 2D slide + friction + stack grid covers the needed behaviour at 2500 leaves.
- Committed brain b0faf113: adapter designed against it (section 10, section 17); its `Brain` call sequence matches the phone hook.
- Concurrent splat task: ScenePanel / SplatViewer / vite.config edits are applied on top of theirs (section 10).
