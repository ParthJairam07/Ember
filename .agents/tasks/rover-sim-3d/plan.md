# Implementation Plan: 2D + 3D rover simulator in the Rover Control scene panel

Work directly in `/Users/donaldlee/Documents/GitHub-Repos/soon2026` on `main`. No worktree, no commits.
All frontend paths are relative to `/Users/donaldlee/Documents/GitHub-Repos/soon2026/frontend` (written `frontend/…`).
The dev server at http://localhost:5173 is already running with HMR (the user is watching). If it is down, start
`npm run dev` in `frontend/` in the background. Never restart it otherwise.

READ-ONLY: nothing under `/Users/donaldlee/Documents/GitHub-Repos/soon2026/apps/expo-app/` may be created, edited,
deleted or moved. Another AI owns it. Leave `frontend/src/wildfire/` alone too.

## What exploration found (facts the plan relies on)

- Frontend: React 19 + Vite 8 + TS 6 + Tailwind 4 (`@tailwindcss/vite`), three `^0.186.1`, `@types/three`,
  `@mkkellogg/gaussian-splats-3d ^0.4.7`. Scripts: `npm run build` (= `tsc -b && vite build`), `npm run lint`
  (= `oxlint`, no config file). Baseline: build passes; lint has 0 errors and 1 existing warning
  (`ScenePanel.tsx` exports `DEFAULT_SCENE`, react(only-export-components)). There is no test runner.
  `bun 1.4.4` is installed, and the brain's own tests run under `bun test`.
- `frontend/tsconfig.app.json` sets `erasableSyntaxOnly: true`. The rover-brain uses constructor parameter
  properties (`leaf-map.ts:80`, `obstacles.ts:256`). I type-checked rover-brain `index.ts` with the app's options:
  it fails with TS1294 at exactly those two places, and passes cleanly with `erasableSyntaxOnly: false`. The app's
  tsconfig is not `strict`, so nothing else breaks.
- Vite's default `server.fs.allow` already reaches the repo root (the root `package.json` has `workspaces`), but
  we set it explicitly anyway so it doesn't depend on that.
- Bun resolves `compilerOptions.paths` from `frontend/tsconfig.json`, even though that file is a
  solution-style config (`files: []` + references). I checked this in a scratch dir:
  `import { RoverSim } from '@rover-brain'` resolved and ran under `bun test`.
- RoverSim API (`apps/expo-app/src/lib/rover-brain/sim.ts`): `new RoverSim(world, Partial<SimOptions>)`,
  `start()`, `step(): SimFrame`, `frame()`, `setPile()`. Public mutable state: `world` (`leaves: [x,z][]`,
  `obstacles?: {center:[x,z], radiusM, heightM}[]`, `pile?`, `start?`, `points?`), `leaves: SimLeaf[]`
  (`{id, position: Vec3, where}`, where `id = index+1`), `x`, `z`, `yawDeg`, `scoop`, `collisions`, `timeMs`,
  `options` (incl. `axleBackM` 0.12, `bodyRadiusM` 0.2, `cameraHeightM` 0.3, `tickMs` 200), and `brain`
  (`brain.options.autopilot.scoopLipM / scoopWidthM`, `brain.pile`, `brain.leaves.snapshot()`).
  `detect()` and `pickUp()` read `sim.leaves[i].position` as the truth, so moving a leaf is just
  `sim.leaves[i].position = [x, 0, z]` plus `sim.world.leaves[i] = [x, z]`. `classify()` is private but runs on
  every `step()`. I confirmed under bun that a moved leaf gets reclassified ('pile' vs 'ground') on the next step.
  `frame()` returns copies, so renderers can treat a frame as an immutable snapshot.
- Coordinates (sim.ts + viewer.ts): flat ground at y = 0, metres. Yaw 0 faces world −z, positive yaw turns
  clockwise seen from above, and forward = (sin yaw, 0, −cos yaw). In 2D, x is right and z is down the screen, so
  the start heading is up. VIEW = {x0:-3.5, x1:3.5, z0:-5, z1:2.3}.
- In three.js (Y up), use world (x, y, z) directly as three (x, y, z). A rover group's `rotation.y = -yawRad`
  (checked: Ry(−yaw)·(0,0,−1) = (sin yaw, 0, −cos yaw)). A top-down camera at +y with `camera.up = (0,0,-1)`
  reproduces the 2D orientation exactly.
- The 2D reference (`.agents/tasks/rover-sim/reference-2d-viewer/`) is identical to what http://127.0.0.1:8765/
  serves (I diffed both index.html and viewer.js). Its controls: World select (3 WORLDS presets), Speed
  (1×/4×/10× default/30×), Seed (number, default 1), Harsh sensors checkbox
  (`{falsePerFrame:0.3, noisePerM:0.04, scoopSuccess:0.7}`), Play/Pause, Reset. Readouts: mode, message, rover time
  m:ss, in pile `n / total`, on ground, scoop (`up · 1 leaf`), trips (`stats.dumped`), collisions, wheels L/R
  (`step.left/right` toFixed(0)), done status. Loop: rAF with a dt clamp of 100 ms, `carry += dt*speed`, step while
  `carry >= tickMs`, stop when done. Draw order and semantics: viewer.ts lines 78–197.
- "Loading scene doesn't work": diagnosed in real Chrome (headless, SwiftShader WebGL, console logged to stderr)
  against the running dev server, with the uncommitted SplatViewer edits in place. The default leaf-floor sample
  reached ready ("leaf floor (sample) · 329,607 splats") with no errors in the console. The uncommitted diff fixes
  the two real causes:
  1. A React StrictMode mount → unmount → mount started a load on the viewer that was being disposed, which
     rejected unhandled with "Scene disposed". The fix is an `await Promise.resolve()` plus a cancelled check.
  2. A missing path under Vite falls back to index.html with a 200, which then parses into garbage. The fix is a
     HEAD check that rejects `text/html` or a non-ok status.
  COOP/COEP is not a factor: `sharedMemoryForWorkers: false` and `gpuAcceleratedSort: false` are already set.
  Not verified: loading a user-picked local `.ply` or `.ksplat` (the blob: path skips the HEAD check, which is correct).
- Real ground at the capture site (reference-frames/frame-03.jpg): mown green grass with dry, sunlit patches, and
  scattered flat or curled maple leaves in tan/straw `#c9a46a`, yellow `#d9b13a`, brown `#8a5a2b`, orange `#c46a26`
  and russet `#9a3f1f`, plus dark grey-brown trunks `#4a4038`. The 3D ground uses this palette: grass
  `#5d7a32`/`#6f8a3a`, dry patches `#8f8a4a`, soil `#5b4630`.

## Design decisions

1. **One sim, many views.** A plain-TS `SimController` (no React) owns the single `RoverSim`, the latest `SimFrame`,
   the trail, playing/speed/seed/noisy/world name, and a `version` counter with `subscribe()`. It is created once
   per `RoverSimulator` mount, above the 2D|3D toggle, so toggling only swaps the renderer and keeps the state.
   Only the active view is mounted; the inactive one is fully disposed. That way two WebGL contexts never run at
   once, and the splat viewer is unmounted while the sim tab is shown.
2. **Stepping lives in the controller's rAF loop**, started and stopped by a hook effect (StrictMode-safe: the
   effect cleanup cancels the rAF, and the controller survives). The views never step the sim. Each view runs its
   own rAF draw that reads `controller.frame`, so there is no React re-render per frame. The sidebar readouts
   subscribe through `useSyncExternalStore` on `version`. The controller emits at most once per rAF, and only when
   something changed.
3. **The brain is consumed through the alias `@rover-brain`**, which points at
   `/Users/donaldlee/Documents/GitHub-Repos/soon2026/apps/expo-app/src/lib/rover-brain/index.ts`. It is wired in
   three places: the Vite `resolve.alias`, `paths` in `tsconfig.app.json`, and `paths` in `tsconfig.json` (for bun
   tests). `erasableSyntaxOnly` is turned off in `tsconfig.app.json`, with a comment saying why, because the brain
   uses parameter properties and Vite/oxc transpiles them fine. This is the only way to type-check the brain's
   live code without editing it.
4. **Leaf drag mutates truth only**: `controller.moveLeaf(id, x, z)` sets `sim.leaves[i].position` and
   `sim.world.leaves[i]`, recomputes `where` for that leaf with the same pile-radius rule as `classify()` (so a
   paused view updates immediately), refreshes `frame = sim.frame()`, and emits. Leaves with `where === 'scoop'`
   are not draggable. The brain's belief is left alone, so the rover only learns the new position when its
   simulated camera/LiDAR next sees the leaf. That is the real-world behaviour the user wants to test.
   Positions are clamped to the VIEW bounds.
5. **The pile mound is rendered from the sim's own formula** (copied from the private `moundPoints()`):
   `heightM = min(0.35, 0.006·n)` and `radiusM = min(pile.radiusM·0.5, 0.1 + 0.015·n)`, so the visible mound
   matches what the LiDAR sees. Piled leaves are lifted onto the mound surface.
6. **Obstacles stay cylinders in the sim**, so the 3D form picks a look from the cylinder: `heightM >= 0.6` is a
   tree trunk (bark cylinder of `radiusM`, slight root flare, trunk drawn to 2.5 m so it reads as a tree), and
   otherwise a rock (a squashed, jittered icosahedron of footprint `radiusM` and height `heightM`). No logs: a
   lying log is not a cylinder footprint the sim can represent (this is listed as a gap).
7. **The ground is a layer behind an interface** `GroundLayer { object: THREE.Object3D; update?(frame); dispose() }`.
   The default is `ProceduralGround`. A future `SplatGround` can wrap `GaussianSplats3D.DropInViewer` (it exists in
   the installed package and is a `THREE.Group`). It would flip the splat's −y-up convention with
   `rotation.x = Math.PI`, and could feed splat centres to `SimWorld.points` for the LiDAR. That is not built now:
   only the interface and a comment pointing at it.
8. **Panel layout.** The scene area becomes a full-width row at the top of `<main>` (`lg:col-span-2`,
   `h-[78vh] min-h-[560px]`), with tabs **Rover simulator** (default) | **Splat file**. `ScenePanel` stays in the
   lower grid next to `CapturePanel` as the "Load splat file" controls; picking a file, URL or sample there switches
   the tab to Splat file. Style: the existing zinc dark Tailwind classes (`rounded-xl border border-zinc-800
   bg-zinc-900/60`, buttons `bg-zinc-800 hover:bg-zinc-700 text-xs font-semibold`, accent sky-400/500).
9. **Leftovers: keep all three.** In the browser check above, the SplatViewer.tsx fixes made the scene load, and
   the regenerated `leaf-floor.splat` and its script edit (more leaves, less bare soil) match each other. The leaf
   floor stays as the optional "Leaf floor (sample)" button in splat mode. `banana.splat` and every reference to it
   are deleted.
10. **Tests: `bun test`, no new dependency.** Tests go in `frontend/tests/` (outside `src`, so `tsc -b` never sees
    `bun:test` types), with `"test": "bun test tests"` added to the package.json scripts. They cover the
    controller, not the WebGL code.
11. **Scene templates**: the 3 reference WORLDS, copied verbatim (the default is 'Cluttered yard', which is known
    to finish in the reference), plus one richer 'Leafy lawn' preset with more leaves for the "good scene with
    leaves" ask: 16 leaves spread over x∈[-3,3], z∈[-4.6,0.4] and kept ≥0.35 m from obstacles, 2 trunks and 1 rock,
    using the same pile. If the brain fails to finish 'Leafy lawn' within 20 sim-minutes in the test, thin it to
    12 leaves and spread them out more. Do not edit the brain.
12. **Decorative litter.** Besides the sim leaves, about 1200 small, dark, flattened decorative leaves are
    instanced into the ground layer for realism. They are clearly darker and smaller than the sim leaves, not
    pickable, and covered by a "Ground litter" overlay toggle (default on). Sim leaves are larger (8–14 cm),
    brighter and curled.

## Gaps in RoverSim (for the rover-brain AI; handled frontend-side, no edits planned)

- No public `moveLeaf(id, x, z)` / `classify()`. The frontend mutates `leaves[i].position` and `world.leaves[i]`
  and recomputes `where` itself.
- Leaves have no orientation, size or species. The frontend derives a stable random size, rotation, curl and
  colour from `id` (and seed).
- Obstacles are upright cylinders only (no logs, boxes or polygons), so a log can't be represented.
- The mound geometry is private (`moundPoints`), so the frontend duplicates its formula and will drift if it
  changes.
- `wheels`, `lastTick`, `tried` and `colliding` are private. Readouts use `frame.tick.step.left/right`.
- No way to add or remove leaves at runtime (the frontend recreates the sim on reset instead).

# Steps

- [ ] 1. Wire the `@rover-brain` alias.
      `frontend/vite.config.ts`: add `resolve.alias` `{ '@rover-brain': <abs path>/apps/expo-app/src/lib/rover-brain/index.ts }`.
      Build the path with `fileURLToPath(new URL('../apps/expo-app/src/lib/rover-brain/index.ts', import.meta.url))`
      (`tsconfig.node.json` already includes node types). Also add
      `server.fs.allow: [searchForWorkspaceRoot(process.cwd()), <abs rover-brain dir>]` (import
      `searchForWorkspaceRoot` from 'vite').
      `frontend/tsconfig.app.json`: add `"paths": { "@rover-brain": ["../apps/expo-app/src/lib/rover-brain/index.ts"] }`
      and change `erasableSyntaxOnly` to `false`, with a `/* rover-brain uses parameter properties */` comment.
      `frontend/tsconfig.json`: add the same `compilerOptions.paths` (for bun).
      Package scripts: add `"test": "bun test tests"`.
      Files: frontend/vite.config.ts, frontend/tsconfig.app.json, frontend/tsconfig.json, frontend/package.json
      Verify: `npm run build` in frontend passes. The alias is only exercised once step 2 imports it, so re-run the
      build after step 2.

- [ ] 2. Create the view-agnostic sim model.
      `frontend/src/sim/worlds.ts`: export `WORLDS: Record<string, SimWorld>` (the 3 presets verbatim from
      viewer.ts lines 3–21, plus 'Leafy lawn' per decision 11), `DEFAULT_WORLD = 'Cluttered yard'`,
      `HARSH_SENSORS = { falsePerFrame: 0.3, noisePerM: 0.04, scoopSuccess: 0.7 }`,
      `VIEW = { x0:-3.5, x1:3.5, z0:-5, z1:2.3 }`, `SPEEDS = [1,4,10,30]` (default 10), and `PILE`.
      `frontend/src/sim/controller.ts`: `class SimController`. Fields: `sim`, `frame`, `trail: [number,number][]`,
      `playing` (default true), `speed`, `seed`, `noisy`, `worldName`, `version`.
      Methods:
      - `reset()`: `new RoverSim(structuredClone(WORLDS[name]), {seed, ...(noisy?HARSH:{})})`, `start()`,
        `frame = sim.frame()`, clear the trail.
      - `setWorld`, `setSeed`, `setNoisy`: each resets.
      - `setSpeed`, `togglePlaying`.
      - `advance(dtMs)`: the reference carry loop, with dt clamped to 100, pushing to the trail, and stopping when done.
      - `moveLeaf(id, x, z)`: per decision 4. Returns false for scoop leaves.
      - `leafAt(x, z, radiusM = 0.15)`: the nearest non-scoop leaf id, or null.
      - `get done()`.
      - `subscribe(fn)` / `getVersion()`.
      - `startLoop()` / `stopLoop()`: rAF calling `advance` and emitting when stepped.
      Also export `formatTime(ms)` and `moundSize(nPiled, pile)` (decision 5), and pure helpers
      `worldToCanvas` / `canvasToWorld(layout)` for the 2D view.
      `frontend/src/sim/useSimController.ts`: create the controller lazily with `useState(() => new SimController())`,
      run an effect that calls `startLoop()` and returns `stopLoop()`, and add a `useSimVersion(controller)` hook
      built on `useSyncExternalStore`. Import everything from '@rover-brain' with `import type` for types
      (verbatimModuleSyntax).
      `frontend/tests/sim/controller.test.ts` (bun:test), covering:
      - reset produces 8 ground leaves for the Cluttered yard;
      - `advance(…)` moves the rover and grows the trail;
      - toggling playing stops stepping;
      - `moveLeaf` changes both `sim.leaves[i].position` and `sim.world.leaves[i]`, and moving into the pile makes
        `frame.stats.inPile` rise immediately while paused;
      - moving a leaf out of view means the brain never targets its old spot for collection (just assert that the
        truth moved and that the frame reflects it after `advance`);
      - `leafAt` hits within 0.15 m and misses beyond;
      - every WORLDS preset (including 'Leafy lawn') clears within 20 sim-minutes at seed 1 (use `sim.run`).
      Files: frontend/src/sim/worlds.ts, frontend/src/sim/controller.ts, frontend/src/sim/useSimController.ts,
      frontend/tests/sim/controller.test.ts
      Verify: `npm test` in frontend passes all tests. `npm run build` passes (proves the alias and the tsconfig
      change). `npm run lint` shows 0 errors and no new warnings.

- [ ] 3. Port the 2D renderer into `frontend/src/components/sim/TopDownView.tsx`.
      Props: `{ controller, overlays }`. A canvas fills its container. Layout runs from a ResizeObserver (dpr-aware,
      same scale/ox/oy math as viewer.ts `layout()`), and a rAF draw reads `controller.frame` and `controller.sim`.
      Port `draw()` faithfully (grid, pile + "PILE" label, obstacles, trail, FOV wedge ±27° to 5 m, corridors
      clamped to 2.5 m, truth leaves 0.06 m, belief rings with target/tentative/failed styling, rover body round the
      axle, scoop line coloured by state, carried-leaf dot, dashed goal line). Leave out the DOM readouts, which move
      to step 5.
      Colours: a `PALETTE` const using the reference's dark-theme values (`--grid #2a2a27` → use `#27272a` zinc to
      match the app, `--pile #e0953a`, `--obstacle #9a978f`, `--leaf #d4a24c`, `--leaf-piled #8a6a3a`,
      `--belief #60a5fa`, `--open rgba(74,222,128,.5)`, `--blocked #f87171`, `--rover #ecebe7`,
      `--scoop-down #4ade80`, `--trail rgba(96,165,250,.35)`, `--fov rgba(96,165,250,.08)`, muted `#8f8c85`), with a
      background of `#18181b`.
      Each layer is gated by `overlays: { trail, fov, corridors, belief, pileZone, litter }`. Truth leaves, obstacles
      and the rover are always drawn.
      Drag: on pointerdown, map to world with `canvasToWorld` and call `controller.leafAt`. On a hit, call
      `setPointerCapture`; on each move, call `controller.moveLeaf`; on up, release. Use cursor `grab` over a leaf
      and `grabbing` while dragging. Dragging works while playing.
      Remove listeners, the observer and the rAF on unmount.
      Files: frontend/src/components/sim/TopDownView.tsx
      Verify: `npm run build` and `npm run lint` are green. The visual check happens in step 5, once the view is
      mounted.

- [ ] 4. Build the 3D scene as plain TS modules under `frontend/src/sim3d/`, then wrap it.
      - `ground.ts`: the `GroundLayer` interface (decision 7) and `ProceduralGround`:
        - a 14×14 m `PlaneGeometry` rotated −π/2, centred on the VIEW centre;
        - a `CanvasTexture` (1024², generated once with value-noise fbm) mixing grass greens, dry patches and soil
          per the palette, with `colorSpace = SRGBColorSpace`, anisotropy and `MeshStandardMaterial` roughness 1;
        - a matching low-contrast bump or roughness from the same canvas;
        - the decorative litter `InstancedMesh` (decision 12), seeded deterministically;
        - a soft fade (fog colour `#9fb4c8`, or a darker edge ring) so the plane edge isn't harsh;
        - `dispose()` frees the geometry, materials and textures.
      - `leaves.ts`: `makeLeafGeometry()`. Build a maple/oval leaf outline with `THREE.Shape` →
        `ShapeGeometry(shape, 12)` about 1 unit long, then curl it by displacing vertex y with
        `k1·x² + k2·|z|` (cup) and a slight tip lift, then `computeVertexNormals`. The material is a
        `MeshStandardMaterial` with `side: DoubleSide`, `vertexColors: false`, and instance colours.
        `class LeafInstances`: an `InstancedMesh` (count = number of leaves) with per-instance scale 0.08–0.14 m,
        yaw, a small tilt, and colour from the 5-colour dry palette. All are deterministic from leaf id, so they stay
        stable across frames. `update(frame, mound)` places 'ground' leaves at y≈0.005, 'pile' leaves on the mound
        surface (a height from the mound profile at that xz, with extra tilt), and 'scoop' leaves at the sim's
        carried position (y 0.1) so they visibly ride in the scoop. Keep an `idAt(instanceId)` map for raycasts.
      - `pile.ts`: the mound mesh. A `SphereGeometry` upper hemisphere scaled to (radiusM, heightM, radiusM) from
        `moundSize()`, in leaf-brown standard material with noise vertex jitter, hidden when n = 0. Plus the pile
        zone overlay: a `RingGeometry(r-0.02, r)` on the ground at the pile radius, in pile orange `#e0953a`,
        transparent, with a faint disc fill.
      - `obstacles.ts`: build a trunk or rock per decision 6 from `sim.world.obstacles`. Rebuild only on reset
        (compare the array identity).
      - `rover.ts`: `class RoverModel` (a THREE.Group whose origin is the axle centre on the ground):
        - chassis box ≈0.30 w × 0.08 h × 0.32 l at y 0.07, dark green `#2f4f2f` or zinc grey;
        - 4 wheels (`CylinderGeometry(0.05, 0.05, 0.035)` rotated about z, black rubber), at ±trackWidthM/2 and
          ±0.1 m fore/aft;
        - a phone mast: a thin pole at the camera position (front, `axleBackM` ahead of the axle) up to
          `cameraHeightM` 0.3, with a phone box 0.075×0.15×0.008 pitched down `cameraPitchDeg`;
        - a scoop group pivoting at the chassis front: a tray (bottom plus 2 sides plus back) of width `scoopWidthM`
          and depth `scoopDepthM`, with its lip reaching `scoopLipM` ahead of the camera when down.
        `update(frame, dtMs)`: position at the axle = cam − fwd·axleBackM, and `rotation.y = −yaw`. The scoop pitch
        target is up = −35° (lifted), down = 0° (lip on the ground) or dump = +55° (tipped forward). Ease it with an
        exponential lerp (`1 − exp(−dt/120ms)`) so it animates. The scoop tray tints green `#4ade80` when down,
        orange when dumping. Spin the wheels from the change in position.
      - `overlays.ts`, each a toggleable object:
        - trail: a `Line` with a preallocated BufferGeometry (grow with setDrawRange, reallocate at double capacity);
        - FOV wedge: a fan `BufferGeometry` on y=0.01, ±27° to 5 m, transparent sky;
        - corridors: `LineSegments` rebuilt each frame from `frame.tick.corridors`, green for open and red for
          blocked, clamped to 2.5 m;
        - belief rings: a pool of `RingGeometry` meshes from `frame.tracked` with the same state styling as 2D;
          the target ring is bigger and brighter;
        - goal: a dashed `Line` with `LineDashedMaterial` + `computeLineDistances`, using the viewer.ts goal logic.
      - `scene.ts`: `class SimScene3D`. Constructor `(container, controller)`:
        - renderer: `WebGLRenderer({ antialias: true })`, ACES tone mapping, SRGB output, shadows (PCFSoft);
        - lighting: a hemisphere light (sky `#cfe3ff`, ground `#4a5a2a`) plus a warm directional sun
          (`#fff1d6`, casting shadows from rover, obstacles and mound, shadow camera fitted to VIEW);
        - sky colour background `#a9c3dd`, with fog;
        - a `PerspectiveCamera(50)` and `OrbitControls` from 'three/examples/jsm/controls/OrbitControls.js'. The
          default view is behind and above the start: position (0, 3.2, 3.6) looking at (0, 0, -1.5).
          Clamp maxPolarAngle to just under π/2 (no going under the ground) and set min/max distance 0.6 / 14;
        - `groundLayer` slot: `setGroundLayer(layer)` disposes the old one;
        - `setOverlays(o)`, `setFollow(bool)`. Follow: each frame, lerp the controls target toward the rover and move
          the camera by the same delta, so orbiting still works;
        - `resetView()`, `topDown()` (camera at (0,9,-1.35), up = (0,0,-1), so it matches the 2D view);
        - rAF render loop reading `controller.frame` (no stepping) with `clock` dt for animation, and a
          ResizeObserver;
        - leaf drag: on pointerdown, raycast `LeafInstances.mesh`. On a hit with a draggable leaf, set
          `controls.enabled = false` and set pointer capture. On move, intersect the ray with `Plane((0,1,0), 0)` and
          call `controller.moveLeaf(id, p.x, p.z)`. On up, re-enable controls. Hover sets cursor `grab`. Use pointer
          events on `renderer.domElement`;
        - `dispose()`: cancel the rAF, disconnect the observer, remove listeners, `controls.dispose()`, traverse the
          scene disposing every geometry, material and texture, `renderer.dispose()`,
          `renderer.forceContextLoss()`, and remove the canvas.
      - `frontend/src/components/sim/ThreeView.tsx`: a div ref. An effect creates `SimScene3D` and disposes it in
        cleanup (StrictMode-safe because each mount gets its own instance). Separate effects push `overlays` and
        `follow` props to it. Small overlay buttons bottom-right: "Follow rover" toggle, "Top-down", "Reset view",
        plus the hint "drag to orbit · scroll to zoom · drag a leaf to move it".
      Files: frontend/src/sim3d/{ground,leaves,pile,obstacles,rover,overlays,scene}.ts,
      frontend/src/components/sim/ThreeView.tsx
      Verify: `npm run build` and `npm run lint` are green (0 errors, no new warnings). Visual checks come in step 5.

- [ ] 5. Create `frontend/src/components/sim/RoverSimulator.tsx` (the simulator panel UI).
      Layout: a grid `[minmax(0,1fr)_260px]` filling its parent's height. The left side is the view area with a
      top-left segmented toggle **2D | 3D** (default 3D). Store the view in `useState`, and also mirror it to
      `localStorage['roverSim.view']` so a reload keeps it. The right column has three zinc cards:
      - **Controls**: Scene template `<select>` over `WORLDS` keys; Speed select 1×/4×/10×/30×; Seed number input;
        Harsh sensors checkbox; Play/Pause + Reset buttons.
      - **Status**: mode (capitalised, large), message, then a dl of Rover time / In the pile / Still on the ground /
        Scoop / Trips to the pile / Collisions / Wheels L / R, with the done line in green. Format exactly as in
        viewer.ts lines 200–212.
      - **Legend + overlays**: one row per legend item with a swatch. Rows with a checkbox: Trail, Camera FOV,
        Corridors (open / red blocked), Brain's leaf map (belief rings; bold = target, grey = unconfirmed), Pile zone,
        Ground litter (3D only). Static rows: Leaf (truth), Trunk / rock, Rover, Scoop down / dump colours. All
        overlays default on.
      Hint text: "Drag any leaf to move it; the rover only finds out when its camera sees it."
      Uses `useSimController` and `useSimVersion` and renders `<TopDownView>` or `<ThreeView>` for the active view.
      Toggling unmounts the other view (decision 1). The controller and its loop keep running, so the time, trail
      and leaves carry over.
      Files: frontend/src/components/sim/RoverSimulator.tsx
      Verify: `npm run build` and `npm run lint` are green. The browser check happens after step 6, when the panel
      is mounted.

- [ ] 6. Integrate into the page, remove the banana, and keep the leaf-floor fixes.
      - `frontend/src/components/scenes.ts` (new): move `SceneSource`, `SAMPLE_SPLAT_URL = '/splats/leaf-floor.splat'`
        and `DEFAULT_SCENE` here. This also fixes the existing only-export-components warning.
      - `SplatViewer.tsx`: keep the uncommitted fixes. Delete `BANANA_SPLAT_URL`, and drop the `SAMPLE_SPLAT_URL`
        export in favour of `scenes.ts`.
      - `ScenePanel.tsx`: title "Load splat file". Remove the Banana button. The buttons are "Leaf floor (sample)",
        "Open .splat / .ply…" and "Clear", plus the URL form. Import from `./scenes`.
      - `App.tsx`: add `const [sceneTab, setSceneTab] = useState<'sim' | 'splat'>('sim')`. The first child of
        `<main>` becomes a `<section className="lg:col-span-2 flex h-[78vh] min-h-[560px] flex-col rounded-xl border border-zinc-800 bg-zinc-950">`
        with a tab header (Rover simulator | Splat file; the active tab is `bg-zinc-800 text-zinc-100`) and a body
        that renders `<RoverSimulator/>` or `<SplatViewer source=… label=…/>` (only the active one is mounted). The
        left column keeps `ScenePanel` + `CapturePanel`, and the `ScenePanel` `onChange` sets the scene and
        `setSceneTab('splat')`. The splat scene state starts as `DEFAULT_SCENE`, but the viewer only mounts when that
        tab is open. `SplatViewer`'s own section has `h-full`, so it fits the tab body (drop its outer border there,
        or keep it; either is fine visually).
      - Delete `frontend/public/splats/banana.splat` (`rm`) and update the `frontend/README.md` "Scene panel" bullet.
        Also add a "Rover simulator" bullet: 2D/3D views over the rover-brain `RoverSim` via `@rover-brain`, scene
        templates, leaf drag, `npm test`.
      - Leave `frontend/scripts/make-leaf-splat.mjs` and `public/splats/leaf-floor.splat` modified as they are
        (decision 9). Leave `Root.tsx` and `wildfire/` untouched.
      Files: frontend/src/components/scenes.ts, SplatViewer.tsx, ScenePanel.tsx, frontend/src/App.tsx,
      frontend/README.md, frontend/public/splats/banana.splat (deleted)
      Verify:
      - `git grep -n -i banana -- frontend ':!frontend/dist'` returns nothing.
      - `npm run build`, `npm run lint` (0 errors, 0 warnings) and `npm test` all pass.
      - In real Chrome on http://localhost:5173/:
        1. The page opens on the Rover simulator tab in 3D, the rover drives itself, and the readouts tick.
        2. Toggle to 2D: the same time, trail and piled count carry over, and toggling back keeps the state.
        3. Switch the scene template and the sim resets.
        4. Drag a leaf in both views while it is playing.
        5. Splat file → Leaf floor (sample) loads ("… splats" badge).
        6. Back to the simulator, with no console errors throughout.
      Headless check recipe (already proven to work): run
      `"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless=new --user-data-dir=/tmp/chr-prof --enable-logging=stderr --v=0 --use-angle=swiftshader --enable-unsafe-swiftshader --virtual-time-budget=20000 --window-size=1400,900 --screenshot=/tmp/sim.png http://localhost:5173/`
      in the background, `sleep 45; kill` it (macOS has no `timeout`), then grep stderr for `CONSOLE` errors and look
      at the screenshot. Delete the /tmp files afterwards.

- [ ] 7. Polish and the final gate.
      Check the 3D look against reference-frames/frame-03.jpg: green lawn, varied curled tan/yellow/brown/orange
      leaves, a visible mound growing at the pile, shadows. Check the scoop animates down while scooping and tips at
      the pile. Check disposal: toggle 2D↔3D 10 times and confirm no "Too many active WebGL contexts" warning in the
      console. Run the full gate.
      Files: any of the above, as needed
      Verify: `npm run build`, `npm run lint` and `npm test` in frontend all pass, and the browser check from step 6
      shows no console errors. `git -C /Users/donaldlee/Documents/GitHub-Repos/soon2026 status --short` shows no
      changes under `apps/expo-app/`.

## Known limitations / not built

- Splat ground capture while the rover moves (only the `GroundLayer` slot and notes exist).
- Brain belief is not reset when a leaf is dragged. This is intentional: the rover rediscovers it.
- Local `.ply`/`.ksplat` file loading was not re-tested in the browser. Only the bundled `.splat` sample was.
