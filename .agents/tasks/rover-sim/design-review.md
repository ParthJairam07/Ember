# Design review: rover simulated environment (`design.md`)

Verdict: **CHANGES_REQUESTED** (2 HIGH, 12 MEDIUM, 11 NIT)

The design is strong where it counts most. The sensor shapes are taken from the real brain and the native contract, the portrait pose and upright-frame mapping are correct (checked numerically against `rover-brain`), the adapter calls the same `Brain` sequence as `use-rover-brain.ts`, and env tests are kept apart from brain scenario runs. What blocks it: one template can't pass its own validation, drop-off physics can't work on the terrain model as written, and several specs either conflict with each other or don't match the real phone pipeline.

## Findings

### 1. HIGH: Reference template exceeds the 2500-leaf cap, so `validateWorld` throws
Where: §12 template table (`reference`) vs §12 validation ("leaves 2500") and NFR-2.
Problem: 20×14 m at 3 /m² = 840, plus carpet 2×3 m at 80 /m² = 480, plus carpet r 1.8 m at 120 /m² ≈ 1221. That's about 2541 leaves before the treeline drift band, which has no density at all. `generateWorld('reference', seed)` would throw "invalid world", and `generate.test.ts` (every template, seeds 1–20) fails. This is the template the user asked to base on IMG_5056.
Fix: set the densities so the total stays under the cap and give the drift a density: base 2 /m² (≈560), path carpet 2×3 m at 80 /m² (480), trunk carpet r 1.5 m at 100 /m² (≈707), treeline drift band 1 m × 20 m at 30 /m² (600). Total ≈ 2347. Add a line under the table: "Each template's expected leaf count must be ≤ 2300 (10 % headroom under the cap); `generate.test.ts` asserts it for seeds 1–20."

### 2. HIGH: Drop-off and up-step physics can't work on a bilinear 5 cm height grid
Where: §9 `terrain.ts` ("HeightGrid (5 cm grid, bilinear)"), §12 `TerrainSpec` ("steps (curbs/walls as height discontinuities)"), §13 Drops.
Problem: drops are detected per wheel per 10 ms step as `h_prev − h_now > fallDropM (0.08)`. At 0.6 m/s a wheel moves 6 mm per step. If a 0.15 m curb lives in a bilinear 5 cm grid, it becomes a ramp, and the per-step change is about 0.15 × 6/50 ≈ 1.8 cm, so it never triggers. The design doesn't say whether `heightAt` evaluates steps analytically. The reverse case is also missing: a wheel on the road driving into a 0.15 m up-step (> `climbStepM` 0.05) should be blocked, but the collision code only tests `Obstacle` shapes. `collision.test.ts` ("drive off curb → dropOffs = 1") and AC10(g) depend on this.
Fix: state that `heightAt(x, z) = smoothGrid(x, z) + Σ step.offset · [signedDistance(step.polyline, x, z) < 0]`, with steps evaluated exactly and never baked into the bilinear grid. The raymarcher uses the same function, with a bisection on sign change across step edges. Add to §13: "Up-step: if a wheel's next contact height exceeds its current one by > `climbStepM`, treat it like a solid collision (revert pose, zero wheel speeds, count a collision episode)." Add a test that drives up the curb from the road and expects `collisions = 1`, no climb.

### 3. MEDIUM: Detection camera's depth is stale, unlike the phone
Where: §14.3 ("snapshots `PhoneCamera` (with the latest depth)").
Problem: on the phone, `nextDetectionFrame` returns the ARKit frame's own pose and its own `sceneDepth` (`CaptureController.swift:436`, `DetectionCamera.depth`). In the sim, the latest depth can be up to 200 ms older than the capture pose. At 0.6 m/s that's up to 12 cm of mismatch, which `locateDetections` would turn into placement error that doesn't exist on the real rover. That makes the sim misleading.
Fix: "At each detection capture `t0`, render a fresh depth map from `C_true(t0)` (same noise model and `lidar` stream) and attach it to that `PhoneCamera`. The 5 Hz tick depth is separate." Add the cost to the NFR-2 budget (≈3 extra scans/s at realistic).

### 4. MEDIUM: Under `ideal`, detection frames run at 100 Hz
Where: §14.3 ("latency … ideal: 0 → delivered in the same tick. The next capture starts at delivery.") and §16 step order.
Problem: with zero latency and "next capture at delivery", a capture starts every 10 ms physics step, so the brain gets about 100 detection frames per second. That reaches `confirmHits` (3) in 30 ms, costs a lot, and is nothing like the phone (a few Hz). "Delivered in the same tick" also conflicts with §16: delivery happens in step (1), capture in step (3), so it actually lands in the next step.
Fix: add a knob `detectPeriodMs` (minimum capture-to-capture interval), set to 333 ms in all presets. Next capture = `max(delivery, lastCapture + detectPeriodMs)`. Ideal latency 0 means "delivered at step (1) of the next physics step". Say so explicitly.

### 5. MEDIUM: AR origin height under realistic/harsh is ambiguous and unrealistic
Where: §11 ("AR origin = initial camera position on the ground with a random yaw").
Problem: "camera position on the ground" could mean the camera's 3D position or the point on the ground below it. ARKit puts the origin at the camera's initial position, so the real floor sits at y ≈ −0.30. If the sim puts it on the ground (floor y = 0), the brain never sees a non-zero floor height, which hides any floor-height bug the real phone would show.
Fix: "`T_ar` maps the camera's initial 3D position (at `cameraHeightM` above the ground) to the AR origin, with yaw ~ U(0, 360°) from the `pose` stream. In AR coordinates the floor is at y ≈ −cameraHeightM. Ideal stays identity (floor y = 0)." Add a detector test under realistic that checks located leaves sit within 3 cm of the AR floor.

### 6. MEDIUM: Projection round-trip test (AC10a) has no defined metric or conditions
Where: AC10(a), §21 `detector.test.ts`.
Problem: "recovers it within 3 cm" doesn't say 3D or horizontal distance, or what the reference point is (leaf centre on terrain, or the leaf top at `y + curl·0.02`). LiDAR sees the stack-grid top, up to 2 cm above the ground. At 1.5 m and an 11° grazing angle, one 256×192 depth pixel spans about 4 cm of ground, so depth quantisation alone takes up most of the budget. As written, the must-pass test can fail for reasons that have nothing to do with a sim bug, or get loosened ad hoc.
Fix: "Metric = horizontal (x, z) distance between the located `world` and the leaf's true centre. Conditions: ideal preset, `depthRes: 'full'`, a single leaf with `curl = 0`, no stack neighbours, distance 1.0–2.0 m, box built from the exact projected outline (no jitter). LiDAR: ≤ 0.03 m. Floor ray with `floorY = 0`: ≤ 0.01 m (it's exact up to the box-centre vs centroid offset). Also assert |world.y − 0| ≤ 0.03 for LiDAR."

### 7. MEDIUM: Detector test asserts the wrong vertical position
Where: §21 `detector.test.ts` ("leaf straight ahead → box centre u within 2 px of 288, v > 384").
Problem: with the camera at 0.3 m pitched 20°, any ground leaf farther than 0.3/tan 20° ≈ 0.82 m sits above the optical axis, so v < 384. Checked numerically with the brain's `makePose` and the design's portrait pose: a leaf 1.5 m ahead projects to (288.0, 289.5). `locateDetections` recovers it exactly, and a leaf 0.3 m right projects to u = 410.6, bearing +11.3°. The u assertion and the convention are right; the v assertion is wrong for typical distances.
Fix: "Leaf at (0, 0, −1.5), camera (0, 0.3, 0) pitch 20 yaw 0 → u = 288 ± 2, v = 289.5 ± 2. Leaf at (0, 0, −0.6) → v = 455 ± 2. Leaf at (0.3, 0, −1.5) → u > 288, bearing +11.3° ± 0.5."

### 8. MEDIUM: Whether the brain owns carry/dump is detected by an ambiguous rule
Where: §17.1 (`carryDump = 'brain'` if "`step.scoop` is ever present and 'dump' is a value the brain can return (checked once: … `stepAutopilot` returns a `scoop` field on the first tick)").
Problem: on the `Brain` path, `tick()` returns no `step` until `start()`, so "the first tick" is undefined. "Ever present" and "checked once" contradict each other, and a runtime check can't tell whether `'dump'` is reachable.
Fix: decide statically at load. `carryDump = 'brain'` iff `typeof rb.Brain === 'function' && typeof rb.Brain.prototype.setPile === 'function' && typeof rb.DEFAULT_AUTOPILOT?.capacity === 'number'`; otherwise `'shim'`. On the raw path it's always `'shim'`. Also, if the brain path is in use and the sim's scoop is full (≥ `scoopCapacity`) while the brain mode is not `returning`/`dumping` for 30 s, show the amber note "Brain doesn't empty a full scoop" in the UI and report, but don't take over.

### 9. MEDIUM: The "brain edits are picked up on Reset" claim is unverified and probably wrong
Where: §10 ("Vite watches imported files outside its root, so edits by the brain agent are served on the next import (and Reset in the UI creates a fresh brain instance from the current module)").
Problem: `import('@rover-brain')` returns the cached module instance, so Reset reuses old code unless Vite's HMR invalidated it. When a brain file changes, Vite propagates the update up the importer graph (brainAdapter → useSimulation → SimView). Depending on React Refresh boundaries, that either re-executes the SimView chain (resetting the sim mid-run) or does a full page reload. Neither matches what's described. The brain agent is editing these files right now (`use-rover-brain.ts` changed while this review was running), so this will happen often.
Fix: pick the behaviour and state it. Recommended: "`brainAdapter.ts` calls `import.meta.hot?.accept()` (self-accept) so brain edits don't propagate further up. On a brain-module update, the adapter sets status `{ kind: 'error', fatal: false, message: 'Brain code changed; press Reset to load it' }` (via `import.meta.hot.on('vite:afterUpdate')`). Reset then calls `import(/* @vite-ignore */ BRAIN_ENTRY + '?t=' + Date.now())` to get the new instance." We must not edit `apps/expo-app`, so the check must not touch it. Verify it in a unit test instead: `createBrainController(mod)` with a fake module A, then `reload()` with a fake module B, and the next `reset` uses B's `Brain`. In the browser, observe it passively when the brain agent next saves a file.

### 10. MEDIUM: "`git status` clean under `apps/expo-app`" can't be the check while another agent edits there
Where: AC12, §23 steps 12 and 18.
Problem: the brain agent is concurrently modifying `apps/expo-app` (see `git-baseline.txt` and the live hook edits). `git status` will show changes there that aren't ours, so the check either always fails or gets ignored.
Fix: "Before starting, record `git diff --stat HEAD -- apps/expo-app` and the file mtimes in `.agents/tasks/rover-sim/git-baseline.txt`. The verification is: (a) none of the sim task's commits or staged changes touch `apps/expo-app` (`git diff --cached --name-only | grep -c '^apps/expo-app' == 0`, and the same for every commit the task makes); (b) the sim implementer never opens a write tool on that path. Changes made by the brain agent are expected and are not a failure."

### 11. MEDIUM: Manual WASD won't respond right after choosing "Manual" in the select
Where: §17.2 `manual.ts` ("ignores text inputs, like `useRover`"), AC6.
Problem: `useRover`'s `isTextInput` treats `SELECT` as a text input. Right after the user picks "Manual WASD" in the controller `<select>`, focus is still on the select, so W/A/S/D are ignored (and letter keys do type-ahead in the select). AC6 fails until the user clicks somewhere else.
Fix: "When the controller select changes to `manual`, call `event.currentTarget.blur()` and focus the sim canvas container (`tabIndex=0`). `manual.ts` ignores only `INPUT[type=text|number]`, `TEXTAREA` and contentEditable, not `SELECT`/`BUTTON`." Add the hint "Click the 3D view, then use WASD" under the select.

### 12. MEDIUM: Leaving and returning to `Splat (file)` mode isn't specified
Where: §19 (`SceneMode`, "Splat (file) … keeps whatever default the splat task sets").
Problem: the requirement is that the splat mode behaves exactly as before, but the design doesn't say which `SceneSource` is shown when you switch back to Splat (the default, or the last one loaded), or what happens to a `blob:` URL from Open file when the mode switches. The current `ScenePanel` revokes the old blob only when a new file replaces it. Unmounting `SplatViewer` and remounting it later would show the wrong scene or a revoked URL.
Fix: "App keeps `lastSplatScene: SceneSource` (initial `DEFAULT_SCENE`) in its own state, separate from `mode`. Choosing Splat (file) restores it. Switching to a template never revokes its blob URL (only `ScenePanel`'s existing replace-file path does). Acceptance: open a local .splat, switch to Sample, switch back → the same file shows."

### 13. MEDIUM: Pile heights in the templates don't follow from the stack model
Where: §8 ("low domes 0.3–0.6 m radius, 8–20 cm high"), §12 `piles` (r 0.3–0.6, 40–120 leaves, "giving 8–20 cm via the stack grid"), §15.1 (`leafLayerM` 0.02 × leaf area).
Problem: a 0.1 m leaf covers about 0.0055 m², so it adds about 1.1e-4 m³. A dome holds πr²h/2. 120 leaves at r 0.3 gives about 9 cm, and 40 leaves at r 0.6 gives under 1 cm, so most piles come out flat. 20 cm at r 0.6 would need about 1000 leaves. DP7 (piles taller than `maxStepM` read as obstacles) depends on these heights, so the brain-facing behaviour is currently undefined.
Fix: make piles height-driven. Each pile draws r ∈ [0.3, 0.4] m and h ∈ [0.08, 0.15] m, then n = ceil(π r² h / 2 / (leafLayerM · meanLeafArea)), roughly 100–340 leaves. Leaves are placed with a radial density ∝ (1 − (d/r)²). `piles` = 5 such piles (total ≤ 1750) plus 30 loose. `forest-floor`'s 2 piles use the same rule. Add to `generate.test.ts`: every pile's max stack height is within ±30 % of its drawn h.

### 14. MEDIUM: Internal conflicts between sections
Where / fix:
- A1 says the phone hook "still uses the older raw-function API". §10 says the hook at b0faf113 uses `Brain`. The current `use-rover-brain.ts` uses `new Brain()`, `observe`, `addScan`, `start`/`stop`, `tick`, `setPile`. Fix A1 to match §10, and relabel the raw-function fallback as "for an older or future brain without `Brain`", not "mirrors today's hook".
- §8 says Dense forest floor uses "the carpet density (100+ /m²)". The §12 table says 600 leaves on 12×12 m (4 /m²). 100 /m² over 144 m² would be 14,400, far over the cap. Fix: `forest-floor` = 4 /m² mulch base (≈580) plus three carpets of r 1.5 m at 100 /m² (≈2120 total, ≤ 2300), and update §8.
- §23 Milestone 1 includes AC10(h) (determinism), but §21's `sim.test.ts` runs on `obstacles` seed 7, which only exists in Milestone 2. Fix: M1's `sim.test.ts` runs `sample` seed 1, realistic, 60 s naive. M2 adds the `obstacles` seed 7 case.
- AC1 lists "Load sample, Open file, Load URL and Clear". The current panel has "Leaf floor", "Banana", "Open .splat / .ply…", "Clear" and "Load URL". Fix: AC1 = "every existing ScenePanel control (Leaf floor, Banana, Open .splat/.ply…, Clear, Load URL) behaves as before".

### 15. NIT: `vitest.report.config.ts` with `mergeConfig` won't replace `include`
Where: §21 Report ("merges `vite.config.ts` and sets `include: ['src/**/*.report.ts']`").
Problem: Vite's `mergeConfig` concatenates arrays, so the report run would include both `*.test.ts` and `*.report.ts`.
Fix: `export default defineConfig({ ...base, test: { ...base.test, include: ['src/**/*.report.ts'], testTimeout: 600_000 } })` (object spread, not `mergeConfig`). Add `vitest.report.config.ts` to `tsconfig.node.json`'s `include` so `tsc -b` checks it.

### 16. NIT: Detection labels don't match the phone's
Where: §14.3 (labels are the query strings, e.g. "a photo of a dead leaf").
Problem: the phone's detector emits the short label keys from `packages/owlvit/export.py`: "dead leaf", "brown leaf", "dry leaf", "fallen leaves", "leaves on ground", "leaf pile". The brain doesn't branch on labels today, but it might later.
Fix: use those six labels; piles prefer "leaf pile".

### 17. NIT: Captions can't be imported from the hook
Where: §17 ("captions from leaves/rejected (same text as the phone)").
Problem: `caption` and `rejectionCaption` live in `apps/expo-app/src/hooks/use-rover-brain.ts`, which imports React Native, not in `rover-brain`. The hook's caption rules also changed during this review (it now captions only floor-ray placements, prefixed "~").
Fix: "`brainAdapter.ts` contains a copy of the caption formatting (distance `toFixed(1)` m · `ahead` / `N° R|L`; rejection texts per reason). It captions every located leaf, prefixing floor-ray ones with '~'. Divergence from the hook is acceptable and noted in a comment."

### 18. NIT: Effective LiDAR range is 4 m, not 5 m
Where: §14.2 (range 5 m, but confidence 0 beyond 4 m).
Problem: `depthToWorld` and `depthAt` drop confidence < 1, so 4–5 m readings are unusable even though they're emitted.
Fix: say so explicitly ("readings 4–5 m are emitted with confidence 0, like ARKit, and ignored by the brain"), or use confidence 1 up to 5 m. Pick the first.

### 19. NIT: Pile spot y in AR coordinates is undefined
Where: §11 / §15.3 (`DropZone` has only x, z; `Pile.center` is a Vec3).
Fix: "`pileAR.center` = `T_ar · (x, terrainHeight(x, z), z)`." The brain uses only horizontal distance, but this keeps it consistent with the phone, which sets y = floorY.

### 20. NIT: InstancedMesh bounding sphere goes stale as leaves move
Where: §18 leaf transforms and interaction picking.
Problem: three r186 `InstancedMesh` caches `boundingSphere` for raycast and culling. Leaves dragged, dumped or carried outside the initial sphere can be missed by picking.
Fix: after a frame with dirty leaves, set `mesh.boundingSphere = null` (recomputed lazily), or set `frustumCulled = false` and recompute the sphere on drag start.

### 21. NIT: Leaf boxes ignore curl height, so leaves disappear at short range
Where: §14.3 (outline of 8 points all at height `y + curl·0.02`).
Problem: seen at 11° grazing from 1.5 m, a flat 10 cm leaf is about 8 px tall. Under `minBoxPx` 6 that cuts detection off near 1.8 m, while curled real leaves (lifted 1–3 cm, §8) look taller.
Fix: the outline uses 8 ellipse points at `y`, plus 4 lobe-tip points at `y + curl·0.03`. The bbox covers all 12.

### 22. NIT: The landscape pose-convention test's expectations aren't stated
Where: §11 (`poseConvention` debug knob, "env tests run both").
Problem: under `landscape`, the upright frame is rotated relative to the world, so AC10(b) ("right of centre") doesn't hold. Only located world points and headings should match.
Fix: "Under `landscape`, assert only that `locateDetections` world points and `headingFromPose` are identical to `arkit-portrait` for the same true camera. AC10(b) runs under `arkit-portrait` only." Also correct DP3: the brain's tests use landscape intrinsics `K` 1920×1440 (`tests/rover-brain/scene.ts`), while its `sim.ts` uses portrait-shaped 1440×1920.

### 23. NIT: Detection-frame zoom is unmentioned
Where: §11 / FR-6.
Fix: one line: "The sim models zoom = 1 (frameSize = 576×768). The phone's digital zoom (`uncrop`) is out of scope."

### 24. NIT: Raw-function fallback path is scope beyond today's needs
Where: §17 Fallback path, DP1.
Problem: the brain and the phone both use `Brain` now. Keeping a second integration path means more code and tests for a hypothetical brain.
Fix: keep it, but limit it to Milestone 2 and to the exports check plus one fake-module test. Milestone 1 implements only the `Brain` path; a missing `Brain` is a fatal "missing exports" error that triggers the naive fallback.

### 25. NIT: Forward-motion direction is not written down
Where: §13 kinematics step 8.
Fix: add "forward unit vector for yaw ψ = (sin ψ, 0, −cos ψ)", which matches `headingFromPose` (yaw 0 → −z, yaw 90° → +x).

## Verified assumptions

- `rover-brain` exports used by the adapter exist: `Brain` (`observe`, `addScan`, `setPile`, `start`, `stop`, `tick`), `brain.obstacles.floorY` (getter), `brain.leaves.snapshot()`, `brain.autopilot.{mode,load,targetId}`, `BrainTick.{corridors,step}`, `AutopilotStep.{left,right,scoop,message,collectedIds,missingId,failedId,dumped}`, `locateDetections`, `scanDepth`, `headingFromPose`, `isPose`, `uprightToSensor`, `unproject`, `scaleIntrinsics`, `decodeDepth`, `corridors`, `DEFAULT_ROVER` (cameraHeightM 0.3, maxStepM 0.08), `DEFAULT_AUTOPILOT` (scoopLipM 0.15, scoopWidthM 0.30, capacity 1, dumpAtM 0.6, lowerMs 500, liftMs 800, dumpMs 1500, backupM 0.35). Loaded through Vite SSR with an `@rover-brain` alias from `frontend/`; it imports cleanly and `new Brain()` works.
- `CameraSnapshot` / `BoxDetection` match the design's `PhoneCamera` / `SimDetection` structurally. They also match the native `DetectionCamera` in `SplatCapture.types.ts`.
- Upright ↔ sensor mapping: `uprightToSensor` maps upright (u, v) to sensor (v·W/Fh, H − u·H/Fw). The design's inverse ((1440 − y)·0.4, x·0.4) is correct. FOV of 50.0° horizontal and 63.7° vertical with fx = 1545 is correct.
- Portrait pose `[−y₀, x₀, z₀]` (= makePose · Rz(−90°)) is a proper rotation. Numerically: a ground leaf 1.5 m ahead lands at upright u = 288.0, a leaf 0.3 m right lands at u = 410.6, `locateDetections` recovers both exactly via floor ray, and `headingFromPose` yaw = 30.000° for yaw 30. Bearing 0 is ahead and + is right.
- Depth is axis depth (`unproject` multiplies by depth, z = −depth). `depthToWorld` samples pixel centres (x + 0.5) with `scaleIntrinsics`, matching §11.
- `estimateFloorY` / `scanDepth` give `floorSource: 'lidar'` or `'camera_height'`. Corridors use `maxStepM`, `maxDipM` 0.1 (so a 0.15 m curb → `'drop'`) and 13 bearings, matching the 13 overlay segments.
- Autopilot halts with "Scoop full: set a pile" when there's no pile, and ignores leaves inside the pile (`inPile`, horizontal distance). It also halts if `scanAgeMs > 1500`, which 5 Hz depth satisfies.
- `use-rover-brain.ts` calls `new Brain()`, `observe(locateDetections(...))` with floorY = `obstacles.floorY ?? origin.y − cameraHeightM`, skips detection results when `!trackingNormal`, and runs `scanDepth → addScan → tick(heading or null)`. The adapter's sequence matches.
- TS1294 on parameter properties at `leaf-map.ts:80` and `obstacles.ts:256` under `erasableSyntaxOnly: true`. With it off, the brain type-checks under the frontend's app options (`tsc` exit 0).
- `searchForWorkspaceRoot(frontend)` returns the repo root (Vite only checks for a `workspaces` field, not membership), so `server.fs.allow` reaches the brain.
- Versions: vite 8.3.5, typescript 6.0.3, three 0.186.1 installed. vitest 5.0.5 exists (latest), with peer `vite ^6.4 || ^7 || ^8`. `three/examples/jsm/controls/OrbitControls.js` exists. `DropInViewer` exists in `@mkkellogg/gaussian-splats-3d` 0.4.7.
- `useRover` binds window keydown/keyup/blur/visibility in one effect, so a `keyboard: false` option that skips it is feasible. `isTextInput` includes SELECT (which leads to finding 11).
- The brain has no `Date.now` / `Math.random`. Its own `sim.ts` uses topSpeed 0.6 m/s and axleBackM 0.12, as A2 and §13 say.
- The scope split is explicit (NFR-3, §6, AC12), and only `brainAdapter.ts` imports `@rover-brain`. Env tests (must pass) are kept apart from `sim:report` (brain metrics, exit 0 unless the sim throws).
- `SplatViewer` / `ScenePanel` are preserved behind `SceneMode.kind === 'splat'`. The `Surface` interface and `world.underlay` give a concrete hook for a future splat.

## Unverified / wrong assumptions

- Wrong: A1, "the phone hook still uses the older raw-function API". It uses `Brain` (finding 14).
- Wrong: §21, "leaf straight ahead → v > 384". At typical distances v < 384 (finding 7).
- Wrong: §8 / §12, the reference template fits the leaf cap (finding 1), and forest-floor density / pile heights follow from the counts (findings 13, 14).
- Wrong: the detection snapshot carrying "the latest depth" matches the phone. The phone's detection camera carries its own frame's depth (finding 3).
- Wrong (minor): DP3 says the brain tests use portrait-shaped intrinsics. `tests/rover-brain/scene.ts` uses landscape `K` 1920×1440 (finding 22).
- Wrong (minor): labels are the query strings. The phone emits short labels (finding 16).
- Unverified: brain edits are picked up on Reset via Vite (finding 9).
- Unverified: drop detection on the terrain model (finding 2).
- Unverified: NFR-2 performance (≤ 20 ms per 256×192 scan in JS, ≤ 12 ms per frame with 2500 leaves) is plausible with tile skipping but untested. Finding 3 adds about 3 scans/s. The browser loop's slow-down-and-report fallback covers it, so it's not a finding.
- Unverified: `git status` as the isolation check while another agent edits `apps/expo-app` (finding 10).
