// ../../../../../../../Users/donaldlee/Documents/GitHub-Repos/soon2026/apps/expo-app/src/lib/rover-brain/geometry.ts
var sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
var dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function rotate(pose, v) {
  return [
    pose[0] * v[0] + pose[4] * v[1] + pose[8] * v[2],
    pose[1] * v[0] + pose[5] * v[1] + pose[9] * v[2],
    pose[2] * v[0] + pose[6] * v[1] + pose[10] * v[2]
  ];
}
function cameraPosition(pose) {
  return [pose[12], pose[13], pose[14]];
}
function headingFromPose(pose) {
  const look = rotate(pose, [0, 0, -1]);
  const length = Math.hypot(look[0], look[2]);
  if (length < 0.05)
    return null;
  const forward = [look[0] / length, 0, look[2] / length];
  const right = [-forward[2], 0, forward[0]];
  return {
    origin: cameraPosition(pose),
    forward,
    right,
    yawDeg: Math.atan2(forward[0], -forward[2]) * 180 / Math.PI
  };
}
function toRover(world, heading) {
  const d = sub(world, heading.origin);
  const right = dot(d, heading.right);
  const forward = dot(d, heading.forward);
  return {
    right,
    forward,
    up: d[1],
    distance: Math.hypot(right, forward),
    bearingDeg: Math.atan2(right, forward) * 180 / Math.PI + 0
  };
}
function angleDiffDeg(a, b) {
  let d = (a - b) % 360;
  if (d <= -180)
    d += 360;
  if (d > 180)
    d -= 360;
  return d;
}
function makePose(position, pitchDeg, yawDeg) {
  const p = -pitchDeg * Math.PI / 180;
  const y = -yawDeg * Math.PI / 180;
  const rx = [
    [1, 0, 0],
    [0, Math.cos(p), -Math.sin(p)],
    [0, Math.sin(p), Math.cos(p)]
  ];
  const ry = [
    [Math.cos(y), 0, Math.sin(y)],
    [0, 1, 0],
    [-Math.sin(y), 0, Math.cos(y)]
  ];
  const r = ry.map((row) => [0, 1, 2].map((j) => row[0] * rx[0][j] + row[1] * rx[1][j] + row[2] * rx[2][j]));
  return [
    r[0][0],
    r[1][0],
    r[2][0],
    0,
    r[0][1],
    r[1][1],
    r[2][1],
    0,
    r[0][2],
    r[1][2],
    r[2][2],
    0,
    position[0],
    position[1],
    position[2],
    1
  ];
}

// ../../../../../../../Users/donaldlee/Documents/GitHub-Repos/soon2026/apps/expo-app/src/lib/rover-brain/obstacles.ts
var DEFAULT_ROVER = {
  widthM: 0.45,
  maxStepM: 0.08,
  maxDipM: 0.1,
  clearanceHeightM: 0.6,
  minPoints: 4,
  cameraHeightM: 0.3,
  bodyBackM: 0.35,
  sideMarginM: 0.1
};
var CORRIDOR_BEARINGS = [-45, -37.5, -30, -22.5, -15, -7.5, 0, 7.5, 15, 22.5, 30, 37.5, 45];
var MAX_RANGE_M = 5;
function corridors(scan, heading, rover = DEFAULT_ROVER, bearings = CORRIDOR_BEARINGS) {
  const half = rover.widthM / 2;
  const strips = bearings.map((bearingDeg) => {
    const b = bearingDeg * Math.PI / 180;
    return { bearingDeg, sin: Math.sin(b), cos: Math.cos(b), blocking: [], observed: 0 };
  });
  const points = scan?.points ?? [];
  const floorY = scan?.floorY ?? 0;
  const besides = { left: 0, right: 0 };
  for (const p of points) {
    const r = toRover(p, heading);
    const height = p[1] - floorY;
    const kind = height > rover.maxStepM && height < rover.clearanceHeightM ? "obstacle" : height < -rover.maxDipM ? "drop" : null;
    if (height >= rover.clearanceHeightM)
      continue;
    if (r.forward <= 0) {
      if (kind && r.forward > -rover.bodyBackM && Math.abs(r.right) < half + rover.sideMarginM) {
        besides[r.right > 0 ? "right" : "left"] += 1;
      }
      continue;
    }
    for (const s of strips) {
      const along = r.right * s.sin + r.forward * s.cos;
      if (along <= 0 || along > MAX_RANGE_M)
        continue;
      if (Math.abs(r.right * s.cos - r.forward * s.sin) > half)
        continue;
      if (along > s.observed)
        s.observed = along;
      if (kind)
        s.blocking.push([along, kind]);
    }
  }
  return strips.map((s) => {
    const beside = s.bearingDeg > 0 ? besides.right : s.bearingDeg < 0 ? besides.left : 0;
    if (beside >= rover.minPoints) {
      return { bearingDeg: s.bearingDeg, clearanceM: 0, blockedBy: "obstacle", observedM: s.observed };
    }
    if (s.blocking.length < rover.minPoints) {
      return { bearingDeg: s.bearingDeg, clearanceM: MAX_RANGE_M, blockedBy: null, observedM: s.observed };
    }
    s.blocking.sort((a, b) => a[0] - b[0]);
    const nearest = s.blocking.slice(0, rover.minPoints);
    const drops = nearest.filter(([, kind]) => kind === "drop").length;
    return {
      bearingDeg: s.bearingDeg,
      clearanceM: nearest[nearest.length - 1][0],
      blockedBy: drops * 2 > nearest.length ? "drop" : "obstacle",
      observedM: s.observed
    };
  });
}
var DEFAULT_OBSTACLE_MEMORY = { maxAgeMs: 1e4, voxelM: 0.05, maxPoints: 20000 };
function inView(p, view, maxDepthM = MAX_RANGE_M) {
  const m = view.pose;
  const d = [p[0] - m[12], p[1] - m[13], p[2] - m[14]];
  const x = m[0] * d[0] + m[1] * d[1] + m[2] * d[2];
  const y = m[4] * d[0] + m[5] * d[1] + m[6] * d[2];
  const z = m[8] * d[0] + m[9] * d[1] + m[10] * d[2];
  const depth = -z;
  if (depth < 0.1 || depth > maxDepthM)
    return false;
  const k = view.intrinsics;
  const u = k.cx + k.fx * x / depth;
  const v = k.cy - k.fy * y / depth;
  return u >= 0 && u < k.width && v >= 0 && v < k.height;
}

class ObstacleMemory {
  options;
  cells = new Map;
  floor = null;
  lastScanAtMs = -Infinity;
  lastTimestamp = 0;
  constructor(options = DEFAULT_OBSTACLE_MEMORY) {
    this.options = options;
  }
  add(scan, nowMs) {
    if (scan.view) {
      const view = scan.view;
      for (const [key, cell] of this.cells) {
        if (inView(cell.point, view))
          this.cells.delete(key);
      }
    }
    const v = this.options.voxelM;
    for (const p of scan.points) {
      const key = `${Math.floor(p[0] / v)},${Math.floor(p[1] / v)},${Math.floor(p[2] / v)}`;
      this.cells.set(key, { point: p, atMs: nowMs });
    }
    if (scan.floorSource === "lidar" || !this.floor || this.floor.source !== "lidar") {
      this.floor = { y: scan.floorY, source: scan.floorSource };
    }
    this.lastScanAtMs = nowMs;
    this.lastTimestamp = scan.timestamp;
    this.prune(nowMs);
  }
  scan(nowMs) {
    if (!this.floor)
      return null;
    this.prune(nowMs);
    return {
      points: [...this.cells.values()].map((c) => c.point),
      floorY: this.floor.y,
      floorSource: this.floor.source,
      timestamp: this.lastTimestamp
    };
  }
  ageMs(nowMs) {
    return nowMs - this.lastScanAtMs;
  }
  get floorY() {
    return this.floor?.y ?? null;
  }
  clear() {
    this.cells.clear();
    this.floor = null;
    this.lastScanAtMs = -Infinity;
  }
  prune(nowMs) {
    for (const [key, cell] of this.cells) {
      if (nowMs - cell.atMs > this.options.maxAgeMs)
        this.cells.delete(key);
    }
    if (this.cells.size > this.options.maxPoints) {
      const oldest = [...this.cells].sort((a, b) => a[1].atMs - b[1].atMs);
      for (const [key] of oldest.slice(0, this.cells.size - this.options.maxPoints))
        this.cells.delete(key);
    }
  }
}
function corridorAt(lanes, bearingDeg) {
  let best;
  for (const lane of lanes) {
    if (!best || Math.abs(lane.bearingDeg - bearingDeg) < Math.abs(best.bearingDeg - bearingDeg))
      best = lane;
  }
  return best;
}

// ../../../../../../../Users/donaldlee/Documents/GitHub-Repos/soon2026/apps/expo-app/src/lib/rover-brain/autopilot.ts
var DEFAULT_AUTOPILOT = {
  alignM: 0.5,
  alignDeg: 5,
  settleMs: 800,
  absentMs: 2500,
  scoopLipM: 0.15,
  scoopWidthM: 0.3,
  scoopPastM: 0.1,
  scoopMaxM: 1,
  scoopClearM: 0.3,
  lowerMs: 500,
  liftMs: 800,
  capacity: 1,
  dumpAtM: 0.6,
  dumpMs: 1500,
  backupM: 0.35,
  pileWeight: 0.5,
  blockedPenaltyM: 2,
  stickM: 2.5,
  approachMaxMs: 90000,
  maxSpeed: 55,
  minSpeed: 28,
  turnSpeed: 35,
  alignTurnSpeed: 22,
  minAlignTurnSpeed: 10,
  alignMaxMs: 15000,
  searchSpeed: 30,
  scoopSpeed: 28,
  backupSpeed: 28,
  turnInPlaceDeg: 25,
  stopDistanceM: 0.45,
  minExploreM: 1,
  exploreMs: 2500,
  detourMs: 3000,
  reverseMs: 1500,
  maxScanAgeMs: 1500
};
var initialAutopilotState = (nowMs, load = 0) => ({
  mode: "searching",
  sinceMs: nowMs,
  load,
  searchedDeg: 0
});
var NO_DETOUR = { detourYawDeg: undefined, detourUntilMs: undefined, reverseUntilMs: undefined };
var clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
var side = (deg) => Math.abs(deg) < 1 ? "ahead" : `${Math.abs(Math.round(deg))}° ${deg > 0 ? "right" : "left"}`;
var horizontal = (a, b) => Math.hypot(a[0] - b[0], a[2] - b[2]);
var scoopFor = (mode) => mode === "scooping" ? "down" : mode === "dumping" ? "dump" : "up";
function out(state, left, right, message, extra = {}) {
  return { state, left, right, scoop: scoopFor(state.mode), message, ...extra };
}
function halt(state, message, extra = {}) {
  return out(state, 0, 0, message, extra);
}
function turnToward(bearingDeg, speed) {
  const s = Math.sign(bearingDeg || 1) * speed;
  return { left: s, right: -s };
}
function pivot(bearingDeg, cfg) {
  return turnToward(bearingDeg, clamp(Math.abs(bearingDeg) / 40 * cfg.turnSpeed, cfg.minAlignTurnSpeed, cfg.turnSpeed));
}
function fineTurn(bearingDeg, cfg) {
  const speed = clamp(Math.abs(bearingDeg) / 20 * cfg.alignTurnSpeed, cfg.minAlignTurnSpeed, cfg.alignTurnSpeed);
  return turnToward(bearingDeg, speed);
}
function arc(bearingDeg, speed, maxTurn = 0.5) {
  const turn = clamp(bearingDeg / 60, -maxTurn, maxTurn);
  return { left: speed * (1 + turn), right: speed * (1 - turn) };
}
function isOpen(lane, needM, cfg) {
  if (!lane)
    return false;
  return lane.blockedBy == null || lane.clearanceM >= Math.max(needM, cfg.stopDistanceM);
}
function arcOpen(lanes, bearingDeg, cfg) {
  const lo = Math.min(0, bearingDeg) - 4;
  const hi = Math.max(0, bearingDeg) + 4;
  return lanes.every((c) => c.bearingDeg < lo || c.bearingDeg > hi || isOpen(c, cfg.stopDistanceM, cfg));
}
function inPile(world, pile) {
  return pile != null && horizontal(world, pile.center) <= pile.radiusM;
}
function chooseTarget(leaves, heading, lanes, cfg = DEFAULT_AUTOPILOT, preferId, pile = null) {
  let best = null;
  for (const leaf of leaves) {
    if (leaf.state !== "confirmed" || inPile(leaf.world, pile))
      continue;
    const r = toRover(leaf.world, heading);
    const lane = Math.abs(r.bearingDeg) <= 50 ? corridorAt(lanes, r.bearingDeg) : undefined;
    const blocked = Math.abs(r.bearingDeg) <= 50 && !isOpen(lane, r.distance - cfg.alignM, cfg);
    const cost = r.distance + (pile ? cfg.pileWeight * horizontal(leaf.world, pile.center) : 0) + (blocked ? cfg.blockedPenaltyM : 0) - (leaf.id === preferId ? cfg.stickM : 0);
    if (!best || cost < best.cost)
      best = { leaf, distance: r.distance, bearingDeg: r.bearingDeg, cost };
  }
  return best && { leaf: best.leaf, distance: best.distance, bearingDeg: best.bearingDeg };
}
function sweptLeaves(leaves, start, heading, cfg = DEFAULT_AUTOPILOT) {
  const travelled = horizontal(heading.origin, start);
  const d = sub(heading.origin, start);
  const dir = travelled > 0.05 ? [d[0] / travelled, d[2] / travelled] : [heading.forward[0], heading.forward[2]];
  return leaves.filter((leaf) => {
    if (leaf.state !== "confirmed" && leaf.state !== "tentative")
      return false;
    const x = leaf.world[0] - start[0];
    const z = leaf.world[2] - start[2];
    const along = x * dir[0] + z * dir[1];
    const across = x * dir[1] - z * dir[0];
    return along >= cfg.scoopLipM && along <= travelled + cfg.scoopLipM && Math.abs(across) <= cfg.scoopWidthM / 2;
  }).map((leaf) => leaf.id);
}
function stepAutopilot(state, input, cfg = DEFAULT_AUTOPILOT) {
  const { heading } = input;
  if (!heading)
    return halt(state, "Stopped: tracking lost");
  if (!(input.scanAgeMs <= cfg.maxScanAgeMs))
    return halt(state, "Stopped: no fresh LiDAR depth");
  switch (state.mode) {
    case "aligning":
      return align(state, input, heading, cfg);
    case "scooping":
      return scoop(state, input, heading, cfg);
    case "lifting":
      return lift(state, input, heading, cfg);
    case "returning":
      return returnToPile(state, input, heading, cfg);
    case "dumping":
      return dump(state, input, heading, cfg);
    case "backing":
      return backUp(state, input, heading, cfg);
    default:
      return seek(state, input, heading, cfg);
  }
}
function seek(state, input, heading, cfg) {
  const { nowMs, leaves, corridors: lanes, pile } = input;
  if (state.load >= cfg.capacity) {
    return pile ? returnToPile({ ...state, mode: "returning", sinceMs: nowMs }, input, heading, cfg) : halt(state, "Scoop full: set a pile");
  }
  const detouring = state.mode === "approaching" && state.detourUntilMs != null && nowMs < state.detourUntilMs;
  const current = detouring ? leaves.find((l) => l.id === state.targetId && l.state === "confirmed") : undefined;
  const target = current ? { leaf: current, ...toRover(current.world, heading) } : chooseTarget(leaves, heading, lanes, cfg, state.targetId, pile);
  if (target) {
    const { leaf, distance, bearingDeg } = target;
    const same = state.mode === "approaching" && state.targetId === leaf.id;
    const next = {
      ...state,
      mode: "approaching",
      sinceMs: same ? state.sinceMs : nowMs,
      targetId: leaf.id,
      searchedDeg: 0,
      ...same ? {} : NO_DETOUR
    };
    const label = `leaf #${leaf.id} · ${distance.toFixed(1)} m · ${side(bearingDeg)}`;
    if (nowMs - next.sinceMs > cfg.approachMaxMs) {
      return halt(initialAutopilotState(nowMs, state.load), `Can't get to leaf #${leaf.id}; giving up on it`, {
        failedId: leaf.id
      });
    }
    if (distance <= cfg.alignM) {
      return halt({ ...next, mode: "aligning", sinceMs: nowMs, alignedAtMs: undefined, ...NO_DETOUR }, `Reached ${label}`);
    }
    return navigate(next, leaf.world, cfg.alignM, 0, label, input, heading, cfg);
  }
  if (state.mode === "exploring" && state.exploreBearingDeg != null && nowMs - state.sinceMs < cfg.exploreMs) {
    const lane = corridorAt(lanes, state.exploreBearingDeg);
    if (isOpen(lane, cfg.minExploreM, cfg)) {
      const { left, right } = arc(state.exploreBearingDeg, cfg.minSpeed);
      return out(state, left, right, `Exploring ${side(state.exploreBearingDeg)}`);
    }
  }
  let searched = state.mode === "searching" ? state.searchedDeg : 0;
  if (state.mode === "searching" && state.lastYawDeg != null) {
    searched += Math.abs(angleDiffDeg(heading.yawDeg, state.lastYawDeg));
  }
  const searching = {
    mode: "searching",
    sinceMs: state.mode === "searching" ? state.sinceMs : nowMs,
    load: state.load,
    searchedDeg: searched,
    lastYawDeg: heading.yawDeg
  };
  if (searched >= 360) {
    if (state.load > 0 && pile) {
      return halt({ ...searching, mode: "returning", sinceMs: nowMs, searchedDeg: 0 }, "No more leaves here; to the pile");
    }
    const open = lanes.filter((c) => isOpen(c, cfg.minExploreM, cfg)).sort((a, b) => Math.abs(a.bearingDeg) - Math.abs(b.bearingDeg) || b.clearanceM - a.clearanceM)[0];
    if (open) {
      return halt({ ...searching, mode: "exploring", sinceMs: nowMs, searchedDeg: 0, exploreBearingDeg: open.bearingDeg }, `Nothing here; exploring ${side(open.bearingDeg)}`);
    }
    searching.searchedDeg = 0;
  }
  const pending = leaves.some((l) => l.state === "tentative" && nowMs - l.lastSeenMs < 1000 && !inPile(l.world, pile));
  if (pending)
    return halt(searching, "Checking a possible leaf");
  return out(searching, cfg.searchSpeed, -cfg.searchSpeed, `Searching · ${Math.round(searched)}°`);
}
function align(state, input, heading, cfg) {
  const { nowMs, leaves, corridors: lanes } = input;
  const leaf = leaves.find((l) => l.id === state.targetId);
  if (!leaf || leaf.state !== "confirmed")
    return seek({ ...state, mode: "searching", sinceMs: nowMs }, input, heading, cfg);
  const r = toRover(leaf.world, heading);
  if (r.distance > cfg.alignM + 0.3)
    return seek({ ...state, mode: "approaching" }, input, heading, cfg);
  const label = `leaf #${leaf.id}`;
  if (nowMs - state.sinceMs > cfg.alignMaxMs) {
    return halt(initialAutopilotState(nowMs, state.load), `Couldn't line up ${label}; giving up on it`, { failedId: leaf.id });
  }
  if (Math.abs(r.bearingDeg) > cfg.alignDeg) {
    const { left, right } = fineTurn(r.bearingDeg, cfg);
    return out({ ...state, alignedAtMs: undefined }, left, right, `Lining up ${label} · ${side(r.bearingDeg)}`);
  }
  const alignedAtMs = state.alignedAtMs ?? nowMs;
  const lined = { ...state, alignedAtMs };
  if (nowMs - alignedAtMs < cfg.settleMs)
    return halt(lined, `Lining up ${label}`);
  if (leaf.lastSeenMs < alignedAtMs) {
    if (nowMs - alignedAtMs < cfg.absentMs)
      return halt(lined, `Looking for ${label}`);
    return halt(initialAutopilotState(nowMs, state.load), `${label} isn't there any more`, { missingId: leaf.id });
  }
  const travel = r.forward - cfg.scoopLipM + cfg.scoopPastM;
  const ahead = corridorAt(lanes, 0);
  if (ahead && ahead.blockedBy != null && ahead.clearanceM < travel + cfg.scoopClearM) {
    return halt(initialAutopilotState(nowMs, state.load), `Can't scoop ${label}: something behind it`, { failedId: leaf.id });
  }
  return halt({ ...lined, mode: "scooping", sinceMs: nowMs, startedAt: heading.origin }, `Scooping ${label}`);
}
function scoop(state, input, heading, cfg) {
  const { nowMs, leaves, corridors: lanes } = input;
  const label = `leaf #${state.targetId}`;
  if (nowMs - state.sinceMs < cfg.lowerMs)
    return halt(state, `Lowering the scoop for ${label}`);
  const start = state.startedAt ?? heading.origin;
  const travelled = horizontal(heading.origin, start);
  const leaf = leaves.find((l) => l.id === state.targetId);
  const r = leaf ? toRover(leaf.world, heading) : null;
  const passed = !r || r.forward <= cfg.scoopLipM - cfg.scoopPastM;
  const ahead = corridorAt(lanes, 0);
  const blocked = ahead != null && ahead.blockedBy != null && ahead.clearanceM < cfg.scoopClearM;
  if (passed || blocked || travelled >= cfg.scoopMaxM) {
    const collectedIds = sweptLeaves(leaves, start, heading, cfg);
    return halt({ ...state, mode: "lifting", sinceMs: nowMs }, blocked ? `Blocked; lifting the scoop` : `Lifting ${label}`, {
      collectedIds
    });
  }
  const bearing = r && r.forward > cfg.scoopLipM + 0.1 ? r.bearingDeg : 0;
  const { left, right } = arc(bearing, cfg.scoopSpeed, 0.3);
  return out(state, left, right, `Scooping ${label}`);
}
function lift(state, input, heading, cfg) {
  const { nowMs, pile } = input;
  if (nowMs - state.sinceMs < cfg.liftMs)
    return halt(state, "Lifting the scoop");
  const load = state.load + 1;
  if (load >= cfg.capacity) {
    return pile ? halt({ ...initialAutopilotState(nowMs, load), mode: "returning" }, "Scoop full; to the pile") : halt(initialAutopilotState(nowMs, load), "Scoop full: set a pile");
  }
  return halt({ ...initialAutopilotState(nowMs, load), lastYawDeg: heading.yawDeg }, `Scooped (${load}/${cfg.capacity})`);
}
function returnToPile(state, input, heading, cfg) {
  const { nowMs, corridors: lanes, pile } = input;
  if (!pile)
    return halt(state, "Scoop full: set a pile");
  const r = toRover(pile.center, heading);
  const label = `pile · ${r.distance.toFixed(1)} m · ${side(r.bearingDeg)}`;
  const ahead = corridorAt(lanes, 0);
  const pileAhead = r.distance <= pile.radiusM + cfg.stopDistanceM && !isOpen(ahead, cfg.stopDistanceM, cfg);
  const arrived = r.distance <= cfg.dumpAtM || pileAhead;
  if (arrived && Math.abs(r.bearingDeg) <= cfg.alignDeg * 2) {
    return halt({ ...state, mode: "dumping", sinceMs: nowMs, ...NO_DETOUR }, "Dumping onto the pile");
  }
  if (arrived) {
    const { left, right } = fineTurn(r.bearingDeg, cfg);
    return out({ ...state, ...NO_DETOUR }, left, right, `Turning to the ${label}`);
  }
  return navigate(state, pile.center, cfg.dumpAtM, pile.radiusM, `the ${label}`, input, heading, cfg);
}
function navigate(state, goal, stopAtM, goalRadiusM, label, input, heading, cfg) {
  const { nowMs, corridors: lanes } = input;
  const r = toRover(goal, heading);
  if (state.reverseUntilMs != null && nowMs < state.reverseUntilMs) {
    return out(state, -cfg.backupSpeed, -cfg.backupSpeed, `Boxed in; backing up from ${label}`);
  }
  const detouring = state.detourYawDeg != null && state.detourUntilMs != null && nowMs < state.detourUntilMs;
  const direct = { ...state, ...NO_DETOUR };
  if (!detouring) {
    if (Math.abs(r.bearingDeg) > cfg.turnInPlaceDeg) {
      const { left, right } = pivot(r.bearingDeg, cfg);
      return out(direct, left, right, `Turning to ${label}`);
    }
    const lane = corridorAt(lanes, r.bearingDeg);
    const open = isOpen(lane, r.distance - stopAtM, cfg) || goalRadiusM > 0 && lane?.blockedBy != null && lane.clearanceM >= r.distance - goalRadiusM;
    if (open && !arcOpen(lanes, r.bearingDeg, cfg)) {
      const { left, right } = pivot(r.bearingDeg, cfg);
      return out(direct, left, right, `Turning to ${label}`);
    }
    if (open) {
      const speed = clamp((r.distance - stopAtM) * 70 + cfg.minSpeed * 0.5, cfg.minSpeed, cfg.maxSpeed);
      const { left, right } = arc(r.bearingDeg, speed);
      return out(direct, left, right, `To ${label}`);
    }
  } else {
    const bearing = angleDiffDeg(state.detourYawDeg, heading.yawDeg);
    if (Math.abs(bearing) > cfg.turnInPlaceDeg || !arcOpen(lanes, bearing, cfg)) {
      const { left, right } = pivot(bearing, cfg);
      return out(state, left, right, `Detouring to ${label}`);
    }
    if (isOpen(corridorAt(lanes, bearing), cfg.minExploreM, cfg)) {
      const { left, right } = arc(bearing, cfg.minSpeed);
      return out(state, left, right, `Detouring to ${label}`);
    }
  }
  const best = lanes.filter((c) => isOpen(c, cfg.minExploreM, cfg)).sort((a, b) => Math.abs(a.bearingDeg - r.bearingDeg) - Math.abs(b.bearingDeg - r.bearingDeg))[0];
  if (best) {
    return halt({ ...state, detourYawDeg: heading.yawDeg + best.bearingDeg, detourUntilMs: nowMs + cfg.detourMs }, `Way to ${label} blocked; detouring ${side(best.bearingDeg)}`);
  }
  const room = (sign) => lanes.filter((c) => Math.sign(c.bearingDeg) === sign).reduce((sum, c) => sum + c.clearanceM, 0);
  const turn = room(1) >= room(-1) ? 1 : -1;
  return halt({
    ...state,
    reverseUntilMs: nowMs + cfg.reverseMs,
    detourYawDeg: heading.yawDeg + turn * 90,
    detourUntilMs: nowMs + cfg.reverseMs + cfg.detourMs
  }, `Boxed in on the way to ${label}; backing up, then ${turn > 0 ? "right" : "left"}`);
}
function dump(state, input, heading, cfg) {
  const { nowMs } = input;
  if (nowMs - state.sinceMs < cfg.dumpMs)
    return halt(state, "Dumping onto the pile");
  return halt({ ...initialAutopilotState(nowMs), mode: "backing", startedAt: heading.origin }, `Dumped ${state.load} scoop${state.load === 1 ? "" : "s"}`, { dumped: state.load });
}
function backUp(state, input, heading, cfg) {
  const { nowMs } = input;
  const travelled = horizontal(heading.origin, state.startedAt ?? heading.origin);
  if (travelled >= cfg.backupM || nowMs - state.sinceMs > 4000) {
    return halt({ ...initialAutopilotState(nowMs), lastYawDeg: heading.yawDeg }, "Backed off the pile");
  }
  return out(state, -cfg.backupSpeed, -cfg.backupSpeed, "Backing off the pile");
}
// ../../../../../../../Users/donaldlee/Documents/GitHub-Repos/soon2026/apps/expo-app/src/lib/rover-brain/leaf-map.ts
var DEFAULT_LEAF_MAP = {
  mergeRadiusM: 0.25,
  confirmHits: 3,
  forgetTentativeMs: 4000,
  maxLeaves: 300,
  missGraceMs: 2000,
  maxAttempts: 3
};
var horizontalDistance = (a, b) => {
  const d = sub(a, b);
  return Math.hypot(d[0], d[2]);
};

class LeafMap {
  options;
  leaves = [];
  nextId = 1;
  constructor(options = DEFAULT_LEAF_MAP) {
    this.options = options;
  }
  observe(observations, nowMs) {
    const ids = observations.map((obs) => {
      const match = this.nearest(obs.world);
      if (match) {
        if (!(match.source === "lidar" && obs.source === "floor")) {
          const weight = Math.min(match.hits, 8);
          match.world = match.world.map((v, i) => (v * weight + obs.world[i]) / (weight + 1));
          if (obs.source === "lidar")
            match.source = "lidar";
        }
        match.hits += 1;
        match.score = Math.max(match.score, obs.score);
        match.lastSeenMs = nowMs;
        if (match.state === "tentative" && match.hits >= this.options.confirmHits)
          match.state = "confirmed";
        if (match.state === "collected" && nowMs - (match.collectedAtMs ?? 0) > this.options.missGraceMs) {
          match.missHits += 1;
          if (match.missHits >= this.options.confirmHits) {
            match.state = match.attempts >= this.options.maxAttempts ? "failed" : "confirmed";
          }
        }
        return match.id;
      }
      const leaf = {
        id: this.nextId++,
        world: [...obs.world],
        label: obs.label,
        score: obs.score,
        hits: 1,
        state: this.options.confirmHits <= 1 ? "confirmed" : "tentative",
        source: obs.source,
        firstSeenMs: nowMs,
        lastSeenMs: nowMs,
        attempts: 0,
        missHits: 0
      };
      this.leaves.push(leaf);
      return leaf.id;
    });
    this.prune(nowMs);
    return ids;
  }
  markCollected(id, nowMs) {
    const leaf = this.leaves.find((l) => l.id === id);
    if (!leaf)
      return;
    leaf.state = "collected";
    leaf.attempts += 1;
    leaf.collectedAtMs = nowMs;
    leaf.missHits = 0;
  }
  markFailed(id) {
    const leaf = this.leaves.find((l) => l.id === id);
    if (leaf)
      leaf.state = "failed";
  }
  get(id) {
    const leaf = this.leaves.find((l) => l.id === id);
    return leaf && { ...leaf, world: [...leaf.world] };
  }
  snapshot() {
    return this.leaves.map((l) => ({ ...l, world: [...l.world] }));
  }
  clear() {
    this.leaves = [];
  }
  nearest(world) {
    let best;
    let bestDistance = this.options.mergeRadiusM;
    for (const leaf of this.leaves) {
      const d = horizontalDistance(leaf.world, world);
      if (d <= bestDistance) {
        best = leaf;
        bestDistance = d;
      }
    }
    return best;
  }
  prune(nowMs) {
    this.leaves = this.leaves.filter((l) => l.state !== "tentative" || nowMs - l.lastSeenMs <= this.options.forgetTentativeMs);
    if (this.leaves.length > this.options.maxLeaves) {
      const rank = (l) => l.state === "tentative" ? 0 : l.state === "confirmed" ? 2 : 1;
      this.leaves.sort((a, b) => rank(a) - rank(b) || a.lastSeenMs - b.lastSeenMs);
      this.leaves = this.leaves.slice(this.leaves.length - this.options.maxLeaves);
    }
  }
}

// ../../../../../../../Users/donaldlee/Documents/GitHub-Repos/soon2026/apps/expo-app/src/lib/rover-brain/brain.ts
var DEFAULT_BRAIN = {
  autopilot: DEFAULT_AUTOPILOT,
  leafMap: DEFAULT_LEAF_MAP,
  obstacleMemory: DEFAULT_OBSTACLE_MEMORY,
  rover: DEFAULT_ROVER,
  selfMaskM: 0.35
};

class Brain {
  options;
  leaves;
  obstacles;
  pile = null;
  autopilot = initialAutopilotState(0);
  running = false;
  dumped = 0;
  constructor(options = {}) {
    this.options = { ...DEFAULT_BRAIN, ...options };
    this.leaves = new LeafMap(this.options.leafMap);
    this.obstacles = new ObstacleMemory(this.options.obstacleMemory);
  }
  observe(observations, frameHeading, nowMs) {
    const carrying = this.autopilot.load > 0;
    const kept = carrying && frameHeading ? observations.filter((o) => toRover(o.world, frameHeading).distance >= this.options.selfMaskM) : observations;
    return this.leaves.observe(kept, nowMs);
  }
  addScan(scan, nowMs) {
    this.obstacles.add(scan, nowMs);
  }
  setPile(pile) {
    this.pile = pile;
  }
  start(nowMs) {
    this.autopilot = initialAutopilotState(nowMs, this.autopilot.load);
    this.running = true;
  }
  stop() {
    this.running = false;
  }
  reset() {
    this.leaves.clear();
  }
  tick(nowMs, heading) {
    const lanes = heading ? corridors(this.obstacles.scan(nowMs), heading, this.options.rover) : [];
    const scanAgeMs = this.obstacles.ageMs(nowMs);
    if (!this.running)
      return { corridors: lanes, scanAgeMs };
    const step = stepAutopilot(this.autopilot, { nowMs, heading, leaves: this.leaves.snapshot(), corridors: lanes, scanAgeMs, pile: this.pile }, this.options.autopilot);
    this.autopilot = step.state;
    for (const id of step.collectedIds ?? [])
      this.leaves.markCollected(id, nowMs);
    if (step.missingId != null)
      this.leaves.markCollected(step.missingId, nowMs);
    if (step.failedId != null)
      this.leaves.markFailed(step.failedId);
    if (step.dumped)
      this.dumped += step.dumped;
    return { corridors: lanes, scanAgeMs, step };
  }
}
// ../../../../../../../Users/donaldlee/Documents/GitHub-Repos/soon2026/apps/expo-app/src/lib/rover-brain/sim.ts
var DEFAULT_SIM = {
  tickMs: 200,
  substeps: 4,
  detectEveryMs: 400,
  topSpeedMps: 0.6,
  trackWidthM: 0.3,
  cameraHeightM: 0.3,
  cameraPitchDeg: 20,
  intrinsics: { fx: 1400, fy: 1400, cx: 720, cy: 960, width: 1440, height: 1920 },
  detectRangeM: 5,
  detectNearP: 0.9,
  detectFarP: 0.3,
  noiseM: 0.01,
  noisePerM: 0.02,
  falsePerFrame: 0,
  scoopSuccess: 0.9,
  scoopDepthM: 0.2,
  axleBackM: 0.12,
  bodyRadiusM: 0.2,
  seed: 1
};
function random(seed) {
  let a = seed >>> 0;
  return () => {
    a = a + 1831565813 >>> 0;
    let t = a;
    t = Math.imul(t ^ t >>> 15, t | 1);
    t ^= t + Math.imul(t ^ t >>> 7, t | 61);
    return ((t ^ t >>> 14) >>> 0) / 4294967296;
  };
}
function cylinderPoints(o) {
  const points = [];
  const top = Math.min(o.heightM, 0.6);
  for (let a = 0;a < 360; a += 10) {
    const x = o.center[0] + o.radiusM * Math.sin(a * Math.PI / 180);
    const z = o.center[1] + o.radiusM * Math.cos(a * Math.PI / 180);
    for (let y = 0.02;y <= top; y += 0.05)
      points.push([x, y, z]);
    for (let f = 0.25;f < 1; f += 0.25) {
      points.push([o.center[0] + (x - o.center[0]) * f, top, o.center[1] + (z - o.center[1]) * f]);
    }
  }
  return points;
}

class RoverSim {
  brain;
  options;
  world;
  timeMs = 0;
  leaves;
  x;
  z;
  yawDeg;
  scoop = "up";
  collisions = 0;
  rand;
  wheels = { left: 0, right: 0 };
  lastDetectMs = -Infinity;
  colliding = false;
  obstaclePoints;
  tried = new Set;
  lastTick = { corridors: [], scanAgeMs: Infinity };
  constructor(world, options = {}, brain = new Brain) {
    this.world = world;
    this.options = { ...DEFAULT_SIM, ...options };
    this.brain = brain;
    this.rand = random(this.options.seed);
    this.leaves = world.leaves.map(([x, z], i) => ({ id: i + 1, position: [x, 0, z], where: "ground" }));
    this.x = world.start?.position[0] ?? 0;
    this.z = world.start?.position[1] ?? 0;
    this.yawDeg = world.start?.yawDeg ?? 0;
    this.obstaclePoints = [...(world.obstacles ?? []).flatMap(cylinderPoints), ...world.points ?? []];
    if (world.pile)
      brain.setPile(world.pile);
    this.classify();
  }
  get pose() {
    return makePose([this.x, this.options.cameraHeightM, this.z], this.options.cameraPitchDeg, this.yawDeg);
  }
  get heading() {
    return headingFromPose(this.pose);
  }
  start() {
    this.brain.start(this.timeMs);
  }
  setPile(pile) {
    this.world.pile = pile ?? undefined;
    this.brain.setPile(pile);
    this.classify();
  }
  step() {
    const o = this.options;
    const pose = this.pose;
    const view = { pose, intrinsics: o.intrinsics };
    const heading = this.heading;
    if (this.timeMs - this.lastDetectMs >= o.detectEveryMs) {
      this.lastDetectMs = this.timeMs;
      this.brain.observe(this.detect(view, heading), heading, this.timeMs);
    }
    this.brain.addScan(this.scan(view), this.timeMs);
    const tick = this.brain.tick(this.timeMs, heading);
    this.lastTick = tick;
    const step = tick.step;
    this.wheels = step ? { left: step.left, right: step.right } : { left: 0, right: 0 };
    this.setScoop(step?.scoop ?? this.scoop);
    const dt = o.tickMs / 1000 / o.substeps;
    for (let i = 0;i < o.substeps; i++) {
      this.move(dt);
      if (this.scoop === "down")
        this.pickUp();
    }
    this.timeMs += o.tickMs;
    this.classify();
    return this.frame();
  }
  run(done, maxMs) {
    let frame = this.frame();
    const end = this.timeMs + maxMs;
    while (this.timeMs < end && !done(frame))
      frame = this.step();
    return frame;
  }
  frame() {
    const count = (where) => this.leaves.filter((l) => l.where === where).length;
    return {
      timeMs: this.timeMs,
      rover: { position: [this.x, this.options.cameraHeightM, this.z], yawDeg: this.yawDeg, pose: this.pose },
      scoop: this.scoop,
      leaves: this.leaves.map((l) => ({ ...l, position: [...l.position] })),
      tracked: this.brain.leaves.snapshot(),
      tick: this.lastTick,
      pile: this.brain.pile,
      stats: {
        onGround: count("ground"),
        inScoop: count("scoop"),
        inPile: count("pile"),
        collisions: this.collisions,
        dumped: this.brain.dumped
      }
    };
  }
  gaussian() {
    const u = Math.max(this.rand(), 0.000000000001);
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * this.rand());
  }
  detect(view, heading) {
    const o = this.options;
    const seen = [];
    const sighting = (truth, distance) => {
      const sd = o.noiseM + o.noisePerM * distance;
      seen.push({
        world: [truth[0] + this.gaussian() * sd, 0, truth[2] + this.gaussian() * sd],
        label: "dry leaf",
        score: 0.3,
        source: distance <= MAX_RANGE_M - 1 ? "lidar" : "floor"
      });
    };
    for (const leaf of this.leaves) {
      if (leaf.where === "scoop" || !inView(leaf.position, view, o.detectRangeM))
        continue;
      const d = toRover(leaf.position, heading).distance;
      const p = o.detectNearP + (o.detectFarP - o.detectNearP) * Math.min(1, d / o.detectRangeM);
      if (this.rand() < p)
        sighting(leaf.position, d);
    }
    const falses = Math.floor(o.falsePerFrame) + (this.rand() < o.falsePerFrame % 1 ? 1 : 0);
    for (let i = 0;i < falses; i++) {
      const forward = 0.5 + this.rand() * (o.detectRangeM - 0.5);
      const right = (this.rand() - 0.5) * forward;
      const p = [
        heading.origin[0] + heading.forward[0] * forward + heading.right[0] * right,
        0,
        heading.origin[2] + heading.forward[2] * forward + heading.right[2] * right
      ];
      if (inView(p, view, o.detectRangeM))
        sighting(p, forward);
    }
    return seen;
  }
  scan(view) {
    const points = [...this.obstaclePoints, ...this.moundPoints()].filter((p) => inView(p, view));
    return { points, floorY: 0, floorSource: "lidar", timestamp: this.timeMs / 1000, view };
  }
  moundPoints() {
    const piled = this.leaves.filter((l) => l.where === "pile");
    if (!piled.length || !this.world.pile)
      return [];
    const c = this.world.pile.center;
    const heightM = Math.min(0.35, 0.006 * piled.length);
    const radiusM = Math.min(this.world.pile.radiusM * 0.5, 0.1 + 0.015 * piled.length);
    return cylinderPoints({ center: [c[0], c[2]], radiusM, heightM });
  }
  move(dt) {
    const o = this.options;
    const left = this.wheels.left / 100 * o.topSpeedMps;
    const right = this.wheels.right / 100 * o.topSpeedMps;
    const v = (left + right) / 2;
    const omegaDeg = (left - right) / o.trackWidthM * 180 / Math.PI;
    const rad = (deg) => deg * Math.PI / 180;
    let ax = this.x - Math.sin(rad(this.yawDeg)) * o.axleBackM;
    let az = this.z + Math.cos(rad(this.yawDeg)) * o.axleBackM;
    const mid = rad(this.yawDeg + omegaDeg * dt / 2);
    ax += Math.sin(mid) * v * dt;
    az -= Math.cos(mid) * v * dt;
    this.yawDeg += omegaDeg * dt;
    this.x = ax + Math.sin(rad(this.yawDeg)) * o.axleBackM;
    this.z = az - Math.cos(rad(this.yawDeg)) * o.axleBackM;
    const h = this.heading;
    for (const leaf of this.leaves) {
      if (leaf.where !== "scoop")
        continue;
      leaf.position = [h.origin[0] + h.forward[0] * 0.08, 0.1, h.origin[2] + h.forward[2] * 0.08];
    }
    const bx = this.x - h.forward[0] * o.axleBackM;
    const bz = this.z - h.forward[2] * o.axleBackM;
    const hit = (this.world.obstacles ?? []).some((ob) => Math.hypot(bx - ob.center[0], bz - ob.center[1]) < o.bodyRadiusM + ob.radiusM);
    if (hit && !this.colliding)
      this.collisions += 1;
    this.colliding = hit;
  }
  pickUp() {
    const o = this.options;
    const lip = this.brain.options.autopilot.scoopLipM;
    const width = this.brain.options.autopilot.scoopWidthM;
    const h = this.heading;
    for (const leaf of this.leaves) {
      if (leaf.where === "scoop" || this.tried.has(leaf.id))
        continue;
      const r = toRover(leaf.position, h);
      if (Math.abs(r.right) > width / 2 || r.forward > lip || r.forward < lip - o.scoopDepthM)
        continue;
      this.tried.add(leaf.id);
      if (this.rand() < o.scoopSuccess)
        leaf.where = "scoop";
    }
  }
  setScoop(position) {
    if (position === this.scoop)
      return;
    if (position === "down")
      this.tried.clear();
    if (position === "dump") {
      const h = this.heading;
      const lip = this.brain.options.autopilot.scoopLipM + 0.05;
      for (const leaf of this.leaves) {
        if (leaf.where !== "scoop")
          continue;
        leaf.position = [
          h.origin[0] + h.forward[0] * lip + this.gaussian() * 0.06,
          0,
          h.origin[2] + h.forward[2] * lip + this.gaussian() * 0.06
        ];
        leaf.where = "ground";
      }
    }
    this.scoop = position;
  }
  classify() {
    const pile = this.world.pile;
    for (const leaf of this.leaves) {
      if (leaf.where === "scoop")
        continue;
      const inside = pile != null && Math.hypot(leaf.position[0] - pile.center[0], leaf.position[2] - pile.center[2]) <= pile.radiusM;
      leaf.where = inside ? "pile" : "ground";
    }
  }
}
// viewer.ts
var pile = { center: [0, 0, 1], radiusM: 1 };
var WORLDS = {
  "Cluttered yard": {
    leaves: [[0, -3], [1.2, -2.6], [-1.5, -3.5], [2.5, -0.5], [-2.2, 0.5], [0.8, -4.2], [-0.6, -1.4], [1.9, -3.8]],
    obstacles: [
      { center: [0, -1.9], radiusM: 0.25, heightM: 1 },
      { center: [1.6, -1.5], radiusM: 0.3, heightM: 0.4 },
      { center: [-1.6, -2.4], radiusM: 0.2, heightM: 1 }
    ],
    pile
  },
  "Leaves behind a trunk": {
    leaves: [[0, -3], [0.3, -3.4], [-0.4, -3.2]],
    obstacles: [{ center: [0, -1.6], radiusM: 0.25, heightM: 1 }],
    pile
  },
  "Open patch": { leaves: [[0.5, -2], [-1, -1.5], [1.5, -2.5], [-0.3, -3], [2, -1], [-2, -2.5]], pile }
};
var $ = (id) => document.getElementById(id);
var canvas = $("view");
var ctx = canvas.getContext("2d");
var worldSel = $("world");
var speedSel = $("speed");
var seedIn = $("seed");
var noisy = $("noisy");
var playBtn = $("play");
for (const name of Object.keys(WORLDS))
  worldSel.add(new Option(name, name));
var sim2;
var frame;
var trail = [];
var playing = true;
function reset() {
  const opts = noisy.checked ? { falsePerFrame: 0.3, noisePerM: 0.04, scoopSuccess: 0.7 } : {};
  sim2 = new RoverSim(structuredClone(WORLDS[worldSel.value]), { seed: Number(seedIn.value) || 1, ...opts });
  sim2.start();
  frame = sim2.frame();
  trail = [];
  draw();
}
var VIEW = { x0: -3.5, x1: 3.5, z0: -5, z1: 2.3 };
var scale = 1;
var ox = 0;
var oy = 0;
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
var px = (x) => ox + x * scale;
var py = (z) => oy + z * scale;
var css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
function circle(x, z, r, fill, stroke, width = 1) {
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
  ctx.strokeStyle = css("--grid");
  ctx.lineWidth = 1;
  for (let x = Math.ceil(VIEW.x0);x <= VIEW.x1; x++) {
    ctx.beginPath();
    ctx.moveTo(px(x), py(VIEW.z0));
    ctx.lineTo(px(x), py(VIEW.z1));
    ctx.stroke();
  }
  for (let z = Math.ceil(VIEW.z0);z <= VIEW.z1; z++) {
    ctx.beginPath();
    ctx.moveTo(px(VIEW.x0), py(z));
    ctx.lineTo(px(VIEW.x1), py(z));
    ctx.stroke();
  }
  if (frame.pile) {
    const c = frame.pile.center;
    circle(c[0], c[2], frame.pile.radiusM, css("--pile-fill"), css("--pile"), 1.5);
    ctx.fillStyle = css("--pile");
    ctx.font = "600 12px system-ui";
    ctx.textAlign = "center";
    ctx.fillText("PILE", px(c[0]), py(c[2] + frame.pile.radiusM) - 8);
  }
  for (const o of sim2.world.obstacles ?? [])
    circle(o.center[0], o.center[1], o.radiusM, css("--obstacle"));
  ctx.strokeStyle = css("--trail");
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  trail.forEach(([x, z], i) => i ? ctx.lineTo(px(x), py(z)) : ctx.moveTo(px(x), py(z)));
  ctx.stroke();
  const yaw = frame.rover.yawDeg * Math.PI / 180;
  const cam = frame.rover.position;
  ctx.fillStyle = css("--fov");
  ctx.beginPath();
  ctx.moveTo(px(cam[0]), py(cam[2]));
  for (let a = -27;a <= 27; a += 3) {
    const b = yaw + a * Math.PI / 180;
    ctx.lineTo(px(cam[0] + Math.sin(b) * 5), py(cam[2] - Math.cos(b) * 5));
  }
  ctx.closePath();
  ctx.fill();
  for (const c of frame.tick.corridors) {
    const b = yaw + c.bearingDeg * Math.PI / 180;
    const d = Math.min(c.clearanceM, 2.5);
    ctx.strokeStyle = c.blockedBy ? css("--blocked") : css("--open");
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(px(cam[0]), py(cam[2]));
    ctx.lineTo(px(cam[0] + Math.sin(b) * d), py(cam[2] - Math.cos(b) * d));
    ctx.stroke();
  }
  for (const l of frame.leaves) {
    if (l.where === "scoop")
      continue;
    circle(l.position[0], l.position[2], 0.06, l.where === "pile" ? css("--leaf-piled") : css("--leaf"));
  }
  const target = frame.tick.step?.state.targetId;
  for (const t of frame.tracked) {
    if (t.state === "collected")
      continue;
    const color = t.state === "failed" ? css("--blocked") : t.state === "tentative" ? css("--muted") : css("--belief");
    circle(t.world[0], t.world[2], t.id === target ? 0.16 : 0.11, undefined, color, t.id === target ? 2.5 : 1.25);
  }
  const fwd = [Math.sin(yaw), -Math.cos(yaw)];
  const ax = cam[0] - fwd[0] * sim2.options.axleBackM;
  const az = cam[2] - fwd[1] * sim2.options.axleBackM;
  circle(ax, az, sim2.options.bodyRadiusM, css("--rover-fill"), css("--rover"), 2);
  const lip = sim2.brain.options.autopilot.scoopLipM;
  const half = sim2.brain.options.autopilot.scoopWidthM / 2;
  const right = [-fwd[1], fwd[0]];
  const scoopColor = frame.scoop === "down" ? css("--scoop-down") : frame.scoop === "dump" ? css("--pile") : css("--rover");
  ctx.strokeStyle = scoopColor;
  ctx.lineWidth = frame.scoop === "down" ? 5 : 3;
  ctx.beginPath();
  ctx.moveTo(px(cam[0] + fwd[0] * lip - right[0] * half), py(cam[2] + fwd[1] * lip - right[1] * half));
  ctx.lineTo(px(cam[0] + fwd[0] * lip + right[0] * half), py(cam[2] + fwd[1] * lip + right[1] * half));
  ctx.stroke();
  if (frame.stats.inScoop > 0)
    circle(cam[0] + fwd[0] * 0.05, cam[2] + fwd[1] * 0.05, 0.06, css("--leaf"));
  circle(cam[0], cam[2], 0.035, css("--rover"));
  const step = frame.tick.step;
  const goalLeaf = frame.tracked.find((t) => t.id === target);
  const goal = step && ["returning", "dumping"].includes(step.state.mode) && frame.pile ? frame.pile.center : goalLeaf && ["approaching", "aligning", "scooping"].includes(step.state.mode) ? goalLeaf.world : null;
  if (goal) {
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = css("--belief");
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(px(cam[0]), py(cam[2]));
    ctx.lineTo(px(goal[0]), py(goal[2]));
    ctx.stroke();
    ctx.setLineDash([]);
  }
  const s = frame.stats;
  const total = frame.leaves.length;
  $("mode").textContent = step ? step.state.mode : "—";
  $("message").textContent = step?.message ?? "Ready";
  $("time").textContent = `${Math.floor(frame.timeMs / 60000)}:${String(Math.floor(frame.timeMs / 1000 % 60)).padStart(2, "0")}`;
  $("piled").textContent = `${s.inPile} / ${total}`;
  $("ground").textContent = String(s.onGround);
  $("scoop").textContent = `${frame.scoop}${s.inScoop ? ` · ${s.inScoop} leaf` : ""}`;
  $("trips").textContent = String(s.dumped);
  $("hits").textContent = String(s.collisions);
  $("wheels").textContent = step ? `${step.left.toFixed(0)} / ${step.right.toFixed(0)}` : "0 / 0";
  const done = s.onGround === 0 && s.inScoop === 0;
  $("status").textContent = done ? `Done: every leaf is in the pile in ${$("time").textContent}` : "";
}
var last = performance.now();
var carry = 0;
function loop(now) {
  const dt = Math.min(100, now - last);
  last = now;
  const done = frame.stats.onGround === 0 && frame.stats.inScoop === 0;
  if (playing && !done) {
    carry += dt * Number(speedSel.value);
    while (carry >= sim2.options.tickMs) {
      carry -= sim2.options.tickMs;
      frame = sim2.step();
      trail.push([frame.rover.position[0], frame.rover.position[2]]);
      if (frame.stats.onGround === 0 && frame.stats.inScoop === 0)
        break;
    }
  }
  draw();
  requestAnimationFrame(loop);
}
playBtn.onclick = () => {
  playing = !playing;
  playBtn.textContent = playing ? "Pause" : "Play";
};
$("reset").onclick = reset;
worldSel.onchange = reset;
seedIn.onchange = reset;
noisy.onchange = reset;
window.addEventListener("resize", () => {
  layout();
  draw();
});
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", draw);
layout();
reset();
requestAnimationFrame(loop);
