/**
 * WHAT HAPPENS BETWEEN "FIRE" AND THE BALL LEAVING? Per shot, the commit and the release.
 *
 *   npm run tool -- tools/releasecheck.ts [--speed 1.0] [--off 40] [--secs 40] [--seeds 2] [--noscatter]
 *                                          [--cycle 0.6] [--eball 0.8] [--dump] [--wasted] [--emptycell] [--bearing 10] [--minshots 200]
 *
 * --dump prints every shot; --wasted prints the tube's state at every pulse that released
 * nothing (which is how the corner-to-corner arch jam was found).
 *
 * The brain commits a shot when it opens the gate; the ball leaves some tenths of a second
 * later, after the servo travels and the belt lifts the ball into the wheel. Every gate in
 * the brain is evaluated at COMMIT. This records both moments for every shot and prints:
 *
 *   - the commit-to-release delay, and how many pulses released nothing at all
 *   - the brain's solved launch at release against the launch the ball actually got
 *   - the chassis velocity at commit against release (what the lead was solved for vs what
 *     the ball inherited)
 *   - P(land) at commit against P(land) re-evaluated at release
 *   - where the ball came down, split by whether the release was still inside the gates
 *
 * Same patrol as tools/fastfire.ts, so the numbers are comparable.
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { RAD, M_TO_IN, inches } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

interface Shot {
  tCommit: number; tRelease: number;
  pCommit: number; pRelease: number;
  vrCommit: number; vrRelease: number;      // true radial chassis velocity, m/s (+ closing)
  vlCommit: number; vlRelease: number;      // true lateral chassis velocity, m/s
  wantSpeed: number; gotSpeed: number;      // brain's solved exit speed at release vs actual
  wantElev: number; gotElev: number;        // degrees
  aimErrRelease: number;                    // gate's pointing error: filtered ESTIMATE vs axis
  cmdErrRelease: number;                    // servo error: COMMAND vs axis. Near zero = the
                                            // axis is obeying and the gap above is filter lag
  hoodErrRelease: number;
  rangeCommit: number; rangeRelease: number;
  holdAtRelease: string;
  /** Off the mouth's opening at release: what the brain believed, and the truth. */
  openBelieved: number; openTrue: number;
  fill: number; pSpeed: number; pStay: number; pAim: number;
  shotsThisPulse: number;
  long: number; lat: number; landed: boolean;
}

async function run(speed: number, standoff_in: number, secs: number, seed: number, noScatter: boolean) {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  spec.drivetrain.maxSpeed_mps = speed;
  if (noScatter) spec.flywheel.scatter = { angle_deg: 0, yaw_deg: 0, speedFrac: 0 };
  // --eball X: ball-ball restitution, the guessed number that decides whether a ball arriving
  // into a part-full pocket clips the pile and comes back out.
  const cy = process.argv.indexOf('--cycle');
  if (cy >= 0) spec.transfer.cycleTime_s = Number(process.argv[cy + 1]);
  // --turret DPS / --accel DPS2 / --yawcap DPS: the head's slew rate, its acceleration, and the
  // chassis yaw rate a shot is still allowed at. The last is DERIVED from the first two (the
  // feed commits ~0.4 s before release and a yawing chassis drags the setpoint for all of it),
  // which is why they are swept together and not one at a time.
  // --aimalpha A: the pole on the turret COMMAND. The readiness gate measures the axis against
  // a different, faster pole (0.6, hardcoded), so these two disagree by more the faster the
  // bearing moves -- and that disagreement is charged to the turret even when the axis is
  // exactly where it was told to be.
  // --perfect: an IDEAL ROBOT. Every source of error outside the flight itself is removed --
  // odometry noise AND its integrated drift, the tag pipeline's noise, its 75 ms latency and
  // its 30 fps frame rate, the IMU's 40 ms lag, the hub's command latency and voltage noise,
  // the per-ball variation in diameter and mass, and the launch scatter. What is left is the
  // aiming arithmetic, the shot table's interpolation, and the physics of entry. If this is
  // not 100%, the residual is a MODELLING gap and not a sensing one, which is the whole
  // point of having the switch.
  //
  // Individual switches so the residual can be attributed rather than guessed at.
  const ideal = process.argv.includes('--perfect');
  const off = (k: string) => ideal || process.argv.includes(k);
  if (off('--noscatter2') || ideal) spec.flywheel.scatter = { angle_deg: 0, yaw_deg: 0, speedFrac: 0 };
  if (off('--noloc')) {
    spec.sensors.localizer.noise = { xy_in: 0, heading_deg: 0, vel_mps: 0, omegaDps: 0 };
    if (spec.sensors.localizer.drift) spec.sensors.localizer.drift.enabled = false;
  }
  if (off('--notag')) {
    spec.sensors.tag.noise = { bearing_deg: 0, rangeFrac: 0, open_deg: 0 };
    spec.sensors.tag.latencyMs = 0;
    spec.sensors.tag.frameRateHz = 240;
  }
  if (off('--nolag')) {
    spec.hub.imuLatencyMs = 0;
    spec.hub.commandLatencyMs = 0;
    spec.hub.voltageNoise_V = 0;
  }
  if (off('--noballvar')) { p.ball.pollen.dVar = 0; p.ball.pollen.mVar = 0; }
  if (process.argv.includes('--gateraw')) spec.turret.gateOnRawAim = true;
  if (process.argv.includes('--predictenc')) spec.turret.predictEncoder = true;
  const oc = process.argv.indexOf('--opencap');
  if (oc >= 0) spec.turret.fireOpenCap_deg = Number(process.argv[oc + 1]);
  const at = process.argv.indexOf('--aimtol');
  if (at >= 0) spec.turret.fireAimTolDeg = Number(process.argv[at + 1]);
  const gs = process.argv.indexOf('--gatespeed');
  if (gs >= 0) spec.transfer.gateSpeed = Number(process.argv[gs + 1]);
  const aa = process.argv.indexOf('--aimalpha');
  if (aa >= 0) spec.turret.aimFilterAlpha = Number(process.argv[aa + 1]);
  const tq = process.argv.indexOf('--turret');
  if (tq >= 0) spec.turret.speed_dps = Number(process.argv[tq + 1]);
  const ac = process.argv.indexOf('--accel');
  if (ac >= 0) spec.turret.accel_dps2 = Number(process.argv[ac + 1]);
  const yc = process.argv.indexOf('--yawcap');
  if (yc >= 0) spec.turret.fireYawCap_dps = Number(process.argv[yc + 1]);
  const eb = process.argv.indexOf('--eball');
  if (eb >= 0) p.ball.e_ball = Number(process.argv[eb + 1]);
  const pool = Array.from({ length: 400 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: pool, alliance: 'red', seed });
  for (const b of w.balls.balls) w.balls.park(b);
  const hive = w.hives.red;
  const mouth = hive.upCellMouthWorld();
  const n0 = hive.upCellMouthNormalWorld();
  const hn = Math.hypot(n0[0], n0[2]) || 1;
  const nrm: [number, number] = [n0[0] / hn, n0[2] / hn];
  const side: [number, number] = [-nrm[1], nrm[0]];
  const y = spec.chassis.height_m / 2 + spec.chassis.clearance_m;
  const hw = w.geom.halfWidth_m - inches(14);
  const onField = (q: Vec3) => Math.abs(q[0]) <= hw && Math.abs(q[2]) <= hw;
  const onArc = (bearDeg: number): Vec3 => {
    const a = Math.atan2(nrm[0], nrm[1]) + bearDeg * (Math.PI / 180);
    return [mouth[0] + Math.sin(a) * inches(standoff_in), y, mouth[2] + Math.cos(a) * inches(standoff_in)];
  };
  // THE ROUTE AND THE GATE ARE TWO DIFFERENT THINGS. This read the (possibly overridden)
  // fire cap, so tightening the gate silently shortened the patrol as well and the two effects
  // could not be told apart. The route now comes from the SHIPPED value unless --pathcap says
  // otherwise, and --opencap moves the gate alone.
  const pc = process.argv.indexOf('--pathcap');
  const capDeg = pc >= 0
    ? Number(process.argv[pc + 1])
    : ((robotSpec as unknown as RobotSpec).turret.fireOpenCap_deg ?? 60);
  let bestLo = 0, bestHi = 0, curLo: number | null = null;
  for (let b = -capDeg; b <= capDeg; b += 2) {
    if (onField(onArc(b))) {
      if (curLo === null) curLo = b;
      if (b - curLo > bestHi - bestLo) { bestLo = curLo; bestHi = b; }
    } else curLo = null;
  }
  // --reach IN: how far each way the pass runs, overriding the sector-derived length. The
  // default stops at the fire cap, which keeps every frame shootable but makes the run SHORT --
  // 69 in at a 40 in stand-off -- so the robot spends its time reversing and never reaches the
  // speed it was asked for. A longer pass costs sector time and buys cruise speed.
  const rq = process.argv.indexOf('--reach');
  const reachOverride = rq >= 0 ? Number(process.argv[rq + 1]) : 0;
  const sweepDeg = (bestHi - bestLo) / 2;
  // ARC: constant RANGE to the mouth, so the robot sweeps a circle and its radial velocity is
  // near zero -- it is a pure sideways test. STRAIGHT: constant PERPENDICULAR offset, so the
  // robot drives a real chord across the front and the range grows towards each end, which is
  // the only way this rig exercises the closing/receding axis at all.
  const straight = process.argv.includes('--straight');
  const reach = reachOverride > 0 ? reachOverride : standoff_in * Math.sin(sweepDeg * Math.PI / 180);
  // --bearing X: stand at X deg off the mouth's opening instead of the arc's end. With the
  // speed at 0 this is a fixed spot, which is how the pile is measured against the bearing.
  const bArg = process.argv.indexOf('--bearing');
  const start = onArc(bArg >= 0 ? Number(process.argv[bArg + 1]) : bestLo);
  // --nosealong: point the NOSE down the pass and drive FORWARD, letting the turret do all the
  // aiming. Every run above instead yaws the chassis to face the hive and strafes sideways along
  // the pass, which is the worst case twice over: a mecanum strafes at about 70% of its forward
  // speed, and a chassis that must keep its nose on the goal yaws at v/R, which is what trips
  // the firing cap. A turret with 540 deg of travel exists precisely so the chassis does not
  // have to do this.
  const noseAlong = process.argv.includes('--nosealong');
  const alongDeg = Math.atan2(side[0], side[1]) * RAD;
  w.robot.place(start, noseAlong ? alongDeg : Math.atan2(mouth[0] - start[0], mouth[2] - start[2]) * RAD);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  brain.state.firing = true;
  let loaded = 0, dir = 1;

  const shots: Shot[] = [];
  let frames = 0, sumV = 0, openFrames = 0;
  const holds = new Map<string, number>();
  let pending: Partial<Shot> | null = null;
  let wasPulsing = false;
  let wasted = 0;
  let shots0 = w.robot.shots;
  let shotsThisPulse = 0;

  // --emptycell: bench every ball once it has settled in the CELL, so the pocket never fills
  // and never tips. The per-shot result is already recorded by then (World.settleShots), so
  // what this measures is the moving robot against the same empty-pocket ceiling the standing
  // one is measured against (tools/ceiling.ts).
  const emptyCell = process.argv.includes('--emptycell');
  const slowFrames = new Map<number, number>();
  const benchSettled = () => {
    for (const b of w.balls.balls) {
      if (!b.body.isEnabled() || b.state !== 'cell') continue;
      const v = b.body.linvel();
      const n = Math.hypot(v.x, v.y, v.z) < 0.15 ? (slowFrames.get(b.id) ?? 0) + 1 : 0;
      slowFrames.set(b.id, n);
      if (n >= 30) w.balls.park(b);
    }
  };
  const truth = () => {
    const c = w.robot.pos;
    const v = w.robot.body.linvel();
    const dx = mouth[0] - c[0], dz = mouth[2] - c[2];
    const d = Math.hypot(dx, dz) || 1;
    const ux = dx / d, uz = dz / d;
    return { vr: v.x * ux + v.z * uz, vl: -v.x * uz + v.z * ux, range: d * M_TO_IN };
  };

  for (let i = 0; i < 60 * secs; i++) {
    while (w.robot.heldBalls().length < spec.hopper.capacity && loaded < pool.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    const c = w.robot.pos;
    const lat = ((c[0] - mouth[0]) * side[0] + (c[2] - mouth[2]) * side[1]) * M_TO_IN;
    if (lat > reach) dir = -1;
    else if (lat < -reach) dir = 1;
    const g: GamepadState = emptyGamepad();
    // Hold the nose either on the hive (default) or down the pass (--nosealong).
    const wantYaw = noseAlong ? alongDeg : Math.atan2(mouth[0] - c[0], mouth[2] - c[2]) * RAD;
    const yawErr = wrapDeg(wantYaw - w.robot.yaw * RAD);
    g.right_stick_x = -Math.max(-0.2, Math.min(0.2, yawErr / 45));
    if (straight) {
      // THE CHASSIS YAWS TO FACE THE HIVE, so a robot-frame strafe does not hold a straight
      // line -- it spirals, which is the fault the arc path's own comment records. Build the
      // wanted velocity in the FIELD frame and rotate it into the sticks.
      const perp = (c[0] - mouth[0]) * nrm[0] + (c[2] - mouth[2]) * nrm[1];
      const push = Math.max(-0.4, Math.min(0.4, (perp * M_TO_IN - standoff_in) / 20));
      const wx = side[0] * dir - nrm[0] * push;
      const wz = side[1] * dir - nrm[1] * push;
      const loc = w.robot.toLocal([wx, 0, wz]);
      g.left_stick_y = -Math.max(-1, Math.min(1, loc[2]));   // robot forward
      g.left_stick_x = -Math.max(-1, Math.min(1, loc[0]));   // robot left
      void 0;
    } else {
      g.left_stick_x = -dir;
      const rangeNow = Math.hypot(mouth[0] - c[0], mouth[2] - c[2]) * M_TO_IN;
      g.left_stick_y = -Math.max(-0.4, Math.min(0.4, (rangeNow - standoff_in) / 20));
    }
    w.setGamepads(g, emptyGamepad());
    const before = w.robot.shots;
    const act = brain.update(w.sensors(), g, w.seq, 1 / 60);
    const st = brain.state;
    // COMMIT: the pulse starts this frame.
    if (st.pulsing && !wasPulsing) {
      if (pending) {
        wasted++;
        if (process.argv.includes('--wasted')) {
          const snap = w.snapshot().robot;
          const c0 = w.robot.pos;
          const hs = w.robot.heldBalls().map((b) => { const q = w.balls.pos(b); const l = w.robot.toLocal([q[0] - c0[0], q[1] - c0[1], q[2] - c0[2]]); const v = b.body.linvel(); return `${(l[0] * 1000).toFixed(0)}/${(l[1] * 1000).toFixed(0)}/${(l[2] * 1000).toFixed(0)}(vy${(v.y * 1000).toFixed(0)}${b.body.isSleeping() ? 'Z' : ''})`; });
          console.log(`  [wasted] t=${w.t.toFixed(2)} prev commit ${(w.t - (pending.tCommit ?? 0)).toFixed(2)} s ago  held ${snap.hopper.count} inLine ${snap.intake.inLine} gate ${snap.transfer.gate.toFixed(2)} sinceFeed ${snap.transfer.sinceFeed.toFixed(2)} rpm ${snap.flywheel.rpm.toFixed(0)} belt ${JSON.stringify(w.robot.debugFeed)} tOmega ${w.robot.motors.get('transfer')?.omega.toFixed(1)}  balls x/y/z mm: ${hs.join(' ')}  hold "${st.hold}"`);
        }
      }
      shotsThisPulse = 0;
      const t = truth();
      pending = { tCommit: st.lastFeedT, pCommit: st.pLand, vrCommit: t.vr, vlCommit: t.vl, rangeCommit: t.range };
    }
    wasPulsing = st.pulsing;
    const sens = w.sensors();
    w.step(act);
    { const v = w.robot.body.linvel(); frames++; sumV += Math.hypot(v.x, v.z); }
    // WHY IS THE GATE SHUT, over every frame rather than only at release. With a perfect ball
    // and an empty pocket the land rate stops being the interesting number and this is what is
    // left: the fraction of the drive the robot is allowed to shoot at all.
    if (st.ready) openFrames++;
    const why = st.hold || (st.ready ? '' : 'wheel off / not armed');
    if (why) { const k = why.replace(/-?[\d.]+/g, 'N'); holds.set(k, (holds.get(k) ?? 0) + 1); }
    if (w.robot.shots > before) {
      shotsThisPulse++;
      const t = truth();
      const ls = w.robot.lastShot!;
      const rec: Shot = {
        tCommit: pending?.tCommit ?? NaN, tRelease: w.t,
        pCommit: pending?.pCommit ?? NaN, pRelease: st.pLand,
        vrCommit: pending?.vrCommit ?? NaN, vrRelease: t.vr,
        vlCommit: pending?.vlCommit ?? NaN, vlRelease: t.vl,
        wantSpeed: st.leadSpeed, gotSpeed: ls.v_exit,
        wantElev: st.leadElevDeg, gotElev: ls.elevDeg,
        aimErrRelease: st.turretAimErrDeg, cmdErrRelease: st.turretErrDeg, hoodErrRelease: st.hoodErrDeg,
        rangeCommit: pending?.rangeCommit ?? NaN, rangeRelease: t.range,
        holdAtRelease: st.hold,
        openBelieved: brain.target().openDeg, openTrue: sens.game.truth.upCellOpenDeg,
        fill: st.cellFill, pSpeed: st.pSpeed, pStay: st.pStayNow, pAim: st.pAim,
        shotsThisPulse,
        long: NaN, lat: NaN, landed: false,
      };
      shots.push(rec);
      pending = null;
    }
    if (emptyCell) benchSettled();
    else if (hive.tips > 0) break;
  }
  for (let k = 0; k < 60 * 3; k++) w.step(brain.update(w.sensors(), emptyGamepad(), w.seq, 1 / 60));
  const log = w.snapshot().shots.slice(shots0);
  for (let i = 0; i < shots.length && i < log.length; i++) {
    shots[i].long = log[i].long_in * 2.54;
    shots[i].lat = log[i].lat_in * 2.54;
    shots[i].landed = log[i].result === 'cell';
  }
  return { shots, wasted, speed: frames ? sumV / frames : 0, frames, openFrames, holds };
}

const wrapDeg = (d: number) => ((d + 540) % 360) - 180;
const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN);
const sd = (a: number[]) => { const m = mean(a); return a.length > 1 ? Math.sqrt(a.reduce((x, y) => x + (y - m) ** 2, 0) / (a.length - 1)) : NaN; };
const f2 = (v: number) => (Number.isFinite(v) ? v.toFixed(2) : '  nan').padStart(6);
const f0 = (v: number) => (Number.isFinite(v) ? v.toFixed(0) : 'nan').padStart(5);

export async function main(argv: string[] = []): Promise<void> {
  const num = (k: string, d: number) => { const i = argv.indexOf(`--${k}`); return i >= 0 ? Number(argv[i + 1]) : d; };
  const speed = num('speed', 1.0), off = num('off', 40), secs = num('secs', 40), seeds = num('seeds', 2);
  const noScatter = argv.includes('--noscatter');
  // --minshots N: keep adding seeds until the sample is at least N balls, so a rate quoted at
  // one speed is not a tighter number than the same rate quoted at another. Capped, because a
  // speed the gate mostly refuses would otherwise run for ever.
  const minShots = num('minshots', 0);
  const maxSeeds = Math.max(seeds, minShots > 0 ? 14 : seeds);
  const all: Shot[] = [];
  const bySeed: Shot[][] = [];
  let wasted = 0;
  let vSum = 0;
  let runs = 0;
  let totFrames = 0, totOpen = 0;
  const whyShut = new Map<string, number>();
  for (let s = 0; s < maxSeeds; s++) {
    if (s >= seeds && all.length >= minShots) break;
    const r = await run(speed, off, secs, 400 + s * 7, noScatter);
    all.push(...r.shots);
    bySeed.push(r.shots);
    wasted += r.wasted;
    vSum += r.speed;
    totFrames += r.frames;
    totOpen += r.openFrames;
    for (const [k, v] of r.holds) whyShut.set(k, (whyShut.get(k) ?? 0) + v);
    runs++;
  }
  // The same cycle time the runs were given, so the histogram's units are the right ones.
  const cyi = argv.indexOf('--cycle');
  const cycle = cyi >= 0 ? Number(argv[cyi + 1]) : (robotSpec as unknown as RobotSpec).transfer.cycleTime_s;
  const withOutcome = all.filter((s) => Number.isFinite(s.long));
  const paired = all.filter((s) => Number.isFinite(s.tCommit));
  console.log(`\nCOMMIT vs RELEASE at ${speed} m/s asked / ${(vSum / Math.max(1, runs)).toFixed(2)} actual, ${off} in, ${runs} seeds${noScatter ? ', scatter OFF' : ''}: ${all.length} shots, ${wasted} pulses released nothing`);
  const dt = paired.map((s) => s.tRelease - s.tCommit);
  console.log(`  commit -> release   ${f2(mean(dt))} +-${f2(sd(dt))} s   (min ${f2(Math.min(...dt))}, max ${f2(Math.max(...dt))})`);
  // BALL TO BALL, which is the number a driver feels. Split per seed: the gap across a seed
  // boundary is not a gap. A refused cycle shows up here as a multiple of cycleTime_s.
  const gaps: number[] = [];
  for (const run of bySeed) {
    for (let i = 1; i < run.length; i++) gaps.push(run[i].tRelease - run[i - 1].tRelease);
  }
  if (gaps.length) {
    const sorted = [...gaps].sort((a, b) => a - b);
    const hist = new Map<string, number>();
    for (const g of gaps) {
      const n = Math.round(g / cycle);
      hist.set(`${n}x`, (hist.get(`${n}x`) ?? 0) + 1);
    }
    console.log(`  ball to ball        ${f2(mean(gaps))} +-${f2(sd(gaps))} s   median ${f2(sorted[Math.floor(sorted.length / 2)])}   min ${f2(sorted[0])}`);
    console.log(`    in units of the ${cycle} s cycle: ${[...hist.entries()].sort().map(([k, v]) => `${k} ${v}`).join('  ')}`);
  }
  console.log(`  P(land) commit      ${f2(mean(paired.map((s) => s.pCommit)))}   at release ${f2(mean(paired.map((s) => s.pRelease)))}`);
  console.log(`  v_radial commit     ${f2(mean(paired.map((s) => s.vrCommit)))}   release ${f2(mean(paired.map((s) => s.vrRelease)))}   |delta| ${f2(mean(paired.map((s) => Math.abs(s.vrRelease - s.vrCommit))))} m/s`);
  console.log(`  v_lateral commit    ${f2(mean(paired.map((s) => s.vlCommit)))}   release ${f2(mean(paired.map((s) => s.vlRelease)))}   |delta| ${f2(mean(paired.map((s) => Math.abs(s.vlRelease - s.vlCommit))))} m/s`);
  const rr = paired.map((s) => s.rangeRelease).filter(Number.isFinite);
  console.log(`  range commit        ${f2(mean(paired.map((s) => s.rangeCommit)))}   release ${f2(mean(paired.map((s) => s.rangeRelease)))} in   spread ${f2(Math.min(...rr))} to ${f2(Math.max(...rr))} in  <- flat on the arc, varies on a straight line`);
  const dS = all.map((s) => s.gotSpeed - s.wantSpeed);
  const dE = all.map((s) => s.gotElev - s.wantElev);
  console.log(`  exit speed got-want ${f2(mean(dS))} +-${f2(sd(dS))} m/s     elevation got-want ${f2(mean(dE))} +-${f2(sd(dE))} deg`);
  console.log(`  aim err at release  ${f2(mean(all.map((s) => Math.abs(s.aimErrRelease))))} deg (gate: estimate vs axis)   servo err ${f2(mean(all.map((s) => Math.abs(s.cmdErrRelease))))} deg (command vs axis)   hood ${f2(mean(all.map((s) => Math.abs(s.hoodErrRelease))))} deg`);
  const holds: Record<string, number> = {};
  for (const s of all) { const k = s.holdAtRelease.replace(/-?[\d.]+/g, 'N') || 'clear'; holds[k] = (holds[k] ?? 0) + 1; }
  console.log('  gate state AT RELEASE:');
  for (const [k, v] of Object.entries(holds).sort((a, b) => b[1] - a[1])) {
    const sub = withOutcome.filter((s) => (s.holdAtRelease.replace(/-?[\d.]+/g, 'N') || 'clear') === k);
    const inN = sub.filter((s) => s.landed).length;
    console.log(`    ${String(v).padStart(4)}  ${k.padEnd(60)} landed ${inN}/${sub.length}   long ${f0(mean(sub.map((s) => s.long)))} +-${f0(sd(sub.map((s) => s.long)))} cm`);
  }
  const multi = all.filter((s) => s.shotsThisPulse > 1).length;
  console.log(`  shots that were the 2nd+ ball of one pulse: ${multi}`);
  const byOpen = (lo: number, hi: number) => withOutcome.filter((s) => s.openTrue >= lo && s.openTrue < hi);
  console.log('  by TRUE angle off the opening:');
  for (const [lo, hi] of [[0, 20], [20, 35], [35, 50], [50, 65], [65, 180]]) {
    const sub = byOpen(lo, hi);
    if (!sub.length) continue;
    const inN = sub.filter((s) => s.landed).length;
    console.log(`    ${String(lo).padStart(3)}-${String(hi).padEnd(3)} deg  n=${String(sub.length).padStart(3)}  landed ${((100 * inN) / sub.length).toFixed(0).padStart(3)}%   believed open ${f0(mean(sub.map((s) => s.openBelieved)))}   pAim ${f2(mean(sub.map((s) => s.pAim)))}  pStay ${f2(mean(sub.map((s) => s.pStay)))}  pSpeed ${f2(mean(sub.map((s) => s.pSpeed)))}  fill ${f2(mean(sub.map((s) => s.fill)))}   long ${f0(mean(sub.map((s) => s.long)))} +-${f0(sd(sub.map((s) => s.long)))}  lat ${f0(mean(sub.map((s) => s.lat)))} +-${f0(sd(sub.map((s) => s.lat)))}`);
  }
  console.log('  by pocket fill (brain count):');
  for (const [lo, hi] of [[0, 1], [1, 3], [3, 5], [5, 99]]) {
    const sub = withOutcome.filter((s) => s.fill >= lo && s.fill < hi);
    if (!sub.length) continue;
    const inN = sub.filter((s) => s.landed).length;
    console.log(`    ${String(lo).padStart(3)}-${String(hi).padEnd(3)}      n=${String(sub.length).padStart(3)}  landed ${((100 * inN) / sub.length).toFixed(0).padStart(3)}%`);
  }
  // THE BOTTLENECK, once the ball and the pocket are perfect. What is left is how much of the
  // drive the robot is ALLOWED to shoot, and every frame it is not is attributed.
  if (totFrames) {
    const secs = totFrames / 60;
    console.log(`  gate open           ${((100 * totOpen) / totFrames).toFixed(0)}% of ${secs.toFixed(0)} s driven   ->  ${(all.length / secs).toFixed(2)} balls/s fired, ${(secs / Math.max(1, withOutcome.filter((x) => x.landed).length)).toFixed(2)} s per ball IN`);
    const top = [...whyShut.entries()].sort((a, b) => b[1] - a[1]).slice(0, 8);
    for (const [k, v] of top) {
      const pct = (100 * v) / totFrames;
      if (pct > 0.05) console.log(`    ${pct.toFixed(1).padStart(5)}%  ${k}`);
    }
  }
  // HOW MUCH OF THIS IS ONE SEED? 264 balls from 4 seeds is not 264 independent trials: within
  // a seed the shots share an arc, a wheel and a tag geometry. The spread across seeds is the
  // honest error bar, and it is always wider than the binomial one.
  const perSeed = bySeed.filter((r) => r.length).map((r) => {
    const w = r.filter((x) => Number.isFinite(x.long));
    return { n: w.length, rate: w.length ? w.filter((x) => x.landed).length / w.length : NaN };
  });
  if (perSeed.length > 1) {
    const rates = perSeed.map((x) => x.rate).filter(Number.isFinite);
    console.log(`  per seed            ${perSeed.map((x) => `${(x.rate * 100).toFixed(0)}% (n=${x.n})`).join('  ')}   spread ${f2(Math.min(...rates) * 100)} to ${f2(Math.max(...rates) * 100)}%`);
  }
  // THE CONTROL FOR --emptycell. The first few balls of a seed go into a pocket that is empty
  // because nothing has landed yet, benching or no benching. If their rate matches the whole
  // run's, the benching is not inflating anything; if it is higher, it is.
  const firstN = 6;
  const early = bySeed.flatMap((r) => r.filter((x) => Number.isFinite(x.long)).slice(0, firstN));
  const late = bySeed.flatMap((r) => r.filter((x) => Number.isFinite(x.long)).slice(firstN));
  const rate = (a: Shot[]) => (a.length ? `${((100 * a.filter((x) => x.landed).length) / a.length).toFixed(0)}% (n=${a.length})` : '-');
  console.log(`  first ${firstN} of each seed  ${rate(early)}   everything after  ${rate(late)}   <- these should agree`);
  // WHAT SEPARATES A MISS FROM A HIT? With the gate wide open the sample finally contains
  // both, so every recorded quantity can be compared across the two groups. A variable that
  // differs is a candidate for what the gate should be testing; one that does not is not.
  if (argv.includes('--why')) {
    const hit = withOutcome.filter((x) => x.landed);
    const miss = withOutcome.filter((x) => !x.landed);
    const cols: [string, (x: Shot) => number][] = [
      ['commit->release s', (x) => x.tRelease - x.tCommit],
      ['|d v_radial| m/s', (x) => Math.abs(x.vrRelease - x.vrCommit)],
      ['|d v_lateral| m/s', (x) => Math.abs(x.vlRelease - x.vlCommit)],
      ['|d range| in', (x) => Math.abs(x.rangeRelease - x.rangeCommit)],
      ['|aim err| deg', (x) => Math.abs(x.aimErrRelease)],
      ['|servo err| deg', (x) => Math.abs(x.cmdErrRelease)],
      ['|hood err| deg', (x) => Math.abs(x.hoodErrRelease)],
      ['|v_lateral| m/s', (x) => Math.abs(x.vlRelease)],
      ['|v_radial| m/s', (x) => Math.abs(x.vrRelease)],
      ['range at release in', (x) => x.rangeRelease],
      ['off-opening deg', (x) => x.openTrue],
      ['pocket fill (belief)', (x) => x.fill],
      ['exit speed got-want', (x) => x.gotSpeed - x.wantSpeed],
      ['elevation got-want', (x) => x.gotElev - x.wantElev],
    ];
    console.log(`  WHAT SEPARATES A MISS FROM A HIT?  ${hit.length} hits, ${miss.length} misses`);
    console.log('    quantity                   hits            misses          separation');
    for (const [name, f] of cols) {
      const h = hit.map(f).filter(Number.isFinite);
      const m = miss.map(f).filter(Number.isFinite);
      if (!h.length || !m.length) continue;
      const pooled = Math.sqrt(((sd(h) ** 2) + (sd(m) ** 2)) / 2);
      const d = pooled > 1e-9 ? (mean(m) - mean(h)) / pooled : 0;
      const flag = Math.abs(d) >= 0.8 ? '  <<< STRONG' : Math.abs(d) >= 0.4 ? '  <  moderate' : '';
      console.log(`    ${name.padEnd(22)} ${f2(mean(h))} +-${f2(sd(h))}   ${f2(mean(m))} +-${f2(sd(m))}   d=${d.toFixed(2).padStart(6)}${flag}`);
    }
  }
  const inN = withOutcome.filter((s) => s.landed).length;
  console.log(`  landed ${inN}/${withOutcome.length} (${withOutcome.length ? ((100 * inN) / withOutcome.length).toFixed(0) : '-'}%)   long ${f0(mean(withOutcome.map((s) => s.long)))} +-${f0(sd(withOutcome.map((s) => s.long)))} cm   lat ${f0(mean(withOutcome.map((s) => s.lat)))} +-${f0(sd(withOutcome.map((s) => s.lat)))} cm`);
  if (argv.includes('--dump')) {
    console.log('   dt    pC    pR    vrC    vrR   vlC    vlR   dSpd   dElev  aimE  hoodE  openB openT  fill  pAim  long   lat  in  hold');
    for (const s of all) {
      console.log(`  ${f2(s.tRelease - s.tCommit)} ${f2(s.pCommit)} ${f2(s.pRelease)} ${f2(s.vrCommit)} ${f2(s.vrRelease)} ${f2(s.vlCommit)} ${f2(s.vlRelease)} ${f2(s.gotSpeed - s.wantSpeed)} ${f2(s.gotElev - s.wantElev)} ${f2(s.aimErrRelease)} ${f2(s.hoodErrRelease)} ${f0(s.openBelieved)} ${f0(s.openTrue)} ${f2(s.fill)} ${f2(s.pAim)} ${f0(s.long)} ${f0(s.lat)}  ${s.landed ? 'Y' : 'n'}  ${s.holdAtRelease}`);
    }
  }
  console.log('');
}
