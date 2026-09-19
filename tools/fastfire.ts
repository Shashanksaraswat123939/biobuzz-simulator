/**
 * SHOOTING FAST WHILE DRIVING FAST. Balls in per second at a held speed and stand-off.
 *
 *   npm run tool -- tools/fastfire.ts [--secs 40] [--seeds 3]
 *
 * The other harnesses cannot answer this. A patrol circling at 42 in tops out near 1.05 m/s
 * however hard the stick is pushed -- the arc is too tight to accelerate round -- so every
 * "at speed" number so far is really a number at about 1 m/s.
 *
 * This drives a LONG straight line across the front of the mouth, turning round well outside
 * the shooting sector so the in-sector part of every crossing is at full speed, and pins the
 * speed with drivetrain.maxSpeed_mps rather than hoping the stick reaches it. Only the frames
 * inside the sector are counted.
 *
 * KNOWN FAULT, READ BEFORE TRUSTING A NUMBER FROM THIS. Above about 0.5 m/s this rig sheds
 * balls out of the bin: it is handed 80-100 POLLEN for every one it fires, against 1.2 in
 * tools/zonerun.ts doing the same job at the same speed. The hopper is therefore empty for
 * most of the drive and every rate below is far too low. It found a real bug on the way --
 * Robot.preload dropped balls in at REST into a moving bin, so each one slammed into the
 * wall and was thrown out, now fixed -- but that only took it from 100 to 80 fed per shot,
 * so something else in here is still wrong. Use zonerun until this says fed/shot near 1.
 *
 * THE HEADLINE IS SECONDS PER BALL IN, not the land rate: a gate that refuses everything but
 * the certain shot reads as a high percentage while scoring less, because the refused cycles
 * are time. transfer.cycleTime_s is 0.6 s, so about 0.7 s per ball is the floor a perfect
 * gate could reach.
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

interface Res { secs: number; shots: number; landed: number; speed: number; why: Record<string, number>; frames: number; open: number;
  cam: { tipping: number; range: number; incidence: number; lens: number; ok: number }; loaded: number; held: number; meanInc: number; tips: number; afterTip: number; longs: number[]; lats: number[] }

async function run(speed: number, standoff_in: number, secs: number, seed: number): Promise<Res | null> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  // Pin the speed with the cap rather than hoping a stick reaches it.
  if (!process.argv.includes('--nocap')) spec.drivetrain.maxSpeed_mps = speed;
  const mpArg = process.argv.indexOf('--minp');
  if (mpArg >= 0) spec.flywheel.minLandProb = Number(process.argv[mpArg + 1]);
  const saArg = process.argv.indexOf('--stateage');
  if (saArg >= 0) spec.sensors.tag.target.maxStateAgeS = Number(process.argv[saArg + 1]);
  const incArg = process.argv.indexOf('--incidence');
  if (incArg >= 0) spec.sensors.tag.maxIncidence_deg = Number(process.argv[incArg + 1]);
  if (process.argv.includes('--freshonly')) spec.sensors.tag.target.fireOnOdometry = false;
  const cyArg = process.argv.indexOf('--cycle');
  if (cyArg >= 0) spec.transfer.cycleTime_s = Number(process.argv[cyArg + 1]);
  const mrArg = process.argv.indexOf('--minrange');
  if (mrArg >= 0) spec.shot = { minRange_in: Number(process.argv[mrArg + 1]) };
  const fovArg = process.argv.indexOf('--fov');
  if (fovArg >= 0) spec.sensors.tag.fov_deg = Number(process.argv[fovArg + 1]);
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
  // Kept for the lateral bookkeeping below.
  const at = (lat_in: number): Vec3 => [
    mouth[0] + nrm[0] * inches(standoff_in) + side[0] * inches(lat_in), y,
    mouth[2] + nrm[1] * inches(standoff_in) + side[1] * inches(lat_in),
  ];
  // STAY ON THE FIELD. This is the whole reason the first version measured nothing: straight
  // out from the red mouth the field runs out at about 55 in (mouth at z 16, wall at 71), so
  // a 70 or 85 in stand-off put the robot 15-30 in THROUGH the wall. Every ball preloaded
  // into it was instantly out of bounds and parked -- 400 fed, 3 fired, and the rig blamed a
  // stale tag. Clamp the line to what the field actually holds.
  const hw = w.geom.halfWidth_m - inches(14);   // half a robot in from the boards
  const onField = (p: Vec3) => Math.abs(p[0]) <= hw && Math.abs(p[2]) <= hw;
  // THE PATH IS AN ARC, SO TEST THE ARC. The radial correction below drives toward a constant
  // RANGE to the mouth, not a constant perpendicular offset, so the robot sweeps a circle.
  // Checking the straight-line end points instead rejected every stand-off past 40 in as "off
  // the field" when the arc through them fits perfectly well.
  const onArc = (bearDeg: number): Vec3 => {
    const a = Math.atan2(nrm[0], nrm[1]) + bearDeg * (Math.PI / 180);
    return [mouth[0] + Math.sin(a) * inches(standoff_in), y, mouth[2] + Math.cos(a) * inches(standoff_in)];
  };
  // FIND THE ARC SEGMENT THAT FITS, rather than demanding the whole arc. Straight out from
  // the mouth the field ends at about 55 in, so a 46 in circle does not fit dead ahead -- but
  // its flanks do, which is exactly where the 7 green squares at 60 in or more live (49-60
  // deg off the opening). Scan the bearings, keep the longest run that is on the field and
  // still inside the fire cap, and patrol that.
  const capDeg = spec.turret.fireOpenCap_deg ?? 60;
  let bestLo = 0, bestHi = 0, curLo: number | null = null;
  for (let b = -capDeg; b <= capDeg; b += 2) {
    if (onField(onArc(b))) {
      if (curLo === null) curLo = b;
      if (b - curLo > bestHi - bestLo) { bestLo = curLo; bestHi = b; }
    } else curLo = null;
  }
  if (bestHi - bestLo < 16) return null;
  const midDeg = (bestLo + bestHi) / 2;
  const sweepDeg = (bestHi - bestLo) / 2;
  const reach = standoff_in * Math.sin(sweepDeg * Math.PI / 180);
  const start = onArc(bestLo);
  void midDeg; void at;
  w.robot.place(start, Math.atan2(mouth[0] - start[0], mouth[2] - start[2]) * RAD);
  if (process.argv.includes('--where')) {
    const hw = w.geom.halfWidth_m * M_TO_IN;
    console.log(`      [place] standoff ${standoff_in} in -> robot at x ${(start[0] * M_TO_IN).toFixed(0)}, z ${(start[2] * M_TO_IN).toFixed(0)} in; field is +-${hw.toFixed(0)} in; reach +-${reach.toFixed(0)} in`);
  }
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  brain.state.firing = !process.argv.includes('--noshoot');
  let loaded = 0, dir = 1;
  const why: Record<string, number> = {};
  let frames = 0, open = 0, sumV = 0;
  const cam = { tipping: 0, range: 0, incidence: 0, lens: 0, ok: 0 };
  let incSum = 0, incN = 0;
  let tips0 = 0, framesAfterTip = 0;
  let shots0 = -1, landed0 = -1;

  for (let i = 0; i < 60 * secs; i++) {
    while (w.robot.heldBalls().length < spec.hopper.capacity && loaded < pool.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    const c = w.robot.pos;
    const lat = ((c[0] - mouth[0]) * side[0] + (c[2] - mouth[2]) * side[1]) * M_TO_IN;
    const dd0 = Math.hypot(mouth[0] - c[0], mouth[2] - c[2]);
    if (lat > reach) dir = -1;
    else if (lat < -reach) dir = 1;
    const g: GamepadState = emptyGamepad();
    // Strafe along the line; a light yaw correction keeps the nose near the hive so the
    // turret is not fighting its end stop. The TURRET does the aiming, not the chassis.
    const yawErr = wrapDeg(Math.atan2(mouth[0] - c[0], mouth[2] - c[2]) * RAD - w.robot.yaw * RAD);
    g.right_stick_x = -Math.max(-0.2, Math.min(0.2, yawErr / 45));
    g.left_stick_x = -dir;
    // HOLD THE STAND-OFF. Strafing is in the ROBOT's frame and the chassis yaws to face the
    // hive, so the "straight line" rotates with it and the robot spirals outward -- the first
    // version drifted past the camera's 120 in range limit and blamed 94% of its lost frames
    // on a stale tag that was really just too far away.
    const rangeNow = dd0 * M_TO_IN;
    g.left_stick_y = -Math.max(-0.4, Math.min(0.4, (rangeNow - standoff_in) / 20));
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq, 1 / 60));

    const v = w.robot.body.linvel();
    const sp = Math.hypot(v.x, v.z);
    const dx = mouth[0] - w.robot.pos[0], dz = mouth[2] - w.robot.pos[2];
    const dd = Math.hypot(dx, dz) || 1;
    const off = Math.acos(Math.max(-1, Math.min(1, -(dx * nrm[0] + dz * nrm[1]) / dd))) * RAD;
    // In the sector and up to speed: the part of the drive being asked about.
    // The "up to speed" filter cannot apply to a standing robot: at speed 0 the test
    // sp > 0 is false on every frame, no frames are counted, and s per ball comes out as
    // 0.07 -- 147 balls in ten seconds, through a mechanism that can fire one every 0.6.
    // EVERY FRAME OF THE PATROL COUNTS, and so does every shot. Filtering the frames but
    // not the shots gave time and shots two different denominators, and the answer came out
    // at 0.42 s per ball -- faster than the 0.60 s the feed physically takes. Any result that
    // beats the mechanism is a broken measurement, not a fast robot. What a driver actually
    // gets is balls per second of DRIVING, lining-up time included, so that is what this is.
    void off; void sp;
    {
      if (shots0 < 0) { shots0 = w.robot.shots; landed0 = w.landedInUpCell("red"); }

      frames++; sumV += sp;
      // WHICH CAMERA GATE REFUSES. Recomputed from truth: tipping, range, panel incidence,
      // and the lens (which is aimed by the TURRET, not the chassis).
      const tagW = hive.upCellTagWorld();
      const tn = hive.upCellTagNormalWorld();
      const tdx = tagW[0] - w.robot.pos[0], tdz = tagW[2] - w.robot.pos[2];
      const td = Math.hypot(tdx, tdz) || 1;
      const inc = Math.acos(Math.max(-1, Math.min(1, (-tdx * tn[0] + -tdz * tn[2]) / (td * (Math.hypot(tn[0], tn[2]) || 1))))) * RAD;
      const bear = Math.atan2(tdx, tdz) * RAD - w.robot.yaw * RAD;
      const fovOff = Math.abs(((bear - w.robot.turretAngle) % 360 + 540) % 360 - 180);
      incSum += inc; incN++;
      if (hive.tips > tips0) framesAfterTip++;
      // A TIP turns the goal away: from this side the up CELL now opens the other way and
      // there is no shot, which a driver answers by repositioning. Counting that time as
      // "the robot refused to shoot" measures the wrong thing -- 61-74% of a 60 s run.
      if (process.argv.includes('--untiltip') && hive.tips > tips0) break;
      if (hive.tipping) cam.tipping++;
      else if (td * M_TO_IN > spec.sensors.tag.maxRange_in) cam.range++;
      else if (inc > spec.sensors.tag.maxIncidence_deg) cam.incidence++;
      else if (fovOff > spec.sensors.tag.fov_deg / 2) cam.lens++;
      else cam.ok++;
      // state.ready, NOT an empty hold string: `st.hold` is '' when the wheel is OFF too,
      // so counting empty holds as "gate open" reports an idle shooter as ready and reads
      // 99% open next to two shots.
      if (brain.state.ready) open++;
      const h = brain.state.hold || (brain.state.ready ? '' : 'wheel off / not armed');
      if (h) why[h.replace(/-?[\d.]+/g, 'N')] = (why[h.replace(/-?[\d.]+/g, 'N')] ?? 0) + 1;
      if (w.sensors().game.hopper === 0) why['HOPPER EMPTY'] = (why['HOPPER EMPTY'] ?? 0) + 1;
    }
  }
  for (let k = 0; k < 60 * 3; k++) w.step(brain.update(w.sensors(), emptyGamepad(), w.seq, 1 / 60));
  // HOW THEY MISS. A systematic bias is free to correct with a trim; random scatter is not,
  // and the two need completely different answers, so never quote one land rate without them.
  const log = w.snapshot().shots.filter((x) => Number.isFinite(x.long_in) && Number.isFinite(x.lat_in));
  const longs = log.map((x) => x.long_in * 2.54);
  const lats = log.map((x) => x.lat_in * 2.54);
  // WHERE DID THEY GO? States of every ball that was fed in.
  if (process.argv.includes('--where')) {
    const by: Record<string, number> = {};
    for (const b of w.balls.balls.slice(0, loaded)) by[b.state] = (by[b.state] ?? 0) + 1;
    const heights = w.balls.balls.slice(0, loaded).map((b) => w.balls.pos(b)[1]).filter((y) => y > -1);
    console.log(`      [where] fed ${loaded}: ${JSON.stringify(by)}  mean height ${(heights.reduce((a, x) => a + x, 0) / Math.max(1, heights.length)).toFixed(2)} m`);
  }
  return {
    secs: Math.max(1e-6, frames / 60),   // time INSIDE the sector, not wall clock
    shots: shots0 < 0 ? 0 : w.robot.shots - shots0,
    landed: landed0 < 0 ? 0 : Math.max(0, w.landedInUpCell('red') - landed0),
    speed: frames ? sumV / frames : 0, why, frames, open, cam, longs, lats, loaded, held: w.robot.heldBalls().length, meanInc: incN ? incSum / incN : 0, tips: w.hives.red.tips, afterTip: framesAfterTip,
  };
}

const wrapDeg = (d: number) => ((d + 540) % 360) - 180;

export async function main(argv: string[] = []): Promise<void> {
  const num = (k: string, d: number) => { const i = argv.indexOf(`--${k}`); return i >= 0 ? Number(argv[i + 1]) : d; };
  const secs = num('secs', 40), seeds = num('seeds', 3);
  console.log(`\nSHOOTING WHILE DRIVING FAST. ${secs} s x ${seeds} seeds, counted only inside the 45 deg sector.`);
  console.log(`transfer.cycleTime_s is ${(robotSpec as unknown as RobotSpec).transfer.cycleTime_s} s, so about 0.7 s per ball is the floor.\n`);
  console.log('  asked   actual    stand-off   shots   in   land%   S PER BALL IN   gate open');
  const sp = process.argv.indexOf('--speed');
  const speeds = sp >= 0 ? [Number(process.argv[sp + 1])] : [0.0001, 1.0, 1.57];
  for (const speed of speeds) {
    for (const off of (process.argv.indexOf('--off') >= 0 ? [Number(process.argv[process.argv.indexOf('--off') + 1])] : [30, 40])) {
      let S = 0, L = 0, T = 0, V = 0, F = 0, O = 0, LD = 0, MI = 0, TIP = 0, AT = 0;
      const LO: number[] = [], LA: number[] = [];
      const cam = { tipping: 0, range: 0, incidence: 0, lens: 0, ok: 0 };
      const why: Record<string, number> = {};
      let skipped = false;
      for (let s = 0; s < seeds; s++) {
        const r = await run(speed, off, secs, 400 + s * 7);
        if (!r) { skipped = true; break; }
        S += r.shots; L += r.landed; T += r.secs; V += r.speed; F += r.frames; O += r.open; LD += r.loaded;
        for (const [k, v] of Object.entries(r.why)) why[k] = (why[k] ?? 0) + v;
        for (const k of Object.keys(cam) as (keyof typeof cam)[]) cam[k] += r.cam[k];
        MI += r.meanInc; TIP += r.tips; AT += r.afterTip; LO.push(...r.longs); LA.push(...r.lats);
      }
      if (skipped) { console.log(`  ${speed.toFixed(2)}   ${String(off).padStart(21)} in   -- does not fit on the field`); continue; }
      const per = L > 0 ? (T / L).toFixed(2) : '   -';
      console.log(`  ${speed.toFixed(2)}   ${(V / seeds).toFixed(2)} m/s   ${String(off).padStart(6)} in   ${String(S).padStart(5)}  ${String(L).padStart(3)}   ${S ? ((L / S) * 100).toFixed(0).padStart(4) : '   -'}%   ${per.padStart(9)} s   ${F ? ((O / F) * 100).toFixed(0).padStart(6) : '     -'}%`);
      for (const [k, v] of Object.entries(why).sort((a, b) => b[1] - a[1]).slice(0, 6)) {
        if (F && v / F > 0.01) console.log(`  ${' '.repeat(50)} ${((v / F) * 100).toFixed(0).padStart(3)}%  ${k}`);
      }
      // BALLS FED vs BALLS FIRED. If the robot is handed far more than it shoots, it is
      // losing them out of the bin while driving, not failing to shoot them.
      if (F) console.log(`  ${' '.repeat(52)} balls fed in: ${LD}  fired: ${S}  -> ${S ? (LD / Math.max(1, S)).toFixed(1) : '?'} fed per shot`);
      const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN);
      const sd = (a: number[]) => { const m = mean(a); return a.length > 1 ? Math.sqrt(a.reduce((x, y) => x + (y - m) ** 2, 0) / (a.length - 1)) : NaN; };
      if (LO.length) console.log(`  ${' '.repeat(50)} miss: downrange ${mean(LO).toFixed(0)} +-${sd(LO).toFixed(0)} cm, sideways ${mean(LA).toFixed(0)} +-${sd(LA).toFixed(0)} cm  (n=${LO.length})`);
      if (F) console.log(`  ${' '.repeat(52)} HIVE tipped ${TIP} times; ${((AT / F) * 100).toFixed(0)}% of the counted drive was AFTER a tip`);
      if (F) console.log(`  ${' '.repeat(52)} mean panel incidence ${(MI / seeds).toFixed(0)} deg (cap ${(robotSpec as unknown as RobotSpec).sensors.tag.maxIncidence_deg})`);
      if (F) console.log(`  ${' '.repeat(52)} camera: decodable ${((cam.ok / F) * 100).toFixed(0)}%  blocked by -- incidence ${((cam.incidence / F) * 100).toFixed(0)}%, lens ${((cam.lens / F) * 100).toFixed(0)}%, range ${((cam.range / F) * 100).toFixed(0)}%, tipping ${((cam.tipping / F) * 100).toFixed(0)}%`);
    }
  }
  console.log('');
}
