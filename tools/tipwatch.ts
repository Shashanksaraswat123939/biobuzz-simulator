/**
 * WHAT DOES THE ROBOT BELIEVE AFTER THE HIVE TIPS? Truth against belief, frame by frame.
 *
 *   npm run tool -- tools/tipwatch.ts [--secs 10] [--range 45]
 *
 * A TIP swaps which CELL is up. The world knows instantly -- `Hive.upCell` is chosen from the
 * rocker's live angle, no flag -- and the robot is told nothing at all, which is the design:
 * the oracle `hiveTipping` was deleted and a camera is never told anything, it just stops
 * decoding. So the robot has to notice by losing the tag and going to look for the new one.
 *
 * This watches whether it ACTUALLY does. It fires until the rocker goes over and then prints,
 * every quarter second: where the mouth really is, where the robot thinks it is, how old its
 * fix is, whether it believes that fix is valid, and whether the turret has started sweeping
 * to search. The gap between the last two is the whole question -- a robot that believes a
 * stale fix is valid never searches, and a camera bolted to the turret only ever sees where
 * the turret is pointing.
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { RAD, M_TO_IN, inches, DEG } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

export async function main(argv: string[] = []): Promise<void> {
  const num = (k: string, d: number) => { const i = argv.indexOf(`--${k}`); return i >= 0 ? Number(argv[i + 1]) : d; };
  const secs = num('secs', 10), range_in = num('range', 45);
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  const pool = Array.from({ length: 200 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: pool, alliance: 'red', seed: 5 });
  for (const b of w.balls.balls) w.balls.park(b);

  // Stand square-ish on the up CELL's opening so the tag decodes cleanly to begin with.
  const hive = w.hives.red;
  const mouth0 = hive.upCellMouthWorld();
  const n0 = hive.upCellMouthNormalWorld();
  const hn = Math.hypot(n0[0], n0[2]) || 1;
  const y = spec.chassis.height_m / 2 + spec.chassis.clearance_m;
  const start: Vec3 = [mouth0[0] + (n0[0] / hn) * inches(range_in), y, mouth0[2] + (n0[2] / hn) * inches(range_in)];
  w.robot.place(start, Math.atan2(mouth0[0] - start[0], mouth0[2] - start[2]) * RAD);

  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  brain.state.firing = true;
  let loaded = 0;
  const g: GamepadState = emptyGamepad();

  /** Where the robot believes the mouth is, in FTC inches -- main.ts draws the violet ring here. */
  const belief = (): { x: number; y: number } => {
    const t = brain.target();
    const pose = brain.pose_();
    const b = (pose.heading + t.azimuthDeg) * DEG;
    return { x: pose.x + t.rangeIn * Math.cos(b), y: pose.y + t.rangeIn * Math.sin(b) };
  };
  /** Where it really is. worldToFtc is x -> -z, y -> -x for this field's frame. */
  const truth = (): { x: number; y: number } => {
    const m = hive.upCellMouthWorld();
    return { x: -m[2] * M_TO_IN, y: -m[0] * M_TO_IN };
  };

  let tipped = -1;
  let lastLine = -1;
  console.log('Firing until the rocker goes over, then watching for ' + secs + ' s.\n');
  console.log('  t     since  tag  age    fresh valid scan  decoding | belief x,y (in)   truth x,y (in)   miss');
  for (let i = 0; i < 60 * 90; i++) {
    while (w.robot.heldBalls().length < spec.hopper.capacity && loaded < pool.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    if (i === 30) brain.state.flywheelOn = true;
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq, 1 / 60));

    if (tipped < 0 && hive.tips > 0) {
      tipped = w.t;
      console.log(`  --- TIP at t=${w.t.toFixed(2)} s. The world switched CELL instantly; the robot was told nothing.`);
    }
    if (tipped < 0) continue;
    if (w.t - lastLine < 0.25) continue;
    lastLine = w.t;
    const t = brain.target();
    const be = belief();
    const tr = truth();
    const miss = Math.hypot(be.x - tr.x, be.y - tr.y);
    console.log(
      `  ${w.t.toFixed(2).padStart(5)} ${(w.t - tipped).toFixed(2).padStart(5)}s  ` +
      `${String(t.id).padStart(2)}  ${(Number.isFinite(t.ageS) ? t.ageS : 99).toFixed(2).padStart(5)}s ` +
      `${t.fresh ? ' yes ' : ' NO  '} ${t.valid ? ' yes ' : ' NO  '} ${t.scanning ? ' YES ' : ' no  '} ` +
      `${w.tagCam.isSeeing(w.t) ? '  yes  ' : '  no   '} | ` +
      `${be.x.toFixed(1).padStart(7)},${be.y.toFixed(1).padStart(7)}  ` +
      `${tr.x.toFixed(1).padStart(7)},${tr.y.toFixed(1).padStart(7)}  ${miss.toFixed(1).padStart(5)} in`,
    );
    if (w.t - tipped > secs) break;
  }
  console.log('\n  "scan" is the turret sweeping to go and find the tag. The camera rides the turret,');
  console.log('  so if that column never says YES the robot is only ever looking where it already was.');
}
