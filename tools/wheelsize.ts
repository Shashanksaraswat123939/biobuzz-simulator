/**
 * WHAT DOES A BIGGER MECANUM ACTUALLY BUY? goBILDA sells 96, 104 and 140 mm.
 *
 *   npm run tool -- tools/wheelsize.ts [--d 96,104,140]
 *
 * A wheel is a gear ratio you cannot change once the chassis is drilled. Free speed goes up
 * with the radius and the torque at the contact patch goes down by exactly the same factor,
 * so a bigger wheel is a taller gear: faster flat out, slower to get there, and worse at
 * holding a line against a pushing opponent.
 *
 * Measured per diameter: the peak speed down the longest clear straight the field has, the
 * time from rest to 1.0 and 1.5 m/s, and the drive current while accelerating. The straight
 * is 2.7 m and the robot is still gaining at the end of it, so PEAK is a lower bound on free
 * speed for every row equally -- compare the rows, not the row against a datasheet.
 *
 * WHAT THIS DOES NOT MODEL, and what decides it anyway: whether the wheel FITS. The axles sit
 * at +-5.5 in fore and aft in a 15.5 in frame, so the wheel's own radius has to clear 7.75 in
 * minus 5.5 in = 2.25 in, which is a 114 mm wheel. That is geometry, not physics, and it is
 * printed below rather than simulated.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { M_TO_IN } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

async function run(d_mm: number) {
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  spec.drivetrain.wheelRadius_m = d_mm / 2000;
  // The governor's open-loop half reads freeSpeed_mps, so it has to move with the wheel or a
  // bigger wheel is measured against the old wheel's idea of flat out.
  spec.drivetrain.freeSpeed_mps = (spec.drivetrain.freeSpeed_mps ?? 2.19) * (d_mm / (robotSpec.drivetrain.wheelRadius_m * 2000));
  const w = new World({ params: p, robot: spec, staging: [], alliance: 'red', seed: 7 });
  for (const b of w.balls.balls) w.balls.park(b);
  const half = w.geom.halfWidth_m - 0.5;
  const start: Vec3 = [0, spec.chassis.height_m / 2 + spec.chassis.clearance_m, -half];
  w.robot.place(start, 0);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  let peak = 0, t10 = Infinity, t15 = Infinity, ampSum = 0, n = 0;
  for (let i = 0; i < 60 * 8; i++) {
    const pos = w.robot.pos;
    if (Math.hypot(pos[0] - start[0], pos[2] - start[2]) > 2 * half - 0.7) break;
    const g: GamepadState = emptyGamepad();
    g.left_stick_y = -1;
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq, 1 / 60));
    const v = w.robot.body.linvel();
    const sp = Math.hypot(v.x, v.z);
    peak = Math.max(peak, sp);
    if (sp >= 1.0 && !Number.isFinite(t10)) t10 = w.t;
    if (sp >= 1.5 && !Number.isFinite(t15)) t15 = w.t;
    if (w.t < 1.0) { ampSum += w.battery.amps; n++; }
  }
  return { peak, t10, t15, amps: ampSum / Math.max(1, n) };
}

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--d');
  const list = (i >= 0 ? argv[i + 1] : '96,104,140').split(',').map(Number);
  await initPhysics();
  const shipped = robotSpec.drivetrain.wheelRadius_m * 2000;
  // Half the frame, minus the axle's offset from the centre: the radius that still fits.
  const halfLen_in = (robotSpec.chassis.length_m * M_TO_IN) / 2;
  const axle_in = 5.5;
  const fits_mm = (halfLen_in - axle_in) * 2 * 25.4;
  console.log(`\nMECANUM DIAMETER, straight-line launch. Shipped: ${shipped.toFixed(0)} mm.\n`);
  console.log('   d (mm)   peak on a 2.7 m run   0 to 1.0 m/s   0 to 1.5 m/s   drive amps, first second   fits?');
  const s = (n: number) => (Number.isFinite(n) ? `${n.toFixed(2)} s` : 'never');
  for (const d of list) {
    const r = await run(d);
    console.log(
      `   ${String(d).padStart(6)}   ${r.peak.toFixed(2).padStart(14)} m/s   ${s(r.t10).padStart(12)}   ` +
      `${s(r.t15).padStart(12)}   ${r.amps.toFixed(1).padStart(21)} A   ${d <= fits_mm ? 'yes' : 'NO'}`,
    );
  }
  console.log(`\n   "fits" is the frame, not the physics: axles at +-${axle_in} in in a ${(halfLen_in * 2).toFixed(1)} in frame leave`);
  console.log(`   room for ${fits_mm.toFixed(0)} mm of wheel before it stands proud of the bumper line.`);
  console.log('   The peak is a LOWER BOUND for every row -- the field has no straight long enough to');
  console.log('   run this drivetrain out. Compare the rows against each other.\n');
}
