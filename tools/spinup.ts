/**
 * WHAT DOES THE SECOND FLYWHEEL MOTOR BUY, AND WHAT DOES THE WHEEL'S INERTIA COST?
 *
 *   npm run tool -- tools/spinup.ts [--rpm 2315] [--motors 1,2] [--inertia 1.21e-4,...]
 *
 * Two times decide whether a shooter feels broken, and they pull in opposite directions.
 *
 *   SPIN-UP, cold to inside the readiness window. Paid at the start of a match and again
 *   after every pre-spin toggle. More inertia makes it worse, more motors make it better.
 *
 *   RECOVERY, the dip a ball takes out of the wheel and the climb back into the window. This
 *   is the one that caps the firing rate, because the gate will not pass a second ball until
 *   the wheel is back. The dip is lossFactor * KE_ball / (I * omega), so MORE INERTIA MAKES
 *   IT SMALLER -- the opposite direction to spin-up. Only the motor count helps both.
 *
 * NECTAR IS NOT FIRED HERE, BECAUSE THIS ROBOT CANNOT FIRE IT: the feed shaft is bored for
 * one POLLEN and a 3.62 in NECTAR will not enter (docs/PHYSICS.md, "Bored for one POLLEN").
 * Its row is what a shaft that took it would cost: the dip is the same formula at NECTAR's
 * 41.3 g against POLLEN's 24.9 g, applied to the wheel directly, and the recovery from it is
 * then MEASURED like any other. Treat the dip as arithmetic and the time as a measurement.
 *
 * The wheel is driven as the hub drives it -- RUN_USING_ENCODER off the motor's own 28-tick
 * encoder, the same as tools/slew.ts -- so these are times a real control loop could expect.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics } from '../packages/core/src/physics/world.js';
import { rpmToRadS } from '../packages/core/src/units.js';
import type { ActuatorFrame, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const TICKS_PER_REV = 28;
const cmd = (rpm: number, feed = false): ActuatorFrame => ({
  seq: 0,
  motors: {
    flywheel: { mode: 'RUN_USING_ENCODER', velocity: (rpm * TICKS_PER_REV) / 60 },
    ...(feed ? { transfer: { mode: 'RUN_WITHOUT_ENCODER' as const, power: 1 } } : {}),
  },
  servos: feed ? { gate: 1 } : {},
});

interface Row { spinUp: number; held: number; dip: number; recover: number; nDip: number; nRecover: number }

async function measure(motorCount: number, I: number, target: number, massRatio: number): Promise<Row> {
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  spec.flywheel.motorCount = motorCount;
  spec.flywheel.I_fly_kgm2 = I;
  spec.flywheel.scatter = { speedFrac: 0, angle_deg: 0, yaw_deg: 0 };
  const staging = [{ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }];
  const w = new World({ params: p, robot: spec, staging, alliance: 'red', seed: 4 });
  for (const b of w.balls.balls) w.balls.park(b);
  w.robot.place([0, spec.chassis.height_m / 2 + spec.chassis.clearance_m, 0], 0);
  const tol = spec.flywheel.tolRpm;
  // LOAD FIRST. The belt takes the better part of a second to walk a ball up to the wheel;
  // dropping one in after the wheel is up would be timing the belt, not the motor.
  w.robot.preload(w.balls, w.balls.balls[0]);

  // SPIN-UP, from a dead stop. "Up" is the readiness window, not the setpoint: a velocity PID
  // settles a little under its target and the last rpm would never arrive.
  let spinUp = Infinity;
  for (let f = 0; f < 60 * 25; f++) {
    w.step(cmd(target));
    if (w.robot.flywheelRpm >= target - tol) { spinUp = w.t; break; }
  }
  // SETTLE, AND SETTLE PROPERLY. The hub's velocity PID in this sim is underdamped: commanded
  // to 2315 from rest a single motor overshoots to about 2860 and rings down for the better
  // part of ten seconds. Three seconds of settling put every dip and every recovery on the
  // side of a swing, which is what made a heavier wheel look like it dipped MORE.
  for (let f = 0; f < 60 * 12; f++) w.step(cmd(target));
  const held = w.robot.flywheelRpm;

  /** Time to climb back to `to`, driven by the flywheel alone. */
  const climbBack = (to: number): number => {
    const t0 = w.t;
    for (let f = 0; f < 60 * 20; f++) {
      w.step(cmd(target));
      if (w.robot.flywheelRpm >= to) return w.t - t0;
    }
    return Infinity;
  };

  // THE SHOT. The dip is read on the firing frame, because the launch takes its energy out in
  // one go -- a running minimum would also collect the sag from the transfer motor's current,
  // which is the belt's bill and not the ball's.
  let before = NaN;
  let dip = NaN;
  for (let f = 0; f < 60 * 20; f++) {
    const prev = w.robot.flywheelRpm;
    w.step(cmd(target, true));
    if (w.robot.shots > 0) { before = prev; dip = prev - w.robot.flywheelRpm; break; }
  }
  const recover = climbBack(before - tol);

  // NECTAR, as arithmetic: same exit speed, same wheel, 1.66x the mass and so 1.66x the dip.
  for (let f = 0; f < 60 * 3; f++) w.step(cmd(target));
  const from = w.robot.flywheelRpm;
  const nDip = dip * massRatio;
  w.robot.flywheelOmega = Math.max(0, rpmToRadS(from - nDip));
  w.robot.motors.get('flywheel')!.omega = w.robot.flywheelOmega;
  const nRecover = climbBack(from - tol);
  return { spinUp, held, dip, recover, nDip, nRecover };
}

export async function main(argv: string[] = []): Promise<void> {
  const arg = (k: string, d: string) => { const i = argv.indexOf(`--${k}`); return i >= 0 ? argv[i + 1] : d; };
  const target = Number(arg('rpm', '2315'));
  const motorList = arg('motors', '1,2').split(',').map(Number);
  const inertias = arg('inertia', '1.21e-4,2.5e-4,3.91e-4,6e-4,9e-4').split(',').map(Number);
  await initPhysics();
  const shipped = (robotSpec as unknown as RobotSpec).flywheel;
  const b = (params as unknown as Params).ball;
  const massRatio = b.nectar.m_kg / b.pollen.m_kg;

  console.log(`FLYWHEEL SPIN-UP AND RECOVERY, to ${target} rpm, inside the ${shipped.tolRpm} rpm readiness window`);
  console.log(`shipped: ${shipped.motorCount} motor(s), I = ${shipped.I_fly_kgm2.toExponential(2)} kg.m^2.`);
  console.log(`NECTAR is ${massRatio.toFixed(2)}x POLLEN's mass and cannot enter the shaft: dip derived, time measured.\n`);
  const s = (n: number) => (Number.isFinite(n) ? n.toFixed(2) : ' none');
  for (const motors of motorList) {
    console.log(`  ${motors} MOTOR${motors > 1 ? 'S' : ''}`);
    console.log('    I (kg.m^2)   as a disc   spin-up   holds |  POLLEN dip  recover |  NECTAR dip  recover');
    for (const I of inertias) {
      const r = await measure(motors, I, target, massRatio);
      const mass = ((2 * I) / (shipped.r_fly_m * shipped.r_fly_m)) * 1000;
      console.log(
        `    ${I.toExponential(2).padStart(10)}   ${mass.toFixed(0).padStart(6)} g   ` +
        `${s(r.spinUp).padStart(6)} s  ${r.held.toFixed(0).padStart(4)} | ${r.dip.toFixed(0).padStart(7)} rpm  ${s(r.recover).padStart(6)} s | ` +
        `${r.nDip.toFixed(0).padStart(7)} rpm  ${s(r.nRecover).padStart(6)} s`,
      );
    }
    console.log('');
  }
  console.log('  "as a disc" is the wheel stack that inertia implies at the shipped radius, so a row');
  console.log('  reads as a part to build rather than a number to type.');
  console.log('  A recovery under one frame (0.02 s) means the dip never left the readiness window.');
  console.log(`  "holds" is the speed the wheel is actually sitting at after 12 s on the ${target} rpm`);
  console.log('  command. A wheel that holds well under target is not slow to recover, it never got there.');
}
