/**
 * THE FEED TUBE, FRAME BY FRAME. Where is every ball the robot holds while it fires?
 *
 *   npm run tool -- tools/feedprobe.ts [--secs 8] [--range 45]
 *
 * tools/releasecheck.ts found that a third of the brain's feed pulses release nothing, and
 * that the delay from commit to release is either 0.13 s (a ball was waiting on the gate) or
 * 0.38 s (it was not). This prints, for every frame the gate is not fully closed, each held
 * ball's height relative to the gate plate and the nip line, so the reason a ball is not on
 * the plate when the pulse comes can be read off rather than guessed.
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { inches } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

export async function main(argv: string[] = []): Promise<void> {
  const num = (k: string, d: number) => { const i = argv.indexOf(`--${k}`); return i >= 0 ? Number(argv[i + 1]) : d; };
  const secs = num('secs', 8), range_in = num('range', 45), fireAt = num('fireat', 1.0);
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  spec.flywheel.minLandProb = 0;
  const staging = Array.from({ length: 40 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging, alliance: 'red', seed: 5 });
  for (const b of w.balls.balls) w.balls.park(b);
  const mouth = w.hives.red.upCellMouthWorld();
  const limit = w.geom.halfWidth_m - 0.35;
  const d = inches(range_in);
  let spot: Vec3 | null = null;
  for (let deg = 0; deg <= 88 && !spot; deg += 2) {
    for (const sign of [1, -1]) {
      const a = (deg * Math.PI) / 180;
      const c: Vec3 = [mouth[0] + sign * d * Math.sin(a), spec.chassis.height_m / 2 + spec.chassis.clearance_m, mouth[2] + d * Math.cos(a)];
      if (Math.abs(c[0]) < limit && Math.abs(c[2]) < limit) { spot = c; break; }
    }
  }
  if (!spot) throw new Error('no spot');
  w.robot.place(spot, (Math.atan2(mouth[0] - spot[0], mouth[2] - spot[2]) * 180) / Math.PI);
  const brain = new BuiltinTeleOp(spec, table);
  let loaded = 0;
  const r = p.ball.pollen.d_m / 2;
  const topY = spec.turret.muzzleHeight_m - spec.chassis.height_m / 2 - spec.chassis.clearance_m;
  const nipLine = topY - r;                 // ball centre at or above this: at the nip
  const gateY = topY - r * 2.1;             // the plate
  console.log(`nip line ${(nipLine * 1000).toFixed(0)} mm, gate plate ${(gateY * 1000).toFixed(0)} mm, ball r ${(r * 1000).toFixed(1)} mm (robot-local Y, chassis centre = 0)`);
  console.log('t      gate  pulse ready  shots  held  [ball heights, mm; * = in shaft column]');
  const g: GamepadState = emptyGamepad();
  let wasPulsing = false;
  for (let i = 0; i < secs * 60; i++) {
    while (w.robot.heldBalls().length < spec.hopper.capacity && loaded < staging.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    if (i === 30) { brain.state.flywheelOn = true; }
    if (i === Math.round(fireAt * 60)) { brain.state.firing = true; }
    w.setGamepads(g, emptyGamepad());
    const before = w.robot.shots;
    const act = brain.update(w.sensors(), g, w.seq, 1 / 60);
    const st = brain.state;
    w.step(act);
    const gate = w.snapshot().robot.transfer.gate;
    const fired = w.robot.shots > before;
    const commit = st.pulsing && !wasPulsing;
    wasPulsing = st.pulsing;
    if (gate > 0.02 || commit || fired || i % 30 === 0 || (argv.includes('--all') && i % 6 === 0)) {
      const c = w.robot.pos;
      const hs = w.robot.heldBalls().map((b) => {
        const q = w.balls.pos(b);
        const l = w.robot.toLocal([q[0] - c[0], q[1] - c[1], q[2] - c[2]]);
        const inCol = Math.abs(l[0]) < 0.03 && Math.abs(l[2] + 0.02) < 0.03;
        return `${(l[1] * 1000).toFixed(0)}${inCol ? '*' : ''}`;
      });
      console.log(`${w.t.toFixed(2).padStart(6)} ${gate.toFixed(2)}  ${st.pulsing ? 'P' : '-'}${commit ? 'C' : ' '}   ${st.ready ? 'R' : '-'}     ${String(w.robot.shots).padStart(3)}${fired ? '!' : ' '}  ${String(w.robot.heldBalls().length).padStart(3)}   ${hs.join(' ')}   ${st.hold}`);
    }
  }
}
