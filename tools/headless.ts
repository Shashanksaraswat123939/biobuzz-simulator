/**
 * Run the world headlessly and let the Java brain drive it over the bridge.
 * This is PLAN.md phase 3's acceptance: a real OpMode scoring in lockstep, no browser.
 *
 *   npm run relay                 # terminal 1
 *   npm run tool -- tools/headless.ts --match          # terminal 2
 *   java -cp "java/out;java/out-teamcode" sim.runner.Main --opmode "Auto Leave + Park"
 *
 * Flags: --match (run the clock), --lockstep (wait for the brain each frame),
 *        --seconds N, --alliance red|blue, --ws ws://host:port
 */
import { WebSocket } from 'ws';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import staging from '../assets/staging.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { M_TO_IN } from '../packages/core/src/units.js';
import type { ActuatorFrame, Alliance, BallKind, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

export async function main(argv: string[] = []): Promise<void> {
  const arg = (k: string, d?: string) => {
    const i = argv.indexOf(`--${k}`);
    return i >= 0 ? argv[i + 1] : d;
  };
  const has = (k: string) => argv.includes(`--${k}`);

  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;
  const balls = (staging.balls as { kind: string; pos: number[] }[]).map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));
  const alliance = (arg('alliance', 'red') as Alliance) ?? 'red';
  const preload = Number(arg('preload', '4'));
  const world = new World({ params: p, robot: spec, staging: balls, alliance, seed: p.sim.seed, preload });

  const lockstep = has('lockstep');
  let lockstepActive = lockstep;
  const seconds = Number(arg('seconds', '45'));
  const url = arg('ws', 'ws://localhost:8765')!;

  let act: ActuatorFrame = { seq: 0, motors: {}, servos: {} };
  let brainSeq = -1;
  let waiting = false;

  const ws = new WebSocket(url);
  await new Promise<void>((resolve, reject) => {
    ws.on('open', () => resolve());
    ws.on('error', reject);
  });
  ws.send(JSON.stringify({ type: 'hello', role: 'world' }));
  console.log(`world connected to ${url} (${lockstep ? 'lockstep' : 'free-run'}), ${preload} POLLEN preloaded`);

  let brainUp = false;
  ws.on('message', (data: Buffer) => {
    const msg = JSON.parse(data.toString());
    if (msg.type === 'actuator') {
      act = msg as ActuatorFrame;
      brainSeq = msg.seq;
      brainUp = true;
      waiting = false;
    } else if (msg.type === 'brain') {
      brainUp = Boolean(msg.connected);
    }
  });

  // Wait for the runner rather than racing it: whichever process starts first should win.
  if (!has('no-wait')) {
    const deadline = Date.now() + Number(arg('wait', '30')) * 1000;
    process.stdout.write('waiting for the Java runner');
    while (!brainUp && Date.now() < deadline) {
      await new Promise((r) => setTimeout(r, 200));
      process.stdout.write('.');
    }
    console.log(brainUp ? ' attached' : ' timed out (running anyway)');
  }

  if (has('match')) world.clock.start();

  const frameDt = p.sim.dt * p.sim.substepsPerFrame;
  const totalFrames = Math.round(seconds / frameDt);
  let lastReport = 0;

  for (let f = 0; f < totalFrames; f++) {
    world.setGamepads(emptyGamepad(), emptyGamepad());
    ws.send(JSON.stringify(world.sensors()));

    if (lockstepActive) {
      waiting = true;
      const deadline = Date.now() + 3000;
      while (waiting && Date.now() < deadline) await new Promise((r) => setTimeout(r, 1));
      if (waiting) {
        // The brain went away (its OpMode ended, usually). Carry on in free-run so the
        // match still plays out and the score is assessed.
        console.log('brain stopped answering; continuing in free-run');
        act = { seq: act.seq, motors: {}, servos: {} };
        lockstepActive = false;
        waiting = false;
      }
    } else {
      await new Promise((r) => setImmediate(r));
    }

    world.telemetry = act.telemetry ?? [];
    world.step(act);

    if (world.t - lastReport >= 2) {
      lastReport = world.t;
      const s = world.snapshot();
      const r = s.robot;
      console.log(
        `t=${world.t.toFixed(1)}s ${s.period.padEnd(10)} ` +
          `pose ${r.ftc.x.toFixed(0)},${r.ftc.y.toFixed(0)} @${r.ftc.heading.toFixed(0)}deg  ` +
          `rpm ${r.flywheel.rpm.toFixed(0)}  shots ${r.flywheel.shots}  hopper ${r.hopper.count}  range ${world.sensors().game.truth.upCellRangeIn.toFixed(0)}in  ` +
          `tips ${s.score[alliance].tips}  score ${s.score[alliance].total}` +
          (brainSeq >= 0 ? '' : '   [no brain yet]'),
      );
    }
    if (world.clock.period === 'FINISHED') break;
  }

  const s = world.snapshot();
  const sc = s.score[alliance];
  console.log('\n--- result ---');
  console.log(`period        ${s.period}`);
  console.log(`LEAVE / PARK  ${sc.leave} / ${sc.park}`);
  console.log(`tips          ${sc.tips} (auto ${sc.autoTips})`);
  console.log(`up CELL       ${sc.upCell}`);
  console.log(`score         auto ${sc.auto}  teleop ${sc.teleop}  total ${sc.total}`);
  console.log(`RP            swarm ${sc.rp.swarm}  pollinator1 ${sc.rp.pollinator1}  pollinator2 ${sc.rp.pollinator2}`);
  console.log(`robot ended   ${(s.robot.p[0] * M_TO_IN).toFixed(1)}, ${(s.robot.p[2] * M_TO_IN).toFixed(1)} in (world)`);
  console.log(`snapshot hash ${world.createSnapshotHash()}`);
  ws.close();
}
