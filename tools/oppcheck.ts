/**
 * Does the OPPONENT actually play? One line per seed.
 *
 *   npm run tool -- tools/oppcheck.ts [--seeds 3] [--motors 1]
 *
 * Plays a whole match with the human robot standing still and the bot doing its thing, then
 * reports what the bot managed on its own: shots away, balls in its up CELL, tips, and score.
 * A bot that scores nothing is a bot that is not worth practising against, and that is not a
 * question the browser can answer quickly -- a match takes two and a half minutes to watch.
 *
 * `--motors N` overrides flywheel.motorCount for the run, which makes this the only harness
 * that answers what a motor is WORTH IN POINTS: same bot, same seeds, same everything else.
 * It also reports the BATTERY at the buzzer and the drive speed in the last thirty seconds
 * against the first thirty, because a shooter held on for two and a half minutes is a
 * continuous current draw and the question of whether it starves the drive at the end is a
 * real one.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import staging from '../assets/staging.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { OpponentBot } from '../packages/core/src/robot/opponentBot.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { FlowerTable } from '../packages/core/src/robot/flowerTable.js';
import { worldToFtc } from '../packages/core/src/field/ftcFrame.js';
import { M_TO_IN } from '../packages/core/src/units.js';
import type { BallKind, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const flowerTbl = FlowerTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/flowertable.csv', import.meta.url), 'utf8'));
const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));
const balls = (staging.balls as { kind: string; pos: number[] }[]).map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));

let phaseT: Record<string, number> = {};
let why: Record<string, number> = {};
async function one(seed: number, trace = false, motors?: number) {
  phaseT = {};
  why = {};
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  if (motors) spec.flywheel.motorCount = motors;
  const world = new World({ params: p, robot: spec, staging: balls, alliance: 'red', seed, preload: spec.hopper.capacity, opponent: true });
  const oppBrain = new BuiltinTeleOp(spec, table, loadLandCal(), null, 'blue', flowerTbl);
  const bot = new OpponentBot();
  const opp = world.opponent!;
  const hive = world.hives[opp.alliance];
  const zone = world.geom.zones.find((z) => z.name === 'LOADING' && z.alliance === opp.alliance)!;
  const ranges = table.rows.map((r) => r.range_in);
  const dt = p.sim.dt * p.sim.substepsPerFrame;
  world.clock.start();
  // The BATTERY, sampled all match: amp-seconds out of the pack, and how fast the bot was
  // actually managing to drive early against late. Speed is only counted while it is going
  // somewhere (the collect and position phases), because a bot parked in front of the mouth
  // is standing still by choice and would drag the average down at either end of the match.
  let ampSeconds = 0;
  const early: number[] = [];
  const late: number[] = [];
  while (world.clock.period !== 'FINISHED') {
    const os = world.opponentSensors();
    const loose: [number, number][] = world.balls.balls
      .filter((b) => b.body.isEnabled() && b.state === 'free' && b.kind !== (opp.alliance === 'red' ? 'nectarBlue' : 'nectarRed'))
      .filter((b) => world.balls.pos(b)[1] * M_TO_IN < 12)
      .map((b) => { const f = worldToFtc(world.balls.pos(b)); return [f[0], f[1]] as [number, number]; });
    const g = bot.update(os, dt, {
      mouth: hive.upCellMouthWorld(),
      mouthNormal: hive.upCellMouthNormalWorld(),
      loading: [zone.min[0], zone.min[2], zone.max[0], zone.max[2]],
      halfWidth_m: world.geom.halfWidth_m,
      band_in: [Math.min(...ranges), Math.max(...ranges)],
      flowers: flowerTbl.flowers,
    }, { loose, remaining: world.clock.remaining, period: world.clock.period, shotsTaken: opp.shots });
    world.setOpponentActuators(oppBrain.update(os, g, world.seq, dt));
    world.setGamepads(emptyGamepad(), emptyGamepad());
    world.step({ seq: world.seq, motors: {}, servos: {} });
    phaseT[bot.phase] = (phaseT[bot.phase] ?? 0) + dt;
    ampSeconds += world.opponentBattery!.amps * dt;
    if (bot.phase === 'collect' || bot.phase === 'position') {
      const v = Math.hypot(opp.vel[0], opp.vel[2]);
      if (world.t < 40) early.push(v);
      else if (world.clock.remaining < 30) late.push(v);
    }
    // WHY IT IS NOT SHOOTING, while it is in the phase whose whole job is shooting. Numbers
    // collapsed out so "12 deg of lead" and "19 deg" are one answer.
    if (bot.phase === 'shoot') {
      // Measured once and worth recording: the latch is on for essentially every loop of
      // this phase and the gate is clear for about 72% of them, yet the bot only manages
      // 0.37 shots a second. It is not the bot's logic and it is not the gate -- it is the
      // shooter's own feed cycle, which is the same ceiling the human robot has.
      const h = oppBrain.state.hold;
      const k = !h ? 'clear to fire' : h.replace(/-?[\d.]+/g, 'N');
      why[k] = (why[k] ?? 0) + 1;
    }
    if (trace) {
      if (world.seq % 120 === 0) console.log(`    t=${world.t.toFixed(0)}s ${world.clock.period} ${bot.phase.padEnd(8)} hop=${os.game.hopper} rng=${os.game.truth.upCellRangeIn.toFixed(0)} open=${os.game.truth.upCellOpenDeg.toFixed(0)} rpm=${os.game.flywheelRpm.toFixed(0)} hold=${oppBrain.state.hold || '-'} at=(${os.localizer.x.toFixed(0)},${os.localizer.y.toFixed(0)}) want=(${bot.target[0].toFixed(0)},${bot.target[1].toFixed(0)}) world=(${opp.pos.map((v)=>(v*M_TO_IN).toFixed(0)).join(',')}) red=(${world.robot.pos.map((v)=>(v*M_TO_IN).toFixed(0)).join(',')}) v=${Math.hypot(opp.vel[0],opp.vel[2]).toFixed(3)}`);
    }
  }
  if (trace) console.log('    time per phase:', Object.entries(phaseT).map(([k, v]) => `${k} ${v.toFixed(0)}s`).join(', '));
  const sc = world.scorer.state[opp.alliance];
  return {
    seed, shots: opp.shots, inCell: world.landedInUpCell(opp.alliance),
    tips: sc.tips, score: sc.total, phase: bot.phase, note: bot.note,
    // WHERE THE POINTS CAME FROM, and more usefully where they did NOT. A bot that never
    // touches a FLOWER leaves the whole endgame on the table and a total does not say so.
    leave: sc.leave, park: sc.park, upCell: sc.upCell, flower: sc.flower,
    garden: sc.garden, bottom: sc.bottomNectar,
    phaseT: { ...phaseT }, why: { ...why },
    soc: world.opponentBattery!.soc, volts: world.opponentBattery!.volts,
    ampHours: ampSeconds / 3600,
    earlyV: early.length ? early.reduce((a, b) => a + b, 0) / early.length : NaN,
    lateV: late.length ? late.reduce((a, b) => a + b, 0) / late.length : NaN,
  };
}

export async function main(argv: string[] = []): Promise<void> {
  await initPhysics();
  const i = argv.indexOf('--seeds');
  const n = i >= 0 ? Number(argv[i + 1]) : 3;
  const mi = argv.indexOf('--motors');
  const motors = mi >= 0 ? Number(argv[mi + 1]) : undefined;
  console.log('\nTHE OPPONENT BOT, a full match per seed, with the human robot standing still.\n');
  console.log(`  flywheel motors: ${motors ?? (robotSpec as unknown as RobotSpec).flywheel.motorCount ?? 1}
`);
  console.log('  seed   shots   in its CELL   tips   points   ended');
  const runs: Awaited<ReturnType<typeof one>>[] = [];
  for (let k = 0; k < n; k++) {
    const r = await one(11 + k * 17, argv.includes('--trace'), motors);
    runs.push(r);
    console.log(`  ${String(r.seed).padStart(4)}   ${String(r.shots).padStart(5)}   ${String(r.inCell).padStart(11)}   ${String(r.tips).padStart(4)}   ${String(r.score).padStart(6)}   ${r.phase}`);
  }
  const mean = (a: number[]) => a.reduce((x, y) => x + y, 0) / (a.length || 1);
  console.log(`\n  mean: ${mean(runs.map((r) => r.shots)).toFixed(1)} shots, ${mean(runs.map((r) => r.tips)).toFixed(1)} tips, ${mean(runs.map((r) => r.score)).toFixed(1)} points`);
  console.log(`  last note: ${runs[runs.length - 1]?.note ?? '-'}\n`);
  console.log('');
  console.log('  WHERE THE POINTS COME FROM (mean per match), and where they do not:');
  const m = (f: (r: typeof runs[0]) => number) => mean(runs.map(f)).toFixed(1);
  console.log(`    TIPS        ${m((r) => r.tips * 20).padStart(6)}   (${m((r) => r.tips)} tips)`);
  console.log(`    LEAVE       ${m((r) => (r.leave ? 3 : 0)).padStart(6)}`);
  console.log(`    PARK        ${m((r) => (r.park ? 5 : 0)).padStart(6)}`);
  console.log(`    up CELL     ${m((r) => r.upCell * 2).padStart(6)}   (${m((r) => r.upCell)} balls at the buzzer)`);
  console.log(`    FLOWERs     ${m((r) => r.flower * 2 + r.bottom * 5).padStart(6)}   (${m((r) => r.flower)} owned elements, ${m((r) => r.bottom)} bottom bonuses)`);
  console.log(`    GARDEN      ${m((r) => r.garden).padStart(6)}`);
  console.log('');
  console.log('  while IN the shoot phase, why it was not firing (share of loops):');
  const tot = runs.reduce((a, r) => a + Object.values(r.why).reduce((x, y) => x + y, 0), 0);
  const agg: Record<string, number> = {};
  for (const r of runs) for (const [k, v] of Object.entries(r.why)) agg[k] = (agg[k] ?? 0) + v;
  for (const [k, v] of Object.entries(agg).sort((a, b) => b[1] - a[1]).slice(0, 6)) {
    console.log(`    ${((v / Math.max(1, tot)) * 100).toFixed(0).padStart(3)}%  ${k}`);
  }
  console.log('');
  console.log('  THE BATTERY, at the buzzer (3.0 Ah pack, flywheel up for most of the match):');
  console.log(`    drawn       ${m((r) => r.ampHours)} Ah of 3.0   ->  ${(mean(runs.map((r) => r.soc)) * 100).toFixed(0)}% left, ${m((r) => r.volts)} V under load`);
  console.log(`    drive speed ${m((r) => r.earlyV)} m/s in the first 40 s   ->  ${m((r) => r.lateV)} m/s in the last 30 s`);
  console.log('    (speed counted only while the bot is driving somewhere, not while it sits and fires.)');
  console.log('');
  console.log('  seconds per phase, mean:');
  for (const k of new Set(runs.flatMap((r) => Object.keys(r.phaseT)))) {
    console.log(`    ${k.padEnd(10)} ${mean(runs.map((r) => r.phaseT[k] ?? 0)).toFixed(0).padStart(4)} s`);
  }
  console.log('');
}
