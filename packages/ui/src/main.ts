/**
 * App shell: the loop, the input, and the panels.
 *
 * All physics lives in @core and all drawing in @render; this file is glue and layout.
 * Three modes share one world:
 *
 *   practice — you drive. Nothing is scripted.
 *   data     — AutoDriver drives, sweeping a range band and logging every shot, then
 *              @core/analysis/report says what went wrong.
 *   test     — sandbox. Every constant is a slider and @core/analysis/sensitivity shows
 *              what each one is doing to the shot before you take it.
 *
 * Every control on the page is listed in the Guide tab with what it does. If a button is
 * here it is wired to something; there is nothing that only looks like a control.
 */
import paramsJson from '../../../config/params.json';
import { fromCellLocal } from '@core/field/geometry.js';
import robotJson from '../../../config/robot.json';
import stagingJson from '../../../assets/staging.json';
import shotCsv from '../../../java/teamcode/assets/shottable.csv?raw';
import shotZoneJson from '../../../config/shotzone.json';
import tagMapJson from '../../../config/tagmap.json';
import { ftcToWorld } from '@core/field/ftcFrame.js';
import tagOffsets from '../../../config/tagoffsets.json';

import { World, initPhysics, emptyGamepad } from '@core/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '@core/robot/builtinTeleOp.js';
import { loadLandCal } from '@core/robot/loadCal.js';
import { loadFlowerTable } from '@core/robot/loadFlower.js';
/** The tubes, read once: the bot needs them as well as the brain. */
const flowerTbl = loadFlowerTable();
import { buildMotor } from '@core/physics/motor.js';
import { AutoDriver, defaultPlan, type AutoPlan } from '@core/robot/autoDriver.js';
import { AutoRoutine } from '@core/robot/autoRoutine.js';
import { OpponentBot } from '@core/robot/opponentBot.js';
import { worldToFtc } from '@core/field/ftcFrame.js';
import { simulateShot, rpmToSpeed } from '@core/physics/ballistics.js';
import { knobs, predict, type Prediction } from '@core/analysis/sensitivity.js';
import { analyse, toCsv, type Report } from '@core/analysis/report.js';
import { Trace } from '@core/analysis/trace.js';
import { M_TO_IN, DEG, RAD, inches } from '@core/units.js';
import { Scene, type CameraMode, type ZonePayload } from '@render/scene.js';
import type { ActuatorFrame, Alliance, BallKind, GamepadState, Params, RobotSpec, Snapshot, Vec3 } from '@core/types.js';
import { readKeyboard, installKeyboard, remap, type Keys, type Paddles } from './input.js';
import { m, cm, cmSigned, mps } from './units.js';
import { Joysticks } from './joystick.js';
import { buildTunePanel, type Tunable } from './tune.js';
import { WorldBridge } from './bridge.js';
import { CONTROLS, renderGuide } from './guide.js';
import { groupPlot, errorVsRange, histogram } from './charts.js';

const $ = <T extends HTMLElement = HTMLElement>(sel: string) => document.querySelector(sel) as T;
const $$ = <T extends HTMLElement = HTMLElement>(sel: string) => Array.from(document.querySelectorAll<T>(sel));

type Mode = 'practice' | 'auto' | 'collect' | 'test';

const baseParams = paramsJson as unknown as Params;
const baseRobot = robotJson as unknown as RobotSpec;
const staging = (stagingJson.balls as { kind: string; pos: number[] }[]).map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));
const shotTable = ShotTable.fromCsv(shotCsv);

let params: Params = structuredClone(baseParams);
let robotSpec: RobotSpec = structuredClone(baseRobot);
let world: World;
let scene: Scene;
/** On-screen sticks, folded into the same gamepad frame as the keyboard and a real pad. */
let sticks: Joysticks;
let brain: BuiltinTeleOp;
let auto: AutoDriver | null = null;
/** The match autonomous, as opposed to the data sweep. Only one of the two ever runs. */
let routine: AutoRoutine | null = null;
let plan: AutoPlan = defaultPlan();
let alliance: Alliance = 'red';
let mode: Mode = 'practice';
let paused = false;
let autoLoad = false;
let useJavaBrain = false;
/**
 * Sim time per wall-clock second. A 40-shot sweep is several minutes at 1x, and nobody
 * needs to watch it; the physics is unchanged, it just gets more steps per frame.
 */
let turbo = 1;
let lastAct: ActuatorFrame = { seq: 0, motors: {}, servos: {} };
const keys: Keys = installKeyboard();
const bridge = new WorldBridge();
/**
 * Flight recorder. It survives a rebuild on purpose: comparing the run before a variable
 * changed with the run after is the whole point, and clearing it on Reset would throw the
 * "before" away every time.
 */
const trace = new Trace();
/** The best land rate the calibration in config/landcal.json actually reached. */
const landCalCeiling: number | undefined = loadLandCal()?.ceiling;

// ---------------------------------------------------------------- setup

async function boot(): Promise<void> {
  await initPhysics();
  build();
  buildTunePanel($('#tune-rows'), params, robotSpec, onTune);
  renderGuide($('#guide'));
  sticks = new Joysticks($('#app'));
  wireUi();
  setMode('practice');
  requestAnimationFrame(loop);
}

function build(): void {
  world = new World({ params, robot: robotSpec, staging, alliance, seed: params.sim.seed, preload: 4, opponent: opponentOn });
  brain = new BuiltinTeleOp(robotSpec, shotTable, loadLandCal(), null, alliance, flowerTbl);
  // A HANDLE FOR MEASURING THE APP ITSELF. Every harness drives a World from Node; none of
  // them drives THIS one, through this input path, at this frame rate -- and "it aims at
  // the wrong hive" is a report about this one. Read-only from the console, never written.
  // `loop` is the app's own frame, exposed so a measurement can step it with synthetic
  // timestamps when the tab is hidden and requestAnimationFrame is not firing.
  (window as unknown as { __sim: unknown }).__sim = { get world() { return world; }, get brain() { return brain; }, loop };
  // The opponent gets its own brain, not a share of yours: it has its own flywheel to spin,
  // its own aim to hold and its own readiness to wait for.
  oppBrain = world.opponent ? new BuiltinTeleOp(robotSpec, shotTable, loadLandCal(), null, alliance === 'red' ? 'blue' : 'red', flowerTbl) : null;
  bot = world.opponent ? new OpponentBot() : null;
  // ZERO THE FIELD FRAME ON THE DRIVER, not on the world's axes.
  //
  // Only field-centric mode (hold Y) uses this; the default is robot-centric, where W is the
  // intake and nothing rotates. It still has to be right for anyone who holds Y.
  //
  // Field-centric rotates the stick by (yaw - headingZero). With headingZero at 0 that makes
  // "up" mean world +Z -- an axis with no relationship to where anyone is standing. The
  // alliance stations are on the +-X walls, so for a red driver "away from me" is +X, and
  // up-stick would have crabbed the robot sideways across their view from the first frame.
  //
  // The robot starts on its own wall facing down-field, so its starting heading IS the
  // driver's "away". Backspace re-zeros it to wherever the robot faces now.
  brain.state.headingZero = world.robot.yaw * RAD;
  auto = null;
  routine = null;
  const specs = world.balls.balls.map((b) => ({ id: b.id, r: b.radius, kind: b.kind as string }));
  const mode0 = scene?.cameraMode ?? 'orbit';
  const traj = scene?.showTrajectory ?? true;
  // ON BY DEFAULT. The up CELL opens toward the audience, so the whole alliance-wall side
  // of the field is behind the mouth and no launch from there can enter -- and the first
  // thing a driver does is press Fire from the start tile and watch nothing happen. The map
  // answers that before it is asked: it is green only where a perfectly aimed shot clears
  // the gate, and it already models the facing (a shooter behind the pocket gets a negative
  // near range, so those squares read zero).
  const zone0 = scene?.showShotZone ?? true;
  scene = new Scene($<HTMLCanvasElement>('#view'), world.geom, specs, robotSpec, alliance);
  scene.cameraMode = mode0;
  scene.showTrajectory = traj;
  // Off until asked for: it is a model map, and a coloured floor that is always on reads
  // like a measurement of where the robot scores, which it is not.
  // The two maps travel together: where a shot is worth taking, and where the goal can be
  // seen at all. Painting the first without the second shows a field the robot cannot use.
  scene.setShotZone({
    ...(shotZoneJson as unknown as ZonePayload),
    tagVisible: new Set(
      (tagMapJson.cells as { x_in: number; z_in: number; visible: boolean }[])
        .filter((c) => c.visible)
        .map((c) => `${c.x_in},${c.z_in}`),
    ),
  });
  scene.showShotZone = zone0;
  scene.resize();
  prediction = null;
  // Handy from the console: sim.world.hives.red.angle, sim.world.robot.flywheelRpm, ...
  (globalThis as unknown as { sim: unknown }).sim = { world, scene, params, robotSpec, brain };
}

/**
 * A knob moved. Cheap ones are already written into `params`/`robotSpec` by the slider and
 * the world reads those objects live, so there is nothing more to do; expensive ones are
 * baked into rigid bodies at construction and need the rebuild the button offers.
 */
function onTune(t: Tunable): void {
  prediction = null;
  $<HTMLButtonElement>('#apply-reset').classList.toggle('on', !!t.rebuild);
}

// ---------------------------------------------------------------- input

/**
 * SLEW THE STICKS. A key is on or off, so a keyboard asks for 0% or 100% of a 1.2 m/s
 * drivetrain with nothing in between -- tap W and the robot leaves. That is what "it moves
 * way too fast" is: not the top speed, which is what a 435 rpm goBILDA on 96 mm wheels
 * actually does, but the fact that every input is a step.
 *
 * Ramping to full over about a fifth of a second gives the thumb something to aim with and
 * costs nothing real: a physical stick has the same travel, and the drivetrain cannot follow
 * a step anyway. Release is deliberately faster than push, because stopping should not have
 * to be planned. A real analog stick is passed through untouched -- it is already analog.
 */
const VIEW_YAW_RATE = 2.4;   // rad/s at full stick: a half turn in 1.3 s
const VIEW_PITCH_RATE = 1.4;
const SLEW_UP = 5.0;    // full deflection in 0.2 s
const SLEW_DOWN = 12.0; // and back to nothing in 0.08 s
/**
 * THE YAW BUTTON RAMPS SLOWER THAN THE DRIVE STICKS DO, and that is the whole point of it.
 *
 * X and B are buttons, so they have no middle. At the drive ramp they reached full rotation
 * in 0.2 s, which is 279 deg/s measured -- and the fire gate refuses above
 * `turret.fireYawCap_dps`, 70 deg/s, because past that the turret setpoint outruns the axis.
 * So every turn input killed the shot, which is exactly the "sometimes it fires and sometimes
 * it does not" report: it was not random, it was whether a thumb was on a turn button.
 *
 * At 1.6 per second a tap of about a sixth of a second lands near a quarter deflection, which
 * is about the cap -- so short taps are a precise, still-firing correction and a held button
 * still builds to full rotation for getting the nose round quickly. That is what the analog
 * stick gave and what the button took away.
 */
const YAW_SLEW_UP = 1.6;
const slewed = { lx: 0, ly: 0, rx: 0 };
let lastFrameDt = 1 / 60;
function slew(now: number, want: number, dt: number): number {
  const rate = Math.abs(want) > Math.abs(now) ? SLEW_UP : SLEW_DOWN;
  const step = rate * dt;
  return Math.abs(want - now) <= step ? want : now + Math.sign(want - now) * step;
}

/**
 * Apply the slew to a digital (keyboard or on-screen) stick frame. The on-screen joystick is
 * analog in principle, but a thumb on a touch screen snaps to the edge much as a key does, so
 * it gets the same treatment; a real gamepad stick does not.
 */
function rampDigital(s: GamepadState, analogSticks = false): GamepadState {
  const dt = Math.min(0.05, lastFrameDt);
  // YAW IS ALWAYS RAMPED, even on a real controller: it comes from the X and B BUTTONS now,
  // and an un-ramped button would slam the chassis to full rotation in one frame. The
  // translation sticks are only ramped when they are digital -- slewing a real analog stick
  // just adds 0.2 s of lag to an input that is already smooth.
  const rate = Math.abs(s.right_stick_x) > Math.abs(slewed.rx) ? YAW_SLEW_UP : SLEW_DOWN;
  const step = rate * dt;
  const wantRx = s.right_stick_x;
  slewed.rx = Math.abs(wantRx - slewed.rx) <= step ? wantRx : slewed.rx + Math.sign(wantRx - slewed.rx) * step;
  if (analogSticks) return { ...s, right_stick_x: slewed.rx };
  slewed.lx = slew(slewed.lx, s.left_stick_x, dt);
  slewed.ly = slew(slewed.ly, s.left_stick_y, dt);
  return { ...s, left_stick_x: slewed.lx, left_stick_y: slewed.ly, right_stick_x: slewed.rx };
}

/**
 * Free-running top speed of the drivetrain, m/s. This is the ceiling the gear bar is a
 * fraction OF. The robot never quite reaches it under load, which is the honest thing for a
 * speed LIMIT to show -- it is the limit being set, not the speed being achieved.
 *
 * config/robot.json's measured value first, and the motor's free speed through its gearing
 * times the wheel radius only as a fallback. The two agreed at 2.19 m/s, which is the whole
 * reason to prefer the measured one: the derivation is right until someone fits a wheel with
 * a different rolling resistance, and then it is silently wrong while the measurement is not.
 */
const topSpeed_ms = robotSpec.drivetrain.freeSpeed_mps
  || buildMotor(robotSpec.drivetrain.motors.fl).freeOmega * robotSpec.drivetrain.wheelRadius_m;

/** The physical right stick, which the brain never sees: it drives the camera. */
let viewStick = { x: 0, y: 0 };

function gamepadState(): GamepadState {
  const g = navigator.getGamepads?.().find((p) => p);
  const k = readKeyboard(keys);
  if (!g) {
    viewStick = { x: k.right_stick_x, y: k.right_stick_y };
    return rampDigital(remap(sticks.merge(k), k.paddles));
  }
  const dz = (v: number) => (Math.abs(v) < 0.09 ? 0 : v);
  const btn = (i: number) => g.buttons[i]?.pressed ?? false;
  // WHAT DOES YOUR PAD ACTUALLY REPORT? Paddles are not standardised: an Xbox Elite ships
  // with its paddles MIRRORING the face buttons, so "M1" arrives as button 0 (A) and not as
  // the index 16 this layout assumes -- which is why the paddle changed the speed gear
  // instead of firing. Both indices fire now; the console still prints what is pressed.
  padWatch(g);
  // M1/M2 sit past the standard 17 on the pads that have them. On a pad that does not, they
  // fall back to D-pad left and right -- which is where the manual turret nudge lived before,
  // so nothing is lost and nothing is doubled up.
  const paddles: Paddles = {
    // THE M1 PADDLE FIRES; ITS FALLBACKS STILL NUDGE. btn(16) used to be OR'd into m1 with
    // D-pad left and the comma key, so binding "M1" to the latch would have taken the
    // anticlockwise turret nudge away from every pad without paddles as well. The paddle is
    // split out instead: 16 fires, 14 and comma go on nudging.
    m1: btn(14) || k.paddles.m1,
    m2: btn(17) || btn(15) || k.paddles.m2,
    // 16 IS THE PADDLE BLOCK; 0 IS THE SAME PADDLE MIRRORING A. A pad that mirrors reports
    // nothing on 16 at all, so reading both costs a pad with real paddles nothing and is the
    // only binding a mirrored one can have. Gear-down moves off A on the pad because of it:
    // Y wraps round instead, and R/F on the keyboard are untouched.
    fire: btn(16) || btn(0),
    rezero: k.paddles.rezero,
  };
  const merged: GamepadState = {
    left_stick_x: dz(g.axes[0] ?? 0) || k.left_stick_x,
    left_stick_y: dz(g.axes[1] ?? 0) || k.left_stick_y,
    right_stick_x: dz(g.axes[2] ?? 0) || k.right_stick_x,
    right_stick_y: dz(g.axes[3] ?? 0) || k.right_stick_y,
    // L2 and R2 are the back and forward throttles now, read as the analog values they are.
    left_trigger: Math.max(g.buttons[6]?.value ?? 0, k.left_trigger),
    right_trigger: Math.max(g.buttons[7]?.value ?? 0, k.right_trigger),
    a: k.a,   // read above as the fire paddle: a mirrored M1 arrives here
    b: (g.buttons[1]?.pressed ?? false) || k.b,
    x: (g.buttons[2]?.pressed ?? false) || k.x,
    y: (g.buttons[3]?.pressed ?? false) || k.y,
    left_bumper: btn(4) || k.left_bumper,   // L1: auto-aim toggle
    right_bumper: (g.buttons[5]?.pressed ?? false) || k.right_bumper,
    dpad_up: (g.buttons[12]?.pressed ?? false) || k.dpad_up,
    dpad_down: (g.buttons[13]?.pressed ?? false) || k.dpad_down,
    dpad_left: false,    // read above as M1's fallback, not as a D-pad press
    dpad_right: false,
    start: (g.buttons[9]?.pressed ?? false) || k.start,
    back: (g.buttons[8]?.pressed ?? false) || k.back,
    left_stick_button: btn(10) || k.left_stick_button,
    right_stick_button: btn(11) || k.right_stick_button,
  };
  viewStick = { x: merged.right_stick_x, y: merged.right_stick_y };
  return rampDigital(remap(sticks.merge(merged), paddles), true);
}

/** Same sticks, edge-triggered buttons released: a toggle fires once per animation frame. */
const neutralEdges = (g: GamepadState): GamepadState => ({ ...g, a: false, x: false, y: false, right_bumper: false, dpad_up: false, left_stick_button: false });


/** The gamepad's own mode and utility buttons, read once per animation frame. */
let padPrev: GamepadState | null = null;
function padShortcuts(g: GamepadState): void {
  const p = padPrev;
  padPrev = { ...g };
  if (!p) return;
  const edge = (a: boolean, b: boolean) => a && !b;
  // Y and L3 used to be a second job for buttons the brain also reads -- auto-fill and pause
  // fired at the same time as the speed gear and the re-zero. One button, one action; both
  // are still on the deck and the keyboard.
  if (edge(g.start, p.start)) primary();                            // Start: the green button
  if (edge(g.back, p.back)) cycleMode();                            // Select: next mode
  // The camera used to cycle on the D-pad. The right stick IS the camera now, and D-pad up is
  // the flywheel pre-spin the brain reads, so cycling here would be a second job for it.
  // Keys 1-5 and the View menu still switch modes.
  if (edge(g.dpad_down, p.dpad_down)) cycleCamera(1);
}

// ---------------------------------------------------------------- loop

let frameMs = 0;
let drawTick = 0;
let acc = 0;
let last = performance.now();

function loop(now: number): void {
  requestAnimationFrame(loop);
  const dtReal = Math.min(0.1, (now - last) / 1000);
  lastFrameDt = dtReal;
  last = now;

  const human = gamepadState();
  padShortcuts(human);
  // THE RIGHT STICK IS THE VIEW. Rates, not steps, so the pan speed is the same on a 144 Hz
  // screen as on a 30 Hz one; squared so a small deflection is a fine correction and a full
  // one is a fast look-behind.
  const sq = (v: number) => v * Math.abs(v);
  scene.nudgeView(-sq(viewStick.x) * VIEW_YAW_RATE * dtReal, sq(viewStick.y) * VIEW_PITCH_RATE * dtReal);

  const t0 = performance.now();
  if (!paused) {
    const frame = params.sim.dt * params.sim.substepsPerFrame;
    // The step size never changes with turbo -- only how many of them fit in one animation
    // frame -- so a fast run and a slow run produce the same trajectory.
    acc = Math.min(acc + dtReal * turbo, frame * 4 * turbo);
    let first = true;
    while (acc >= frame) {
      const sensors = world.sensors();
      // In data mode the AutoDriver replaces the human's hands, not the brain: it produces
      // a gamepad, and the same TeleOp code path flies the shot.
      const g = auto && auto.phase !== 'done'
        ? auto.update(sensors, frame, world.robot.shots)
        : routine && routine.phase !== 'done'
          ? routine.update(sensors, frame, world.robot.shots, world.clock.remaining, brain.target())
          : first ? human : neutralEdges(human);
      world.setGamepads(g, emptyGamepad());
      if (bot && oppBrain && world.opponent) {
        const os = world.opponentSensors();
        world.setOpponentActuators(oppBrain.update(os, bot.update(os, frame, opponentField(), opponentSight()), world.seq, frame));
      }
      lastAct = useJavaBrain ? bridge.exchange(sensors) : brain.update(sensors, g, world.seq, frame);
      world.telemetry = lastAct.telemetry ?? [];
      world.step(lastAct);
      acc -= frame;
      if (first) keys.pressed.clear();
      first = false;
      if (autoLoad && world.seq % 20 === 0) topUpHopper();
    }
  }
  frameMs = frameMs * 0.9 + (performance.now() - t0) * 0.1;

  // While fast-forwarding, draw one frame in four. The 675k-triangle CAD field is a real
  // cost, and during a collection run the physics is what you are here for.
  const snap = world.snapshot();
  if (!paused) trace.sample(snap, alliance);
  drawTick++;
  if (turbo === 1 || drawTick % 4 === 0) {
    // WHERE THE ROBOT THINKS THE MOUTH IS, drawn beside where it actually is.
    //
    // Built from the robot's own estimate and nothing else: the fused pose (odometry with tag
    // corrections) plus the target estimate's range and bearing, which is the tag when it can
    // see one and the surveyed geometry carried on odometry when it cannot. The HEIGHT is the
    // one baked number in it, because the robot never estimates height -- the shot table
    // encodes it in every hood angle it was solved with.
    //
    // Deliberately NOT world.aimPoint(): that is the physics telling you the answer, which is
    // the thing this marker exists to be compared against.
    {
      const t = brain.target();
      const p = brain.pose_();
      if (t.valid) {
        const b = (p.heading + t.azimuthDeg) * Math.PI / 180;
        scene.setBelief(ftcToWorld([
          p.x + t.rangeIn * Math.cos(b),
          p.y + t.rangeIn * Math.sin(b),
          tagOffsets.states.A.mouthHeight_in,
        ] as Vec3));
      } else {
        scene.setBelief(null);
      }
    }
    scene.update(snap, world.aimPoint(), predictShot(snap));
    // THE LINE OF SIGHT the tag pipeline is working along. Green when the camera decoded this
    // frame, red when the geometry refuses -- which is the only on-screen answer to "why is it
    // not shooting". Drawn from where the MODEL puts the camera: the robot's tracked point at
    // muzzle height, because tagCamera.ts has no mount offset.
    {
      const hive = world.hives[alliance];
      const rp = world.robot.pos;
      const eye: Vec3 = [rp[0], robotSpec.turret.muzzleHeight_m, rp[2]];
      scene.setSightLine(hive.upCellTagWorld(), eye, world.tagCam.isSeeing(world.t));
      // THE PATROL SECTOR, re-aimed every frame: a TIP swaps which CELL is up and the new one
      // opens the other way, so a wedge painted once would point at a pocket that has gone.
      const inner = inches(robotSpec.shot?.minRange_in ?? 30);
      const outer = inches(robotSpec.sensors.tag.maxRange_in ?? 120);
      scene.setPatrolSector(
        hive.upCellMouthWorld(), hive.upCellMouthNormalWorld(),
        robotSpec.shot?.patrolHalfAngle_deg ?? 0, inner, outer,
      );
    }
    scene.render();
    paint(snap);
  }
}

/**
 * THE OPPONENT. Off by default -- a second robot on the field changes every measurement the
 * Data and Test modes take, so it is a thing you switch on to practise against, not a thing
 * that is quietly always there.
 */
let opponentOn = false;
let oppBrain: BuiltinTeleOp | null = null;
let bot: OpponentBot | null = null;

let trajCache: Vec3[] | null = null;
let trajAge = 0;

/**
 * The arc the ball would fly if it were fired right now.
 * Recomputed every few frames at a coarse step: at 1/240 over a two-second lob this is a
 * ~500-step integration, and doing it every frame starved the physics loop badly enough
 * that the sim ran at well under real time.
 */
function predictShot(s: Snapshot): Vec3[] | null {
  if (!scene.showTrajectory) return null;
  if (s.robot.flywheel.rpm < 200 && brain.state.targetRpm <= 0) return null;
  // NO FIX, NO PREDICTION. While the camera is searching there is no shot being lined up:
  // the turret is sweeping to find the tag, and the table is being asked for a range of 0,
  // which it clamps to its nearest row. Drawing that produced a confident yellow arc swinging
  // out across the field at whatever the sweep happened to be pointing at -- the UI asserting
  // an intention the robot does not have, and the first thing anyone asks about. The blue
  // trail beside it is a record of a ball that really flew, so it stays either way.
  if (brain.target().scanning) return null;
  // EVERY OTHER FRAME, not every fourth. The aim moves while you drive, and a curve four
  // frames stale lags the turret visibly -- it was still pointing where the robot used to be
  // aiming. The integrator is the expensive part, so this is as cheap as it can be made
  // without the curve reading as laggy; the live trail beside it is free.
  if (trajAge-- > 0 && trajCache) return trajCache;
  trajAge = 1;
  const mz = world.robot.muzzle();
  // THE SHOT THE AIM IS SOLVING FOR, not the one the wheel could take this instant.
  //
  // This used to integrate the LIVE rpm and the live hood, so while the wheel spun up the arc
  // crawled out of the muzzle and fell short of everything -- it only agreed with the aim in
  // the moment the shot actually went. As an aiming aid that is backwards: what you want to
  // see is where the ball WILL go when the gate opens, so you can put it on the mouth and
  // wait. Once the wheel is at speed and the hood has arrived the two are the same curve.
  const targetRpm = brain.state.targetRpm > 0 ? brain.state.targetRpm : s.robot.flywheel.rpm;
  const speed = rpmToSpeed(targetRpm, robotSpec.flywheel.k, robotSpec.flywheel.r_fly_m);
  // THE BALL LEAVES WITH THE MUZZLE'S VELOCITY, and so must the curve. This integrated the
  // exit velocity alone, which is exact standing still and wrong by the whole lead on the
  // move: at 1.2 m/s across the mouth the aim correctly points the muzzle 30-40 deg
  // upstream of the hive, and this drew a parabola landing 30-40 deg upstream of the hive --
  // over by the opponent's -- while the actual ball, exit plus chassis, went in. Measured
  // in the app itself (window.__sim, 13.7 s of full-stick driving): the four worst frames
  // had the muzzle 38-40 deg off our CELL with the chassis not yawing and the aim error
  // under 3 deg. "It aims at the wrong hive" was this line. Same term as Robot.launch():
  // v_cg + omega x r, with r from the tracked point to the muzzle.
  const cv = s.robot.v;
  const om = (s.robot.omegaDps * Math.PI) / 180;
  const rx = mz.pos[0] - s.robot.p[0], rz = mz.pos[2] - s.robot.p[2];
  const gx = mz.dir[0] * speed + cv[0] - om * rz;
  const gy = mz.dir[1] * speed + cv[1];
  const gz = mz.dir[2] * speed + cv[2] + om * rx;
  const gspeed = Math.hypot(gx, gy, gz);
  const traj = simulateShot(params, {
    from: mz.pos, azimuth: Math.atan2(gx, gz), elevation: Math.asin(gy / gspeed), speed: gspeed,
    radius: params.ball.pollen.d_m / 2,
    mass: params.ball.pollen.m_kg,
    spin: robotSpec.flywheel.type === 'single' ? speed / (params.ball.pollen.d_m / 2) : 0,
  }, 8, 1 / 120);
  trajCache = traj.points;
  return trajCache;
}

// ---------------------------------------------------------------- paint

const row = (label: string, value: string, cls = '') => `<span>${label}</span><b class="${cls}">${value}</b>`;

function paint(s: Snapshot): void {
  const r = s.robot;
  const mins = Math.floor(s.remaining / 60);
  const secs = Math.floor(s.remaining % 60);
  $('#period').textContent = s.period;
  $('#time').textContent = `${mins}:${String(secs).padStart(2, '0')}`;
  // THE SCOREBOARD, LIVE. Everything positional is credited at the buzzer and not before,
  // which is the rule and which left the header reading 0 to 0 for two and a half minutes
  // while the CELLs filled up. The projection is the same arithmetic on the same census, so
  // the number never jumps at the buzzer -- it just stops being a projection.
  const finished = s.period === 'FINISHED';
  const proj = finished ? { red: s.score.red.total, blue: s.score.blue.total } : world.projectedScore();
  $('#red-total').textContent = String(proj.red);
  $('#blue-total').textContent = String(proj.blue);
  const board = $<HTMLElement>('.scores');
  board.title = finished
    ? 'Final score.'
    : 'What each alliance would score if the buzzer went now. Balls in a CELL, a FLOWER or a GARDEN are credited at the END of the match, so this is a projection until the clock stops.';
  board.classList.toggle('projected', !finished);

  const rangeIn = Math.hypot(world.aimPoint()[0] - r.p[0], world.aimPoint()[2] - r.p[2]) * M_TO_IN;
  // Whichever autonomous holds the sticks says what it is doing; otherwise the brain does.
  const autoNote = auto && auto.phase !== 'done' ? auto.note
    : routine && routine.phase !== 'done' ? `${routine.phase.toUpperCase()}: ${routine.note}`
    : null;
  set('#st-note', autoNote ?? brain.state.note, brain.state.ready ? 'on' : 'off');
  // The camera's lock, in three words. `SEARCHING` means the turret is sweeping and nothing
  // else on this list is a measurement of anything yet.
  const tg = brain.target();
  set(
    '#st-tag',
    tg.scanning ? 'SEARCHING'
      // ODOMETRY IS NOT A STALE TAG, it is a different source, and an age is meaningless for
      // it -- printing one gave "CELL A - Infinity ms", which is the UI describing a number
      // it does not have. Say where the aim came from instead.
      : tg.fromOdometry ? `CELL ${tg.id === 2 ? 'B' : 'A'} · odometry`
      : `CELL ${tg.id === 2 ? 'B' : 'A'} · ${(tg.ageS * 1000).toFixed(0)} ms`,
    tg.fresh ? 'on' : 'off',
  );
  // TRUE range, and the HUD says so: the robot is working off `tg.rangeIn`, which is what
  // the camera told it, and the difference between the two is the error to watch.
  set('#st-range', m(rangeIn));
  set('#st-turret', `${r.turret.angleDeg.toFixed(0)}°`, r.turret.atLimit ? 'off' : '');
  set('#st-rpm', `${r.flywheel.rpm.toFixed(0)}`, brain.state.ready ? 'on' : '');
  set('#st-hopper', `${r.hopper.count}/${r.hopper.capacity}`, r.hopper.count ? '' : 'off');
  set('#st-batt', `${r.battery.volts.toFixed(1)} V`, r.battery.volts < 11.5 ? 'off' : '');
  // Is the aim MEASURED or dead-reckoned? That is the difference between a bearing with no
  // heading error in it and one carrying however far the IMU has drifted this match.
  const tag = brain.state.tagLocked;
  set('#st-tag', tag ? `locked, ${brain.state.tagPx.toFixed(0)} px` : 'no tag — odometry', tag ? 'on' : 'off');
  const oppRow = $<HTMLElement>('#st-opp-row');
  oppRow.style.display = bot ? '' : 'none';
  if (bot) set('#st-opp', bot.note, bot.phase === 'shoot' ? 'on' : '');
  // DRIVE SPEED GEAR. The bar is the fraction; the number is what that fraction is worth in
  // m/s, derived from the drive motor's free speed and the wheel radius rather than written
  // down, so changing either in config/robot.json moves the readout with it.
  const gearFrac = brain.state.speedScale;
  // The m/s CAP, when one is set, is the tighter of the two limits and the one the driver
  // needs to see: it is the number a refused shot is refused in.
  const capMps = robotSpec.drivetrain.maxSpeed_mps ?? 0;
  const gearMps = gearFrac * topSpeed_ms;
  const shown = capMps > 0 ? Math.min(gearMps, capMps) : gearMps;
  set('#st-speed', `${shown.toFixed(2)} m/s${brain.state.speedCapped ? ' CAP' : ''}`,
    brain.state.speedCapped ? 'off' : gearFrac < 1 ? 'off' : 'on');
  $<HTMLElement>('#st-speedbar').style.width = `${gearFrac * 100}%`;

  // THE SHOT-ZONE LEGEND, with live counts. Four colours went onto the field with nothing to
  // read them by, which is how "the CELL does not open this way" -- 474 of 841 squares, and
  // correct -- came to be reported as the map being broken. The counts are taken at the
  // CURRENT velocity, so the green number falls as you drive.
  const lg = $<HTMLElement>('#legend');
  lg.hidden = !scene.showShotZone;
  if (scene.showShotZone) {
    const t = scene.zoneTally();
    $('#lg-good').textContent = String(t.good);
    $('#lg-near').textContent = String(t.tooNear);
    $('#lg-far').textContent = String(t.tooFar);
    $('#lg-behind').textContent = String(t.behind);
    $('#lg-noroom').textContent = String(t.noRoom);
  }


  paintDeck();

  const tab = $('.tabs button.on').dataset.tab;
  if (tab === 'robot') paintRobot(s, rangeIn);
  else if (tab === 'predict') paintPredict(rangeIn);
  else if (tab === 'analysis') paintAnalysis(s);

  $('#perf').textContent = `${frameMs.toFixed(1)} ms`;
}

function set(sel: string, text: string, cls = ''): void {
  const el = $(sel);
  el.textContent = text;
  el.className = cls;
}

function paintRobot(s: Snapshot, rangeIn: number): void {
  const r = s.robot;
  $('#r-pose').innerHTML = [
    row('field x, y', `${(r.ftc.x * 0.0254).toFixed(2)}, ${(r.ftc.y * 0.0254).toFixed(2)} m`),
    row('heading', `${r.ftc.heading.toFixed(1)}°`),
    row('speed', mps(r.speed)),
    // TRUTH, and labelled as such. These two say where the CELL really is; the tag rows
    // below say what the robot believes, and the gap between them is the thing to watch.
    row('range to CELL (true)', m(rangeIn)),
    row('bearing to CELL (true)', `${world.sensors().game.truth.upCellAzimuthDeg.toFixed(1)}°`),
  ].join('');

  // WHAT THE ROBOT CAN SEE. Without this the panel shows a robot that always knows where the
  // goal is, which is exactly the impression the old oracle gave.
  const tag = brain.target();
  $('#r-pose').innerHTML += [
    // SEARCHING is a state the robot is MEANT to be in, not a fault, so it is amber and it
    // matches the HUD. Red is for "this cannot work": turret past its stop, flat battery.
    row('tag', tag.scanning ? 'SEARCHING' : `CELL ${tag.id === 2 ? 'B' : 'A'}`, tag.fresh ? 'good' : 'bad'),
    // WHERE THE AIM CAME FROM, which is the thing to read before trusting any row below it.
    // On odometry the bearing is as good as the pose and the ROCKER STATE is a guess until a
    // tag has been seen, so it says which of those two it is.
    row(
      'source',
      tag.scanning ? 'searching' : tag.fromOdometry ? (tag.stateFromTag ? 'odometry' : 'odometry, state assumed') : 'tag',
      tag.fromOdometry ? (tag.stateFromTag ? 'bad' : 'err') : tag.fresh ? 'good' : 'bad',
    ),
    // A dash is not a reading, so it does not get a status colour. Colouring the placeholder
    // put a red "no value" next to an amber "no value" and made a searching robot look broken.
    row('fix age', tag.scanning || tag.fromOdometry ? '—' : `${(tag.ageS * 1000).toFixed(0)} ms`, tag.scanning || tag.fromOdometry ? '' : tag.fresh ? 'good' : 'bad'),
    // WHERE IT THINKS THE HIVE IS, and whether that is its own measurement or the surveyed
    // prior it started from. This is the landmark the whole aim hangs off once the tag is out
    // of view, so "measured" vs "assumed" is the thing to know about it.
    row(
      'hive pivot',
      `${(tag.anchor.x * 0.0254).toFixed(2)}, ${(tag.anchor.y * 0.0254).toFixed(2)} m`,
      tag.anchorFromTag ? 'good' : 'bad',
    ),
    row('pivot from', tag.anchorFromTag ? 'measured' : 'surveyed prior', tag.anchorFromTag ? 'good' : 'bad'),
    row('range it believes', tag.scanning ? '—' : m(tag.rangeIn)),
    row('mouth angle', tag.scanning ? '—' : `${tag.openDeg.toFixed(0)}°`, tag.scanning || tag.openDeg <= 75 ? '' : 'err'),
  ].join('');

  $('#r-shoot').innerHTML = [
    row('aim mode', brain.state.autoAim ? 'automatic' : 'manual'),
    row('turret', `${r.turret.angleDeg.toFixed(1)}° → ${r.turret.targetDeg.toFixed(1)}°`, r.turret.atLimit ? 'err' : ''),
    row('turret rate', `${r.turret.omegaDps.toFixed(0)} °/s`),
    row('motion lead', `${brain.state.leadDeg.toFixed(1)}°`),
    row('hood', `${r.hood.angleDeg.toFixed(1)}°`),
    row('flywheel', `${r.flywheel.rpm.toFixed(0)} / ${brain.state.targetRpm.toFixed(0)} rpm`, brain.state.ready ? 'good' : 'bad'),
    row('exit speed', r.lastShot ? `${r.lastShot.v_exit.toFixed(2)} m/s` : '—'),
    row('feed ready in', `${Math.max(0, r.transfer.cycleTime - r.transfer.sinceFeed).toFixed(2)} s`),
    row('shots fired', String(r.flywheel.shots)),
    row('balls out of play', String(s.outOfPlay), s.outOfPlay ? 'bad' : ''),
  ].join('');

  const hive = s.hives.find((h) => h.alliance === alliance)!;
  const restoring = Math.abs(hive.gravityTorque_Nm);
  // Only the component OPPOSING gravity counts: after a tip, balls in the now-down CELL
  // push the rocker further onto its stop, which is not progress toward the next one.
  const pushing = restoring > 1e-6 ? -hive.ballTorque_Nm * Math.sign(hive.gravityTorque_Nm) : 0;
  const progress = restoring > 1e-6 ? Math.max(0, Math.min(1.5, pushing / restoring)) : 0;
  const fill = $('#tip-fill');
  fill.style.width = `${Math.min(100, (progress / 1.5) * 100)}%`;
  fill.className = progress >= 1 ? 'over' : progress > 0.75 ? 'near' : '';
  // MOVING IS NOT THE SAME AS GOING OVER. `hive.tipping` is any rotation above 0.2 rad/s,
  // which is the right test for the fire gate and the wrong one for this label: balls landing
  // in the pocket rock the rocker on its stop, so the panel announced "Tipping now" at 49% of
  // the torque needed within a second of the page loading. It is a tip only when the balls
  // are actually beating gravity.
  $('#tip-label').textContent = hive.tipping && progress >= 0.9
    ? 'Tipping now.'
    : hive.tipping
      ? `Rocking on its stop, ${(progress * 100).toFixed(0)}% of the way to going over.`
      : progress <= 0
        ? 'Resting on its stop. The mark is where the balls beat gravity.'
        : `${(progress * 100).toFixed(0)}% of the torque needed to go over.`;

  // BOTH CELLS PUSH. `ballTorque` is the net over every ball on the rocker, and after a tip
  // the ones dumped into the now-down CELL are still sitting there holding it down. Showing
  // that net on a line under "elements in up CELL" reads as a contradiction -- four balls
  // and a torque pushing the wrong way -- so the two are split out. The down-CELL load is
  // real and it is why a tipped HIVE is harder to tip back.
  const upIds = new Set(hive.perBallTorque.filter((b) => hive.upCell === 'A' ? b.lever_in > 0 : b.lever_in < 0).map((b) => b.id));
  const sgn = restoring > 1e-6 ? Math.sign(hive.gravityTorque_Nm) : 1;
  const upPush = -hive.perBallTorque.filter((b) => upIds.has(b.id)).reduce((a, b) => a + b.torque_Nm, 0) * sgn;
  const downHold = -hive.perBallTorque.filter((b) => !upIds.has(b.id)).reduce((a, b) => a + b.torque_Nm, 0) * sgn;

  $('#r-hive').innerHTML = [
    row('elements in up CELL', String(hive.ballsInUpCell)),
    row('up CELL pushes over', `${upPush.toFixed(3)} N·m`, upPush > restoring ? 'good' : ''),
    row('down CELL holds it', `${(-downHold).toFixed(3)} N·m`, downHold < -1e-6 ? 'bad' : ''),
    row('net from balls', `${pushing.toFixed(3)} N·m`, pushing > restoring ? 'good' : ''),
    row('gravity holds', `${restoring.toFixed(3)} N·m`),
    row('angle', `${hive.angleDeg.toFixed(2)}°`),
    row('up CELL faces', hive.upCell),
    row('tips this match', String(hive.tips), hive.tips > 0 ? 'good' : ''),
  ].join('');

  $('#r-wheels').innerHTML = r.wheels
    .map((w) => row(w.name.toUpperCase(), `${w.cmd.toFixed(2)} cmd  ${w.force_N.toFixed(0)} N  slip ${w.slip.toFixed(2)}`, Math.abs(w.slip) > 0.25 ? 'bad' : ''))
    .join('');

  $('#r-telemetry').innerHTML = s.telemetry.length
    ? s.telemetry.map(([k, v]) => row(k, v)).join('')
    : row('—', 'no telemetry');
}

// ---------------------------------------------------------------- predictor

let prediction: Prediction | null = null;
let predictedAt = 0;
/** The solution being analysed, kept so the panel can show what it is reporting on. */
let predShot = { hood: 0, rpm: 0, speed: 0 };

/**
 * The sensitivity table. It costs about 20 trajectory integrations, so it is recomputed at
 * most twice a second and only while its own tab is open.
 */
function paintPredict(rangeIn: number): void {
  const now = performance.now();
  if (!prediction || now - predictedAt > 600) {
    predictedAt = now;
    // Analyse the shot the robot WOULD take from here -- the shot table's own solution for
    // this range -- not whatever the idle hardware happens to be sitting at. Otherwise the
    // panel reports on a 0 rpm shot that never leaves, which tells you nothing.
    const mz = world.robot.muzzle();
    const table = shotTable.lookup(rangeIn);
    const hood = robotSpec.hood.enabled
      ? robotSpec.hood.angleRange_deg[0] + table.hoodPos * (robotSpec.hood.angleRange_deg[1] - robotSpec.hood.angleRange_deg[0])
      : robotSpec.hood.fixedAngle_deg;
    const speed = rpmToSpeed(table.rpm, robotSpec.flywheel.k, robotSpec.flywheel.r_fly_m);
    predShot = { hood, rpm: table.rpm, speed };
    const radius = params.ball.pollen.d_m / 2;
    const mouth = world.hives[alliance].upCellMouthWorld();
    prediction = predict(
      params,
      {
        from: mz.pos, azimuth: mz.azimuth, elevation: hood * DEG, speed, radius,
        mass: params.ball.pollen.m_kg,
        spin: robotSpec.flywheel.type === 'single' ? speed / radius : 0,
      },
      mouth[1],
      rangeIn,
      knobs({
        tolRpm: robotSpec.flywheel.tolRpm,
        hoodSteps: 200,
        scatterDeg: robotSpec.flywheel.scatter.angle_deg,
        scatterSpeedFrac: robotSpec.flywheel.scatter.speedFrac,
        k: robotSpec.flywheel.k,
        rFly: robotSpec.flywheel.r_fly_m,
      }),
    );
  }
  const p = prediction;
  const ok = Number.isFinite(p.reach_in);

  $('#p-summary').innerHTML = [
    row('range to the CELL', m(p.target_in)),
    row('shot table solution', `${predShot.hood.toFixed(0)}° at ${predShot.rpm.toFixed(0)} rpm`),
    row('exit speed', `${predShot.speed.toFixed(2)} m/s`),
    row('this shot lands at', ok ? m(p.reach_in) : 'out of range'),
    row('miss', ok ? `${cmSigned(p.bias_in)} ${p.bias_in > 0 ? 'long' : 'short'}` : '—',
      ok ? (Math.abs(p.bias_in) < 6 ? 'good' : 'bad') : 'err'),
    row('predicted group, 1σ', `± ${cm(p.spread_in)}`, p.spread_in < 12 ? 'good' : 'bad'),
  ].join('');

  $('#p-rows').innerHTML = p.rows.map((k, i) => {
    const per = Number.isFinite(k.perStep_in) ? `${cmSigned(k.perStep_in)}/${k.step}${k.unit}` : '—';
    return `<div class="brow ${i === 0 ? 'lead' : ''}" data-key="${k.key}">
      <span class="n" title="${esc(k.why)}">${k.label} &nbsp;<span style="opacity:.6">${per}</span></span>
      <span class="t"><i style="width:${(k.share * 100).toFixed(0)}%"></i></span>
      <span class="v">${cm(k.contrib_in)}</span>
    </div>`;
  }).join('');

  const top = p.rows[0];
  $('#p-why').innerHTML = top
    ? `<b>${esc(top.label)}</b> owns ${(top.share * 100).toFixed(0)}% of the predicted spread. ${esc(top.why)}`
    : '';
}

const esc = (s: string) => s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);

// ---------------------------------------------------------------- analysis

let chartedAt = -1;

/**
 * THE MATCH, in the terms it is scored in, plus what the shooter actually did.
 *
 * Live rather than only at the buzzer, because "how am I doing" is a question you ask while
 * driving; the header says FINAL once the clock has stopped, so a number you are reading is
 * never ambiguous about whether it is still moving.
 *
 * Accuracy is landed over settled, from the shot log, and a shot counts as landed only when
 * it ends up in our OWN up CELL -- not when it passes through the volume, and not when it
 * lands in the down CELL or the opponent's hive, both of which the old census counted.
 *
 * The odds line is the other half. Accuracy is what happened; the odds are what the robot
 * believes about the shot it is lining up now, through the measured curve in
 * config/landcal.json, against the ceiling that curve actually reached. A gate set above
 * that ceiling cannot be met by any shot this shooter can take, and the honest thing is to
 * say so rather than leave the driver wondering why it will not fire.
 */
function paintMatch(s: Snapshot): void {
  const sc = s.score[alliance];
  const settled = s.shots.filter((x) => x.result !== 'flight');
  const landed = settled.filter((x) => x.result === 'cell').length;
  const acc = settled.length ? landed / settled.length : 0;
  const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
  const sd = (a: number[]) => {
    if (a.length < 2) return NaN;
    const mu = mean(a);
    return Math.sqrt(a.reduce((x, v) => x + (v - mu) ** 2, 0) / (a.length - 1));
  };
  const lng = settled.map((x) => x.long_in).filter(Number.isFinite);
  const lat = settled.map((x) => x.lat_in).filter(Number.isFinite);
  const done = s.period === 'FINISHED';

  $('#a-match-h').textContent = done ? 'Match — FINAL' : `Match — ${s.period}`;
  $('#a-match').innerHTML = [
    row('score', String(sc.total), done ? 'good' : ''),
    row('LEAVE / PARK', `${sc.leave ? 3 : 0} + ${sc.park ? 5 : 0}`, sc.leave || sc.park ? 'good' : ''),
    row('TIPS', `${sc.tips} × 20`, sc.tips ? 'good' : ''),
    row('in our up CELL', String(world.landedInUpCell(alliance))),
    row('shots fired', String(s.shots.length)),
    row('landed', `${landed} of ${settled.length}`),
    row('accuracy', settled.length ? `${(acc * 100).toFixed(0)}%` : '—',
      !settled.length ? '' : acc > 0.6 ? 'good' : acc > 0.3 ? 'bad' : 'err'),
    row('group, downrange', lng.length > 1 ? `${cmSigned(mean(lng))} ± ${cm(sd(lng))}` : '—',
      lng.length > 1 ? (Math.abs(mean(lng)) < 6 ? 'good' : 'bad') : ''),
    row('group, lateral', lat.length > 1 ? `${cmSigned(mean(lat))} ± ${cm(sd(lat))}` : '—',
      lat.length > 1 ? (Math.abs(mean(lat)) < 5 ? 'good' : 'bad') : ''),
  ].join('');

  const p = brain.state.pLand;
  const ceiling = landCalCeiling;
  const gate = robotSpec.flywheel.minLandProb ?? 0;
  $('#a-odds').textContent = p < 0
    ? 'Odds: this shot table predates the columns the land model needs, so the gate falls through to the rpm window.'
    : `Odds on the shot now: ${(p * 100).toFixed(0)}%`
      + (brain.state.calibrated ? ' (calibrated)' : ' (raw model)')
      + `, gate at ${(gate * 100).toFixed(0)}%`
      + (ceiling !== undefined ? `, best measured ${(ceiling * 100).toFixed(0)}%` : '')
      + (ceiling !== undefined && gate > ceiling ? ' — THE GATE IS ABOVE THE CEILING: no shot can meet it.' : '');
}

/**
 * THE SCOREBOARD, itemised and live, both alliances side by side.
 *
 * A total tells you how you are doing and nothing about what to do next. The breakdown says
 * which avenue is empty -- and it is how the opponent bot was caught scoring exactly 0 from
 * FLOWERs and, before that, exactly 0 from LEAVE because nobody was assessing it.
 */
function paintBoard(s: Snapshot): void {
  const done = s.period === 'FINISHED';
  const parts = done
    ? { red: { ...s.score.red, bottomNectar: s.score.red.bottomNectar * 5, tips: s.score.red.tips * 20, leave: s.score.red.leave ? 3 : 0, park: s.score.red.park ? 5 : 0, upCell: s.score.red.upCell * 2, flower: s.score.red.flower * 2, garden: s.score.red.garden, total: s.score.red.total },
        blue: { ...s.score.blue, bottomNectar: s.score.blue.bottomNectar * 5, tips: s.score.blue.tips * 20, leave: s.score.blue.leave ? 3 : 0, park: s.score.blue.park ? 5 : 0, upCell: s.score.blue.upCell * 2, flower: s.score.blue.flower * 2, garden: s.score.blue.garden, total: s.score.blue.total } }
    : world.projectedParts();
  const rows: [string, keyof typeof parts.red][] = [
    ['TIPS', 'tips'], ['LEAVE', 'leave'], ['PARK', 'park'],
    ['balls in the up CELL', 'upCell'], ['FLOWERs owned', 'flower'],
    ['bottom NECTAR', 'bottomNectar'], ['GARDEN', 'garden'],
  ];
  const cell = (v: number) => `<td class="${v ? '' : 'zero'}">${v}</td>`;
  $('#a-board').innerHTML =
    `<tr><th>${done ? 'final' : 'projected'}</th><th class="red">RED</th><th class="blue">BLUE</th></tr>`
    + rows.map(([label, k]) => `<tr><td>${label}</td>${cell(Number(parts.red[k]))}${cell(Number(parts.blue[k]))}</tr>`).join('')
    + `<tr class="sum ${alliance === 'red' ? 'mine' : ''}"><td>total</td><td>${parts.red.total}</td><td>${parts.blue.total}</td></tr>`;
}

function paintAnalysis(s: Snapshot): void {
  paintBoard(s);
  paintMatch(s);
  const running = !!auto && auto.phase !== 'done';
  $('#a-plan').innerHTML = [
    row('shots to collect', String(plan.shots)),
    row('range sweep', `${m(plan.range_in[0])}–${m(plan.range_in[1])}`),
    row('bearing spread', `± ${plan.bearing_deg}°`),
    row('fire while moving', plan.onTheMove ? 'yes' : 'no'),
    row('progress', running ? `${auto!.shotsAsked} / ${plan.shots} — ${auto!.phase}` : s.shots.length ? 'finished' : 'not started'),
  ].join('');
  $<HTMLButtonElement>('#a-start').textContent = running ? 'Stop collection' : 'Start collection';
  $('#a-trace-note').textContent = `Flight recorder: ${trace.length} samples at ${trace.hz} Hz, ${(trace.length / trace.hz).toFixed(0)} s of run. It keeps recording across a reset so you can compare before and after a change.`;

  const rep: Report = analyse(s.shots, robotSpec.flywheel.tolRpm);
  $('#a-rows').innerHTML = [
    row('shots landed', `${rep.n}`),
    row('into the CELL', `${rep.landed} (${(rep.landRate * 100).toFixed(0)}%)`, rep.landRate > 0.6 ? 'good' : rep.landRate > 0.3 ? 'bad' : 'err'),
    row('downrange bias', cmSigned(rep.bias_in), Math.abs(rep.bias_in) < 6 ? 'good' : 'bad'),
    row('downrange spread 1σ', `± ${cm(rep.sd_in)}`, rep.sd_in < 18 ? 'good' : 'bad'),
    row('lateral bias', cmSigned(rep.latBias_in), Math.abs(rep.latBias_in) < 5 ? 'good' : 'bad'),
    row('lateral spread 1σ', `± ${cm(rep.latSd_in)}`, rep.latSd_in < 12 ? 'good' : 'bad'),
    row('rpm error at fire', `${rep.rpmErr_rpm.toFixed(0)} rpm`, rep.rpmErr_rpm < robotSpec.flywheel.tolRpm ? 'good' : 'bad'),
    row('fired off speed', String(rep.firedOffSpeed), rep.firedOffSpeed ? 'bad' : 'good'),
  ].join('');
  $('#a-verdict').textContent = rep.verdict;

  const cal = robotSpec.calibration;
  $('#a-cal').innerHTML = [
    row('range trim now', cmSigned(cal.rangeTrim_in), cal.rangeTrim_in ? 'good' : ''),
    row('turret trim now', `${cal.turretTrim_deg >= 0 ? '+' : ''}${cal.turretTrim_deg.toFixed(2)}°`, cal.turretTrim_deg ? 'good' : ''),
    row('this run suggests', rep.fix.worthIt
      ? `${cmSigned(rep.fix.rangeTrim_in)}, ${rep.fix.turretTrim_deg >= 0 ? '+' : ''}${rep.fix.turretTrim_deg.toFixed(2)}°`
      : 'no change', rep.fix.worthIt ? 'bad' : 'good'),
  ].join('');
  $('#a-fixwhy').textContent = rep.fix.why;
  $<HTMLButtonElement>('#a-apply').disabled = !rep.fix.worthIt;
  $('#a-faults').innerHTML = rep.faults.length
    ? rep.faults.map((f) => `<li>${esc(f)}</li>`).join('')
    : '<li class="cap">Nothing stood out.</li>';

  const worst = Math.max(1, ...rep.byRange.map((b) => b.meanMiss_in));
  $('#a-bins').innerHTML = rep.byRange.length
    ? rep.byRange.map((b) => `<div class="brow">
        <span class="n">${m(b.range_in)} &nbsp;<span style="opacity:.6">${b.landed}/${b.n} in</span></span>
        <span class="t"><i style="width:${((b.meanMiss_in / worst) * 100).toFixed(0)}%"></i></span>
        <span class="v">${b.meanMiss_in.toFixed(0)}"</span></div>`).join('')
    : '<p class="cap">No finished shots yet.</p>';

  // Charts are only redrawn when a shot actually finishes: redrawing three canvases at
  // 60 Hz to show the same dots is pure heat.
  const finished = s.shots.filter((x) => x.result !== 'flight').length;
  if (finished !== chartedAt) {
    chartedAt = finished;
    const mouthR = world.hives[alliance].upCell.halfInterior[2] * M_TO_IN;
    groupPlot($<HTMLCanvasElement>('#a-scatter'), s.shots, mouthR);
    errorVsRange($<HTMLCanvasElement>('#a-vsrange'), s.shots);
    histogram($<HTMLCanvasElement>('#a-hist'), s.shots);
  }

  $('#a-log').innerHTML = s.shots.slice(-12).reverse().map((x) =>
    row(`#${x.n} ${m(x.rangeIn, 1)} ${x.rpm.toFixed(0)}rpm`,
      x.result === 'flight' ? 'in flight' : `${x.result === 'cell' ? 'CELL' : 'miss'} ${cmSigned(x.long_in, 0)}/${cmSigned(x.lat_in, 0)}`,
      x.result === 'cell' ? 'good' : x.result === 'miss' ? 'bad' : ''),
  ).join('') || row('—', 'no shots yet');
}

// ---------------------------------------------------------------- modes and deck

/**
 * The action deck, per mode. Every entry runs real code and reports its own state; there is
 * nothing here that looks like a control and does nothing.
 */
interface Action { label: string; title: string; run: () => void; on?: () => boolean }

const DECK: Record<Mode, Action[]> = {
  practice: [
    { label: 'Auto-aim (L1)', title: 'Turret and hood solve for the CELL continuously, including a lead for the robot’s own motion. Off, the , and . keys aim it by hand. L1 on the pad, T on the keyboard — M1 and M2 are the manual nudge, not this.', run: () => (brain.state.autoAim = !brain.state.autoAim), on: () => brain.state.autoAim },
    { label: 'Auto-fire (R1)', title: 'Latch. Spins the flywheel, waits for it to be in tolerance and the turret to be on target, then feeds at the cycle time until you press it again. R1 or the M1 paddle on the pad, space on the keyboard. L3 (G) fires by hand instead, for as long as you hold it.', run: () => (brain.state.firing = !brain.state.firing), on: () => brain.state.firing },
    { label: 'FLOWER lob', title: 'Aim at the nearest FLOWER and lob into the top of its tube instead of shooting the CELL. A different table: hood 57-80 deg at about 1200 rpm, a third of the CELL shot. The tube is a 4.0 in hole for a 2.8 in ball, so stand 12-16 in off it -- closer and the bumper is against the column, further and the lob runs out of hood.', run: () => (brain.state.flowerMode = !brain.state.flowerMode), on: () => brain.state.flowerMode },
    { label: 'Opponent', title: 'Put a real robot on the other alliance and play against it. It collects, lines up on its own CELL’s opening, fires through the same readiness gate you do, tips its own HIVE and PARKs at the buzzer — driving a real chassis through a real brain, so nothing it does is something you could not. Toggling it rebuilds the match.', run: () => { opponentOn = !opponentOn; build(); }, on: () => opponentOn },
    { label: 'Empty hopper', title: 'Practice aid: set down everything the robot is carrying, on the tiles behind it so the intake does not swallow it again on the next frame. The balls stay IN PLAY rather than being deleted — 40 POLLEN is a fixed budget and the HIVE only tips when enough of them are in a CELL, so a button that ate four would change the game and not just the robot.', run: () => emptyHopper() },
    { label: 'Auto-fill hopper', title: 'Practice aid, not a game rule: quietly picks up the nearest POLLEN off the floor whenever the hopper has room, so you can work on aiming without driving a collection lap.', run: () => setAutoLoad(!autoLoad), on: () => autoLoad },
    { label: 'Robot: box', title: 'Which robot YOU are driving, on screen. Cycles through every assets/robot-*.glb that tools/cad2robot.py has converted from a team’s STEP, and back to the procedural box this simulator was built around. It is a SKIN: the physics is the one chassis config/robot.json describes, whatever is drawn over it. The wheels, intake roller, flywheel and hood of a converted robot are separate meshes and turn on the same numbers the box’s do; a robot whose CAD has no turret gets no turret, because none of these have one.', run: () => cycleSkin('player') },
    { label: 'Rival: box', title: 'The same, for the opponent. Wearing a different CAD from yours is the point: two identical grey robots on one field cannot be told apart at a glance while driving.', run: () => cycleSkin('opponent') },
    { label: 'Joystick', title: 'On-screen sticks: left translates, right looks around. They feed the same gamepad frame the keyboard and a real controller do, so a phone or a trackpad can drive without either.', run: () => (sticks.visible = !sticks.visible), on: () => sticks.visible },
    { label: 'Shot zone', title: 'Green where a perfectly aimed shot clears the land-probability gate, red where it does not, using the hood and rpm the table commands at that range and the CELL mouth as seen from that spot. A MODEL map (tools/shotzone.ts), not a record of what this robot has hit.', run: () => (scene.showShotZone = !scene.showShotZone), on: () => scene.showShotZone },
    { label: 'Patrol zone', title: 'The wedge either side of the up CELL’s opening that the driver should stay inside — robot.json shot.patrolHalfAngle_deg, adjustable in Variables. It is NOT the fire gate (turret.fireOpenCap_deg, 60 deg, wider): this is where a shot is worth TAKING rather than where one is allowed. With every gate lifted, 260 shots land 94% at 0-20 deg off the opening, 100% at 20-35, 89% at 35-50 and 8% at 50-65 — the far edge is where the mouth is half shut AND where the robot is reversing, because the extreme bearing is the end of a pass.', run: () => (scene.showPatrolSector = !scene.showPatrolSector), on: () => scene.showPatrolSector },
    { label: 'Belief', title: 'A VIOLET ring where the robot THINKS the up CELL mouth is, beside the orange one at where it actually is. Built from the robot’s own estimate only: the fused pose — odometry with tag corrections — plus the target range and bearing it is aiming with. The distance between the two rings is the localisation error, to scale, instead of two numbers in a panel to subtract.', run: () => (scene.showBelief = !scene.showBelief), on: () => scene.showBelief },
    { label: 'Sight line', title: 'The ray the tag pipeline is trying to decode along, from the AprilTag panel on the up CELL to the camera. GREEN while it is decoding, RED while the geometry refuses -- out of range, too far round the side, or the rocker mid-swing. It is drawn from the robot’s tracked point at muzzle height because that is where the model puts the camera: there is no mount offset yet.', run: () => (scene.showSightLine = !scene.showSightLine), on: () => scene.showSightLine },
    { label: 'Pause', title: 'Freeze the physics. The view still moves.', run: () => togglePause(), on: () => paused },
    { label: 'Reset', title: 'Rebuild the match: robot back on its start tile, balls re-staged, score and shot log cleared.', run: () => build() },
  ],
  auto: [
    { label: 'Shot zone', title: 'Green where a perfectly aimed shot clears the land-probability gate. Worth leaving on here: it shows you the sector the routine is driving to, and why it cannot shoot from the start tile.', run: () => (scene.showShotZone = !scene.showShotZone), on: () => scene.showShotZone },
    { label: 'Speed: 1x', title: 'Sim seconds per real second. The step size never changes, so a 4x run flies exactly the same trajectories.', run: () => cycleTurbo(), on: () => turbo > 1 },
    { label: 'Pause', title: 'Freeze the physics mid-routine. Resuming continues from where it stopped.', run: () => togglePause(), on: () => paused },
    { label: 'Reset', title: 'Rebuild the match and clear the routine.', run: () => build() },
  ],
  collect: [
    { label: 'Shots: 20', title: 'How many samples this run collects. Below about 20 the statistics are noise.', run: () => { plan.shots = plan.shots >= 80 ? 20 : plan.shots + 20; }, },
    { label: 'Moving', title: 'Fire while still rolling instead of settling first. This is what the motion lead exists for, and the difference between the two runs tells you whether it works.', run: () => (plan.onTheMove = !plan.onTheMove), on: () => plan.onTheMove },
    { label: 'Empty hopper', title: 'Practice aid: set down everything the robot is carrying, on the tiles behind it so the intake does not swallow it again on the next frame. The balls stay IN PLAY rather than being deleted — 40 POLLEN is a fixed budget and the HIVE only tips when enough of them are in a CELL, so a button that ate four would change the game and not just the robot.', run: () => emptyHopper() },
    { label: 'Auto-fill hopper', title: 'Required for a long run: the robot only carries a few balls and the sweep needs more.', run: () => setAutoLoad(!autoLoad), on: () => autoLoad },
    { label: 'Speed: 1x', title: 'Sim seconds per real second. The physics step never changes, so a 16x run gives exactly the same trajectories as a 1x one — it just does not make you watch.', run: () => cycleTurbo(), on: () => turbo > 1 },
    { label: 'Pause', title: 'Freeze the physics mid-run. Resuming continues the same plan.', run: () => togglePause(), on: () => paused },
    { label: 'Reset', title: 'Rebuild the match and clear the shot log.', run: () => build() },
  ],
  test: [
    { label: 'Auto-aim (L1)', title: 'Turret and hood solve for the CELL continuously. Off, the , and . keys aim it by hand.', run: () => (brain.state.autoAim = !brain.state.autoAim), on: () => brain.state.autoAim },
    { label: 'Auto-fire (R1)', title: 'Latch. Spins up and feeds at the cycle time until pressed again. L3 fires by hand.', run: () => (brain.state.firing = !brain.state.firing), on: () => brain.state.firing },
    { label: 'Empty hopper', title: 'Practice aid: set down everything the robot is carrying, on the tiles behind it so the intake does not swallow it again on the next frame. The balls stay IN PLAY rather than being deleted — 40 POLLEN is a fixed budget and the HIVE only tips when enough of them are in a CELL, so a button that ate four would change the game and not just the robot.', run: () => emptyHopper() },
    { label: 'Auto-fill hopper', title: 'Keeps the hopper topped up from the floor so a test run does not stop for ammunition.', run: () => setAutoLoad(!autoLoad), on: () => autoLoad },
    { label: 'Drop a POLLEN in the CELL', title: 'Places one POLLEN into your up CELL by hand. The quickest way to watch the HIVE tip: it takes 12.', run: () => dropBall() },
    { label: 'Shot arc', title: 'Two curves. YELLOW is the prediction: what the solver says the shot the aim is lining up will do, drawn with the same integrator the shot table is built from. It is hidden while the camera is searching, because then there is no shot being lined up. BLUE is the trail the last ball actually flew. When they lie on top of each other the model is right; where they part company is the thing worth chasing.', run: () => (scene.showTrajectory = !scene.showTrajectory), on: () => scene.showTrajectory },
    { label: 'Patrol zone', title: 'The wedge either side of the up CELL’s opening that the driver should stay inside — robot.json shot.patrolHalfAngle_deg, adjustable in Variables. It is NOT the fire gate (turret.fireOpenCap_deg, 60 deg, wider): this is where a shot is worth TAKING rather than where one is allowed. With every gate lifted, 260 shots land 94% at 0-20 deg off the opening, 100% at 20-35, 89% at 35-50 and 8% at 50-65 — the far edge is where the mouth is half shut AND where the robot is reversing, because the extreme bearing is the end of a pass.', run: () => (scene.showPatrolSector = !scene.showPatrolSector), on: () => scene.showPatrolSector },
    { label: 'Belief', title: 'A VIOLET ring where the robot THINKS the up CELL mouth is, beside the orange one at where it actually is. Built from the robot’s own estimate only: the fused pose — odometry with tag corrections — plus the target range and bearing it is aiming with. The distance between the two rings is the localisation error, to scale, instead of two numbers in a panel to subtract.', run: () => (scene.showBelief = !scene.showBelief), on: () => scene.showBelief },
    { label: 'Sight line', title: 'The ray the tag pipeline is trying to decode along, from the AprilTag panel on the up CELL to the camera. GREEN while it is decoding, RED while the geometry refuses -- out of range, too far round the side, or the rocker mid-swing. It is drawn from the robot’s tracked point at muzzle height because that is where the model puts the camera: there is no mount offset yet.', run: () => (scene.showSightLine = !scene.showSightLine), on: () => scene.showSightLine },
    { label: 'Colliders', title: 'Show the convex shapes the solver actually collides with, instead of the CAD skin drawn over them.', run: () => (scene.showColliders = !scene.showColliders), on: () => scene.showColliders },
    { label: 'Robot: box', title: 'Which robot YOU are driving, on screen. Cycles through every assets/robot-*.glb that tools/cad2robot.py has converted from a team’s STEP, and back to the procedural box this simulator was built around. It is a SKIN: the physics is the one chassis config/robot.json describes, whatever is drawn over it. The wheels, intake roller, flywheel and hood of a converted robot are separate meshes and turn on the same numbers the box’s do; a robot whose CAD has no turret gets no turret, because none of these have one.', run: () => cycleSkin('player') },
    { label: 'Rival: box', title: 'The same, for the opponent. Wearing a different CAD from yours is the point: two identical grey robots on one field cannot be told apart at a glance while driving.', run: () => cycleSkin('opponent') },
    { label: 'Joystick', title: 'On-screen sticks: left translates, right looks around. They feed the same gamepad frame the keyboard and a real controller do, so a phone or a trackpad can drive without either.', run: () => (sticks.visible = !sticks.visible), on: () => sticks.visible },
    { label: 'Shot zone', title: 'Green where a perfectly aimed shot clears the land-probability gate, red where it does not, using the hood and rpm the table commands at that range and the CELL mouth as seen from that spot. A MODEL map (tools/shotzone.ts), not a record of what this robot has hit.', run: () => (scene.showShotZone = !scene.showShotZone), on: () => scene.showShotZone },
    { label: 'Patrol zone', title: 'The wedge either side of the up CELL’s opening that the driver should stay inside — robot.json shot.patrolHalfAngle_deg, adjustable in Variables. It is NOT the fire gate (turret.fireOpenCap_deg, 60 deg, wider): this is where a shot is worth TAKING rather than where one is allowed. With every gate lifted, 260 shots land 94% at 0-20 deg off the opening, 100% at 20-35, 89% at 35-50 and 8% at 50-65 — the far edge is where the mouth is half shut AND where the robot is reversing, because the extreme bearing is the end of a pass.', run: () => (scene.showPatrolSector = !scene.showPatrolSector), on: () => scene.showPatrolSector },
    { label: 'Speed: 1x', title: 'Sim seconds per real second. The physics step never changes, so the trajectories are identical — it just runs more of them per frame.', run: () => cycleTurbo(), on: () => turbo > 1 },
    { label: 'Pause', title: 'Freeze the physics.', run: () => togglePause(), on: () => paused },
    { label: 'Reset', title: 'Rebuild the match with the current variables.', run: () => build() },
  ],
};

/**
 * WEAR A REAL TEAM'S CAD. tools/cad2robot.py converts a robot STEP into assets/robot-<name>.glb
 * with its wheels, intake roller, flywheel and hood as separate named meshes; the scene rigs
 * those to the same handles the procedural robot uses, so a borrowed robot's parts turn on the
 * same numbers yours do. 'box' is the procedural robot this simulator was built around.
 *
 * It CYCLES rather than opening a menu because there are a handful of skins, not a hundred,
 * and a button you can hit while driving beats a dialog you cannot.
 */
function cycleSkin(which: 'player' | 'opponent'): void {
  const list = Scene.skins();
  const cur = which === 'player' ? scene.playerSkin : scene.opponentSkin;
  const next = list[(list.indexOf(cur) + 1) % list.length];
  void scene.setSkin(which, next).then(paintDeck);
}

function buildDeck(): void {
  const host = $('#deck');
  host.innerHTML = '';
  for (const [i, a] of DECK[mode].entries()) {
    const b = document.createElement('button');
    b.textContent = a.label;
    b.title = a.title;
    b.dataset.i = String(i);
    b.onclick = () => { a.run(); paintDeck(); };
    host.appendChild(b);
  }
}

function paintDeck(): void {
  const list = DECK[mode];
  for (const b of $$<HTMLButtonElement>('#deck button')) {
    const a = list[Number(b.dataset.i)];
    if (!a) continue;
    if (a.on) b.classList.toggle('on', a.on());
    if (a.label.startsWith('Shots:')) b.textContent = `Shots: ${plan.shots}`;
    if (a.label.startsWith('Speed:')) b.textContent = `Speed: ${turbo}x`;
    if (a.label.startsWith('Robot:')) b.textContent = `Robot: ${scene.playerSkin}`;
    if (a.label.startsWith('Rival:')) b.textContent = `Rival: ${scene.opponentSkin}`;
  }
  const running = !!auto && auto.phase !== 'done';
  const autoRunning = !!routine && routine.phase !== 'done';
  const p = $<HTMLButtonElement>('#primary');
  p.textContent = mode === 'collect'
    ? running ? 'Stop collection' : 'Run collection'
    : mode === 'auto'
      ? autoRunning ? 'Stop autonomous' : 'Run autonomous'
      : world.clock.period === 'STAGING' ? 'Start match' : paused ? 'Resume' : 'Pause';
  p.title = mode === 'collect'
    ? 'Start the autonomous sweep described in the Analysis tab.'
    : mode === 'auto'
      ? 'Restart the match and hand the sticks to the 30-second autonomous routine.'
      : 'Start the 2:38 match clock, or pause it once it is running.';
}

function setMode(m: Mode): void {
  mode = m;
  for (const b of $$<HTMLButtonElement>('#mode button')) b.classList.toggle('on', b.dataset.mode === m);
  buildDeck();
  // Each mode opens on the tab it exists for.
  showTab(m === 'collect' ? 'analysis' : m === 'test' ? 'predict' : 'robot');
  if (m !== 'collect' && auto) auto = null;
  if (m !== 'auto' && routine) routine = null;
  if (m !== 'collect') turbo = 1;   // never fast-forward while a human is driving
  if (m === 'auto') {
    autoLoad = false;               // the whole question is what it does with ONE preload
    scene.cameraMode = 'follow';
    $<HTMLSelectElement>('#camera').value = 'follow';
  }
  if (m === 'collect') {
    autoLoad = true;                 // a 40-shot sweep needs more balls than the robot holds
    scene.cameraMode = 'follow';
    $<HTMLSelectElement>('#camera').value = 'follow';
  }
  paintDeck();
}

const TURBO = [1, 4, 16];
const cycleTurbo = () => { turbo = TURBO[(TURBO.indexOf(turbo) + 1) % TURBO.length]; };

const MODES: Mode[] = ['practice', 'auto', 'collect', 'test'];
const cycleMode = () => setMode(MODES[(MODES.indexOf(mode) + 1) % MODES.length]);

const CAMERAS: CameraMode[] = ['orbit', 'follow', 'fpv', 'top', 'muzzle'];
function cycleCamera(d: number): void {
  const i = (CAMERAS.indexOf(scene.cameraMode) + d + CAMERAS.length) % CAMERAS.length;
  scene.cameraMode = CAMERAS[i];
  $<HTMLSelectElement>('#camera').value = CAMERAS[i];
}

/**
 * Build the match autonomous from the world as it stands: where the mouth is, which way it
 * opens, where our LOADING zone is, and what range band the shot table actually solves. All
 * four are read out of the live world, so none of them is a number written into the routine.
 */
function makeRoutine(): AutoRoutine {
  const hive = world.hives[alliance];
  const zone = world.geom.zones.find((z) => z.name === 'LOADING' && z.alliance === alliance)!;
  const ranges = shotTable.rows.map((r) => r.range_in);
  return new AutoRoutine({
    mouth: hive.upCellMouthWorld(),
    mouthNormal: hive.upCellMouthNormalWorld(),
    loading: [zone.min[0], zone.min[2], zone.max[0], zone.max[2]],
    halfWidth_m: world.geom.halfWidth_m,
    band_in: [Math.min(...ranges), Math.max(...ranges)],
    flowers: flowerTbl?.flowers,
  });
}

/** The green button: whatever the current mode's main verb is. */
function primary(): void {
  if (mode === 'auto') {
    if (routine && routine.phase !== 'done') { routine = null; return paintDeck(); }
    // Always from the top of AUTO: the routine is written against a 30 s period and a full
    // preload, and running it from the middle of TELEOP would be measuring something else.
    build();
    routine = makeRoutine();
    world.clock.start();
    paused = false;
    paintDeck();
    return;
  }
  if (mode === 'collect') {
    if (auto && auto.phase !== 'done') { auto = null; }
    else {
      const m = world.hives[alliance].upCellMouthWorld();
      auto = new AutoDriver(plan, [m[0], m[2]], world.geom.halfWidth_m);
      world.clock.startTeleOp();
      paused = false;
    }
  } else if (world.clock.period === 'STAGING') {
    world.clock.start();
    paused = false;
  } else {
    togglePause();
  }
  paintDeck();
}

// ---------------------------------------------------------------- wiring

function showTab(name: string): void {
  for (const b of $$('.tabs button')) b.classList.toggle('on', b.dataset.tab === name);
  for (const s of $$('#panel section')) s.classList.toggle('on', s.dataset.panel === name);
}

function wireUi(): void {
  for (const b of $$<HTMLButtonElement>('.tabs button')) b.onclick = () => showTab(b.dataset.tab!);
  for (const b of $$<HTMLButtonElement>('#mode button')) b.onclick = () => setMode(b.dataset.mode as Mode);
  $('#primary').onclick = primary;

  $<HTMLSelectElement>('#alliance').onchange = (e) => {
    alliance = (e.target as HTMLSelectElement).value as Alliance;
    build();
  };
  $<HTMLSelectElement>('#camera').onchange = (e) => { scene.cameraMode = (e.target as HTMLSelectElement).value as CameraMode; };
  $<HTMLSelectElement>('#brain-select').onchange = (e) => {
    useJavaBrain = (e.target as HTMLSelectElement).value === 'java';
    if (useJavaBrain) bridge.connect(paintBrain); else bridge.disconnect();
    paintBrain();
  };

  $('#apply-reset').onclick = () => { build(); $('#apply-reset').classList.remove('on'); };
  $('#tune-default').onclick = () => {
    params = structuredClone(baseParams);
    robotSpec = structuredClone(baseRobot);
    buildTunePanel($('#tune-rows'), params, robotSpec, onTune);
    build();
  };

  $('#a-start').onclick = primary;
  $('#a-clear').onclick = () => { world.shotLog.length = 0; chartedAt = -1; };
  $('#a-csv').onclick = () => download('shots', toCsv(world.shotLog));
  $('#a-apply').onclick = () => {
    const rep = analyse(world.shotLog, robotSpec.flywheel.tolRpm);
    if (!rep.fix.worthIt) return;
    // Trims ACCUMULATE. Each run measures the error that is left after the last trim, so
    // adding is what converges; replacing would undo the previous correction every time.
    robotSpec.calibration.rangeTrim_in += rep.fix.rangeTrim_in;
    robotSpec.calibration.turretTrim_deg += rep.fix.turretTrim_deg;
    world.shotLog.length = 0;
    chartedAt = -1;
  };
  $('#a-calclear').onclick = () => {
    robotSpec.calibration.rangeTrim_in = 0;
    robotSpec.calibration.turretTrim_deg = 0;
  };
  $('#a-trace').onclick = () => download('trace', trace.toCsv());

  installGrip();
  addEventListener('resize', () => { scene.resize(); chartedAt = -1; });
  addEventListener('keydown', (e) => {
    if ((e.target as HTMLElement).tagName === 'INPUT') return;
    const k = e.key.toLowerCase();
    if (e.key === 'Enter') primary();
    else if (k === 'p') togglePause();
    // N, not R: R is the speed gear now, and rebuilding the match under a driver's thumb
    // because they reached for more speed is the worst possible key collision.
    else if (k === 'n') build();
    else if (k === 'l') setAutoLoad(!autoLoad);
    else if (k === 'm') cycleMode();
    else if (k >= '1' && k <= '5') {
      scene.cameraMode = CAMERAS[Number(k) - 1];
      $<HTMLSelectElement>('#camera').value = CAMERAS[Number(k) - 1];
    }
  });
  paintBrain();
}

/**
 * Drag the panel's left edge to resize it. The width lives on :root so the grip, the panel
 * and the canvas all follow from one number, and it is remembered per browser because a
 * driver who widened it once should not have to do it again.
 */
function installGrip(): void {
  const grip = $('#grip');
  const apply = (px: number) => {
    const w = Math.max(300, Math.min(innerWidth * 0.7, px));
    document.documentElement.style.setProperty('--panel-w', `${w}px`);
    scene.resize();
    chartedAt = -1;
    try { localStorage.setItem('panelW', String(w)); } catch { /* private window */ }
  };
  try {
    const saved = Number(localStorage.getItem('panelW'));
    if (saved > 0) apply(saved);
  } catch { /* private window */ }
  grip.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    grip.classList.add('dragging');
    grip.setPointerCapture(e.pointerId);
    const move = (m: PointerEvent) => apply(innerWidth - m.clientX);
    const up = () => {
      grip.classList.remove('dragging');
      removeEventListener('pointermove', move);
      removeEventListener('pointerup', up);
    };
    addEventListener('pointermove', move);
    addEventListener('pointerup', up);
  });
}

/** Hand a CSV to the browser's downloader. */
function download(kind: string, text: string): void {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type: 'text/csv' }));
  a.download = `biobuzz-${kind}-${new Date().toISOString().slice(0, 19).replace(/[:T-]/g, '')}.csv`;
  a.click();
  URL.revokeObjectURL(a.href);
}

/**
 * Print which gamepad button indices are down, once per change.
 *
 * Exists because paddle numbering is not standardised and the only reliable way to learn a
 * pad's mapping is to press the button and look. Console only: it is a setup aid, not a
 * readout anybody needs while driving.
 */
let padSeen = '';
function padWatch(g: Gamepad): void {
  const down = g.buttons.map((b, i) => (b.pressed ? i : -1)).filter((i) => i >= 0);
  const key = down.join(',');
  if (key === padSeen) return;
  padSeen = key;
  const NAMES: Record<number, string> = {
    0: 'A', 1: 'B', 2: 'X', 3: 'Y', 4: 'L1', 5: 'R1', 6: 'L2', 7: 'R2',
    8: 'back', 9: 'start', 10: 'L3', 11: 'R3',
    12: 'D-pad up', 13: 'D-pad down', 14: 'D-pad left', 15: 'D-pad right',
    16: 'paddle 16', 17: 'paddle 17', 18: 'paddle 18', 19: 'paddle 19',
  };
  console.log(down.length
    ? `[pad] pressed: ${down.map((i) => `${i} (${NAMES[i] ?? '?'})`).join(', ')}`
    : '[pad] released');
}

function setAutoLoad(on: boolean): void {
  autoLoad = on;
}

/**
 * Practice aid: keep the hopper topped up, so aiming can be worked on without driving a
 * collection lap first. Not a game rule.
 *
 * IT MUST NOT RUN DRY, which it did. It only ever took POLLEN lying loose on the tiles, and
 * the field holds forty: once they were in the CELL, in a FLOWER, or benched, there was
 * nothing left to pick up. The robot fired for ten or fifteen seconds and then sat there
 * with an empty hopper, which reads exactly like the feed breaking.
 *
 * So it falls back, in order: a POLLEN on the floor (nearest first, which is what a real
 * intake would get), then any POLLEN out of play at all -- benched, or sitting in a CELL
 * that has already been counted. This is a cheat either way; a cheat that stops working
 * after fifteen seconds is just a bug wearing a disclaimer.
 */
/**
 * What the opponent's routine steers by: its own hive's live mouth, its own LOADING zone, and
 * the shot table's band. Rebuilt each frame because a TIP turns the mouth round, and a bot
 * that cached it would spend the rest of the match firing at the back of its own goal.
 */
function opponentField() {
  const opp = world.opponent!;
  const hive = world.hives[opp.alliance];
  const zone = world.geom.zones.find((z) => z.name === 'LOADING' && z.alliance === opp.alliance)!;
  const ranges = shotTable.rows.map((r) => r.range_in);
  return {
    mouth: hive.upCellMouthWorld(),
    mouthNormal: hive.upCellMouthNormalWorld(),
    loading: [zone.min[0], zone.min[2], zone.max[0], zone.max[2]] as [number, number, number, number],
    halfWidth_m: world.geom.halfWidth_m,
    band_in: [Math.min(...ranges), Math.max(...ranges)] as [number, number],
    flowers: flowerTbl?.flowers,
  };
}

/** The loose balls it is allowed to go for: on the floor, in play, and not our NECTAR. */
function opponentSight() {
  const opp = world.opponent!;
  const theirs = opp.alliance === 'red' ? 'nectarBlue' : 'nectarRed';
  const loose: [number, number][] = [];
  for (const b of world.balls.balls) {
    if (!b.body.isEnabled() || b.state !== 'free' || b.kind === theirs) continue;
    const p = world.balls.pos(b);
    if (p[1] * M_TO_IN > 12) continue;          // in a CELL or up a FLOWER, not on the floor
    const f = worldToFtc(p);
    loose.push([f[0], f[1]]);
  }
  return { loose, remaining: world.clock.remaining, period: world.clock.period, shotsTaken: opp.shots };
}

function topUpHopper(): void {
  const r = world.robot;
  if (r.hopper.length >= robotSpec.hopper.capacity) return;
  const p = r.pos;
  let best: (typeof world.balls.balls)[number] | null = null;
  let bestD = Infinity;
  let spare: (typeof world.balls.balls)[number] | null = null;
  for (const b of world.balls.balls) {
    if (b.kind !== 'pollen') continue;
    if (b.state === 'hopper' || b.state === 'intake' || b.state === 'flight') continue;
    // NEVER OUT OF A CELL OR A FLOWER. Those balls are SCORED. Taking one back is not a
    // practice aid, it is un-scoring a point -- and it emptied the CELL faster than the robot
    // could fill it, so the HIVE never reached the ball torque it needed to go over. A
    // recycler that reaches into the goal is worse than one that runs dry.
    if (b.state === 'cell' || b.state === 'flower') continue;
    if (b.state === 'free' && b.body.isEnabled()) {
      const q = world.balls.pos(b);
      if (q[1] <= inches(8)) {            // lying on the tiles, where an intake could reach it
        const d = Math.hypot(q[0] - p[0], q[2] - p[2]);
        if (d < bestD) { bestD = d; best = b; }
        continue;
      }
    }
    // Benched by park(): out of play and nobody's points. Fair to recycle.
    if (b.state === 'parked' && !spare) spare = b;
  }
  const take = best ?? spare;
  if (take) world.robot.preload(world.balls, take);
}

/**
 * Practice aid, the inverse of topUpHopper: put down everything the robot is carrying.
 *
 * THE BALLS STAY IN PLAY. Parking them would be simpler and it would quietly delete scoring
 * elements from the match -- 40 POLLEN is a fixed budget and the HIVE only goes over when
 * enough of them are in a CELL, so a button that eats four of them changes the game rather
 * than the robot. They are set down on the tiles instead, where an intake could pick them up
 * again.
 *
 * BEHIND the robot, not in front. The intake runs continuously (a real one does), so anything
 * dropped at the mouth is swallowed on the next frame and the button looks broken.
 */
function emptyHopper(): void {
  const r = world.robot;
  const held = r.heldBalls();
  if (!held.length) return;
  const back = robotSpec.chassis.length_m / 2 + 0.26;
  const lim = world.geom.halfWidth_m - 0.25;
  held.forEach((b, i) => {
    // Fanned out by more than a ball's width: two released into the same spot are two bodies
    // starting inside each other, which the contact solver answers by firing them apart.
    const across = (i - (held.length - 1) / 2) * (b.radius * 2.4);
    const w = r.toWorld([across, 0, -back]);
    const at: Vec3 = [
      Math.max(-lim, Math.min(lim, r.pos[0] + w[0])),
      b.radius + 0.005,
      Math.max(-lim, Math.min(lim, r.pos[2] + w[2])),
    ];
    world.balls.release(b, at, [0, 0, 0], [0, 0, 0], 'free');
  });
}

function paintBrain(): void {
  const detail: Record<string, string> = {
    off: 'relay down — run npm run relay',
    connecting: 'connecting',
    waiting: 'waiting for the Java runner',
    live: 'live',
    error: 'relay unreachable — run npm run relay',
  };
  $('#brain').textContent = useJavaBrain ? `java: ${detail[bridge.status]}` : 'built-in';
}

function togglePause(): void {
  paused = !paused;
  paintDeck();
}

/** Hand-drop a POLLEN into the up CELL: the quickest way to watch a tip happen. */
function dropBall(): void {
  const hive = world.hives[alliance];
  const free = world.balls.balls.find((b) => b.state === 'free' && b.kind === 'pollen' && b.body.isEnabled() && world.balls.pos(b)[1] < inches(6));
  if (!free) return;
  const cell = hive.upCell;
  const u = -cell.halfInterior[1] + free.radius + 0.02;
  const t = cell.halfInterior[2] - free.radius - 0.02 - Math.random() * 0.06;
  const local: Vec3 = fromCellLocal(cell, (Math.random() - 0.5) * inches(14), u, t);
  world.balls.release(free, hive.toWorld(local), [0, 0, 0], [0, 0, 0], 'cell');
}

void CONTROLS;
void DEG;
void lastAct;
boot();
