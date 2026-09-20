/**
 * A driver for the sim's own UI, so the app is usable with a gamepad before the Java
 * runner is attached. It is deliberately a mirror of what `TeleOpMain` does on the hub
 * and it is NOT the deliverable -- `java/teamcode` is. If the two ever disagree, the Java
 * is right. Keeping it here (rather than in the UI) means it is testable headlessly.
 */
import { clamp, DEG, RAD, inches, wrapPi, rpmToRadS } from '../units.js';
import { pThread } from '../physics/ballistics.js';
import type { LandCalibration } from './entryModel.js';
import type { HoodCell, HoodTable } from './hoodTable.js';
import type { FlowerCell, FlowerTable } from './flowerTable.js';
import { TagTarget, type TagTargetState } from './tagTarget.js';
import { PoseFuser } from './poseFuser.js';
import { shotIsBlocked, type ObstacleSpec } from './clearShot.js';
import tagOffsets from '../../../../config/tagoffsets.json' with { type: 'json' };
import type { ActuatorFrame, GamepadState, RobotSpec, SensorFrame } from '../types.js';

export interface ShotRow {
  range_in: number; hoodPos: number; rpm: number; margin: number;
  /** Absolute elevation this row was solved at, degrees. Absent in tables written before it. */
  hoodDeg?: number;
  /** The exit-speed band that threads the mouth, m/s. */
  speedLo?: number;
  speedHi?: number;
  /** One sigma of exit-speed error from all launch scatter, m/s. */
  sigmaSpeed?: number;
  /** Measured fraction of balls arriving like this that stay in (tools/entrycheck.ts). */
  pStay?: number;
  /**
   * Half-width of the CELL mouth ACROSS the shot line, metres, less the ball's radius.
   * Constant for every row -- it is field geometry -- but it rides in the table because the
   * table is the only field artefact the hub reads.
   */
  halfLat_m?: number;
}

export class ShotTable {
  constructor(readonly rows: ShotRow[]) {
    this.rows = [...rows].sort((a, b) => a.range_in - b.range_in);
  }

  static fromCsv(csv: string): ShotTable {
    const rows: ShotRow[] = [];
    for (const line of csv.trim().split(/\r?\n/).slice(1)) {
      const [r, h, rpm, m, deg, sLo, sHi, sig, ps, lat] = line.split(',').map(Number);
      if (Number.isFinite(r)) {
        rows.push({
          range_in: r, hoodPos: h, rpm, margin: m,
          hoodDeg: Number.isFinite(deg) ? deg : undefined,
          speedLo: Number.isFinite(sLo) ? sLo : undefined,
          speedHi: Number.isFinite(sHi) ? sHi : undefined,
          sigmaSpeed: Number.isFinite(sig) ? sig : undefined,
          pStay: Number.isFinite(ps) ? ps : undefined,
          halfLat_m: Number.isFinite(lat) ? lat : undefined,
        });
      }
    }
    return new ShotTable(rows);
  }

  lookup(range_in: number): ShotRow {
    const r = this.rows;
    if (!r.length) return { range_in, hoodPos: 0.5, rpm: 0, margin: 0 };
    if (range_in <= r[0].range_in) return r[0];
    if (range_in >= r[r.length - 1].range_in) return r[r.length - 1];
    for (let i = 1; i < r.length; i++) {
      if (range_in <= r[i].range_in) {
        const t = (range_in - r[i - 1].range_in) / (r[i].range_in - r[i - 1].range_in);
        const mix = (k: keyof ShotRow): number | undefined => {
          const a = r[i - 1][k] as number | undefined;
          const b = r[i][k] as number | undefined;
          return a === undefined || b === undefined ? undefined : a + t * (b - a);
        };
        return {
          range_in,
          hoodPos: r[i - 1].hoodPos + t * (r[i].hoodPos - r[i - 1].hoodPos),
          rpm: r[i - 1].rpm + t * (r[i].rpm - r[i - 1].rpm),
          margin: Math.min(r[i - 1].margin, r[i].margin),
          hoodDeg: mix('hoodDeg'),
          speedLo: mix('speedLo'),
          speedHi: mix('speedHi'),
          sigmaSpeed: mix('sigmaSpeed'),
          // Entry probability is NOT interpolated optimistically: between two rows, take the
          // worse one. A shot halfway between a 90% row and a 40% row is not 65% reliable in
          // any sense the robot can bank on.
          pStay: r[i - 1].pStay === undefined || r[i].pStay === undefined
            ? undefined
            : Math.min(r[i - 1].pStay as number, r[i].pStay as number),
          halfLat_m: mix('halfLat_m'),
        };
      }
    }
    return r[r.length - 1];
  }

  /**
   * The range band where a shot is most likely to LAND: where DriveToRange wants the robot.
   *
   * This was the band with the widest speed MARGIN, which is the 30-38 in rows -- the very
   * ranges where the fewest balls stay in. Margin is how much wheel error still threads the
   * mouth; a steep close lob threads with huge margin and then bounces back out, and the
   * measured stay rate climbs from 86% at 30-42 in to 95% at 54 and 99% at 78
   * (tools/ceiling.ts; tools/landrate.ts lands 12 of 12 at 55 and 70 in). So the band is the
   * rows within five points of the best per-shot ceiling the table carries, which is the same
   * product the readiness gate scores a shot by. Rows without the model fall back to margin.
   */
  bestBand(): [number, number] {
    if (!this.rows.length) return [60, 90];
    const ceiling = (r: ShotRow): number => {
      if (r.speedLo === undefined || r.speedHi === undefined || r.sigmaSpeed === undefined) return r.margin;
      return pThread(r.speedLo, r.speedHi, (r.speedLo + r.speedHi) / 2, r.sigmaSpeed) * (r.pStay ?? 1);
    };
    const best = Math.max(...this.rows.map(ceiling));
    const good = this.rows.filter((r) => ceiling(r) >= best - 0.05);
    return [good[0].range_in, good[good.length - 1].range_in];
  }
}

/**
 * Aim that accounts for the robot's own motion.
 *
 * The ball leaves with v_exit*dir + v_robot, so the shot has to be solved for the velocity
 * the BALL must have in the ground frame -- which is the table's answer, as a vector -- and
 * the robot's own velocity subtracted from it.
 *
 *     ground horizontal   S*cos(el) along the bearing        vertical   S*sin(el)
 *
 * THE VERTICAL IS PART OF THE ANSWER, and leaving it out is what broke this. The first
 * version solved the horizontal triangle only and kept the hood where the table put it, so
 * the ball left with a vertical of mag*tan(el) instead of S*sin(el) -- the horizontal ground
 * track was perfect and the hang time was wrong. Closing at 0.4 m/s from 40 in, the exit
 * speed came down from 5.28 to 4.09 m/s at a fixed 70 deg, which drops the vertical from
 * 4.97 to 3.85 and the ball NEVER REACHES the mouth's 1.46 m: not a miss, a shot that cannot
 * arrive. Receding sailed over it the same way. tools/movingfire.ts read that as "shooting
 * while accelerating is limited by the flywheel tachometer".
 *
 * Three components, three unknowns -- azimuth, ELEVATION and speed -- so solve all three:
 *
 *     el = atan2(vert, mag)        speed = hypot(mag, vert)
 *
 * Measured with no scatter and no gate (tools/_lead.ts): exact at every velocity and range,
 * against 14-49 cm long for lateral motion and no arrival at all for radial.
 *
 * It also all but removes the flywheel from the problem, which is the second prize. Over
 * +-0.8 m/s of closing speed at 40 in the old lead swung the target 1283-3388 rpm and the
 * wheel slews 1102 rpm/s; this one asks for 2242-2477, nine times less, and gives the rest
 * to a hood servo that is commanded rather than measured.
 *
 * Only velocity is compensated, not acceleration: over a one-second flight the a*t^2 term
 * is small next to the 1-2 deg of launch scatter, and a lead that differentiates a noisy
 * velocity is worse than no lead at all. (`transfer.leadLatency_s` is a different thing --
 * it predicts the velocity at RELEASE, not during the flight.)
 */
export function leadShot(
  bearingDeg: number,
  tableSpeed: number,
  elevationDeg: number,
  vxField: number,
  vyField: number,
  headingDeg: number,
  /** The hood's travel. The solved elevation is clamped into it. */
  hoodRange: readonly [number, number] = [-90, 90],
): { azimuthDeg: number; speed: number; elevationDeg: number; clamped: number; outrun: number } {
  const horiz = tableSpeed * Math.cos(elevationDeg * DEG);
  const vert = tableSpeed * Math.sin(elevationDeg * DEG);
  const still = { azimuthDeg: bearingDeg, speed: tableSpeed, elevationDeg, clamped: 0, outrun: 0 };
  if (horiz < 1e-3) return still;

  const bearingField = (headingDeg + bearingDeg) * DEG;
  const wantX = horiz * Math.cos(bearingField) - vxField;
  const wantY = horiz * Math.sin(bearingField) - vyField;
  const mag = Math.hypot(wantX, wantY);
  if (mag < 1e-3) return still;

  // HOW FAR THE HOOD FELL SHORT OF THE SOLUTION, degrees. 0 means the aim was achievable.
  //
  // Past the hood's travel there is no launch that matches the table's vector at all, and the
  // clamp below degrades smoothly rather than dividing by cos(85 deg) -- which is right, and
  // silent. A degraded solution is not a shot: the ball leaves with a vertical belonging to an
  // elevation the hood could not reach, and goes wherever that puts it. Measured, a robot
  // driving diagonally at 1.1 m/s from 45 in put five of five balls out, 138 cm wide, with the
  // velocity estimate good to 0.05 m/s and the turret nowhere near its stop. The aim was not
  // wrong; the aim was IMPOSSIBLE, and nothing said so.
  const wantEl = Math.atan2(vert, mag) * RAD;
  const clamped = Math.abs(wantEl - clamp(wantEl, hoodRange[0], hoodRange[1]));

  // WHEN THE ROBOT OUTRUNS THE BALL SIDEWAYS, m/s over.
  //
  // The lead cancels the robot's velocity by pointing the launch the other way, and the most
  // it can cancel ACROSS the shot line is the ball's own horizontal speed. Past that there is
  // no azimuth that works -- the ball is carried sideways faster than it is being thrown.
  //
  // It is not an exotic case. Horizontal speed is S*cos(elevation), and the table uses a
  // steep hood up close: at 45 in it asks for 69 deg, which leaves about 0.8 m/s of
  // horizontal out of a 2.3 m/s launch. A robot doing 1.06 m/s diagonally is FASTER THAN THE
  // BALL sideways. Measured, five of five such shots went out, 132 cm wide, with the velocity
  // estimate good to 0.05 m/s, the hood unclamped and the turret nowhere near its stop: the
  // aim was executed exactly and the aim was impossible.
  const cross = Math.abs(-vxField * Math.sin(bearingField) + vyField * Math.cos(bearingField));
  const outrun = Math.max(0, cross - horiz);

  return {
    clamped,
    outrun,
    // WRAP. atan2 returns (-180, 180] and the heading is subtracted from it, so the result
    // can land anywhere in (-540, 540). Unwrapped, a bearing of +90 came out as -270 and
    // clamped to the turret's -120 limit -- the robot aimed at its end stop and fired over
    // the wall. Every azimuth crossing the +-180 seam did this.
    azimuthDeg: wrapPi((Math.atan2(wantY, wantX) * RAD - headingDeg) * DEG) * RAD,
    // Clamped, and the speed kept as the magnitude of the vector we wanted rather than
    // re-solved for the clamped angle: past the hood's travel no launch matches the table's
    // vector at all, and this degrades smoothly instead of dividing by cos(85 deg). It bites
    // only when charging the goal at most of top speed from close in.
    elevationDeg: clamp(Math.atan2(vert, mag) * RAD, hoodRange[0], hoodRange[1]),
    speed: Math.hypot(mag, vert),
  };
}

/**
 * Velocity of the ball's EXIT POINT in the FTC field frame, m/s: v_cg + omega x r.
 *
 * leadShot() subtracts the velocity the ball inherits from the robot. The velocity it
 * inherits is the MUZZLE's, and on a yawing robot that is not the chassis's: the muzzle sits
 * muzzleOffset_m out along the shot line from the turret axis, so it swings at omega*r of its
 * own. At the 0.12 m of this robot and 90 deg/s that is 0.19 m/s, which puts a 60 in shot 4.7 in
 * sideways -- wider than the clearance to the lip. Reading the chassis velocity alone made the
 * error invisible to the aim AND (Robot.launch) absent from the flight, so the two agreed with
 * each other and neither agreed with a real robot.
 *
 * THE TURRET'S ACTUAL ANGLE, not its target: the muzzle is where the turret IS. The axis is
 * acceleration limited and a big swing takes most of a second, so on the commanded angle r
 * points somewhere the hardware has not reached.
 *
 * The turret axis is taken to sit over the tracked point, which is what Robot.muzzle() builds
 * (its pivot is [0, h, 0] in the body frame). Move the axis off the origin there and this has
 * to gain the same offset in the same commit, or the aim compensates a term the flight does
 * not have -- which is the bug this function exists to fix, with the sign flipped.
 */
export function muzzleVelocity(
  vxField: number,
  vyField: number,
  /** Yaw rate, deg/s, CCW positive -- the localizer's, noise and all. */
  omegaDps: number,
  headingDeg: number,
  turretActualDeg: number,
  /** Signed along the shot line: + ahead of the turret axis, - behind it. */
  muzzleOffset_m: number,
): { vx: number; vy: number } {
  const w = omegaDps * DEG;
  const shot = (headingDeg + turretActualDeg) * DEG;
  const rx = muzzleOffset_m * Math.cos(shot);
  const ry = muzzleOffset_m * Math.sin(shot);
  return { vx: vxField - w * ry, vy: vyField + w * rx };
}

/**
 * P(a ball stays in) against how many are already in the CELL. MEASURED, tools/whatmisses.ts,
 * 385 settled shots split into quartiles by the pocket's contents at the moment each left:
 *
 *   0 in: 95%    3 in: 88%    5 in: 85%    8 in: 60%
 *
 * A 35 point spread -- twice the next strongest feature and five times either of the two the
 * model was already built on. A ball arriving into a part-full pocket clips the ones already
 * there, which docs/DECISIONS.md has described for a while without anything acting on it.
 *
 * It is the one thing tools/landcal.ts structurally could not find: it fires ten shots into
 * an empty CELL and stops, so every sample it has ever taken was of an empty pocket, and the
 * curve it fitted was flat because within that slice nothing varies.
 */
const FILL_CURVE: [number, number][] = [[0, 0.95], [3, 0.88], [5, 0.85], [8, 0.60]];

/** Linear between the measured points, flat past either end. */
export function fillFactor(fill: number): number {
  const c = FILL_CURVE;
  if (fill <= c[0][0]) return c[0][1];
  if (fill >= c[c.length - 1][0]) return c[c.length - 1][1];
  for (let i = 1; i < c.length; i++) {
    if (fill <= c[i][0]) {
      const t = (fill - c[i - 1][0]) / (c[i][0] - c[i - 1][0]);
      return c[i - 1][1] + t * (c[i][1] - c[i - 1][1]);
    }
  }
  return c[c.length - 1][1];
}

/** One detent of the drive speed gear. Five gears spans the useful range without a menu. */
export const SPEED_STEP = 0.2;

/**
 * How fast the speed cap's feedback trim moves, per (m/s of error) per second. Slow enough
 * not to fight the velocity filter's 50 ms of lag, quick enough to arrive inside a launch.
 */
const CAP_TRIM = 0.8;

/**
 * How far above the open-loop fraction the cap's trim may reach.
 *
 * MEASURED, and it is the whole anti-lunge. An integrator against a 50 ms lagged velocity
 * winds up while the robot is still accelerating and then coasts past, so the bound -- not
 * the gain -- is what holds the peak down: at 1.25 every cap went 10% over. The floor alone
 * peaks at 0.92 of the cap (tools/topspeed.ts), so anything up to 1/0.92 = 1.087 still lands
 * under it, and that is also enough to close the ~9% rolling-resistance shortfall the trim
 * exists for. Swept at 1.08/1.14/1.18/1.22: the peak stays under the cap only at 1.08, and
 * the whole range delivers 91-95% of the speed asked for.
 *
 * RE-MEASURE with topspeed.ts after any drivetrain change: if the floor-only peak moves,
 * this moves with it.
 */
const CAP_TRIM_MAX = 1.08;
const gear = (v: number) => Math.round(clamp(v, SPEED_STEP, 1) / SPEED_STEP) * SPEED_STEP;

export interface TeleOpState {
  autoAim: boolean;
  /** Fire is a latch, not a trigger you hold: the cycle time paces it, not your thumb. */
  firing: boolean;
  /**
   * Heading the field frame is measured from, degrees. Field-centric drive rotates the stick
   * by (yaw - this), so tapping the re-zero makes 'up' mean whichever way the robot faces now.
   */
  headingZero: number;
  /** Manual turret command when auto-aim is off. */
  turretManualDeg: number;
  /**
   * Drive speed gear, 0..1, multiplying every translation and rotation command. SPEED_STEP
   * wide, so it is a ratchet with a small number of detents rather than a continuous trim
   * nobody can return to a known value.
   */
  speedScale: number;
  /** True while drivetrain.maxSpeed_mps is actively holding the robot back. */
  speedCapped: boolean;
  /** Balls the robot believes are already in the up CELL. Its own, since the last TIP. */
  cellFill: number;
  /**
   * Pre-spin latch. Firing implies it, so a driver never has to arm two things to shoot;
   * it exists on its own only so the wheel can be brought up before committing.
   */
  flywheelOn: boolean;
  ready: boolean;
  readyCount: number;
  /**
   * Probability THIS shot lands, right now: the chance the exit speed falls in the band that
   * threads the mouth, times the measured chance a ball arriving like that stays in.
   * -1 when the table predates the columns needed to compute it.
   */
  pLand: number;
  /** The model's raw score before calibration. Kept so the two can be compared. */
  pLandRaw: number;
  /** True when pLand has been through a measured calibration and is a real probability. */
  calibrated: boolean;
  /**
   * FLOWER mode: aim at the nearest tube and lob into its top instead of shooting the CELL.
   * A different target, a different table, and about a third of the exit speed.
   */
  flowerMode: boolean;
  /** Which term of the FLOWER gate is failing, for diagnosis. */
  flowerWhy: string;
  /** The tube being aimed at, its range and the solution, when in FLOWER mode. */
  flower: { index: number; range_in: number; bearingDeg: number; cell: FlowerCell | null } | null;
  /** Why the shot is being held, or '' if it is not. */
  hold: string;
  targetRpm: number;
  turretErrDeg: number;
  /**
   * THE POINTING ERROR, which is not the servo error above.
   *
   * `turretErrDeg` is command minus axis, and the command is `aimHold` -- a one-pole filter
   * on the solution. A filter lags whatever it is fed, so on a chassis that is yawing the
   * command trails the true bearing and the axis sits neatly on a command that is itself
   * wrong. The gate was reading that: it saw under 3 deg and called the shot aimed while the
   * muzzle was pointing somewhere else entirely. Measured in tools/movingfire.ts as a
   * +37 +- 57 cm LATERAL BIAS on the wobbling case, on a field whose two CELLs are 65 cm
   * apart -- which is the whole of the 'it puts them in the other alliance's hive' report.
   *
   * This one is the raw solution minus the axis, so the lag is inside it.
   */
  turretAimErrDeg: number;
  /** The mouth's half-width as seen along this shot, m. Shrinks with the cosine off-axis. */
  halfLatNow: number;
  /** Hood angle minus the angle this shot needs, degrees. The lead moves it every loop. */
  hoodErrDeg: number;
  /**
   * How much of this shot's speed band the robot's own acceleration will eat before the
   * ball leaves, as a fraction. Above 1 the shot is refused.
   */
  accelBudget: number;
  /** Degrees the hood fell short of the elevation the motion lead solved. 0 is achievable. */
  aimClampedDeg: number;
  /** m/s by which the chassis outruns the ball's own horizontal speed across the shot line. */
  aimOutrun: number;
  /** The exit speed and elevation the motion lead solved for, before the hood clamp. */
  leadSpeed: number; leadElevDeg: number;
  /** The table row the lead started from: its hood angle and rpm. Diagnostics. */
  rowHoodDeg: number; rowRpm: number;
  /** Radial velocity the lead worked from, m/s. Compare against the truth to see it lag. */
  leadVRadial: number;
  /** Inches the table lookup was moved to where the ball will actually leave from. */
  rangeLeadIn: number;
  /** The other HIVE's structure is across the shot line. No hood angle fixes that. */
  blocked: boolean;
  /** How far the motion lead moved the aim, degrees. */
  leadDeg: number;
  /** The bearing the lead ASKED for, before the turret's travel clamped it. */
  leadAzDeg: number;
  /** How far outside its travel that bearing was. Non-zero means the turret cannot take it. */
  turretPastStopDeg: number;
  note: string;
  /** Match time of the last feed pulse, and whether one is running. Transfer.java's timers. */
  lastFeedT: number;
  pulsing: boolean;
  /** Closing speed on the mouth, m/s. The fixed-speed table's second axis. */
  vRadial: number;
  /**
   * The three factors `pLandRaw` is the product of, kept apart so the dead one can be found:
   * the chance the launch threads the mouth, the measured chance a ball arriving like that
   * stays in, and the chance the aim is inside the mouth laterally. -1 when not computed.
   */
  pSpeed: number;
  pStayNow: number;
  pAim: number;
  /** True when the aim is coming off the AprilTag rather than off the localizer. */
  tagLocked: boolean;
  /** How many pixels across the tag is right now. Below the decode floor it is 0. */
  tagPx: number;
}

export const newTeleOpState = (): TeleOpState => ({
  autoAim: true, firing: false,
  turretManualDeg: 0, flywheelOn: false, speedScale: 1, speedCapped: false, cellFill: 0,
  ready: false, readyCount: 0, targetRpm: 0, turretErrDeg: 0, turretAimErrDeg: 0, halfLatNow: -1, hoodErrDeg: 0, leadDeg: 0, leadAzDeg: 0, turretPastStopDeg: 0,
  pLand: -1, pLandRaw: -1, calibrated: false, hold: '', note: '',
  flowerMode: false, flower: null, flowerWhy: '',
  lastFeedT: -999, pulsing: false, vRadial: 0, tagLocked: false, tagPx: 0, pSpeed: -1, pStayNow: -1, pAim: -1, headingZero: 0, blocked: false, accelBudget: 0, aimClampedDeg: 0, aimOutrun: 0, leadSpeed: 0, leadElevDeg: 0, leadVRadial: 0, rangeLeadIn: 0, rowHoodDeg: 0, rowRpm: 0,
});

const edge = (now: boolean, was: boolean) => now && !was;

export class BuiltinTeleOp {
  readonly state = newTeleOpState();
  private prev: GamepadState | null = null;

  constructor(
    private readonly spec: RobotSpec,
    private readonly table: ShotTable,
    /**
     * Measured score -> real-frequency mapping. Without it `pLand` is the raw model score,
     * which tools/gatecal.ts showed to be about 20 points optimistic, so the threshold does
     * not mean what it says.
     */
    private readonly landCal: LandCalibration | null = null,
    /**
     * The FIXED-SPEED table. When present the wheel is held at one speed all match and the
     * hood does the aiming, which takes the flywheel out of the control loop entirely -- it
     * never chases a target, so it never lags one, and its tachometer stops gating shots.
     */
    private readonly hoodTable: HoodTable | null = null,
    /**
     * Which HIVE this robot shoots into. It picks the baked mouth position the odometry
     * fallback aims at, and which CELL is up at the buzzer, so a brain built for the wrong
     * alliance aims across the field the moment the camera blinks.
     */
    alliance: 'red' | 'blue' = 'red',
    /**
     * The FLOWER solution and where the four tubes are. Without it FLOWER mode is simply
     * unavailable and says so, rather than aiming at a target it has no shot for.
     */
    private readonly flowerTable: FlowerTable | null = null,
  ) {
    const tg = spec.sensors.tag.target;
    this.targetFix = new TagTarget({
      // The same numbers the hub gets, from the same generated file -- the mirror has to
      // apply the tag -> mouth offset or it aims 2.8 in off where the deliverable does.
      mouthDx: { 1: tagOffsets.states.A.groundOffset_in, 2: tagOffsets.states.B.groundOffset_in },
      mouthFacingX: {
        1: tagOffsets.states.A.mouthNormalZ >= 0 ? 1 : -1,
        2: tagOffsets.states.B.mouthNormalZ >= 0 ? 1 : -1,
      },
      // A PRIOR AND A RIGID OFFSET, not a hardcoded goal position: the robot measures the
      // HIVE's pivot for itself the moment it decodes a tag, and aims off that.
      anchorPrior: tagOffsets.anchorPrior_in[alliance],
      mouthFromAnchor: {
        1: tagOffsets.states.A.mouthFromAnchor_in,
        2: tagOffsets.states.B.mouthFromAnchor_in,
      },
      anchorAlpha: tg.anchorAlpha ?? 0.15,
      fireOnOdometry: tg.fireOnOdometry !== false,
      // Unset means the old behaviour, which is to trust a decoded state for ever.
      maxStateAgeS: tg.maxStateAgeS ?? Infinity,
      measAlpha: tg.measAlpha ?? 0.35,
      // The rocker starts on the stop its alliance's CELL A sits up on for red, B for blue
      // (Hive's constructor: side = red ? -1 : +1). That is a fact at the buzzer and an
      // assumption from the first TIP nobody saw, which is what stateFromTag is for.
      startId: alliance === 'red' ? 1 : 2,
      holdS: tg.holdS,
      maxFireAgeS: tg.maxFireAgeS,
      scanRateDps: tg.scanRateDps,
      turretMinDeg: spec.turret.range_deg[0],
      turretMaxDeg: spec.turret.range_deg[1],
    });
    const fuse = spec.sensors.localizer.fuse;
    this.fuser = new PoseFuser({
      gain: fuse?.gain ?? 0.15,
      headingGain: fuse?.headingGain ?? 0.05,
      rejectOver_in: fuse?.rejectOver_in ?? 36,
    });
    // The TAG's own surveyed position, not the mouth's: pivot, out to the CELL, then back off
    // the rigid tag -> mouth correction. This is the known point the fix is measured against.
    const anchor = tagOffsets.anchorPrior_in[alliance];
    const tagOf = (st: { mouthFromAnchor_in: { x: number }; groundOffset_in: number }) =>
      ({ x: anchor.x + st.mouthFromAnchor_in.x - st.groundOffset_in, y: anchor.y });
    this.tagField = { 1: tagOf(tagOffsets.states.A), 2: tagOf(tagOffsets.states.B) };
    const ob = tagOffsets.obstacle_in;
    this.obstacle = {
      x: ob[alliance].x,
      y: ob[alliance].y,
      radius_in: ob.radius_in,
      halfWidth_in: ob.halfWidth_in,
    };
  }

  /** The pose the aim actually used: odometry plus every tag correction so far. */
  pose_(): { x: number; y: number; heading: number } { return this.pose; }
  /** Tag fixes taken, thrown out, and how far the last one moved the estimate. */
  fuseStats(): { applied: number; rejected: number; lastIn: number } {
    return { applied: this.fuser.applied, rejected: this.fuser.rejected, lastIn: this.fuser.lastCorrectionIn };
  }

  /**
   * The current target fix, for anything outside the brain that has to steer by it -- the
   * AUTO routine decides where to stand from the same estimate the shooter aims with, rather
   * than from a truth block the robot does not have. One loop behind, which does not matter
   * to a driving decision.
   */
  target(): TagTargetState { return this.tgt; }

  /** Recent flywheel readings, for the moving average the gate compares against tolRpm. */
  private readonly rpmHistory: number[] = [];
  /** Last velocity sample and the filtered acceleration built from it, for the lead. */
  private lastVel = { x: 0, y: 0, t: 0 };
  private accel = { x: 0, y: 0 };
  /** The gimbal's held bearing: filtered and deadbanded, so the axis locks instead of hunting. */
  /** FLOWER mode's own settle counter; the CELL path owns st.readyCount. */
  private flowerSettled = 0;
  private aimHold = 0;
  /**
   * The solution with the per-loop noise taken out but none of the motion: the estimate
   * of where the muzzle SHOULD be pointing that the gate measures the axis against.
   *
   * `aimRawDeg` is one localizer sample's worth of solution. At a 27 deg lead, 0.04 m/s
   * of velocity noise is 1.1 deg of bearing, so measuring the axis against the raw sample
   * tripped the 3 deg gate on noise 5-10% of the loops on a steady 1 m/s leg while the axis
   * was within a degree of where the filtered command had put it (tools/zonerun.ts
   * --aimtrace). Measuring against the filtered COMMAND hid the real lag instead, which is
   * how the wobbling case threw 37 cm wide. This carries the known rates forward the same
   * way the command does and takes only the noise out, with a faster pole than the command
   * so a real lag still shows.
   */
  private aimEst = 0;
  /** One-pole filtered localizer velocity. The lead is only ever as good as this. */
  private velFilt = { x: 0, y: 0 };
  /** Adaptive part of the speed cap. See the governor in update(). */
  private capGain = 1;
  /**
   * How full the robot believes the up CELL is: its own scored balls since the last TIP.
   *
   * It cannot see into the pocket, so this is a running sum of the calibrated P(land) of
   * every ball it has fed -- which is what a team does by counting. Reset when the tag ID
   * changes, because that IS the tip and the tip empties the CELL.
   */
  private cellFill = 0;
  private lastTagId = 0;
  /**
   * WHERE THE GOAL IS, from the camera rather than from the world. The mirror of the Java
   * deliverable's TagTargetProvider, and the reason every aiming number below is now an
   * estimate: `s.game.truth` is still in the frame and reading it here would be cheating.
   */
  private readonly targetFix: TagTarget;
  /** Tag fixes correcting the dead-reckoned pose. The other half of the loop from TagTarget. */
  private readonly fuser: PoseFuser;
  /** The SURVEYED tag position per rocker state, which is what makes a fix a pose measurement. */
  private readonly tagField: Record<number, { x: number; y: number }>;
  /** The corrected pose, so telemetry and the AUTO routine read what the aim used. */
  private pose = { x: 0, y: 0, heading: 0 };
  /** The OTHER HIVE, as an obstacle. Its CELLs sweep a disc, so its state does not matter. */
  private readonly obstacle: ObstacleSpec;
  private lastFusedT = -1e9;
  /** Last answer from it, so telemetry and the gate read the same one. */
  private tgt: TagTargetState = {
    azimuthDeg: 0, rangeIn: 0, openDeg: 180, valid: false, fresh: false,
    ageS: Infinity, lostLock: false, scanning: true, id: 0,
    fromOdometry: false, stateFromTag: false,
    anchor: { x: 0, y: 0 }, anchorFromTag: false,
  };
  /** The fixed-speed solution for this loop, or null when there is no shot from here. */
  private hoodCell: HoodCell | null = null;

  /** Servo position for a hood ANGLE, which is what both tables now deal in. */
  private hoodCommand(leadElevDeg: number): number {
    const [lo, hi] = this.spec.hood.angleRange_deg;
    // The fixed-speed table owns the hood outright when it is loaded; otherwise it is the
    // lead's solved elevation, which equals the shot table's own angle when standing still.
    const deg = this.hoodCell ? this.hoodCell.mid : leadElevDeg;
    return clamp((deg - lo) / Math.max(1e-6, hi - lo), 0, 1);
  }

  /** Hood angle the servo is actually at, from its reported position. */
  private hoodActualDeg(pos: number): number {
    const [lo, hi] = this.spec.hood.angleRange_deg;
    return lo + pos * (hi - lo);
  }

  /**
   * Robot-centric mecanum drive plus the mechanisms, from one gamepad.
   * `dt` is the frame period; the manual turret slews at a rate, not per-call, so the aim
   * does not move faster on a faster machine.
   */
  update(s: SensorFrame, g: GamepadState, seq: number, dt = 1 / 60): ActuatorFrame {
    const st = this.state;
    const p = this.prev;
    if (p) {
      // SPEED GEAR, Y up and A down. A driver wants a speed limit they can SET, not one they
      // have to keep a thumb on: the gear survives letting go of the stick, which is what
      // "crawl for the last six inches, then go" actually needs. The hold-to-crawl bumper is
      // still there on top of it for a momentary dab.
      if (edge(g.y, p.y)) st.speedScale = gear(st.speedScale + SPEED_STEP);
      if (edge(g.a, p.a)) st.speedScale = gear(st.speedScale - SPEED_STEP);
      if (edge(g.dpad_up, p.dpad_up)) st.flywheelOn = !st.flywheelOn;
      if (edge(g.x, p.x)) st.autoAim = !st.autoAim;
      // RE-ZERO THE FIELD FRAME. Field-centric is only as good as the heading it rotates by,
      // and a real IMU drifts; every driver wants a "forward is where I am pointing now"
      // button. Square the robot up to the field and tap it.
      if (edge(g.left_stick_button, p.left_stick_button)) st.headingZero = s.imu.yaw;
      if (edge(g.right_bumper, p.right_bumper)) st.firing = !st.firing;
      // FLOWER mode on the same kind of edge. The manual turret nudge reads dpad_left as
      // well, and deliberately: nudging by hand is for when auto-aim is off, and FLOWER
      // mode aims itself, so the two are never wanted at once.
      if (edge(g.dpad_left, p.dpad_left) && this.flowerTable) st.flowerMode = !st.flowerMode;
    }
    this.prev = { ...g };

    // ---- drive
    // ROBOT-CENTRIC by default: the stick points at the ROBOT'S NOSE, which is the intake.
    //
    // This was field-centric, on the argument that a driver should not have to track which way
    // the chassis is pointing. That argument is right for a robot you only have to POSITION and
    // wrong for this one: almost everything the driver does with the chassis is aim the INTAKE
    // at a ball, and the intake is bolted to the front. Field-centric makes "drive at that
    // ball" a mental rotation on every approach, and the robot that results feels like it is
    // being flown rather than driven -- which is exactly the complaint.
    //
    // The turret is why this costs nothing: the chassis never has to face the goal, so the
    // nose is free to mean "where I am collecting from" all match.
    //
    // Field-centric is still there, held on R3, measured from whatever heading the re-zero
    // (L3) last called forward.
    const slow = (g.left_bumper ? 0.35 : 1) * st.speedScale;
    const sx = -g.left_stick_y * slow;   // stick up
    const sy = -g.left_stick_x * slow;   // stick left
    const om = -g.right_stick_x * slow;

    const fieldCentric = g.right_stick_button;   // hold R3 for field-centric; robot-centric otherwise
    const h = (s.imu.yaw - st.headingZero) * DEG;
    let vx = fieldCentric ? sx * Math.cos(h) + sy * Math.sin(h) : sx;   // robot forward
    let vy = fieldCentric ? -sx * Math.sin(h) + sy * Math.cos(h) : sy;  // robot left

    // THE SPEED CAP, in m/s, governed against the MEASURED speed rather than computed from
    // power. The drive gear above is a power fraction and power is not speed -- it changes
    // with the battery, the floor and which way a mecanum is pointed (this robot does
    // 2.19 m/s forward and 1.62 strafing on the same stick, tools/topspeed.ts). A shot,
    // meanwhile, is refused in m/s: charging the mouth faster than the ball's own horizontal
    // speed leaves no launch under the hood's stop, so inside 50 in at 1.7 m/s the robot
    // cannot shoot at all (tools/frontcheck.ts). A driver who wants to stay shootable needs
    // to ask for a SPEED, and that is this.
    //
    // Translation only: rotation is not what outruns the ball, and scaling it here would
    // make the robot turn slower whenever it happened to be moving fast.
    //
    // Governed off the previous frame's filtered velocity -- velFilt is updated further down
    // -- which is 16 ms stale and entirely good enough for a limiter.
    const cap = this.spec.drivetrain.maxSpeed_mps ?? 0;
    const free = this.spec.drivetrain.freeSpeed_mps ?? 0;
    st.speedCapped = false;
    if (cap > 0) {
      // OPEN LOOP SETS THE FLOOR, so the robot never launches past the cap. Feedback alone
      // cannot do this: it only sees an overshoot once the velocity filter has caught up,
      // and measured that way a 0.80 m/s cap peaked at 1.29 (tools/topspeed.ts).
      const floor = free > 0 ? Math.min(1, cap / free) : 1;
      const now = Math.hypot(this.velFilt.x, this.velFilt.y);
      // AND THE TRIM WALKS IT BACK UP, because power is not speed. cap/free assumes they are
      // proportional; rolling resistance means reaching `cap` costs more than cap/free of
      // full power, so the floor alone always UNDER-delivers -- asking for 1.70 got 1.56.
      // The trim closes that, and on a real robot it is also what absorbs a flat battery and
      // a full hopper. Bounded to [floor, 1]: a limiter may take power away and must never
      // add any the driver did not ask for.
      //
      // Held at the floor below half the cap, which is the anti-windup. Left integrating
      // while parked the gain reaches its ceiling and the next launch starts unlimited --
      // the exact lunge the floor exists to prevent. The threshold is half rather than
      // something tighter because a HIGH one is its own trap: at 0.85 the trim engaged,
      // pushed the robot up, dropped back under the threshold, snapped to the floor and
      // parked in a limit cycle at 0.87 of the cap, which read as the bound being too tight
      // when the bound was not involved at all.
      const ceil = Math.min(1, floor * CAP_TRIM_MAX);
      this.capGain = now > cap * 0.5
        ? clamp(this.capGain + (cap - now) * CAP_TRIM * dt, floor, ceil)
        : floor;
      if (this.capGain < 1) {
        vx *= this.capGain;
        vy *= this.capGain;
        st.speedCapped = true;
      }
    }

    // The wire carries what the motor is actually told, so a reversed motor is negated
    // here exactly as the Java does with setDirection(REVERSE). The world then applies the
    // physical reversal; forgetting this half makes the robot sit still with all four
    // wheels fighting each other.
    const denom = Math.max(1, Math.abs(vx) + Math.abs(vy) + Math.abs(om));
    const dm = this.spec.drivetrain.motors;
    const dir = (w: 'fl' | 'fr' | 'bl' | 'br') => (dm[w].reversed ? -1 : 1);
    const motors: ActuatorFrame['motors'] = {
      fl: { mode: 'RUN_WITHOUT_ENCODER', power: (dir('fl') * (vx - vy - om)) / denom },
      fr: { mode: 'RUN_WITHOUT_ENCODER', power: (dir('fr') * (vx + vy + om)) / denom },
      bl: { mode: 'RUN_WITHOUT_ENCODER', power: (dir('bl') * (vx + vy - om)) / denom },
      br: { mode: 'RUN_WITHOUT_ENCODER', power: (dir('br') * (vx - vy + om)) / denom },
    };

    // ---- intake: always running, because a real one is. The left trigger reverses it.
    const intake = g.left_trigger > 0.1 ? -1 : 1;
    motors.intake = { mode: 'RUN_WITHOUT_ENCODER', power: intake };

    // ---- aim
    //
    // WHERE THE GOAL IS, FIRST, because nothing below means anything without it. The camera
    // rides the turret and is blind most of the time: out of the lens, past its range, too
    // far round the side, and -- the case that used to be free -- for the whole of a TIP.
    // `tgt` is the fix carried on odometry between detections, and `tgt.fresh` is the only
    // thing that may fire. `s.game.truth` is right there and reading it is cheating.
    // CORRECT THE POSE FIRST, then aim with it. A tag at a surveyed position is a measurement
    // of the ROBOT before it is anything else, and odometry now drifts, so this is the only
    // thing keeping the estimate tied to the field.
    this.pose = this.fuser.propagate(s.t, s.localizer);
    if (s.tag && s.tag.sampleT > this.lastFusedT) {
      this.lastFusedT = s.tag.sampleT;
      const tf = this.tagField[s.tag.id];
      const face = s.tag.id === 2
        ? (tagOffsets.states.B.mouthNormalZ >= 0 ? 1 : -1)
        : (tagOffsets.states.A.mouthNormalZ >= 0 ? 1 : -1);
      if (tf) this.pose = this.fuser.observe(tf.x, tf.y, s.tag.bearingDeg, s.tag.rangeIn, face, s.tag.openDeg, s.tag.sampleT);
    }
    const tgt = this.targetFix.update(s.t, this.pose, s.tag, dt,
      s.tag ? this.fuser.poseAt(s.tag.sampleT) : undefined);
    this.tgt = tgt;

    // The table is looked up at the TRIMMED range. A group that lands 8 in long means the
    // table's answer for R actually reaches R+8, so asking it for R-8 lands on the mouth --
    // one number, measured from a collected run, that corrects every range at once. This is
    // what a team adjusts between matches instead of regenerating the table.
    const rawVx = s.localizer.vx * 0.0254;
    const rawVy = s.localizer.vy * 0.0254;
    const a = clamp(this.spec.sensors.localizer.velFilterAlpha ?? 1, 0.01, 1);
    this.velFilt.x += a * (rawVx - this.velFilt.x);
    this.velFilt.y += a * (rawVy - this.velFilt.y);
    const velX = this.velFilt.x;
    const velY = this.velFilt.y;

    // LOOK THE TABLE UP AT THE RANGE THE BALL WILL LEAVE AT, not the one it is committed at.
    //
    // The feed takes about four tenths of a second from the gate opening to the ball clearing
    // the nip, and the wheel is chasing a target that moves the whole time: a robot closing at
    // 1 m/s covers 40 cm in that window, so the rpm the shot needs at RELEASE is not the rpm
    // the table was asked for at commit. Measured at 0.7 of stick, the wheel arrived +26 rpm
    // fast closing (shots 18-24 cm LONG) and 60-80 rpm slow receding and strafing (13-34 cm
    // SHORT) -- the same wheel, the same gate, the error signed by the direction of travel.
    //
    // Predicting the range costs nothing and points the wheel where it needs to be. The
    // velocity is the filtered one for the same reason the lead uses it: an unfiltered radial
    // velocity would jitter the table lookup.
    const cal = this.spec.calibration ?? { rangeTrim_in: 0, turretTrim_deg: 0 };
    // ONE HEADING, THE FUSED ONE. `tgt.azimuthDeg` is relative to the fused pose's heading;
    // the lead used to convert it to the field frame with `s.imu.yaw`, which is 40 ms stale
    // (hub.imuLatencyMs) and uncorrected. On a chassis yawing at the 70 deg/s cap that is
    // nearly 3 deg of frame mismatch between the bearing and the velocity being subtracted
    // from it. AimController.java has always used the localizer's heading for all of it.
    const heading = this.pose.heading;
    const bearingNow = (heading + tgt.azimuthDeg) * DEG;
    const vrNow = velX * Math.cos(bearingNow) + velY * Math.sin(bearingNow);
    const rangeLead = this.spec.flywheel.rangeLead_s ?? 0;
    const rangeAtRelease = tgt.rangeIn - (vrNow * rangeLead) / 0.0254;
    st.rangeLeadIn = rangeAtRelease - tgt.rangeIn;
    // THE ROW IS FOR NOW; ONLY THE WHEEL LOOKS AHEAD.
    //
    // This looked the whole row up at the predicted release range, and handed the hood and
    // the lead that row too. The hood is a servo and the lead is arithmetic: both are
    // re-solved every loop right up to the frame the ball leaves, so the row they need is
    // the one for where the robot IS, and a row for 0.15 s ahead is a row for a shot that is
    // 0.15 * v_radial too close. Measured with scatter off (tools/flightcheck.ts): closing at
    // 0.5 m/s from 52 in the ball crossed the mouth plane 6.6 cm LOW; with the row at the
    // current range it crossed 1 cm high. The wheel is the one thing that lags -- that is
    // what the range lead was measured against -- so the look-ahead goes to the rpm target
    // alone, as the rpm the table will want by the time the wheel has got there.
    const row = this.table.lookup(tgt.rangeIn - cal.rangeTrim_in);
    const rowAhead = this.table.lookup(rangeAtRelease - cal.rangeTrim_in);
    // INSIDE THE TABLE'S FIRST ROW THERE IS NO SHOT. lookup() clamps to the nearest row and
    // says nothing, so a robot 6 in from the hive fired the 30 in solution and put every
    // ball into the front of the pocket. The shot map has always painted this band as "too
    // close"; the brain now agrees with it.
    const rangeHere = tgt.rangeIn - cal.rangeTrim_in;
    // A FLOOR ABOVE THE TABLE'S OWN. The table solves a shot from 30 in, and one exists --
    // it threads the mouth. It just does not STAY: close in the only arc that fits is a steep
    // lob, and a steep lob arrives nearly vertically and bounces back out of the pocket. The
    // per-shot ceiling is 86-88% at 30-42 in against 95% at 54 (tools/ceiling.ts), and the
    // shots bear it out -- 385 settled shots split by range land 71% under 42 in and 83-88%
    // from 44 to 46 (tools/whatmisses.ts), the largest spread of any feature recorded.
    //
    // So this is not the table's limit, it is where the shot becomes worth taking.
    const floor_in = this.spec.shot?.minRange_in ?? 0;
    const inTable = this.table.rows.length === 0
      || (rangeHere >= Math.max(this.table.rows[0].range_in, floor_in)
        && rangeHere <= this.table.rows[this.table.rows.length - 1].range_in);
    const ticksPerDeg = this.spec.turret.motor.ticksPerDeg ?? 8;
    const hoodDeg = this.spec.hood.enabled
      ? this.spec.hood.angleRange_deg[0] + row.hoodPos * (this.spec.hood.angleRange_deg[1] - this.spec.hood.angleRange_deg[0])
      : this.spec.hood.fixedAngle_deg;
    const tableSpeed = this.spec.flywheel.k * this.spec.flywheel.r_fly_m * rpmToRadS(row.rpm);
    st.rowHoodDeg = hoodDeg;
    st.rowRpm = row.rpm;
    // LEAD ON THE VELOCITY THE ROBOT WILL HAVE WHEN THE BALL LEAVES, not the one it has now.
    //
    // leadShot's own note says acceleration is not worth compensating because the a*t^2 term
    // over a one-second flight is small next to launch scatter. That is true and it is about
    // the wrong interval: the ball does not care what the robot does after release. What
    // matters is the gap between COMMANDING the shot and the ball LEAVING -- the feed pulse
    // plus the wheel's own lag -- because the exit speed was chosen for the velocity at the
    // start of it. First-order kinematics covers it: v_release = v + a*tau.
    //
    // The measurement that forced this: driving with a wobbling stick, the robot fired MORE
    // than in any other case and landed NOTHING, with the lowest rpm error at fire of the
    // lot. The wheel was exactly on its target; the target was stale.
    // FILTER THE REPORTED VELOCITY BEFORE AIMING ON IT. The whole lead hangs off this number,
    // and on a real robot it is a differentiated encoder rather than the ground truth the
    // simulator used to hand over. With the odometry noise now modelled at all -- 0.04 m/s,
    // which is a degree of bearing against a 2.3 m/s ball -- the raw reading jitters the lead
    // azimuth, the turret chases the jitter, its tracking error never settles under the gate's
    // 3 deg, and the robot stops shooting: three of the shoot tests fired nothing.
    //
    // One pole, because the thing being estimated moves on the timescale of the robot's own
    // acceleration (tenths of a second) and the noise is per loop. alpha = 0.25 at 60 Hz is
    // about 50 ms of lag for roughly a third of the noise, which is the trade a team makes on
    // a real puck.
    const tau = this.spec.transfer.leadLatency_s ?? 0;
    let leadVx = velX;
    let leadVy = velY;
    if (tau > 0) {
      const dt = s.t - this.lastVel.t;
      if (dt > 1e-4 && this.lastVel.t > 0) {
        // Low-passed, because this differentiates a velocity estimate. In the sim that
        // estimate is exact; on a robot it is odometry and the filter is what stops a single
        // noisy sample from throwing the aim. Prefer an IMU's own accelerometer if there is
        // one -- it measures acceleration instead of inferring it.
        const k = 0.25;
        this.accel.x += k * ((velX - this.lastVel.x) / dt - this.accel.x);
        this.accel.y += k * ((velY - this.lastVel.y) / dt - this.accel.y);
        // DEADBAND, because this differentiates a velocity and a standing robot still jitters.
        //
        // The correction is only worth making when it exceeds what the wheel can resolve: one
        // encoder count over a 20 ms window is about 107 rpm, and the lead moves the target by
        // roughly 912 rpm per m/s, so a*tau has to be worth more than 0.12 m/s to mean
        // anything. Below that it is noise being fed into the aim -- and it cost shots: a
        // stationary rig that had been firing three times in ten seconds fired twice.
        const dv = Math.hypot(this.accel.x, this.accel.y) * tau;
        if (dv > 0.12) {
          leadVx = velX + this.accel.x * tau;
          leadVy = velY + this.accel.y * tau;
        }
      }
      this.lastVel = { x: velX, y: velY, t: s.t };
    }
    // Where the turret ACTUALLY is, from its encoder -- not where it was told to go. The
    // axis is acceleration limited, so a 137 deg swing takes most of a second, and firing
    // on the commanded angle means firing at nothing.
    // ONE FRAME STALE, BY CONSTRUCTION. The sensor frame is built before the world steps, so
    // the encoder the brain reads is always a loop old. At 60 Hz that is 16.7 ms, and during a
    // reversal the required bearing moves 150-200 deg/s -- 2.5 to 3.3 deg, which is the whole
    // of the 3 deg readiness window. The gate then refuses for "turret off target" on an axis
    // that is exactly where it was told to be, and no slew rate can fix a measurement delay.
    //
    // `predictEncoder` carries the reading forward by the rate the encoder itself reports.
    // That rate is measured over the hub's own 20 ms window and rounded to whole ticks, so it
    // is not free of lag either; this closes most of the gap, not all of it.
    const tSense = s.motors.turret;
    const turretActualDeg = tSense
      ? (tSense.pos + (this.spec.turret.predictEncoder ? tSense.vel * dt : 0)) / ticksPerDeg
      : 0;
    // THE BALL INHERITS THE MUZZLE'S VELOCITY, NOT THE CHASSIS'S. omega x r, added to both
    // consumers of the velocity below: the lead that cancels it and the radial axis of the
    // hood table. Robot.launch() applies the matching term to the flight.
    const mv = muzzleVelocity(leadVx, leadVy, s.localizer.omega, heading, turretActualDeg, this.spec.turret.muzzleOffset_m);
    const muzzleDvx = mv.vx - leadVx;
    const muzzleDvy = mv.vy - leadVy;
    const lead = leadShot(tgt.azimuthDeg, tableSpeed, hoodDeg, mv.vx, mv.vy, heading, this.spec.hood.angleRange_deg);

    // ---- FIXED-SPEED PATH: the wheel holds one speed and the hood aims.
    //
    // The robot's radial velocity -- how fast it is closing on the mouth -- is the table's
    // second axis rather than something to cancel. Positive is closing. Lateral motion barely
    // moves the answer, which is why this is the only component that has to be known.
    const bearingField = bearingNow;
    const vRadial = (velX + muzzleDvx) * Math.cos(bearingField) + (velY + muzzleDvy) * Math.sin(bearingField);
    st.vRadial = vRadial;
    this.hoodCell = this.hoodTable && !this.hoodTable.isEmpty
      ? this.hoodTable.lookup(tgt.rangeIn - cal.rangeTrim_in, vRadial)
      : null;
    // ---- FLOWER MODE: a different target, a different table, a third of the exit speed.
    //
    // The tube is a 4.0 in hole at 22.6 in and a POLLEN is 2.80, so the whole tolerance is
    // 0.6 in either side and the lob has to come DOWN through it -- a ball passing over is
    // a ball behind the flower. tools/flowertable.ts solves the pair per stand-off; here
    // the robot only has to pick the nearest tube and read the row.
    //
    // It aims off the FUSED POSE, not off a tag: the tubes carry no AprilTag, so a flower
    // shot is exactly as good as the odometry, which is 1.1 in over a 30 s lap
    // (tools/driftcheck.ts) against 0.6 in of hole. That is why the gate below wants the
    // fix fresh even though it is not aiming at the fix.
    let flowerLead = null as null | { azimuthDeg: number; cell: FlowerCell };
    if (st.flowerMode && this.flowerTable && !this.flowerTable.isEmpty) {
      const near = this.flowerTable.nearest(this.pose.x, this.pose.y);
      // FROM THE MUZZLE, NOT THE ROBOT'S CENTRE. tools/flowertable.ts flies every row from
      // the muzzle, and the pose is the chassis origin: the muzzle sits muzzleOffset_m out
      // along the shot line. On a 50 in CELL shot that 4.7 in is 9% and nobody notices; on
      // a 14 in flower it is a THIRD of the range, and the first version lobbed 68 balls
      // without one going in.
      const muzzleOff_in = this.spec.turret.muzzleOffset_m / 0.0254;
      const cell = near ? this.flowerTable.lookup(near.range_in - muzzleOff_in) : null;
      st.flower = near ? { ...near, cell } : null;
      if (near && cell) {
        flowerLead = { azimuthDeg: wrapPi((near.bearingDeg - this.pose.heading) * DEG) * RAD, cell };
      }
    } else {
      st.flower = null;
    }
    st.leadDeg = wrapPi((lead.azimuthDeg - tgt.azimuthDeg) * DEG) * RAD;
    st.leadAzDeg = lead.azimuthDeg;
    let turretDeg: number;
    /** The unfiltered solution, kept so the gate can see the filter's own lag. */
    let aimRawDeg: number | null = null;
    if (st.autoAim) {
      // Lead-corrected bearing to the up CELL, relative to the robot's heading.
      // The lateral trim also absorbs a turret encoder zero that is a degree out, which
      // looks identical in the data and has the same fix.
      // A GIMBAL, NOT A WEATHERVANE.
      //
      // The aim is re-solved every loop from a localizer carrying 0.5 in and 0.5 deg of noise,
      // so the raw solution jitters even with the robot parked. Commanding it straight through
      // made the axis hunt continuously: it never settled, the readiness gate saw an error
      // that never stopped moving, and every shot left while the head was still travelling.
      //
      // Two things fix that, and both are what a gimbal does. A one-pole filter on the
      // COMMAND takes the noise out without lagging a real slew -- the bearing to a fixed goal
      // moves on the timescale of the robot's own driving, tens of milliseconds, and the noise
      // is per-frame. And a deadband holds the command still while the solution is inside it,
      // so the axis stops rather than creeping: below the launch yaw scatter there is nothing
      // to chase, because a tenth of a degree of aim is invisible next to a degree of scatter.
      // In FLOWER mode the turret points at the tube, not at the CELL.
      const raw = (flowerLead ? flowerLead.azimuthDeg : lead.azimuthDeg) - cal.turretTrim_deg;
      aimRawDeg = raw;
      // FEED THE CHASSIS ROTATION FORWARD THROUGH THE FILTER. The one-pole filter below is
      // for localizer noise, and it lags whatever it is fed; a chassis turning at w drags the
      // required bearing at exactly -w, which is not noise and should not be filtered. Left
      // to the filter, 66 deg/s of spin is about 2 deg of lag on the command, and the gate --
      // now measuring the true pointing error -- refused 80% of the spinning case's loops for
      // "turret N deg off". The turn rate is known from the IMU, so the accumulator is moved
      // by it first and the filter is left with only the part it is for.
      //
      // AND THE BEARING'S OWN RATE FROM TRANSLATION, for the same reason. A robot crossing the
      // mouth at v sees the bearing to a fixed target turn at (v x r) / R^2 -- 0.84 m/s at
      // 38 in is 50 deg/s -- which the filter lagged just as it lagged the yaw. With the cap
      // raised to where the shots land, "turret N deg off" became the top hold at speed:
      // 22-35% of loops at 0.6-0.85 m/s (tools/zonerun.ts --cap). Both rates are known from
      // the localizer, so both go through the accumulator before the filter sees anything.
      const rangeNowM = Math.max(0.3, inches(tgt.rangeIn));
      const bearingRate = ((velX * Math.sin(bearingField) - velY * Math.cos(bearingField)) / rangeNowM) * RAD;
      this.aimHold += (bearingRate - s.localizer.omega) * dt;
      this.aimEst += (bearingRate - s.localizer.omega) * dt;
      this.aimEst += wrapPi((raw - this.aimEst) * DEG) * RAD * 0.6;
      this.aimEst = wrapPi(this.aimEst * DEG) * RAD;
      // WHAT THE READINESS GATE MEASURES THE AXIS AGAINST.
      //
      // `aimEst` is a one-pole (0.6) on the solution, and the COMMAND is a different one-pole
      // (0.35) with a deadband. Two filters on one signal, so even a turret with infinite slew
      // sitting exactly on its command carries a standing error of aimEst - aimHold, and that
      // error grows with bearing rate. It is charged to the turret and no amount of slew rate
      // can remove it (measured: 261 -> 1400 deg/s changes nothing).
      //
      // `gateOnRawAim` measures against the UNFILTERED solution instead, which is the honest
      // reference when the signal is clean. It is off by default because with real localizer
      // noise the raw solution jitters by about a degree and the gate would trip on noise --
      // which is the fault aimEst was introduced to avoid.
      aimRawDeg = this.spec.turret.gateOnRawAim ? raw : this.aimEst;
      const dead = this.spec.turret.aimDeadband_deg ?? 0.25;
      if (Math.abs(wrapPi((raw - this.aimHold) * DEG) * RAD) > dead) {
        const a = clamp(this.spec.turret.aimFilterAlpha ?? 0.35, 0.01, 1);
        this.aimHold += wrapPi((raw - this.aimHold) * DEG) * RAD * a;
        // And keep the accumulator on the circle. Letting it run is what walked it off.
        this.aimHold = wrapPi(this.aimHold * DEG) * RAD;
      }
      // UNWIND RATHER THAN PIN. A bearing is only ever known modulo 360, so a turret with more
      // than a full turn of travel can reach most of them two ways -- and which way matters,
      // because a robot spinning on the spot drives the bearing through +-180 again and again.
      //
      // Clamping the wrapped solution pinned the axis at its stop and left it there: the gate
      // refused the shot (correctly) and nothing ever recovered, so the robot came round and
      // the turret was still parked against the same end. Picking whichever of want, want-360
      // and want+360 is INSIDE the travel and nearest the axis now makes it take the short way
      // and keep tracking; with 370 deg of travel there is always at least one, so the dead
      // cone directly behind the robot is gone.
      const [lo, hi] = this.spec.turret.range_deg;
      const here = turretActualDeg;

      // UNWIND BEFORE THE STOP, NOT AFTER IT.
      //
      // The axis has 370 deg of travel for a bearing that lives on a 360 deg circle, so the
      // two wraps of a solution are BOTH reachable only in a 10 deg window either side of
      // +-180. That window is the only moment there is a choice -- and the choice decides the
      // whole of the next revolution. Taking the nearer one, which is what this did, picks
      // the wrap the chassis is about to drive off the end of maybe half the time; from then
      // on the axis is pinned against its stop, tracking nothing, until the robot happens to
      // come back round.
      //
      // Measured over a 12 s spin with tools/turretcheck.ts: commanded past the stop 66% of
      // the time, and the muzzle sat closer to the OPPONENT's hive than to ours 37% of the
      // time, up to 180 deg off. That is the "it aims at the wrong hive" report -- it was
      // never aiming anywhere, it had run out of travel.
      //
      // So at the crossing, look ahead: the required angle drifts at minus the chassis yaw
      // rate, and a wrap that will be outside the travel in `lookahead` seconds is refused
      // while the other one is still on offer. One decision per revolution, taken early,
      // while both options are cheap.
      //
      // THE CANDIDATE LIST HAS TO START FROM A CANONICAL ANGLE. `aimHold` is an accumulator --
      // it integrates a filtered delta and is never wrapped -- so a robot that keeps turning
      // one way walks it off the circle: after two revolutions it is at 700 deg, none of
      // 700 / 340 / 1060 is inside +-185, and the loop below finds no candidate at all. It
      // then fell through to `want = this.aimHold`, which clamps hard against the stop and
      // stays there. THAT is the bug behind "auto-aim is on and it is not aiming" and behind
      // the muzzle sitting on the opponent's side: measured over a 12 s spin, commanded past
      // the stop 66% of the time and pointing nearer the wrong hive 37% of the time.
      //
      // Wrapping to (-180, 180] first guarantees the base candidate is always reachable, so
      // the choice below is a real choice between wraps rather than a fallback.
      const base = wrapPi(this.aimHold * DEG) * RAD;
      const firingNow = st.firing || g.b;
      const drift = -s.localizer.omega;
      const look = this.spec.turret.unwindLookahead_s ?? 1.5;
      let want = base;
      let bestScore = Infinity;
      for (const cand of [base, base - 360, base + 360]) {
        if (cand < lo || cand > hi) continue;
        // SECONDS OF TRACKING THIS WRAP BUYS, before the drift carries it off the end. A
        // straight "does it run out within `look`" test is no use: spin hard enough and every
        // wrap runs out, both get the same answer, and the choice falls back to whichever is
        // nearer -- which is the behaviour being replaced.
        const margin = drift >= 0 ? hi - cand : cand - lo;
        const lasts = Math.abs(drift) > 1 ? margin / Math.abs(drift) : Infinity;
        // Travel is charged at what it actually costs in seconds, so the two are comparable:
        // going the long way round is 360 deg at the axis's own slew rate.
        const cost = Math.abs(cand - here) / Math.max(1, this.spec.turret.speed_dps);
        // UNWIND ON YOUR OWN TIME, NOT ON THE SHOT. A traverse takes 360 deg at the axis's
        // slew rate -- about 1.4 s -- and it has to happen somewhere. Doing it while the
        // driver is lining up is the worst moment: the muzzle sweeps the field, crosses the
        // opponent's hive, and the gate refuses the whole way round. So while nothing is
        // being fired, a wrap that leaves the axis near the middle of its travel is worth
        // real seconds later, and is bought now when it costs nothing.
        const idlePull = firingNow ? 0 : Math.abs(cand) / Math.max(1, hi) * look * 0.5;
        const score = cost - Math.min(look, lasts) + idlePull;
        if (score < bestScore) { bestScore = score; want = cand; }
      }
      turretDeg = clamp(want, lo, hi);
      // HOW FAR PAST THE END STOP THE SHOT WANTED TO BE, which is not the same question as
      // how far the axis is from its command and is the one nothing was asking.
      //
      // `turretErrDeg` below is measured against the CLAMPED command, so an axis pinned on its
      // stop reports a fraction of a degree of error and the readiness gate calls it aimed. A
      // robot turning under the turret runs out of travel constantly -- measured over a
      // spinning run, the lead asked for a bearing 17.7 +- 18.5 deg OUTSIDE +-120, the gate
      // said on-target, and the shots went out up to 50 deg wide: lateral miss -51 +- 65 cm
      // against +-11 standing still, and the single largest error term in the whole harness.
      // `TurretTracker.canReach` on the hub has always tested this; the mirror never did.
      st.turretPastStopDeg = Math.abs(want - turretDeg);
    } else {
      // Rate control, in degrees per second: the axis has its own acceleration limit, so
      // this is a request, not a teleport. 100 deg/s covers the full arc in about 2.4 s.
      const nudge = (g.dpad_right ? 1 : 0) - (g.dpad_left ? 1 : 0);
      st.turretManualDeg = clamp(st.turretManualDeg + nudge * 100 * dt, this.spec.turret.range_deg[0], this.spec.turret.range_deg[1]);
      turretDeg = st.turretManualDeg;
      st.turretPastStopDeg = 0;
    }
    st.turretErrDeg = wrapPi((turretDeg - turretActualDeg) * DEG) * RAD;
    // wrapPi takes the short way round, so this is the pointing error whichever wrap the
    // axis is sitting on -- exactly the quantity the gate should have been using.
    st.turretAimErrDeg = aimRawDeg === null ? st.turretErrDeg : wrapPi((aimRawDeg - turretActualDeg) * DEG) * RAD;
    if (this.spec.turret.enabled) {
      motors.turret = { mode: 'RUN_TO_POSITION', target: Math.round(turretDeg * ticksPerDeg), power: 1 };
    }

    // ---- flywheel with a readiness gate (this is what FlywheelGate does on the hub)
    // The lead also changes how fast the ball has to leave: shooting while closing needs
    // less, shooting while retreating needs more.
    // ONE SPEED, ALL MATCH, when the fixed-speed table is loaded: the lead's rpm correction is
    // exactly the thing that made the wheel chase a moving target, so there is none.
    const leadRpm = flowerLead ? flowerLead.cell.rpm
      : this.hoodTable && !this.hoodTable.isEmpty
      ? this.hoodTable.fixedRpm
      : (st.autoAim && tableSpeed > 0 ? (row.rpm * lead.speed) / tableSpeed : row.rpm) + (rowAhead.rpm - row.rpm);
    // Firing implies spinning. Asking a driver to arm the wheel and then arm the feed is
    // two controls where the game only has one decision: shoot or do not.
    const wheelOn = st.flywheelOn || st.firing;
    st.targetRpm = wheelOn ? clamp(leadRpm, 0, this.spec.flywheel.maxRpm) : 0;
    const f = this.spec.flywheel;
    // FILTER THE TACHOMETER. The hub reports whole encoder counts over a 20 ms window and the
    // flywheel is direct-driven on 28 ticks a rev, so a single reading is quantised to about
    // 107 rpm -- worth 15 in of range. Comparing that against a 30 rpm tolerance is comparing
    // against noise. A moving average over a few loops costs lag and buys resolution, and it
    // is the same handful of lines on the hub as it is here.
    const n = Math.max(1, Math.round(this.spec.flywheel.rpmFilterFrames ?? 1));
    this.rpmHistory.push(s.game.flywheelRpm);
    while (this.rpmHistory.length > n) this.rpmHistory.shift();
    const rpm = this.rpmHistory.reduce((a, b) => a + b, 0) / this.rpmHistory.length;

    // FEEDFORWARD PLUS P ON THE FILTERED SPEED, which is what FlywheelGate.java does.
    //
    // This used to hand the target to the hub's own velocity PIDF (RUN_USING_ENCODER). That
    // is a different robot from the deliverable: the hub's loop is fed by the hub's own
    // velocity estimate, quantised to about 107 rpm on this encoder, and no filtering a team
    // writes can get between the two. The Java has always driven the wheel open-loop from a
    // feedforward with a P trim, which lets the SAME filtered reading serve the controller
    // and the readiness gate. Measured 31.5% against 48% for a smoothed loop, and the mirror
    // disagreeing with the deliverable about something this basic is its own bug.
    //
    // Gains are MEASURED (tools/flywheeltune.ts --ff), not derived from the free speed. The
    // generic kS = 0.03, kV = 1/freeRpm asks for 0.497 duty at 2800 rpm where the wheel needs
    // 0.466, which parks it about 190 rpm high and never opens a 60 rpm window.
    const kV = f.kV ?? 1 / 6000;
    const kS = f.kS ?? 0.03;
    const kP = f.kP ?? 0.00025;
    const volts = s.battery.volts < 6 ? 12 : s.battery.volts;
    const ff = (kS + kV * st.targetRpm) * (12 / volts);
    motors.flywheel = wheelOn && st.targetRpm > 0
      ? { mode: 'RUN_WITHOUT_ENCODER', power: clamp(ff + kP * (st.targetRpm - rpm), -1, 1), brake: false }
      : { mode: 'RUN_WITHOUT_ENCODER', power: 0, brake: false };

    // Same guard as the Java FlywheelGate: a live target moves every loop, so readiness
    // must not be keyed to it changing at all.
    // ---- WILL THIS SHOT LAND? A probability, not a tolerance.
    //
    // The old gate asked "is the wheel within tolRpm of target", which is a proxy with two
    // problems. It is the same window at every range, though a 30 in shot tolerates six
    // times the speed error a 130 in one does; and it says nothing at all about whether a
    // ball that threads the mouth then stays in it, which the measurement says varies from
    // 25% to 96% depending only on how the ball arrives.
    //
    // So compute the thing itself:
    //
    //     P(land) = P(exit speed lands in the threading band) x P(stays in | arrival)
    //
    // The first factor uses the speed the wheel is ACTUALLY doing as the mean, so a wheel
    // that is off-target does not fail a threshold test -- it lowers a probability, by an
    // amount that depends on how forgiving this particular shot is. The second is measured
    // by tools/entrycheck.ts and carried in the table.
    const exitNow = f.k * f.r_fly_m * rpmToRadS(rpm);
    // MEASURED IN THE FRAME THE BAND WAS SOLVED IN. speedLo/speedHi thread the mouth from a
    // STANDING robot at the table's own hood angle; the lead moves both the angle and the
    // speed, so the band moves with it. Comparing the raw reading against a fixed band marked
    // every moving shot as unlikely no matter how well aimed it was. Zero correction at rest.
    const exitRel = exitNow - (lead.speed - tableSpeed);
    const haveModel = row.speedLo !== undefined && row.speedHi !== undefined
      && row.sigmaSpeed !== undefined && row.pStay !== undefined;
    // ---- AND WILL IT BE POINTING THE RIGHT WAY?
    //
    // The two factors above are both about the shot's LENGTH. Neither can see left and
    // right, and left and right is where the shots were going: tools/missmix.ts found 33%
    // of shots at 70 in missing WIDE, a failure mode the probability could not represent at
    // all -- so it happily reported 85% for a shot pointing a degree and a half off.
    //
    // The bearing error at the target is range x tan(aim error), and the launch adds a yaw
    // scatter of its own, so the arrival across the shot line is normal about the current
    // pointing error. That makes it the same integral as the speed band, over the mouth's
    // width instead of its depth.
    //
    // It is range-dependent in a way a fixed "within 3 degrees" gate never was: 3 deg is 2
    // in at 40 in and 4 in at 70 in, against a half-width of 8 in.
    const rangeM = inches(tgt.rangeIn);
    const sigmaLat = rangeM * Math.tan(f.scatter.yaw_deg * DEG);
    const meanLat = rangeM * Math.tan(st.turretAimErrDeg * DEG);
    // THE MOUTH IS NARROWER FROM THE SIDE. `halfLat_m` is the CELL's half-width measured
    // square on; the opening a ball has to fit through is that width SEEN ALONG THE SHOT,
    // which is halfLat*cos(off-axis) -- half of it at 60 deg. The brain was using the
    // square-on figure at every bearing, so a shot from the edge of the sector scored the
    // same pAim as one from straight in front, and the gate passed it.
    //
    // Measured: 37 shots at 1.55 m/s across the front of the hive, every one cleared by the
    // gate, NONE credited, and 36 of them never reached the mouth's height at all. The shots
    // were being taken from the sector's edges -- which is where a robot crossing at speed
    // spends most of its time -- and the model could not see that the hole had shrunk.
    //
    // tools/shotzone.ts has always rebuilt the whole aperture per square, which is why the
    // map and the robot disagreed about the same spot. This is the brain's cheap version of
    // the same geometry: one cosine, no solver call per loop.
    // THE POCKET'S DEPTH IS NOT PART OF THE APERTURE. MEASURED, tools/lostzone.ts.
    //
    // This was width*cos(beta) - depth*sin(beta) for one commit (84525e6), on the argument
    // that a slot seen off its normal has its own depth cutting across the opening, and it
    // cost the shot zone 20 of its 52 green squares. The argument is wrong, and wrong in the
    // expensive direction: that formula is the clear straight line THROUGH a slot, which is
    // what a ball would need if it had to reach the back wall untouched. A ball does not. It
    // has to cross the MOUTH and stay in the pocket, and the depth is behind the mouth --
    // arriving against the far wall is a ball in the CELL.
    //
    // Stood on the 20 squares it deleted and fired 160 balls with the gate forced open: 156
    // in, 98%, against 89% from the squares that stayed green. The worst of them sits 61 deg
    // off the opening -- past the 55 deg where the depth term says there is no hole at all --
    // and lands 88%. tools/obliquity.ts, which justified the change, only ever evaluated the
    // formula; it never fired a ball. So the aperture is the projection, cos(beta), and
    // nothing else.
    const cosB = Math.cos(tgt.openDeg * DEG);
    const halfLatNow = row.halfLat_m === undefined
      ? undefined
      : Math.max(0, row.halfLat_m * cosB);
    st.halfLatNow = halfLatNow ?? -1;
    const pAim = halfLatNow === undefined
      ? 1
      : pThread(-halfLatNow, halfLatNow, meanLat, sigmaLat);
    // FIXED-SPEED MODEL. The uncertainty has moved from the wheel to the hood, so the first
    // factor is a normal integral over HOOD ANGLE instead of exit speed: the band the table
    // measured, against the sigma it measured, centred on where the hood actually IS rather
    // than where it was told to go. The wheel contributes nothing to this term, because it is
    // not moving -- which was the entire point.
    const cell = this.hoodCell;
    const hoodNow = this.hoodActualDeg(s.servos.hood?.pos ?? 0.5);
    const fixedP = cell && wheelOn
      ? pThread(cell.lo, cell.hi, hoodNow, cell.sigmaHood) * cell.pStay * pAim
      : -1;
    // HOW FULL THE POCKET IS, which is the fourth factor and the strongest of the four.
    //
    // Reset on a change of tag ID: that IS the tip, and the tip empties the CELL. Until the
    // camera has ever seen a tag the count stands, because odometry cannot see a tip.
    if (tgt.id > 0 && tgt.id !== this.lastTagId) {
      if (this.lastTagId !== 0) this.cellFill = 0;
      this.lastTagId = tgt.id;
    }
    st.cellFill = this.cellFill;
    const pFill = fillFactor(this.cellFill);
    const rawP = this.hoodTable && !this.hoodTable.isEmpty
      ? fixedP
      : haveModel && wheelOn
        ? pThread(row.speedLo as number, row.speedHi as number, exitRel, row.sigmaSpeed as number) * (row.pStay as number) * pAim * pFill
        : -1;
    // THE THREE FACTORS, KEPT SEPARATELY. The product was the only thing recorded, so when
    // tools/landcal.ts found it had no predictive power there was no way to ask WHICH of the
    // three is the dead one. They are cheap to carry and they are the whole diagnosis.
    st.pSpeed = this.hoodTable && !this.hoodTable.isEmpty
      ? (cell && wheelOn ? pThread(cell.lo, cell.hi, hoodNow, cell.sigmaHood) : -1)
      : (haveModel && wheelOn ? pThread(row.speedLo as number, row.speedHi as number, exitRel, row.sigmaSpeed as number) : -1);
    st.pStayNow = this.hoodTable && !this.hoodTable.isEmpty ? (cell?.pStay ?? -1) : ((row.pStay as number) ?? -1);
    st.pAim = pAim;
    st.pLandRaw = rawP;
    // Calibrated if a measurement is available, raw otherwise -- and `calibrated` says which,
    // so a threshold is never quietly compared against the wrong kind of number.
    st.pLand = rawP < 0 ? -1 : this.landCal ? this.landCal.apply(rawP) : rawP;
    st.calibrated = this.landCal !== null;

    const minP = f.minLandProb ?? 0.9;
    // The probability gate is STRICTLY ON TOP of the old RPM window, never instead of it.
    //
    // Replacing the window outright looked cleaner and had a nasty edge: with the threshold
    // set to 0 -- which is what every mechanism test and every calibration run does, because
    // they need an unfiltered sample -- `pLand >= 0` is always true, so the speed check
    // disappeared completely and the robot would fire part-way through spin-up at whatever
    // RPM it happened to be at. Keeping the window as a floor means a threshold of 0 is
    // exactly the old behaviour, and every value above it is a real added restriction.
    const inWindow = Math.abs(rpm - st.targetRpm) < f.tolRpm && rpm > st.targetRpm * f.minRpmFrac;
    // With the fixed-speed table the readiness question changes: the wheel is always at its
    // one speed, so what has to arrive is the HOOD. A servo settles in tens of milliseconds
    // against the flywheel's tenths of a second, and it is a commanded position rather than a
    // measured speed -- there is nothing in the loop to be wrong about.
    //
    // AND IT IS NOW THE QUESTION FOR THE SPEED-SOLVING TABLE TOO. `hoodThere` used to be
    // hard-wired true whenever the fixed-speed table was absent, from when the hood only ever
    // held the table's stationary angle and arrived long before the wheel did. The motion lead
    // solves the hood now -- that is what makes shooting on the move work at all -- so the
    // hood is the axis carrying the correction, it moves every loop, and nothing was waiting
    // for it. Shots went out mid-slew, at an elevation that belonged to a velocity the robot
    // had already left, and the probability model could not see it happen: there is no hood
    // term in the speed-solving product, so a shot taken 10 deg off still scored 85%.
    //
    // A servo has no measurement uncertainty -- it is commanded, not read -- so this is a
    // readiness question rather than another factor in the probability. Once the hood IS at
    // the solved elevation the ball leaves with the table's launch vector exactly, and the
    // speed band the table measured standing still is valid again.
    const usingHood = !!(this.hoodTable && !this.hoodTable.isEmpty);
    st.hoodErrDeg = usingHood ? (cell ? hoodNow - cell.mid : NaN) : hoodNow - lead.elevationDeg;
    const hoodThere = usingHood
      ? (!!cell && hoodNow >= cell.lo && hoodNow <= cell.hi)
      : Math.abs(st.hoodErrDeg) <= (this.spec.hood.tolDeg ?? 2);
    const haveShot = (!usingHood || !!cell) && inTable;
    const probOk = usingHood ? st.pLand >= minP : !haveModel || st.pLand >= minP;
    // IS THE MOUTH STILL OPEN TOWARDS US? A TIP swaps which CELL is up and the new one opens
    // the other way, so a robot that was square onto the goal is suddenly behind it. Nothing
    // checked this: the stopped control case tipped the hive at t=20 s and then spent 70 s
    // firing 40 more balls into the back of the pocket, all counted as misses, which is most
    // of why it read 0.12 landed per second against tools/landcal.ts's 85%.
    //
    // 75 deg rather than 90: at 90 the mouth is exactly edge-on and its opening has no area
    // at all, so the last few degrees are shots that cannot geometrically enter.
    // HOW FAR OFF THE OPENING IS STILL A SHOT. The mouth is a slot, so what a ball has to fit
    // through is the opening seen edge-on: 14 in of depth becomes 14*cos(off-axis), and at
    // 62 deg that is 6.6 in for a 2.8 in ball. 75 was a geometric guess -- at 90 the aperture
    // has no area at all, so anything under it "can" enter -- and it is far too generous.
    // Swept, it costs shots at both ends: see fireOpenCap_deg in config/robot.json.
    const openCap = this.spec.turret.fireOpenCap_deg ?? 75;
    const mouthOpen = tgt.openDeg <= openCap;
    // IS THE OTHER HIVE IN THE WAY? The two rockers are 25.5 in apart and 20 in wide, so a
    // shot taken from the far side of theirs crosses their structure -- and the solver never
    // knew, because it integrates a ball through empty air and checks only the target's own
    // lips. The physics gives both rockers colliders, so the ball really does hit.
    const mouthX = this.pose.x + tgt.rangeIn * Math.cos((this.pose.heading + tgt.azimuthDeg) * DEG);
    const mouthY = this.pose.y + tgt.rangeIn * Math.sin((this.pose.heading + tgt.azimuthDeg) * DEG);
    const blocked = tgt.valid && shotIsBlocked(this.pose.x, this.pose.y, mouthX, mouthY, this.obstacle);
    st.blocked = blocked;

    // SPINNING FASTER THAN THE TURRET CAN FOLLOW.
    //
    // The feed commits about leadLatency_s + the pulse before the ball is gone, and a chassis
    // yawing at w drags the turret's setpoint at w for all of it. The axis slews at 261 deg/s
    // and the gate lets a shot through at 3 deg of error, so the error at RELEASE is w times
    // that delay -- at 100 deg/s of spin the setpoint moves 40 deg while the ball is on its
    // way to the nip, and the turret is nowhere near it. Measured spinning on the spot: 75% in
    // with 6 wild of 24, downrange 49 +-86 cm, against 100% for every other driving case.
    //
    // STRATEGY.md 7.2 caps deliberate turning while firing for this reason. Enforcing it turns
    // a wild shot into a held one, which costs a cycle and saves a ball. The readiness check
    // for turret ERROR cannot do this on its own: the axis is on target when the shot is
    // committed and behind by the time it leaves.
    const yawCap = this.spec.turret.fireYawCap_dps ?? Infinity;
    const yawOk = Math.abs(s.localizer.omega) <= yawCap;

    // AN IMPOSSIBLE AIM IS NOT A SHOT. leadShot clamps the solved elevation into the hood's
    // travel and degrades smoothly, which is the right thing to do and completely silent --
    // so a robot charging the goal at speed from close in fired a launch vector the hood
    // could not produce and the ball went wherever that put it. Half a degree of clamp is
    // noise; past a degree the vertical no longer belongs to the shot.
    // Half a degree of hood clamp is noise; past a degree the vertical no longer belongs to
    // the shot. Any outrun at all is fatal -- there is no azimuth, not a degraded one.
    // HOW MUCH OF THE SHOT IS THE ROBOT'S OWN MOTION.
    //
    // The lead is exact on paper and every term feeding it checks out -- the hood arrives, the
    // turret arrives, the velocity estimate is good to 0.05 m/s, nothing is clamped and the
    // robot is not outrunning the ball. And past about twenty degrees of lead the shots go
    // out anyway, a metre long and a metre wide, while a seventeen-degree lead lands 100%.
    //
    // A large lead means the ball's ground track is mostly the CHASSIS and only a little the
    // launch, so every small error in the chassis velocity is multiplied into the shot instead
    // of added to it. Rather than pretend to know which term dominates, this is set from the
    // measurement: sweep it, take the angle where the shots stop landing. tools/movingtune.ts
    // --lead does the sweep and prints the table it came from.
    const leadCap = this.spec.turret.fireLeadCap_deg ?? Infinity;
    const leadOk = Math.abs(st.leadDeg) <= leadCap;
    const aimPossible = lead.clamped <= 1.0 && lead.outrun <= 0.02;
    st.aimClampedDeg = lead.clamped;
    st.aimOutrun = lead.outrun;
    st.leadSpeed = lead.speed;
    st.leadElevDeg = lead.elevationDeg;
    // The radial velocity the LEAD actually worked from, after filtering and the
    // release-time prediction. Not the same as the raw localizer reading.
    st.leadVRadial = mv.vx * Math.cos(bearingField) + mv.vy * Math.sin(bearingField);

    // ACCELERATING OUT OF THE BAND BEFORE THE BALL LEAVES.
    //
    // The lead cancels the velocity the robot HAS. The feed commits about `leadLatency_s`
    // before the ball is actually gone, and in that time an accelerating robot grows a
    // velocity the shot was never solved for. Velocity itself is free -- a steady 1.5 m/s
    // holds the required exit speed perfectly still -- so this is an ACCELERATION budget,
    // and docs/DECISIONS.md names the wobble case as the one that spends it: it fires more
    // than any other case, with the LOWEST rpm error at release, and lands least. The wheel
    // really is on its target; the target is wrong by the time the ball goes.
    //
    // THE BUDGET COMES FROM THE TABLE, not from a tuned constant. speedLo..speedHi is the
    // exit-speed band that threads the mouth at this range, so half its width is the speed
    // error this particular shot tolerates -- six times wider at 30 in than at 130. A change
    // dv in radial velocity moves the required exit speed by about cos(elevation)*dv, so the
    // shot is refused when what the robot will have grown by release is worth more than the
    // band can absorb. Nothing here needs re-tuning when the shooter changes.
    const tauFeed = this.spec.transfer.leadLatency_s ?? 0;
    const aRad = Math.hypot(this.accel.x, this.accel.y);
    const dSpeedByRelease = Math.abs(Math.cos(lead.elevationDeg * DEG)) * aRad * tauFeed;
    const halfBand = row.speedLo !== undefined && row.speedHi !== undefined
      ? (row.speedHi - row.speedLo) / 2
      : Infinity;
    const accelOk = dSpeedByRelease <= halfBand;
    st.accelBudget = Number.isFinite(halfBand) && halfBand > 0 ? dSpeedByRelease / halfBand : 0;
    const atSpeed = wheelOn && st.targetRpm > 0 && inWindow && probOk && hoodThere && haveShot;
    // LEAKY, NOT A HARD RESET. One bad loop used to throw away the whole settle and start
    // again from zero, so a tachometer that dips out of its window for a single frame cost
    // three more -- and at speed the target rpm is moving, so it dips often. Decrementing
    // instead means a flicker costs one loop and a genuine loss of readiness still walks the
    // counter down to zero in three.
    st.readyCount = atSpeed ? st.readyCount + 1 : Math.max(0, st.readyCount - 1);
    // A FLOWER LOB IS A DIFFERENT SHOT AND A DIFFERENT GATE. None of the CELL's conditions
    // apply to it -- there is no mouth to be square to, no tipping rocker, no motion lead
    // worth the name at 1.1 m/s of exit speed -- and two of its own do: the stand-off has to
    // be inside the solved band, and the hood has to have arrived, because at 0.6 in of
    // tolerance the hood IS the shot.
    if (flowerLead) {
      const hoodOk = Math.abs(hoodNow - flowerLead.cell.hoodDeg) <= (this.spec.hood.tolDeg ?? 2);
      // ITS OWN SETTLE COUNTER. The CELL path increments st.readyCount further down, which
      // this branch returns before reaching -- so the count never grew, ready stayed false
      // for ever, and the first version lobbed nothing while reporting the wheel on speed,
      // the hood on angle and the hold string empty. A gate that reads clear and fires
      // nothing is the worst of both.
      const flowerOk = inWindow && hoodOk && Math.abs(st.turretAimErrDeg) < 3
        && st.turretPastStopDeg < 0.5 && yawOk;
      // A COUNTER OF ITS OWN. Sharing st.readyCount does not work: the CELL path zeroes it a
      // few lines above, on ITS conditions, which are never met in FLOWER mode -- so the
      // count went 0 -> 1 -> 0 -> 1 for ever and readySteps was never reached. Every gate
      // term read true and the robot lobbed nothing, which took a per-frame dump to see.
      this.flowerSettled = flowerOk ? this.flowerSettled + 1 : 0;
      st.readyCount = this.flowerSettled;
      st.flowerWhy = `win=${inWindow ? 1 : 0} hood=${hoodOk ? 1 : 0} aim=${Math.abs(st.turretAimErrDeg) < 3 ? 1 : 0} stop=${st.turretPastStopDeg < 0.5 ? 1 : 0} yaw=${yawOk ? 1 : 0}`;
      st.ready = this.flowerSettled >= f.readySteps && flowerOk;
      st.hold = !wheelOn ? ''
        : !st.flower?.cell ? `no FLOWER within ${this.flowerTable?.maxRange.toFixed(0) ?? 0} in - DRIVE UP TO ONE`
        : !inWindow ? `wheel ${rpm.toFixed(0)}/${st.targetRpm.toFixed(0)} rpm`
        : !hoodOk ? `hood ${hoodNow.toFixed(0)} deg, the lob wants ${flowerLead.cell.hoodDeg.toFixed(0)}`
        : !yawOk ? `turning too fast to aim: ${Math.abs(s.localizer.omega).toFixed(0)} deg/s`
        : Math.abs(st.turretAimErrDeg) >= 3 ? `turret ${st.turretAimErrDeg.toFixed(0)} deg off`
      // THE WHEEL IS NOT ON SPEED YET, and until this branch existed nothing said so.
      // `inWindow` gates firing through `atSpeed` but had no line in this chain, so a robot
      // whose flywheel was chasing a moving target reported "clear" and did not shoot --
      // 19% of a strafing pass at 50 in (tools/frontcheck.ts). FLOWER mode has always
      // printed it, and so has AimController.java ("spinning up", in this same position in
      // the chain); the TS mirror was the one that did not.
      : !inWindow ? `wheel ${rpm.toFixed(0)}/${st.targetRpm.toFixed(0)} rpm - spinning up`
        : '';
      const ftp = this.spec.transfer;
      // THE BELT HAS TO RUN. This branch returns before the CELL path sets motors.transfer,
      // so the first version lobbed nothing at all while reporting "clear to fire": the
      // tube stayed empty, no ball ever reached the nip, and the gate had nothing to let
      // through. Same belt, same reason as the CELL shot -- it keeps the feed loaded.
      motors.transfer = { mode: 'RUN_WITHOUT_ENCODER', power: wheelOn ? 1 : 0 };
      const mayFeed = st.ready && s.t - st.lastFeedT >= ftp.cycleTime_s;
      if (mayFeed) { st.lastFeedT = s.t; st.pulsing = true; }
      if (st.pulsing && s.t - st.lastFeedT >= ftp.feedPulse_s) st.pulsing = false;
      return {
        seq, motors,
        servos: {
          hood: this.hoodCommand(flowerLead.cell.hoodDeg),
          gate: st.pulsing && st.ready ? this.spec.transfer.gate.open : this.spec.transfer.gate.closed,
        },
        telemetry: [['mode', 'FLOWER'], ['tube', String(st.flower?.index ?? -1)],
          ['range in', (st.flower?.range_in ?? 0).toFixed(1)],
          ['hood deg', flowerLead.cell.hoodDeg.toFixed(1)], ['rpm', flowerLead.cell.rpm.toFixed(0)],
          ['hold', st.hold || 'clear']],
      };
    }
    // HOW CLOSE THE AIM HAS TO BE BEFORE A SHOT IS ALLOWED. Was a hardcoded 3 degrees, which
    // is loose: measured with an otherwise perfect robot, the 0.13 s the gate servo takes to
    // open was quietly doing the fine settling this gate never demanded, and shortening the
    // servo dropped the land rate from 98% to 78% (tools/releasecheck.ts --gatespeed). A
    // criterion the shot actually depends on belongs in the gate, not in a servo's travel time.
    const aimTol = this.spec.turret.fireAimTolDeg ?? 3;
    st.ready = st.readyCount >= f.readySteps && Math.abs(st.turretAimErrDeg) < aimTol
      && st.turretPastStopDeg < 0.5 && tgt.fresh && !blocked && mouthOpen && accelOk && yawOk && aimPossible && leadOk;

    st.hold = !wheelOn ? ''
      : tgt.scanning ? `searching for the tag (turret sweeping ${tgt.azimuthDeg.toFixed(0)} deg)`
      : !tgt.fresh ? `tag fix ${(tgt.ageS * 1000).toFixed(0)} ms old - holding`
      : blocked ? 'the other HIVE is in the way - DRIVE ROUND IT'
      : !mouthOpen ? `${tgt.openDeg.toFixed(0)} deg off the opening, cap ${openCap.toFixed(0)} - DRIVE ROUND`
      : !inTable ? `${tgt.rangeIn.toFixed(0)} in is outside the table (${this.table.rows[0]?.range_in ?? 0}-${this.table.rows[this.table.rows.length - 1]?.range_in ?? 0}) - BACK OFF`
      : !accelOk ? `accelerating out of the band: ${(st.accelBudget * 100).toFixed(0)}% of it before the ball leaves`
      : !yawOk ? `turning too fast to aim: ${Math.abs(s.localizer.omega).toFixed(0)} deg/s, cap ${yawCap.toFixed(0)}`
      : !leadOk ? `too much of this shot is the robot's own motion: ${Math.abs(st.leadDeg).toFixed(0)} deg of lead, cap ${leadCap.toFixed(0)} - SLOW DOWN`
      : lead.outrun > 0.02 ? `moving sideways faster than the ball flies: ${lead.outrun.toFixed(2)} m/s over - SLOW DOWN or back off`
      : !aimPossible ? `no launch fits: the hood is ${lead.clamped.toFixed(0)} deg short of the shot this motion needs`
      : st.turretPastStopDeg >= 0.5 ? `turret cannot reach, ${st.turretPastStopDeg.toFixed(0)} deg past its stop`
      : Math.abs(st.turretAimErrDeg) >= aimTol ? `turret ${st.turretAimErrDeg.toFixed(1)} deg off, tol ${aimTol}`
      // No cell is a real answer, not a failure: there is no hood angle that scores from here
      // at this closing speed, and saying so beats holding with an unexplained low number.
      : usingHood && !cell ? 'no shot from here at this speed'
      : !hoodThere ? `hood ${hoodNow.toFixed(0)} deg, want ${(usingHood ? cell?.mid : lead.elevationDeg)?.toFixed(0)}`
      : !haveModel && !usingHood ? ''
      // A threshold ABOVE THE MEASURED CEILING cannot be met by any shot this shooter can
      // take, so the robot sits there forever printing a number that reads like bad luck.
      // Name it, or the honest answer (the shooter cannot do it) looks like a jam.
      : st.pLand < minP ? `P(land) ${(st.pLand * 100).toFixed(0)}% < ${(minP * 100).toFixed(0)}%`
        + (this.landCal && minP > this.landCal.ceiling
          ? ` - UNREACHABLE, best measured ${(this.landCal.ceiling * 100).toFixed(0)}%`
          : '')
      // Everything is right and has not been right for long enough yet. Also never reported.
      : st.readyCount < f.readySteps ? `settling ${st.readyCount}/${f.readySteps}`
      : '';

    // ---- feed
    // The belt runs the whole time the shooter is armed, so the feed tube stays loaded
    // against the gate; the GATE is the release. Gating the belt instead emptied the tube
    // back into the hopper after every shot and cost a second per cycle re-lifting the
    // same ball.
    const wantFire = st.firing || g.b;
    // METER THE BALLS, the way Transfer.java does: one pulse per shot, and no pulse until
    // cycleTime has passed since the last one.
    //
    // This used to hold the gate open for as long as the robot was ready, and relied -- by
    // accident -- on readiness FLICKERING to break the stream into single balls. Once the
    // tachometer was filtered, readiness stopped flickering and the magazine emptied itself
    // through the open gate: four balls in, one shot, three gone. The deliverable never had
    // this bug, which is the point of the mirror agreeing with it.
    const tp = this.spec.transfer;
    const canFeed = s.t - st.lastFeedT >= tp.cycleTime_s && !st.pulsing;
    if (wantFire && st.ready && canFeed) {
      st.pulsing = true;
      st.lastFeedT = s.t;
      // COUNT IT OUT, by its own odds rather than as a whole ball: the robot cannot see
      // whether it went in, and adding 1.0 for a shot with a 60% chance would have the
      // pocket full long before it is.
      this.cellFill += Math.max(0, st.pLand);
    }
    if (st.pulsing && s.t - st.lastFeedT >= tp.feedPulse_s) st.pulsing = false;
    // THE GATE STAYS OPEN ONLY WHILE THE SHOT IS STILL GOOD.
    //
    // The decision to feed is taken about four tenths of a second before the ball actually
    // leaves -- the feed pulse plus the climb up the tube -- and it used to be final: the
    // pulse opened the gate for its 0.25 s whatever happened next. Every axis keeps tracking
    // in the meantime, so the AIM at release is current; what is stale is the PERMISSION.
    //
    // It shows up as a small tail of badly wrong shots rather than as a loss of precision.
    // Shuttling fore and aft at 0.5 Hz the typical shot is fine -- median 1 cm off line, IQR
    // [-10, +10] cm downrange, better than standing still -- while 20 of 108 landed more than
    // 60 cm out. So the release re-checks the two things that can go bad inside those four
    // tenths and that are what make a shot WILD rather than merely imprecise: the wheel
    // sagging under its floor, and the turret running out of travel under a turning chassis.
    //
    // NOT the full readiness latch. That was the first attempt and it is far too strict: it
    // needs three consecutive good loops, the tachometer flickers in and out of a 60 rpm
    // window, and requiring an unbroken 0.19 s while the gate servo travels took the stopped
    // case from 78 shots to 3. These three are smooth over the pulse and do not flicker.
    //
    // The turret's own tracking error belongs here for the same reason as the end stop: a
    // chassis spinning under the turret drags it off target during the four tenths, and every
    // wild shot left in the turning case was the same picture -- 134 deg/s of yaw with the
    // axis 10.8 deg behind, approved when it was still on target and fired when it was not.
    //
    // THE WHEEL IS NOT RE-CHECKED HERE, and two attempts to do it both made things worse. A
    // floor -- `rpm > target * minRpmFrac` -- is the wrong shape, because a robot whose range
    // is growing has a target rising ahead of the wheel the whole way, so the floor is
    // permanently unmet: it took the strafing case from 0.28 landed per second to zero, on a
    // run where every shot it used to take went in. Re-checking P(land) instead is the right
    // shape and the wrong input, since it rides the tachometer's 107 rpm quantisation and
    // flickers: stopped fell from 0.36 to 0.12 and the shuttle to zero.
    //
    // It does not need re-checking anyway. The shots that used to go out on a sagging wheel
    // were the bare grip wheel's doing -- one ball took 210 rpm out of it -- and putting a
    // real flywheel behind the wheel cut that to 70 and took the wild shots with it. The two
    // that remain here are geometric, smooth over the pulse, and cannot be fixed by a part.
    // RE-CHECK THE PERMISSION, NOT JUST THE AIM. The feed commits about 0.4 s before the
    // ball clears the nip, and the aim keeps tracking for all of it -- so what goes stale is
    // whether the shot was ever allowed, which is PHYSICS_AND_SIMULATION.md section 3.4's
    // point exactly. At 1.55 m/s the robot covers 62 cm in that window: the range grows by
    // about 20 in and the bearing swings 18 deg off the opening.
    //
    // Measured, crossing the front of the hive at full stick: 37 shots, every one cleared at
    // commit, NONE credited, 36 never reaching the mouth's height. Their release state was
    // 81-83 in at 62-63 deg off the opening with P(land) 0.12-0.38 -- all three past their
    // gates by the time the ball actually went. The wheel floor deliberately stays out of
    // this (the doc's other half): the range is growing, so the rpm target moves under the
    // shot and a floor there refuses everything while changing nothing.
    const stillGood = st.turretPastStopDeg < 0.5 && Math.abs(st.turretAimErrDeg) < aimTol
      && mouthOpen && probOk && haveShot;
    const mayFire = st.pulsing && stillGood;
    motors.transfer = { mode: 'RUN_WITHOUT_ENCODER', power: wheelOn ? 1 : 0 };

    st.note = !wheelOn
      ? 'shooter idle'
      : !atSpeed
        ? `spinning up ${rpm.toFixed(0)}/${st.targetRpm.toFixed(0)}`
        : !tgt.fresh
          ? (tgt.scanning ? 'searching for the tag' : `tag fix ${(tgt.ageS * 1000).toFixed(0)} ms old`)
          : Math.abs(st.turretAimErrDeg) >= 3
            ? `turret slewing ${st.turretAimErrDeg.toFixed(0)} deg`
            : s.game.hopper === 0
              ? 'hopper empty'
              : st.firing
                ? 'FIRING'
                : 'ready to fire';

    return {
      seq,
      motors,
      servos: {
        // FLOWER mode returned above, so this is always the CELL shot.
        hood: st.autoAim ? this.hoodCommand(lead.elevationDeg) : 0.5,
        gate: mayFire ? this.spec.transfer.gate.open : this.spec.transfer.gate.closed,
      },
      telemetry: [
        // Metres for the reader; the hub's own sensor stays in the FTC frame's inches.
        // The fix and how old it is, first: every other number on this list is derived from
        // it, and when it is stale they are all describing where the goal used to be.
        ['tag', tgt.scanning ? 'SEARCHING' : `CELL ${tgt.id === 2 ? 'B' : 'A'}, ${(tgt.ageS * 1000).toFixed(0)} ms old`],
        ['range', `${(tgt.rangeIn * 0.0254).toFixed(2)} m`],
        ['target rpm', st.targetRpm.toFixed(0)],
        ['rpm', rpm.toFixed(0)],
        ['ready', String(st.ready)],
        ['P(land)', st.pLand < 0 ? 'no model' : `${(st.pLand * 100).toFixed(0)}%${st.calibrated ? '' : ' (UNCALIBRATED)'}`],
        ['holding', st.hold || '-'],
        ['hopper', String(s.game.hopper)],
        ['margin', `${(row.margin * 100).toFixed(1)}%`],
        ['lead deg', st.leadDeg.toFixed(1)],
        ['turret err', st.turretErrDeg.toFixed(1)],
        ['aim err', st.turretAimErrDeg.toFixed(1)],
        ['past stop', st.turretPastStopDeg.toFixed(1)],
        ['hood err', Number.isFinite(st.hoodErrDeg) ? st.hoodErrDeg.toFixed(1) : '-'],
      ],
    };
  }
}

