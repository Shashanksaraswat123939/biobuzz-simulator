/** Shared types for the world. No DOM, no engine handles. */

export type Vec3 = [number, number, number];
export type Alliance = 'red' | 'blue';
export type BallKind = 'pollen' | 'nectarRed' | 'nectarBlue';
export type Period = 'STAGING' | 'AUTO' | 'TRANSITION' | 'TELEOP' | 'FINISHED';

// ---------------------------------------------------------------- config

export interface Params {
  sim: { dt: number; substepsPerFrame: number; solverIterations: number; ccdOnBalls: boolean; seed: number; lockstep: boolean };
  env: { rho: number; g: number; tileSize_m: number; tileThick_m: number; fieldInside_m: number; wallGlassTop_m: number; railTop_m: number; tileMu: number };
  ball: {
    pollen: { d_m: number; m_kg: number; dVar: number; mVar: number };
    nectar: { d_m: number; m_kg: number; dVar: number; mVar: number };
    Cd: number; clSlope: number; clSMax: number; spinDecay: number;
    e_foam: number; e_poly: number; e_ball: number; rollMu: number; mu: number;
  };
  hive: {
    /** Aim this fraction of the pocket's depth INSIDE the mouth. 0 aims at the lip. */
    aimDepthFrac?: number;
    /**
     * How much of a ball radius the solver keeps clear of each lip when it decides a
     * trajectory threads the mouth. 1 is the geometric answer (the ball's centre passes a
     * full radius inside each lip); the pocket as Rapier enforces it is more forgiving,
     * because a ball that grazes a lip is deflected inward, not out. MEASURED by
     * tools/bandcheck.ts. Scales the solved speed band and therefore the threading factor.
     */
    lipClearanceFrac?: number;
    massKg: number; cgOffset_m: Vec3; Ipivot_kgm2: number;
    pivotY_m: number; redX_m: number; blueX_m: number;
    restAngles_deg: [number, number]; frictionTorque_Nm: number;
    damperC_Nms: number; damperEngage_deg: number; limitRestitution: number;
    ballsToTipOverride: number | null;
  };
  battery: { capacity_Ah: number; Rint_ohm: number; VocBySoC: [number, number][] };
  match: { auto_s: number; transition_s: number; teleop_s: number; flowerUnlock_s: number };
}

export interface MotorSpec { variant: string; gearRatio: number; reversed?: boolean; ticksPerDeg?: number }

export interface RobotSpec {
  name: string; version: number;
  chassis: { length_m: number; width_m: number; height_m: number; mass_kg: number; Izz_kgm2: number | null; cgOffset_m: Vec3; clearance_m: number };
  drivetrain: {
    type: string; wheelRadius_m: number; wheelbase_m: number; track_m: number;
    rollerAngle_deg: number; mu: number; eta: number; rollingRes_N: number;
    /**
     * Governor on commanded ground speed, m/s. Undefined or <= 0 means no cap.
     *
     * NOT the drive gear. The gear (Y/A) is a POWER fraction and power is not speed: half
     * power against rolling resistance is not half the ground speed, and it changes with the
     * battery. This is the limit in the units a driver and a shot table both think in.
     */
    maxSpeed_mps?: number;
    /**
     * What the drivetrain does at full stick with nothing in the way, m/s. MEASURED.
     *
     * Only the feed-forward half of the cap needs it: without it the governor can only react
     * to a robot that is ALREADY too fast, and a limiter that works by hauling the robot back
     * overshot its 0.8 m/s setting by 0.49 (tools/topspeed.ts).
     */
    freeSpeed_mps?: number;
    motors: Record<'fl' | 'fr' | 'bl' | 'br', MotorSpec>;
  };
  intake: {
    motor: MotorSpec;
    /** The opening at the front of the chassis. It is a real hole in the shell. */
    mouth: { width_m: number; height_m: number; depth_m: number };
    captureThreshold: number; captureDelay_s: number; length_m: number; speed_mps: number;
    /** Roller radius, its grip on a ball, and how hard it is preloaded onto one. */
    rollerRadius_m: number; rollerMu: number; squeeze_N: number;
  };
  hopper: { capacity: number };
  /** Shot policy that is not the table's and not the flywheel's. */
  shot?: {
    /**
     * Refuse shots closer than this, inches. 0 means only the table's own lower end applies.
     * Close in the only arc that fits is a steep lob that bounces back out, so a shot exists
     * there and is not worth taking -- see the note in BuiltinTeleOp.
     */
    minRange_in?: number;
    /**
     * Half-width of the patrol sector about the CELL's opening, degrees. A DRIVING guide, not a
     * gate: `turret.fireOpenCap_deg` decides whether a shot is allowed, this decides where the
     * robot should be. Drawn on the floor by the Patrol zone overlay.
     */
    patrolHalfAngle_deg?: number;
  };
  transfer: { motor: MotorSpec;
    /**
     * Seconds between the lead being computed and the ball actually leaving the muzzle.
     * The shot is set up for the velocity the robot has when the gate is commanded, and the
     * ball leaves after the feed pulse and the wheel's own lag -- while ACCELERATING those
     * are different velocities, and the shot carries a correction the robot has grown out of.
     * 0 is the old behaviour: compensate velocity only.
     */
    leadLatency_s?: number;
    /** Inside width of the feed tube, m. Must pass a NECTAR and refuse two POLLEN on the diagonal. */
    boreSize_m?: number;
    cycleTime_s: number; feedPulse_s: number; feedTransit_s: number;
    /** Gate servo travel rate, position units per second. Sets the commit-to-release delay. */
    gateSpeed?: number;
    /** Fraction of the indexer's push that acts UPWARD, as a real indexer wheel does. */
    indexLift?: number;
    /** Feed belt drive pulley radius: belt speed is motor omega times this. */
    beltRadius_m: number; gate: { enabled: boolean; servo: string; open: number; closed: number } };
  turret: { enabled: boolean; type: 'motor' | 'servo'; motor: MotorSpec; range_deg: [number, number];
    speed_dps: number; accel_dps2: number; muzzleOffset_m: number; muzzleHeight_m: number;
    /** Hold the command still while the solution is inside this, so the axis locks instead of creeping. */
    aimDeadband_deg?: number;
    /** One-pole filter on the commanded bearing. 1 is straight through, which is what hunted. */
    aimFilterAlpha?: number;
    /** Hold fire above this chassis yaw rate: the setpoint outruns the axis. deg/s. */
    fireYawCap_dps?: number;
    /** Hold fire when the motion lead exceeds this: the shot is mostly chassis, not launch. */
    fireLeadCap_deg?: number;
    /** How close the aim must be before a shot is allowed, degrees. Default 3. */
    fireAimTolDeg?: number;
    /** Gate on the UNFILTERED aim solution rather than the filtered estimate. Clean signals only. */
    gateOnRawAim?: boolean;
    /** Carry the one-frame-stale encoder reading forward by its reported rate before gating. */
    predictEncoder?: boolean;
    /** Hold fire this far off the mouth's opening: the aperture closes with the cosine. */
    fireOpenCap_deg?: number;
    /**
     * Seconds ahead the wrap choice looks when the bearing crosses the axis's dead angle.
     * A wrap the chassis's current rotation will drive off the end within this long is
     * refused while the other one is still reachable.
     */
    unwindLookahead_s?: number };
  hood: { enabled: boolean; servo: string; angleRange_deg: [number, number]; fixedAngle_deg: number; speed_dps: number;
    /** How close the hood must be to the angle the shot needs before firing, degrees. */
    tolDeg?: number };
  flywheel: { type: 'single' | 'dual'; motor: MotorSpec;
    /** Ticks per WHEEL revolution the brain reads. Defaults to the motor's own encoder. */
    encoderTicksPerRev?: number;
    /** Seconds ahead to look the shot table up: the range the ball will LEAVE at. */
    rangeLead_s?: number;
    I_fly_kgm2: number; r_fly_m: number; k: number; lossFactor: number; minRpmFrac: number; readySteps: number;
    /** Do not fire unless P(land) is at least this. Replaces tolRpm as the speed gate. */
    minLandProb?: number;
    /**
     * Loops of moving average over the hub's flywheel velocity before the gate uses it.
     * 1 is the raw reading. The hub reports counts over a 20 ms window and the flywheel runs
     * 28 ticks a rev, so one count is about 107 rpm -- averaging is how a team gets a number
     * worth comparing against a tolerance, at the cost of lag.
     */
    /**
     * How many motors drive the flywheel. Torque scales with it and so does the rotor inertia
     * they add, which is what decides whether the wheel can TRACK a shot-on-the-move lead.
     */
    motorCount?: number;
    rpmFilterFrames?: number;
    /** Open-loop feedforward, MEASURED: duty = kS + kV * rpm, with kP trimming the error. */
    kS?: number; kV?: number; kP?: number;
    tolRpm: number; maxRpm: number; dragQuad_Nms2: number; coulomb_Nm: number; scatter: { angle_deg: number; yaw_deg: number; speedFrac: number } };
  /**
   * Field trims against the shot table. A real team adjusts these between matches rather
   * than regenerating a table, and the Analysis tab computes them from a collected run.
   */
  calibration: { rangeTrim_in: number; turretTrim_deg: number };
  sensors: {
    /**
     * The AprilTag camera. Turret-mounted, so its boresight IS the turret's bearing.
     * Optional: a robot without one falls back to the localizer for aiming, which is the
     * behaviour every version before this had.
     */
    camera?: {
      mount: 'turret' | 'chassis';
      widthPx: number; heightPx: number; hfov_deg: number;
      cornerNoise_px: number; tagSize_in: number;
      minTagPx: number; maxObliquity_deg: number; latencyMs: number;
    };
    imu: { latencyMs: number }; distance: { name: string; mount_m: Vec3; dir: Vec3; unit: string; max_m: number }[]; localizer: { source: string; noise: {
    xy_in: number; heading_deg: number;
    /** One sigma on the REPORTED velocity, m/s. The motion lead is built on this number. */
    vel_mps?: number;
    /** One sigma on the reported yaw rate, deg/s. */
    omegaDps?: number;
  };
    /** One-pole alpha the brain filters the reported velocity with before aiming on it. */
    velFilterAlpha?: number;
    /**
     * Integrated dead-reckoning error. Absent or disabled falls back to truth + white noise,
     * which is a GPS rather than odometry and makes the tag look optional.
     */
    drift?: {
      enabled?: boolean;
      /** One sigma on the distance scale, drawn once per run. */
      scaleErr: number;
      /** One sigma on the constant gyro bias, deg/s, drawn once per run. */
      gyroBias_dps: number;
      /** Random walk on that bias, deg/s per root second. */
      gyroWalk_dps: number;
      /** White noise on the yaw rate, deg/s. */
      gyroNoise_dps: number;
      /** Slip as a fraction of each step, both axes. */
      slipFrac: number;
    };
    /** Tag fixes correcting the dead-reckoned pose. See packages/core/src/robot/poseFuser.ts. */
    fuse?: { gain: number; headingGain: number; rejectOver_in: number } };
    /** The tag pipeline. Absent means the old oracle, which is not a thing a robot has. */
    tag: TagCameraSpec & {
      target: {
        holdS: number;
        maxFireAgeS: number;
        scanRateDps: number;
        /** May a shot be taken on odometry alone, with no tag in view? */
        fireOnOdometry?: boolean;
      /** How long a decoded rocker state may be trusted without seeing a tag again, s. */
      maxStateAgeS?: number;
        /** One-pole on the HIVE pivot the robot tracks for itself. */
        anchorAlpha?: number;
        /** How much of each new detection to take into the target estimate. */
        measAlpha?: number;
      };
    } };
  hardware: Record<string, string>;
  hub: { loopPeriodMs: number; bulkCacheMode: string; imuLatencyMs: number; encoderVelocityWindowMs: number; commandLatencyMs: number; velocityPid: { p: number; i: number; d: number; f: number; settleMs: number }; voltageNoise_V: number };
  limits: { startingCube_in: number; expansion_in: [number, number, number]; maxMotors: number; maxServos: number };
}

// ---------------------------------------------------------------- bridge

export type RunMode = 'RUN_WITHOUT_ENCODER' | 'RUN_USING_ENCODER' | 'RUN_TO_POSITION' | 'STOP_AND_RESET_ENCODER';

export interface MotorCmd {
  mode: RunMode;
  power?: number;
  velocity?: number;
  target?: number;
  brake?: boolean;
  /**
   * Sticky "zero the encoder" flag. An ActuatorFrame carries only the final state of each
   * motor for the frame, so a STOP_AND_RESET_ENCODER immediately followed by another
   * setMode (the usual idiom) would otherwise never reach the world at all.
   */
  reset?: boolean;
}
export interface MotorState { pos: number; vel: number; amps: number }

export interface ActuatorFrame {
  type?: 'actuator';
  seq: number;
  motors: Record<string, MotorCmd>;
  servos: Record<string, number>;
  telemetry?: [string, string][];
  log?: string[];
}

export interface GamepadState {
  left_stick_x: number; left_stick_y: number; right_stick_x: number; right_stick_y: number;
  left_trigger: number; right_trigger: number;
  a: boolean; b: boolean; x: boolean; y: boolean;
  dpad_up: boolean; dpad_down: boolean; dpad_left: boolean; dpad_right: boolean;
  left_bumper: boolean; right_bumper: boolean;
  start: boolean; back: boolean;
  left_stick_button: boolean; right_stick_button: boolean;
}

/** What the tag pipeline hands over: the shape of one `AprilTagDetection`, nothing more. */
export interface TagSighting {
  /** 1 = CELL A, 2 = CELL B. A TIP swaps which is up, so the tag in view changes with it. */
  id: number;
  /** Bearing to the tag relative to the ROBOT's heading, degrees CCW. */
  bearingDeg: number;
  rangeIn: number;
  /** How far off square the tag's face is, degrees. 0 is dead on, 90 is edge-on. */
  openDeg: number;
  /** World time the photons left -- NOT the time this was read. The consumer ages it. */
  sampleT: number;
}

/** A tag camera, as the pipeline behaves rather than as the geometry would like. */
export interface TagCameraSpec {
  enabled: boolean;
  /** Horizontal field of view, degrees. The gate is against the TURRET: the lens rides it. */
  fov_deg: number;
  maxRange_in: number;
  /** Past this incidence the tag face has too little projected area to decode. */
  maxIncidence_deg: number;
  frameRateHz: number;
  latencyMs: number;
  noise: {
    bearing_deg: number;
    /** One sigma on range as a FRACTION of range: tag ranging is an apparent-size estimate. */
    rangeFrac: number;
    open_deg: number;
  };
}

export interface SensorFrame {
  type: 'sensor';
  seq: number;
  t: number;
  match: { period: Period; remaining: number; started: boolean; stopped: boolean };
  motors: Record<string, MotorState>;
  servos: Record<string, { pos: number }>;
  imu: { yaw: number; pitch: number; roll: number; yawRate: number };
  battery: { volts: number };
  distance: Record<string, number>;
  localizer: { x: number; y: number; heading: number; vx: number; vy: number; omega: number };
  gamepad1: GamepadState;
  gamepad2: GamepadState;
  /**
   * The tag pipeline's output: the most recent DETECTION, or null when the camera has never
   * seen the tag. Stale by construction -- `sampleT` is when the photons left, not now.
   */
  tag: TagSighting | null;
  game: {
    /** Balls the robot is holding, from its own break beam. A robot-side signal, not truth. */
    hopper: number;
    /** The tachometer. Also robot-side. */
    flywheelRpm: number;
    /**
     * GROUND TRUTH. MEASUREMENT ONLY -- a brain that reads this is cheating.
     *
     * These four were `game.upCellAzimuthDeg` and friends, read straight off the world by
     * both brains: perfect bearing, perfect range, and the exact instant the HIVE went over.
     * No robot has any of that. They live behind `truth` now so that every consumer has to
     * say so, and the robot aims off `tag` above like a robot does. Tools and the HUD use
     * these to MEASURE the error the robot is making, which is the only honest use for them.
     */
    truth: { upCellAzimuthDeg: number; upCellRangeIn: number; hiveTipping: boolean; upCellOpenDeg: number };
  };
}

// ---------------------------------------------------------------- snapshot

export interface BallSnapshot { id: number; kind: BallKind; p: Vec3; r: number;
  /**
   * `parked` is OUT OF PLAY: benched below the floor with its collider off. It is a state of
   * its own rather than a flavour of `free` because every "find a loose ball on the tiles"
   * search in the repo filters on `state === 'free'` and a low Y -- and a benched ball passes
   * both. Loading one puts a disabled body in the hopper: the count goes up, the indexer
   * cannot move it, and the robot sits on a full hopper reporting FIRING and never fires.
   * Making it a distinct state fixes every one of those searches at once.
   */
  state: 'free' | 'parked' | 'intake' | 'hopper' | 'flight' | 'cell' | 'flower' }

export interface HiveSnapshot {
  alliance: Alliance;
  angleDeg: number;
  omegaDps: number;
  upCell: 'A' | 'B';
  ballsInUpCell: number;
  ballTorque_Nm: number;
  gravityTorque_Nm: number;
  netTorque_Nm: number;
  perBallTorque: { id: number; kind: BallKind; torque_Nm: number; lever_in: number }[];
  tips: number;
  tipping: boolean;
}

export interface WheelSnapshot { name: string; cmd: number; omega: number; torque_Nm: number; force_N: number; normal_N: number; slip: number; amps: number }

/** Per-shot record. This is the thing you tune an autonomous routine against. */
export interface ShotRecord {
  n: number;
  t: number;
  rangeIn: number;
  bearingDeg: number;
  hoodDeg: number;
  rpm: number;
  targetRpm: number;
  exitSpeed: number;
  /** 'cell' once it settles in the goal, 'miss' if it ends anywhere else. */
  result: 'flight' | 'cell' | 'miss';
  /** Horizontal distance from the CELL mouth where it ended, inches. */
  missBy: number;
  /** Signed downrange error, inches: positive is LONG (past the mouth). */
  long_in: number;
  /** Signed lateral error, inches: positive is left of the shot line. */
  lat_in: number;
  /**
   * Bearing error the ball ACTUALLY left with, degrees: the launch azimuth minus the true
   * azimuth from the muzzle to the mouth, including the launch yaw scatter. Recorded
   * because the land-probability model assumes this is the configured scatter (1 sigma =
   * 1 deg) and nothing was checking that assumption -- the arrival spread says otherwise.
   */
  aimErrDeg: number;
}

export interface Snapshot {
  t: number;
  seq: number;
  period: Period;
  remaining: number;
  balls: BallSnapshot[];
  hives: HiveSnapshot[];
  robot: {
    p: Vec3; yawDeg: number; v: Vec3; speed: number; omegaDps: number;
    ftc: { x: number; y: number; heading: number };
    wheels: WheelSnapshot[];
    battery: { volts: number; soc: number; amps: number };
    /** `power` is what it was TOLD; `omega` is what the shaft is doing. A jam is the gap. */
    intake: { power: number; omega: number; inLine: number; transit: number[] };
    hopper: { count: number; capacity: number; kinds: BallKind[] };
    transfer: { sinceFeed: number; cycleTime: number; ready: boolean; gate: number };
    turret: { angleDeg: number; targetDeg: number; omegaDps: number; atLimit: boolean };
    hood: { angleDeg: number };
    flywheel: { rpm: number; targetRpm: number; amps: number; shots: number };
    lastShot: { v_exit: number; elevDeg: number; azDeg: number; spin: number; t: number } | null;
    flags: string[];
  };
  /** The opponent robot, when the match has one. Same shape; it is the same class. */
  opponent?: Snapshot['robot'];
  score: ScoreState;
  telemetry: [string, string][];
  /** Every shot this match, oldest first. */
  shots: ShotRecord[];
  /** Balls that have left the field and are out of play. */
  outOfPlay: number;
}

export interface ScoreState {
  red: AllianceScore;
  blue: AllianceScore;
  fouls: { t: number; rule: string; note: string }[];
}

export interface AllianceScore {
  tips: number; autoTips: number;
  leave: boolean; park: boolean;
  upCell: number; flower: number; garden: number; bottomNectar: number;
  auto: number; teleop: number; total: number;
  rp: { swarm: boolean; pollinator1: boolean; pollinator2: boolean };
}
