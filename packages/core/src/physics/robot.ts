/**
 * The box robot: chassis, mecanum drivetrain through tyres, and the mechanisms.
 *
 * Robot-local frame (matching the world): +Z forward, +Y up, +X left. That is the same
 * handedness as FTC's robot convention (x forward, y left, z up), just relabelled, which
 * is why `robot.json` mounts are written x-forward/y-left/z-up and permuted on the way in.
 *
 * The drivetrain is not four wheel bodies. Per PLAN.md 8.4 each wheel produces a force
 * along its roller-permitted direction from motor torque, limited by mu*N with weight
 * transfer, and the four are mapped onto the chassis with the mecanum Jacobian transpose.
 */
import type RAPIER_NS from '@dimforge/rapier3d-compat';
import { buildMotor, motorTorque, motorCurrent, radToTicks, type MotorModel } from './motor.js';
import { HubMotorLoop } from '../robot/hubEmulation.js';
import { clamp, DEG, RAD, M_TO_IN, radSToRpm, rpmToRadS } from '../units.js';
import type { ActuatorFrame, Alliance, MotorCmd, RobotSpec, Params, Snapshot, Vec3, BallKind } from '../types.js';
import { allianceOfNectar } from '../rules/scoring.js';
import type { BallSet, Ball } from './balls.js';
import type { Rng } from '../io/rng.js';
import { GROUPS } from './groups.js';

type RAPIER = typeof RAPIER_NS;

export const DRIVE_WHEELS = ['fl', 'fr', 'bl', 'br'] as const;
export type WheelName = (typeof DRIVE_WHEELS)[number];

/** Mecanum Jacobian rows: wheel speed = [forward, left, yaw*h] . [vx, vy, omega]. */
const J: Record<WheelName, [number, number, number]> = {
  fl: [1, -1, -1],
  fr: [1, 1, 1],
  bl: [1, 1, -1],
  br: [1, -1, 1],
};

interface MotorRuntime {
  name: string;
  model: MotorModel;
  loop: HubMotorLoop;
  omega: number; // output shaft rad/s
  rad: number; // integrated output shaft angle
  duty: number;
  torque: number;
  amps: number;
  brake: boolean;
}

interface ServoRuntime {
  pos: number;
  target: number;
  speed: number; // units of position per second
}


export class Robot {
  readonly body: RAPIER_NS.RigidBody;
  readonly motors = new Map<string, MotorRuntime>();
  readonly servos = new Map<string, ServoRuntime>();

  // drivetrain state
  private wheelSlip: Record<WheelName, number> = { fl: 0, fr: 0, bl: 0, br: 0 };
  private wheelForce: Record<WheelName, number> = { fl: 0, fr: 0, bl: 0, br: 0 };
  private wheelNormal: Record<WheelName, number> = { fl: 0, fr: 0, bl: 0, br: 0 };
  private lastLocalVel: [number, number] = [0, 0];
  private accel: [number, number] = [0, 0];
  /** Last forward force applied to the chassis, N. Diagnostic only. */
  debugFx = 0;
  /** Balls the intake roller gripped last step. Diagnostic, read by tools/mechcheck.ts. */
  debugIntake = { touched: 0, f: 0 };
  /** Feed diagnostics: balls the indexer pushed and balls the belt lifted, last step. */
  debugFeed = { indexed: 0, lifted: 0, running: false };

  // mechanisms. These are DERIVED from where the balls physically are, every step --
  // nothing here is a counter that the code increments and then believes.
  /** Balls whose centre is inside the bin this step. */
  hopper: Ball[] = [];
  /** Balls inside the feed shaft, lowest first. */
  private inShaft: Ball[] = [];
  private sinceFeed = 999;
  /**
   * A feed pulse owes exactly one ball. Armed when the gate servo passes half travel opening,
   * spent by the launch, and expired once the servo is fully shut again -- so a ball that the
   * pulse admitted fires when it reaches the wheel even if the servo has started back, and a
   * ball that arrives after the gate has shut waits for the next pulse.
   */
  private gateArmed = false;
  /** The servo has been fully shut since the last opening, so the next opening is a new pulse. */
  private gateShut = true;
  /** Top face of the bin floor, robot-local. */
  private binFloorY = 0;
  /** The gate plate across the feed tube. Enabled means closed. */
  private gateCollider: RAPIER_NS.Collider | null = null;
  /** Inside of the feed shaft, robot-local: half-extent and centre height range. */
  private shaft = { half: 0.055, z: -0.02, topY: 0.11, capY: 0.17, wall: 0.008, gateY: 0.05 };
  turretAngle = 0;
  turretOmega = 0;
  turretTargetDeg = 0;
  turretAtLimit = false;
  hoodAngle = 0;
  flywheelOmega = 0;
  shots = 0;
  lastShot: Snapshot['robot']['lastShot'] = null;
  /** Which ball the last shot used, and what RPM was asked for. For the shot log. */
  lastShotBallId = -1;
  /**
   * Flywheel speed at the instant the ball left, before the shot took energy out of it.
   * Reading `flywheelRpm` after the fact reports the dip, which made every shot look like
   * it had gone out off-speed.
   */
  lastShotRpm = 0;
  lastTargetRpm = 0;
  flags: string[] = [];

  private readonly halfSum: number;
  private readonly startPose: { p: Vec3; yaw: number };

  constructor(
    R: RAPIER,
    world: RAPIER_NS.World,
    private readonly params: Params,
    readonly spec: RobotSpec,
    private readonly rng: Rng,
    start: { p: Vec3; yaw: number },
    /** Which NECTAR this robot is allowed to touch. G408: the other colour is rejected. */
    readonly alliance: Alliance = 'red',
  ) {
    const c = spec.chassis;
    this.halfSum = (spec.drivetrain.wheelbase_m + spec.drivetrain.track_m) / 2;
    this.startPose = { p: [...start.p] as Vec3, yaw: start.yaw };

    const izz = c.Izz_kgm2 ?? (c.mass_kg * (c.length_m * c.length_m + c.width_m * c.width_m)) / 12;
    this.body = world.createRigidBody(
      R.RigidBodyDesc.dynamic()
        .setTranslation(start.p[0], start.p[1], start.p[2])
        .setRotation(quatY(start.yaw))
        // The tyre model applies forces at the CG, so there is no roll/pitch moment to
        // resolve; locking those axes keeps the box upright instead of letting solver
        // noise tip it. Yaw stays free.
        .restrictRotations(false, true, false)
        .setLinearDamping(0)
        .setAngularDamping(0.05),
    );
    // Mass MUST go on the collider. RigidBodyDesc.setAdditionalMass /
    // setAdditionalMassProperties are silently ignored in this Rapier build: the body
    // comes out with mass 0 and invMass 0, i.e. immovable, and no force does anything.
    // ---- the shell, as actual geometry rather than one solid box.
    //
    // A solid cuboid cannot have anything inside it, so the intake and the hopper had to be
    // a state machine: a ball that touched the front vanished and reappeared as a number.
    // This builds the chassis the way it is actually built -- a floor, two sides, a back, a
    // front wall that stops ABOVE the intake mouth, and a vertical feed shaft up the turret
    // axis -- so a ball is a rigid body the whole way through the robot. What the hopper
    // holds is then whatever fits in it, which is the honest answer.
    const t = 0.008;                       // plate thickness, 5/16 in polycarb
    const hw = c.width_m / 2;
    const hh = c.height_m / 2;
    const hl = c.length_m / 2;
    const ip0 = spec.intake;
    const mouthH = ip0.mouth.height_m;
    const floorY = -hh + t * 1.5;          // top face of the floor plate: shell() takes
                                           // HALF-extents, so the plate is 2t thick
    this.binFloorY = floorY;

    const shell = (half: Vec3, at: Vec3, mass = 0, pitch = 0) => {
      let d = R.ColliderDesc.cuboid(half[0], half[1], half[2])
        .setTranslation(at[0], at[1], at[2])
        .setRotation({ x: Math.sin(pitch / 2), y: 0, z: 0, w: Math.cos(pitch / 2) })
        // Nearly frictionless, with Rapier's Min rule rather than its default Average.
        //
        // The floor plate is the only part of the shell that can touch a tile, and all
        // ground traction is supposed to come from the tyre model below. At 0.25 against
        // the tiles' 0.85 the plate dragged at about 35 N -- most of the drivetrain's
        // output -- and top speed fell from 63 to 45 in/s. Balls inside the bin do not
        // need shell friction: the walls contain them, and a bin whose floor is slippery
        // is a bin the indexer can actually sweep.
        .setFriction(0.02)
        .setFrictionCombineRule(R.CoefficientCombineRule.Min)
        .setRestitution(0.08)
        .setRestitutionCombineRule(R.CoefficientCombineRule.Min)
        .setCollisionGroups(GROUPS.robot);
      if (mass > 0) {
        // Mass MUST go on a collider. RigidBodyDesc.setAdditionalMass and
        // setAdditionalMassProperties are silently ignored in this Rapier build: the body
        // comes out with mass 0 and invMass 0, i.e. immovable, and no force does anything.
        d = d.setMassProperties(mass, { x: c.cgOffset_m[0], y: c.cgOffset_m[1], z: c.cgOffset_m[2] }, { x: izz, y: izz, z: izz }, { x: 0, y: 0, z: 0, w: 1 });
      } else {
        d = d.setMassProperties(0, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0, w: 1 });
      }
      world.createCollider(d, this.body);
    };

    // floor: carries the whole chassis mass and inertia
    shell([hw, t, hl], [0, -hh + t / 2, 0], c.mass_kg);
    // sides and back, full height
    shell([t, hh, hl], [hw - t, 0, 0]);
    shell([t, hh, hl], [-(hw - t), 0, 0]);
    shell([hw, hh, t], [0, 0, -(hl - t)]);
    // FRONT: only above the intake mouth, so the mouth is a real opening a ball drives into
    const frontH = (c.height_m - (t * 2 + mouthH)) / 2;
    shell([hw, frontH, t], [0, -hh + t * 2 + mouthH + frontH, hl - t]);
    // NO RAMP. There was one, and a plate that reaches out to tile level to catch a ball
    // also drags on the tile and cost the robot three quarters of its top speed -- it made
    // the robot a plough. A real over-the-bumper intake does not use a ramp either: the
    // compliant roller squeezes the ball and carries it up over the floor plate's edge,
    // which is what the lift term in stepIntake models. The ball cannot escape UNDER the
    // robot because a 71 mm ball does not fit through a 20 mm ground clearance.
    this.buildFeedShaft(R, world, shell);

    const addMotor = (name: string, spec2: { variant: string; gearRatio: number; reversed?: boolean }) => {
      const model = buildMotor(spec2);
      const maxTicks = (model.ticksPerRev * radSToRpm(model.freeOmega)) / 60;
      this.motors.set(name, { name, model, loop: new HubMotorLoop(spec.hub, maxTicks), omega: 0, rad: 0, duty: 0, torque: 0, amps: 0, brake: true });
    };
    for (const w of DRIVE_WHEELS) addMotor(w, spec.drivetrain.motors[w]);
    addMotor('intake', spec.intake.motor);
    addMotor('transfer', spec.transfer.motor);
    addMotor('flywheel', spec.flywheel.motor);
    if (spec.turret.enabled) addMotor('turret', spec.turret.motor);

    this.servos.set('hood', { pos: 0.5, target: 0.5, speed: spec.hood.speed_dps / Math.max(1, spec.hood.angleRange_deg[1] - spec.hood.angleRange_deg[0]) });
    this.servos.set('gate', { pos: 0, target: 0, speed: 4 });
    this.hoodAngle = spec.hood.enabled ? mid(spec.hood.angleRange_deg) : spec.hood.fixedAngle_deg;
  }

  // ------------------------------------------------------------------ pose

  /** Place the turret directly, for measurement rigs that are not testing the servo. */
  setTurretForTest(deg: number): void { this.turretAngle = deg; this.turretTargetDeg = deg; }

  /** Chassis velocity, world m/s. The intake needs it: slip is relative to the ROBOT. */
  get vel(): Vec3 {
    const v = this.body.linvel();
    return [v.x, v.y, v.z];
  }

  get pos(): Vec3 {
    const t = this.body.translation();
    return [t.x, t.y, t.z];
  }

  get yaw(): number {
    const r = this.body.rotation();
    return 2 * Math.atan2(r.y, r.w);
  }

  /** World vector -> robot local (+Z forward, +X left). */
  toLocal(v: Vec3): Vec3 {
    const a = -this.yaw;
    const c = Math.cos(a);
    const s = Math.sin(a);
    return [v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c];
  }

  /** Robot local vector -> world. */
  toWorld(v: Vec3): Vec3 {
    const c = Math.cos(this.yaw);
    const s = Math.sin(this.yaw);
    return [v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c];
  }

  /** `robot.json` mounts are x-forward, y-left, z-up; the world is x-left, y-up, z-forward. */
  static mountToLocal(m: Vec3): Vec3 {
    return [m[1], m[2], m[0]];
  }

  // ------------------------------------------------------------------ step

  /**
   * One physics step's worth of robot. Call before world.step().
   * Returns the total current draw so the battery can sag.
   */
  preStep(act: ActuatorFrame, dt: number, t: number, volts: number, balls: BallSet, aimTarget: Vec3 | null): number {
    this.flags = [];
    // Rapier accumulates addForce/addTorque across steps until they are reset.
    this.body.resetForces(false);
    this.body.resetTorques(false);
    this.applyServos(act, dt);

    let amps = 0;
    amps += this.stepDrivetrain(act, dt, volts);
    amps += this.stepFlywheel(act, dt, volts);
    amps += this.stepTurret(act, dt, volts);
    amps += this.stepSimpleMotor('intake', act, dt, volts);
    amps += this.stepSimpleMotor('transfer', act, dt, volts);

    // Order matters and it is the order the real machine works in: find out where every
    // ball actually is, then drive the rollers, then the belt, then check the nip. The
    // census is first so the intake and the transfer are both reasoning about the same
    // instant rather than about each other's leftovers.
    this.censusBalls(balls);
    this.stepIntake(dt, balls);
    this.stepTransfer(dt, balls);
    this.stepNip(balls, t);
    void aimTarget;

    for (const m of this.motors.values()) m.loop.updateReportedVelocity(t, radToTicks(m.model, m.rad));
    this.sampleFlywheelEncoder(t);
    return amps;
  }

  private applyServos(act: ActuatorFrame, dt: number): void {
    for (const [name, s] of this.servos) {
      const cmd = act.servos?.[name];
      if (cmd !== undefined) s.target = clamp(cmd, 0, 1);
      const step = s.speed * dt;
      s.pos += clamp(s.target - s.pos, -step, step);
    }
    const hp = this.spec.hood;
    if (hp.enabled) {
      const hood = this.servos.get('hood')!;
      this.hoodAngle = hp.angleRange_deg[0] + hood.pos * (hp.angleRange_deg[1] - hp.angleRange_deg[0]);
    } else {
      this.hoodAngle = hp.fixedAngle_deg;
    }
  }

  private commandDuty(m: MotorRuntime, cmd: MotorCmd | undefined, dt: number): number {
    m.brake = cmd?.brake ?? true;
    const ticks = radToTicks(m.model, m.rad);
    const velTicks = m.loop.reportedVel;
    let duty = m.loop.duty(cmd, ticks, velTicks, dt);
    if (m.model.reversed) duty = -duty;
    m.duty = duty;
    return duty;
  }

  /** Motors whose load is a fixed viscous drag: intake rollers, the transfer belt. */
  private stepSimpleMotor(name: string, act: ActuatorFrame, dt: number, volts: number): number {
    const m = this.motors.get(name);
    if (!m) return 0;
    const duty = this.commandDuty(m, act.motors?.[name], dt);
    const tau = motorTorque(m.model, duty, m.omega, volts, m.brake);
    const load = 5e-4 * m.omega; // rollers and belts: light viscous drag
    const inertia = Math.max(m.model.inertia, 1e-6);
    m.omega += ((tau - load) / inertia) * dt;
    m.rad += m.omega * dt;
    m.torque = tau;
    m.amps = motorCurrent(m.model, tau);
    return m.amps;
  }

  private stepDrivetrain(act: ActuatorFrame, dt: number, volts: number): number {
    const dt2 = dt;
    const d = this.spec.drivetrain;
    const c = this.spec.chassis;
    const r = d.wheelRadius_m;

    const lv = this.body.linvel();
    const local = this.toLocal([lv.x, lv.y, lv.z]);
    const vx = local[2]; // forward
    const vy = local[0]; // left
    const omegaYaw = this.body.angvel().y;

    this.accel = [(vx - this.lastLocalVel[0]) / dt2, (vy - this.lastLocalVel[1]) / dt2];
    this.lastLocalVel = [vx, vy];

    // Normal loads with weight transfer off the last step's acceleration.
    const m = c.mass_kg;
    const g = this.params.env.g;
    const h = c.height_m / 2 + c.cgOffset_m[1];
    const dNx = (m * this.accel[0] * h) / (2 * d.wheelbase_m);
    const dNy = (m * this.accel[1] * h) / (2 * d.track_m);
    const base = (m * g) / 4;
    const N: Record<WheelName, number> = {
      fl: Math.max(0, base - dNx - dNy),
      fr: Math.max(0, base - dNx + dNy),
      bl: Math.max(0, base + dNx - dNy),
      br: Math.max(0, base + dNx + dNy),
    };
    this.wheelNormal = N;

    // ponytail: one slip stiffness for every surface. Per-tile / per-wheel values would need
    // measurements we do not have; kSlip only has to be stiff enough to look rigid below the
    // friction limit, and the implicit update below keeps it stable at 1/240 s.
    const kSlip = 400;
    let Fx = 0;
    let Fy = 0;
    let Mz = 0;
    let amps = 0;

    for (const w of DRIVE_WHEELS) {
      const mot = this.motors.get(w)!;
      const duty = this.commandDuty(mot, act.motors?.[w], dt2);
      const [jf, jl, jh] = J[w];
      const vGround = jf * vx + jl * vy + jh * this.halfSum * omegaYaw;

      const tau = motorTorque(mot.model, duty, mot.omega, volts, mot.brake);
      const I = Math.max(mot.model.inertia + 0.0002, 1e-6);

      // Implicit in omega so the stiff slip spring cannot blow up at 1/240 s.
      const omegaNext = (mot.omega + (dt2 / I) * (tau + kSlip * r * vGround)) / (1 + (dt2 / I) * kSlip * r * r);
      let f = kSlip * (omegaNext * r - vGround);
      const limit = d.mu * N[w];
      if (Math.abs(f) > limit) {
        f = Math.sign(f) * limit;
        mot.omega += ((tau - f * r) / I) * dt2; // saturated: the wheel spins up on its own
      } else {
        mot.omega = omegaNext;
      }
      mot.rad += mot.omega * dt2;
      mot.torque = tau;
      mot.amps = motorCurrent(mot.model, tau);
      amps += mot.amps;

      this.wheelSlip[w] = omegaNext * r - vGround;
      this.wheelForce[w] = f;

      const fe = f * d.eta;
      Fx += jf * fe;
      Fy += jl * fe;
      Mz += jh * fe * this.halfSum;
    }

    // Rolling resistance and a little body drag, opposing motion.
    const speed = Math.hypot(vx, vy);
    if (speed > 1e-3) {
      Fx -= (d.rollingRes_N * vx) / speed;
      Fy -= (d.rollingRes_N * vy) / speed;
    }

    const world = this.toWorld([Fy, 0, Fx]);
    this.debugFx = Fx;
    this.body.addForce({ x: world[0], y: 0, z: world[2] }, true);
    this.body.addTorque({ x: 0, y: Mz, z: 0 }, true);

    if (Object.values(this.wheelSlip).some((s) => Math.abs(s) > 0.25)) this.flags.push('WHEEL SLIP');
    return amps;
  }

  private stepFlywheel(act: ActuatorFrame, dt: number, volts: number): number {
    const m = this.motors.get('flywheel')!;
    const f = this.spec.flywheel;
    this.lastTargetRpm = this.targetFlywheelRpm(act);
    const duty = this.commandDuty(m, act.motors?.['flywheel'], dt);
    // TWO MOTORS ON ONE WHEEL is the usual answer to a flywheel that cannot keep up, and it
    // is not free: each one adds its rotor inertia to the thing it is trying to accelerate,
    // and each one draws its own current from the same battery.
    const n = Math.max(1, Math.round(f.motorCount ?? 1));
    const tau = n * motorTorque(m.model, duty, m.omega, volts, false); // a flywheel always floats
    const I = f.I_fly_kgm2 + n * m.model.inertia;
    const drag = f.dragQuad_Nms2 * m.omega * Math.abs(m.omega) + f.coulomb_Nm * Math.sign(m.omega);
    m.omega = Math.max(0, m.omega + ((tau - drag) / I) * dt);
    m.rad += m.omega * dt;
    m.torque = tau;
    m.amps = n * motorCurrent(m.model, tau / n);
    this.flywheelOmega = m.omega;
    return m.amps;
  }

  /**
   * The turret is a real axis with mass, so it obeys a trapezoidal profile: it cannot leave
   * standstill at full slew rate, and it has to start slowing down before it arrives or it
   * would sail past the target. The deceleration bound sqrt(2*a*err) is what keeps it from
   * overshooting -- without it a rate limit alone oscillates around the bearing forever.
   */
  private stepTurret(act: ActuatorFrame, dt: number, volts: number): number {
    const t = this.spec.turret;
    if (!t.enabled) return 0;
    const m = this.motors.get('turret')!;
    const cmd = act.motors?.['turret'];
    const duty = this.commandDuty(m, cmd, dt);

    const ticksPerDeg = t.motor.ticksPerDeg ?? m.model.ticksPerRev / 360;
    if (cmd?.mode === 'RUN_TO_POSITION') {
      this.turretTargetDeg = (cmd.target ?? 0) / ticksPerDeg;
    } else if (duty !== 0) {
      // A raw power command is a rate request, as it would be on the hub.
      this.turretTargetDeg = this.turretAngle + duty * t.speed_dps * 0.25;
    }
    this.turretTargetDeg = clamp(this.turretTargetDeg, t.range_deg[0], t.range_deg[1]);

    const err = this.turretTargetDeg - this.turretAngle;
    const aMax = Math.max(1, t.accel_dps2);
    const vMax = Math.max(1, t.speed_dps);
    // One step can only change the rate by a*dt, so the profile cannot resolve anything
    // finer than that. Without an explicit landing case it dithers either side of the
    // target forever at +-a*dt instead of arriving.
    const vQuantum = aMax * dt;
    if (Math.abs(err) <= Math.max(0.02, Math.abs(this.turretOmega) * dt) && Math.abs(this.turretOmega) <= vQuantum * 2) {
      // Land the position, but bleed the last of the rate off at the acceleration limit
      // rather than discarding it -- an instant stop from even 7 deg/s reads as an
      // acceleration spike an order of magnitude past what the axis can do.
      this.turretAngle = this.turretTargetDeg;
      this.turretOmega -= Math.sign(this.turretOmega) * Math.min(Math.abs(this.turretOmega), vQuantum);
    } else {
      // Fastest we may be going and still stop exactly on target. sqrt(2*a*err) is the
      // continuous bound; in discrete time it overshoots by up to half a step, so back it
      // off by half a velocity quantum. Without that the axis arrives at a hard stop still
      // moving and the limit has to discard the rate, which is a fake acceleration spike.
      const vStop = Math.max(0, Math.sqrt(2 * aMax * Math.abs(err)) - vQuantum * 0.5);
      const vWant = Math.sign(err) * Math.min(vMax, vStop);
      this.turretOmega += clamp(vWant - this.turretOmega, -vQuantum, vQuantum);
      this.turretAngle += this.turretOmega * dt;
    }

    // Hard stops: the axis cannot travel past them, and it arrives there stopped.
    this.turretAtLimit = false;
    if (this.turretAngle <= t.range_deg[0]) {
      this.turretAngle = t.range_deg[0];
      this.turretOmega = Math.max(0, this.turretOmega);
      this.turretAtLimit = true;
    } else if (this.turretAngle >= t.range_deg[1]) {
      this.turretAngle = t.range_deg[1];
      this.turretOmega = Math.min(0, this.turretOmega);
      this.turretAtLimit = true;
    }
    m.omega = this.turretOmega * DEG;
    m.rad = (this.turretAngle * ticksPerDeg * 2 * Math.PI) / m.model.ticksPerRev;
    // Draw current in proportion to how hard the profile is working the motor.
    m.torque = motorTorque(m.model, duty, m.omega, volts, true);
    const effort = Math.min(1, Math.abs(this.turretOmega) / vMax + (this.turretAtLimit ? 0 : 0.1));
    m.amps = motorCurrent(m.model, m.torque) * Math.max(0.05, effort);
    return m.amps;
  }

  /**
   * The feed shaft: a square tube up the turret's rotation axis.
   *
   * A turret has to be fed on its own axis or the feed would have to rotate with it, which
   * is why every real turreted shooter has a tube like this. Balls enter through a gap in
   * the front wall at the bottom, the transfer belt drives them up the inside, and the top
   * of the tube IS the flywheel nip. Four walls with a gap, and the gap is what makes it a
   * mechanism instead of a pipe.
   */
  /**
   * The feed shaft: a square tube up the turret's rotation axis.
   *
   * A turret has to be fed on its own axis, or the feed would have to rotate with it --
   * which is why every real turreted shooter has a tube like this. Balls are swept in
   * through gaps at the bottom, the belt drives them up the inside, and the top of the
   * tube is the flywheel nip.
   *
   * Three details here were each worth a wasted afternoon:
   *  - it is open at the FRONT AND BACK. With only a front opening, any ball that ended up
   *    behind the tube was unreachable and sat in the bin for the whole match.
   *  - the walls run past the nip up to a stop. Ending them at the nip left a ball-sized
   *    gap on all four sides exactly where the ball arrives at 0.8 m/s, and it popped out
   *    sideways and fell back down.
   *  - a cap across the top. Without it the BELT fires the ball out of the tube.
   */
  private buildFeedShaft(
    R: RAPIER,
    world: RAPIER_NS.World,
    shell: (half: Vec3, at: Vec3, mass?: number, pitch?: number) => void,
  ): void {
    void R;
    void world;
    const t = 0.006;
    // SINGLE FILE, AND WIDE ENOUGH FOR A NECTAR. Both, which the old note said was
    // impossible -- and the arithmetic it argued from was right while its conclusion was not.
    //
    // It read: a bore that takes a 3.62 in NECTAR is 4.1 in, and two 2.8 in POLLEN fit side
    // by side in 4.1 in, so they wedge and the magazine jams solid. True. But the binding
    // number is not 2 x 2.8 = 5.6 in of side-by-side, it is the DIAGONAL, 2.8 x sqrt(2) =
    // 3.96 in: below that no two POLLEN can sit in the bore in any orientation. So anything
    // in [3.62, 3.96) takes a NECTAR and is still single file, and 3.80 in sits in the
    // middle of that window with 0.17 in of margin each side.
    //
    // The 3.20 in bore this used to have was not merely conservative, it JAMMED. A POLLEN
    // has 0.2 in of clearance in it, and with that little a ball that enters slightly
    // crooked wedges in the doorway, below `entryY` -- which leaves `entryBusy` true for
    // ever, so the indexer admits nothing more while the belt cannot free what is stuck.
    // Measured with tools/_feed-style tracing: on seed 43 one ball sat at -2.23 in for the
    // whole run with the belt driving and the gate cycling, `shots 0`. That is the README's
    // "counts its hopper down from 6 to 2 and the world records shots 0" (PHYSICS 9.14) and
    // the "indexer cannot feed from a nearly empty bin" entry in docs/DECISIONS.md (9.13):
    // one bore, three findings.
    const r = this.params.ball.pollen.d_m / 2;
    const half = (this.spec.transfer.boreSize_m ?? 0.0965) / 2;
    const topY = this.spec.turret.muzzleHeight_m - this.spec.chassis.height_m / 2 - this.spec.chassis.clearance_m;
    const capY = topY + r + t * 2;
    const z = -0.02;
    const gateY = topY - r * 2.1;
    this.shaft = { half, z, topY, capY, wall: t, gateY };

    const lo = this.binFloorY;
    const midY = (lo + capY) / 2;
    const h = (capY - lo) / 2;

    // Two solid sides, full height.
    shell([t, h, half + t], [half + t, midY, z]);
    shell([t, h, half + t], [-(half + t), midY, z]);

    // Front and back: closed above one ball's height, open below it. Those two openings are
    // the whole mechanism -- they are what the indexer sweeps balls through.
    const gate = lo + r * 2.4;
    for (const sz of [1, -1]) {
      shell([half + t, (capY - gate) / 2, t], [0, (capY + gate) / 2, z + sz * (half + t)]);
    }

    // The stop the ball waits against at the nip.
    shell([half + t, t, half + t], [0, capY, z]);

    // THE GATE, as an actual plate across the tube rather than as a boolean.
    //
    // Without it the belt has to be switched off between shots, the tube empties itself
    // back into the bin every cycle, and the robot re-lifts the same ball four times. With
    // it the belt runs continuously, the magazine stays loaded against the gate, and
    // "ready to fire" becomes a physical event: the plate retracts and the next ball rises
    // into the wheel. The servo drives `setEnabled` on this collider each step.
    this.gateCollider = world.createCollider(
      R.ColliderDesc.cuboid(half, t / 2, half)
        .setTranslation(0, gateY, z)
        .setMassProperties(0, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0, w: 1 })
        .setFriction(0.02)
        .setFrictionCombineRule(R.CoefficientCombineRule.Min)
        .setRestitution(0.02)
        .setCollisionGroups(GROUPS.robot),
      this.body,
    );
  }

  // ------------------------------------------------------------ mechanisms


  /**
   * The intake, as traction rather than as a trigger volume.
   *
   * A roller does not "capture" a ball: it touches it, and the difference between the
   * roller surface speed and the ball surface speed produces a friction force, capped by
   * how hard the ball is pressed against it. That is the same slip-and-cap model the tyres
   * use a hundred lines up, and it is why this reads the way it does. Run the roller
   * backwards and the same force ejects the ball -- there is no separate "spit" code path,
   * because a real intake does not have one either.
   *
   * What follows from doing it this way, none of it written down anywhere: a ball at a bad
   * angle gets spat back out, a bin that is physically full refuses the next one, and
   * chasing a ball at roller speed picks up nothing at all.
   */
  private stepIntake(_dt: number, balls: BallSet): void {
    const ip = this.spec.intake;
    const m = this.motors.get('intake')!;
    const duty = m.duty;
    this.debugIntake = { touched: 0, f: 0 };
    if (Math.abs(duty) < 0.02) return;

    // Surface speed from the ACTUAL shaft speed, not from the command: a stalled roller
    // has no surface speed and therefore no grip, which is what a jam is.
    const vSurface = m.omega * (ip.rollerRadius_m ?? 0.035);
    const p = this.pos;
    const v = this.vel;
    const hl = this.spec.chassis.length_m / 2;
    const mouthHalfW = ip.mouth.width_m / 2;
    const zFront = hl + ip.mouth.depth_m;
    const zBack = hl - 0.06;
    const g = this.params.env.g;

    for (const b of balls.balls) {
      if (b.state === 'flight' || b.state === 'cell' || !b.body.isEnabled()) continue;
      const bp = balls.pos(b);
      const rel = this.toLocal([bp[0] - p[0], bp[1] - p[1], bp[2] - p[2]]);
      if (Math.abs(rel[0]) > mouthHalfW + b.radius) continue;
      if (rel[2] < zBack || rel[2] > zFront) continue;
      if (rel[1] > this.binFloorY + ip.mouth.height_m + b.radius) continue;

      // G408: NEVER CONTROL THE OPPONENT'S NECTAR.
      //
      // A real robot enforces this with a colour sensor at the mouth that reverses the roller
      // when it sees the wrong colour, and that is exactly what this does -- the sign of the
      // surface speed flips for that one ball, so the same traction model that pulls a legal
      // ball in pushes an illegal one out. There is no separate reject path, no teleport, and
      // no rule check: an opponent NECTAR simply cannot be driven into the bin, so the
      // violation is impossible rather than penalised.
      const reject = b.kind !== 'pollen' && allianceOfNectar(b.kind) !== this.alliance;
      const surface = reject ? -vSurface : vSurface;

      // Slip is roller surface speed minus the ball speed, both in the ROBOT frame.
      const bv = b.body.linvel();
      const rv = this.toLocal([bv.x - v[0], bv.y - v[1], bv.z - v[2]]);
      const slip = -surface - rv[2];                   // +Z is forward; the roller pulls -Z

      // Normal load: the ball weight plus the squeeze from the roller sitting below ball
      // height. Without a squeeze term an intake could never lift anything.
      const N = b.mass * g + (ip.squeeze_N ?? 6) * Math.min(1, Math.abs(duty));
      const fMax = (ip.rollerMu ?? 1.1) * N;
      const f = clamp(slip * 40 * b.mass, -fMax, fMax);

      this.debugIntake.touched++;
      this.debugIntake.f = f;
      // Wake it explicitly. addForce's wakeUp flag does not reliably wake a sleeping body
      // in this Rapier build: a ball that had gone to sleep in the bin sat there with a
      // net upward force on it and zero contacts, not moving, for a whole match.
      b.body.wakeUp();
      const w = this.toWorld([0, 0, f]);
      b.body.addForce({ x: w[0], y: 0, z: w[2] }, true);
      if (duty > 0 && !reject) {
        // Below the bin floor the ball is still outside and the roller has to lift it over
        // the floor plate's edge; once it is over, the roller presses it DOWN into the bin.
        // Getting this backwards wedged every ball under the chassis.
        const climbing = rel[1] < this.binFloorY + b.radius;
        b.body.addForce({ x: 0, y: Math.abs(f) * (climbing ? 1.4 : -0.35), z: 0 }, true);
      }
      // Reaction on the chassis: a robot really does feel a ball it picks up at speed.
      this.body.addForce({ x: -w[0], y: 0, z: -w[2] }, true);
    }
  }

  /**
   * Who is in the bin and who is in the shaft, worked out from where the balls ARE.
   *
   * There is no add/remove bookkeeping anywhere. A ball is in the hopper because its
   * centre is inside the hopper, and it stops being in the hopper the moment it physically
   * leaves. That is the difference between a count you can trust and a count that drifts
   * out of step with the picture.
   */
  private censusBalls(balls: BallSet): void {
    this.lastCensus = balls;
    const p = this.pos;
    const c = this.spec.chassis;
    const hw = c.width_m / 2;
    const hl = c.length_m / 2;
    const hh = c.height_m / 2;
    this.hopper = [];
    this.inShaft = [];
    for (const b of balls.balls) {
      if (b.state === 'flight' || b.state === 'cell' || !b.body.isEnabled()) continue;
      const bp = balls.pos(b);
      const rel = this.toLocal([bp[0] - p[0], bp[1] - p[1], bp[2] - p[2]]);

      // The shaft is tested on its OWN extent, not the chassis box. Its top sticks out
      // above the chassis, so a box test dropped the ball from the census exactly when it
      // reached the nip -- the belt stopped driving it and it fell back down the tube.
      const sh = this.shaft;
      // A ball is in the tube when the BALL fits in the bore, not when its CENTRE does.
      //
      // `sh.half + 0.01` counted a ball whose centre was still 10 mm OUTSIDE the bore wall,
      // i.e. one more than half of it still in the doorway. The belt then lifted it, its
      // crown caught the lintel above the opening, and it stopped dead with the belt still
      // driving into it -- a ball permanently in the magazine that could never reach the
      // nip. This pairs with the indexer lift: with a purely horizontal push almost nothing
      // ever got far enough in to jam, so the generous test looked harmless.
      const fit = sh.half - b.radius + 0.004;
      const inShaft =
        Math.abs(rel[0]) < fit &&
        Math.abs(rel[2] - sh.z) < fit &&
        rel[1] > this.binFloorY - 0.02 && rel[1] < sh.capY + 0.02;
      if (inShaft) {
        this.inShaft.push(b);
        b.state = 'intake';       // on its way up to the shooter
        continue;
      }
      const inBox = Math.abs(rel[0]) < hw && Math.abs(rel[2]) < hl && rel[1] > -hh && rel[1] < hh;
      if (!inBox) {
        if (b.state === 'hopper' || b.state === 'intake') b.state = 'free';
        continue;
      }
      this.hopper.push(b);
      b.state = 'hopper';
    }
    // Highest first: the next one to fire is the one nearest the nip.
    this.inShaft.sort((a, b2) => balls.pos(b2)[1] - balls.pos(a)[1]);
  }

  /**
   * The transfer: an indexer at the shaft opening and a belt up the inside of it.
   *
   * Both are traction, like the intake. The consequence worth knowing is that cycle time is
   * no longer a number the code obeys -- it is how long the belt actually takes to carry a
   * ball up the tube, which depends on belt speed, ball mass and whether anything is in the
   * way. `transfer.cycleTime_s` survives only as a floor, because a real indexer is also
   * rate-limited by its own geometry.
   */
  private stepTransfer(dt: number, balls: BallSet): void {
    const tp = this.spec.transfer;
    const gate = this.servos.get('gate')!;
    const gateOpen = !tp.gate.enabled || Math.abs(gate.pos - tp.gate.open) < 0.25;
    // The gate is a plate, and this is where the servo moves it. Balls stack against it
    // while it is closed, which is what keeps the magazine loaded between shots.
    //
    // AND A PLATE CANNOT CLOSE THROUGH A BALL. The collider used to come back the instant the
    // servo dropped under 0.75, on a clock, and the belt was still lifting the admitted ball
    // through the plane it closes across: measured (tools/feedprobe.ts) the ball's centre sat
    // 10-20 mm past the plate when it re-appeared inside it, and the contact solver threw the
    // ball whichever way was nearer -- sometimes up into the wheel, sometimes back under the
    // gate to wait a whole cycle. A real flap presses on the ball and closes once it has gone
    // by; that is what this does.
    if (gateOpen || this.gateBlocked(balls)) this.gateCollider?.setEnabled(false);
    else this.gateCollider?.setEnabled(true);
    const m = this.motors.get('transfer')!;
    this.sinceFeed += dt;
    this.debugFeed = { indexed: 0, lifted: 0, running: m.duty > 0.2 };
    // The BELT does not care about the gate: a real feed belt runs continuously and the
    // gate decides what gets through. Switching the belt off instead let the whole tube
    // empty back into the bin after every shot.
    if (m.duty <= 0.2) return;

    const p = this.pos;
    const sh = this.shaft;
    const beltV = Math.abs(m.omega) * (tp.beltRadius_m ?? 0.024);
    const g = this.params.env.g;

    // ---- the belt, on everything already in the tube
    for (const b of this.inShaft) {
      const bv = b.body.linvel();
      // The belt has to beat gravity to lift anything, so its cap is written as a multiple
      // of the ball's weight -- a raw newton figure would silently stop working the moment
      // someone made the ball heavier in the Variables panel.
      const slip = beltV - bv.y;
      const f = clamp(slip * 30 * b.mass, -4 * b.mass * g, 4 * b.mass * g);
      this.debugFeed.lifted++;
      b.body.wakeUp();
      b.body.addForce({ x: 0, y: f, z: 0 }, true);
      // Damp the sideways rattle: a ball pinballing between two walls a few mm apart bleeds
      // its whole climb into noise and never reaches the top.
      b.body.addForce({ x: -bv.x * 2 * b.mass, y: 0, z: -bv.z * 2 * b.mass }, true);
      // AND KEEP THE COLUMN SINGLE FILE. The bore is 25 mm wider than the ball, so a chassis
      // accelerating sideways can push successive balls into alternate CORNERS of the square
      // bore, where the stack becomes an arch: each ball pressed diagonally into two walls by
      // the one below, the belt lifting all four, nothing moving. Measured by
      // tools/releasecheck.ts --wasted: four balls at x = -14, +13, -13, +13 mm, the top one
      // 11 mm short of the wheel, the belt at full speed, 30 s of pulses feeding nothing. A
      // real belt runs up one wall and presses the ball against the opposite one, so the
      // balls ride in one line and cannot stagger; this is that constraint as a soft spring
      // toward the bore axis.
      // ponytail: a spring, not a modelled belt face. Model the belt as a wall contact if the
      // single-file assumption ever has to be measured rather than imposed.
      const q = balls.pos(b);
      const rel = this.toLocal([q[0] - p[0], q[1] - p[1], q[2] - p[2]]);
      const centre = this.toWorld([-rel[0] * 400 * b.mass, 0, -(rel[2] - sh.z) * 400 * b.mass]);
      b.body.addForce({ x: centre[0], y: 0, z: centre[2] }, true);
    }

    // ---- the indexer, metered to ONE ball at a time
    //
    // This is the part that has to be metered, and it took a jam to see why. Pushing every
    // eligible ball at the tube at once sent two in through the two opposite openings on
    // the same step; they wedged diagonally against each other inside the bore, the belt
    // drove both further into the wedge, and the robot fired one ball and then sat there
    // for the rest of the match. A real indexer has pockets and admits one ball per pocket,
    // so this admits one ball at a time and waits for the entry to clear -- which is the
    // same rule, expressed as a condition instead of as a wheel.
    // One ball diameter of travel: once the last one admitted has cleared the entry by
    // its own width, the next may come in. Blocking on the whole lower tube instead meant
    // a loaded magazine never admitted anything again.
    const entryY = this.binFloorY + this.params.ball.pollen.d_m * 1.1;
    const entryBusy = this.inShaft.some((b) => {
      const q = balls.pos(b);
      return this.toLocal([q[0] - p[0], q[1] - p[1], q[2] - p[2]])[1] < entryY;
    });
    if (entryBusy) return;

    // Of the balls that can physically enter, take the one already closest to the tube.
    let best: Ball | null = null;
    let bestRel: Vec3 | null = null;
    let bestD = Infinity;
    for (const b of this.hopper) {
      // Anything too fat for the bore is left in the bin rather than rammed at the opening.
      // The geometry does the filtering, which is how a real indexer rejects the wrong game
      // element too: NECTAR is 3.62 in and this bore takes 2.8 in.
      if (b.radius > sh.half - 0.002) continue;
      const q = balls.pos(b);
      const rel = this.toLocal([q[0] - p[0], q[1] - p[1], q[2] - p[2]]);
      // Only balls low enough to go through a gap; one sitting on top of another cannot.
      if (rel[1] > this.binFloorY + b.radius * 2.2) continue;
      const d = Math.hypot(rel[0], rel[2] - sh.z);
      if (d < bestD) {
        bestD = d;
        best = b;
        bestRel = rel;
      }
    }
    if (!best || !bestRel) return;

    // Steer to the nearer OPENING, not to the tube's axis. Aiming at the axis pressed every
    // ball that was beside the tube into a solid side wall, where it sat for the rest of the
    // match -- the push was pointing through 6 mm of polycarbonate. So a ball that is not
    // yet lined up with a gap is walked around to that gap's mouth first.
    const rel = bestRel;
    const side = rel[2] >= sh.z ? 1 : -1;
    const mouthZ = sh.z + side * (sh.half + best.radius + 0.008);
    const lined = Math.abs(rel[0]) < sh.half;
    const toward: Vec3 = lined
      ? [-rel[0] * 3, 0, sh.z - rel[2]]
      : [-rel[0], 0, mouthZ - rel[2]];
    const n = Math.hypot(toward[0], toward[2]);
    if (n < 1e-4) return;
    const push = 4.0 * best.mass * g;
    this.debugFeed.indexed++;
    best.body.wakeUp();
    const w = this.toWorld([(toward[0] / n) * push, 0, (toward[2] / n) * push]);
    // A REAL INDEXER LIFTS AS WELL AS SHOVES.
    //
    // It is a wheel pressed against the ball, and its contact patch is below the ball's
    // centre, so it rolls the ball up and over as it sweeps it along. A purely horizontal
    // push -- which is what this was -- is the one thing the hardware cannot do, and it
    // does not work even in principle: a ball resting on the bin floor is pinned by its own
    // floor contact, and 0.95 N of horizontal push moves it nothing at all. Probing it
    // showed a 0.5 m/s velocity kick erased inside a single step while the ball had no
    // contacts but the floor.
    //
    // The consequence was not a dead feed but an intermittent one, which is worse, because
    // it looked like tuning. Balls reached the shaft only when something else happened to
    // knock them there, so the feed rate depended on how much the robot was being shaken --
    // which is why a shot table that fired LESS OFTEN fed itself even less and ended a
    // 70 s run with six balls in the bin and none in the magazine.
    best.body.addForce({ x: w[0], y: (tp.indexLift ?? 0.6) * push, z: w[2] }, true);
  }

  /** Is a ball across the gate plate's plane, so the plate cannot close? */
  private gateBlocked(balls: BallSet): boolean {
    const p = this.pos;
    const sh = this.shaft;
    return this.inShaft.some((b) => {
      const q = balls.pos(b);
      const y = this.toLocal([q[0] - p[0], q[1] - p[1], q[2] - p[2]])[1];
      return Math.abs(y - sh.gateY) < b.radius + sh.wall / 2 + 0.002;
    });
  }

  /**
   * The nip: where the flywheel meets the ball at the top of the shaft.
   *
   * This is the one place still modelled as an impulse rather than as contact, and the
   * reason is numerical rather than convenience. The wheel turns at ~3500 rpm, so the nip
   * opens and closes in about 400 microseconds; resolving that as contact needs a timestep
   * two orders of magnitude below the 1/240 s this runs at, and at 1/240 the ball simply
   * passes through the wheel between steps. What IS real is everything that decides whether
   * a shot happens at all: a ball has to physically be at the top of the tube, the wheel has
   * to actually be turning, and the energy comes out of the wheel own inertia.
   */
  private stepNip(balls: BallSet, t: number): void {
    if (this.flywheelOmega < 20) return;
    // THE GATE HAS TO MEAN SOMETHING. This fired whatever was sitting at the nip as soon as
    // the cycle timer came round, and never looked at the gate -- so the gate only decided
    // whether the NEXT ball climbed, and the one already in the chamber went whatever the
    // readiness check said. Every gate in the brain (the opening cap, P(land), the release
    // re-check) was therefore advisory for the very next shot.
    //
    // Measured: crossing the front of the hive at 1.55 m/s, three shots left at 81-83 in and
    // 62-63 deg off the opening with P(land) 0.12-0.38 -- past the cap and far under the
    // threshold at the moment they went -- while the hold string said "DRIVE ROUND" for 47%
    // of the run. None of them landed. The gate was closed; the ball went anyway.
    //
    // The servo is the release, as `Transfer` on the hub has it: a ball at the nip goes when
    // the gate is open and waits when it is not.
    //
    // ONE BALL PER PULSE, NOT ONE BALL PER CLOCK AND NOT ONE PER SERVO POSITION.
    //
    // Two things used to decide the release besides the ball being at the wheel, and both
    // were clocks racing each other. `sinceFeed`, reset at the previous RELEASE, had to reach
    // cycleTime_s -- while the brain runs the same 0.6 s from the previous COMMIT, so the two
    // ran at one period with a different phase. And the servo had to still be past half travel
    // at the instant the ball arrived: a ball waiting ABOVE the plate arrived 0.13 s after the
    // commit (the servo reaching 0.5) and went; a ball waiting UNDER it needed the plate to
    // clear (0.19 s) and then 80 mm of climb (0.2 s), arrived at 0.38-0.40 s, and found the
    // servo already back through 0.5 at 0.375 s. Measured with tools/releasecheck.ts on the
    // 40 in patrol: delays of 0.13 OR 0.38 s and nothing between, and 13 of 41 pulses released
    // NOTHING -- each a wasted 0.6 s cycle. Standing still it was 107 of 118.
    //
    // Transfer.java is the authority on the cycle time and has one clock. What the world owes
    // it is that one pulse feeds one ball: a latch, armed when the servo opens, spent by the
    // launch, and expired once the servo is fully shut. A ball the pulse admitted fires when
    // it reaches the wheel whatever the servo is doing by then; a ball arriving after the gate
    // has shut waits for the next pulse; and a second ball cannot follow the first out because
    // the plate (which now waits for the first to pass, see stepTransfer) holds it.
    const gate = this.servos.get('gate');
    if (gate && this.spec.transfer.gate?.enabled !== false) {
      const open = this.spec.transfer.gate.open ?? 1;
      const closed = this.spec.transfer.gate.closed ?? 0;
      const isOpen = Math.abs(gate.pos - open) <= Math.abs(gate.pos - closed);
      // FULLY SHUT, then open, is a pulse. Half-shut and back up is not: the brain's release
      // re-check can drop the servo for a frame or two mid-pulse, and counting that as a
      // second opening fed a second ball on the same request (tools/releasecheck.ts).
      if (Math.abs(gate.pos - closed) < 0.05) { this.gateArmed = false; this.gateShut = true; }
      else if (isOpen && this.gateShut) { this.gateArmed = true; this.gateShut = false; }
      if (!this.gateArmed) return;
    } else if (this.sinceFeed < this.spec.transfer.cycleTime_s) {
      return;                      // no gate modelled: the clock is all there is
    }
    const p = this.pos;
    for (const b of this.inShaft) {
      const bp = balls.pos(b);
      const rel = this.toLocal([bp[0] - p[0], bp[1] - p[1], bp[2] - p[2]]);
      if (rel[1] < this.shaft.topY - b.radius) continue;
      this.launch(b, balls, t);
      this.sinceFeed = 0;
      this.gateArmed = false;
      return;               // one ball per pass, like one ball per wheel revolution
    }
  }

  /**
   * Where a ball may actually be released without starting inside the chassis.
   * `robot.json` can perfectly well put the muzzle inside the box (the default 0.28 m
   * muzzle height sits inside a 0.30 m tall chassis); a ball spawned there gets flung
   * sideways by the contact solver, which looks exactly like a mis-aimed shooter.
   * So the exit point is walked along the shot direction until it clears the box.
   */
  private exitPoint(muzzleWorld: Vec3, dirWorld: Vec3, ballR: number): Vec3 {
    const c = this.spec.chassis;
    const p = this.pos;
    const local = this.toLocal([muzzleWorld[0] - p[0], muzzleWorld[1] - p[1], muzzleWorld[2] - p[2]]);
    const d = this.toLocal(dirWorld);
    const half: Vec3 = [c.width_m / 2 + ballR, c.height_m / 2 + ballR, c.length_m / 2 + ballR];

    // Ray-box exit for a ray starting INSIDE the box: the smallest distance at which it
    // crosses any slab's far plane, not the largest. (Taking the max launched the ball
    // 0.6 m up the barrel and every shot sailed over the HIVE.)
    let s = Infinity;
    for (let i = 0; i < 3; i++) {
      if (Math.abs(local[i]) > half[i]) return muzzleWorld; // already outside
      if (Math.abs(d[i]) < 1e-9) continue;
      const t = ((d[i] > 0 ? half[i] : -half[i]) - local[i]) / d[i];
      if (t >= 0 && t < s) s = t;
    }
    if (!Number.isFinite(s)) return muzzleWorld;
    s += 1e-3;
    return [muzzleWorld[0] + dirWorld[0] * s, muzzleWorld[1] + dirWorld[1] * s, muzzleWorld[2] + dirWorld[2] * s];
  }

  /** Muzzle position and aim direction in world coordinates. */
  muzzle(): { pos: Vec3; dir: Vec3; azimuth: number; elevation: number } {
    const t = this.spec.turret;
    const c = this.spec.chassis;
    const az = this.yaw + (t.enabled ? this.turretAngle * DEG : 0);
    const el = this.hoodAngle * DEG;
    const p = this.pos;
    const pivotLocal: Vec3 = [0, t.muzzleHeight_m - c.height_m / 2, 0];
    const pw = this.toWorld(pivotLocal);
    const pos: Vec3 = [
      p[0] + pw[0] + t.muzzleOffset_m * Math.sin(az),
      p[1] + pw[1],
      p[2] + pw[2] + t.muzzleOffset_m * Math.cos(az),
    ];
    const dir: Vec3 = [Math.sin(az) * Math.cos(el), Math.sin(el), Math.cos(az) * Math.cos(el)];
    return { pos, dir, azimuth: az, elevation: el };
  }

  private launch(b: Ball, balls: BallSet, t: number): void {
    const f = this.spec.flywheel;
    const mz = this.muzzle();
    const I = f.I_fly_kgm2 + this.motors.get('flywheel')!.model.inertia;

    // Captured before the shot takes energy out of the wheel, further down: reading it
    // afterwards reports the dip, which made every shot look like it went out off-speed.
    this.lastShotRpm = this.flywheelRpm;

    let vExit = f.k * this.flywheelOmega * f.r_fly_m;
    // Scatter: the single biggest reason a real shooter misses.
    const s = f.scatter;
    vExit *= 1 + this.rng.gauss(0, s.speedFrac);
    const dEl = this.rng.gauss(0, s.angle_deg) * DEG;
    const dAz = this.rng.gauss(0, s.yaw_deg) * DEG;
    const az = mz.azimuth + dAz;
    const el = mz.elevation + dEl;
    const dir: Vec3 = [Math.sin(az) * Math.cos(el), Math.sin(el), Math.cos(az) * Math.cos(el)];

    // THE BALL LEAVES WITH THE MUZZLE'S VELOCITY, NOT THE CHASSIS'S: v_muzzle = v_cg + omega x r.
    // This used to add body.linvel() alone, which is the velocity of the tracked point at the
    // centre of the chassis. The muzzle sits muzzleOffset_m out from the turret axis, so a robot
    // yawing at 90 deg/s swings it at omega*r = 0.19 m/s of its own -- 4.7 in of lateral miss at
    // 60 in, and NOT a miss any aiming code could correct, because the physics the shot was aimed
    // at was not the physics the ball got. builtinTeleOp.muzzleVelocity() / ShotLead.muzzleVelocity()
    // are the other half of this and have to move with it.
    //
    // The lever arm is the MUZZLE, not exitPoint() below: that offset exists to spawn the ball
    // clear of the chassis collider and is not a claim about where it leaves the hood.
    const cv = this.body.linvel();
    const w = this.body.angvel();
    const p0 = this.pos;
    const rx = mz.pos[0] - p0[0], ry = mz.pos[1] - p0[1], rz = mz.pos[2] - p0[2];
    const mvx = cv.x + (w.y * rz - w.z * ry);
    const mvy = cv.y + (w.z * rx - w.x * rz);
    const mvz = cv.z + (w.x * ry - w.y * rx);
    const vel: Vec3 = [dir[0] * vExit + mvx, dir[1] * vExit + mvy, dir[2] * vExit + mvz];

    // Backspin for a single-wheel shooter: axis horizontal and left of the shot, so the
    // Magnus force in aero.ts (w_hat x v) points up.
    let spin: Vec3 = [0, 0, 0];
    if (f.type === 'single' && vExit > 0.1) {
      const mag = vExit / b.radius;
      const n = Math.hypot(dir[0], dir[2]) || 1;
      spin = [(-dir[2] / n) * mag, 0, (dir[0] / n) * mag];
    }

    balls.release(b, this.exitPoint(mz.pos, dir, b.radius), vel, spin, 'flight');

    // The shot costs the flywheel energy: omega -= lossFactor * KE_ball / (I * omega).
    if (this.flywheelOmega > 1) {
      const ke = 0.5 * b.mass * vExit * vExit;
      this.flywheelOmega = Math.max(0, this.flywheelOmega - (f.lossFactor * ke) / (I * this.flywheelOmega));
      this.motors.get('flywheel')!.omega = this.flywheelOmega;
    }
    this.shots++;
    this.lastShotBallId = b.id;
    this.lastShot = { v_exit: vExit, elevDeg: el * RAD, azDeg: az * RAD, spin: Math.hypot(spin[0], spin[2]), t };
  }

  // ------------------------------------------------------------------ io

  /**
   * What the brain reads back. The wire is in the motor's ELECTRICAL frame, so a motor
   * wired backwards reports its encoder backwards too -- exactly as a hub does, where
   * positive power always makes the position count up. Reporting the physical frame
   * instead makes a reversed wheel's encoder cancel its partner's, and a drivetrain whose
   * average travel is always zero drives until it times out.
   */
  motorState(name: string): { pos: number; vel: number; amps: number } {
    const m = this.motors.get(name);
    if (!m) return { pos: 0, vel: 0, amps: 0 };
    const sign = m.model.reversed ? -1 : 1;
    const raw = radToTicks(m.model, m.rad);
    return { pos: Math.round(m.loop.position(raw)) * sign, vel: m.loop.reportedVel * sign, amps: m.amps };
  }

  isBusy(name: string): boolean {
    return this.motors.get(name)?.loop.busy ?? false;
  }

  /** Ground truth, for the shot log and the analysis. The brain does not get this. */
  get flywheelRpm(): number {
    return radSToRpm(this.flywheelOmega);
  }

  /**
   * What the hub's velocity estimate says the flywheel is doing -- counts over a 20 ms
   * window, quantised to whole encoder ticks, which at this gearing is worth about 107 rpm
   * a count. This is what the readiness gate is allowed to see, because it is what
   * `motor.getVelocity()` returns on the robot. Feeding it the true omega made every
   * conclusion about firing windows unportable.
   */
  get reportedFlywheelRpm(): number {
    const m = this.motors.get('flywheel');
    if (!m) return 0;
    const cpr = this.spec.flywheel.encoderTicksPerRev;
    if (!cpr) return (m.loop.reportedVel * 60) / m.model.ticksPerRev;
    return (this.flyEnc.vel * 60) / cpr;
  }

  /**
   * The flywheel's OWN encoder, which is not the one the hub's velocity PID reads.
   *
   * Two different measurements that used to be one. A REV hub runs RUN_USING_ENCODER off the
   * motor's built-in encoder and nothing else -- so that loop, its PIDF gains and its
   * `maxTicksPerSec` scaling all stay on the 5202's 28 ticks a rev, and pointing them at an
   * external encoder makes the hub's own `f` term 73x too strong (measured: the wheel settled
   * at 5926 rpm against a 2800 target).
   *
   * A Through Bore on the WHEEL shaft is wired to a spare encoder port and read by the team's
   * code, which is what `BuiltinTeleOp` does: RUN_WITHOUT_ENCODER plus its own feedforward and
   * P term, closed on this number. That is the reading the readiness gate tests, and it is the
   * one worth making fine: 28 ticks a rev is a 107 rpm lattice over the hub's 20 ms window,
   * which is wider than the whole 60 rpm readiness window.
   */
  private flyEnc = { hist: [] as { t: number; ticks: number }[], vel: 0 };

  private sampleFlywheelEncoder(t: number): void {
    const cpr = this.spec.flywheel.encoderTicksPerRev;
    if (!cpr) return;
    const m = this.motors.get('flywheel');
    if (!m) return;
    // Same whole-count-over-a-window model as the hub's, because that is how any quadrature
    // count is read; only the resolution differs.
    const raw = (m.rad / (2 * Math.PI)) * cpr;
    this.flyEnc.hist.push({ t, ticks: Math.floor(raw) });
    const window = this.spec.hub.encoderVelocityWindowMs / 1000;
    while (this.flyEnc.hist.length > 2 && t - this.flyEnc.hist[0].t > window) this.flyEnc.hist.shift();
    const first = this.flyEnc.hist[0];
    const dt = t - first.t;
    if (dt > 1e-6) this.flyEnc.vel = Math.round((raw - first.ticks) / dt);
  }

  targetFlywheelRpm(act: ActuatorFrame): number {
    const cmd = act.motors?.['flywheel'];
    if (!cmd) return 0;
    const m = this.motors.get('flywheel')!;
    if (cmd.velocity !== undefined) return (cmd.velocity * 60) / m.model.ticksPerRev;
    return (cmd.power ?? 0) * radSToRpm(m.model.freeOmega);
  }

  snapshot(volts: number, soc: number, amps: number, ftc: { x: number; y: number; heading: number }): Snapshot['robot'] {
    const v = this.body.linvel();
    const p = this.pos;
    const hopperKinds: BallKind[] = [...this.hopper, ...this.inShaft].map((b) => b.kind);
    return {
      p,
      yawDeg: this.yaw * RAD,
      v: [v.x, v.y, v.z],
      speed: Math.hypot(v.x, v.z),
      omegaDps: this.body.angvel().y * RAD,
      ftc,
      wheels: DRIVE_WHEELS.map((w) => {
        const m = this.motors.get(w)!;
        return { name: w, cmd: m.duty, omega: m.omega, torque_Nm: m.torque, force_N: this.wheelForce[w], normal_N: this.wheelNormal[w], slip: this.wheelSlip[w], amps: m.amps };
      }),
      battery: { volts, soc, amps },
      intake: { power: this.motors.get('intake')!.duty, omega: this.motors.get('intake')!.omega, inLine: this.inShaft.length, transit: [] },
      // Everything the robot is holding, bin plus magazine. Reporting only the bin said
      // "hopper empty" while a ball was still sitting at the nip about to be fired.
      hopper: { count: this.hopper.length + this.inShaft.length, capacity: this.spec.hopper.capacity, kinds: hopperKinds },
      transfer: { sinceFeed: this.sinceFeed, cycleTime: this.spec.transfer.cycleTime_s, ready: this.sinceFeed >= this.spec.transfer.cycleTime_s, gate: this.servos.get('gate')!.pos },
      turret: { angleDeg: this.turretAngle, targetDeg: this.turretTargetDeg, omegaDps: this.turretOmega, atLimit: this.turretAtLimit },
      hood: { angleDeg: this.hoodAngle },
      flywheel: { rpm: this.flywheelRpm, targetRpm: 0, amps: this.motors.get('flywheel')!.amps, shots: this.shots },
      lastShot: this.lastShot,
      flags: this.flags,
    };
  }

  /**
   * Put a ball in the bin: match preloads, the practice auto-fill, and tests.
   *
   * It PLACES the ball, it does not append to a list. The ball is dropped into the first
   * free slot in the bin and then it is on its own -- if the bin is full it will not fit,
   * and the refusal comes from the geometry rather than from a capacity check.
   */
  preload(balls: BallSet, b: Ball): boolean {
    this.lastCensus = balls;
    // NOT guarded on isEnabled() on purpose. Parking is how a harness clears the field, and
    // `park everything, then preload N` is the standard rig in tests/shoot.test.ts and half
    // the tools -- preload's whole job there is to bring a benched ball back into play.
    //
    // The thing that must not pick up a parked ball is a positional SEARCH for a loose ball
    // on the tiles: park() benches them below the floor, so `state === 'free'` plus a low Y
    // matches the entire bench. Those call sites check isEnabled(); this one must not.
    if (this.hopper.length + this.inShaft.length >= this.spec.hopper.capacity) return false;
    const sh = this.shaft;
    // Six slots around the tube on the bin floor, then a second layer on top of them: a
    // 43 cm bin holds more than six of these, and a preload that quietly fails because it
    // only knew about one layer is a hopper that will not fill.
    const slots: Vec3[] = [];
    const step = b.radius * 2.2;
    for (const layer of [0, 1]) {
      for (const sz of [1, -1]) {
        for (const sx of [0, 1, -1]) {
          slots.push([
            sx * step,
            this.binFloorY + b.radius + 0.005 + layer * b.radius * 2.1,
            // RETRACTED BUG REPORT, left here because the mistake is worth not repeating.
            //
            // This was written up as "the robot cannot hold a magazine": place it, preload
            // four POLLEN, arm nothing, and three of the four appeared to leave through the
            // intake mouth. They did not. The probe placed the robot at z = 2.38 m against a
            // 1.80 m field half-width -- outside the wall, where there is no floor. The robot
            // fell, the balls fell with it, and their UNCHANGED position relative to a
            // departing robot was read as balls leaving the robot. tools/landrate.ts has the
            // on-field guard; the probe did not. Placed legally, all four sit still.
            //
            // The relative frame is the trap: "the balls did not move relative to the robot"
            // and "the balls left the robot" look identical if the robot is what moved.
            sh.z + sz * (sh.half + b.radius + 0.012 + layer * 0.004),
          ]);
        }
      }
    }
    const p = this.pos;
    for (const local of slots) {
      const w = this.toWorld(local);
      const at: Vec3 = [p[0] + w[0], p[1] + w[1], p[2] + w[2]];
      // Do not stack one on top of another: a ball spawned inside another one gets flung
      // out of the robot by the contact solver.
      const clash = [...this.hopper, ...this.inShaft].some((o) => {
        const q = balls.pos(o);
        return Math.hypot(q[0] - at[0], q[1] - at[1], q[2] - at[2]) < b.radius * 2;
      });
      if (clash) continue;
      // AT THE BIN'S VELOCITY, NOT AT REST. A ball dropped in stationary while the chassis
      // is moving is instantly a ball travelling backwards relative to the bin: it slams into
      // the wall and the contact solver throws it out. Standing still nothing notices, which
      // is why every stationary rig has been fine -- but a harness reloading a robot at
      // 1.5 m/s fed it 800 balls to fire 4 (tools/fastfire.ts), because each one left as fast
      // as it arrived. Same term as launch(): v_cg + omega x r at the slot.
      const cv = this.body.linvel();
      const av = this.body.angvel();
      const rx = at[0] - p[0], ry = at[1] - p[1], rz = at[2] - p[2];
      const bv: Vec3 = [
        cv.x + (av.y * rz - av.z * ry),
        cv.y + (av.z * rx - av.x * rz),
        cv.z + (av.x * ry - av.y * rx),
      ];
      balls.release(b, at, bv, [0, 0, 0], 'hopper');
      this.hopper.push(b);
      return true;
    }
    return false;
  }

  /** Balls the robot is holding, so the world can put them back on a reset. */
  /** The set the census last ran over, so place() knows what it is carrying. */
  private lastCensus: BallSet | null = null;

  heldBalls(): Ball[] {
    return [...this.hopper, ...this.inShaft];
  }

  reset(): void {
    this.body.setTranslation({ x: this.startPose.p[0], y: this.startPose.p[1], z: this.startPose.p[2] }, true);
    this.body.setRotation(quatY(this.startPose.yaw), true);
    this.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    this.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    for (const m of this.motors.values()) {
      m.omega = 0;
      m.rad = 0;
      m.duty = 0;
      m.torque = 0;
      m.amps = 0;
      m.loop.reset();
    }
    this.hopper = [];
    this.inShaft = [];
    this.sinceFeed = 999;
    this.gateArmed = false;
    this.gateShut = true;
    this.turretAngle = 0;
    this.turretOmega = 0;
    this.turretTargetDeg = 0;
    this.turretAtLimit = false;
    this.flywheelOmega = 0;
    this.shots = 0;
    this.lastShot = null;
    this.lastLocalVel = [0, 0];
    this.accel = [0, 0];
  }

  /** Teleport, for the scenario editor and for "put me back on the wall". */
  /**
   * TELEPORT THE ROBOT AND WHAT IS INSIDE IT.
   *
   * The hopper is a census by POSITION -- a ball counts as held when it is inside the
   * chassis box, which is why an intake that misses is an intake that misses. So moving
   * the chassis alone leaves the balls standing in the old spot, and one step later the
   * census calls them free: a robot placed with four preloaded POLLEN had an empty hopper
   * by the next frame. Every measurement rig papered over that by re-preloading each step,
   * which meant none of them could see it, and a rig that quietly refills the magazine is
   * not measuring the shooter it claims to.
   */
  place(p: Vec3, yawDeg: number): void {
    const carried = this.lastCensus
      ? this.heldBalls().map((b) => {
        const bp = this.lastCensus!.pos(b);
        const c = this.pos;
        return { b, local: this.toLocal([bp[0] - c[0], bp[1] - c[1], bp[2] - c[2]]) };
      })
      : [];
    this.body.setTranslation({ x: p[0], y: p[1], z: p[2] }, true);
    this.body.setRotation(quatY(yawDeg * DEG), true);
    this.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    this.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    for (const { b, local } of carried) {
      const w = this.toWorld(local);
      this.lastCensus!.release(b, [w[0] + p[0], w[1] + p[1], w[2] + p[2]], [0, 0, 0], [0, 0, 0], b.state);
    }
  }
}

// Put the turret at an angle without running its servo. Measurement rigs only.
export function quatY(a: number): { x: number; y: number; z: number; w: number } {
  return { x: 0, y: Math.sin(a / 2), z: 0, w: Math.cos(a / 2) };
}

const mid = (r: [number, number]) => (r[0] + r[1]) / 2;

export const rpmToTicksPerSec = (rpm: number, ticksPerRev: number) => (rpm * ticksPerRev) / 60;
export const inchesOf = (m: number) => m * M_TO_IN;
export const rpmOf = rpmToRadS;
