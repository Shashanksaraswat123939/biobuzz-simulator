# What the simulator actually models, and where each rule lives

Every claim here names the file that implements it. If a claim and the code disagree, the
code is right and this file is stale — say so.

Nothing physical is written in code. Every constant is read from `config/params.json`,
`config/robot.json` or `config/motors.json`, each value carries a `_source` saying whether
it was measured, taken from the CAD, or guessed, and the **Variables** tab writes to those
same objects while the world is running.

---

## 1. Solver

| | |
|---|---|
| Engine | Rapier 3D 0.20 (`@dimforge/rapier3d-compat`), WASM |
| Step | `params.sim.dt` = 1/240 s, `substepsPerFrame` = 4 → one animation frame is 1/60 s |
| Solver iterations | `params.sim.solverIterations` |
| CCD | on balls only (`params.sim.ccdOnBalls`) — a ball at 14 m/s moves 58 mm per step, more than its own radius |
| Determinism | same seed + same command log ⇒ bit-identical snapshots (`tests/determinism.test.ts`) |

**Collision groups** — `packages/core/src/physics/groups.ts`. The rocker collides with balls
only. The real A-frame is open where the rocker swings, and any convex approximation of it
jams the pocket against a panel. The robot cannot reach the rocker anyway: R102 caps
expansion at 29 in and the pivot is at 44 in.

**Mass must go on the collider.** `RigidBodyDesc.setAdditionalMass` and
`setAdditionalMassProperties` are silently ignored in this Rapier build — the body comes out
with mass 0 and invMass 0, i.e. immovable, and no force does anything. Every mass in this
project is set with `ColliderDesc.setMassProperties`.

**Friction and restitution are combined with `Min`, not Rapier's default `Average`.** The
chassis shell against 0.85 tile friction averaged to 0.435, which dragged the robot at about
60 N — most of the drivetrain's output — and held it to half its top speed.

---

## 2. Drivetrain — `packages/core/src/physics/robot.ts`

Not a velocity you set. A torque that makes a force that accelerates a mass.

1. **Motor** (`motors.ts`): each DC motor is a first-order model — stall torque, free speed,
   winding resistance, rotor inertia — from `config/motors.json`. Torque falls linearly with
   speed and with the battery voltage it actually has.
2. **Hub loop** (`hub.ts`): `RUN_USING_ENCODER` and `RUN_TO_POSITION` run a PIDF exactly as
   the Control Hub does, at `hub.loopPeriodMs`, with encoder velocity measured over
   `encoderVelocityWindowMs` and command latency of `commandLatencyMs`. Anti-windup is
   conditional integration.
3. **Mecanum kinematics**: wheel speeds → chassis motion through the Jacobian transpose.
4. **Tyre model**: each wheel's force comes from the slip between its commanded surface
   speed and the ground speed under it, capped by `µ·N`. `N` includes weight transfer from
   the previous step's acceleration. This is why flooring it on low `env.tileMu` spins the
   wheels instead of accelerating, and why strafing is about 70% as efficient as driving
   straight.
5. **Battery** (`battery.ts`): total current sags the pack by `I·Rint`, and the sag feeds
   back into every motor's available torque next step.

Measured top speed 62.8 in/s against 62 in/s hand-computed from the motor curve
(`tests/drivetrain.test.ts`).

---

## 3. Intake, hopper and feed — all contact, no state machine

This was a scripted pipeline once: a ball that touched a trigger box vanished and reappeared
as a number in an array. It is now geometry and forces, end to end. `tools/mechcheck.ts`
drives each stage separately and reports where a ball actually got to.

**The chassis is built out of plates**, not one cuboid: floor, two sides, a back, a front
wall that stops *above* the intake mouth, and a vertical feed tube up the turret axis. A ball
is a rigid body the whole way through the robot.

**Intake** (`stepIntake`). The roller does not "capture" anything. Slip between the roller's
surface speed and the ball's own speed produces a friction force capped by `rollerMu · N`,
where `N` is the ball's weight plus `squeeze_N` from the roller being preloaded onto it —
the same slip-and-cap form as the tyres. Consequences that nobody wrote down: running the
roller backwards ejects a ball through the same code, chasing a ball at roller speed picks up
nothing, and a stalled roller has no surface speed and therefore no grip, which is what a jam
is. The roller also presses the ball *up* while it is still below the bin floor and *down*
once it is over the lip.

> There is no ramp. There was one; a plate reaching down to tile level to catch a ball also
> drags on the tile, and it cost three quarters of the robot's top speed. A real
> over-the-bumper intake does not use one either — the compliant roller carries the ball over
> the floor plate's edge. A 71 mm ball cannot escape under a 20 mm ground clearance.

**Hopper** (`censusBalls`). There is no add/remove bookkeeping. A ball is in the hopper
because its centre is inside the hopper, recomputed every step. Capacity is emergent: the bin
holds what fits. `hopper.capacity` is a rules figure and a preload guard, not a gate.

**Feed tube.** A square bore up the turret's rotation axis, because a turret has to be fed on
its own axis. Three details each cost an afternoon:

- **Open front *and* back.** With one opening, any ball that ended up behind the tube was
  unreachable and sat in the bin for the whole match.
- **Walls run past the nip to a stop.** Ending them at the nip left a ball-sized gap on all
  four sides exactly where the ball arrives at 0.8 m/s.
- **Bored for one POLLEN, not for NECTAR.** Sizing it to take the larger element made the
  bore 4.1 in, and two 2.8 in POLLEN wedge diagonally in 4.1 in. A single-file magazine is
  single file or it is not a magazine. NECTAR will not enter, which is correct.

**Indexer** (`stepTransfer`). Metered to one ball at a time, admitted only when the entry has
cleared by a ball's width — pushing every eligible ball at once sent two in through opposite
openings on the same step and jammed the bore solid. It steers to the nearer *opening*, not
to the tube's axis; aiming at the axis pressed balls into a solid side wall.

**Gate** (`gateCollider`). An actual plate across the bore, enabled and disabled by the servo.
The belt runs continuously and the gate is the release, so the magazine stays loaded between
shots. Gating the belt instead emptied the tube back into the bin every cycle.

Two rules make one pulse mean one ball, and both were found by watching the tube frame by
frame (`tools/feedprobe.ts`, `tools/releasecheck.ts`):

- **The plate cannot close through a ball.** The collider used to come back on a clock, 0.31 s
  after the pulse, while the belt was still lifting the admitted ball through its plane; the
  solver then threw the ball whichever way was nearer, sometimes back under the gate for a whole
  cycle. It now waits for the ball to pass.
- **A pulse owes exactly one ball.** The nip used to require the servo to still be past half
  travel at the instant the ball arrived, on top of its own 0.6 s clock reset at the previous
  *release* — while the brain runs the same 0.6 s from the previous *commit*. A ball waiting
  above the plate went 0.13 s after the commit; one waiting under it needed the plate to clear
  and 80 mm of climb, arrived at 0.38–0.40 s, and found the servo back through half travel at
  0.375 s. Delays were 0.13 s or 0.38 s and nothing between, and a third of the pulses on the
  40 in patrol released nothing. The release is now a latch: armed when the servo opens from
  fully shut, spent by the launch, expired once it is fully shut again. Every pulse fires one
  ball, 0.12 s after the commit with a ball staged.

**Nip** (`stepNip`). **This is the one place still modelled as an impulse rather than as
contact, and the reason is numerical.** The wheel turns at ~3500 rpm, so the nip opens and
closes in about 400 µs; resolving that as contact needs a timestep two orders of magnitude
below 1/240 s, and at 1/240 s the ball passes through the wheel between steps. Everything
that *decides* whether a shot happens is real: a ball has to physically be at the top of the
tube, the wheel has to be turning, and the energy comes out of the wheel's own inertia.

---

## 4. Shooter

**Exit speed** `v = k · ω · r_fly`, and the shot costs the wheel
`lossFactor · KE_ball / (I · ω)` — so the wheel visibly dips after each shot and the cycle
rate is limited by how fast it recovers (`tests/shoot.test.ts`).

**Scatter** (`flywheel.scatter`): Gaussian noise on elevation, yaw and speed at the moment of
release. This is the single largest cause of a wide group, and the Predictor says so.

**Flight** (`ballistics.ts`): explicit integration at 1/480 s with

- gravity,
- quadratic drag `½ρ C_d A v²`,
- Magnus lift `½ρ C_l A v²` along `ŵ × v̂`, with `C_l = clSlope · S` saturating at `clSMax`,
- spin decay per step.

`C_d` and `clSlope` are **guesses** and labelled as such — a 26-hole hollow ball is not a
smooth sphere.

**Aim** (`builtinTeleOp.ts` / `ShotLead.java`). The ball leaves with `v_exit·d̂ + v_robot`, so
the shot is solved for the velocity the **ball** must have in the field frame — which is the
table's answer as a *vector* — with the robot's own velocity subtracted from it. Three
components, three unknowns, so all three are solved:

```
horiz = S·cos(el) along the bearing        vert = S·sin(el)
mag   = |horiz·b̂ − v_robot|
                                azimuth = ∠(horiz·b̂ − v_robot)
                                el      = atan2(vert, mag)
                                speed   = hypot(mag, vert)
```

> **The vertical is part of the answer.** This solved the horizontal triangle only and left
> the hood at the table's angle, so the ball went out with a vertical of `mag·tan(el)` rather
> than `S·sin(el)`: the ground track exact, the hang time wrong. Closing at 0.4 m/s from 40 in
> that drops the exit speed from 5.28 to 4.09 m/s at a fixed 70° hood, the vertical from 4.97
> to 3.85 — and the ball **never reaches** the mouth's 1.46 m. Not a miss; a shot that cannot
> arrive. Retreating sailed over it the same way. `tools/leadcheck.ts` prints both, and the
> corrected lead is exact to the centimetre at every velocity and range it covers.

That also all but takes the flywheel out of it. Over ±0.8 m/s of closing speed at 40 in the
old lead swung the target 1283–3388 rpm against a wheel that slews 1102 rpm/s; this one asks
for 2241–2477, and gives the rest to a hood servo that is **commanded** rather than measured
and tracks 15 m/s² of radial acceleration against the flywheel's 1.2.

Only flight acceleration is left uncompensated: over a one-second flight the `a·t²` term is
small next to 1–2° of launch scatter. `transfer.leadLatency_s` is a different thing — it
predicts the velocity at *release*, and now that the wheel barely moves it is worth nothing
either way (0.36 landed per second at τ = 0, 0.15 and 0.3 alike). It is kept because a real
hood will lag in a way this one does not.

**And the permission has to still be good when the ball leaves.** The feed commits about four
tenths of a second before release — the pulse plus the climb up the tube — and the gate shuts
again if the turret runs out of travel or falls behind in the meantime. Not the full readiness
latch, which flickers on a quantised tachometer, and not the wheel, whose floor is unmeetable
for a robot whose range is growing.

> **`turretErrDeg` is measured against the CLAMPED command**, so an axis pinned on its ±120°
> stop used to report a fraction of a degree of error. Spinning, the lead asked for a bearing
> 17.7 ± 18.5° outside the travel, the gate called it aimed, and shots went out up to 50° wide.
> The aim keeps what the lead ASKED for and refuses the shot when the clamp bites —
> `TurretTracker.canReach` on the hub always did this; the mirror did not.

**The hood has to have arrived.** It is the axis carrying the correction now, so the readiness
gate waits for it — `hood.tolDeg`, 2°, derived from what the wheel is already allowed (60 rpm
is ±21 cm of range, 1° of hood is 3.5 cm, so the RPM window is worth about 6° of hood). A servo
is *commanded*, not measured, so this is readiness rather than another factor in the
probability: once it is there the ball leaves with the table's launch vector and the speed band
measured standing still applies again. It closes on under 1% of loops.

> The azimuth **must** be wrapped. `atan2` returns (−180, 180] and the heading is subtracted
> from it, so the result can land anywhere in (−540, 540). Unwrapped, a bearing of +90° came
> out as −270°, clamped to the turret's −120° limit, and the robot fired over the wall. Every
> azimuth crossing the ±180° seam did this. `tests/shotlead.test.ts` guards it.

**Turret axis**: trapezoidal motion profile with real velocity and acceleration limits, so a
137° swing takes most of a second. Firing on the *commanded* angle instead of the encoder
angle means firing at nothing.

**Shot table** (`tools/shottable.ts`): for each range, the hood angle whose feasible exit-speed
band is widest. The aperture is modelled as a near lip to clear and a far lip to stay under —
aiming at the mouth's centre is not enough, because a ball can pass through that point while
still climbing and clip the near lip.

**Will this shot land?** `P(exit speed threads the mouth) × P(pointing inside it) × P(it stays
in)`, put through a measured score-to-frequency curve (`config/landcal.json`,
`tools/landcal.ts`) so `flywheel.minLandProb` is a probability and not a score with a percent
sign. Both the simulator's mirror and `AimController` on the hub compute it — the Java copy had
been sitting there fully implemented, self-checked, and called by nothing, so the deliverable's
only gate was the RPM window.

> **A threshold above the ceiling is not caution, it is a robot that will not shoot.** This was
> 0.900 against a measured ceiling of 0.898, so it could be satisfied only by rounding: 0.00
> landed per second standing still at 50 in where the same robot with the gate open landed 0.45.
> `tools/movingfire.ts --sweep` prices the threshold in *balls per second* — refusing a 60%
> shot only pays if a better one arrives inside the 1.5 s cycle it costs — and 0.85 gives up
> nothing while taking the hit rate of the shots it allows from 64% to 90–100%. The curve is
> now fitted over three driving states as well as three ranges, because the gate is used on a
> robot that is usually moving: it lands 80% standing still and 76% on the move.

**Calibration** (`robot.calibration`): `rangeTrim_in` and `turretTrim_deg`. The table is
looked up at `(range − rangeTrim)`, so a group landing 8 in long gets a trim of +8. The
Analysis tab computes both from a collected run and the **Apply calibration** button writes
them. Trims accumulate, because each run measures what is left after the last one.

---

## 5. HIVE — `packages/core/src/physics/hive.ts`

An over-centre see-saw on a revolute joint. At θ = 0 its CG is directly above the pivot, so
gravity's restoring torque is `m·g·r·sin θ` — about **0.62 N·m** at the ±30.04° stops. A ball
in the up CELL pushes back with `m·g·z`, where `z` is its horizontal distance from the pivot
axis. When the balls win, it goes over.

**Nothing scripts the tip.** There is no ball counter and no threshold. The Robot tab shows
the torque balance that produces it, per ball, with each ball's lever arm.

Measured onset: **12 POLLEN** or **8 NECTAR**, pooling at a 9–10 in lever arm.

A match starts with **3 NECTAR in each upward-facing CELL** — manual §10.3.1 B.i, Fig 10-2.
`World.stageCells()` places them against the back wall after the rockers settle on their
stops. Measured there, they sit at a 9.6 in lever and hold the rocker **48% of the way over**,
against STRATEGY.md §4.2's predicted 40–55%: so the first tip, the one both alliances race
for, costs about **six** POLLEN, not twelve.

This used to read "a match starts with empty CELLs", on two grounds. The first — that the
STEP's own six NECTAR are the CAD's display state — is true, and they are still cleared: the
CAD fills all four CELLs and the manual fills two, so the staging is done explicitly rather
than inherited. The second — that staged balls rendered floating under the CAD skin — was a
symptom of the pocket being 11° out (item 3 below) and went with it.

---

## 6. What the data tools measure

| Tool | Question it answers |
|---|---|
| `tools/mechcheck.ts` | does the mechanism work *as a mechanism* — does a ball get picked up, held, lifted and fired |
| `tools/collect.ts` | fire N shots across a range sweep; bias, spread, land rate, faults |
| `tools/collect.ts --calibrate N` | collect, apply the trim it implies, repeat — does the bias converge |
| `tools/audit.ts` | do balls gain energy, tunnel, jitter or go non-finite; does the robot drift |
| `tools/hivedrop.ts` | how many balls tip the HIVE |
| `tools/shottable.ts` | regenerate the shot table |
| Predictor tab | ∂(landing point)/∂(each variable) × that variable's 1σ — the error budget |

**The model is validated against itself.** A 30-shot run measures a downrange spread of
±11.7 in; the Predictor, from the same constants, predicts ±10.3 in. Agreement between a
measurement and a model derived independently of it is the reason the Predictor's advice is
worth acting on.

**Arrival, not resting place.** A shot's error is measured where the ball crossed the mouth's
height on the way down, not where it stopped rolling. A ball that drops an inch wide of the
lip and then bounces forty inches across the field was an inch out; scoring it as forty makes
the statistics measure the floor instead of the shooter. This alone moved the reported spread
from 40 in to 12 in.

---

## 7. Known approximations

Listed because they are the ceiling on how far this can be trusted, not because they are
fine.

1. **The nip is an impulse.** Section 3. Numerical, not laziness.
2. **`ball.Cd`, `ball.clSlope`, `ball.e_poly` are guesses.** They own about 40% of the
   predicted shot spread between them. These are the three numbers most worth measuring on a
   real ball.
3. **The CELL pocket is a reconstruction**, but it is no longer keyed off the assembly
   centroid, and the 11° error that came from doing so is fixed. The pocket's PLACEMENT (an
   arm 16.44 in long at 70.03° in the body frame) and its ORIENTATION (89.99°, so 30° above
   horizontal at the 30° stop) are two separate angles; sharing one for both tilted the mouth
   11° too steep and put the lips at 52.5/62.7 in against the manual's 53.5/65.6.

   Both now come from `up_back_skin_bbox_in`, the pocket's floor plate. The earlier attempt
   that "failed" used `up_floor_bbox_in` as a POSITION, which it is not — an axis-aligned box
   of a tilted plate does not give its centre. It does give the plate's EXTENT, and that is
   all that is needed: 12.95 in of Y over 7.48 in of Z is a plate 14.955 in long at 30.01°
   off vertical, and stepping 12.04 in up its normal puts the lips at 53.49 and 65.61 in. The
   manual and the CAD agree to 0.02 in. Still a box reconstruction, still not the STEP's own
   faces, but no longer wrong about where the opening is. See docs/DECISIONS.md.
4. **Chassis rotations are locked in roll and pitch.** The tyre model applies forces at the
   CG, so there is no roll moment to resolve, and leaving the axes free let solver noise tip
   the box over.
5. **Aero on balls inside the robot** is computed and negligible rather than special-cased.
6. **The target comes off a camera now, and the camera is usually blind.** This used to read
   "the target's bearing and range are exact", and it was the largest remaining lie in the
   project: `game.upCellAzimuthDeg`, `upCellRangeIn`, `upCellOpenDeg` and `hiveTipping` came
   straight off the world with no noise, no latency, no field of view and no way to be
   invalid, and **both brains aimed on them**. Every aiming result here was measured on top
   of a robot that always knew where the goal was and knew the instant the HIVE went over.

   **The tag is where the CAD puts it, and it is not the mouth.** `cad/parts.json` has four
   am-5888 panels, two per hive, one per CELL, **all four bolted to a rocker — there is not a
   single static fiducial on this field**. The panel sits 14.10 in from the pivot at 6.1° off
   the rocker's long axis; the mouth of the same CELL is 22.07 in out at 14.7°, about 10 in
   away. The camera is aimed at the *panel* and swings with the rocker exactly as the pocket
   does, and `tools/tagoffsets.ts` bakes the rigid panel → mouth correction the robot has to
   add: **±2.78 in along FTC +X, sign set by the tag ID**. That makes the ID load-bearing in
   the way LOCALIZATION argues it should be — the ID is the rocker state sensor, and getting
   it wrong is worth 5.6 in of aim plus a mouth facing the other way.

   `packages/core/src/physics/tagCamera.ts` is what replaced it. It emits an
   `AprilTagProcessor`-shaped detection — id, bearing, range, tag yaw — at 30 fps with 75 ms
   of latency, and emits *nothing* outside a 60° lens, past 120 in, past 65° of incidence, or
   while the rocker is swinging. The lens rides the **turret**, per LOCALIZATION phase 4.
   Noise is shaped like a real tag pose rather than flat: 0.5° on bearing, 4% of range on
   range (it is an apparent-size estimate), and 6° on tag yaw, which is the worst-conditioned
   axis and unfortunately the one the "is the mouth still open towards me" gate reads.

   `control/TagTargetProvider.java` — **in TeamCode, so it ships** — turns that into an aim.
   A detection becomes a field-frame point once; the bearing and range are re-derived from
   the localizer's pose every loop after that. Fresh fix: shoot. Stale fix: aim, hold fire.
   No fix: sweep the turret and go and find the tag. No field geometry is baked into the
   robot at all, so it re-acquires from wherever it actually is.

   **`hiveTipping` is gone and nothing replaced it, which is the point.** A rocker going over
   blinds the camera; the fix ages past `maxFireAgeS` and the shot is refused for staleness.
   A TIP, an occlusion and simply looking the wrong way are the same fact to a camera, and
   holding fire is the right answer to all three. What is still not modelled: the fix is built
   against the pose *now* rather than the pose when the photons left, which is worth v·latency
   (4.4 in at 1.5 m/s) — see the `ponytail:` note in `robot/tagTarget.ts`.

   What it cost, measured (`tools/movingfire.ts --gate`, landed/s, oracle → camera): stopped
   0.85 → 0.95, closing 0.95 → 0.95, strafing 0.80 → 0.80, shuttling 0.85 → 0.85, wobbling
   0.75 → 0.65. The pooled rates barely move; **the lateral spread is where it shows**, going
   from ±2–3 cm to ±4–7 cm, which is the bearing sigma arriving exactly where it should. The
   gate is open less often too: "clear to fire" falls from 95–100% to 81–97%. AUTO is
   unchanged at 8 points and 3.2 landed of 4 fired, because AUTO stands still and square onto
   the goal at 45 in — which is the easy case for vision, and worth saying rather than
   claiming the change was free.
7. **The muzzle's POSITION is still the robot's tracked point; its VELOCITY no longer is.**
   While the lead holds the aim off the bearing, a muzzle 0.12 m out on the turret sits a
   centimetre or two off the shot line, and that offset is still ignored when the range and
   bearing are taken. Its *velocity* was ignored too, and that half was not small: a chassis
   yawing at 90°/s swings the muzzle at ω r = 0.19 m/s, which is 4.7 in of lateral miss at
   60 in — wider than the 0.5 in of clearance to a lip. `Robot.launch()` now gives the ball
   v_cg + ω × r and `muzzleVelocity()` / `ShotLead.muzzleVelocity()` subtract the same term
   before solving, so the two agree. On the same seeds that took the turning case's median
   lateral error from −7 cm to −3 cm and let it fire 39 shots where it had fired 31; every
   case with ω = 0 is unchanged to the centimetre, which is the expected signature.

   The earlier reading here — "worth a degree or two … so left alone" — was wrong because it
   priced the term against a simulator that did not have it either. Aim and flight agreeing
   with each other is not the same as either agreeing with a robot, and `tools/shoterror.ts`
   printed a `muzzle v_lat` column against a world in which that velocity was never applied.
