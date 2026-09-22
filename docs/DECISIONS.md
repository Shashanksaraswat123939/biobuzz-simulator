# Decisions and deviations log

Append-only. When reality disagrees with `PLAN.md`, add an entry here rather than
silently diverging. Keep entries short. Newest at the bottom.

Format:

```
## YYYY-MM-DD — short title
Plan said:   …
Found:       …
Did instead: …
Costs/risks: …
Who/where:   agent or person, files touched
```

---

## 2026-09-16 — Plan v2: Java-first brain
Plan said:   v1 had no Control Hub bridge and no Java; the robot brain was implied to be TypeScript.
Found:       The team wants the control code to port to the real REV Control Hub quickly.
Did instead: Split into a TypeScript "world" and a Java 8 "brain" written against the real FTC SDK
             interfaces, with an SDK shim, fake hardware (simsdk) and a WebSocket bridge. `java/teamcode/`
             is the portable deliverable. See PLAN.md §0.4, §3, §7, §9, §14.
Costs/risks: Shim drift vs the real SDK — mitigated by a CI compile check against the SDK jars or a manual
             port at each phase boundary. Two toolchains instead of one.
Who/where:   plan author; PLAN.md, AGENT_PROMPT.md

## 2026-09-16 — CAD units and axes
Plan said:   (nothing yet)
Found:       The STEP declares metres but OpenCASCADE (OCP) imports it in millimetres; Y is up in the file;
             inside width is 141.35 in with 23.5 in tiles, not 144/24.
Did instead: `cad/analyse_step.py` divides by 25.4; PLAN.md §1 records the CAD numbers as ground truth.
Costs/risks: Anyone re-exporting with another tool must re-check a POLLEN measures 71.1 mm.
Who/where:   plan author; cad/analyse_step.py, cad/cad-summary.json

## 2026-09-16 — No STEP→mesh asset pipeline; the field is built from CAD numbers
Plan said:   §6 `tools/cad2assets.py` tessellates the STEP into glTF render meshes and
             decomposed convex colliders, via OCP + Blender + V-HACD.
Found:       The only geometry the physics actually needs is the CELL pocket, the frame, the
             FLOWER tubes and the perimeter — and the plan itself (§6.4, §7) requires the
             pocket to be *convex pieces*, not a mesh, or balls float on a hull across the
             mouth. Tessellating 373 k triangles to then throw them away and hand-author
             convex boxes is a day of work that changes no number.
Did instead: `packages/core/src/field/geometry.ts` builds everything parametrically from the
             numbers in `cad/cad-summary.json`, with the CAD constants in one exported `CAD`
             block. The pocket is a 20 × 14 × 12.04 in box of five convex plates; its fit was
             checked against the CAD up-CELL bbox and agrees to better than 1 in on every face
             (`tests/geometry.test.ts`). The staged positions of all 56 balls *are* taken from
             the CAD, by `tools/cad2staging.mjs` reading `cad/parts.json`.
Costs/risks: No pretty field mesh — the renderer draws the same primitives the physics uses,
             which at least guarantees what you see is what collides. The pocket is a box, not
             the real pentagon, so where balls pool is approximate: they settle at a 9-10 in
             lever arm here. If that number matters more later, decompose the real ribs.
Who/where:   packages/core/src/field/geometry.ts, tools/cad2staging.mjs, tests/geometry.test.ts

## 2026-09-16 — The rocker is an over-centre see-saw, and the blue one starts mirrored
Plan said:   §1.4 gives a CG "1.83 in above the pivot, 1.05 in toward the down-CELL side" and
             rest angles ±30°, without saying how the two relate.
Found:       Rotating the CAD CG offset into the rocker body frame (θ = 0) puts it 2.11 in
             *straight up* over the pivot, to within 0.2°. So θ = 0 is the unstable
             over-centre point, gravity torque is m·g·r·sin θ, and at the ±30.04° stops that
             is 0.626 N·m — exactly the figure §1.6 quotes. The two CELLs are 158° apart on
             the body at ±79°, and the rest angle 30.04° falls straight out of the CAD.
             Separately, `cad/parts.json` shows the blue rocker staged *mirrored*: red's
             AUDIENCE cell is up (bottom skin Y 51.95, Z +13.30) while blue's SCORING cell is
             up (Y 51.95, Z −13.30). The two up-CELLs face opposite Z.
Did instead: `params.hive.cgOffset_m` is now documented and stored in the body frame, and the
             blue Hive starts at +restAngle. Alliance stations are at ±X (that is where the
             STEP stages the NECTAR and the loose POLLEN), so the LOADING ZONE and GARDEN
             were moved there.
Costs/risks: None known; all of it is asserted in `tests/geometry.test.ts`.
Who/where:   config/params.json, packages/core/src/field/geometry.ts, physics/hive.ts

## 2026-09-16 — Three Rapier behaviours that silently broke the hive
Plan said:   §8.3 "Rapier revolute joint on the pivot line... limits at the two damper-contact
             angles", and §17 "if a result looks surprising: first units, then collider
             decomposition, then the timestep".
Found:       Three separate things, each of which alone stopped the hive tipping:
             1. `JointData.limitsEnabled/limits` set on the *descriptor* is silently ignored.
                The rocker had no end stops at all and would spin to 10 800 deg/s.
             2. `RigidBody.addForce`/`addTorque` **persist across steps** until
                `resetForces`/`resetTorques`. Applying pivot friction every substep compounded
                it into tens of N·m, so the rocker would not move under 20 N·m. The drivetrain
                and the ball aero had the same bug.
             3. With those fixed, the rocker still would not move: its pocket was jammed
                against the static frame approximation (the ACM panels at Z = ±19.48 in sit
                where the down-CELL swings to Z = −25 in).
Did instead: 1. `joint.setLimits(min, max)` on the created `RevoluteImpulseJoint`.
             2. `resetForces`/`resetTorques` at the top of every `preStep`.
             3. `packages/core/src/physics/groups.ts`: the rocker collides with balls only.
                The robot cannot reach a 44 in pivot under R102's 29 in cap anyway.
             Also `CoefficientCombineRule.Min` for restitution instead of Rapier's default
             Average — with Average every ball bounced back out of the CELL mouth.
Costs/risks: The rocker/robot exclusion means a rules-illegal tall robot could pass through
             the hive. Acceptable; the validator flags over-height robots instead.
Who/where:   packages/core/src/physics/{hive,robot,balls,world,groups}.ts, tests/hive.test.ts

## 2026-09-16 — Phase 1 result: 12 POLLEN or 8 NECTAR tip the HIVE
Plan said:   §1.6 hypothesised 8–17 POLLEN depending on where balls pool, and noted the
             community default (ftc_demo) of 8 POLLEN-equivalents. "Neither number is trusted."
Found:       With the CAD mass estimate (2.38 kg), pivot friction 0.05 N·m and elements placed
             gently into the up CELL, the simulated onset is **12 POLLEN** or **8 NECTAR**.
             Balls pool at a **9–10 in lever arm** — between the plan's 5–8 in guess from the
             floor bbox and the 11–12 in the pocket-box geometry predicts. Ball torque at
             onset is 0.729 N·m against 0.609 N·m of gravity, and the tip takes 0.88 s.
Did instead: `tools/hivedrop.ts` produces this on demand and `tests/hive.test.ts` locks in the
             ordering (NECTAR < POLLEN, heavier rocker needs more).
Costs/risks: It rests on the guessed rocker mass and on the pocket being a box. Both are
             config; sweep them. This replaces the plan's table with a simulated one, not
             with a measured one — November still decides.
Who/where:   tools/hivedrop.ts, tests/hive.test.ts

## 2026-09-16 — Solver iterations: 16, measured not guessed
Plan said:   `params.sim.solverIterations: 8`.
Found:       With 56 balls, two rockers and the robot: peak rocker drift 0.102 deg/s at 4
             iterations, 0.053 at 8, 0.004 at 16, 0.003 at 24 — costing 1.05 / 1.57 / 2.59 /
             4.22 ms per rendered frame.
Did instead: 16. A still rocker with ~380 fps of headroom.
Who/where:   config/params.json

## 2026-09-16 — Tools run through Vite, not node --experimental-strip-types
Plan said:   (nothing) — tools are `.ts` under `tools/`.
Found:       Node's type stripping does not do the ESM `.js` -> `.ts` specifier mapping that
             TypeScript requires, so `node --experimental-strip-types tools/x.ts` cannot
             resolve any core import.
Did instead: `tools/run.mjs` (17 lines) uses Vite's own SSR loader, which is already a
             dependency. `npm run tool -- tools/hivedrop.ts --mass 2.4`.
Costs/risks: Tools must export `main(args)`. No new dependency.
Who/where:   tools/run.mjs, package.json

## 2026-09-16 — Aiming now leads for the robot's own velocity
Plan said:   §7.8 "TurretTracker: turn the turret so the muzzle bearing equals the bearing to
             the up-CELL mouth". Nothing about shooting on the move.
Found:       The world adds chassis velocity to every ball (`v = v_exit*dir + v_chassis`,
             physics/robot.ts `launch`), but the aim was purely geometric. A robot moving
             2 m/s across the line of fire threw the ball >10 deg off target and nothing
             compensated. Verified by `tools/shootercheck.ts`.
Did instead: `ShotLead` (java/teamcode/control) and `leadShot` (core/robot/builtinTeleOp.ts)
             solve the triangle exactly rather than approximating it: the horizontal velocity
             the ball must leave with is (table speed along the bearing) minus (robot
             velocity), so the turret points along that vector and the flywheel is asked for
             |that vector| / cos(elevation). Closing shots need less RPM, retreating more.
             Asserted in tests/shotlead.test.ts and java SelfCheck (both check that the ball's
             resulting bearing lands on the target, not just that a number changed).
Costs/risks: Velocity only, not acceleration: over a ~1 s flight the a*t^2 term is small next
             to the 1-2 deg of launch scatter, and differentiating a noisy velocity estimate
             would be worse than no lead. On the hub the lead is only as good as the
             Localizer's velocity, which is why it reads through the interface.
Who/where:   java/teamcode/control/{ShotLead,AimController}.java, packages/core/src/robot/builtinTeleOp.ts

## 2026-09-16 — Turret coverage is +-120 deg, and the chassis never turns to aim
Plan said:   §7.5 turret `range_deg` e.g. +-90 from forward.
Found:       Measured with `tools/shootercheck.ts`: with the chassis heading held fixed, the
             turret alone reaches the up CELL from every bearing between -120 and +120 deg.
             Outside that the shot is correctly refused ("turret cannot reach") rather than
             silently mis-aimed -- the driver has to turn the robot, which is the truth.
Did instead: Kept `range_deg: [-120, 120]`. `TurretTracker.canReach` gates the shot and
             `AimController.status()` says why.
Who/where:   config/robot.json, java/teamcode/control/TurretTracker.java

## 2026-09-16 — Flywheel inertia and the omega -> exit-speed chain, checked
Plan said:   §2 "Flywheel inertia | Gecko 96 mm 1.21e-4 ... kg.m^2 | shot-sim | fixed".
Found:       1.21e-4 kg.m^2 at a 96 mm wheel implies 105 g as a solid disc or 53 g as a rim,
             which is what a compliant 96 mm wheel actually weighs. With the motor rotor
             reflected the total is 1.31e-4, giving a spin-up time constant
             I*w_free/tau_stall = 0.74 s. Exit speed measured against k*omega*r at 1500-5500
             RPM agrees to better than 0.5% (the residual is the configured 1.5% scatter),
             and each shot costs the wheel 195 RPM at 1500 up to 667 RPM at 5500, rising with
             speed as lossFactor*KE/(I*omega) should.
Who/where:   tools/shootercheck.ts

## 2026-09-16 — The brain half: shim, simsdk, bridge and runner, with no Gradle
Plan said:   §3.3/§4 build the Java with Gradle, pull Gson from Maven, use Java-WebSocket,
             and keep the shim honest with a CI compile against the season's SDK AARs.
Found:       No Gradle and no Maven on this machine, so nothing can be fetched.
Did instead: `javac` only, driven by `java/build.sh`. Dependencies replaced by the JDK:
             `java.net.http.HttpClient` has a WebSocket client built in, and `sim.bridge.Json`
             is a ~180-line reader/writer. TeamCode is compiled `--release 8` against the SHIM
             ONLY, which is the compile check that matters: nothing sim-side is even on its
             classpath, so an accidental `import sim.*` cannot compile.
Costs/risks: The shim is hand-written, so SDK drift is caught by the manual port at phase
             boundaries rather than by CI. Appendix E is the contract.
Who/where:   java/build.sh, java/{shim,bridge,simsdk,runner}

## 2026-09-16 — Constants are generated, not parsed, on the hub
Plan said:   §9.1 `RobotConfig.java` loads robot.json with Gson through a `ConfigSource`.
Found:       No Gson, and reading a file during `init()` is how OpModes get killed by the
             watchdog. PLAN.md §14.2 already offers the alternative.
Did instead: `tools/genconstants.mjs` bakes `config/robot.json` and the generated shot table
             into `RobotConstants.java` and `ShotTableData.java`. One origin for every number
             still holds -- edit robot.json, re-run the generator, the Java follows.
Costs/risks: The generator has to be re-run after editing robot.json. `java/build.sh` does not
             run it automatically because it needs Node.
Who/where:   tools/genconstants.mjs, java/teamcode/config/

## 2026-09-16 — Five bugs found by actually running an OpMode over the bridge
Found:       Each of these silently produced a robot that did nothing, and none would have
             shown up without running the Java against the world end to end:
             1. `RigidBodyDesc.setAdditionalMass*` is ignored in this Rapier build -- the
                robot came out with mass 0 and invMass 0, i.e. immovable. Mass has to go on a
                collider (`ColliderDesc.setMassProperties`). Same for the rocker.
             2. Rapier averages friction coefficients, so the chassis shell's 0.02 paired with
                the tiles' 0.85 gave an effective 0.435 -- about 60 N of drag, most of the
                drivetrain's output. `CoefficientCombineRule.Min` fixes it; top speed went from
                30 in/s to 62.8 in/s against a hand-computed 62.
             3. An ActuatorFrame carries only each motor's FINAL state for the frame, so
                `setMode(STOP_AND_RESET_ENCODER)` followed immediately by `setMode(...)` -- the
                standard idiom -- never reached the world. Added a sticky `reset` flag cleared
                once the frame ships.
             4. The world reported encoder counts in the PHYSICAL frame while powers were in
                the motor's electrical frame, so a reversed wheel's encoder cancelled its
                partner's and the drivetrain's average travel was always zero.
             5. The runner serialised the actuator frame while the OpMode thread was still
                writing it (ConcurrentModificationException). `ActuatorFrame` now hands over a
                finished map per device under one lock.
Who/where:   packages/core/src/physics/{robot,hive}.ts, robot/hubEmulation.ts,
             java/bridge/ActuatorFrame.java, java/simsdk/SimDcMotorEx.java, java/runner/Main.java

## 2026-09-16 — Three bugs in TeamCode itself, found the same way
Found:       1. `FlywheelGate.setTargetRpm` reset readiness whenever the target changed at
                all. A live target is a continuously varying double (range jitter times the
                motion lead), so readiness was reset every single loop and the gate never
                opened. Now only a change worth more than half the tolerance counts.
             2. `DriveToRange` has a +-6 in deadband around the shot table's best range. At
                24 in -- outside a table whose floor is 30 in -- that deadband said "close
                enough" and the robot stopped somewhere it could never score from. The
                deadband now only applies inside the usable band.
             3. Reading an encoder in the same loop as `STOP_AND_RESET_ENCODER` returns the
                value from before the reset (true on a hub too). On the return leg of a
                there-and-back that looks like "already arrived" and the move is skipped.
                `resetAndSettle` waits for the count to actually reach zero.
             Also: driving open loop drifted about a foot laterally over a 30 in leg, so both
             autos now hold heading on the IMU.
Who/where:   java/teamcode/control/{FlywheelGate,DriveToRange}.java, opmodes/Auto*.java

## 2026-09-16 — Match preloads, and a measured ball count
Plan said:   §9.3 `AutoOneTip` fires "the 4 preloaded POLLEN". Nothing said where they come from.
Found:       `Hopper.setCount()` only set TeamCode's dead-reckoning counter; the WORLD's hopper
             was empty, so the transfer had nothing to feed and the auto fired zero shots.
Did instead: `WorldOptions.preload` takes the nearest N POLLEN off the field into the robot's
             hopper at construction and on reset (4 in the app, `--preload` headless). And
             TeamCode gained `BallCounter`, the same `tryGet` shape as `Localizer` and
             `TargetProvider`: the sim supplies a real count, the hub returns null and the
             Hopper dead-reckons, with telemetry saying which it is.
Who/where:   packages/core/src/physics/world.ts, java/teamcode/control/BallCounter.java,
             java/teamcode/subsystems/Hopper.java, java/simsdk/SimBallCounter.java

## 2026-09-16 — Phase 3 and phase 5 acceptance, in lockstep
`Auto Leave + Park`: LEAVE + PARK, 8 points.
`Auto One Tip`:      LEAVE + PARK, 6 shots, 1 TIP, 28 points in autonomous.
Both driven by the real Java OpModes over the bridge against the headless world.
Determinism: same seed and command log give bit-identical snapshot hashes over three runs
(`tests/determinism.test.ts`). `World.reset()` restores game state but NOT the hash, because
Rapier warm-starts its contact solver -- only rebuilding the World reproduces a hash, which is
what the app's reset button now does.

## 2026-09-16 — The turret is a real axis: velocity AND acceleration limited
Plan said:   §7.5 the turret has `speed_dps`, and §9.2 `TurretTracker` is "slew-rate limited".
Found:       A rate limit alone means the axis leaves standstill at full slew speed and stops
             dead on arrival -- infinite acceleration in both directions. Nothing about the
             turret's settling time was real, so the readiness gate's "turret slewing" state
             was optimistic.
Did instead: `Robot.stepTurret` runs a trapezoidal profile with `speed_dps` and a new
             `turret.accel_dps2`: the rate ramps at the acceleration limit and starts braking
             at sqrt(2*a*err) so it arrives stopped instead of overshooting. Three details the
             discrete form needs, each of which showed up as a test failure first:
              - the continuous sqrt(2*a*err) bound overshoots by up to half a step, so it is
                backed off by half a velocity quantum; otherwise the axis reaches a hard stop
                still doing 32 deg/s and the limit has to discard the rate (a fake 1900
                deg/s^2 spike);
              - one step can only change the rate by a*dt, so without an explicit landing case
                the profile dithers either side of the target forever;
              - the landing bleeds the last of the rate off at the acceleration limit rather
                than zeroing it, for the same reason as the first point.
             The snapshot now carries `turret.omegaDps`, `targetDeg` and `atLimit`, and the UI
             shows the rate.
Costs/risks: `accel_dps2: 900` is a guess (0.2 s to reach the 180 deg/s slew limit). It should
             be derived from motor torque over turret MOI once a real turret exists -- the
             config key and its `_accelSource` note are there for that.
Who/where:   config/robot.json, packages/core/src/physics/robot.ts, tests/turret.test.ts

## 2026-09-16 — The rocker is drawn as the CAD part, not as its collision boxes
Plan said:   §6 render meshes come from tessellating the STEP; §5 the renderer draws what the
             physics collides with.
Found:       Drawing only the five convex plates per CELL is honest but unreadable -- it looks
             like a crate, not like am-5853, and you cannot tell which way a CELL faces.
Did instead: The pocket walls are still drawn (they ARE the colliders, so what you see still
             collides), and the recognisable parts are added on top as render-only geometry
             sized from `cad/parts.json`: pentagon end ribs (am-5866, 21.2 x 14.3 in), the
             basket base tube out to the pivot (am-5868, 1 in square, 17.6 in long), two
             10.5 in churros bracing the mouth (am-5867) and the AprilTag panel (am-5888,
             17 x 4.3 in). The A-frame is likewise drawn as the four splayed legs the CAD
             has -- from the foot bars at X +-24.1, Z +-18.3 up to the top corners at
             X +-12.6, Y 41.3 -- rather than the four vertical posts the physics uses.
Costs/risks: The render and the collision shape now differ, which is the thing §5 warns about.
             The difference is additive only (dressing, never a surface a ball can touch), and
             the pocket the balls actually meet is still drawn.
Who/where:   packages/render/src/scene.ts

## 2026-09-16 — UI: a tip meter, first-person view, and auto-load
Found:       The single number people want from this simulator is "how close is the HIVE to
             going over", and it was buried in two torque readouts of opposite sign.
Did instead: A meter on the Hive tab showing the ball torque OPPOSING gravity as a fraction of
             the restoring torque, with the 100 % line marked. Only the opposing component
             counts -- after a tip the balls in the now-down CELL push the rocker onto its
             stop, which read as "158 % of the way over" until this was fixed.
             Also: a first-person camera (key 4) that looks where the CHASSIS points, which is
             the view that makes an independently-aimed turret make sense; and an Auto-load
             toggle (key L) that keeps the hopper topped up from balls on the floor so aiming
             can be practised without a collection lap. Auto-load takes real balls off the
             real field and the field does run out -- it is a practice aid, not a game rule.
Who/where:   index.html, packages/ui/src/{main.ts,style.css}, packages/render/src/scene.ts

## 2026-09-16 — Drive is robot-centric by default
Plan said:   §9.2 `Drivetrain` does "field/robot-centric mecanum drive"; §9.3 TeleOpMain is
             "field-centric drive".
Found:       Field-centric referenced to FTC +X means the stick's "forward" is a fixed field
             direction, so from the default start pose (heading 90 deg) pressing W made the
             robot STRAFE. Measured: W moved the robot 30.7 in along FTC +X while it was
             facing FTC +Y. Correct field-centric, and it feels broken.
Did instead: Robot-centric is the default -- the stick drives the robot the way it points.
             Field-centric is a toggle (C / L3) and is referenced to a heading zero the driver
             can reset (Z / Back), not to FTC +X.
             While checking this: A and D looked asymmetric (30.6 in vs 4.2 in). Not a bug --
             the default start pose is 4 in off the wall, so one direction runs out of field.
Who/where:   packages/core/src/robot/builtinTeleOp.ts, packages/ui/src/input.ts,
             tests/drivetrain.test.ts

## 2026-09-16 — Latching controls and a button deck
Found:       Holding a key to run the intake and holding another to fire is how a gamepad
             works, not how the robot works. A real intake runs the whole match, and the
             shot rate is set by `transfer.cycleTime_s`, not by how long a thumb is down --
             so holding the fire button was doing nothing the cycle timer was not already
             doing.
Did instead: `intakeOn` defaults to TRUE and is a toggle; the left trigger still spits and the
             right trigger still forces it on while it is switched off. `firing` is a latch:
             press once and it keeps feeding at the cycle time until pressed again, with
             gamepad B kept as a non-latching hold for single shots.
             Added an action deck down the left of the app so none of it needs the keyboard:
             match control, the five robot latches, spit-one-out, and the practice aids
             (auto-load, fill hopper, drop a POLLEN, drive mode). Buttons and keys drive the
             same state through one `ACTIONS` table, so they cannot disagree, and each toggle
             reports its own state back for the lit styling.
Who/where:   packages/core/src/robot/builtinTeleOp.ts, packages/ui/src/{main.ts,input.ts,style.css},
             index.html, docs/CONTROLS.md

## 2026-09-16 — The STEP really is tessellated now (reversing the earlier shortcut)
Plan said:   §6 `tools/cad2assets.py` turns the STEP into render meshes.
Earlier:     I skipped it and rebuilt the field from `cad-summary.json` numbers, arguing the
             physics only needs convex boxes. True for the physics, wrong for the view -- the
             field looked like crates and the FLOWERs were featureless tubes.
Did instead: `tools/cad2assets.py` loads the STEP through OCP XCAF (which IS installed:
             OCP 7.9.3.1), tessellates by role and writes `assets/field.glb` via trimesh.
             171 parts, ~675k triangles in about 40 s. Fasteners, cable ties and anything
             under 900 mm^3 are dropped (676 parts) -- they are the part count and none of the
             silhouette. Tiles are not exported either; the renderer draws them procedurally
             and they were 80k triangles of flat squares.
             The PHYSICS is untouched and still uses the convex boxes: a mesh collider becomes
             its hull, and a hull across the CELL mouth is the "balls float on an invisible
             lid" bug. A **Colliders** toggle in the app swaps the CAD skin for the boxes so
             the difference is inspectable rather than hidden.
Costs/risks: 12 MB asset, gitignored and regenerated. The app falls back to the procedural
             stand-in if it is missing, so a clean checkout still runs.
Who/where:   tools/cad2assets.py, packages/render/src/scene.ts, .gitignore

## 2026-09-16 — This STEP carries no colour, so colour comes from what each part is
Found:       `SetColorMode(True)` and XCAFDoc_ColorTool return nothing for all 171 parts --
             the AndyMark export has no colour data at all.
Did instead: `tools/cad2assets.py` assigns a colour per part NAME and bakes it as vertex
             colours: alliance red/blue for the goal ribs, skins and basket parts, aluminium
             for churros, tubes, brackets and the A-frame, near-white for the AprilTag panels
             and the HIPS flower pipes, tinted glass for the perimeter. One `vertexColors`
             material then draws the whole field.
Who/where:   tools/cad2assets.py

## 2026-09-16 — Three rendering bugs worth naming
1. **Re-parenting inside `Object3D.traverse`** silently skipped half the meshes: the walk
   mutates the children arrays it is iterating. Collect first, reparent after. Symptom was
   the red rocker loading and the blue one staying a box.
2. **Ball holes merged into spikes.** The 26 hole axes (a cube's 6 face + 12 edge + 8 corner
   directions, which is exactly 26) are about 35 deg apart at the closest; at a 0.30 rad
   half-angle the holes overlapped and dissolved the sphere. 0.135 rad leaves them distinct.
3. **Vite served the 12 MB GLB as a module** (200 OK, unparseable). `import ... from
   '...glb?url'` makes it an asset. It is also cached in a static promise now, because
   rebuilding the Scene on every reset was re-downloading the whole field.
Who/where:   packages/render/src/scene.ts

## 2026-09-16 — Balls staged outside the glass are out of play, not floating
Found:       The STEP stages 16 POLLEN and 10 NECTAR at X = +-73..77 in -- beyond the 70.675 in
             wall. Those are the human player's hands, and rendering them left balls hanging
             in mid-air outside the field with nothing under them.
Did instead: `World.parkOffField()` disables anything staged outside the perimeter at
             construction and on reset. They are out of play until a human hands them in
             (G426/G427), which is exactly what the rules say.
Who/where:   packages/core/src/physics/world.ts

## 2026-09-16 — A robot instead of a box
Found:       "A box with a yellow stick on top" made it impossible to see where the intake or
             the outtake were, which matters because the turret aims independently of the
             chassis.
Did instead: Render-only detail: chassis rails and side panels, four mecanum wheels whose
             rollers sit at +-45 deg and which SPIN at their real omega, a roller intake
             across the front with a mouth and a lip, a transparent hopper you can see the
             ball stack in, and a turret carrying the flywheel (spinning at its real RPM)
             behind a barrel that is unmistakably the outtake. The physics is still one box
             plus the tyre model, exactly as `robot.json` describes it.
Who/where:   packages/render/src/scene.ts

---

## The intake, hopper and feed became real mechanisms

**Was:** a state machine. A ball that touched a trigger box in front of the robot was
disabled, removed from the solver, and appended to an array; a timer moved it along an
"intake line"; another timer fed it to the muzzle. Nothing collided with anything.

**Now:** the chassis is built out of plates — floor, sides, back, a front wall that stops
above the intake mouth, and a vertical feed tube up the turret axis. A ball is a rigid body
the whole way through the robot. The roller grips it by slip-limited friction, the indexer
sweeps it into the tube, the belt lifts it, a gate plate holds it, and the nip fires it.

**What this cost, honestly.** Six real jams, each of which looked perfectly reasonable in
the source:

1. `applyAero` ran *after* the robot and reset every ball's accumulated force, so the intake
   and feed silently did nothing at all. Aero now runs first.
2. The shell was given 0.25 friction so balls would grip inside the bin. It is also the only
   part that touches a tile, and at 0.25 against the tiles' 0.85 it dragged at ~35 N and top
   speed fell from 63 to 45 in/s. Back to 0.02; the bin's walls contain balls without it.
3. A ramp was added to lift balls over the floor plate's edge. A plate that reaches tile
   level also *drags* on the tile — it turned the robot into a plough. Deleted; the roller's
   lift term does that job, which is what a real compliant roller does.
4. The feed tube was open only at the front, so any ball that ended up behind it was
   unreachable for the rest of the match. Both ends are open now.
5. The tube was bored for the larger game element (4.1 in) — and two 2.8 in POLLEN wedge
   diagonally in 4.1 in. Bored for one POLLEN now. NECTAR will not enter, which is correct.
6. The indexer pushed *every* eligible ball at the tube at once, sending two in through the
   two openings on the same step. Metered to one at a time.

**And one that only showed up under load:** the belt was gated on "ready to fire", so the
tube emptied back into the bin after every shot and the robot re-lifted the same ball. The
gate is a real plate now, the belt runs continuously, and the magazine stays loaded.

Verified by `tools/mechcheck.ts`, which drives each stage separately: a ball is collected off
the floor in 0.57 s, six preloads fire 6 of 6, and the full suite still passes.

**What is still an impulse:** the nip. At 3500 rpm it opens and closes in ~400 µs, two orders
of magnitude below the 1/240 s timestep. Everything that *decides* whether a shot happens is
physical; the momentum transfer is not.

## A shot's error is measured where it ARRIVES

`missBy` used to be the distance from the CELL mouth to where the ball stopped rolling. A
ball that dropped an inch wide of the lip and then bounced forty inches across the field was
scored as a forty-inch miss, so the statistics were measuring the floor, not the shooter.
Now the world watches each shot down and records the point where it crossed the mouth's
height descending. Reported spread fell from ±40 in to ±12 in without a single physics
change — the old number was an artefact.

## Shots logged the flywheel speed AFTER the shot dipped it

`lastShotRpm` was read after `launch()` had already taken the ball's energy out of the wheel,
so every shot in the log looked like it had gone out off-speed and the Analysis tab reported
a readiness-gate fault that did not exist. Captured at the top of `launch()` now.

## Matches started with the HIVE 57% tipped

The STEP stages six NECTAR inside the two CELLs. That is the CAD's display state, not a match
start. Left in, every match began with the balls already more than half way to overcoming
gravity, and they rendered as balls floating under the CAD skin. `parkOffField` now takes
anything above the pivot out of play, and the hive census ignores parked balls — `park()`
disables a body but leaves it where it was, so without that check they went on contributing
torque after being removed.

## Tried: deriving the CELL pocket radius from the CAD floor plate. Reverted.

The pocket is built around the CELL assembly's centroid (14.96 in from the pivot), which is
arguably too far inside the pocket it describes. `cad-summary.json` has `up_floor_bbox_in`,
whose centre is at 15.71 in, suggesting a pocket centre near 21.7 in.

It is wrong. That bbox is the *axis-aligned* box of a plate tilted 30°, so its centre is not
the plate's radial position — and with the larger radius the CAD's own staged balls end up
below the floor it implies, and the pocket pokes outside the CAD's up-CELL bbox. Two geometry
tests caught it. The centroid is the better of the two estimators; the real fix is to derive
the pocket from the STEP's faces, which has not been done. Logged in `docs/PHYSICS.md` §7.

## Calibration is a closed loop, not advice

The Analysis tab computes the trim a run implies and **Apply calibration** writes it into
`robot.calibration`; the shot table is then looked up at `(range − rangeTrim)`. Verified with
`npm run tool -- tools/collect.ts --shots 24 --calibrate 2`:

| round | bias | spread | trim applied first |
|---|---|---|---|
| 0 | +3.4 in | ±8.9 in | — |
| 1 | **−0.2 in** | ±11.2 in | range +3.4 in, turret +0.85° |
| 2 | −1.3 in | ±10.9 in | none (inside the run noise) |

Bias converges, spread does not move, and the third round correctly declines to chase the
sample. That is the whole claim the split between bias and spread makes, and it holds.

## The flatter shot table was never the problem; the feed was

The HIVE-tip scenario failed after the shot table's objective changed, and the obvious story
was retention: a flatter shot lands nearer the mouth, so it has further to roll back down.
`tools/retention.ts` was written to test that story and killed it.

| table | shots in 70 s | peak in CELL | lost, not via a tip |
|---|---|---|---|
| steep (margin only) | 30 | 8 | 12 |
| flat (margin x entry) | 14 | 6 | **2** |

The flat table *retained better*. It simply fired half as often, and the two suspects have
opposite fixes, so the plausible one would have cost a day. Two real bugs, both of which had
already been fixed in the sister build and neither of which had been ported here:

1. **The indexer pushed horizontally only** (`addForce({x, y: 0, z})`). A ball sitting on the
   bin floor is held by its own floor contact; 0.95 N sideways does not lift it over the lip.
   Balls reached the magazine only when something else shook them there — which is why a
   table that fires *less* also feeds *less*, a loop that made the flat table look worse than
   it was.
2. **The shaft census counted a ball as "in the tube" with its centre 10 mm outside the
   bore** (`sh.half + 0.01`), i.e. half of it still in the doorway. The belt then lifted it
   into the lintel and jammed it.

They mask each other, which is why neither showed up alone: fixing the push *first* took
firing from 13 shots to **zero**, a ball wedged permanently in the doorway. The census fix is
its companion, not a separate improvement.

| | before | after |
|---|---|---|
| steep | 25 shots | **44** |
| flat | 13 shots | **44** |
| frames ready with an empty magazine | 1377 / 2292 | **7 / 8** |

`tools/retention.ts` also had to be fixed before it could be believed: its first version
counted every decrease in the CELL population as a ball lost, and a tip empties the CELL on
purpose. It scored the best possible outcome as the worst, reporting 19 "lost" for a table
that was tipping the hive repeatedly.

## The shot table was never the problem either. The firing window was.

With the feed fixed, the flat table and the steep one land identically -- 48.5% against 47.9%
over 278 shots, 0.1 standard errors apart. The entry model predicted a rout. Finding out why
took four measurements, and each one killed the conclusion before it:

**1. The misses are not bounce-outs.** `tools/missmix.ts` splits them using the arrival point
every shot already records. Only 4% of shots arrived over the opening and came back out; 29%
arrived beyond the far lip and 17% went wide. Optimising what a ball does once it reaches the
mouth cannot fix a shot that never reaches it -- which is why the flat table bought nothing.

**2. The shot leaves wrong, it does not fly wrong.** `tools/aimbias.ts` re-runs the SOLVER at
the conditions each shot actually left with. It puts them 5.2 in long against 8.8 in observed,
so the flight is close to right and the exit conditions are not. Exit speed matched
`k*r*omega` to 0.002 m/s and the sensed range was exact, which leaves the wheel speed: it
fires a standing **+38 rpm above target**.

**3. The table is centred; the window is not.** `tools/apercheck.ts` replays each table row
through the solver with no simulation at all. Every row clears the near lip by 3.5-7.6 in,
passes under the far lip by 4.0-8.9 in, and crosses the mouth's centre height within 1.5 in
of its centre. The table is fine. The same tool prices the error:

| | inches of range |
|---|---|
| +10 rpm | 1.4 |
| +38 rpm (the measured standing error) | **5.0** |
| +-120 rpm (`tolRpm`, the window it fires inside) | **+-14.9** |
| +1 deg hood | 1.4 |

The hole is 8.9 in deep. The firing window alone permits a miss nearly twice the size of the
target, and that is the spread; the +38 rpm standing error is the bias.

**4. A trim is worth 38 points, which is the tell.** `tools/trimsweep.ts` sweeps the field
trims against balls in the CELL rather than against a proxy: `rangeTrim` +6 in takes the land
rate from 51.2% to 84.4%, six standard errors. A table that is right cannot be improved by
lying to it about the range by half a foot -- so the trim is not a calibration here, it is a
measurement of the rpm bias in disguise, and the honest fix is upstream of it.

### Do not compute rangeTrim from the mean downrange error

`config/robot.json` documents the recipe as "the mean signed downrange error of a collected
run", and there is a trap next to it. The shots that LAND arrive 5.8 in long while all shots
average 8.8 in, which reads like the aim point really being 5.8 in past the mouth centre. It
is survivorship: with the group centred past the target, the ones that go in are the ones that
happened to be least long. Trusting it would have parked the group on a point the measurement
says is 33 points worse.

## The brain was reading a tachometer no team can buy

Chasing the +38 rpm standing error found something larger. `tools/flywheeltune.ts` shows the
hub's velocity loop settling to **zero** offset with zero ripple at every integral gain, at the
speed the table actually asks for -- so the bias is not the controller, it is a limit cycle:
fire, dip, recover, overshoot, satisfy the window at the top of the bounce, fire again.

But reading the loop to find that out turned up two places where the sim was flattering the
robot:

1. `HubMotorLoop` stored a REAL-VALUED encoder position, so its 20 ms velocity window was
   about fifty times better than a quadrature counter's.
2. `sensors().game.flywheelRpm` returned `radSToRpm(flywheelOmega)` -- ground truth. The
   readiness gate never saw the hub's estimate at all.

The flywheel is direct-driven on 28 ticks a rev. At 2800 rpm that is 26 counts in a 20 ms
window, so **one count is 107 rpm**, about 15 in of range. A Control Hub cannot tell 2800 from
2850. The encoder now floors to whole ticks and the brain reads the hub's estimate; ground
truth stays for the shot log and the analysis, where it belongs.

This matters because the previous session's sweep had recommended `tolRpm` 15 -- best in the
sim, and **not implementable on any FTC robot**. It was an artefact of the perfect tachometer.

### What replaced it, measured on the honest encoder

| velocity window | fired | rate |   | tolRpm | fired | rate |
|---|---|---|---|---|---|---|
| **20 ms** | 102 | **87.3%** |   | 120 | 102 | 87.3% |
| 50 ms | 98 | 84.7% |   | **60** | 82 | **92.7%** |
| 100 ms | 89 | 80.9% |   | 30 | **0** | never fires |
| 200 ms | 82 | 80.5% |   | | | |

Filtering the velocity HURTS: the extra resolution is worth less than the lag it costs, and a
lagging estimate fires on stale data. And 30 rpm never fires because 30 rpm is below what the
hub can report.

60 rpm works for a reason worth writing down: `readySteps` requires three consecutive in-window
readings, and with a +-107 rpm quantisation those are three near-independent draws. Three of
them landing near target is evidence the wheel really is there. The gate is acting as a
statistical filter over a noisy sensor, and that is a filter a team can implement exactly as
written.

With the window at 60, both field trims measure out at zero: the best combination beats zero
trim by 0.6 points, 0.1 standard errors. The +6 in trim that looked worth 38 points was
correcting a bias that only existed because the robot was firing at the top of a limit cycle it
could see and now cannot.

## Three land-rate metrics, two of them wrong

Every number about how well this robot shoots depends on what "landed" means, and it took two
wrong definitions to arrive at one that holds.

| | what it counts | how it fails |
|---|---|---|
| `inCell + tips * 12`, capped at shots | survivors, with a guess for tips | **saturates.** Once the robot is good enough to tip, the expression returns the shot count whatever happened. Three different turret trims printed exactly 100.0%. |
| sum of RISES in the CELL census | every ball that ever entered | **overcounts.** Credits a ball that drops in, bounces out and ends on the floor. Read 92.7% where the settled per-shot rate was 61%. |
| **census at each tip, plus the census at the end** | what was in the CELL when it mattered | nothing assumed, cannot saturate, a visit counts for nothing |

The two survivors are both legitimate and they measure different things, which is worth keeping
straight:

  - `tools/landcal.ts` scores each shot by where it SETTLES. That is the right input for a
    calibration, because a calibration needs a per-shot outcome.
  - `tools/landrate.ts` scores the RUN by what is in the CELL at the end. That is what the game
    pays for.

They disagree by about 13 points -- 61% settled against 48% still there -- and the gap is real:
later shots knock earlier ones out of the pocket. That is the retention effect that was
originally, and wrongly, blamed on the flatter shot table. It exists; it is a property of a
filling pocket, not of the trajectory.

The correction reversed a conclusion. Under the overcounting metric, filtering the flywheel
velocity looked harmful (20 ms window 87.3%, 200 ms 80.5%) and the answer was "do not filter".
Under the scoring metric it is the opposite, and not by a little: 19.5% unfiltered against 44.9%
at 200 ms. A metric that rewards transients rewards firing fast and loose, because a ball that
bounces through the pocket scores the same as one that stays.

## The gate now ships to the hub, because the hub is the deliverable

`builtinTeleOp.ts` says it plainly at the top: the TypeScript brain is a mirror, `java/teamcode`
is the deliverable, and if the two disagree the Java is right. The land-probability gate had
been built entirely on the mirror. `ShotTableData.java` carried four columns -- range, hood,
rpm, margin -- so the hub could not have computed P(land) even if it wanted to.

Ported:

  - `genconstants.mjs` now emits `SPEED_LO`, `SPEED_HI`, `SIGMA_SPEED`, `P_STAY` and
    `HALF_LAT_M` alongside the existing columns, plus the measured calibration curve from
    `config/landcal.json` as `CAL_SCORE` / `CAL_OBSERVED` / `CAL_CEILING`.
  - `control/LandProbability.java` is the model: `normalCdf` (A&S 26.2.17), `pThread`, the
    piecewise-linear calibration, and `pLand` combining speed, bearing and entry. The hub
    still solves nothing -- it interpolates the solver's own outputs.
  - `FlywheelGate` gained the velocity filter as a ring buffer, because readiness is what it
    owns and a single `getVelocity()` reading is quantised to about 107 rpm.

`SelfCheck` runs the same assertions as `tests/landprob.test.ts` -- the normal CDF values, the
two-sigma band, the zero-scatter degenerate case, that the calibration never promises more than
the ceiling, and that pointing 12 degrees off scores worse than pointing at the goal. 15
assertions became 24, and a drift between the two halves now fails the Java build rather than
being discovered in a match.

## RETRACTED: "the robot cannot hold a magazine" was a bug in the probe

Reported here as the top open defect, twice. It was wrong.

The probe placed the robot at **z = 2.38 m against a field half-width of 1.80 m** -- outside
the wall, where there is no floor. The robot free-fell at 9.81 m/s^2 from the first step, the
balls fell with it, and their unchanged position *relative to the robot* was read as the balls
leaving. `tools/landrate.ts` guards placement with `Math.abs(c[2]) < limit`; the probe did not.
Placed legally the robot settles at y = 0.154 m and all four POLLEN sit still indefinitely.

The trap is the relative frame: "the balls did not move relative to the robot" and "the balls
left the robot" produce identical numbers when the robot is the thing that moved. The tell was
there and I read past it -- the balls' vertical velocity was NEGATIVE and growing by 9.81 m/s
per second, which is not what being flung out of a bin looks like, it is what falling looks
like.

## OPEN: the indexer cannot feed from a nearly empty bin

Found while chasing the above, and this one is real. Same rig, robot on the field, shooter
armed, gate open, varying only how many POLLEN are preloaded:

| preloaded | reaches the magazine | shots in 6 s |
|---|---|---|
| 4 | **never** (`shaft=0` throughout) | **0** |
| 6 | yes | 1 |
| 8 | yes | 1 |

With four balls the indexer pushes the nearest one to about 40 mm from the tube and it stalls
there. With six, the extra weight and jostling get one in. So the robot stops feeding when it
is down to its last few -- which in a match is exactly when it is trying to finish a cycle.

Every harness hides it by topping the hopper up every frame from an unlimited supply, which is
why 82 tests pass and none of them sees it.

## SUPERSEDED: the robot cannot hold a magazine

Place the robot, preload four POLLEN, arm nothing at all and step the world. Three of the four
are outside the robot within a tenth of a second, moving at over a metre a second, having left
through the intake mouth. No belt, no indexer, no shooter -- the balls are already at z = +0.20
in the robot frame one frame after placement, and `preload()` puts them at +0.068.

It has been there all along and every harness hides it: `landRate`, `retention` and the shoot
tests all top the hopper up each frame, so a magazine that empties itself looks like a magazine
that is being fired. It only surfaced when the feed was metered properly, because until then
the gate was held open whenever the robot was ready and the balls left as shots instead.

Not fixed. The obvious suspect is wrong: the slots do overlap the feed tube's outer wall by
about 4 mm on paper -- the stand-off is measured from `sh.half`, the BORE, and the wall is a
plate 2t thick outside it -- and correcting that moves nothing, so the balls are not being
placed where the formula says. The next step is to log the world position `preload()` actually
releases each ball at, rather than reasoning about the slot list.

Impact: every land-rate number in this document was measured with a topped-up hopper, so they
describe a robot with an infinite magazine. In a match this robot would spill its preload.

## OPEN: no land-rate metric in this repo is trustworthy yet

`tools/gatecal.ts`, run immediately after `tools/landcal.ts`, on the same robot and the same
table, reported **1.0%** where landcal had just measured **66%** settled per shot. Both cannot
be right, and the two are wrong in opposite directions:

  - `tools/landrate.ts` counts `ballsInUpCell`, which is one CELL of one rocker. The rocker
    ROCKS -- that is the whole mechanism -- and a rotation short of a scored tip carries the
    balls into the down CELL, where this reads zero. A run can score all afternoon and report
    nothing. It also explains the same tool reading 31%, 48% and 1% across three sessions.
  - `tools/landcal.ts` uses the shot log's `result`, set by `world.trackBallStates`, which
    tests `pointInCell` over EVERY cell of BOTH hives. It counts the down CELL and it counts
    the opponent's hive, so it is generous.

Neither is "POLLEN in our up CELL at the buzzer, plus what a tip dumped". Until one of them is,
every land rate in this document -- and `flywheel.minLandProb`, which was set from the landcal
fit -- is provisional. The A/B between the two shot tables is unsettled for the same reason:
the last run put the STEEP table 24.8 points ahead, the opposite of every previous measurement,
which is what a metric sensitive to rocker angle rather than to scoring would do.

The fix is one honest census: balls inside the up CELL of the alliance's own hive, sampled at
the buzzer and at each tip. It should live in `World`, next to the scoring, so that every tool
shares it instead of each one inventing its own.

## OPEN: the robot cannot shoot on the move at all

`tools/collect.ts --shots 40` fires forty shots standing still. The same run with `--moving`
fires **one**, and that one leaves 280 rpm outside its band.

The cause is the interaction between two things that are each individually reasonable. The
motion lead recomputes the required exit speed every loop -- shooting while closing needs less,
while retreating needs more -- so the target rpm is a moving goalpost. The readiness gate needs
the wheel inside `tolRpm` for `readySteps` consecutive loops. A wheel chasing a target that
moves faster than it can track never gets three loops in a row, so the gate never latches.

`FlywheelGate.java` has the same structure and the same guard (`setTargetRpm` resets the band
count when the target moves by more than half the tolerance), so the deliverable inherits it.

This matters more than it looks. The lead is a real piece of work with its own tests, and it
exists precisely so the robot can shoot without stopping -- but as configured, it is machinery
that can never be used. Either the gate has to tolerate a moving target (compare against a
PREDICTED rpm at the moment of release rather than the current one), or the robot's doctrine is
stop-then-shoot and the lead is dead weight. Both are defensible; having the lead and not being
able to use it is not.

### What that means for the shot-zone overlay

The overlay paints the STATIONARY map. It briefly repainted from the live velocity, which is
correct physics -- the scatter is a fraction of exit speed, so retreating widens the group and
closing tightens it, worth 58 squares against 0 at 1.5 m/s in the model. But it was the wrong
thing to draw twice over: the useful question for a floor map is "if I go there, can I shoot",
not "if I were there moving as I am now", and the measurement says the robot would not take the
shot at all. The per-cell parameters still ship and `repaintZone` still takes a velocity, so it
is one line to turn back on when firing on the move works.

## Shooting on the move: it is an ACCELERATION budget, not a speed limit

The lead changes the required exit speed, and for this shooter exit speed is `k*r*omega`, so
the target rpm moves by a fixed amount per m/s of RADIAL closing speed -- 442/cos(hood), which
is 688 rpm per m/s at a 50 deg hood and 1431 at 72 deg. Therefore:

    d(target rpm)/dt  =  (rpm per m/s)  x  (radial ACCELERATION)

Velocity does not appear. A steady 1.5 m/s holds the target rpm perfectly still; only changing
the closing speed moves it. `tools/slew.ts` measures both halves:

| | speeding up | slowing down | acceleration budget at 61 deg hood |
|---|---|---|---|
| 1 motor | 1102 rpm/s | 1588 rpm/s | **1.2 m/s^2** |
| 2 motors | 1687 rpm/s | 2700 rpm/s | 1.9 m/s^2 |
| 3 motors | 2077 rpm/s | 3600 rpm/s | 2.3 m/s^2 |

That is why `tools/collect.ts --moving` fired 1 shot in 40: the AutoDriver drives a range
sweep, accelerating and braking the whole way, so it is over the budget continuously. It was
never a test of shooting at a steady speed.

`tools/movingfire.ts` holds the stick still instead. Four seeds pooled, because the first run
of this quoted a closing rate off a single landed ball:

| case | seconds | shots/s | landed/s | landed | rpm error at fire |
|---|---|---|---|---|---|
| stopped | 120 | 0.42 | 0.26 +- 0.05 | 31 | 50 rpm |
| steady CLOSING at 0.23 m/s | 24 | 0.46 | **0.46 +- 0.14** | 11 | 191 rpm |
| steady STRAFING | 120 | 0.27 | 0.21 +- 0.04 | 25 | 318 rpm |
| closing with the stick WOBBLING | 24 | 0.62 | **0.00** | 0 | 37 rpm |

**Steady motion shoots as well as standing still, in either direction.** Closing measures
higher than stopped and strafing measures lower, and neither gap is more than about one and a
half standard errors -- they are the same number. The prediction held: velocity is not the
variable.

The wobble case is the one that fails, and it fails in an informative way. It FIRES MORE than
any other case, 0.62 shots a second, with the LOWEST rpm error at fire, 37 rpm -- and lands
nothing at all. So the wheel really is on its target at release; the target is simply wrong by
then. The lead is computed when the shot is commanded and the ball leaves after the feed
pulse, so under acceleration it flies with a correction for a velocity the robot no longer has.
That is a latency fault, not a torque one, and it has a cheap fix: lead on the velocity
PREDICTED at release (v + a*latency), or refuse the shot while |radial acceleration| is over
the budget in the table above.

## Shooting while ACCELERATING: the kinematics are easy, the tachometer is not

The accelerating case fires more than any other and lands nothing, with the wheel dead on its
target at release -- so the target is stale, and the fix looks like one line of kinematics:
lead on the velocity at RELEASE rather than the velocity now, `v_release = v + a*tau`.

It is one line, it works, and it is not enough.

| motion | tau | landed | downrange bias |
|---|---|---|---|
| stick wobbling | 0 | 0 | **-57 cm** |
| stick wobbling | 0.30 s | 0 | **-26 cm** |
| steady ramp | 0 | 2 | -47 cm |
| steady ramp | 0.30 s | 2 | **-24 cm** |

The prediction halves the bias in both, which is what the kinematics predict. The landed rate
barely moves, because the other half is the FIRING WINDOW: +-60 rpm is +-21 cm of range against
a pocket 22.5 cm deep. Standing still that error is random, the group straddles the hole and
half the shots drop in. Accelerating, a wheel chasing a falling target lags one way only, so
the whole group goes short and none of it does.

Two further things worth keeping:

**An oscillation defeats a first-order predictor at exactly the wrong moment.** The gate fires
when the wheel matches its target, which is near a velocity peak -- and at a peak the
acceleration is zero, so `v + a*tau` says "it will stay here" one instant before it reverses.
A steady ramp is the case the predictor is built for and it does better there.

**Applied unconditionally it cost stationary shots.** It differentiates a velocity estimate and
a standing robot jitters; a rig that fired three times in ten seconds fired twice. It is now
deadbanded where `a*tau` stops being worth more than the encoder can resolve -- one count over
a 20 ms window is 107 rpm, so below about 0.4 m/s^2 the correction is smaller than the
measurement it would be based on.

Which is the finding: **shooting while accelerating is limited by the flywheel tachometer, not
by the equations of motion.** Better resolution on the flywheel shaft -- a higher-CPR encoder,
or gearing the existing one up -- is what buys it, and it is the same fix that would let
`tolRpm` come down from 60.

## Scrapping the RPM lead: hold the wheel still and let the HOOD do the compensating

Every reason shooting-while-accelerating is hard is a property of the FLYWHEEL, not of the
problem:

  - it has inertia, so it lags a moving target (1102 rpm/s available against 912 rpm/s demanded
    per m/s of radial speed);
  - its speed has to be MEASURED, and one encoder count over a 20 ms window is 107 rpm, so the
    firing window is +-60 rpm -- +-21 cm of range into a pocket 22.5 cm deep;
  - leading on the predicted release velocity halves the resulting bias and can do no better,
    because you cannot correct an error smaller than you can measure.

None of it is true of the hood. It is a servo: commanded to a position, nothing to measure,
nothing to chase. So run the wheel at ONE speed all match and solve for range with the hood --
and stop trying to cancel the robot's velocity at all. It is simply part of the launch:

    horizontal  S*cos(theta) + v        vertical  S*sin(theta)

a different launch speed AND a different launch angle, both known exactly at the instant the
ball goes. `tools/fixedspeed.ts` asks whether a hood angle exists for every range and every
speed the robot can be doing:

| fixed rpm | exit speed | solved, of 35 |
|---|---|---|
| 2600 | 5.88 m/s | 20 |
| 3000 | 6.79 m/s | 27 |
| **3400** | **7.69 m/s** | **28** |
| 3800 | 8.60 m/s | 27 |

At 3400 rpm, 28 of 35 combinations of range (1.0-2.5 m) and radial velocity (-1.5 to +1.5 m/s)
have a hood solution inside the mechanism's existing 30-85 deg travel. The gaps are the corners:
closing fast at short range wants a hood past vertical, retreating fast at long range wants more
speed than one fixed rpm has. A second rpm for the far zone would cover most of what is left --
and it would still be constant DURING a shot, which is all that matters.

### What it buys

| | flywheel | hood |
|---|---|---|
| has to move | 912 rpm per m/s | 8.0 deg per m/s |
| can move at | 1102 rpm/s | 120 deg/s |
| so it tracks | **1.2 m/s^2** | **15.0 m/s^2** |
| set to | a measured speed, +-60 rpm = +-21 cm | a commanded position, 1 deg = 3.5 cm |

No FTC robot accelerates at 15 m/s^2, so the acceleration budget stops being a constraint. And
the precision improves by roughly six times STANDING STILL, because the hood has no measurement
in its loop to be wrong about.

Not implemented yet -- this is the solver saying the geometry allows it. What it needs: the
shot table regenerated as hood-versus-(range, radial velocity) at a fixed rpm, the brain
commanding a constant wheel speed, and the readiness gate reduced to "is the hood there yet",
which is a servo position and settles in tens of milliseconds rather than tenths of a second.

## BUILT AND MEASURED: the fixed-speed shooter is worse. Here is why, exactly.

Built end to end -- `tools/hoodtable.ts` generates hood-versus-(range, radial velocity) at one
wheel speed, `HoodTable` interpolates it, `BuiltinTeleOp` takes it as a fourth argument and
switches the wheel to a constant target with readiness on the HOOD instead of the wheel, and
`tools/shooterab.ts` runs both shooters over identical seeds and scripted driving.

It loses, and not narrowly:

| case | speed-solving | fixed-speed + hood |
|---|---|---|
| stopped | 0.28 landed/s | **0.02** |
| steady closing | 0.33 | 0.00, "no shot from here at this speed" |
| steady strafing | 0.24 | 0.07 |
| accelerating | 0.00 | 0.00 |

### The reason, which is the useful part

A fixed speed forces a single tradeoff and there is no setting that wins it:

| fixed rpm | exit | cells with a solution | mean hood band | mean sigma | cells where band/2 > sigma |
|---|---|---|---|---|---|
| 2600 | 5.88 m/s | 40 | 6.62 deg | 3.30 deg | **63%** |
| 3000 | 6.79 | 71 | 4.80 | 2.84 | 39% |
| 3400 | 7.69 | 103 | 3.41 | 2.60 | 18% |
| 3800 | 8.60 | 120 | 1.56 | 1.85 | 4% |
| 4200 | 9.50 | 117 | 0.92 | 1.10 | **0%** |

COVERAGE NEEDS SPEED AND PRECISION NEEDS SLOWNESS. A slow wheel gives a forgiving hood band
but only reaches a third of the envelope; a fast one reaches everything and the band closes to
under a degree, narrower than the 1 deg of launch elevation scatter.

And the deeper point: **the error I moved was never the dominant one.** The wheel's 1.5%
speed scatter is present in BOTH designs and is worth 13 cm of range in the speed-solving
shooter and 18 cm in the fixed-speed one -- against a pocket 22.5 cm deep. Changing which axis
compensates does not remove it; it just re-expresses it, and at a higher wheel speed it makes
it worse.

What the fixed-speed design DOES remove is the wheel's tracking LAG, which is the thing that
kills the accelerating case. That was the right diagnosis. It is simply worth less than the
coverage and precision it costs.

### So the real lever

`flywheel.scatter.speedFrac = 0.015`, which is an UNMEASURED guess. Everything above is
downstream of it. A shooter that puts the same speed on every ball -- a dual-wheel nip, a
consistent compression, a hood that does not let the ball skip -- moves every number in this
document, standing still or moving. Nothing in the aiming can fix a ball that leaves at a
different speed each time.

The fixed-speed path is left in the code behind `BuiltinTeleOp`'s fourth argument, unused by
the app: it is a working implementation of a measured dead end, and cheap to re-test if the
shooter's consistency ever changes.


## 2026-09-17 — The motion lead never solved the hood, so no moving shot could arrive
Plan said:   Compensate the robot's own velocity so the ball flies at the target.
Found:       `leadShot` / `ShotLead.solve` solved the horizontal triangle only — azimuth and exit
             speed — and left the hood at the shot table's angle. That holds the ground track
             exactly and breaks the hang time: the ball leaves with a vertical of `mag·tan(el)`
             instead of `S·sin(el)`. Closing at 0.4 m/s from 40 in it drops from 4.97 to 3.85 m/s
             and the ball NEVER REACHES the mouth's 1.46 m; retreating sails over it. Three
             commits had been spent on the flywheel's tracking lag, on a lead that could not have
             landed a moving shot from a perfect shooter.
Did instead: Solve all three unknowns from the table's launch VECTOR — `el = atan2(vert, mag)`,
             `speed = hypot(mag, vert)` — clamped to the hood's travel. Exactly the arithmetic
             `tools/hoodtable.ts:launch()` already used for the fixed-speed shooter. Wired into
             `AimController`, which had been constructing `ShotLead` and never calling it, so the
             hub led on nothing at all while the sim's mirror led on two axes of three.
             `tools/leadcheck.ts` prints the landing error with no scatter, gate or feeder: 0 cm
             everywhere against NEVER ARRIVES.
Costs/risks: The hood now carries the correction, so it must track; measured at 8.0 deg per m/s
             against 120 deg/s of travel, and `tools/fixedspeed.ts` prices that at 15 m/s² of
             radial acceleration against the drivetrain's ~2. Past the hood's stops the shot is
             clamped and degrades rather than failing — only reachable charging the goal at most
             of top speed from close in. `transfer.leadLatency_s` is now worth nothing measurable
             and is kept only as a knob for a real hood's lag.
Who/where:   builtinTeleOp.ts, ShotLead.java, AimController.java, tests/shotlead.test.ts,
             SelfCheck.java, tools/leadcheck.ts, config/robot.json

## 2026-09-17 — tools/movingfire.ts was measuring a stationary robot off the end of the table
Plan said:   (nothing — the tool is its own answer)
Found:       Three faults compounding. It started the robot 41 in from the mouth (the wall is at
             1.45 m and the mouth at 0.40 m), then settled for 90 frames WITH THE CASE'S STICK
             HELD, which drove a closing run to 21 in before a single shot was judged — below the
             shot table's first row, where every lookup returns the 30 in answer. The drive loop
             then broke out on its first frame, and because fire is a LATCH the case took nearly
             all of its shots during the six-second settling tail, standing still. "Closing,
             stick wobbling lands nothing" was a stopped robot at an unsolvable range.
Did instead: Place at a requested range on the side the CELL mouth OPENS TO (the far corner is
             the back of the goal — placed there the stopped control landed 2 shots in 80 s);
             spin the wheel up standing still, THEN settle the drive; count only in-band time and
             only shots fired while driving; pool short passes until the requested seconds
             accumulate, because a 3.59 m field with the goal in the middle sustains steady
             motion in no direction for twenty seconds.
Costs/risks: Rates are pooled over passes and seeds, so a case with little runway has fewer
             independent samples than its second count suggests. The band [33, 82] in is the
             table's coverage less the field, not a physical limit.
Who/where:   tools/movingfire.ts

## 2026-09-17 — The intake roller was spinning about the world vertical
Plan said:   (nothing — render only)
Found:       The roller was laid down with `rotation.z = PI/2` and then driven with `rotation.y`.
             An Object3D's euler is XYZ, so the y term turned the laid-down cylinder about the
             WORLD vertical: the axle swept round like a clock hand instead of the roller turning
             on it, and on screen the intake was a bar pivoting diagonally out of the robot's
             front corner. It could not have shown the spin even with the axis right — a smooth
             one-colour cylinder looks identical at every angle. It was also driven from the
             commanded POWER, so a jam still looked like a running intake, and it drew a tilted
             scoop plate that `robot.ts` deletes in as many words ("NO RAMP").
Did instead: Lay the roller over once at build time and turn the group about its own X, the way
             the mecanum wheels forty lines up already did. Compliant wheels with tangential
             treads, so the rotation reads. Driven from the shaft's measured `omega`, scaled by a
             constant so 116 rad/s does not strobe at 60 Hz. Sized from `robot.json`'s mouth and
             roller radius rather than four hand-tuned numbers, so the picture cannot drift from
             what `stepIntake` sweeps.
Costs/risks: `Scene` now takes the RobotSpec. The displayed spin rate is 0.12 of the real one —
             proportional, so a stall still stops it, but it is not a tachometer.
Who/where:   packages/render/src/scene.ts, packages/core/src/types.ts, physics/robot.ts, ui/main.ts

## 2026-09-17 — The readiness gate could not see the hood, which is now the aiming axis
Plan said:   (nothing — the gate grew with the shooter)
Found:       `hoodThere` was hard-wired true whenever the fixed-speed table was absent, from
             when the hood only ever held the shot table's stationary angle and arrived long
             before the flywheel did. The motion lead SOLVES the hood now, so it moves every
             loop and carries most of the correction — and nothing waited for it. Shots left
             mid-slew at an elevation belonging to a velocity the robot had already left, and
             the probability model could not see it happen: there is no hood term anywhere in
             the speed-solving product, so a shot taken 10° off still scored 85%.
Did instead: Gate on the hood having arrived, in both table paths. A servo is COMMANDED rather
             than measured, so this is a readiness question and not another factor in the
             probability — once the hood is there the ball leaves with the table's launch
             vector and the speed band measured standing still is valid again. `hood.tolDeg`
             is 2°, derived from what the wheel is already allowed: tolRpm = 60 is worth ±21 cm
             of range and 1° of hood is 3.5 cm, so 60 rpm ≈ 6° of hood.
Costs/risks: It closes on under 1% of loops at this servo speed, so it costs almost nothing —
             but a real servo is slower than 120 °/s and will need it loosened, or the shot
             will start waiting on the hood instead of the wheel.
Who/where:   builtinTeleOp.ts, AimController.java, config/robot.json, types.ts

## 2026-09-17 — The gate was set above the best the shooter can do, so it held fire
Plan said:   Refuse a shot unless P(land) clears `flywheel.minLandProb`.
Found:       `minLandProb` was 0.900 against a calibration whose measured CEILING was 0.8983.
             The threshold was two tenths of a point ABOVE anything the shooter can honestly
             claim, so it could be met only by rounding, and the robot sat in perfectly good
             zones holding fire. Measured with the app's own setting: 0.00 landed per second
             standing still at 50 in, where the same robot with the gate open landed 0.45.
             This is what "it only shoots sometimes" was. The calibration was also fitted from
             a robot that never moved, then applied to a robot that mostly does.
Did instead: `tools/landcal.ts` now samples three driving states as well as three ranges, so
             the curve describes the conditions the gate is used in — and it reports the split
             (standing still 80% of 117, on the move 76% of 176, which is the corrected lead
             showing up as a number). Then `tools/movingfire.ts --sweep` prices the threshold
             in BALLS PER SECOND rather than per shot, because refusing a 60% shot is only
             right if a better one turns up inside the 1.5 s the refused cycle costs. 0.85
             gives up nothing (0.45 / 0.65 / 0.45, identical to an open gate) and takes the
             hit rate of the shots it allows from 64% to 90–100%. 0.90 is zero everywhere.
Costs/risks: The number is only meaningful against the curve it was measured with. Re-run
             landcal and then the sweep whenever the shooter changes. The ceiling itself is a
             fact about the shooter, not the gate: no threshold above 0.898 can ever be met.
Who/where:   config/robot.json, config/landcal.json, tools/landcal.ts, tools/movingfire.ts

## 2026-09-17 — LandProbability was implemented, self-checked and called by nothing
Plan said:   The hub refuses a shot it does not expect to land.
Found:       `LandProbability` is complete on the Java side, with its constants generated into
             `ShotTableData` and its maths asserted by the build's self-check — and no caller.
             The deliverable's only gate was the RPM window: one fixed band at every range,
             blind to whether the turret was pointing into the mouth and blind to whether a
             ball arriving like that stays in. The simulator's mirror had all three factors.
Did instead: Wired it into `AimController`, against the same baked calibration, with the
             measured speed mapped back into the frame the table's band was solved in. The hub
             now prints `P(land) 67% < 85%` where it used to say READY and miss.
Costs/risks: The hub will now hold fire in places it previously shot. That is the point, but
             it makes `minLandProb` a live number on the robot rather than a simulator setting.
Who/where:   AimController.java, Turret.java, RobotConfig.java, genconstants.mjs

## 2026-09-17 — The readiness gate was a decision, not a permission
Plan said:   Fire when the shot is good.
Found:       `feedOne` commits about four tenths of a second before the ball leaves — the feed
              pulse plus the climb up the tube — and that was final. Every axis keeps tracking
              in the meantime, so the AIM at release is current; what went stale was the
              PERMISSION. It showed up as a tail of badly wrong shots rather than as lost
              precision, which is why every summary statistic had hidden it: shuttling fore and
              aft the TYPICAL shot was better than a stationary one (median 1 cm off line, IQR
              [−10, +10] downrange) while 20 of 108 landed over 60 cm out.
Did instead: The release re-checks the two things that can go bad inside those four tenths and
              are geometric rather than measured — the turret running out of travel, and the
              turret falling behind under a turning chassis. NOT the full readiness latch: that
              needs three consecutive good loops against a tachometer quantised to ~107 rpm a
              count, and demanding an unbroken 0.19 s while the gate servo travels took the
              stopped case from 78 shots to 3. NOT the wheel either — a floor is unmeetable for
              a robot whose range is growing (it took strafing to zero), and re-checking P(land)
              rides the same quantisation and flickers.
Costs/risks: A robot spinning at 134 deg/s now refuses most shots. That is correct — the turret
              cannot hold the goal — but it is a visible behaviour change.
Who/where:   builtinTeleOp.ts, Transfer.java, AimController.java

## 2026-09-17 — The gate could not tell an aimed turret from one on its end stop
Plan said:   Hold fire unless the turret is on target.
Found:       `turretErrDeg` is measured against the CLAMPED command. The turret travels ±120°;
              when the lead asked for more the command was silently clamped and the error read a
              fraction of a degree on an axis pinned against its stop. Over a spinning run the
              lead asked for a bearing 17.7 ± 18.5 deg OUTSIDE the travel, the gate said
              on-target, and the shots went out up to 50 deg wide: lateral miss −51 ± 65 cm
              against ±11 standing still, and the largest single error term in the harness.
Did instead: Keep what the lead ASKED for, before the clamp, and refuse the shot when the two
              differ. `TurretTracker.canReach` on the hub has always tested this; the mirror
              never did, and the mirror is what every tool in this repo measures.
Costs/risks: None found. It only ever refuses shots that could not have been aimed.
Who/where:   builtinTeleOp.ts

## 2026-09-17 — The flywheel was not one a real FTC shooter would build
Plan said:   A single bare 6000 rpm motor direct-driving a compliant wheel.
Found:       At 1.21e-4 kg·m² — a bare 105 g grip wheel — one POLLEN takes 210 rpm out of the
              wheel on its way past, three and a half times the 60 rpm firing window. The
              tachometer cannot see it coming either: the hub reports velocity quantised to
              about 107 rpm a count on a 28-tick encoder, filtered over six loops. Every wild
              shot left in the shuttling case was that gap — fired with the true speed 367 ± 101
              rpm under target while the filtered reading said it was fine.
Did instead: The wheel plus a 96 × 12 mm 6061 disc behind it: 3.91e-4 kg·m², 0.34 kg on the
              shaft, which is what a team building a shooter that works actually bolts on. The
              dip falls to 70 rpm and spin-up goes 0.74 s → 2.0 s, which is normal. Adding
              INERTIA rather than a second motor because the robot is already at FTC's eight-
              motor limit (four drive, intake, transfer, flywheel, turret) and a second flywheel
              motor would have to come out of one of those.
Costs/risks: Slower spin-up. The shot table is unaffected — it depends on k and r_fly, not I —
              but `flywheeltune --ff` was re-fitted and landcal re-run, because the calibration
              is only meaningful against the shooter it was measured on.
Who/where:   config/robot.json, tools/shootercheck.ts

## 2026-09-17 — The robot had a perfect localizer, and its noise config was dead
Plan said:   `sensors.localizer.noise` — xy_in and heading_deg.
Found:       Both were zero and neither was read by anything: `sensors()` handed the brain
              ground-truth position, heading AND velocity. The velocity is the one that matters,
              because the whole motion lead is built on it — so a lead validated against a
              perfect estimate had never been validated at all. There was not even a field for
              velocity noise.
Did instead: Wired the config to the sensor, added `vel_mps` and `omegaDps`, and set them from
              a two-pod odometry puck: 0.04 m/s, which against a 2.3 m/s ball horizontal is
              about a degree of bearing — the same order as the launch yaw scatter already
              modelled. Drawn from the world's seeded RNG, so determinism holds.
              It immediately stopped the robot shooting: the raw reading jitters the lead
              azimuth, the turret chases the jitter, its tracking error never settles under the
              gate's 3 deg, and three shoot tests fired nothing. The answer is the one a real
              team reaches for — filter the reported velocity before aiming on it. One pole,
              alpha 0.25, about 50 ms of lag against a quantity that moves on the timescale of
              the robot's own acceleration.
Costs/risks: The TARGET bearing and range are still exact, and that is now the bigger lie: on a
              real robot they come from an AprilTag pipeline with its own noise and 50–100 ms of
              latency. Listed in PHYSICS.md §7.
Who/where:   world.ts, builtinTeleOp.ts, AimController.java, config/robot.json, types.ts

## 2026-09-17 — Backspin does not help the ball stay in the CELL; it costs about 7 points
Plan said:   (nothing — the single-wheel layout was assumed)
Found:       The usual intuition is that a single wheel's backspin digs into the pocket and
              kills the ball's forward speed. Measured with no shooter and no flight in the way
              (`tools/spincheck.ts`, which injects at the mouth with and without the spin
              `Robot.launch` actually produces): backspin 75.2% +-1.8 against 85.1% +-1.5 with
              none, over 576 balls a side, worse in 8 of 9 cells of the band the table arrives
              in. Outside two standard errors. Friction at the pocket floor pushes a
              backspinning ball back toward the mouth it came in through, which is the opening.
              End to end, each layout given the table its own physics implies, a dual-wheel
              shooter also solves MORE ranges (25 of 31 against 22) for 75 rpm more exit speed,
              the Magnus lift being what it gives up.
Did instead: NOTHING — kept the single wheel, for a reason that has nothing to do with entry.
              The hood is the single-wheel layout's aiming surface, and the motion lead now
              solves the HOOD: that is what made shooting on the move work at all. A dual-wheel
              shooter has no hood, so the whole correction goes back onto the flywheel, which is
              the design that could not track a moving target. The 7 points are cheaper than
              that.
Costs/risks: The entry result rests directly on `ball.e_poly` and `ball.mu`, and the flight side
              on `ball.clSlope` -- all three are flagged guesses, and they are exactly the three
              most worth measuring on a real ball. Re-run tools/spincheck.ts once they are, and
              be ready for the sign to change.
Who/where:   tools/spincheck.ts, tools/entrycheck.ts (entryRate now takes a spinFactor)

## 2026-09-17 — A second flywheel motor is worth a lot, and the robot has no port for it
Plan said:   One motor on the flywheel.
Found:       Measured through the gate the app enforces, landed per second, heavy flywheel:
              one motor 0.36 / 0.56 / 0.00 / 0.32 / 0.28 across stopped, closing, strafing,
              shuttling, closing-while-wobbling; two motors 0.36 / 0.68 / 0.24 / 0.36 / 0.36.
              The second motor is what makes the RECEDING case possible at all -- the target rpm
              rises with the range and one motor cannot chase it. It is not a substitute for the
              flywheel inertia and the inertia is not a substitute for it: two motors on the OLD
              light wheel gives back the wild shuttling shots (lat 17 +-55 cm) and craters the
              stopped case to 0.12, because inertia fixes the per-shot dip and motors fix the
              chase. Both together is strictly best.
Did instead: NOT SHIPPED. The robot is at FTC's eight-motor limit -- four drive, intake,
              transfer, flywheel, turret -- so a second flywheel motor has to come out of
              another subsystem, and which one is a design decision about someone's robot rather
              than a config edit. The three real ways to free the port: gang the feed belt off
              the intake shaft (the belt already runs continuously and the gate is the release,
              so this costs almost nothing behaviourally), put the indexer on a CR servo (ten of
              the twelve servo ports are free), or put the turret on a servo.
Costs/risks: Two motors also double the flywheel's current draw on a 12 V pack already running
              four drive motors, which is not modelled as a brownout risk.
Who/where:   measured with config/robot.json flywheel.motorCount, which the physics supports

## 2026-09-17 — The ball leaves with the MUZZLE's velocity, and the aim now subtracts it
Plan said:   docs/PHYSICS.md known-limitation 7: the muzzle's offset and the omega x r velocity
              a turning chassis gives it are "worth a degree or two ... so left alone".
Found:       Left alone was the wrong reading, and the reason it looked small is the
              interesting part. Robot.launch() added body.linvel() -- the tracked point's
              velocity -- and leadShot()/ShotLead.solve() subtracted the same thing, so the aim
              and the flight agreed with each other. Two halves agreeing is not either half
              being right: a real muzzle 0.12 m out on the turret swings at omega*r, which at
              90 deg/s is 0.19 m/s and 4.7 in of lateral miss at 60 in, against 0.5 in of
              clearance to a lip. tools/shoterror.ts had been printing a `muzzle v_lat` column
              the whole time, correlated against a world in which that velocity was never
              applied. Localizer.getOmegaDps() was implemented on both sides and read by nobody.
Did instead: Both halves, in one change, because either alone is worse than neither.
              Robot.launch() gives the ball v_cg + omega x r about the muzzle point (not
              exitPoint(), which is a spawn offset to clear the chassis collider and not a claim
              about where the ball leaves the hood). builtinTeleOp.muzzleVelocity() and
              ShotLead.muzzleVelocity() compute the same vector from the localizer's omega and
              the turret's MEASURED angle, and it feeds both consumers of the velocity: the lead
              and the hood table's radial axis. Same seeds, tools/shoterror.ts, only those two
              files differing:

                              lat median, cm      shots taken
                turning 50in    -7  ->  -3         31 -> 39
                turn+drive      -3  ->  -3         52 -> 50
                stopped          0  ->  -0         67 -> 60
                shuttling       -0  ->   0         60 -> 60
                closing 76in    -0  ->  -0         24 -> 24

              Every case with omega = 0 is unchanged to the centimetre, which is the signature
              this term should have; the turning case is where it lives and it halved there.
              The simulator cannot measure the part that matters most -- that java/teamcode is
              now correct against real hardware rather than against this world's own omission.
Costs/risks: The lever arm assumes the turret axis sits over the tracked point, which is what
              Robot.muzzle() builds (pivot [0, h, 0] in the body frame). Move the axis off the
              origin there without moving it here and the aim compensates a term the flight does
              not have -- this bug with the sign flipped. tests/shotlead.test.ts round-trips the
              two against each other and is what fails if someone does.
              The muzzle's POSITION is still ignored when range and bearing are taken; that half
              really is a centimetre or two and is still open.
Who/where:   packages/core/src/physics/robot.ts, packages/core/src/robot/builtinTeleOp.ts,
              java/teamcode/.../control/{ShotLead,AimController}.java, tests/shotlead.test.ts,
              java/runner/.../SelfCheck.java, docs/PHYSICS.md

## 2026-09-17 — The release re-check is the two turret conditions, on BOTH sides
Plan said:   AGENT_PROMPT.md section 4: java/teamcode is the deliverable and the mirror mirrors it.
Found:       They had become two robots. Commit 109c21e's own message rejected both a wheel floor
              ("unmeetable for a robot whose range is growing -- it took strafing to zero") and a
              P(land) re-check ("rides the same quantisation and flickers, stopped 0.36 -> 0.12"),
              and then each half kept one of them -- a different one. The mirror had since been
              reduced to the two turret conditions; AimController still ANDed in the wheel floor.
Did instead: AimController.update() re-checks canReach() && onTarget(3.0) and nothing else, the
              same expression the mirror uses. cfg.flywheelMinRpmFrac is untouched and still
              drives Flywheel's readiness gate, which is a different question from whether a feed
              already in flight should be cancelled.
Costs/risks: Nothing re-checks the wheel between the decision and release now. That is deliberate
              -- it is what the readiness latch is for -- but it means a wheel that sags inside
              those 0.4 s fires anyway.
Who/where:   java/teamcode/.../control/AimController.java

## 2026-09-17 — Three rules the config was breaking, and a test that now reads them
Plan said:   config/robot.json limits: maxMotors 8, maxServos 12; hopper capacity 6.
Found:       R503 caps servos at 8, not 12 -- two hubs give 12 PORTS, and ports only cap the rule
              further. G407 caps CONTROLled SCORING ELEMENTS at 4, not 6, so every cycle time and
              autonomous ball budget in the repository was computed for a robot that would be
              penalised. Both numbers were in the spec type and READ BY NOTHING, which is also how
              a nine-motor configuration (flywheel motorCount 2 on top of an eight-entry hardware
              map) reached the working tree. Separately, the chassis block's _source had been
              overwritten with sensors.localizer.noise._source word for word, and docs/VARIABLES.md
              is generated from that field, so the generated documentation stated that the chassis
              mass came from an odometry puck.
Did instead: maxServos 8, hopper capacity 4, chassis _source restored (without the old note's
              "at the weight limit", which cites a limit R104 says does not exist). tests/rules.test.ts
              counts the hardware map against both limits, charging a multi-motor flywheel one port
              per motor -- it reports 9 against the motorCount: 2 config, which is the check that
              was missing. tools/vars.mjs now fails when two blocks share a _source longer than 80
              characters. tools/movingfire.ts tops the hopper up to spec.hopper.capacity rather
              than to a hard-coded 7.
Costs/risks: Capacity 4 was thought unusable because the indexer could not feed from a nearly empty
              bin (the OPEN entry above). Re-measured: it feeds. tests/shoot.test.ts fires 12 shots
              in 23 s from a 4-ball bin against 11 in 67 s from a 6-ball one, so indexLift 0.6
              closed that one; consider the OPEN entry answered.
              The earlier entry's "ten of the twelve servo ports are free" is now six of eight.
              Freeing the eighth motor for a second flywheel motor is still unshipped and still the
              open design decision.
Who/where:   config/robot.json, tests/rules.test.ts, tools/vars.mjs, tools/movingfire.ts,
              java/teamcode/.../config/{RobotConfig,RobotConstants}.java, tools/genconstants.mjs


## 2026-09-17 — The CELL pocket was 11 deg out, and three layers of calibration sat on it
Plan said:   geometry.ts derived the pocket's radial direction from the CELL assembly
              centroid: atan2(11.28, 53.77 - 43.95) = 48.96 deg from vertical, and used that
              one angle for BOTH where the pocket sits and which way it points.
Found:       Those are two different angles, and this rocker holds them 20 deg apart. Taking
              one for the other put the mouth's lips at 52.5 and 62.7 in against the manual's
              53.5 and 65.6 (Fig 9-10), the apex 2.9 in low, and the opening facing 41 deg
              above horizontal where the real one faces 30.

              The CAD settles it without reference to the manual. `up_back_skin_bbox_in` is
              the pocket's FLOOR plate: 12.95 in of Y over 7.48 in of Z, so the plate is
              hypot(12.95, 7.48) = 14.955 in long -- the 14 in mouth plus its skin -- and its
              long axis is atan2(7.48, 12.95) = 30.01 deg off vertical. The pocket axis is
              that plate's normal, 59.99 deg from vertical. Walk 12.04 in (cellDepth) up the
              normal from the plate's centre (53.525, 5.360) and the mouth centre lands at
              Y 59.55; step +-7 in along the plate and the lips land at 53.49 and 65.61. The
              manual says 53.5 and 65.6. Two independent sources, 0.02 in apart.

              PHYSICS 9.3 proposes a different repair -- keep one angle and re-derive the
              radius from the lip midpoint -- and its formula does not survive the CAD: it
              puts the pocket centre at Z 21.8 in against the CAD's 11.3, on an arm 25 in long,
              on a frame whose half-depth is 19.5 in. The lip heights come out right and the
              pocket ends up somewhere the rocker is not. Decoupling the two angles gets both.
Did instead: CellGeometry carries `bodyAngle_rad` (the ARM: 70.03 deg in the body frame, at
              radius 16.438 in) and `axisAngle_rad` (the pocket's own axis: 89.99 deg, so 60
              deg from vertical once the rocker sits on its 30 deg stop). REST_ANGLE_DEG is
              now the CAD's `cells.arm_tilt_deg` = 30 exactly, cited rather than derived from
              a centroid -- it is a STOP angle, it sets the holding torque and therefore the
              tip threshold, and tangling it with the pocket is what made this look as though
              moving the pocket had to move the stops.

              Five call sites had each inlined `(radius + u) * cos(bodyAngle)`, so each had
              inlined the same mistake; they all go through `fromCellLocal()` now.
              tests/geometry.test.ts asserts the manual's lip heights directly.
Costs/risks: Everything fitted against the old pocket had to be refitted, in this order,
              because each stage feeds the next: entry.json (tools/entrycheck.ts) -> the hood
              range -> shottable.csv -> landcal.json -> the minLandProb sweep. Do not move the
              pocket without re-running that chain: a shot table aimed at a pocket it was not
              fitted to is worse than either error alone.
Who/where:   packages/core/src/field/geometry.ts, physics/{hive,world}.ts, render/scene.ts,
              ui/main.ts, tools/{shottable,hivedrop,cellprobe}.ts, tests/geometry.test.ts

## 2026-09-17 — What the corrected pocket was worth
Plan said:   the shot table sat at 75-85 deg of hood, lobbing balls in nearly vertically, and
              tools/landcal.ts measured 66% settled per shot.
Found:       Refitting the chain moved every stage of it, and the answers now agree with the
              design table in PHYSICS 2.4 -- which was derived from the manual with no
              simulator in the loop, so this is a cross-check and not a tautology:

                                   before          after       PHYSICS 2.4 predicted
                arrival      ~84 deg down    30 deg down              25-30 deg down
                hood range          30-85          40-80                       47-69
                apex                    -     64 in mean                    61-63 in
                landcal               66%     85% standing, 89% ON THE MOVE
                gate 0.70     unreachable     calibrated, 77% ungated

              End to end, tools/movingfire.ts, 90 s of in-band time per case, landed per
              second, against the same harness at 109c21e:

                                  before   after
                stopped             0.12    0.53
                steady closing      0.58    0.71
                steady strafing     0.07    0.79
                fore/aft shuttle    0.14    0.50
                closing, wobbling   0.51    0.43

              The lateral bias went with it: stopped was +24 +-13 cm and is +2, the shuttle
              was +37 +-54 and is -1 +-7.
Did instead: Nothing beyond the chain itself. Two tests had encoded the OLD pocket's
              behaviour and were rewritten rather than relaxed. The entry model's "a fast
              steep arrival is worse than a slow one" INVERTS, and there is a mechanism for
              it: the pocket axis is 30 deg above horizontal, so a ball arriving 45 deg down
              comes in almost along the axis, strikes the flat floor plate square on and
              rebounds out the way it came -- that row collapses to 0% above 6 m/s. A steeper
              arrival hits obliquely and is trapped by the far lip. It is the same friction
              story tools/spincheck.ts found for backspin. The shot zone map's near/far
              ordering now holds on its own and needed no change.
Costs/risks: The entry grid rests on e_poly, ball.mu and clSlope, all three flagged guesses
              (PHYSICS 9.20, protocol 14.2). The winning corner of that grid is a measurement
              and it will move when they are measured -- re-run tools/entrycheck.ts then.
              The wobbling case is the one that did not improve; that is the known
              release-latency fault, not a geometry one.
Who/where:   config/{entry,landcal,robot}.json, java/teamcode/assets/shottable.csv,
              tests/{landprob,geometry}.test.ts

## 2026-09-17 — A tip swaps which way the goal faces, and nothing knew
Plan said:   `game.hiveTipping` is enough: do not shoot at a moving goal (G417).
Found:       It covers the tip and not the minute after it. A TIP makes the OTHER CELL the up
              one, and the new one opens the other way: measured here, the mouth jumps from
              Z +15.7 in to -16.1 in the instant the rocker goes over. A robot that was square
              onto the goal is now standing behind it, and no launch can enter.

              Nothing checked. The stopped control case tipped the HIVE at t=20 s and then
              spent 70 s reporting "clear to fire" and putting 40 more balls into the back of
              the pocket, every one scored as a miss. That is most of why the stopped case
              read 0.12 landed per second while tools/landcal.ts, which stops before a tip,
              measured 85% -- and why a 20 s run and a 90 s run of the same case disagreed by
              a factor of four.
Did instead: `game.upCellOpenDeg`: the angle between the up CELL's outward mouth normal and
              the direction to the robot. Both brains hold fire past 75 deg (at 90 the opening
              is exactly edge-on and has no area at all) and say why -- "mouth faces away,
              N deg off its opening - DRIVE ROUND". On the hub it arrives through
              TargetProvider, where a real robot would get it from the tag leaving view.

              tools/movingfire.ts ends a pass at a TIP, for the same reason it ends one that
              leaves the band: the question was "can this robot shoot from here", and after a
              tip the premise is void. On the TIP and not on the live angle -- the rocker ROCKS
              when a ball lands and its mouth normal swings with it, and gating the pass on the
              instantaneous angle ended every stopped pass after 4.3 s and three shots. The
              brain is right to hold through a rock; the harness is not right to call the pass
              over because of one.
Costs/risks: In the simulator this is ground truth, like the bearing and the range, so it
              inherits PHYSICS 9.10: a real pipeline has latency and dropout and this has
              neither. The 75 deg is geometric, not measured.
Who/where:   packages/core/src/physics/{world,hive}.ts, robot/builtinTeleOp.ts,
              java/teamcode/.../control/{TargetProvider,AimController}.java,
              java/simsdk/.../SimTargetProvider.java, tools/movingfire.ts

## 2026-09-17 — One bore, three findings
Plan said:   the feed tube is sized for ONE POLLEN, 3.20 in, because a NECTAR-sized bore is
              4.1 in and two 2.8 in POLLEN fit side by side in 4.1 in.
Found:       The arithmetic is right and the conclusion does not follow. The number that
              decides "single file" is the DIAGONAL of two POLLEN, 2.8*sqrt(2) = 3.96 in, not
              5.6 in of side by side. Anything in [3.62, 3.96) takes a NECTAR and is still
              single file.

              And 3.20 in did not merely refuse NECTAR -- PHYSICS 9.12, which alone means the
              simulator could not shoot the ball worth 2.5 points in the HIVE and the entire
              endgame in the FLOWERS. It left a POLLEN 0.2 in of clearance a side, and with
              that little a ball entering slightly crooked WEDGES in the doorway, below
              `entryY`. That latches `entryBusy` true for ever: the indexer admits nothing
              more, and the belt cannot free what is stuck. Traced on seed 43 -- one ball
              sitting at -2.23 in for an entire run with the belt driving and the gate
              cycling, `shots 0`. That is the README's "counts its hopper down from 6 to 2 and
              the world records shots 0" (9.14) and the OPEN "indexer cannot feed from a
              nearly empty bin" entry above (9.13). One number, three findings; consider that
              OPEN entry answered.
Did instead: `transfer.boreSize_m` = 0.0965 (3.80 in), in the middle of the legal window with
              0.17 in of margin each side. The same seed fires normally now.
              tests/rules.test.ts asserts the bore passes a NECTAR, refuses two POLLEN on the
              diagonal, and leaves more than 0.3 in of clearance.
Costs/risks: A wider bore is easier to jam two balls into if the indexer's metering ever
              regresses; the diagonal is the guard, and it is a test now. The bore is a design
              number, not a measured one - CALIBRATE against the real tube once it exists.
Who/where:   packages/core/src/physics/robot.ts, types.ts, config/robot.json,
              tests/rules.test.ts

## 2026-09-17 — Three game-model errors: staged NECTAR, FLOWER ownership, the land census
Plan said:   "A match starts with empty CELLs"; a FLOWER belongs to whoever has more NECTAR in
              it; each tool counts landings its own way.
Found:       All three wrong, and the first is the most consequential thing in the game model.

              STAGED NECTAR (9.4). Manual 10.3.1 B.i stages "3 NECTAR in each upward-facing
              CELL of corresponding color", and Fig 10-2 shows them. The second reason the old
              note gave -- that staged balls rendered floating under the CAD skin -- was a
              symptom of the pocket being 11 deg out, and went with it. Measured after the fix:
              3 NECTAR settle at a 9.6 in lever and hold the rocker 48% of the way over,
              against STRATEGY.md section 4.2's predicted 40-55%. So a real first tip costs
              about six POLLEN where an empty-CELL simulator charged twelve, and every
              autonomous plan timed here was wrong in the same direction.

              FLOWER OWNERSHIP (9.5). Manual 10.5.2: "the ALLIANCE that has the TOP-MOST
              NECTAR of its color owns that FLOWER." The scorer compared COUNTS, so an
              alliance that caps the opponent's three with one of its own was scored the loser
              of that flower. That made the whole endgame of STRATEGY.md section 8 -- plug
              early, cap late, saturate the tube -- invisible, and capping look worthless.

              THE LAND CENSUS (9.15). Two definitions, disagreeing. `ballsInUpCell` reads one
              CELL of one rocker, and the rocker ROCKS, so a rotation short of a scored tip
              carried balls into the down CELL where it read zero. The shot log's `result`
              tested `pointInCell` over every cell of BOTH hives, so it counted the down CELL
              and the opponent's. gatecal read 1.0% where landcal had just measured 66% on the
              same robot and the same table.
Did instead: World.stageCells(), called after the rockers are back on their stops -- a reset
              from a tipped hive otherwise staged into the CELL about to swing underneath.
              Scorer takes `flowers[].topNectar`, found by height in the sweep that already
              located the bottom-most one.
              World.landedInUpCell(alliance) is the single census: balls in OUR up CELL now,
              plus whatever was in it at the instant of each TIP, sampled every step so a tip
              is caught before the pocket empties. The shot log's `result` and
              tools/landrate.ts both read it. landrate reports a consistent 63% at 40, 55 and
              70 in where it used to read 1%.
Costs/risks: landrate subtracts the three staged NECTAR as a constant; if the staging count
              changes, that constant has to change with it.
Who/where:   packages/core/src/physics/{world,hive}.ts, rules/scoring.ts, tools/landrate.ts,
              tests/determinism.test.ts


## 2026-09-17 — "The ball lands and goes under the HIVE": a parked ball is not out of play
Plan said:   `BallSet.park()` takes a ball out of play by disabling its collider.
Found:       It disabled the collider and left the ball exactly where it stood, which is out of
              play to the solver and to nothing else. The CAD stages NECTAR inside the CELLs, so
              after parkOffField() three of them sat in the up CELL at Y 49.9 in with no
              collider -- still DRAWN (the renderer's only visibility rule is `p.y > -0.5`),
              still labelled `cell` by trackBallStates, and still counted by endOfMatchCounts.

              A live shot flew straight through them and came to rest behind them. From the
              outside that is exactly "the ball landed and went under the HIVE", and it is why
              chasing it in the physics found nothing: 40 balls injected at the mouth put ZERO
              under the rocker. The ghosts were never colliding with anything.

              Measured in the running app: 26 parked balls, all 26 with `meshVisible: true`.
Did instead: park() moves the ball to y = -5 m as well as disabling it. One move, because all
              three consumers key off position: it is under the renderer's cutoff, outside every
              scoring volume, and inside no cell. reset() still restores every ball to its home.
              tests/hive.test.ts asserts a parked ball is below the floor and scores nothing.
Costs/risks: Anything that read a parked ball's coordinates now reads the bench instead. Nothing
              did; `preload` picks by index and `reset` restores from `home`.
Who/where:   packages/core/src/physics/balls.ts, tests/hive.test.ts

## 2026-09-17 — The staged NECTAR were mirrored, so the two hives disagreed by eight inches
Plan said:   stage 3 NECTAR against the pocket's back wall, offset to one lip.
Found:       `t` runs ACROSS the mouth, and cell B's axis is the mirror of cell A's, so the same
              signed offset is the low lip on one rocker and the high lip on the other. The same
              instruction put red's NECTAR at Y 58.7 in and blue's at 50.4 -- eight inches apart
              on two rockers that are mirror images of each other.
Did instead: Stage centred across the mouth (t = 0) and let gravity settle them, which is also
              the only claim the manual actually makes. Both hives now settle 3 at Y 50.3 in,
              Z +-9.6, holding +-0.295 N.m.
Who/where:   packages/core/src/physics/hive.ts

## 2026-09-17 — Auto mode: the autonomous routine, and why it has to drive round
Plan said:   the UI had Practice, Data and Test. The AUTO period existed on the clock and
              nothing played it.
Found:       Worth having on its own, and it turned up the thing a new user hits first: from the
              default start pose the up CELL's mouth is 109 deg off its opening, because the
              robot starts against its own alliance wall and the CELL opens toward the audience.
              An auto that stands still and fires scores nothing however well it aims, and a
              human pressing Fire on the start tile sees a robot that looks broken.
Did instead: `AutoRoutine`: LEAVE -> position -> shoot -> park -> done, emitting GamepadState
              into the same BuiltinTeleOp a human drives, so no shot takes a privileged path.
              Nothing in it is a magic coordinate -- the shooting spot comes from where the mouth
              is and which way it opens, the park spot from the LOADING zone in
              buildFieldGeometry, the stand-off from the shot table's own band. Move the hive or
              rebuild the table and the routine follows.

              Measured, tools/autocheck.ts, four seeds, a full 30 s period each:
              4.3 fired, 3.3 landed, LEAVE 4/4, PARK 4/4.

              Two bugs found writing it, both worth recording. The routine first drove
              confidently to a spot 113 deg off the opening: it was built in WORLD (x, z) and
              steered against the localizer's FTC (x, y), which are a permutation apart -- it
              now goes through `worldToFtc`, the one place that conversion is allowed. And the
              stand-off was the middle of the table's band, 90 in, which is a spot that does not
              exist on a 141 in field and got clamped into the wall; it is a quarter into the
              band now.

              AutoDriver and AutoRoutine share one `driveTo`, so there is one piece of driving
              code to be wrong rather than two.
Costs/risks: The routine shoots from one spot. It does not re-position after a TIP -- it goes and
              parks instead, on the grounds that PARK is worth more than the shot it would give
              up with seconds left. Worth revisiting if the first tip starts landing early.
Who/where:   packages/core/src/robot/{autoRoutine,autoDriver}.ts, packages/ui/src/{main,guide}.ts,
              index.html, tools/autocheck.ts

## 2026-09-17 — A slider that cannot say its own value rewrites it
Plan said:   the Variables panel is live sliders over the constants the simulator runs on.
Found:       Two of them could not represent the config they shipped with, which is not cosmetic:
              the panel writes straight into the objects the physics uses, so a slider pinned at
              its end shows a number the simulator is not using AND rewrites the constant to the
              nearest value it can say the moment it is touched.

              `Flywheel inertia` ran 0.0005..0.01 against a config of 0.000391, so one drag moved
              the wheel's inertia by 28% -- and inertia is what sets the per-shot dip, which is
              what the last shooter decision turned on.

              `Hopper capacity` ran to 12 under a hint reading "Rules cap this; check the
              manual". A cap written in a hint is a suggestion, and the config it shipped with
              (6) was already over G407's 4.
Did instead: Ranges that contain their values, a step fine enough to land on them, and the
              hopper capped at the rule. tests/ui.test.ts checks all four properties across
              every slider: in range, on a step, round-trips, and never offers an illegal
              setting.
Who/where:   packages/ui/src/tune.ts, tests/ui.test.ts

## 2026-09-17 — Dead code, and why none of it was visible
Plan said:   tsconfig had `strict: true`.
Found:       Strict does not include the unused checks, so seventeen dead things had accumulated
              unseen: a private method nobody called (`Robot.mouthBox`), an unused interface
              (`CarriedBall`), a field written twice and read never (`Hive.lastTipT`), four
              constructor refs stored and never used, a parameter passed to `run()` in
              shoterror that the caller already prints itself, and seven unused imports.
Did instead: Removed all of it, and turned on `noUnusedLocals`, `noUnusedParameters` and
              `noFallthroughCasesInSwitch` so the next one cannot accumulate quietly. `npx tsc
              --noEmit` is the check; it is clean.
Costs/risks: The unused-parameter rule means a genuinely-unused argument now has to be named
              `_x` on purpose, which is the point.
Who/where:   tsconfig.json, packages/core/src/physics/{balls,battery,hive,robot,world}.ts,
              tools/{entrycheck,fixedspeed,hoodtable,leadcheck,shootercheck,shoterror,shotzone}.ts

## 2026-09-18 — Rebuilding the shot table around the entry model: TRIED AND REJECTED BY MEASUREMENT

Plan said:   `tools/entrycheck.ts` reported the table asking for arrival conditions worth 33%
             retention when the best cell in the grid is 92%. That gap looked like free
             points: put the measured entry rate in the objective and the table stops
             choosing shots that thread the mouth and bounce straight back out.

Found:       The objective ALREADY multiplies `entry.lookup(v, d)` in -- it has since the
             entry model existed. What was stale was the grid, not the code. So the change
             actually under test was "re-solve the table against a denser measurement".

             The grid was re-measured at 64 balls a cell (up from 24; 9m25s). That mattered:
             the two grids disagree substantially, which means the 24-ball one was
             noise-dominated and any conclusion drawn from it was too. With the denser grid
             the table re-solved from an arrival of 6.2 m/s at 75 deg down to 3.5 m/s at
             33 deg -- squarely into the high-retention corner, exactly as intended.

             And it measured WORSE end to end. Same harness, same seeds, only the table
             changed:

               case        before   after
               stopped        88%     84%
               wobbling       91%     80%
               spinning       89%     91%
               FAST wobble    79%     81%
               closing/receding/strafing  100%  100%

             No case improved beyond binomial noise and the largest single move, wobbling at
             -11 points, went the wrong way.

Did instead: Reverted `shottable.csv`. KEPT the 64-ball `entry.json`, because more samples is
             strictly better data and the measurement is worth having whatever is done with
             it. Note that the CSV's own `pStay` column therefore predates the current grid:
             it records what the shipped table's choices were worth under the 24-ball
             measurement, not under this one.

Why it probably failed, for whoever tries again: `entryRate` injects balls AT the mouth with
             a chosen speed and descent and the aim jittered by a ball radius. A real shot
             does not arrive from that distribution -- its lateral offset, spin axis and
             arrival angle are all correlated through the trajectory that produced them, and
             the grid treats them as independent axes. Optimising hard against a marginal
             distribution can easily move the answer somewhere the joint distribution does not
             reward. The fix is not a denser grid; it is measuring retention along REAL
             trajectories (fire from the shot pose, vary the table, score the outcome), which
             is what `tools/landrate.ts` already does end to end and costs far more per sample.

Costs/risks: The entry model still earns its place -- without it the objective is margin-only
             and picks 81 deg lobs. This says the model is worth having and not worth
             re-optimising against at this sample size.
Who/where:   config/entry.json (regenerated, n=64), java/teamcode/assets/shottable.csv
             (unchanged), tools/entrycheck.ts, tools/shottable.ts

## 2026-09-18 — Re-fitting the land calibration, and finding the score does not predict

Plan said:   the calibration had 6 bins from 331 shots and the top four all read 0.964, so it
             could not tell 85% from 96% anywhere the robot actually shoots. Re-run it with
             more samples and let the bins get finer.

Found:       Both changes worked and the conclusion is not the one expected.

             `nBins` was capped at 6 whatever the sample size, so a longer run only made the
             bins fatter. Capped at 12 now, about 80 samples each. And the lowest bin had been
             score 0.49 because nothing worse was ever sampled -- adding the FAST wobble
             driving case (wobble 0.7, the one where the gate now refuses a third of the
             loops) pulled the bottom of the sampled range down to 0.05.

             839 settled shots, 10 bins of 84:

               predicted    observed
               0.05-0.48         85%
               0.49-0.64         88%
               0.64-0.67         82%
               0.67-0.83         42%
               0.83-0.85         77%
               0.85-0.85         81%
               0.85-0.85         82%
               0.85-0.88         70%
               0.88-0.95         71%
               0.95-0.97         76%

             THE SCORE CARRIES NO INFORMATION. Shots the model rates 5-48% land 85% of the
             time; shots it rates 95-97% land 76%. It is not merely uncalibrated, it is
             uncorrelated, and over the top half it is slightly INVERTED.

             That explains a run of results that looked unrelated. Raising the gate refused
             30% of the loops on the FAST wobble case and did not improve its hit rate,
             because the gate is filtering on noise. The shot-zone map's colours are that same
             score, so the green is not telling a driver where the shot is better. And the
             monotone forcing -- which exists so a noisy bin cannot invert the curve -- turns
             "no signal" into a flat line at 0.88 that LOOKS like a calibration.

             The 0.67-0.83 bin at 42% over 84 shots is not noise (se 5 points) and is the
             thread worth pulling: something specific about those shots is wrong.

Did instead: Shipped the 839-shot fit, because it is the honest measurement and the previous
             one was hiding this behind six coarse bins. The practical effect is that
             `minLandProb` is a no-op again -- everything calibrates to about 0.88, so no
             threshold under that refuses anything. That is now TRUE rather than an artefact
             of the clamp fixed earlier today, and it should stay visible until the score is
             worth gating on.

Costs/risks: The gate is currently decoration. Do not raise `minLandProb` expecting it to do
             anything; it will either do nothing or stop the robot shooting entirely at 0.89.
Who/where:   tools/landcal.ts (bin cap, FAST wobble case), config/landcal.json

## 2026-09-18 — Why P(land) predicted nothing: the entry model had the range slope backwards

Plan said:   the 839-shot calibration showed the model's P(land) carrying no information --
             a 1-point gap between the bottom and top half of its own predictions. Find which
             part is dead.

Found:       `pLandRaw` is a product of three factors and only the product was ever recorded,
             so there was no way to ask which. Recording them separately answered it in one
             run. Land rate below vs above each factor's own median, 839 shots:

               pSpeed (threads the mouth)    72% -> 79%    +6
               pStay  (stays in once there)  84% -> 73%   -11   INVERTED
               pAim   (lateral)              83% -> 73%   -10
               pLand  (the product)          75% -> 76%    +1

             And the cause, per range:

               range   landed   model pSpeed  pStay  pAim  product
                40 in     63%            87%    96%  100%      84%
                55 in     81%            98%    87%  100%      85%
                70 in     84%            66%    75%  100%      48%

             CLOSE SHOTS LAND WORST AND THE MODEL RATED THEM BEST. pStay falls with range in
             the model and rises with range in reality, so it was not merely miscalibrated,
             it had the sign of the slope wrong -- and since pAim also rises as range falls,
             both read as inverted for the same single reason.

             That is the same failure as the shot-table rebuild rejected earlier today, and
             for the same reason: `pStay` comes from `tools/entrycheck.ts`, which injects
             balls AT the mouth on independent speed and descent axes. A real shot's arrival
             angle, speed, lateral offset and spin are correlated through the trajectory that
             produced them. The marginal distribution does not transfer.

Did instead: Replaced the shot table's `pStay` column with retention MEASURED from the 839
             real shots -- 0.725 at 40 in, 0.829 at 55, 1.0 at 70 -- rather than modelled from
             injected balls. Same harness, same shots, only the prediction changed:

               pStay   -11  ->  +17     (now the strongest single factor)
               pLand    +1  ->  +13     (was carrying no information)

             and the model now tracks reality per range: 63% actual against 64% predicted at
             40 in, 81% against 78% at 55.

Costs/risks: ONLY 40-70 in IS MEASURED. Below 40 the endpoint is held; above 70 it is held at
             1.0, which is certainly optimistic -- a 150 in shot does not retain perfectly.
             Extending `RANGES` in tools/landcal.ts past 70 in needs the harness fixed first
             (the longer passes "failed to place on the field or ran out of firing window").
             Until then the score is honest in the middle and optimistic at the long end.

             END-TO-END HIT RATES DID NOT MOVE: 88/100/100/100/91/100/79/89 before and after.
             That is consistent rather than disappointing -- movingtune fires from a FIXED
             range, and almost all the recovered signal is BETWEEN ranges. Within one range
             the only remaining predictor is pSpeed at +6. Where this pays is anything that
             CHOOSES a range: the autonomous stand-off, the opponent's, and the shot map.

             `pAim` is 100% at all three measured ranges and contributes nothing. Left in
             because it is the term that would matter past 100 in, where nothing is measured.
Who/where:   tools/landcal.ts (per-factor and per-range breakdown, raw sample dump),
             java/teamcode/assets/shottable.csv (pStay column), config/landcal.json,
             config/landcal-samples.json, config/shotzone.json,
             packages/core/src/robot/builtinTeleOp.ts (the three factors on the state)

## 2026-09-18 — Why the ball arrived at the wrong height on the move: four faults, none of them the flight

Plan said:   "Do a deep analysis of the entire physics and projectile motion ... it's struggling
             to hit the right height while moving." The suspects, in the order everyone reaches
             for them: the drag and Magnus constants, the integrator, the lead's vertical.

Found:       `tools/flightcheck.ts` (new) fires with scatter and ball variance OFF and splits
             every flight into four parts -- launch geometry, integrator, execution, aim --
             each measured against the next from the same initial state.

             THE FLIGHT IS FINE. Rapier at 1/240 s with its own angular damping and
             `simulateShot` at 1/480 s agree to 0.4-0.7 cm at the mouth plane from one initial
             state, standing still at 36, 50 and 70 in and moving. The muzzle is 0.4 cm above
             the table's assumption. The mouth lips are at the manual's 53.5 / 65.6 in, so
             PHYSICS_AND_SIMULATION.md section 9.3 was already fixed. Receding and strafing
             were within 2 cm. Nothing in aero.ts or ballistics.ts moved.

             CLOSING WAS 6.6 CM LOW, with the wheel 3 rpm off, the hood 0.03 deg off and the
             velocity estimate 0.005 m/s off. `rangeLead_s` (0.15 s, measured against the
             WHEEL's lag) looked the whole row up at the predicted release range and handed
             the hood and the lead that row too. The hood is a servo and the lead is
             arithmetic; both are re-solved every loop up to the frame the ball leaves, so a
             row for 0.15 s ahead is a row for a shot 0.15 * v_radial too close.

             THE TABLE ZIGZAGGED BETWEEN BRANCHES from 74 to 102 in: 60 deg / 2570 rpm at 74
             next to 70.7 deg / 2901 at 78, then 60 at 94, 66.7 at 98, 50.7 at 102. The robot
             interpolates between rows, so crossing 74-78 in it was handed 65 deg at 2735 --
             halfway between two solutions. `preferHoodPos`'s continuity weight was 0.04 per
             unit of hood against a score of order 1, i.e. nothing.

             THE BRAIN FIRED FROM INSIDE THE TABLE. `ShotTable.lookup` clamps to the nearest
             row and says nothing; a robot 6 in from the hive fired the 30 in solution. The
             Java deliverable has always refused this through `usable(range)`; the mirror
             never did.

             REGENERATING THE TABLE THREW AWAY A CALIBRATION. The `pStay` column had been
             hand-edited to the measured retention (2026-09-18, above). `tools/shottable.ts`
             wrote the entry model's 0.58-0.65 back over it, and nothing said so.

             AND TWO METRICS WERE LYING. `Robot.lastTargetRpm` is duty times free speed --
             meaningless for the open-loop feedforward the brain sends -- so `tools/shoterror.ts`
             reported +64 rpm on a stationary wheel that was 6 rpm off; the "+66 to +103 rpm at
             release" carried in these notes as an unchased systematic error was that number.
             And the seven "wild" turning shots that went two metres long with every release
             term on target were all fired into a pocket already holding eight or more balls
             (flightcheck --case turning --scatter --gate: the first eight score, the ninth
             diverges from the solver before the mouth). shoterror's harness never emptied the
             pocket; a real rocker tips at four to six.

Did instead: The row is looked up at the CURRENT range and only the wheel's rpm target carries
             the 0.15 s look-ahead (as the rpm the table will want by the time the wheel is
             there). `bestShot` takes `maxHoodJumpDeg` and `buildTable` passes 5, anchored on
             the first range; the table is now monotone, hood 73 -> 44 deg and 2210 -> 3343
             rpm over 30-150 in, apex 60-70 in, descent 25-36 deg -- the shape of the design
             table in PHYSICS_AND_SIMULATION.md section 2.4. The brain holds with "outside the
             table - BACK OFF" past either end. `buildTable` reads the retention per range from
             `config/landcal-samples.json` (landed / threading factors) so a regenerated table
             keeps the measurement. shoterror measures rpm against the brain's target and ends
             a pass when the pocket holds six.

             Measured, scatter off, at the mouth plane: closing 0.23 m/s from 42 in +0.5 cm,
             from 33 in +1.8; closing 0.5 m/s from 52 in -3.6 (the same row shows -1.5
             standing still from that 56 deg off-axis spot); turning +-1 cm on eleven of
             eleven. Gate and scatter on (tools/movingfire.ts): stopped 85%, closing 95%,
             strafing 80%, shuttle 90%, wobble 80%, long error 2 / 1 / -2 / 4 cm. Land rate by
             range from a fresh tools/landcal.ts run: 81% at 40 in, 86 at 50, 90 at 60, 87 at
             70, 88 at 80 -- against 62 / 78 / 89 / 84 / 78 before. Ceiling 92% (was 89).

             tests/ballistics.test.ts (new): the vacuum parabola, the drag and lift laws, the
             world against the solver from one state, table continuity, the out-of-table
             refusal. Section 11 of the physics document asked for these; there were none.

Costs/risks: 30 in with the 2 in range trim is now "outside the table": the effective minimum
             is 32 in, which the shot map already painted as too close. The wheel-only range
             lead measured 3.6 cm low against 1.0 cm high with no lead at all on the one
             scatter-off case; both are inside a 30 cm mouth and the gated harness could not
             tell them apart. Re-sweep it with tools/movingtune.ts --rangelead if the flywheel
             changes. The calibration curve is nearly flat (0.78-0.89), so the calibrated map
             reads 0.86 almost everywhere a shot exists; the map's shape test now reads the raw
             score. The wobble case's 26 +- 55 cm lateral in movingfire WAS the pile: the
             rebuilt shoterror, ending each pass when the pocket holds six, reads closing +
             wobble at 62 in as long 6 +- 9, lateral -6 +- 10, nothing wild (14 shots); stopped
             3 +- 10 / -1 +- 5, shuttling 7 +- 9 / -2 +- 8, closing 76 in 3 +- 10 / -2 +- 5,
             none wild; turning 3 wild of 48 (was 7 of 62), median 4 cm. movingfire still fills
             the pocket and its wobble column should be read with that in mind.
Who/where:   tools/flightcheck.ts (new), tests/ballistics.test.ts (new),
             packages/core/src/robot/builtinTeleOp.ts (row at now, wheel-only lead, inTable,
             rowHoodDeg/rowRpm), packages/core/src/physics/ballistics.ts (maxHoodJumpDeg),
             tools/shottable.ts (branch, loadMeasuredStay), java/teamcode/assets/shottable.csv,
             config/shotzone.json, config/robot.json (rangeLead_s_source), tools/shoterror.ts,
             tools/shotzone.ts (raw), tests/landprob.test.ts

## 2026-09-18 — Shooting at speed: the band measured, the cap re-measured, the estimator carrying the motion

Plan said:   "Move at high speed, aim perfectly and shoot every time in the green zone; 90%
             accuracy and good fire timings." The patrol harness (tools/zonerun.ts, new)
             drives the shooting sector at a held speed with the latch on and the real
             rocker, which is that question asked the way a driver asks it.

Found:       THE BAND WAS NARROWER THAN THE POCKET, half of it resolution. tools/bandcheck.ts
             (new) scales the world's wheel against the brain's and lands four balls a step:
             the pocket accepts -6..+5% of exit speed at 40 in, -6..+5 at 55, -5..+5 at 70.
             The solver said +-3.4%. speedBand searched in 0.125 m/s steps, +-2.3% at 5.4
             m/s -- coarser than the band it resolved -- and at 0.025 m/s the geometric
             aperture alone solves +-4.6% at 56 in. The rest is the lips deflecting a
             grazing ball inward: hive.lipClearanceFrac = 0.5 (measured) solves +-5.6 / 5.1 /
             4.4, inside the measurement everywhere.

             THE RANGE LEAD IS ZERO. Re-swept on the wheel target alone: 0 s 91%, 0.15 s
             86%, 0.30 s 87%; the wheel is 2 rpm off at release and slews eight times faster
             than the target moves.

             THE LEAD CAP WAS 20 DEG FROM THE OLD LEAD. tools/movingtune.ts --lead cannot
             re-measure it -- none of its cases reach 12 deg of lead, so it read the same 106
             shots at every cap. zonerun --cap, at 0.80 m/s: 20 lands 88% of 8 shots at 0.06
             balls/s with 89% of loops refused; 45 lands 94% of 16; no cap is identical to
             45 because the outrun and hood-travel gates take over past it. Set to 45.

             THE AIM FILTER LAGGED BOTH KNOWN RATES, and the gate measured noise. The filter
             on the turret command is for localizer noise; a chassis yawing at w drags the
             bearing at -w, and one crossing the mouth at 0.84 m/s from 38 in turns it at
             50 deg/s. Both were being filtered as if they were noise (66 deg/s of spin was
             refused 80% of the time for "turret 2 deg off"). And the pointing error was the
             raw solution minus the axis: at a 27 deg lead, 0.04 m/s of velocity noise is
             1.1 deg, and on a steady 1 m/s leg the 3 deg gate tripped on noise 5-10% of the
             loops while the axis was within a degree of where it had been sent. At each
             reversal the lead swings 55 deg in 0.3 s and the axis is 11 deg behind for
             real; that hold is honest.

             A TIP CREDITED NOTHING. Hive.tips increments at 90% of the far stop; the balls
             left the CELL's volume a quarter of a second before. 28 shots, 0 credited, on a
             stationary robot that had put nine in and tipped the hive.

Did instead: Both rates fed forward through the aim accumulator; the pointing error measured
             against an estimate that carries them and takes out only the per-loop noise,
             with a faster pole than the command so a real lag still shows. Cap 45. Range
             lead 0. Clearance 0.5, band search at 0.025 m/s. A tip credits the most the
             cell held since the last one. Cycle time re-swept and kept at 1.0 s (90%);
             0.8 s is the throughput option at 88%.

             MEASURED. Spinning at 66 deg/s: 94% in, nothing wild (was refused). Patrol,
             60 s x 2 seeds, real rocker, land% and balls credited per second:

               0.00 m/s   79%   0.73/s
               0.62 m/s   88%   0.64/s     (was 87%, 0.23 with the cap at 20)
               0.79 m/s   88%   0.64/s     (was 88%, 0.06)
               0.83 m/s   88%   0.56/s     (was 88%, 0.06)
               0.84 m/s   79%   0.51/s     (was 88%, 0.06)

             Calibration against the shipped table: 85 / 92 / 90 / 85 / 94% at 40-80 in,
             model 92 / 94 / 92 / 87 / 85 -- within a few points everywhere. Ceiling 94%.

Costs/risks: 1.5 m/s is not this drivetrain: four 5203-312s on 48 mm wheels free-run at 1.57
             m/s and load to about 1.2, and the patrol tops out at 0.84 with the range held.
             The design shot has 2.1 m/s of horizontal speed, so closing at 1.2 m/s asks the
             hood for 83 deg against its 80. What holds shots at speed now is the reversal
             itself, 14-23% of loops, which is the axis's 261 deg/s. The stationary patrol
             reads 79% because it fires into a pocket that tips at twelve; the last few of
             those meet the pile. 100% is not on offer from a shooter with 1.5% of speed
             scatter into a mouth 3 sigma deep, and nothing here pretends otherwise.

             THE TURRET'S TRAVEL IS NOT THE WRONG-HIVE STORY ANY MORE. A continuous turret
             (+-3600, a slip ring with no stop) against the roaming driver reads 2.1% of
             loops more than 45 deg off our CELL -- the same as +-270 -- so the residual is
             not the unwind. It is the chassis: a button-slammed turn is 273 deg/s against a
             261 deg/s axis, and a 500 deg/s axis brings it to 1.5%, 900 to 1.0%. That is a
             servo choice, not a control one; the config stays at +-270.

             AND THE PICTURE WAS LYING. Measured in the app itself through its real input
             path (a read-only window.__sim handle, the frame loop stepped by hand while the
             tab is hidden): 13.7 s of full-stick driving, the muzzle never more than 45 deg
             off our CELL, and the four worst frames 38-40 deg off with the chassis not
             yawing and the aim error under 3 deg. That is the lead doing its job at 1.2 m/s.
             But the on-screen predicted arc integrated the exit velocity alone -- exact at
             rest, wrong by the whole lead on the move -- so it drew the ball landing 30-40
             deg upstream of the hive, by the opponent's, while the real ball went in. "It
             aims at the wrong hive most of the time" was the curve, not the aim. It now
             adds v_cg + omega x r, the same term Robot.launch() has.

             FULL STICK, MEASURED. Sixteen single crossings of a +-50 deg sector at 38 in
             with no range hold (tools/zonerun.ts --pass --nohold), 0.95 m/s actual, latch
             on, real rocker: 103 shots, 96 credited, 93%, 1.00 balls a second, long
             -0 +- 11 cm, lateral -2 +- 9. The patrol could not reach that speed -- a
             mecanum takes half a second to reverse and oscillates at the sector's edge --
             which is why the earlier table tops out at 0.84.
Who/where:   tools/zonerun.ts, tools/bandcheck.ts (new), packages/core/src/robot/builtinTeleOp.ts
             (feed-forwards, aimEst), packages/core/src/physics/ballistics.ts (steps),
             packages/core/src/physics/world.ts (tip credit), config/robot.json
             (fireLeadCap_deg, rangeLead_s, cycleTime_s notes), config/params.json
             (lipClearanceFrac), tools/shottable.ts / hoodtable.ts / shotzone.ts (clearance)

## 2026-09-19 — The feed was racing itself: a third of the pulses fired nothing, and the delay was a coin flip

Plan said:   "Move at high speed, aim perfectly, shoot every time." The complaint from the driver
             was that sometimes the timing was wrong, sometimes the accuracy, sometimes the
             movement, and that the readiness check ought to account for the time a shot takes
             to leave.

Found:       THE AIM WAS NOT THE PROBLEM. tools/shoterror.ts, every driving case: long +-8-10 cm,
             lateral +-3-5 cm at 50 in, no term correlating with the miss. tools/releasecheck.ts
             (new) records every shot at COMMIT and at RELEASE; with scatter off the ball crossed
             the mouth within +-6 cm long and +-7 wide on the 40 in patrol at 1 m/s. The exit
             speed and elevation the ball got matched the brain's solution to 0.02 m/s and 0.2
             deg. So where did a third of them go?

             THE FEED. The commit-to-release delay was 0.13 s OR 0.38 s and nothing between,
             and 13 of 41 pulses at 1 m/s -- 107 of 118 standing still -- released NOTHING,
             each a wasted 0.6 s cycle. tools/feedprobe.ts, the tube frame by frame:

               - Robot.stepNip refused to fire until `sinceFeed`, reset at the previous RELEASE,
                 reached cycleTime_s. The brain runs the same 0.6 s from the previous COMMIT.
                 Same period, different phase, and the brain's pulse arrived 0.13 s before the
                 world would allow the shot.
               - The nip also required the servo to still be past half travel when the ball
                 arrived. A ball staged above the plate arrived 0.13 s after the commit and
                 went. One waiting under the plate needed the plate to clear (0.19 s) and 80 mm
                 of climb (0.2 s), arrived at 0.38-0.40 s, and met a servo back through 0.5 at
                 0.375 s. Frame quantisation decided which.
               - The plate re-enabled on a clock, 0.31 s after the pulse, whichever ball was
                 straddling it; the solver threw that ball up into the wheel or back under the
                 gate, and the next pulse inherited whichever it was.

             THE DELIVERABLE HAD TWO FEED BUGS OF ITS OWN, which is the README's "the Java
             decides to fire and the ball never leaves". Transfer.java ran the belt only for
             the 0.25 s pulse -- a ball has about 0.7 s of tube to climb -- and the OpModes ran
             the intake only on a trigger, so the stop at the end of LEAVE threw all four
             preloads out of the mouth while the dead-reckoned hopper went on saying 4
             (tools/headless.ts now prints the world's own hopper count and true range).

             THE MIRROR CONVERTED THE BEARING WITH THE WRONG HEADING. The lead's field-frame
             conversion used `s.imu.yaw`, 40 ms stale and uncorrected, while the bearing it was
             converting is relative to the fused pose's heading; the Java has always used the
             localizer's heading throughout. Nearly 3 deg of frame mismatch at the 70 deg/s
             yaw cap.

             WHAT ACTUALLY LIMITS ACCURACY ON THE MOVE IS THE POCKET FILLING. The per-shot
             dump is unambiguous: in every seed the first ten or eleven balls land and the last
             two or three before the tip miss, at any angle. e_ball (ball-ball restitution,
             a GUESS at 0.8) moves this: 66% at 0.8, 73% at 0.5, 79% at 0.3 on the same seeds
             with scatter off. That is the number to measure on a real ball.

             AND THERE IS NO FLATTER SHOT TO LEAD WITH. tools/flatbranch.ts: the shipped table
             already returns the flattest arc that threads at every range (a floor of 2.0 m/s
             horizontal returns the identical rows). The ball's own horizontal at 40-50 in is
             1.8-2.1 m/s, so sideways motion past about 1.2 m/s cannot be led under the 45 deg
             cap, and past 1 m/s the lead puts the tag outside the turret camera's 30 deg
             half-lens. Fast shooting from 40 in has to be radial, or from 60 in out.

Did instead: One clock. The nip is a latch: armed when the servo opens from fully shut, spent by
             the launch, expired once it is fully shut again. A ball the pulse admitted fires
             when it reaches the wheel whatever the servo is doing by then; a ball arriving
             after the gate has shut waits; a half-shut-and-back flicker from the release
             re-check is not a second pulse. The plate cannot close through a ball. The belt
             holds the column single file with a soft spring toward the bore axis, because a
             plate that waits let four balls stack corner to corner in the square bore and
             arch: the top one 11 mm short of the wheel, the belt driving, thirty seconds of
             pulses feeding nothing (releasecheck --wasted).

             Transfer.setBeltOn: the belt runs whenever the flywheel has a target. The OpModes
             run the intake throughout, TeleOpMain reversing it on the left trigger as the
             mirror does. The mirror uses the fused heading for the lead.

             MEASURED, tools/fastfire.ts --untiltip, 40 in, 3 seeds, 40 s:

               speed      land%   s per ball IN     (before, STATUS.md)
               standing   100%    1.53 s            95%   1.41 s (zoneaudit: now 96%, 1.09 s)
               0.39 m/s    75%    0.80 s            74%   1.01 s
               0.60 m/s    69%    0.86 s
               0.78 m/s    71%    0.97 s            82%   1.15 s
               0.99 m/s    68%    1.08 s            75%   1.34 s

             Every row scores more per second. The land rate on the move is down because the
             robot now really fires every 0.6 s, and the decision above measured what a ball
             arriving while the last one is still settling does. See the cycle-time row below
             for the trade.

Costs/risks: The last balls before a tip will keep missing until e_ball is measured. The Java
             AutoOneTip now keeps its preloads and spins up, but parks 62 deg off the opening
             where the turret camera cannot decode the tag, and holds fire; autoRoutine.ts
             drives round for this reason and the OpMode does not yet. The centring spring is
             a constraint imposed rather than a belt face modelled.
Who/where:   packages/core/src/physics/robot.ts (stepNip latch, gateBlocked, belt centring),
             packages/core/src/robot/builtinTeleOp.ts (heading), java/teamcode Transfer.java,
             Robot.java, AutoOneTip.java, AutoLeavePark.java, TeleOpMain.java,
             tools/releasecheck.ts (new), tools/feedprobe.ts (new), tools/headless.ts

## 2026-09-19 — Is this the mathematical maximum? Yes for the aim, at every speed; the pile is the rest

Plan said:   "Find the mathematical maximum, test it, and make the robot do the best thing at
             every speed automatically."

Found:       tools/ceiling.ts gives the per-shot ceiling from launch scatter and the entry
             model: 86% at 30 in, 88% at 42, 95% at 54, 99% at 78-90. To test the ROBOT
             against it the pocket has to stay empty, so tools/releasecheck.ts --emptycell
             benches every ball as it settles. Moving, three seeds, 170-200 shots a cell:
             40 in 96 / 93 / 95% at 0.4 / 0.7 / 1.0 m/s; 52 in 96 / 93 / 96%. Standing with
             scatter off: 100% and 92%. The aim is at the ceiling at every speed, and above
             the model's own 40 in number, so there is no per-speed policy to write.

             The group was 5-7 cm long with scatter off at both ranges. rangeTrim_in 2 -> 4
             centres it to within a centimetre (100% at 40 and 52 in, scatter off).

             Standing still at 40 in with the pocket filling: 67% from 5 deg off, 71% from
             25, 75% from 45. The bearing is a small effect; the pile is the loss, standing or
             moving. Range is the lever the table already carries: stay rate 86% at 30-42 in,
             95% at 54, 99% at 78, and tools/landrate.ts lands 12/12 at 55 and 70 in at the
             0.61 s floor. ShotTable.bestBand()/bestRange() ranked rows by speed MARGIN, which
             put the robot's own ranging at 30-38 in -- the rows where the fewest balls stay.

Did instead: bestBand ranks by the per-shot ceiling (pThread x pStay, the readiness gate's
             own product) and takes every row within five points of the best: 58-142 in on
             the shipped table. The Java bestRange aims at the near edge plus the ranging
             tolerance, 64 in, because farther buys nothing and costs field. tests/shoot.test.ts
             pins the near edge at 54 or more.

Costs/risks: The per-shot ceiling with a filling pocket still hangs on ball.e_ball, a guess.
             The empty-pocket numbers are the maximum this shooter model can do; a real
             shooter with tighter scatter does better, a looser one worse.
Who/where:   tools/releasecheck.ts (--emptycell, --bearing), config/robot.json (rangeTrim_in),
             packages/core/src/robot/builtinTeleOp.ts (bestBand), java/teamcode
             control/ShotTable.java (bestRange), tests/shoot.test.ts

## 2026-09-19 — Every speed, 200 balls each: the aim is at its ceiling and the delay is the mechanism

Plan said:   "Test every speed for 200 balls."

Found:       tools/releasecheck.ts --minshots 200 keeps adding seeds until the sample is the
             same size at every speed, so no row is a tighter number than its neighbour, and
             --emptycell benches settled balls so what is measured is the AIM and not the pile.
             40 in stand-off, 1699 balls, against a model ceiling of 88%:

               asked   actual   balls  accuracy  ball-to-ball  commit->release
               stand   0.00      264     98%       0.60 s        0.13 s
               0.25    0.14      264     95%       0.60 s        0.13 s
               0.50    0.39      258     98%       0.61 s        0.14 s
               0.75    0.60      247     96%       0.64 s        0.14 s
               1.00    0.79      229     98%       0.68 s        0.14 s
               1.25    0.95      226     92%       0.70 s        0.14 s
               1.50    1.00      211     92%       0.74 s        0.14 s

             At or above the ceiling everywhere. The first real cost of speed appears at
             0.95 m/s: the downrange group goes from +-8 to +-17 cm and accuracy gives up six
             points. ASKED IS NOT ACTUAL past 1 m/s -- the 40 in arc is too tight to accelerate
             round and a mecanum takes half a second to reverse, so 1.25 and 1.50 both measure
             1.0 m/s and the difference between their rows is the reversals, not the speed.

             THE DELAY IS THREE NUMBERS. Commit to release is 0.13-0.14 s, the gate servo
             reaching half travel plus the belt lifting the staged ball. Ball to ball with the
             gate open is 0.60 s EXACTLY -- transfer.cycleTime_s -- and standing 30 deg off the
             opening 260 of 260 gaps were one cycle. In the patrol the median stays 0.60 s at
             every speed while the MEAN climbs to 0.74: a refused cycle costs a whole 0.6 s,
             so the histogram is one cycle or two and almost nothing between (at 1.0 m/s, 195
             gaps of one against 30 of two). The reversal at the end of each pass is what
             refuses them, with the turret slewing and the yaw cap over.

             AND THE SECTOR EDGE IS NOT A FAIR STANDING SPOT. At speed 0 the patrol parks at
             the arc's end, 59 deg off the opening against a 60 deg cap, where the release
             re-check keeps withdrawing permission mid-pulse: 66 wasted pulses and a 1.39 s
             mean gap. That is the cap working, not a fault, but it measures the cap rather
             than the shooter -- hence --bearing.

Did instead: Nothing to change in the robot: there is no per-speed policy to write when the
             aim is already at its ceiling at every speed. --minshots, --bearing and the
             ball-to-ball histogram are recorded so the claim can be re-run.
Who/where:   tools/releasecheck.ts, docs/STATUS.md, README.md

## 2026-09-20 — Every gate is a flat line: the score does not depend on any threshold in this robot

Plan said:   "I want the gate open high AND the accuracy high." Reasonable, and the first four
             attempts to get it were all thresholds, and all of them failed the same way.

Found:       tools/releasecheck.ts gained --perfect (every sensor, lag and ball variance off),
             --emptycell (the pocket never fills) and a per-frame tally of WHY the gate is shut.
             With a perfect robot at 1.00 m/s on the 40 in arc it is open 76%, and the refusals
             are 12% "turning too fast" and 9% "turret off target", both at the reversal.

             NOTHING BOUGHT ANYTHING. Each of these opens the gate and none changes seconds per
             ball landed, which stays 0.75-0.83 s throughout:

               turret 261 -> 1400 deg/s        76 -> 77% open, 98 -> 98% landed
               + yaw cap 70 -> 400 deg/s       -> 83% open, 89% landed
               + unfiltered turret command     -> 85% open, 83% landed
               + gate on the raw solution      -> 88% open, 81% landed
               + encoder prediction            -> 97% open, 77% landed
               + aim window 3 -> 6 deg         -> 98% open, 78% landed

             Gate 76 -> 98%, accuracy 98 -> 78%, s per ball IN 0.77 -> 0.78. The gate is very
             nearly a perfect discriminator: the frames it refuses are the frames whose shots
             miss, so no threshold can buy throughput from it.

             THE 9% "TURRET OFF TARGET" IS A MEASUREMENT DELAY, NOT A MECHANISM. The sensor
             frame is built before the world steps, so the encoder the brain reads is always one
             loop old. At 60 Hz that is 16.7 ms, and a reversal moves the required bearing at
             150-200 deg/s -- 2.5 to 3.3 deg against a 3 deg window. Carrying the reading
             forward by its own reported rate takes that refusal from 9.4% to 0.9% and the gate
             from 88% to 97%. It is why 5.4x of turret slew changed nothing: the turret was
             never what the gate was measuring.

             The gate was ALSO charging the turret for a filter mismatch: the command is a
             one-pole at 0.35 with a deadband, the gate compared the axis against a DIFFERENT
             one-pole at 0.6, so even an infinite turret sitting exactly on its command carries
             the difference, and it grows with bearing rate.

             WHERE THE MISSES ACTUALLY ARE, with every gate lifted (260 shots):

               0-20 deg off the opening   94%
               20-35                     100%
               35-50                      89%
               50-65                       8%   <- 48 shots, 4 landed

             Hits against misses, the three that separate: off-opening 26 vs 48 deg, lateral
             speed at release 1.14 vs 0.41 m/s, and the CHANGE in lateral speed during the
             0.13 s the ball is in the tube, 0.09 vs 0.40 m/s. The extreme bearing IS the end of
             the pass, so the aperture is half shut (cos 55 = 0.57) at the same moment the lead
             is solving for a velocity the ball will not have.

Did instead: Nothing to the shipping gate, because there is nothing there to win. The lever is
             the ROUTE. Stock robot, real noise, pocket filling, gate untouched at 60 deg,
             varying only how far down the sector the patrol drives:

               patrol +-60 deg (current)   66% landed   1.19 s per ball IN
               patrol +-45 deg             73% landed   0.97 s per ball IN
               patrol +-35 deg             67% landed   1.27 s per ball IN

             +-45 deg is worth 7 points of accuracy and 18% more balls scored per second, with
             no code and no config change. +-35 is worse because a shorter pass is proportionally
             more reversing and the achieved speed falls to 0.91 m/s.

Costs/risks: 80-90 shots a row on the route table, so 0.97 against 1.19 is real and the exact
             optimum is not. About 300 balls a row would pin it. The four new config fields all
             default to the existing behaviour and none is on: transfer.gateSpeed (was a
             hardcoded 4, and it IS the 0.13 s commit-to-release window), turret.fireAimTolDeg
             (was a hardcoded 3), turret.gateOnRawAim and turret.predictEncoder (both
             diagnostic, and both measured to be worth nothing at the bottom line on the real
             robot: encoder prediction halves the false refusal, 10.9% to 6.9%, and scores
             1.19 s per ball either way).
Who/where:   tools/releasecheck.ts (--perfect, --emptycell, --straight, --nosealong, --reach,
             --pathcap, --opencap, --why and the refusal tally), packages/core/src/types.ts,
             packages/core/src/robot/builtinTeleOp.ts, packages/core/src/physics/robot.ts

## 2026-09-22 — One flywheel motor, because the heavy wheel already bought what the second one sold
Plan said:   Two motors on the flywheel (2026-09-17 above), and a port freed by putting the
             turret on a servo to pay for it.
Found:       The case for the second motor was measured against the BARE grip wheel, before the
             wheel became a 339 g stack. Re-measured at the shipped 3.91e-4 kg.m^2 with
             tools/spinup.ts, to inside the 60 rpm readiness window: spin-up 1.52 s on one motor
             against 0.98 s on two; recovery from a POLLEN 0.17 s against 0.12 s. Both hold the
             speed -- one motor settles at 2259 rpm on a 2315 command, two at 2251 -- so the
             sustained speed was never what the second motor was for. And the DIP is identical on
             one motor and two at every inertia, because it is lossFactor*KE/(I*omega) and the
             motor count is not in it: 199 rpm on the bare wheel, 64 at the shipped inertia,
             29 at 9e-4. Inertia fixes the dip; motors only fix the climb back.
Did instead: SHIPPED motorCount 1 and dropped flywheelB from the hardware map, which puts the
             robot on seven motors with a port free.
Costs/risks: It costs SHOTS, not accuracy. tools/movingtune.ts over the same eleven cases: 102
             moving shots at 81% in on two motors, 84 at 80% on one -- 18% fewer, same quality.
             They come out of the receding cases, exactly where 2026-09-17 said they would:
             "receding 0.25" is clear to fire on 100% of loops with two motors and 52% with one,
             27% of them settling. If the freed port is not spent on something worth more than
             18% of the firing rate, put the motor back.
Who/where:   tools/spinup.ts (new), tools/movingtune.ts; config/robot.json flywheel.motorCount
             and hardware, tests/rules.test.ts counts the ports
