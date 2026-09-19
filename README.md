# BIOBUZZ Simulator

A 3D physics simulator of the FTC 2026–27 BIOBUZZ field, with a robot whose control code is
real FTC Java — written against the Control Hub SDK so it ports by copying files.

**`docs/PHYSICS.md` is the one to read first**: what is modelled, which file implements it,
and what is still approximate. `docs/DECISIONS.md` is everywhere reality disagreed with the
plan (which is a lot, and the useful part). `docs/VARIABLES.md` is every tunable, generated
from `config/` so it cannot drift. The app's **Guide** tab documents every control.

---

## Run it

```bash
npm install
python tools/cad2assets.py    # once: turns the 35 MB STEP into assets/field.glb
npm run dev
```

Without that second step the app still runs — it falls back to procedural stand-in geometry
and says so in the console.

Open http://localhost:5180 (`PORT=... npm run dev` to move it). Drive with **WASD** — robot-centric, so W is whichever way the INTAKE points — turn with **Q/E**, `F` spins the flywheel,
`Space` fires, `H` hand-drops a POLLEN into your up CELL, `L` auto-loads the hopper, `R`
resets. Cameras on `1`–`5` (orbit / follow / top / first-person / muzzle). Full key list is in
the app's **Keys** tab. A gamepad works too.

**Does the HIVE actually rotate from the balls?** Yes — nothing scripts it. The Hive tab has a
meter showing how much of gravity's restoring torque the balls have overcome; at 100 % it goes
over. Measured onset: **12 POLLEN or 8 NECTAR**.

The five panels:

| Tab | What it shows |
|---|---|
| **Drive** | pose in FTC coordinates, range to the CELL, turret, hood, flywheel, hopper, cycle timer, battery |
| **Hive** | a meter for how close the rocker is to going over, its angle and rate, and **the per-ball torque breakdown with each ball's lever arm** — the thing this project exists for |
| **Robot** | mass, cycle time, last shot's exit speed and elevation |
| **Tune** | live sliders over the parameters that are still guesses (rocker mass, CG, pivot friction, damper, Cd, Magnus, restitution, tile friction, battery R) |
| **Keys** | controls |

## Run a real FTC OpMode against it

The Java is the deliverable. It runs on a laptop JVM against fake hardware, talking to the
world over a WebSocket.

```bash
bash java/build.sh
```

That compiles the SDK shim and `java/teamcode` with `--release 8` against **the shim only**,
then the sim-side code, then runs a self-check. If it builds there it builds on a Control Hub.

Three terminals:

```bash
npm run relay
```
```bash
npm run tool -- tools/headless.ts --match --lockstep --preload 6
```
```bash
java -cp "java/out;java/out-teamcode" sim.runner.Main --opmode "Auto One Tip"
```

(`--list` shows the OpModes, exactly as a Driver Station would.) To watch it instead of
reading numbers, skip the headless world, open the app and switch **Brain** to *Java*.

Current results, in lockstep:

| OpMode | Result |
|---|---|
| `Auto Leave + Park` | LEAVE + PARK, **8 pts** |
| `Auto One Tip` | LEAVE + PARK, **8 pts** — see below |

> `Auto One Tip` **does not currently score its shots**. Two of the reasons were in the
> deliverable and are fixed: the Java ran its feed belt only for the 0.25 s pulse, so no ball
> could climb the tube (`Transfer.setBeltOn`), and it ran the intake only on a trigger, so the
> stop at the end of LEAVE threw all four preloads out of the mouth while the dead-reckoned
> hopper count went on saying 4 (`tools/headless.ts` now prints the world's own hopper count
> and true range so this cannot hide again). What remains is the routine: from where it parks
> the CELL is 62° off its opening and the turret camera cannot decode the tag, so it holds
> fire with `tag fix Infinity ms old`. The built-in `autoRoutine.ts` drives round for exactly
> this reason; the Java OpMode does not yet.

## The tools

```bash
npm run tool -- tools/hivedrop.ts          # how many balls tip the HIVE
npm run tool -- tools/landrate.ts          # land rate vs range
npm run tool -- tools/leadcheck.ts         # does the motion lead land the shot? (no scatter)
npm run tool -- tools/movingfire.ts        # shooting while moving, and while accelerating
npm run tool -- tools/shoterror.ts         # where a moving shot's error actually comes from
npm run tool -- tools/spincheck.ts --full  # single-wheel backspin vs a dual-wheel shooter
npm run tool -- tools/shottable.ts         # regenerate the shot table
npm run tool -- tools/tagoffsets.ts        # regenerate the tag -> CELL mouth correction
npm run tool -- tools/tagmap.ts            # where on the floor the tag can be READ at all
npm run tool -- tools/hoodsweep.ts         # which hood range this robot needs
npm run tool -- tools/shootercheck.ts      # turret coverage, flywheel MOI, exit speed
npm run tool -- tools/flywheeltune.ts      # hub velocity-loop settling and ripple
python tools/cad2assets.py                 # tessellate the STEP -> assets/field.glb (~40 s)
node tools/cad2staging.mjs                 # regenerate ball staging from the CAD
node tools/genconstants.mjs                # robot.json + shot table -> Java constants
npm run tool -- tools/drivedemo.ts         # drive around and shoot from each stop
node tools/vars.mjs                        # regenerate docs/VARIABLES.md
npm test                                   # 125 tests
```

## What it currently says

**[docs/STATUS.md](docs/STATUS.md) is the current, measured state** — what it scores, what the
ceilings are, and what is wrong. It is kept honest by naming the tool behind every number, so
anything in it can be re-run and checked.

The short version, as of 19 September 2026:

| | accuracy | time per ball IN |
|---|---|---|
| standing on a green square | **96%** | **1.09 s** |
| driving at 0.39 m/s, 40 in out | 75% | **0.80 s** |
| driving at 0.78 m/s, 40 in out | 71% | 0.97 s |
| autonomous (5 runs) | 86% | 8 points, LEAVE 5/5, PARK 5/5 |

Into an **empty** pocket the robot lands **92–98% at every speed it can reach**, measured over
1699 balls with at least 200 at each speed (`tools/releasecheck.ts --emptycell --minshots 200`).
That is the ceiling this shooter's scatter allows. A ball leaves 0.13 s after the brain commits
it, and the gap between balls is 0.60 s — the mechanism's own floor — at every speed. What misses is the pile: the last balls before the HIVE tips, into a pocket
already holding eight or more, standing or moving. How hard a ball bounces off another ball
(`ball.e_ball`) is a guess that moves that by 13 points; it is a measurement, not code. The
robot's own ranging now drives to 58 in and out, where a ball that arrives stays in.

This section used to carry a page of results that had drifted out of date — a 38–50% land
rate and an 85° hood, neither of which has been true for some time. Numbers live in STATUS.md
now so there is one place to keep current instead of two that disagree.

## Layout

```
config/          params.json, robot.json, motors.json  -- every physical constant, with _source
packages/core/   the world: physics, rules, no DOM. Runs in Node for tests and sweeps.
packages/render/ Three.js. Draws the tessellated CAD; "Colliders" shows what physics sees.
packages/ui/     app shell, input, panels, bridge client
packages/bridge/ relay.mjs -- forwards between the browser world and the Java brain
java/shim/       the FTC SDK surface, re-declared (PLAN.md Appendix E)
java/teamcode/   ← THE DELIVERABLE. Java 8, SDK-only, copies onto a Control Hub
java/simsdk/     fake DcMotorEx, Servo, IMU... backed by the bridge
java/bridge/     JSON + WebSocket, JDK-only
java/runner/     OpMode registry and the hub's lifecycle
tools/           experiments and generators
tests/           125 tests: geometry, hive, drivetrain, turret, shooting, lead, tag, determinism
```

## Porting to the hub

1. Copy `java/teamcode/org/firstinspires/ftc/teamcode/**` into `TeamCode/src/main/java/...`.
2. Name the devices in the Robot Controller config as `config/robot.json → hardware`.
3. `RobotConstants.java` and `ShotTableData.java` are generated — no file parsing on the hub.
4. `Localizer`, `TagCamera` and `BallCounter` all come from `hardwareMap.tryGet(...)`, which
   returns `null` on the hub until you supply one. `TagCamera` is the one that matters: wrap
   `AprilTagProcessor.getDetections()` — id, `ftcPose.bearing`, `ftcPose.range`, `ftcPose.yaw`
   and `frameAcquisitionNanoTime` — and `TagTargetProvider` does the rest, because the fusion
   ships in TeamCode rather than living in the simulator. **There is no oracle fallback:** no
   camera means no target and the robot holds fire, which is what a robot with no vision does.
5. `./gradlew :TeamCode:assembleDebug`.
