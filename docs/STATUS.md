# Where this stands

Everything below is measured, by a named tool, at the commit it says. No number here is
estimated, and where a number is bad it is written down as bad. Re-run the tool to check any
of it — that is the point of naming them.

Last measured: 19 September 2026. 169 TypeScript tests pass, 52 Java self-check assertions
pass, the site builds.

---

## 1. What the thing is

A physics simulator for the FTC 2026-27 BIOBUZZ game, written so a real OpMode can be driven
against it. Two implementations of the same robot brain are kept in step on purpose:

| | what it is | why |
|---|---|---|
| `packages/core/src/robot/builtinTeleOp.ts` | the brain the simulator runs | fast to iterate |
| `java/teamcode/` | **the deliverable** — real FTC code | what actually goes on the robot |

They mirror each other and tests grep both. When they disagree, the Java is usually right: it
has been the one without the bug more than once.

Balls fly on the project's own integrator (drag + Magnus, 3-D, 1/480 s). Rapier handles
contact and the rest of the field. Every physical constant lives in `config/*.json` with a
`_source` note saying whether it was measured, cited, derived or guessed, and every one is a
live slider in the **Variables** tab.

---

## 2. What it scores right now

### Standing still, on a square the map calls green

`tools/zoneaudit.ts` — 24 squares, 4 balls each

| balls | in | accuracy | time per ball |
|---|---|---|---|
| 91 | 87 | **96%** | **1.09 s** |

(Was 95% at 1.41 s. The feed used to waste a third of its pulses; see section 4.)

Grey squares fired 0 balls, correctly.

### By distance

`tools/landrate.ts` — 12 balls per range

| range | in | accuracy | time per ball |
|---|---|---|---|
| 40 in | 11/11 | 100% | 0.78 s |
| 55 in | 10/12 | 83% | 0.79 s |
| 70 in | 12/12 | 100% | 0.66 s |

### By angle round the side

`tools/obliquity.ts` — 10 balls each, from 55 in

| off the opening | accuracy |
|---|---|
| 40° | 80% |
| 50° | 90% |
| 60° | 100% |
| 70°+ | never fires — the camera cannot read the tag past 65° |

### Driving and shooting

`tools/fastfire.ts --untiltip --seeds 3` — 40 in stand-off, each seed until the HIVE tips

| speed | balls | in | accuracy | time per ball IN | gate open | was (time per ball) |
|---|---|---|---|---|---|---|
| standing | 32 | 32 | 100% | 1.53 s | 47% | — |
| 0.39 m/s | 48 | 36 | 75% | **0.80 s** | 89% | 1.01 s |
| 0.60 m/s | 49 | 34 | 69% | 0.86 s | 85% | — |
| 0.78 m/s | 48 | 34 | 71% | 0.97 s | 81% | 1.15 s |
| 0.99 m/s | 53 | 36 | 68% | 1.08 s | 70% | 1.34 s |

Every row scores more per second than before. The land rate on the move is lower than the
82% this table used to show at 0.73 m/s, and the per-shot record says why (`tools/releasecheck.ts
--dump`): in every seed the first ten or eleven balls land and the last two or three before the
tip miss, whatever the angle. The pocket is full and the pile rejects the arrival. Standing
still the same pile does not — 100% — so the moving shots arrive somewhere the standing ones
do not. That is the open accuracy question, and section 4 has the one number that moves it.

### The mathematical maximum, and whether the robot reaches it

`tools/releasecheck.ts --emptycell` benches every ball once it settles in the CELL, so the
pocket never fills: the moving robot measured against the same empty-pocket ceiling
`tools/ceiling.ts` computes for a standing one. `--minshots 200` keeps adding seeds until the
sample is the same size at every speed, so no row is a tighter number than its neighbour.

**Every speed, 40 in stand-off, 1699 balls in total.** The model's ceiling here is 88%.

| asked | actual | balls | accuracy | ball to ball | commit → release |
|---|---|---|---|---|---|
| standing | 0.00 m/s | 264 | **98%** | 0.60 s | 0.13 s |
| 0.25 m/s | 0.14 m/s | 264 | **95%** | 0.60 s | 0.13 s |
| 0.50 m/s | 0.39 m/s | 258 | **98%** | 0.61 s | 0.14 s |
| 0.75 m/s | 0.60 m/s | 247 | **96%** | 0.64 s | 0.14 s |
| 1.00 m/s | 0.79 m/s | 229 | **98%** | 0.68 s | 0.14 s |
| 1.25 m/s | 0.95 m/s | 226 | **92%** | 0.70 s | 0.14 s |
| 1.50 m/s | 1.00 m/s | 211 | **92%** | 0.74 s | 0.14 s |

**The aim is at or above its ceiling at every speed the robot can reach.** Motion costs
nothing the sample can resolve until about 0.95 m/s, where the group starts to widen
(±17 cm long against ±8 at rest) and accuracy gives up six points. The robot beats the model's
own 40 in figure because the entry model's stay rate is pessimistic there.

**Asked is not actual past 1 m/s.** The patrol tops out at 1.00 m/s however hard it is pushed:
the arc at 40 in is too tight to accelerate round and a mecanum takes half a second to reverse.
Anything above that is a number about the drivetrain, not the shooter.

With scatter off the group is centred to within a centimetre at both ranges (`rangeTrim_in` is
4.0; it was 2.0 and the group sat 5–7 cm long).

### The delay between shots

Three different numbers, and only the last is the one a driver feels:

| | |
|---|---|
| commit → release | **0.13–0.14 s** — the gate servo reaching half travel plus the belt lifting the staged ball into the wheel |
| ball to ball, gate open | **0.60 s exactly** — `transfer.cycleTime_s`, the mechanism's floor. Standing at 30° off the opening, 260 of 260 gaps were one cycle |
| ball to ball, in the patrol | 0.60 s median at every speed; the **mean** climbs 0.60 → 0.74 s from standing to 1.0 m/s |

The mean climbs because a refused cycle costs a whole 0.6 s, not a fraction: the histogram is
one cycle or two cycles and almost nothing else (at 1.0 m/s, 195 gaps of one against 30 of
two). What refuses them is the reversal at the end of each pass, where the turret is slewing
and the yaw cap is over.

**Everything lost on the move is the pile.** Standing still at 40 in with the pocket filling,
firing until the tip: 67% from 5° off the opening, 71% from 25°, 75% from 45°. Same loss as
the moving rows above it, at any bearing. What does move it is range — the stay rate climbs
from 86% at 30–42 in to 95% at 54 and 99% at 78, and `tools/landrate.ts` lands 12 of 12 at
both 55 and 70 in at the 0.61 s mechanism floor — so **the robot's own ranging now drives to
the landing-ceiling band (58 in and out) instead of the widest-margin band (30–38 in)**, in
both brains (`ShotTable.bestBand` / `ShotTable.bestRange`).

The cycle time does not change the land rate (`--cycle` 0.6 / 0.8 / 1.0 at 0.60 m/s: 73 / 72 /
75% at 0.91 / 1.22 / 1.39 s per ball in), so it stays at the mechanism's 0.6 s.

**Best operating point: 40 in, 0.4–0.8 m/s.** Faster than that the reversals and the yaw cap
hold the gate shut a fifth of the time.

### Autonomous

`tools/autocheck.ts` — 5 runs

4.2 balls fired, 3.6 in (**86%**), LEAVE 5/5, PARK 5/5, 8 points.

The Java `Auto One Tip` in lockstep: LEAVE + PARK, 8 points, and now keeps its preloads and
spins up — it used to run the belt only during the feed pulse and the intake only on a trigger,
so the stop after LEAVE threw all four balls out of the mouth. It still fires nothing, because
it parks 62° off the CELL's opening where the turret camera cannot decode the tag. The routine
has to drive round, as `autoRoutine.ts` does.

---

## 3. The ceilings, so it is clear what is left

`tools/ceiling.ts`

**Accuracy is nearly maxed.** A shot can miss with perfect aim, because the launch scatter
(1.5% of exit speed, 1° of elevation) is irreducible:

| range | best possible per shot |
|---|---|
| 30 in | 86.2% |
| 42 in | 87.7% |
| 54 in | 94.7% |
| 82 in | 99.7% |

Close in the ceiling is low because the only arc that fits is a steep lob, and a steep lob
arrives nearly vertically and bounces back out. The measured ceiling across everything, on the
move, is **93.7%** (`config/landcal.json`, 708 settled shots).

At 40–46 in the ceiling is 88% and the robot gets 82% — **93% of what is available.**

**Time is where the room is.** `transfer.cycleTime_s` is 0.60 s (mechanism floor: elevator
0.26 + gate 0.08 + margin), so 1.67 balls a second is the hard limit.

| gate open | time per ball at 90% landed |
|---|---|
| 100% | 0.67 s |
| 80% | 0.83 s |
| 51% | 1.31 s |
| 30% | 2.22 s |

The best row fires 1.05 balls/s against a possible 1.67 — **59% of the machine's capacity.**
That is the headroom worth chasing, not the accuracy.

---

## 4. What is wrong, in order of how much it costs

### The probability gate — FIXED, and what it cost to find

**Was:** `minLandProb` at 0.5, 0.7, 0.8 and 0.9 fired the identical shots; 0.92 fired none.
A cliff, not a dial, because the calibrated P(land) only ever took two values.

**Why:** two separate faults, both in the sampling rather than the fitting.

1. `tools/landcal.ts` positioned the robot by walking the off-axis angle up from 0 and taking
   the FIRST spot that fitted on the field — so every one of its 708 samples came from the
   easiest geometry at that range, all landing 85–94%. A curve fitted on nothing but good
   shots cannot learn to spot a bad one. It now sweeps 0°, 20°, 35° and 50°.
2. It fires ten shots per configuration, into an **empty pocket** — so it structurally could
   never see the strongest effect there is.

**The missing term: how full the CELL already is.** `tools/whatmisses.ts`, 385 settled shots
split by the pocket's contents at the moment each left:

| balls already in | 0 | 3 | 5 | 8 |
|---|---|---|---|---|
| land rate | 95% | 88% | 85% | **60%** |

A **35-point spread** — twice the next strongest feature, and five times either of the two the
model was actually built on. A ball arriving into a part-full pocket clips the ones already
there, which `docs/DECISIONS.md` had described for a while with nothing acting on it.

`fillFactor()` is now the fourth factor in P(land), in both languages. The robot cannot see
into the pocket, so it counts its own scored balls — a running sum of each shot's own odds,
reset when the tag ID changes, because that is the tip and the tip empties the CELL.

**Result.** The fit now spans **0.84 to 1.00** instead of two values, and the score finally
separates a good shot from a bad one:

| | before | after |
|---|---|---|
| pLand, low half vs high half | 91% vs 94% (**−3 pts**) | 74% vs 89% (**+15 pts**) |
| predictions span | 0.61–0.97 | 0.12–0.85 |

And the threshold is a real dial (40 in, 0.73 m/s):

| minLandProb | accuracy | time per ball | gate open |
|---|---|---|---|
| 0.7 (shipping) | 82% | 1.15 s | 84% |
| 0.85 | 97% | 14.1 s | 11% |
| 0.90 | 100% | 16.6 s | 10% |

**The default is unchanged at 0.7, which is below the curve's 0.84 floor and so still passes
everything.** That is deliberate: the dial now works, and what it reveals is that buying
accuracy costs an order of magnitude in time, because a robot that refuses a filling pocket
just waits. The right answer to a full CELL is to tip it and move, not to stand there being
choosy — but that is a behaviour nobody has built yet, so the knob ships open and documented.

One caution: the 1.00 ceiling comes from 75–81 samples. Treat anything above 0.95 as unproven.

### The HIVE tips, and then the goal faces away

After about 12 balls the rocker goes over. The up CELL becomes the other one, which opens the
other way, so from the same spot there is no shot and the tag is edge-on at 110–129°. At
1.57 m/s over 60 s the hive tips 4 times and **61–74% of the drive is post-tip**. The robot is
right to refuse; the driver has to reposition. Any measurement that does not account for this
is measuring a robot standing on the wrong side of a turned-over goal.

### Fast driving has almost nowhere to happen

- Straight out from the red mouth **the field ends at about 55 in** (mouth at z = 16, boards at 71).
- Of the 52 green squares, **7 are at 60 in or more** — the range the hood needs to lead a
  1.7 m/s shot — and all 7 sit 49–60° off the opening.
- At 1.7 m/s the robot **cannot shoot inside 50 in at all**: the ball's own horizontal speed
  there (1.4–2.1 m/s) is no greater than the chassis, so the launch would have to go nearly
  straight up, past the hood's 80° stop. It fires fine from 60 in out.

### The feed was racing itself — FIXED

`tools/releasecheck.ts` (new) records every shot at the moment the brain commits it and the
moment the ball leaves. On the 40 in patrol at 1 m/s the delay between the two was 0.13 s or
0.38 s and nothing in between, and **13 of 41 pulses released nothing** — 107 of 118 standing
still — each one a wasted 0.6 s cycle. `tools/feedprobe.ts` (new) shows the tube frame by frame:
the world ran a cycle clock of its own from the previous *release* while the brain runs the
same 0.6 s from the previous *commit*, the nip refused a ball once the servo was back through
half travel, and the plate re-closed on a timer inside whichever ball was straddling it. The
release is now a latch (one pulse, one ball, whatever the servo does after), the plate waits
for the ball, and the belt holds the column single file because a plate that waits let four
balls arch corner to corner across the square bore and jam for 30 s. Every pulse now fires one
ball 0.12 s after the commit with a ball staged (`tests/shoot.test.ts` pins it).

The aim was never the problem: `tools/shoterror.ts` puts every driving case at ±8–10 cm long
and ±3–5 cm wide at 50 in, and with scatter off the ball crosses the mouth within ±6 cm.

### What still misses on the move: a full pocket

The last two or three balls before a tip land 40–60%. `ball.e_ball`, the ball-to-ball
restitution, is a **guess** at 0.8 and is what decides whether an arrival clips the pile and
comes back out: on the same seeds with scatter off, 66% at 0.8, 73% at 0.5, 79% at 0.3
(`releasecheck --eball`). That is the number to measure on a real ball before anything in the
aim is touched.

### Smaller, known, measured

- **There is no flatter shot to lead with.** `tools/flatbranch.ts`: the shipped table already
  returns the flattest arc that threads at every range. The ball's horizontal at 40–50 in is
  1.8–2.1 m/s, so sideways motion past about 1.2 m/s cannot be led under the 45° cap, and past
  about 1 m/s the lead puts the tag outside the turret camera's 30° half-lens. Fast shooting
  from 40 in has to be radial, or from 60 in out.
- **First shot after arming costs 5.8 s** — the belt only runs once the flywheel is on, so the
  tube primes from cold. Arm early and it is free.
- **The camera cannot read the tag past 65° of incidence**, and the tag rides the rocker.
- **Receding** — the target rpm rises with range faster than the wheel follows. The gate
  refuses, correctly.

---

## 5. Settled, do not re-litigate

Each of these was measured and the answer was "leave it alone". Re-measure only if the thing
underneath changes.

| tried | result |
|---|---|
| turret error gate 3° → 5° | 1.37 → 1.32 s/ball but 75% → 70% landed. Kept 3°. |
| off-opening cap 60° → 65° | 1.37 → 1.56 s/ball and 75% → 66%. More shots, fewer scored. Kept 60°. |
| trusting odometry 1.5 → 3 → 6 s | 53 → 66 shots, the **same 43** landed. Blind shots do not score. |
| raising the incidence limit 65 → 75° | identical; the panel is at 85°. |
| wider lens 60 → 80° | removes the lens block, incidence takes over. No gain. |
| subtracting pocket depth from the aperture | deleted 20 green squares that measure 98%. Reverted. |

---

## 6. Where to look

| for | read |
|---|---|
| every constant and where it came from | `docs/VARIABLES.md` (generated) |
| why a thing is the way it is | `docs/DECISIONS.md` |
| the ballistics | `docs/PHYSICS.md` |
| pose and vision | `docs/LOCALIZATION.md` |
| driver controls | `docs/CONTROLS.md` |

**A measurement that beats the mechanism is a broken measurement, not a fast robot.** Four
tools in this project have printed times faster than the 0.60 s the feed physically takes, and
every one of them was wrong. If a number looks too good, check it against `cycleTime_s` first.
