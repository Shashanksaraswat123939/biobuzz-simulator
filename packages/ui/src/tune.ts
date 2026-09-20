/**
 * Live sliders over every constant the simulator actually runs on.
 *
 * The rule this panel exists to enforce: nothing physical is written in code. Each entry
 * reads and writes a field in `config/params.json` or `config/robot.json`, and the world is
 * rebuilt from those objects, so what you drag here is the same number the physics uses.
 * The hint says where the value came from — measured, from the CAD, or a guess — because a
 * guess with a big effect is the one worth going and measuring.
 */
import { clamp } from '@core/units.js';
import type { Params, RobotSpec } from '@core/types.js';

export interface Tunable {
  group: string;
  label: string;
  hint: string;
  min: number;
  max: number;
  step: number;
  get: (p: Params, r: RobotSpec) => number;
  set: (p: Params, r: RobotSpec, v: number) => void;
  /** True if changing this needs the world rebuilt rather than just written through. */
  rebuild?: boolean;
  fmt?: (v: number) => string;
}

const f1 = (v: number) => v.toFixed(1);
const f2 = (v: number) => v.toFixed(2);
const f3 = (v: number) => v.toFixed(3);
const f0 = (v: number) => v.toFixed(0);

export const TUNABLES: Tunable[] = [
  // ---------------------------------------------------------------- driving
  { group: 'Driving', label: 'Patrol half-angle (deg)', fmt: f0,
    hint: 'How far round the CELL’s opening the driver should go, drawn on the floor by the Patrol zone overlay. NOT the fire gate — that is the turret’s off-opening cap at 60 deg and it is wider. Measured on the stock robot with the gate untouched, varying only how far the patrol drives: ±60 deg lands 66% at 1.19 s per ball IN, ±45 lands 73% at 0.97, ±35 lands 67% at 1.27. The far edge of the legal sector is where the mouth is half shut AND where the robot is reversing, because the extreme bearing is the end of a pass.',
    min: 10, max: 60, step: 1,
    get: (_p, r) => r.shot?.patrolHalfAngle_deg ?? 35,
    set: (_p, r, v) => { r.shot = { ...(r.shot ?? {}), patrolHalfAngle_deg: v }; } },
  // ---------------------------------------------------------------- shooter
  { group: 'Shooter', label: 'Flywheel transfer k', fmt: f3,
    hint: 'Fraction of rim speed the ball leaves with. GUESS. Exit speed is k·ω·r, so this scales every shot — a 5% error here is a 5% range error at every distance.',
    min: 0.20, max: 0.90, step: 0.005,
    get: (_p, r) => r.flywheel.k, set: (_p, r, v) => (r.flywheel.k = v) },
  { group: 'Shooter', label: 'Flywheel inertia (kg·m²)', fmt: (v) => v.toExponential(2),
    hint: 'Sets how far the wheel dips when a ball goes through, and so how fast you can cycle.',
    // The range has to CONTAIN the configured value, or the slider renders pinned at its end
    // showing a number the physics is not using -- and the first drag silently rewrites the
    // constant to the nearest thing the slider can say. This one read 0.0005 against a config
    // of 0.000391, so touching it moved the wheel's inertia by 28%. tests/ui.test.ts locks it.
    min: 0.0001, max: 0.01, step: 0.00001,
    get: (_p, r) => r.flywheel.I_fly_kgm2, set: (_p, r, v) => (r.flywheel.I_fly_kgm2 = v) },
  { group: 'Shooter', label: 'Flywheel radius (mm)', fmt: f0,
    hint: 'Rim radius. Exit speed is linear in it.',
    min: 25, max: 76, step: 1,
    get: (_p, r) => r.flywheel.r_fly_m * 1000, set: (_p, r, v) => (r.flywheel.r_fly_m = v / 1000) },
  { group: 'Shooter', label: 'Energy loss per shot', fmt: f2,
    hint: 'How much more than the ball’s kinetic energy the wheel gives up — slip and squash.',
    min: 1.0, max: 2.5, step: 0.05,
    get: (_p, r) => r.flywheel.lossFactor, set: (_p, r, v) => (r.flywheel.lossFactor = v) },
  { group: 'Shooter', label: 'Speed tolerance (rpm)', fmt: f0,
    hint: 'How close to target the wheel must be before the gate opens. Tighter means fewer bad shots and a slower cycle.',
    min: 20, max: 400, step: 10,
    get: (_p, r) => r.flywheel.tolRpm, set: (_p, r, v) => (r.flywheel.tolRpm = v) },
  { group: 'Shooter', label: 'Launch scatter, elevation (deg)', fmt: f2,
    hint: 'Shot-to-shot elevation noise. Usually the single largest cause of a wide group.',
    min: 0, max: 4, step: 0.05,
    get: (_p, r) => r.flywheel.scatter.angle_deg, set: (_p, r, v) => (r.flywheel.scatter.angle_deg = v) },
  { group: 'Shooter', label: 'Launch scatter, yaw (deg)', fmt: f2,
    hint: 'Shot-to-shot left/right noise. Shows up as lateral spread in the Analysis tab.',
    min: 0, max: 4, step: 0.05,
    get: (_p, r) => r.flywheel.scatter.yaw_deg, set: (_p, r, v) => (r.flywheel.scatter.yaw_deg = v) },
  { group: 'Shooter', label: 'Launch scatter, speed (%)', fmt: f2,
    hint: 'Shot-to-shot speed noise from how the ball is gripped.',
    min: 0, max: 5, step: 0.05,
    get: (_p, r) => r.flywheel.scatter.speedFrac * 100, set: (_p, r, v) => (r.flywheel.scatter.speedFrac = v / 100) },

  // ---------------------------------------------------------------- turret
  { group: 'Turret and hood', label: 'Turret max speed (deg/s)', fmt: f0,
    hint: 'How fast the axis can slew. Too slow and shots leave before the aim has arrived.',
    min: 30, max: 720, step: 10,
    get: (_p, r) => r.turret.speed_dps, set: (_p, r, v) => (r.turret.speed_dps = v) },
  { group: 'Turret and hood', label: 'Turret acceleration (deg/s²)', fmt: f0,
    hint: 'Sets how much of a swing is spent accelerating. The profile is trapezoidal. A servo head reaches its slew limit in a few hundredths; a geared motor takes a fifth of a second.',
    min: 100, max: 8000, step: 50,
    get: (_p, r) => r.turret.accel_dps2, set: (_p, r, v) => (r.turret.accel_dps2 = v) },
  { group: 'Turret and hood', label: 'Muzzle height (mm)', fmt: f0, rebuild: true,
    hint: 'Where the ball leaves the robot. Changes the whole shot table.',
    min: 150, max: 660, step: 5,
    get: (_p, r) => r.turret.muzzleHeight_m * 1000, set: (_p, r, v) => (r.turret.muzzleHeight_m = v / 1000) },
  { group: 'Turret and hood', label: 'Hood minimum (deg)', fmt: f0, rebuild: true,
    hint: 'Flattest shot the hood can make.',
    min: 10, max: 60, step: 1,
    get: (_p, r) => r.hood.angleRange_deg[0], set: (_p, r, v) => (r.hood.angleRange_deg[0] = v) },
  { group: 'Turret and hood', label: 'Hood maximum (deg)', fmt: f0, rebuild: true,
    hint: 'Steepest shot. Extending this past 60° is what made long shots land instead of rebounding.',
    min: 40, max: 88, step: 1,
    get: (_p, r) => r.hood.angleRange_deg[1], set: (_p, r, v) => (r.hood.angleRange_deg[1] = v) },

  // ---------------------------------------------------------------- ball and air
  { group: 'Ball and air', label: 'Drag coefficient Cd', fmt: f2,
    hint: 'GUESS. 0.45 is pickleball-derived and uncited; a 26-hole hollow ball is not a smooth sphere.',
    min: 0.20, max: 0.90, step: 0.01,
    get: (p) => p.ball.Cd, set: (p, _r, v) => (p.ball.Cd = v) },
  { group: 'Ball and air', label: 'Magnus slope Cl', fmt: f2,
    hint: 'GUESS. Backspin lift per unit spin ratio. Lifts long shots and shortens flat ones.',
    min: 0, max: 0.50, step: 0.01,
    get: (p) => p.ball.clSlope, set: (p, _r, v) => (p.ball.clSlope = v) },
  { group: 'Ball and air', label: 'Air density (kg/m³)', fmt: f3,
    hint: 'Venue altitude and temperature. 1.225 is sea level at 15 °C.',
    min: 0.95, max: 1.30, step: 0.005,
    get: (p) => p.env.rho, set: (p, _r, v) => (p.env.rho = v) },
  { group: 'Ball and air', label: 'POLLEN mass (g)', fmt: f1, rebuild: true,
    hint: 'Heavier balls fly flatter and carry less. Measure a real one.',
    min: 15, max: 60, step: 0.5,
    get: (p) => p.ball.pollen.m_kg * 1000, set: (p, _r, v) => (p.ball.pollen.m_kg = v / 1000) },
  { group: 'Ball and air', label: 'Restitution on polycarbonate', fmt: f2,
    hint: 'GUESS. How hard a ball bounces inside the CELL. High values throw good shots back out.',
    min: 0.10, max: 0.90, step: 0.01,
    get: (p) => p.ball.e_poly, set: (p, _r, v) => (p.ball.e_poly = v) },

  // ---------------------------------------------------------------- hive
  { group: 'Hive', label: 'Rocker mass (kg)', fmt: f2, rebuild: true,
    hint: 'CAD estimate 2.38, plausible 1.5–3.5. The biggest lever on how many balls tip it.',
    min: 1.0, max: 4.0, step: 0.05,
    get: (p) => p.hive.massKg, set: (p, _r, v) => (p.hive.massKg = v) },
  { group: 'Hive', label: 'CG height above pivot (mm)', fmt: f0, rebuild: true,
    hint: 'CAD 53.6 mm. With the mass this sets the restoring torque holding the rocker down.',
    min: 12, max: 127, step: 1,
    get: (p) => p.hive.cgOffset_m[1] * 1000, set: (p, _r, v) => (p.hive.cgOffset_m[1] = v / 1000) },
  { group: 'Hive', label: 'Pivot friction (N·m)', fmt: f3, rebuild: true,
    hint: 'Unknown. Dry friction at the axle; delays the tip and adds hysteresis.',
    min: 0, max: 0.5, step: 0.005,
    get: (p) => p.hive.frictionTorque_Nm, set: (p, _r, v) => (p.hive.frictionTorque_Nm = v) },
  { group: 'Hive', label: 'Damper (N·m·s)', fmt: f2, rebuild: true,
    hint: 'Blum 970A, no published curve. Slows the slam at each end stop.',
    min: 0, max: 8, step: 0.1,
    get: (p) => p.hive.damperC_Nms, set: (p, _r, v) => (p.hive.damperC_Nms = v) },

  // ---------------------------------------------------------------- chassis
  { group: 'Chassis', label: 'Robot mass (kg)', fmt: f1, rebuild: true,
    hint: 'Inspection limit is 42 lb / 19.05 kg. Heavier accelerates slower and pushes harder.',
    min: 8, max: 19, step: 0.25,
    get: (_p, r) => r.chassis.mass_kg, set: (_p, r, v) => (r.chassis.mass_kg = v) },
  { group: 'Chassis', label: 'Tile friction µ', fmt: f2,
    hint: 'Rubber wheel on foam tile. Sets how hard the robot can accelerate before it slips.',
    min: 0.30, max: 1.40, step: 0.01,
    get: (p) => p.env.tileMu, set: (p, _r, v) => (p.env.tileMu = v) },
  { group: 'Chassis', label: 'Drivetrain efficiency', fmt: f2,
    hint: 'Gearbox and chain losses between the motor and the wheel.',
    min: 0.50, max: 1.00, step: 0.01,
    get: (_p, r) => r.drivetrain.eta, set: (_p, r, v) => (r.drivetrain.eta = v) },
  { group: 'Chassis', label: 'Speed cap (m/s)', fmt: (v) => (v <= 0 ? 'off' : f2(v)),
    // LEFT END IS OFF, not "very slow". A cap of 0.05 m/s is a robot that cannot drive, and
    // the shipping default is no cap at all, so the slider has to be able to SAY no cap --
    // otherwise the only way back from having dragged it is to edit the JSON.
    hint: 'Limits ground speed in m/s, which is the unit a refused shot is refused in: charging the mouth faster than the ball’s own horizontal speed leaves no launch under the hood’s stop, so at 1.7 m/s the robot cannot shoot inside 50 in. The drive gear (Y/A) is a POWER fraction and is not this. Off at the left end.',
    min: 0, max: 2.2, step: 0.05,
    get: (_p, r) => r.drivetrain.maxSpeed_mps ?? 0,
    set: (_p, r, v) => (r.drivetrain.maxSpeed_mps = v) },
  { group: 'Chassis', label: 'Battery internal R (Ω)', fmt: f3,
    hint: 'Sets the voltage sag under load, and so the top speed late in a match.',
    min: 0.01, max: 0.30, step: 0.005,
    get: (p) => p.battery.Rint_ohm, set: (p, _r, v) => (p.battery.Rint_ohm = v) },

  // ---------------------------------------------------------------- cycle
  { group: 'Cycle', label: 'Transfer cycle time (s)', fmt: f2,
    hint: 'Minimum gap between two shots. The floor on how fast you can empty a hopper.',
    min: 0.2, max: 3.0, step: 0.05,
    get: (_p, r) => r.transfer.cycleTime_s, set: (_p, r, v) => (r.transfer.cycleTime_s = v) },
  { group: 'Cycle', label: 'Hopper capacity', fmt: f0, rebuild: true,
    // Capped at the RULE, not at what a bin could physically hold. It used to go to 12 under
    // a hint that said "rules cap this; check the manual", which is a constraint written as a
    // suggestion -- and the config it shipped with (6) was already illegal.
    hint: 'How many SCORING ELEMENTS the robot may control at once. G407 caps this at 4.',
    min: 1, max: 4, step: 1,
    get: (_p, r) => r.hopper.capacity, set: (_p, r, v) => (r.hopper.capacity = Math.round(v)) },
];

/**
 * Build the sliders, grouped. `onChange` fires on every drag with whether that knob needs a
 * rebuild, so the caller can write cheap values straight through and defer the expensive ones.
 */
export function buildTunePanel(
  host: HTMLElement,
  params: Params,
  robot: RobotSpec,
  onChange: (t: Tunable) => void,
): void {
  host.innerHTML = '';
  let group = '';
  for (const t of TUNABLES) {
    if (t.group !== group) {
      group = t.group;
      const h = document.createElement('h4');
      h.textContent = group;
      host.appendChild(h);
    }
    const fmt = t.fmt ?? f3;
    const wrap = document.createElement('div');
    wrap.className = 'tune';
    const start = t.get(params, robot);
    wrap.innerHTML = `
      <div class="top"><span>${t.label}${t.rebuild ? ' *' : ''}</span>
        <input type="number" min="${t.min}" max="${t.max}" step="${t.step}" value="${start}" />
      </div>
      <input type="range" min="${t.min}" max="${t.max}" step="${t.step}" value="${start}" />
      <div class="ends"><i>${fmt(t.min)}</i><i>${fmt(t.max)}</i></div>
      <small>${t.hint}</small>`;
    const slider = wrap.querySelector<HTMLInputElement>('input[type=range]')!;
    const box = wrap.querySelector<HTMLInputElement>('input[type=number]')!;
    // The filled part of the track is where the value sits in its own span. Chrome will not
    // paint a range track from the value on its own, so it is a custom property the two
    // inputs both keep up to date.
    const paint = (v: number) => {
      wrap.style.setProperty('--fill', `${((v - t.min) / Math.max(1e-9, t.max - t.min)) * 100}%`);
      wrap.classList.toggle('changed', Math.abs(v - start) > t.step / 2);
    };
    const apply = (raw: number, echo: HTMLInputElement) => {
      // A TYPED VALUE IS STILL A LEGAL VALUE. The panel writes straight into the objects the
      // physics runs on, so an out-of-range or off-step entry would put the simulator in a
      // state no slider can represent or return from.
      const v = clamp(Math.round(raw / t.step) * t.step, t.min, t.max);
      t.set(params, robot, v);
      echo.value = String(Number(v.toFixed(6)));
      paint(v);
      onChange(t);
    };
    slider.oninput = () => apply(Number(slider.value), box);
    box.onchange = () => { if (Number.isFinite(Number(box.value))) apply(Number(box.value), slider); else box.value = String(t.get(params, robot)); };
    paint(start);
    host.appendChild(wrap);
  }
  const note = document.createElement('p');
  note.className = 'cap';
  note.innerHTML = '* takes effect on <b>Apply &amp; restart</b>. Everything else is live.';
  host.appendChild(note);
}
