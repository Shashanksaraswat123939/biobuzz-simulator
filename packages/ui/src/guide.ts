/**
 * Every control in the app, with what it actually does.
 *
 * This is the reference the Guide tab renders and the only place the control list is
 * written down, so it cannot drift from the keymap the way a hand-kept README does.
 */

export interface ControlDoc {
  /** What you press. */
  key: string;
  /** The same thing on a gamepad, if it has one. */
  pad?: string;
  what: string;
  /** What it does, in one or two sentences. */
  why: string;
}

export interface ControlGroup {
  title: string;
  blurb?: string;
  items: ControlDoc[];
}

export const CONTROLS: ControlGroup[] = [
  {
    title: 'Driving',
    blurb: 'Mecanum, and ROBOT-CENTRIC: forward is the robot’s NOSE, which is where the intake is. Almost everything you do with the chassis is aim the intake at a ball, and a field-relative stick makes that a mental rotation on every approach. Hold C (R3) for field-centric if you prefer it.',
    items: [
      { key: 'W S', pad: 'left stick ↕', what: 'Forward and back, along the nose', why: 'On a keyboard the stick ramps to full over about a fifth of a second rather than stepping, because a key has no middle and a 1.6 m/s drivetrain asked for in one step is undriveable.' },
      { key: '—', pad: 'R2 / L2', what: 'Forward and back on the triggers', why: 'Adds to the stick rather than replacing it, so you can strafe with your thumb and throttle with your finger at the same time. Analog: half a trigger is half the command.' },
      { key: 'A D', pad: 'left stick ↔', what: 'Strafe left and right', why: 'Sideways without turning, and the sides turn with the robot. Mecanum strafing is about 70% as efficient as driving straight and the simulator charges you that.' },
      { key: 'Q E', pad: 'X / B', what: 'Turn left and right', why: 'X turns left, B turns right. THE BUTTON RAMPS ON PURPOSE, and slower than the drive sticks: a tap of up to about 200 ms stays under 70 °/s, which is the rotation rate the fire gate will still shoot through, and holding it builds to a full 270 °/s for getting the nose round. Before that ramp existed every turn was full rotation and every turn killed the shot.' },
      { key: 'R F', pad: 'Y (wraps) / A', what: 'Drive speed gear, up and down', why: 'Five gears. On the pad, Y alone walks them: past the top gear it wraps back to the bottom, because A is the fire paddle on a controller that mirrors M1 onto it. The bar under the status strip reads the top speed the gear allows in m/s, worked out from the drive motor’s free speed and the wheel radius rather than written down.' },
      { key: 'C', pad: 'R3', what: 'Field-centric while held', why: 'Forward becomes away-from-the-driver-station whatever way the robot points, measured from the last re-zero. On C rather than Ctrl: a modifier is the one key whose keyup you reliably miss, and a missed keyup leaves the drive frame stuck in the mode you are not in.' },
      { key: 'Backspace', what: 'Re-zero the field frame', why: 'Makes “away” mean whichever way the robot faces now. Keyboard only — every one of the pad’s standard buttons already has a job.' },
    ],
  },
  {
    title: 'Shooting',
    blurb: 'The intake always runs, like a real one, and it rejects the opponent’s NECTAR by reversing — G408 is enforced by the roller, not by a penalty. The gate opens itself when a shot is cleared, so there is nothing to arm by hand.',
    items: [
      { key: 'Space', pad: 'R1 / M1 / A', what: 'Auto-fire on/off', why: 'A latch, not a trigger: leave it on and a ball goes every time the shot clears the gate — the wheel inside its tolerance band, the turret on target, and the CELL mouth open towards you — paced by the transfer cycle rather than your thumb. Also a button on the action bar.' },
      { key: 'T', pad: 'L1', what: 'Auto-aim on/off', why: 'On: the turret and hood solve for your up CELL every loop, including a lead for the robot’s own velocity, so you can shoot while driving across the shot. Off: the turret holds where you last put it.' },
      { key: 'G', pad: 'L3', what: 'Fire, while held', why: 'By hand, for when you want exactly one. Same gate as the latch; let go and it stops.' },
      { key: ', .', pad: 'D-pad ← → / M2', what: 'Nudge the turret anticlockwise and clockwise', why: 'By hand, when auto-aim is off. Slews at 100 °/s; the axis has its own acceleration limit, so it does not snap. The M1 paddle used to do the anticlockwise nudge and now fires instead, so on a pad with paddles that half lives on D-pad left; the comma and full-stop keys are unchanged.' },
      { key: 'V', pad: 'D-pad ↑', what: 'Pre-spin the flywheel', why: 'Optional. Brings the wheel up before you commit, so the first shot does not wait about three seconds for spin-up. Firing does this anyway.' },
      { key: 'Z', pad: 'D-pad ↓', what: 'Reverse the intake', why: 'Spits out whatever is at the mouth. The same traction model that pulls a ball in pushes it back out; there is no separate eject.' },
    ],
  },
  {
    title: 'Finding the goal',
    blurb:
      'The robot is not told where the HIVE is. It reads an AprilTag on the up CELL through a '
      + 'camera bolted to the turret — 60° of lens, 30 frames a second, 75 ms behind, and blind '
      + 'past 120 in or 65° round the side. Everything the shooter does hangs off that one '
      + 'measurement, so this is the first thing to check when it will not shoot.',
    items: [
      {
        key: 'SEARCHING',
        what: 'It cannot see the tag',
        why: 'The turret is sweeping to find one. This is normal, not a fault — and note that from the '
          + 'start tile it is also CORRECT: the mouth faces about 109° away from there, so no shot could '
          + 'enter even if it could see. Drive round into the shooting sector and it locks on within a sweep.',
      },
      {
        key: 'CELL A/B',
        what: 'Locked on, and which CELL',
        why: 'The two CELLs carry different tags, so the ID is how the robot knows which one is up — it is '
          + 'the only way it ever learns the HIVE tipped. Green means the fix is fresh enough to shoot on.',
      },
      {
        key: 'fix age',
        what: 'How old the last detection is',
        why: 'Never zero: the pipeline is 75 ms behind and runs slower than the control loop. Under 250 ms it '
          + 'may fire. Over that it keeps aiming — carrying the fix forward on odometry — but holds fire, '
          + 'because a shot solved off a stale fix is solved off where the goal WAS.',
      },
      {
        key: 'mouth angle',
        what: 'How square you are onto the opening',
        why: 'Past 75° the opening has too little area to enter and the gate refuses the shot. A TIP swaps '
          + 'which CELL is up and the new one opens the other way, so this can jump from fine to hopeless '
          + 'without you moving — drive round.',
      },
      {
        key: 'Sight line',
        what: 'The ray it is trying to decode along',
        why: 'Turn it on in the deck. It draws the line from the AprilTag panel on the up CELL to the '
          + 'camera: GREEN while it is decoding, RED while the geometry refuses. It is the difference '
          + 'between "the robot is broken" and "you are stood round the side of the mouth". Note it '
          + 'starts at the robot’s centre at muzzle height, because that is where the model puts the '
          + 'camera — there is no mount offset yet.',
      },
      {
        key: 'range',
        what: 'Two of them, on purpose',
        why: 'The Robot panel shows “range to CELL (true)” and “range it believes” side by side. The first is '
          + 'the world’s; the second is what the camera told the robot. The gap between them is the error the '
          + 'shooter is actually working against.',
      },
    ],
  },
  {
    title: 'View',
    blurb: 'The right stick is the VIEW, in every camera mode — including the ones that ride the robot, which could not look around at all before.',
    items: [
      { key: '← →', pad: 'right stick ↔', what: 'Look left and right', why: 'Swings the eye round the robot in Follow, Driver and Muzzle, and swings the orbit in Orbit. Full deflection is about a half turn a second; a small one is a fine correction, because the response is squared.' },
      { key: '↑ ↓', pad: 'right stick ↕', what: 'Look up and down', why: 'The same elevation everywhere, so the view reads the same when you switch modes.' },
      { key: 'left-drag', what: 'Orbit the camera', why: 'Anywhere on the field. The camera never moves the robot.' },
      { key: 'right-drag', what: 'Pan', why: 'Slides the view across the field, the way every other 3D view does. Middle-drag and shift+drag do the same.' },
      { key: 'scroll', what: 'Zoom', why: '' },
      { key: '1 – 5', what: 'Camera presets', why: 'Orbit, Follow (behind the robot), Driver (on the robot, looking where it points), Overhead, Muzzle (down the barrel — the turret is usually pointed somewhere the chassis is not).' },
    ],
  },
  {
    title: 'Match and modes',
    items: [
      { key: 'Enter', pad: 'Start', what: 'The green button', why: 'Starts the match clock in Practice and Test; runs or stops the autonomous routine in Auto; starts or stops the collection run in Data.' },
      { key: 'P', what: 'Pause', why: 'Freezes the physics. The camera still moves. Also a button on the action bar; it has no pad binding any more, because L3 re-zeros the field frame and one button doing two jobs is how the old layout got confusing.' },
      { key: 'N', what: 'New match', why: 'Rebuilds from the current variables: robot on its start tile, balls re-staged, score and shot log cleared. On N rather than R, because R is the speed gear — wiping the match because a driver reached for more speed is the worst collision there is.' },
      { key: 'L', what: 'Auto-fill the hopper', why: 'Practice aid, not a game rule: quietly picks up the nearest POLLEN lying on the floor whenever there is room. It takes real balls off the real field, so the field does run out.' },
      { key: 'M', pad: 'Select', what: 'Next mode', why: 'Cycles Practice → Auto → Data → Test.' },
    ],
  },
];

export const MODES: { name: string; what: string }[] = [
  {
    name: 'Practice',
    what: 'You drive, the clock runs, and the scoring is the real scoring. Nothing is scripted — the HIVE tips because the balls in it beat gravity, not because a counter reached a number.',
  },
  {
    name: 'Auto',
    what: 'The 30-second autonomous, played out. The routine LEAVEs the wall, drives round into the shooting sector — it has to, because the up CELL opens toward the audience and from the start tile the mouth is about 109° off its opening, so no launch from there can enter — empties the preload into the CELL, and PARKs before the buzzer. It drives through the same gamepad frames a human would produce, so nothing about the shot is privileged. `npm run tool -- tools/autocheck.ts` runs the same routine headlessly over several seeds, which is the honest way to judge it.',
  },
  {
    name: 'Data',
    what: 'The robot drives itself through a sweep of firing positions across a range band, settles (or keeps rolling, if you ask it to), and takes one shot per sample. It goes through the same aim solver and readiness gate you do, so the numbers mean something. When it finishes, the Analysis tab separates bias from spread and names the fault.',
  },
  {
    name: 'Test',
    what: 'Sandbox. Every physical constant is a slider in Variables, and the Predictor shows what each one is doing to the shot you are about to take — measured by re-running the trajectory with that one value nudged, not estimated.',
  },
];

export const PANELS: { name: string; what: string }[] = [
  { name: 'Robot', what: 'Live state: pose, what the camera can see of the goal and how stale it is, shooter, the HIVE tip meter with the torque balance behind it, per-wheel slip, and whatever the brain is putting on telemetry.' },
  { name: 'Predictor', what: 'The error budget for the current shot. Each row is (inches the ball moves per unit of that variable) × (that variable’s own 1 σ uncertainty). The bar is its share of the total variance. Hover a row name for where its uncertainty comes from.' },
  { name: 'Analysis', what: 'Starts with the SCOREBOARD: where both alliances’ points come from, itemised. Everything positional is credited at the buzzer, so until the clock stops those are projections — what you would score if it went now, from the same arithmetic as the final. A dimmed row is an avenue you are not using. Then the shot log and what it means: bias is a table error that one offset fixes; spread is a repeatability error that no offset fixes. Export CSV to take the raw log elsewhere.' },
  { name: 'Variables', what: 'Every constant the simulator runs on, read from config/*.json. Rows marked * are baked into rigid bodies and need Apply & restart; everything else is live.' },
  { name: 'Guide', what: 'This page.' },
];

/** Overlays that live on the field rather than in the panel. */
export const OVERLAYS: { name: string; what: string }[] = [
  { name: 'Shot zone', what: 'Where a shot is worth taking, with a legend bottom-left that counts the squares live. GREEN lands. BROWN is inside the shot table’s nearest row — drive back. RED is too far, or no launch fits. GREY is the one worth knowing: the CELL does not open that way and no launch can enter, which is over half the field and is why you cannot score from your own start tile. It is redrawn at the speed you are ACTUALLY doing, because the speed a shot has to leave at depends on what the chassis is doing — the green visibly shrinks as you drive at it. A TIP turns the mouth round and takes the whole map with it.' },
  { name: 'Shot arc', what: 'YELLOW is where the solver says this shot will go, drawn with the integrator the shot table is built from. BLUE is where the last ball actually went. Where they part company is the thing worth chasing.' },
  { name: 'AprilTag', what: 'The status strip says whether the turret camera can read the tag on your own up CELL. Locked means the aim is MEASURED and carries no heading error; no tag means it has fallen back to odometry and inherits the IMU’s drift.' },
];

export function renderGuide(host: HTMLElement): void {
  const esc = (s: string) => s.replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' })[c]!);
  const parts: string[] = [];

  parts.push('<div class="guide-group"><h5>Modes</h5>');
  for (const m of MODES) parts.push(`<div class="guide-item"><kbd>${m.name}</kbd><p>${esc(m.what)}</p></div>`);
  parts.push('</div>');

  for (const g of CONTROLS) {
    parts.push(`<div class="guide-group"><h5>${esc(g.title)}</h5>`);
    if (g.blurb) parts.push(`<p class="cap">${esc(g.blurb)}</p>`);
    for (const c of g.items) {
      const pad = c.pad ? ` <span style="opacity:.6">/ ${esc(c.pad)}</span>` : '';
      parts.push(`<div class="guide-item"><kbd>${esc(c.key)}</kbd><p><b>${esc(c.what)}</b>${pad}${c.why ? `<br>${esc(c.why)}` : ''}</p></div>`);
    }
    parts.push('</div>');
  }

  parts.push('<div class="guide-group"><h5>Panels</h5>');
  for (const p of PANELS) parts.push(`<div class="guide-item"><kbd>${p.name}</kbd><p>${esc(p.what)}</p></div>`);
  parts.push('</div>');

  parts.push('<div class="guide-group"><h5>On the field</h5>');
  for (const o of OVERLAYS) parts.push(`<div class="guide-item"><kbd>${o.name}</kbd><p>${esc(o.what)}</p></div>`);
  parts.push('</div>');

  host.innerHTML = parts.join('');
}
