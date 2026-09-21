/** Keyboard mapped onto a Gamepad, so the brain only ever sees one input shape. */
import { emptyGamepad } from '@core/physics/world.js';
import type { GamepadState } from '@core/types.js';

export interface Keys {
  /** Keys currently held: for continuous controls like drive. */
  down: Set<string>;
  /**
   * Keys pressed since the last read, latched on keydown. Polling `down` would drop a tap
   * that starts and ends between two animation frames, which is most of them.
   */
  pressed: Set<string>;
}

export function installKeyboard(): Keys {
  const keys: Keys = { down: new Set(), pressed: new Set() };
  addEventListener('keydown', (e) => {
    if ((e.target as HTMLElement).tagName === 'INPUT') return;
    const k = e.key.toLowerCase();
    keys.down.add(k);
    if (!e.repeat) keys.pressed.add(k);
    // Space scrolls and the arrows scroll; neither should while driving.
    if (e.key === ' ' || e.key.startsWith('Arrow')) e.preventDefault();
  });
  addEventListener('keyup', (e) => keys.down.delete(e.key.toLowerCase()));
  addEventListener('blur', () => {
    keys.down.clear();
    keys.pressed.clear();
  });
  return keys;
}

/**
 * The two paddle buttons. `Gamepad.buttons` only standardises 17 entries, and M1/M2 on the
 * pads that have them sit past that in a vendor block, so they are read positionally and fall
 * back to L1/L2 on a pad without them. Carried beside the frame rather than added to
 * `GamepadState`, which is the wire the Java OpMode sees and has no such button.
 */
export interface Paddles {
  m1: boolean;
  m2: boolean;
  /**
   * The M1 paddle, on the auto-fire latch. Separate from `m1` because m1's fallbacks -- D-pad
   * left and the comma key -- still nudge the turret, and a pad without paddles must not lose
   * that control just because a pad with them gained a second way to shoot.
   */
  fire: boolean;
  /**
   * Re-zero the field frame. Keyboard only: every one of the pad's 17 standard buttons now
   * has a job, and inventing an 18th to put this on would be a button nobody's pad has.
   * R3 gives field-centric on demand, so this is the rare correction rather than the control.
   */
  rezero: boolean;
}

/**
 * PHYSICAL keyboard state, in gamepad shape: which BUTTON is down, not what it does. The
 * mapping from button to action lives in one place (`remap` in main.ts) so the keyboard and a
 * real controller cannot drift apart.
 *
 * `pressed` is NOT cleared here: a frame may read the input and then run no physics step,
 * and a toggle consumed by nobody is a toggle the user has to press twice. The caller
 * clears it once a step has actually seen it.
 */
export function readKeyboard(keys: Keys): GamepadState & { paddles: Paddles } {
  const g = emptyGamepad() as GamepadState & { paddles: Paddles };
  const on = (k: string) => keys.down.has(k);
  // Left stick: drive. W/S forward and back, A/D strafe.
  g.left_stick_y = (on('s') ? 1 : 0) - (on('w') ? 1 : 0);
  g.left_stick_x = (on('d') ? 1 : 0) - (on('a') ? 1 : 0);
  // Right stick: the view. Arrows, because that is what a keyboard user reaches for.
  g.right_stick_x = (on('arrowright') ? 1 : 0) - (on('arrowleft') ? 1 : 0);
  g.right_stick_y = (on('arrowdown') ? 1 : 0) - (on('arrowup') ? 1 : 0);
  // The TRIGGERS are forward and back as well, for a thumb that is busy strafing.
  g.right_trigger = on('arrowup') && on('shift') ? 1 : 0;   // see the pad: R2 forward
  g.left_trigger = on('arrowdown') && on('shift') ? 1 : 0;  //               L2 back
  // Face buttons: Q/E turn (X/B), R/F change gear (Y/A).
  g.x = on('q');
  g.b = on('e');
  g.y = keys.pressed.has('r');
  g.a = keys.pressed.has('f');
  g.right_bumper = on(' ');    // R1: auto-fire latch
  g.left_bumper = keys.pressed.has('t');   // L1: auto-aim toggle
  g.dpad_up = keys.pressed.has('v');       // pre-spin the flywheel
  g.dpad_down = on('z');                   // reverse the intake
  // L3 is a HOLD now that it fires by hand: `pressed` is a one-frame pulse, which would
  // give a single ball per press however long the key is down.
  g.left_stick_button = on('g');                // L3: fire while held
  // Field-centric is a HOLD, and it used to be on Ctrl. A modifier is the one key whose
  // keyup you reliably miss -- Ctrl+R, Ctrl+Shift+I, alt-tab -- and a missed keyup leaves
  // the drive frame silently stuck in the mode you are not in.
  g.right_stick_button = on('c');
  // M1 / M2: nudge the turret anticlockwise and clockwise.
  // No keyboard equivalent for the fire paddle: space already is the latch.
  g.paddles = { m1: on(','), m2: on('.'), fire: false, rezero: keys.pressed.has('backspace') };
  return g;
}

/**
 * PHYSICAL buttons to what they DO, in one place.
 *
 * The pad the driver holds and the frame the brain reads are deliberately not the same thing.
 * The brain's `GamepadState` is the wire the Java OpMode sees, so its field names are fixed;
 * the driver's layout is not.
 *
 *   left stick   translate              right stick  the VIEW (never the robot)
 *   R2 / L2      forward / back         X / B        turn left / right
 *   Y / A        speed gear up / down   R1           auto-fire latch
 *   L1           auto-aim toggle        L3           fire while held
 *   M1           auto-fire latch        M2           turret clockwise
 *   R3           hold for field-centric
 *   D-pad up     pre-spin the flywheel  D-pad down   reverse the intake
 *   D-pad left/right nudge the turret anti/clockwise on a pad without paddles.
 *
 * Turning moved off the right stick because the right stick moves the camera, and the brain's
 * yaw command is still `right_stick_x` -- so X/B are synthesised into it here, BEFORE the
 * ramp, which means a button press feathers the yaw exactly like a stick deflection instead
 * of slamming it. Nothing downstream of this function knows the layout changed.
 */
export function remap(phys: GamepadState, pad: Paddles): GamepadState {
  // THE TRIGGERS DRIVE FORWARD AND BACK, added to the stick rather than replacing it, so a
  // driver can strafe on the stick and throttle on the triggers at the same time. Stick up is
  // negative, which is why R2 subtracts.
  const throttle = clamp(phys.left_stick_y + phys.left_trigger - phys.right_trigger, -1, 1);
  return {
    ...phys,
    left_stick_y: throttle,
    right_stick_x: (phys.b ? 1 : 0) - (phys.x ? 1 : 0),  // B right, X left
    x: phys.left_bumper,             // L1: auto-aim toggle
    // R1 IS THE LATCH. It is the button a thumb rests on, and the shooter's honest rate is
    // set by the transfer cycle, not by how fast you can tap -- so the trigger that felt
    // like the main one was the one that could not keep up with the hardware. Holding it
    // by hand is still there, moved to L3.
    // R1 OR THE M1 PADDLE. One latch, two buttons: the brain toggles on the EDGE of this, so
    // the two cannot fight -- whichever is pressed first flips it and the other is a no-op
    // until both are released.
    right_bumper: phys.right_bumper || pad.fire,   // R1 / M1: auto-fire latch
    b: phys.left_stick_button,              // L3: fire while held
    left_bumper: false,              // the crawl button is gone; the speed gear replaced it
    left_trigger: phys.dpad_down ? 1 : 0,   // D-pad down: reverse the intake
    left_stick_button: pad.rezero,          // keyboard only: re-zero the field frame
    // M1 / M2 nudge the turret by hand when auto-aim is off. The brain reads them as the
    // D-pad, which is where that control used to live.
    dpad_left: pad.m1,
    dpad_right: pad.m2,
    dpad_down: false,
  };
}

const clamp = (v: number, lo: number, hi: number) => (v < lo ? lo : v > hi ? hi : v);
