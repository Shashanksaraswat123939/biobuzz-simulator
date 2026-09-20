/**
 * Three.js view of the world. It draws the same primitives the physics collides with, so
 * what you see is what the solver sees -- there is no separate render mesh to drift out of
 * step with the colliders.
 */
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
// `?url` so Vite treats the 12 MB STEP tessellation as an asset instead of trying to parse
// it as a module (which serves a 200 that GLTFLoader cannot read).
/**
 * The tessellated field, IF it has been generated.
 *
 * A plain `import ... from '../../../assets/field.glb?url'` is a build-time dependency, and
 * the file is deliberately not in git -- it is 19 MB generated from the STEP by
 * tools/cad2assets.py, and .gitignore says so. So a fresh clone could not build at all: vite
 * stopped with "Could not resolve ../../../assets/field.glb?url" before it rendered anything.
 *
 * `import.meta.glob` matches nothing when the file is absent, which turns a build error into
 * `undefined` -- and the procedural stand-in below is already the answer to that.
 */
const CAD_URLS = import.meta.glob('../../../assets/field.glb', {
  query: '?url',
  import: 'default',
  eager: true,
}) as Record<string, string>;
const fieldUrl: string | undefined = Object.values(CAD_URLS)[0];
import { type FieldGeometry, type BoxPiece, fromCellLocal } from '@core/field/geometry.js';
import { inches, M_TO_IN, DEG } from '@core/units.js';
import { pThread } from '@core/physics/ballistics.js';
import { loadLandCal } from '@core/robot/loadCal.js';
import type { RobotSpec, Snapshot, Vec3 } from '@core/types.js';

const COL = {
  tile: 0x39434f,
  tileAlt: 0x323b46,
  wall: 0x4a525c,
  rail: 0x1d2228,
  frame: 0x98a2ae,
  red: 0xd0342c,
  blue: 0x2f6fd0,
  pollen: 0xf2b705,
  robot: 0x5b6470,
  flower: 0x4bbf8a,
  zone: 0xffffff,
  // the practice room the field stands in
  roomFloor: 0x2a9db0,
  roomFloorAlt: 0x2690a2,
};

export type CameraMode = 'orbit' | 'follow' | 'top' | 'muzzle' | 'fpv';

/** How much of a shaft's real speed to show, so fast parts do not strobe. See update(). */
const SPIN_SHOWN = 0.12;

/**
 * POLLEN and NECTAR are hollow 26-hole balls, and that is most of what they look like.
 * Punching real holes in the geometry would cost a CSG per ball; an alpha map on a sphere
 * gives the same read for one 256px canvas, and the holes are see-through from both sides.
 */
/** A square bar spanning two points. Diagonal members are most of a real FTC field. */
function bar(a: Vec3, b: Vec3, thick: number, mat: THREE.Material): THREE.Mesh {
  const dir = new THREE.Vector3(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
  const len = dir.length();
  const m = new THREE.Mesh(new THREE.BoxGeometry(thick, thick, len), mat);
  m.position.set((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2);
  m.lookAt(new THREE.Vector3(b[0], b[1], b[2]));
  return m;
}

function holedBallTexture(): THREE.Texture {
  const W = 512;
  const H = 256;
  // 26 hole axes: the 6 face, 12 edge and 8 corner directions of a cube. That is exactly
  // the 26 a POLLEN has, and it spaces them evenly without any pole pile-up.
  const axes: [number, number, number][] = [];
  for (const v of [-1, 0, 1]) {
    for (const w of [-1, 0, 1]) {
      for (const u of [-1, 0, 1]) {
        if (u || v || w) {
          const n = Math.hypot(u, v, w);
          axes.push([u / n, v / n, w / n]);
        }
      }
    }
  }
  // Hole half-angle. The closest two of the 26 axes are about 35 deg apart (edge to
  // corner), so anything past ~0.15 rad makes neighbouring holes merge and the ball
  // dissolves into spikes.
  const cosLimit = Math.cos(0.135);

  const cv = document.createElement('canvas');
  cv.width = W;
  cv.height = H;
  const g = cv.getContext('2d')!;
  const img = g.createImageData(W, H);
  for (let y = 0; y < H; y++) {
    const theta = (y / (H - 1)) * Math.PI; // 0 at +Y pole
    const sy = Math.cos(theta);
    const st = Math.sin(theta);
    for (let x = 0; x < W; x++) {
      const phi = (x / W) * Math.PI * 2;
      const dx = st * Math.cos(phi);
      const dz = st * Math.sin(phi);
      // How far into the nearest hole this texel is, so the edge can be faded rather than cut.
      let best = -1;
      for (const a of axes) {
        const d = dx * a[0] + sy * a[1] + dz * a[2];
        if (d > best) best = d;
      }
      const i = (y * W + x) * 4;
      // SOFT EDGES, and a shading map instead of an alpha punch.
      //
      // The holes used to be cut with alphaMap + alphaTest 0.5 on a double-sided sphere: a
      // hard binary cut, so every hole edge was a staircase of aliased pixels, and looking
      // through the holes at the far inside surface made the ball read as a speckled blob
      // rather than a ball. Shading them dark keeps the 26-hole look, costs no transparency,
      // and lets the sphere be lit normally.
      const edge = 0.012;
      const t = Math.min(1, Math.max(0, (best - (cosLimit - edge)) / edge));
      const v = Math.round(255 * (1 - 0.82 * t));
      img.data[i] = img.data[i + 1] = img.data[i + 2] = v;
      img.data[i + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  const tex = new THREE.CanvasTexture(cv);
  tex.wrapS = THREE.RepeatWrapping;
  tex.wrapT = THREE.ClampToEdgeWrapping;
  return tex;
}

/** The map tools/shotzone.ts writes, with everything the live recompute needs. */
export interface ZoneCellData {
  x_in: number;
  z_in: number;
  p: number;
  /** Why there is no shot here. See tools/shotzone.ts. */
  why?: 'behind' | 'tooFar' | 'tooNear' | 'noShot' | 'noRoom';
  k?: {
    lo: number; hi: number; sigma: number; pStay: number; halfLat: number;
    ux: number; uz: number; dist: number; commanded: number; cosEl: number;
  };
}

export interface ZonePayload {
  threshold: number;
  step_in: number;
  maxSpeed: number;
  yawScatterDeg: number;
  cells: ZoneCellData[];
  cellsTipped?: ZoneCellData[];
  /**
   * WHERE THE TAG CAN BE DECODED FROM (tools/tagmap.ts), keyed 'x,z' on the same grid. A spot
   * the physics loves is worth nothing if the robot cannot see the goal from it, and since
   * the target stopped being handed over for free that is a real second constraint. Absent on
   * an older map, in which case nothing is dimmed and the overlay behaves as it used to.
   */
  tagVisible?: Set<string>;
}

export class Scene {
  /** Measured score -> frequency, so the painted number means what the gate means. */
  private readonly zoneCal = loadLandCal();

  readonly scene = new THREE.Scene();
  readonly camera: THREE.PerspectiveCamera;
  readonly renderer: THREE.WebGLRenderer;
  cameraMode: CameraMode = 'orbit';
  showTrajectory = true;
  /** Draw the line of sight from the tag panel to the camera. */
  showSightLine = true;
  /** Draw the robot's believed mouth position beside the true one. */
  showBelief = true;
  /** Where the robot believes the mouth is, world metres. Null hides the marker. */
  private belief: Vec3 | null = null;

  /** Set the robot's believed mouth position. Nothing here derives it -- the brain does. */
  setBelief(p: Vec3 | null): void {
    this.belief = p;
  }

  private ballMeshes: THREE.Mesh[] = [];
  private rockers: THREE.Group[] = [];
  private robotGroup = new THREE.Group();
  /**
   * The OPPONENT, drawn plainly on purpose.
   *
   * Your robot is the CAD one, with its wheels and its hood and its intake roller turning.
   * The opponent is a coloured box with a nose and a turret stick, because the two have to be
   * instantly distinguishable at a glance while driving -- and because everything you need to
   * read off it is where it is, which way its intake points, and where its turret is aimed.
   */
  private opponentGroup: THREE.Group | null = null;
  private opponentTurret: THREE.Group | null = null;
  private opponentHood: THREE.Mesh | null = null;
  private opponentFlywheel: THREE.Mesh | null = null;
  private turretGroup = new THREE.Group();
  private hoodMesh!: THREE.Mesh;
  private wheelMeshes: THREE.Object3D[] = [];
  private intakeRoller!: THREE.Group;
  private flywheelMesh!: THREE.Mesh;
  private spin = { wheel: 0, intake: 0, fly: 0 };
  private trajLine: THREE.Mesh;
  /**
   * WHERE THE BALL ACTUALLY WENT, sampled from the live flight rather than integrated.
   *
   * The yellow curve is a PREDICTION -- what the solver says this launch should do. A trail
   * behind the ball in the air is the other half, and having both on screen at once is the
   * only way to see them disagree. When they lie on top of each other the model is right;
   * when they part company, the gap is the thing worth chasing (it is how the 14.5 cm the
   * muzzle-inside-the-chassis was costing showed up).
   */
  private ballTrail: THREE.Mesh;
  private trailPts: THREE.Vector3[] = [];
  /** The shot the drawn trail belongs to. A new shot clears it; a recycled ball does not. */
  private trailShot = -1;
  /** The convex boxes the physics actually uses. Hidden unless you ask for them. */
  private colliderMeshes: THREE.Object3D[] = [];
  /** Stand-in geometry, shown only until the CAD arrives (or if it never does). */
  private proceduralMeshes: THREE.Object3D[] = [];
  /** CAD geometry, once assets/field.glb has loaded. */
  private cadRockers: (THREE.Object3D | null)[] = [null, null];
  private orbit = { theta: -Math.PI / 2, phi: 1.0, dist: 6.5, target: new THREE.Vector3(0, 0.7, 0) };
  /**
   * Where the driver is LOOKING, relative to whatever the camera mode would otherwise show.
   * The right stick drives this, so "look behind me" is a thumb movement in every mode
   * instead of a mode change: in orbit it swings the orbit itself, and in the modes that ride
   * the robot it swings the eye around the robot, which is the one thing those modes could
   * not do. `phi` is shared with the orbit so the elevation reads the same everywhere.
   */
  viewYaw = 0;
  private aimMarker: THREE.Mesh;
  private beliefMarker!: THREE.Mesh;
  /** Where the ROBOT thinks the mouth is. Not the same object as the one above. */
  /**
   * Where the viewer is looking, degrees, in the same frame as the robot's heading.
   * Right-drag turns this; the UI feeds it to the brain so the robot follows the view.
   */

  constructor(
    private readonly canvas: HTMLCanvasElement,
    private readonly geom: FieldGeometry,
    ballSpecs: { id: number; r: number; kind: string }[],
    /** The same robot.json the physics builds from, so the picture cannot drift from it. */
    private readonly robotSpec: RobotSpec,
    /** Which side you are on. The opponent is drawn in the other alliance's colour. */
    private readonly alliance: 'red' | 'blue' = 'red',
  ) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.scene.background = new THREE.Color(0xd8dde3);
    this.scene.fog = new THREE.Fog(0xd8dde3, 26, 46);

    this.camera = new THREE.PerspectiveCamera(50, 1, 0.05, 100);
    this.camera.position.set(0, 3, 6);

    // Indoor lighting: a bright ceiling bounce plus two soft overheads. No coloured rims --
    // this is a gymnasium, and the shapes should read by their own colour.
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x8a9199, 1.5));
    const sun = new THREE.DirectionalLight(0xffffff, 1.35);
    sun.position.set(6, 12, 7);
    this.scene.add(sun);
    const fill = new THREE.DirectionalLight(0xffffff, 0.55);
    fill.position.set(-7, 8, -6);
    this.scene.add(fill);

    this.buildRoom();
    this.buildField();
    this.buildHives();
    this.buildRobot();

    // One material per ball kind, so the colour code is unmistakable:
    // POLLEN yellow, red NECTAR red, blue NECTAR blue.
    const holes = holedBallTexture();
    const ballMat: Record<string, THREE.Material> = {};
    for (const [kind, colour] of [['pollen', COL.pollen], ['nectarRed', COL.red], ['nectarBlue', COL.blue]] as const) {
      ballMat[kind] = new THREE.MeshStandardMaterial({
        color: colour, roughness: 0.38, metalness: 0.0,
        emissive: colour, emissiveIntensity: kind === 'pollen' ? 0.10 : 0.16,
        // The hole pattern shades the surface instead of cutting it away: opaque, single
        // sided, properly lit. Transparency on a 35 mm ball at this distance was noise.
        map: holes,
      });
    }
    for (const b of ballSpecs) {
      // 32x24 rather than 20x14: at 35 mm across, a coarse sphere reads as a faceted lump the
      // moment it is anywhere near the camera, and the triangles are free at this count.
      const m = new THREE.Mesh(new THREE.SphereGeometry(b.r, 32, 24), ballMat[b.kind] ?? ballMat.pollen);
      this.ballMeshes.push(m);
      this.scene.add(m);
    }

    // A TUBE, not a line.
    //
    // THREE.Line is one pixel wide and stays one pixel wide: WebGL ignores linewidth on every
    // desktop driver, so the arc was a hairline that aliased into dashes against the field and
    // vanished end-on. A tube is real geometry -- it has thickness at any zoom, it is lit, and
    // it reads as an arc in three dimensions instead of a scratch on the screen.
    this.trajLine = new THREE.Mesh(
      new THREE.BufferGeometry(),
      new THREE.MeshStandardMaterial({
        color: 0xffd166, emissive: 0xffb020, emissiveIntensity: 0.55,
        roughness: 0.5, transparent: true, opacity: 0.9,
      }),
    );
    this.trajLine.renderOrder = 3;
    this.scene.add(this.trajLine);

    // The flown path, in a cooler colour so it cannot be mistaken for the prediction.
    this.ballTrail = new THREE.Mesh(
      new THREE.BufferGeometry(),
      new THREE.MeshStandardMaterial({
        color: 0x5ad1ff, emissive: 0x2aa8e0, emissiveIntensity: 0.6,
        roughness: 0.5, transparent: true, opacity: 0.95,
      }),
    );
    this.ballTrail.renderOrder = 4;
    this.ballTrail.visible = false;
    this.scene.add(this.ballTrail);

    // THE LINE OF SIGHT, tag panel to camera.
    //
    // The single most confusing thing about the camera is that "it cannot see it" is
    // invisible: the robot just sits there sweeping, or aims on odometry, and nothing on
    // screen says why. This draws the ray the pipeline is trying to decode along -- GREEN
    // while it is decoding, RED while the geometry refuses -- so the grazing limit and the
    // rocker swinging become things you watch rather than things you read about.
    this.sightLine = new THREE.Mesh(
      new THREE.BufferGeometry(),
      new THREE.MeshBasicMaterial({ color: 0x4faa63, transparent: true, opacity: 0.5, depthWrite: false }),
    );
    this.sightLine.renderOrder = 3;
    this.sightLine.visible = false;
    this.scene.add(this.sightLine);

    this.aimMarker = new THREE.Mesh(
      new THREE.RingGeometry(0.06, 0.09, 24),
      new THREE.MeshBasicMaterial({ color: 0xffd166, side: THREE.DoubleSide, transparent: true, opacity: 0.9 }),
    );
    this.scene.add(this.aimMarker);

    // WHERE THE ROBOT THINKS THE MOUTH IS.
    //
    // The ring above is drawn at the TRUE mouth, straight off the physics, so it does not
    // move when the robot is wrong -- and nothing on screen showed the robot's own belief
    // at all. The estimation error had to be inferred from a near miss.
    //
    // This one is the belief the shooter is aiming with: the tag when it can see one,
    // odometry plus the surveyed geometry when it cannot. THE GAP BETWEEN THE TWO RINGS IS
    // THE ERROR, drawn to scale. Violet, so it is neither the yellow prediction nor the
    // blue flown path.
    this.beliefMarker = new THREE.Mesh(
      new THREE.RingGeometry(0.10, 0.125, 24),
      new THREE.MeshBasicMaterial({ color: 0xc77dff, side: THREE.DoubleSide, transparent: true, opacity: 0.85 }),
    );
    this.beliefMarker.renderOrder = 3;
    this.beliefMarker.visible = false;
    this.scene.add(this.beliefMarker);

    // WHERE THE ROBOT THINKS THE MOUTH IS.
    //
    // The ring above is drawn at the TRUE mouth, straight off the physics, so it does not
    // move when the robot is wrong -- and nothing on screen showed the robot's own belief at
    // all. The estimation error had to be inferred from a near miss, or read as the gap
    // between two numbers in a panel.
    //
    // This one is the belief: the target estimate the shooter is actually aiming with, which
    // comes from the tag when it can see one and from odometry plus the surveyed geometry
    // when it cannot. The DISTANCE BETWEEN THE TWO RINGS is the error, drawn to scale.
    // Violet, because it is neither the yellow prediction nor the blue flown path.
    this.beliefMarker = new THREE.Mesh(
      new THREE.RingGeometry(0.10, 0.125, 24),
      new THREE.MeshBasicMaterial({ color: 0xc77dff, side: THREE.DoubleSide, transparent: true, opacity: 0.85 }),
    );
    this.beliefMarker.renderOrder = 3;
    this.beliefMarker.visible = false;
    this.scene.add(this.beliefMarker);

    this.applyVisibility();
    this.loadCad();
    this.attachControls();
  }

  private static warnedNoCad = false;
  private colliders = false;
  private cadStatics: THREE.Object3D[] = [];
  private cadLoaded = false;
  private sightLine!: THREE.Mesh;
  /** The patrol sector on the floor: where the driver should stay, not where a shot is legal. */
  private patrolMesh: THREE.Mesh | null = null;
  private patrolKey = '';
  showPatrolSector = false;
  private zoneMesh: THREE.Mesh | null = null;
  private zone: ZonePayload | null = null;
  private zoneSide: 0 | 1 = 0;
  /** Whose hive the map was computed for. tools/shotzone.ts builds it for red. */
  private zoneAlliance: 'red' | 'blue' = 'red';
  /** Velocity the painted texture was computed at, so it is only redone when it matters. */
  private zoneVel: [number, number] = [0, 0];

  /**
   * Paint the field with where a shot is worth taking (tools/shotzone.ts).
   *
   * One textured plane a centimetre off the tiles, not a mesh per square: the map is a
   * picture, and a picture is a texture.
   *
   * It is a MODEL map -- it says where the physics is forgiving for a perfectly aimed shot,
   * which is not the same as where this robot has been measured hitting anything.
   */
  setShotZone(payload: ZonePayload): void {
    this.zone = payload;
    if (this.zoneMesh) {
      this.scene.remove(this.zoneMesh);
      (this.zoneMesh.material as THREE.MeshBasicMaterial).map?.dispose();
      this.zoneMesh.geometry.dispose();
      this.zoneMesh = null;
    }
    if (!payload.cells?.length) return;
    const mesh = new THREE.Mesh(
      new THREE.PlaneGeometry(2 * this.geom.halfWidth_m, 2 * this.geom.halfWidth_m),
      new THREE.MeshBasicMaterial({ transparent: true, depthWrite: false }),
    );
    mesh.rotation.x = -Math.PI / 2;
    mesh.position.y = 0.01;
    mesh.renderOrder = 2;
    mesh.visible = false;
    this.zoneMesh = mesh;
    this.scene.add(mesh);
    this.repaintZone([0, 0]);
  }

  /**
   * THE PATROL SECTOR, painted on the tiles: the wedge either side of the up CELL's opening
   * that the driver should stay inside.
   *
   * It is NOT the fire gate. `turret.fireOpenCap_deg` decides whether a shot is allowed and is
   * a different, wider number. This is where a shot is still worth TAKING, and the two differ
   * because the far edge of the legal sector is also the end of a pass: the mouth's usable
   * width falls with the cosine of the bearing while the robot is simultaneously reversing, so
   * the lead is solving for a velocity the ball will not have. Measured with every gate lifted,
   * 260 shots: 94% landed at 0-20 deg off the opening, 100% at 20-35, 89% at 35-50, and 8% at
   * 50-65 (tools/releasecheck.ts --why).
   *
   * Drawn from the UP CELL's mouth and re-aimed every frame, because a TIP swaps which CELL is
   * up and the new one opens the other way -- a sector painted once would be pointing at a
   * pocket that is no longer there.
   */
  setPatrolSector(
    mouth: Vec3 | null,
    normal: Vec3 | null,
    halfAngleDeg: number,
    innerRadius_m: number,
    outerRadius_m: number,
  ): void {
    if (!this.showPatrolSector || !mouth || !normal || !(halfAngleDeg > 0)) {
      if (this.patrolMesh) this.patrolMesh.visible = false;
      return;
    }
    const half = (halfAngleDeg * Math.PI) / 180;
    // Geometry only depends on the angle and the radii, so it is rebuilt when those change and
    // not when the rocker tips -- the tip is a rotation, and a rotation is free.
    const key = `${halfAngleDeg}|${innerRadius_m.toFixed(3)}|${outerRadius_m.toFixed(3)}`;
    if (!this.patrolMesh || this.patrolKey !== key) {
      if (this.patrolMesh) {
        this.scene.remove(this.patrolMesh);
        this.patrolMesh.geometry.dispose();
        (this.patrolMesh.material as THREE.Material).dispose();
      }
      // Centred on the ring's own +X, so aiming it is one rotation about Y below.
      const geo = new THREE.RingGeometry(innerRadius_m, outerRadius_m, 64, 1, -half, 2 * half);
      const mat = new THREE.MeshBasicMaterial({
        color: 0x36a2c9, transparent: true, opacity: 0.16,
        depthWrite: false, side: THREE.DoubleSide,
      });
      const mesh = new THREE.Mesh(geo, mat);
      // Ry(alpha) * Rx(-90): lay the ring flat, then swing its centre onto the mouth normal.
      mesh.rotation.order = 'YXZ';
      mesh.rotation.x = -Math.PI / 2;
      // Above the shot-zone plane at 0.01 so the two do not fight for the same pixels.
      mesh.position.y = 0.013;
      mesh.renderOrder = 3;
      this.patrolMesh = mesh;
      this.patrolKey = key;
      this.scene.add(mesh);
    }
    const n = Math.hypot(normal[0], normal[2]) || 1;
    const nx = normal[0] / n;
    const nz = normal[2] / n;
    this.patrolMesh.position.x = mouth[0];
    this.patrolMesh.position.z = mouth[2];
    // Rx(-90) leaves local +X on world +X; Ry(alpha) then sends it to (cos a, 0, -sin a).
    this.patrolMesh.rotation.y = Math.atan2(-nz, nx);
    this.patrolMesh.visible = true;
  }

  /**
   * The ray the tag pipeline is trying to decode along: panel -> camera.
   *
   * GREEN while it decodes, RED while the geometry refuses. This is the only thing on screen
   * that makes the camera's limits visible -- without it, "blind" and "looking the wrong way"
   * and "the rocker is mid-swing" all look identical, which is a robot standing still for no
   * stated reason.
   *
   * ponytail: drawn from the robot's tracked point at muzzle height, because that is where
   * the MODEL puts the camera -- there is no mount offset in tagCamera.ts, so inventing one
   * here would draw a lie. Give the camera a real mount and this should follow it.
   */
  setSightLine(from: Vec3 | null, to: Vec3 | null, decoding: boolean): void {
    if (!this.showSightLine || !from || !to) {
      this.sightLine.visible = false;
      return;
    }
    const a = new THREE.Vector3(from[0], from[1], from[2]);
    const b = new THREE.Vector3(to[0], to[1], to[2]);
    if (a.distanceTo(b) < 1e-3) {
      this.sightLine.visible = false;
      return;
    }
    this.sightLine.geometry.dispose();
    this.sightLine.geometry = new THREE.TubeGeometry(new THREE.LineCurve3(a, b), 1, decoding ? 0.006 : 0.003, 6, false);
    const mat = this.sightLine.material as THREE.MeshBasicMaterial;
    mat.color.setHex(decoding ? 0x4faa63 : 0xe0453c);
    mat.opacity = decoding ? 0.55 : 0.28;
    this.sightLine.visible = true;
  }

  /**
   * THE MAP MOVES WITH THE ROBOT, which is the whole reason the cells carry parameters
   * rather than a finished probability.
   *
   * The lead keeps a shot's ground path the same whatever the robot is doing, so the
   * aperture, the speed band and the entry rate belong to the SPOT and were solved once.
   * What driving changes is the speed the shot has to leave at -- retreating needs a faster
   * one -- and since launch scatter is a fraction of exit speed, the error at the mouth
   * grows with it. That is arithmetic per cell, so it can run every frame.
   */
  private repaintZone(vel: [number, number]): void {
    const z = this.zone;
    const mesh = this.zoneMesh;
    if (!z || !mesh) return;
    this.zoneVel = vel;
    const cells = (this.zoneSide === 1 ? z.cellsTipped : z.cells) ?? z.cells;
    const hw = this.geom.halfWidth_m;
    const px = 512;
    const cv = document.createElement('canvas');
    cv.width = px;
    cv.height = px;
    const g = cv.getContext('2d');
    if (!g) return;
    g.clearRect(0, 0, px, px);

    const sigmaYaw = Math.tan((z.yawScatterDeg * Math.PI) / 180);
    const half = (z.step_in * 0.0254) / 2;
    const w = Math.max(2, ((half * 2) / (2 * hw)) * px);
    const toPx = (m: number) => ((m + hw) / (2 * hw)) * px;

    for (const c of cells) {
      const k = c.k;
      let p = 0;
      if (k) {
        const horiz = k.commanded * k.cosEl;
        const required = Math.hypot(horiz * k.ux - vel[0], horiz * k.uz - vel[1]) / k.cosEl;
        if (required <= z.maxSpeed) {
          const sigma = k.sigma * (required / Math.max(k.commanded, 1e-6));
          const speed = pThread(k.lo, k.hi, k.commanded, sigma);
          const aim = pThread(-k.halfLat, k.halfLat, 0, k.dist * sigmaYaw);
          p = this.zoneCal ? this.zoneCal.apply(speed * aim * k.pStay) : speed * aim * k.pStay;
        }
      }
      // A RAMP, not three buckets. The calibration curve is a step function fitted to bins,
      // so thresholding it turned neighbouring squares that differ by a percent into a
      // red/green checkerboard -- the map looked like confetti rather than a place to stand.
      const t = Math.min(1, p / Math.max(z.threshold, 1e-6));
      // THREE DIFFERENT DEAD SQUARES, THREE DIFFERENT COLOURS. They were all one red, so
      // "the goal does not open this way" -- which is most of the field, and correct -- read
      // as a broken map. Grey means turn round; the reds mean move.
      g.fillStyle = p > 0
        ? `hsla(${(8 + 124 * t).toFixed(0)}, 72%, 46%, ${(0.20 + 0.42 * t).toFixed(3)})`
        : c.why === 'noRoom'
          // A PALE WASH, not near-black. This was rgb(20,22,26), which is within a few counts
          // of the tile colour underneath it -- so the band was painted, measurably, and
          // still read as "the map stops here". A dead square has to look MARKED, not dark.
          ? 'rgba(168, 176, 190, 0.30)'           // the chassis does not fit this close to a wall
          : c.why === 'behind'
            ? 'rgba(70, 78, 90, 0.26)'            // the CELL does not open this way
            : c.why === 'tooNear'
              ? 'rgba(150, 96, 24, 0.22)'         // inside the table's closest row
              : 'rgba(140, 32, 32, 0.20)';        // too far, or no launch fits
      g.fillRect(toPx(c.x_in * 0.0254) - w / 2, toPx(c.z_in * 0.0254) - w / 2, w, w);
    }

    const tex = new THREE.CanvasTexture(cv);
    tex.colorSpace = THREE.SRGBColorSpace;
    const mat = mesh.material as THREE.MeshBasicMaterial;
    mat.map?.dispose();
    mat.map = tex;
    mat.needsUpdate = true;
  }

  /**
   * How many squares are in each state right now, for the legend. Counted at the CURRENT
   * velocity, so the green number falls as you drive -- which is what the map is for and
   * what a static picture cannot say.
   */
  zoneTally(): { good: number; tooNear: number; tooFar: number; behind: number; noRoom: number } {
    const out = { good: 0, tooNear: 0, tooFar: 0, behind: 0, noRoom: 0 };
    const z = this.zone;
    if (!z) return out;
    const cells = (this.zoneSide === 1 ? z.cellsTipped : z.cells) ?? z.cells;
    const v = this.zoneVel;
    const sigmaYaw = Math.tan((z.yawScatterDeg * Math.PI) / 180);
    for (const c of cells) {
      const k = c.k;
      if (!k) { out[c.why === 'tooNear' ? 'tooNear' : c.why === 'behind' ? 'behind' : c.why === 'noRoom' ? 'noRoom' : 'tooFar']++; continue; }
      const horiz = k.commanded * k.cosEl;
      const required = Math.hypot(horiz * k.ux - v[0], horiz * k.uz - v[1]) / k.cosEl;
      let p = 0;
      if (required <= z.maxSpeed) {
        const sigma = k.sigma * (required / Math.max(k.commanded, 1e-6));
        const raw = pThread(k.lo, k.hi, k.commanded, sigma) * pThread(-k.halfLat, k.halfLat, 0, k.dist * sigmaYaw) * k.pStay;
        p = this.zoneCal ? this.zoneCal.apply(raw) : raw;
      }
      if (p >= z.threshold) out.good++;
      else out.tooFar++;
    }
    return out;
  }


  /**
   * Follow the rocker. A TIP puts it on its other stop and the up CELL becomes the other
   * one, so the mouth swings across the pivot and the zone goes with it. An overlay that did
   * not switch would send the driver to the wrong half of the field for the rest of the match.
   */
  private setZoneSide(side: 0 | 1): void {
    if (side === this.zoneSide || !this.zoneMesh) return;
    this.zoneSide = side;
    this.repaintZone(this.zoneVel);
  }

  set showShotZone(on: boolean) {
    if (this.zoneMesh) this.zoneMesh.visible = on;
  }
  get showShotZone(): boolean {
    return this.zoneMesh?.visible ?? false;
  }

  /** Show the convex shapes the solver sees instead of the CAD skin. */
  set showColliders(on: boolean) {
    this.colliders = on;
    this.applyVisibility();
  }
  get showColliders(): boolean {
    return this.colliders;
  }

  /**
   * Three layers occupy the same space, so exactly one is shown at a time:
   * the CAD skin, the procedural stand-in (before field.glb loads), and the collision boxes.
   */
  private applyVisibility(): void {
    const showCad = this.cadLoaded && !this.colliders;
    for (const m of this.colliderMeshes) m.visible = this.colliders;
    for (const m of this.cadStatics) m.visible = showCad;
    for (const r of this.cadRockers) if (r) r.visible = showCad;
    for (const m of this.proceduralMeshes) m.visible = !showCad && !this.colliders;
  }

  /**
   * Load the tessellated STEP (tools/cad2assets.py). Until it arrives -- or if it is missing
   * because nobody has run the pipeline -- the procedural geometry stands in, so the app
   * still works from a clean checkout.
   */
  private loadCad(): void {
    // One fetch per page load, not per Scene: reset rebuilds the world and would otherwise
    // re-download the whole field every time.
    if (!fieldUrl) {
      // Not an error: the stand-in geometry is the documented fallback, and the console note
      // says how to get the real thing.
      if (!Scene.warnedNoCad) {
        Scene.warnedNoCad = true;
        console.info('assets/field.glb not built — showing the procedural field. Run `python tools/cad2assets.py` for the CAD one.');
      }
      return;
    }
    if (!Scene.cadPromise) {
      Scene.cadPromise = new Promise((resolve, reject) => new GLTFLoader().load(fieldUrl, resolve, undefined, reject));
    }
    Scene.cadPromise.then(
      (loaded) => {
        const gltf = { scene: loaded.scene.clone(true) };
        // The mesh carries a colour per vertex, assigned per part by tools/cad2assets.py,
        // so one material serves every part and the alliance panels, aluminium and AprilTag
        // panels all come out right.
        const cad = new THREE.MeshStandardMaterial({ vertexColors: true, metalness: 0.35, roughness: 0.5, side: THREE.DoubleSide });
        const glass = new THREE.MeshStandardMaterial({ color: 0x9fd8f5, metalness: 0.05, roughness: 0.12, transparent: true, opacity: 0.16, side: THREE.DoubleSide, depthWrite: false });

        // Collect first, THEN reparent: re-parenting inside traverse() mutates the very
        // children arrays it is walking, which silently skips half the meshes.
        const meshes: THREE.Mesh[] = [];
        gltf.scene.traverse((o) => {
          if (o instanceof THREE.Mesh) meshes.push(o);
        });

        for (const o of meshes) {
          const name = (o.name || o.parent?.name || '').toLowerCase();
          if (name.includes('rocker')) {
            const red = name.includes('red');
            const i = red ? 0 : 1;
            const pivotX = red ? this.geom.hiveX_m.red : this.geom.hiveX_m.blue;
            // The STEP holds the rocker at its CAD rest angle in world coordinates. Move the
            // pivot to the origin, then undo that rest angle, and what is left is the body
            // frame the live joint angle is applied to.
            const inner = new THREE.Group();
            o.position.set(-pivotX, -this.geom.pivotY_m, 0);
            o.material = cad;
            inner.add(o);
            inner.rotation.x = red ? this.geom.restAngle_rad : -this.geom.restAngle_rad;
            this.rockers[i]?.add(inner);
            this.cadRockers[i] = inner;
          } else {
            o.material = name.includes('perimeter') ? glass : cad;
            this.cadStatics.push(o);
            this.scene.add(o);
          }
        }

        this.cadLoaded = true;
        this.applyVisibility();
      },
      () => {
        // No field.glb: keep the procedural stand-in and say so once.
        console.warn('assets/field.glb unreadable — run `python tools/cad2assets.py` to regenerate it');
      },
    );
  }

  /** Shared across Scene instances so a reset does not re-fetch the field. */
  private static cadPromise: Promise<{ scene: THREE.Group }> | null = null;

  // ---------------------------------------------------------------- build

  private boxMesh(p: BoxPiece, mat: THREE.Material): THREE.Mesh {
    const m = new THREE.Mesh(new THREE.BoxGeometry(p.half[0] * 2, p.half[1] * 2, p.half[2] * 2), mat);
    m.position.set(p.pos[0], p.pos[1], p.pos[2]);
    m.rotation.x = p.rotX;
    return m;
  }

  /**
   * The room the field stands in.
   *
   * Floor only, deliberately. Walls gave the field a place to be but they also boxed the
   * camera in: from any low angle you were looking at grey plasterboard instead of the
   * robot. So the room is a floor that runs past the field on every side, a few landmarks
   * to judge heading against, and open air above -- which reads as a venue and never gets
   * between you and the shot.
   */
  private buildRoom(): void {
    const hw = this.geom.halfWidth_m;
    const R = hw + 4.2;
    const room = new THREE.Group();

    // 60 cm commercial tile, two tones, instanced: 2 draw calls for the whole floor.
    const t = 0.6;
    const n = Math.ceil(R / t);
    const quad = new THREE.PlaneGeometry(t * 0.985, t * 0.985);
    const tileA = new THREE.InstancedMesh(quad, new THREE.MeshStandardMaterial({ color: COL.roomFloor, roughness: 0.75 }), n * n * 4);
    const tileB = new THREE.InstancedMesh(quad, new THREE.MeshStandardMaterial({ color: COL.roomFloorAlt, roughness: 0.75 }), n * n * 4);
    const m4 = new THREE.Matrix4();
    const rx = new THREE.Matrix4().makeRotationX(-Math.PI / 2);
    let ia = 0;
    let ib = 0;
    for (let i = -n; i < n; i++) {
      for (let j = -n; j < n; j++) {
        m4.copy(rx).setPosition(t * (i + 0.5), -0.02, t * (j + 0.5));
        if ((i + j) & 1) tileB.setMatrixAt(ib++, m4);
        else tileA.setMatrixAt(ia++, m4);
      }
    }
    tileA.count = ia;
    tileB.count = ib;
    room.add(tileA, tileB);

    // Two landmarks on the audience side, low enough to stay out of the shot: the scoring
    // table and a banner board behind it. Without something asymmetric out there, the
    // Driver camera gives you no way to tell which end of the field you are facing.
    const board = new THREE.Mesh(
      new THREE.BoxGeometry(3.4, 0.9, 0.06),
      new THREE.MeshStandardMaterial({ color: 0x232830, roughness: 0.65 }),
    );
    board.position.set(0, 0.95, -(hw + 2.6));
    room.add(board);
    for (const sx of [-1, 1]) {
      const post = new THREE.Mesh(new THREE.BoxGeometry(0.05, 1.4, 0.05), new THREE.MeshStandardMaterial({ color: 0x6d757e, metalness: 0.4, roughness: 0.5 }));
      post.position.set(sx * 1.6, 0.7, -(hw + 2.6));
      room.add(post);
    }
    room.add(this.table(0, -(hw + 1.5), 2.4));

    this.scene.add(room);
  }

  /** A folding table: top plus four legs. One landmark, not a furniture showroom. */
  private table(x: number, z: number, len: number, ry = 0): THREE.Group {
    const g = new THREE.Group();
    const topMat = new THREE.MeshStandardMaterial({ color: 0xb9c0c8, roughness: 0.6 });
    const legMat = new THREE.MeshStandardMaterial({ color: 0x6d757e, roughness: 0.5, metalness: 0.4 });
    const top = new THREE.Mesh(new THREE.BoxGeometry(len, 0.04, 0.75), topMat);
    top.position.y = 0.74;
    g.add(top);
    for (const sx of [-1, 1]) {
      for (const sz of [-1, 1]) {
        const leg = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.72, 0.04), legMat);
        leg.position.set(sx * (len / 2 - 0.08), 0.36, sz * 0.31);
        g.add(leg);
      }
    }
    g.position.set(x, 0, z);
    g.rotation.y = ry;
    return g;
  }

  private buildField(): void {
    const hw = this.geom.halfWidth_m;
    const tile = inches(23.5);
    const n = Math.round((hw * 2) / tile);
    const tiles = new THREE.Group();
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < n; j++) {
        const mat = new THREE.MeshStandardMaterial({ color: (i + j) % 2 ? COL.tile : COL.tileAlt, roughness: 0.95 });
        const m = new THREE.Mesh(new THREE.BoxGeometry(tile * 0.985, 0.015, tile * 0.985), mat);
        m.position.set(-hw + tile * (i + 0.5), -0.0075, -hw + tile * (j + 0.5));
        tiles.add(m);
      }
    }
    this.scene.add(tiles);

    // perimeter
    // THE PERIMETER, VISIBLE. It was pale blue at 0.22 opacity over a teal room floor, which is
    // the same argument as no wall at all: you could not tell where the field ended, and a ball
    // that stopped against it looked like a ball that stopped in mid air. Dark grey and mostly
    // opaque reads as a boundary from every angle; a solid rail caps it so the top edge is a
    // line rather than a fade.
    const wallMat = new THREE.MeshStandardMaterial({ color: COL.wall, transparent: true, opacity: 0.55, roughness: 0.8, metalness: 0.05, side: THREE.DoubleSide });
    const railMat = new THREE.MeshStandardMaterial({ color: COL.rail, roughness: 0.6, metalness: 0.2 });
    const h = this.geom.railTopY_m;
    for (const [dx, dz] of [[1, 0], [-1, 0], [0, 1], [0, -1]] as const) {
      const g = new THREE.PlaneGeometry(hw * 2, h);
      const m = new THREE.Mesh(g, wallMat);
      m.position.set(dx * hw, h / 2, dz * hw);
      m.rotation.y = dx !== 0 ? Math.PI / 2 : 0;
      this.scene.add(m);
      const rail = new THREE.Mesh(new THREE.BoxGeometry(dx !== 0 ? 0.04 : hw * 2 + 0.04, 0.05, dx !== 0 ? hw * 2 + 0.04 : 0.04), railMat);
      rail.position.set(dx * hw, h, dz * hw);
      this.scene.add(rail);
      const kick = new THREE.Mesh(new THREE.BoxGeometry(dx !== 0 ? 0.03 : hw * 2, 0.10, dx !== 0 ? hw * 2 : 0.03), railMat);
      kick.position.set(dx * hw, 0.05, dz * hw);
      this.scene.add(kick);
    }

    // The A-frame as the CAD actually builds it: four legs splaying from foot bars at the
    // +-X walls up to two top corners, and the top bar between them. `geom.frame` is the
    // physics approximation (posts and panels); this is what it looks like.
    const frameMat = new THREE.MeshStandardMaterial({ color: COL.frame, roughness: 0.55, metalness: 0.45 });
    const stand = (m: THREE.Object3D) => { this.proceduralMeshes.push(m); this.scene.add(m); return m; };
    const footY = inches(0.6);
    const topY = inches(41.45);
    const footX = inches(24.08);
    const footZ = inches(18.27);
    const cornerX = inches(12.61);
    for (const sx of [-1, 1]) {
      for (const sz of [-1, 1]) {
        stand(bar([sx * footX, footY, sz * footZ], [sx * cornerX, topY, 0], inches(1.1), frameMat));
      }
      // foot bar along Z at each end
      const foot = new THREE.Mesh(new THREE.BoxGeometry(inches(2), inches(2.1), inches(38.9)), frameMat);
      foot.position.set(sx * footX, footY, 0);
      stand(foot);
      const corner = new THREE.Mesh(new THREE.BoxGeometry(inches(3.3), inches(4.7), inches(3)), frameMat);
      corner.position.set(sx * cornerX, inches(41.26), 0);
      stand(corner);
    }
    const topBar = new THREE.Mesh(new THREE.BoxGeometry(inches(24), inches(1), inches(1)), frameMat);
    topBar.position.set(0, topY, 0);
    stand(topBar);

    // FLOWERs, built the way am-5855 actually is: four vertical HIPS pipes standing on a
    // base plate, three layer plates threaded onto them (cad-summary ring_y_in), and the
    // backstop disc on top. It is a CAGE, not a tube -- you can see the ball inside it,
    // which is the whole reason the real part is made of pipes.
    const pipeMat = new THREE.MeshStandardMaterial({ color: 0xf2f5f8, roughness: 0.45 });
    const plateMat = new THREE.MeshStandardMaterial({ color: COL.frame, roughness: 0.5, metalness: 0.4 });
    const bandMat = new THREE.MeshBasicMaterial({ color: COL.flower, transparent: true, opacity: 0.11, side: THREE.DoubleSide, depthWrite: false });
    for (const f of this.geom.flowers) {
      const fg = new THREE.Group();
      fg.position.set(f.x_m, 0, f.z_m);
      const pipeR = inches(0.5);
      const ringR = f.openingR_m + pipeR;   // pipe centres sit just outside the opening
      for (let k = 0; k < 4; k++) {
        const a = (k / 4) * Math.PI * 2 + Math.PI / 4;
        const pipe = new THREE.Mesh(new THREE.CylinderGeometry(pipeR, pipeR, f.topY_m, 10), pipeMat);
        pipe.position.set(Math.cos(a) * ringR, f.topY_m / 2, Math.sin(a) * ringR);
        fg.add(pipe);
      }
      // The three layer plates: annular, so the 4 in throat stays open all the way down.
      for (const y of [f.scoreLow_m - inches(4.14), f.scoreLow_m, f.scoreHigh_m]) {
        const plate = new THREE.Mesh(new THREE.RingGeometry(f.openingR_m, ringR + pipeR, 24), plateMat);
        plate.rotation.x = -Math.PI / 2;
        plate.position.y = y;
        plate.material.side = THREE.DoubleSide;
        fg.add(plate);
      }
      // Backstop disc, closed: this is what a ball entering from above bounces off.
      const back = new THREE.Mesh(new THREE.CylinderGeometry(ringR + pipeR, ringR + pipeR, inches(0.25), 24), plateMat);
      back.position.y = f.topY_m + inches(0.12);
      fg.add(back);
      stand(fg);

      // The scoring band, drawn even with the CAD showing: a ball between these two plates
      // is scored, and you cannot see that from the structure alone.
      const band = new THREE.Mesh(
        new THREE.CylinderGeometry(f.openingR_m * 0.99, f.openingR_m * 0.99, f.scoreHigh_m - f.scoreLow_m, 20, 1, true),
        bandMat,
      );
      band.position.set(f.x_m, (f.scoreLow_m + f.scoreHigh_m) / 2, f.z_m);
      this.scene.add(band);
      // The two boundaries themselves. A tinted column alone is ambiguous about where it
      // starts and stops, and where it stops is the difference between scored and not.
      for (const y of [f.scoreLow_m, f.scoreHigh_m]) {
        const ring = new THREE.Mesh(
          new THREE.TorusGeometry(f.openingR_m * 1.02, 0.004, 6, 24),
          new THREE.MeshBasicMaterial({ color: COL.flower }),
        );
        ring.rotation.x = Math.PI / 2;
        ring.position.set(f.x_m, y, f.z_m);
        this.scene.add(ring);
      }
    }

    // zones
    for (const z of this.geom.zones) {
      const g = new THREE.PlaneGeometry(z.max[0] - z.min[0], z.max[2] - z.min[2]);
      const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color: z.alliance === 'red' ? COL.red : COL.blue, transparent: true, opacity: z.name === 'LOADING' ? 0.18 : 0.30, side: THREE.DoubleSide }));
      m.rotation.x = -Math.PI / 2;
      m.position.set((z.min[0] + z.max[0]) / 2, 0.004, (z.min[2] + z.max[2]) / 2);
      this.scene.add(m);
      void COL.zone;
    }
  }

  /**
   * The rocker, shaped like the CAD part rather than like its collision boxes.
   *
   * The pocket walls ARE the colliders (so what you see still collides), but the pentagon
   * end ribs, the basket arm out to the pivot, the churro bracing across the mouth and the
   * AprilTag panel are the real am-5853 parts, sized from cad/parts.json. Render only --
   * none of it is in the physics, which is why it can be shaped honestly.
   */
  private buildHives(): void {
    const g0 = this.geom;
    for (const [i, alliance] of (['red', 'blue'] as const).entries()) {
      const g = new THREE.Group();
      g.position.set(alliance === 'red' ? g0.hiveX_m.red : g0.hiveX_m.blue, g0.pivotY_m, 0);
      const colour = alliance === 'red' ? COL.red : COL.blue;

      const skin = new THREE.MeshStandardMaterial({
        color: colour, roughness: 0.45, metalness: 0.1,
        transparent: true, opacity: 0.55, side: THREE.DoubleSide,
      });
      const ribMat = new THREE.MeshStandardMaterial({ color: colour, roughness: 0.4, metalness: 0.15, side: THREE.DoubleSide });
      const alu = new THREE.MeshStandardMaterial({ color: 0xcbd5e1, metalness: 0.75, roughness: 0.3 });

      const wire = new THREE.MeshBasicMaterial({ color: 0x67e8f9, wireframe: true });
      for (const cell of g0.cells) {
        // the collision shell: what the solver actually sees
        for (const piece of cell.pieces) {
          const solid = this.boxMesh(piece, skin);
          g.add(solid);
          this.proceduralMeshes.push(solid);
          const box = this.boxMesh(piece, wire);
          g.add(box);
          this.colliderMeshes.push(box);
        }

        /** pocket-local (x, u, t) -> rocker body frame */
        const at = (x: number, u: number, t: number): Vec3 => fromCellLocal(cell, x, u, t);
        const hu = cell.halfInterior[1];
        const ht = cell.halfInterior[2];
        const hx = cell.halfInterior[0];

        // Pentagon end ribs (am-5866), one at each end of the 20 in width.
        const profile = new THREE.Shape();
        const pts: [number, number][] = [
          [hu, ht], [hu, -ht], [-hu * 0.45, -ht], [-hu, -ht * 0.55], [-hu, ht * 0.55], [-hu * 0.45, ht],
        ];
        profile.moveTo(pts[0][0], pts[0][1]);
        for (const [u, t] of pts.slice(1)) profile.lineTo(u, t);
        profile.closePath();
        for (const sx of [-1, 1]) {
          const rib = new THREE.Mesh(new THREE.ExtrudeGeometry(profile, { depth: inches(0.25), bevelEnabled: false }), ribMat);
          // the shape lives in (u, t); stand it up and slide it to the end of the pocket
          rib.rotation.set(0, Math.PI / 2, 0);
          rib.position.set(sx * hx, 0, 0);
          const holder = new THREE.Group();
          holder.add(rib);
          const hub = fromCellLocal(cell, 0, 0, 0);
          holder.position.set(0, hub[1], hub[2]);
          holder.rotation.x = cell.axisAngle_rad;
          g.add(holder);
          this.proceduralMeshes.push(holder);
        }

        // Basket base tube (am-5868): the arm from the pivot out to the pocket.
        const arm = bar([0, 0, 0], at(0, -hu + inches(1), 0), inches(1), alu);
        g.add(arm);
        this.proceduralMeshes.push(arm);

        // Two 10.5 in churros (am-5867) bracing the mouth, one at each end.
        for (const sx of [-1, 1]) {
          const ch = bar(at(sx * hx * 0.92, hu * 0.55, -ht * 0.8), at(sx * hx * 0.92, -hu * 0.1, ht * 0.9), inches(0.35), alu);
          g.add(ch);
          this.proceduralMeshes.push(ch);
        }

        // AprilTag panel (am-5888) on the underside, facing out of the mouth.
        const tag = new THREE.Mesh(
          new THREE.PlaneGeometry(inches(17), inches(4.3)),
          new THREE.MeshStandardMaterial({ color: 0xf8fafc, roughness: 0.9, side: THREE.DoubleSide }),
        );
        const tagPos = at(0, -hu + inches(0.4), -ht * 0.55);
        tag.position.set(tagPos[0], tagPos[1], tagPos[2]);
        tag.rotation.x = cell.axisAngle_rad + Math.PI / 2;
        g.add(tag);
        this.proceduralMeshes.push(tag);
      }

      // the pivot axle itself (am-5881 spacers ride on this line)
      const axle = new THREE.Mesh(new THREE.CylinderGeometry(inches(0.5), inches(0.5), inches(22), 14), alu);
      axle.rotation.z = Math.PI / 2;
      g.add(axle);
      this.proceduralMeshes.push(axle);

      this.scene.add(g);
      this.rockers[i] = g;
    }
  }

  /**
   * A robot rather than a box: mecanum wheels you can see turn, a roller intake across the
   * front with a visible mouth, a hopper you can see balls sitting in, and a turret carrying
   * the shooter with a barrel that is clearly the OUTtake. Render only -- the physics is
   * still one box plus the tyre model, which is what `robot.json` describes.
   */
  /** Chassis, a nose showing which way the intake faces, and a turret stick. */
  /**
   * THE OPPONENT IS THE SAME ROBOT, in the other alliance's colour.
   *
   * It used to be a box with a nose and a stick, on the reasoning that you only need to
   * see where it is. That is wrong for practice: most of reading an opponent is seeing
   * which way its INTAKE points (where it is about to go) and which way its TURRET points
   * (whether it is about to shoot), and a stick shows neither convincingly.
   */
  private buildOpponent(): void {
    const b = this.buildChassis(this.alliance === 'red' ? COL.blue : COL.red);
    this.opponentGroup = b.group;
    this.opponentTurret = b.turret;
    this.opponentHood = b.hood;
    this.opponentFlywheel = b.flywheel;
    this.scene.add(b.group);
  }

  /**
   * ONE ROBOT BUILDER, USED TWICE. The player had rails, panels, a real intake, a hood whose
   * angle IS the launch elevation and a flywheel you can watch turn; the opponent was a
   * coloured box with a nose and a stick. You cannot practise against a box -- half of
   * reading an opponent is seeing which way its intake and its turret point.
   *
   * So the model is built once and coloured twice. Everything the player has, the opponent
   * has, because it is the same geometry driven by the same snapshot fields.
   */
  private buildChassis(bodyColour: number): {
    group: THREE.Group; turret: THREE.Group; hood: THREE.Mesh;
    flywheel: THREE.Mesh; roller: THREE.Group; wheels: THREE.Object3D[];
  } {
    const _group = new THREE.Group();
    const _turret = new THREE.Group();
    let _hood!: THREE.Mesh;
    let _flywheel!: THREE.Mesh;
    let _roller!: THREE.Group;
    let _wheels: THREE.Object3D[] = [];
    // FROM THE CONFIG, WHICH IS FROM ROBOT_BUILD.md. These were three literals describing a
    // 16.9 in square box -- a placeholder that stopped matching the moment the build spec's
    // 15.5 x 17.5 x 11.6 in footprint went into config/robot.json. A model that does not
    // change with the robot is a picture, not a view of it.
    const rc = this.robotSpec.chassis;
    const c = { L: rc.length_m, W: rc.width_m, H: rc.height_m };
    const frame = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.5, roughness: 0.45 });
    const panel = new THREE.MeshStandardMaterial({ color: bodyColour, metalness: 0.2, roughness: 0.4, transparent: true, opacity: 0.35, side: THREE.DoubleSide });
    const rubber = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.9 });
    const roller = new THREE.MeshStandardMaterial({ color: 0xf59e0b, roughness: 0.7 });
    const steel = new THREE.MeshStandardMaterial({ color: 0xcbd5e1, metalness: 0.7, roughness: 0.3 });

    // chassis rails
    for (const sy of [-1, 1]) {
      for (const [w, d, ox, oz] of [[c.W, 0.03, 0, c.L / 2], [c.W, 0.03, 0, -c.L / 2], [0.03, c.L, c.W / 2, 0], [0.03, c.L, -c.W / 2, 0]] as const) {
        const rail = new THREE.Mesh(new THREE.BoxGeometry(w, 0.05, d), frame);
        rail.position.set(ox, (sy * c.H) / 2 - sy * 0.025, oz);
        _group.add(rail);
      }
    }
    // side panels so it reads as a body, not a cage
    for (const sx of [-1, 1]) {
      const side = new THREE.Mesh(new THREE.PlaneGeometry(c.L, c.H * 0.8), panel);
      side.position.set((sx * c.W) / 2, 0, 0);
      side.rotation.y = Math.PI / 2;
      _group.add(side);
    }

    // four mecanum wheels, with rollers at 45 deg so the direction reads
    _wheels = [];
    for (const sx of [-1, 1]) {
      for (const sz of [-1, 1]) {
        const hub = new THREE.Group();
        const wr = this.robotSpec.drivetrain.wheelRadius_m;
        const tyre = new THREE.Mesh(new THREE.CylinderGeometry(wr, wr, 0.038, 18), rubber);
        tyre.rotation.z = Math.PI / 2;
        hub.add(tyre);
        for (let i = 0; i < 8; i++) {
          const a = (i / 8) * Math.PI * 2;
          const r = new THREE.Mesh(new THREE.CapsuleGeometry(0.008, 0.028, 3, 6), steel);
          r.position.set(0, Math.cos(a) * 0.045, Math.sin(a) * 0.045);
          // rollers lie at +-45 deg, mirrored across the diagonals like a real mecanum set
          r.rotation.set(0, 0, Math.PI / 2);
          r.rotateOnAxis(new THREE.Vector3(1, 0, 0), a);
          r.rotateOnAxis(new THREE.Vector3(0, 1, 0), (sx * sz > 0 ? 1 : -1) * Math.PI / 4);
          hub.add(r);
        }
        // Axles where ROBOT_BUILD.md section 4.2 puts them: track 13.5 in across, wheelbase
        // 11.0 in along. The Z was the literal 0.33 m, which is neither.
        hub.position.set((sx * (c.W + 0.03)) / 2, -c.H / 2 + this.robotSpec.drivetrain.wheelRadius_m, (sz * this.robotSpec.drivetrain.wheelbase_m) / 2);
        _group.add(hub);
        _wheels.push(hub);
      }
    }

    // INTAKE: compliant wheels on a hex shaft, across a mouth you can see into.
    //
    // IT SPUN ABOUT THE WRONG AXIS. The cylinder was laid down with rotation.z = PI/2 and
    // then driven with rotation.y, and an Object3D's euler is XYZ, so the y term turned the
    // laid-down roller about the WORLD vertical: the axle swept round like a clock hand
    // instead of the roller turning on it. On screen the intake was a bar pivoting diagonally
    // out of the robot's front corner. The wheels a hundred lines up had it right all along --
    // lay the tyre over inside a hub and spin the HUB -- so this does the same, and there is
    // now nothing to get wrong twice.
    //
    // IT ALSO COULD NOT HAVE SHOWN THE SPIN if the axis had been right: a smooth 12-sided
    // cylinder of one colour looks identical at every angle. Compliant wheels with treads on
    // them are what a real over-the-bumper intake has, and they turn visibly.
    //
    // Sized from robot.json rather than from four hand-tuned numbers that happened to agree
    // with it. `stepIntake` sweeps a volume mouth.width x mouth.height x mouth.depth off the
    // front at bin-floor height, and the roller is drawn inside that volume, so a picture that
    // disagrees with the physics now needs the config to disagree with itself first.
    const ip = this.robotSpec.intake;
    const rollerR = ip.rollerRadius_m ?? 0.035;
    const mouthW = ip.mouth.width_m;
    const binFloorY = -c.H / 2 + 0.012;                  // robot.ts: -hh + t * 1.5
    const intake = new THREE.Group();
    _roller = new THREE.Group();
    // Laid over ONCE, here, where nothing animates it. update() turns the group about its
    // own X, which after this is the axle.
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, mouthW + 0.04, 6), steel);
    shaft.rotation.z = Math.PI / 2;
    _roller.add(shaft);
    const wheels = 5;
    for (let i = 0; i < wheels; i++) {
      const x = ((i - (wheels - 1) / 2) / Math.max(1, wheels - 1)) * (mouthW - 0.05);
      const w = new THREE.Mesh(new THREE.CylinderGeometry(rollerR, rollerR, 0.030, 16), roller);
      w.rotation.z = Math.PI / 2;
      w.position.x = x;
      _roller.add(w);
      // TREADS, proud of the rim. A coaxial flange was the first attempt and it is invisible
      // for the same reason the bare cylinder was: anything with the axle for an axis of
      // symmetry looks identical at every angle, so the part reads as dead however fast it is
      // turning. The mecanum wheels forty lines up already had the answer -- put something
      // off-axis on the rim -- and this is the same trick.
      for (let k = 0; k < 3; k++) {
        const a = (k / 3) * Math.PI * 2;
        const tread = new THREE.Mesh(new THREE.BoxGeometry(0.036, 0.009, rollerR * 0.85), rubber);
        tread.position.set(x, Math.cos(a) * rollerR, Math.sin(a) * rollerR);
        // +a, not -a. Rx(a) takes the box's Y to the radial direction at this angle and its
        // Z to the tangent, which is a tread lying ON the rim; Rx(-a) leaves the long axis
        // pointing neither way, and three of those read as an auger rather than a wheel.
        tread.rotation.x = a;
        _roller.add(tread);
      }
    }
    intake.add(_roller);

    // The mouth: two side cheeks, and nothing under the roller. NO RAMP -- there was a tilted
    // plate here reaching down and forward, and `robot.ts` deletes the physical one
    // in as many words ("a plate that reaches out to tile level to catch a ball also drags on
    // the tile", three quarters of the robot's top speed). Drawing a plough the robot does not
    // have is the same lie in a different medium.
    for (const sx of [-1, 1]) {
      const cheek = new THREE.Mesh(new THREE.BoxGeometry(0.010, ip.mouth.height_m, ip.mouth.depth_m + 0.04), frame);
      cheek.position.set(sx * (mouthW / 2 + 0.005), ip.mouth.height_m / 2 - rollerR, -0.02);
      intake.add(cheek);
    }
    // Sit the roller so it just clears the bin floor: that is the squeeze `squeeze_N` stands
    // for, and it is why the lift term in stepIntake can carry a ball over the floor's edge.
    intake.position.set(0, binFloorY + rollerR, c.L / 2 + ip.mouth.depth_m / 2);
    _group.add(intake);

    // a nose stripe, so facing is unmistakable from the top camera
    const nose = new THREE.Mesh(new THREE.BoxGeometry(0.30, 0.012, 0.04), new THREE.MeshBasicMaterial({ color: 0xffffff }));
    nose.position.set(0, c.H / 2 + 0.008, c.L / 2 - 0.04);
    _group.add(nose);

    // hopper: an open bin you can see the stack of balls sitting in
    const bin = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.20, 0.24), new THREE.MeshStandardMaterial({ color: 0x0ea5e9, transparent: true, opacity: 0.22, side: THREE.DoubleSide }));
    bin.position.set(0, -c.H / 2 + 0.12, -0.02);
    _group.add(bin);

    // TURRET, shaped like a real FTC shooter rather than a cannon.
    //
    // A lazy-susan ring carries a deck; two side plates stand on the deck and hold a single
    // grippy flywheel; a curved HOOD wraps over the top of the flywheel, and the ball is
    // squeezed between the two and leaves along the tangent at the hood's lip. That is the
    // whole mechanism, and every part of it here does the job its real counterpart does:
    // changing the hood angle rotates the wrap, which is exactly what aims the shot.
    const alum = new THREE.MeshStandardMaterial({ color: 0xaeb6c0, metalness: 0.65, roughness: 0.32, side: THREE.DoubleSide });
    const dark = new THREE.MeshStandardMaterial({ color: 0x3a4250, metalness: 0.4, roughness: 0.5 });
    const grip = new THREE.MeshStandardMaterial({ color: 0x23282f, roughness: 0.95 });

    // lazy-susan: a bearing race with visible teeth, then the deck it turns
    const race = new THREE.Mesh(new THREE.CylinderGeometry(0.115, 0.115, 0.012, 32), dark);
    _turret.add(race);
    const teeth = new THREE.Mesh(new THREE.CylinderGeometry(0.108, 0.108, 0.016, 48), alum);
    teeth.position.y = 0.012;
    _turret.add(teeth);
    const deck = new THREE.Mesh(new THREE.CylinderGeometry(0.100, 0.100, 0.008, 24), alum);
    deck.position.y = 0.024;
    _turret.add(deck);

    // side plates: the load-bearing part of every shooter ever built
    const FLY_Y = 0.085;       // flywheel axle height above the turret origin
    const FLY_R = 0.050;
    for (const side of [-1, 1]) {
      const plateShape = new THREE.Shape();
      plateShape.moveTo(-0.075, 0);
      plateShape.lineTo(0.075, 0);
      plateShape.lineTo(0.075, 0.055);
      plateShape.absarc(0, 0.061, 0.075, 0, Math.PI, false);
      plateShape.lineTo(-0.075, 0);
      // lightening hole, because a real plate has one and it reads instantly as aluminium
      const hole = new THREE.Path();
      hole.absarc(0, 0.061, 0.030, 0, Math.PI * 2, true);
      plateShape.holes.push(hole);
      const plate = new THREE.Mesh(new THREE.ExtrudeGeometry(plateShape, { depth: 0.004, bevelEnabled: false }), alum);
      plate.rotation.y = Math.PI / 2;
      plate.position.set(side * 0.048, 0.028, 0);
      _turret.add(plate);
    }

    // the flywheel: a grippy wheel on a visible axle, between the plates
    _flywheel = new THREE.Mesh(new THREE.CylinderGeometry(FLY_R, FLY_R, 0.055, 24), grip);
    _flywheel.rotation.z = Math.PI / 2;
    _flywheel.position.set(0, FLY_Y, 0);
    // four rim markers: a plain black wheel spinning at 4000 rpm reads as stationary
    // without something on it to watch.
    for (let k = 0; k < 4; k++) {
      const a = (k / 4) * Math.PI * 2;
      const mark = new THREE.Mesh(new THREE.BoxGeometry(0.009, 0.058, 0.009), alum);
      mark.position.set(Math.cos(a) * FLY_R * 0.82, 0, Math.sin(a) * FLY_R * 0.82);
      _flywheel.add(mark);
    }
    _turret.add(_flywheel);
    const axle = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.115, 8), steel);
    axle.rotation.z = Math.PI / 2;
    axle.position.set(0, FLY_Y, 0);
    _turret.add(axle);

    // feed ramp up from the hopper, so you can see where the ball comes from
    const ramp = new THREE.Mesh(new THREE.BoxGeometry(0.085, 0.003, 0.13), alum);
    ramp.position.set(0, 0.048, -0.055);
    ramp.rotation.x = 0.45;
    _turret.add(ramp);

    // HOOD: a curved wrap concentric with the flywheel, held one ball-radius off it. It
    // pivots about the flywheel axis, and the ball leaves tangentially at its lip -- so the
    // hood angle you see IS the launch elevation.
    _hood = new THREE.Group() as unknown as THREE.Mesh;
    // The wrap spans local angle a in [0, WRAP], measured from +Z (straight ahead) up and
    // over the wheel, so a = 0 IS the exit lip. Rotating the group by -elevation about X
    // then puts the lip at exactly the elevation the shot leaves at -- the hood you see is
    // the hood the ballistics uses.
    const WRAP = Math.PI * 0.83;
    const gap = FLY_R + inches(2.5);               // wheel radius + a POLLEN
    const wrap = new THREE.Mesh(
      new THREE.CylinderGeometry(gap, gap, 0.062, 30, 1, true, 0, WRAP),
      new THREE.MeshStandardMaterial({ color: 0xd7dde4, metalness: 0.45, roughness: 0.35, side: THREE.DoubleSide }),
    );
    wrap.rotation.z = Math.PI / 2;                 // cylinder axis Y -> X, cross-section into YZ
    _hood.add(wrap);
    // edge ribs: what stops a real sheet hood flexing, and they make the curve read
    for (const sx of [-1, 1]) {
      const rib = new THREE.Mesh(new THREE.TorusGeometry(gap, 0.004, 6, 26, WRAP), alum);
      rib.rotation.y = -Math.PI / 2;               // torus XY plane -> YZ, angle from +Z
      rib.position.x = sx * 0.031;
      _hood.add(rib);
    }
    // the lip the ball leaves over, tangent to the wrap at a = 0
    const exit = new THREE.Mesh(new THREE.BoxGeometry(0.062, 0.026, 0.004), new THREE.MeshStandardMaterial({ color: COL.pollen, roughness: 0.5 }));
    exit.position.set(0, -0.010, gap);
    _hood.add(exit);
    // the link that sets the angle, from the deck up to the back of the wrap
    const link = new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.004, 0.08, 6), steel);
    link.position.set(0.042, Math.sin(WRAP) * gap * 0.5 - 0.03, Math.cos(WRAP) * gap * 0.5);
    link.rotation.x = 0.7;
    _hood.add(link);
    _hood.position.set(0, FLY_Y, 0);
    _turret.add(_hood);

    _turret.position.set(0, c.H / 2 - 0.02, 0);
    _group.add(_turret);
    return { group: _group, turret: _turret, hood: _hood, flywheel: _flywheel, roller: _roller, wheels: _wheels };
  }

  private buildRobot(): void {
    const b = this.buildChassis(COL.robot);
    this.robotGroup = b.group;
    this.turretGroup = b.turret;
    this.hoodMesh = b.hood;
    this.flywheelMesh = b.flywheel;
    this.intakeRoller = b.roller;
    this.wheelMeshes = b.wheels;
    this.scene.add(this.robotGroup);
  }

  // ---------------------------------------------------------------- update

  update(s: Snapshot, aim: Vec3 | null, traj: Vec3[] | null): void {
    for (let i = 0; i < s.balls.length; i++) {
      const b = s.balls[i];
      const m = this.ballMeshes[i];
      if (!m) continue;
      m.visible = b.p[1] > -0.5;
      m.position.set(b.p[0], b.p[1], b.p[2]);
    }

    for (const [i, h] of s.hives.entries()) {
      const g = this.rockers[i];
      if (g) g.rotation.x = h.angleDeg * DEG;
      if (h.alliance === this.zoneAlliance) this.setZoneSide(h.upCell === 'A' ? 0 : 1);
    }

    // THE MAP IS PAINTED FOR A STATIONARY ROBOT, deliberately, and it used to repaint from
    // the live velocity every time that moved by 0.15 m/s.
    //
    // Two reasons it does not any more. The question a driver asks of a floor map is "if I
    // GO there, can I shoot?", and the answer to that is the stationary one -- painting every
    // distant cell with the velocity the robot happens to have right now answers "if I were
    // over there moving like this", which is a hypothetical nobody asked.
    //
    // And measurement killed it: tools/collect.ts --moving fired ONE shot out of forty, with
    // that shot 280 rpm outside its band, where the same run standing still fired forty. The
    // lead moves the target rpm every loop and the readiness gate needs three consecutive
    // loops inside 60 rpm, so on the move the gate essentially never latches. A map that
    // turns greener as you drive at the hive would be describing shots this robot cannot
    // take, so it is repainted at the speed the robot is ACTUALLY doing.
    //
    // The cells carry parameters rather than a finished probability precisely so this can be
    // done every frame: the lead keeps a shot's ground path the same whatever the robot is
    // doing, so the aperture, the speed band and the entry rate belong to the SPOT and were
    // solved once. What driving changes is the speed the shot must LEAVE at -- retreating
    // needs a faster one -- and launch scatter is a fraction of exit speed, so the error at
    // the mouth grows with it. The green is therefore a different shape at 1 m/s than it is
    // standing still, and the map was drawing the standing-still one all match.
    if (this.showShotZone) {
      const v: [number, number] = [s.robot.v[0], s.robot.v[2]];
      // Repaint on a real change only. It rasterises 841 cells into a 512 px canvas, which is
      // cheap but not free, and below a tenth of a metre per second the picture does not
      // visibly move -- so this runs a few times a second while driving and never when parked.
      if (Math.hypot(v[0] - this.zoneVel[0], v[1] - this.zoneVel[1]) > 0.1) this.repaintZone(v);
    }

    if (s.opponent) {
      if (!this.opponentGroup) this.buildOpponent();
      const o = s.opponent;
      this.opponentGroup!.visible = true;
      this.opponentGroup!.position.set(o.p[0], o.p[1], o.p[2]);
      this.opponentGroup!.rotation.y = o.yawDeg * DEG;
      this.opponentTurret!.rotation.y = o.turret.angleDeg * DEG;
      // The hood and the wheel are the other half of reading it: a raised hood and a
      // spinning wheel say it is about to shoot, which a turret stick never could.
      if (this.opponentHood) this.opponentHood.rotation.x = -o.hood.angleDeg * DEG;
      if (this.opponentFlywheel) this.opponentFlywheel.rotation.x -= o.flywheel.rpm * 0.0008;
    } else if (this.opponentGroup) {
      this.opponentGroup.visible = false;
    }

    const r = s.robot;
    this.robotGroup.position.set(r.p[0], r.p[1], r.p[2]);
    this.robotGroup.rotation.y = r.yawDeg * DEG;
    this.turretGroup.rotation.y = r.turret.angleDeg * DEG;
    this.hoodMesh.rotation.x = -r.hood.angleDeg * DEG;

    // Moving parts actually move: wheels at their own rate, the intake roller while it is
    // running, the flywheel at its real RPM. It is the quickest read on what the robot is
    // doing without looking at a single number.
    const dt = 1 / 60;
    r.wheels.forEach((w, i) => {
      const hub = this.wheelMeshes[i];
      if (hub) hub.rotation.x += w.omega * dt;
    });
    // About X, which is the axle after the roller was laid over at build time -- and from the
    // shaft's MEASURED speed, not from the power it was told to draw. A stalled roller has no
    // surface speed and therefore no grip, which is what a jam is (robot.ts stepIntake); one
    // drawn from the command spins merrily through a jam and hides the only symptom there is.
    //
    // Geared down for the eye, not for the truth: a 5203-1150 turns 116 rad/s, which is 110
    // degrees a frame at 60 Hz -- past the strobe limit, where a spinning wheel reads as
    // stopped or as running backwards. The factor is a constant, so half speed still looks
    // like half speed and a stall still stops dead.
    this.spin.intake += r.intake.omega * SPIN_SHOWN * dt;
    this.intakeRoller.rotation.x = this.spin.intake;
    this.spin.fly += (r.flywheel.rpm / 60) * 2 * Math.PI * dt;
    this.flywheelMesh.rotation.y = this.spin.fly;

    if (aim) {
      this.aimMarker.position.set(aim[0], aim[1], aim[2]);
      this.aimMarker.lookAt(this.camera.position);
      this.aimMarker.visible = true;
    } else {
      this.aimMarker.visible = false;
    }

    if (this.belief && this.showBelief) {
      this.beliefMarker.position.set(this.belief[0], this.belief[1], this.belief[2]);
      this.beliefMarker.lookAt(this.camera.position);
      this.beliefMarker.visible = true;
    } else {
      this.beliefMarker.visible = false;
    }

    // ---- the live trail: follow the ball from THIS shot, and keep it drawn until the next
    // one, so a miss can be looked at after it has landed.
    //
    // KEYED ON THE SHOT COUNTER, not on the ball's id. Balls are recycled -- one that lands is
    // picked up and fired again with the same id -- so `id !== trailBall` never fired for it
    // and the new flight's points were appended to the old flight's. The curve then ran from
    // the muzzle, out to wherever the first shot landed, back to the muzzle and out again:
    // two arcs joined by a straight line, which is the "the blue line just freezes there"
    // report. It was not frozen, it was two shots in one curve.
    //
    // And with a fast cycle there is more than one ball in the air, so `find` is not good
    // enough either: it returns whichever sits earliest in the array, which is the OLDER
    // shot. Follow the one nearest where this trail already is.
    if (s.robot.flywheel.shots !== this.trailShot) {
      this.trailShot = s.robot.flywheel.shots;
      this.trailPts = [];
    }
    const inFlight = s.balls.filter((b) => b.state === 'flight');
    if (inFlight.length) {
      const anchor = this.trailPts.length
        ? this.trailPts[this.trailPts.length - 1]
        : new THREE.Vector3(s.robot.p[0], s.robot.p[1], s.robot.p[2]);
      let flying = inFlight[0];
      let best = Infinity;
      for (const b of inFlight) {
        const d = anchor.distanceToSquared(new THREE.Vector3(b.p[0], b.p[1], b.p[2]));
        if (d < best) { best = d; flying = b; }
      }
      const p = new THREE.Vector3(flying.p[0], flying.p[1], flying.p[2]);
      const last = this.trailPts[this.trailPts.length - 1];
      if (!last || last.distanceTo(p) > 0.02) this.trailPts.push(p);
      // One flight is a couple of hundred samples at 2 cm; anything past that is a bug
      // feeding it, and an unbounded array would take the frame rate down with it.
      if (this.trailPts.length > 400) this.trailPts.shift();
    }
    if (this.showTrajectory && this.trailPts.length > 2) {
      this.ballTrail.geometry.dispose();
      this.ballTrail.geometry = new THREE.TubeGeometry(
        new THREE.CatmullRomCurve3(this.trailPts), Math.max(8, this.trailPts.length), 0.009, 8, false,
      );
      this.ballTrail.visible = true;
    } else {
      this.ballTrail.visible = false;
    }

    if (this.showTrajectory && traj && traj.length > 1) {
      this.trajLine.geometry.dispose();
      // Thin the integrator's output before building the tube: it hands over hundreds of
      // points a few millimetres apart, and a curve through those is all noise and no shape.
      const step = Math.max(1, Math.floor(traj.length / 48));
      const pts = traj.filter((_, i) => i % step === 0 || i === traj.length - 1)
        .map((q) => new THREE.Vector3(q[0], q[1], q[2]));
      this.trajLine.geometry = new THREE.TubeGeometry(
        new THREE.CatmullRomCurve3(pts), Math.max(8, pts.length * 2), 0.011, 8, false,
      );
      this.trajLine.visible = true;
    } else {
      this.trajLine.visible = false;
    }

    this.placeCamera(s);
  }

  /**
   * Swing the view. `dYaw`/`dPitch` are already rate x dt; pitch is clamped short of the
   * poles because the orbit formula degenerates there and the view flips.
   */
  nudgeView(dYaw: number, dPitch: number): void {
    if (!dYaw && !dPitch) return;
    this.viewYaw += dYaw;
    if (this.cameraMode === 'orbit') this.orbit.theta += dYaw;
    this.orbit.phi = Math.min(Math.PI - 0.12, Math.max(0.12, this.orbit.phi + dPitch));
  }

  private placeCamera(s: Snapshot): void {
    const r = s.robot;
    const o = this.orbit;
    switch (this.cameraMode) {
      case 'top':
        this.camera.position.set(0, 6.2, 0.001);
        this.camera.lookAt(0, 0, 0);
        break;
      case 'follow': {
        // Behind the robot, at the orbit's own elevation so the wheel still tilts the view.
        const yaw = r.yawDeg * DEG + this.viewYaw;
        const back = 2.1 + o.phi * 0.5;
        this.camera.position.set(r.p[0] - Math.sin(yaw) * back, r.p[1] + 0.55 + o.phi * 0.9, r.p[2] - Math.cos(yaw) * back);
        this.camera.lookAt(r.p[0] + Math.sin(yaw) * 1.5, r.p[1] + 0.25, r.p[2] + Math.cos(yaw) * 1.5);
        break;
      }
      case 'fpv': {
        // Driver's eye: on the robot, looking where the robot is pointed. The turret is
        // free to be aimed somewhere else entirely, which is the whole point of having one.
        const yaw = r.yawDeg * DEG + this.viewYaw;
        // Pitch the gaze with the same phi the other modes use: 1.0 rad is the neutral the
        // orbit starts at, so a driver who has not touched the stick looks level.
        const pitch = (1.0 - o.phi) * 1.6;
        this.camera.position.set(r.p[0] + Math.sin(yaw) * 0.18, r.p[1] + 0.22, r.p[2] + Math.cos(yaw) * 0.18);
        this.camera.lookAt(
          r.p[0] + Math.sin(yaw) * 6,
          r.p[1] + 0.22 + 0.9 + pitch * 6,
          r.p[2] + Math.cos(yaw) * 6,
        );
        break;
      }
      case 'muzzle': {
        const yaw = (r.yawDeg + r.turret.angleDeg) * DEG;
        this.camera.position.set(r.p[0] + Math.sin(yaw) * 0.1, r.p[1] + 0.25, r.p[2] + Math.cos(yaw) * 0.1);
        const el = r.hood.angleDeg * DEG;
        this.camera.lookAt(
          r.p[0] + Math.sin(yaw) * Math.cos(el) * 4,
          r.p[1] + 0.25 + Math.sin(el) * 4,
          r.p[2] + Math.cos(yaw) * Math.cos(el) * 4,
        );
        break;
      }
      default:
        this.camera.position.set(
          o.target.x + o.dist * Math.sin(o.phi) * Math.cos(o.theta),
          o.target.y + o.dist * Math.cos(o.phi),
          o.target.z + o.dist * Math.sin(o.phi) * Math.sin(o.theta),
        );
        this.camera.lookAt(o.target);
    }
  }

  private attachControls(): void {
    let dragging = false;
    let lx = 0;
    let ly = 0;
    this.canvas.addEventListener('pointerdown', (e) => {
      lx = e.clientX;
      ly = e.clientY;
      // LEFT ORBITS, RIGHT AND MIDDLE PAN, which is what every other 3D view does and what
      // a hand reaches for without being told. Panning was shift-drag only: it worked, and
      // nobody found it, so the camera could orbit and zoom but never look at a different
      // part of the field. Shift-drag still pans for anyone who learned it that way.
      //
      // (Right-drag used to turn the ROBOT, which made the chassis chase the camera every
      // time you looked around -- fine in a shooter, awful when holding a firing position.)
      dragging = true;
      this.canvas.setPointerCapture(e.pointerId);
    });
    const release = (e: PointerEvent) => {
      dragging = false;
      try { this.canvas.releasePointerCapture(e.pointerId); } catch { /* already gone */ }
    };
    this.canvas.addEventListener('pointerup', release);
    this.canvas.addEventListener('pointercancel', release);
    addEventListener('blur', () => { dragging = false; });
    this.canvas.addEventListener('pointermove', (e) => {
      if (!dragging || this.cameraMode !== 'orbit') return;
      const dx = e.clientX - lx;
      const dy = e.clientY - ly;
      const panning = e.shiftKey || e.buttons === 2 || e.buttons === 4;

      if (panning) {
        // Shift-drag pans: slide the orbit target across the camera's own screen plane, so
        // it follows the mouse whichever way the camera happens to be facing.
        const scale = this.orbit.dist * 0.0016;
        const right = new THREE.Vector3().setFromMatrixColumn(this.camera.matrix, 0);
        const up = new THREE.Vector3().setFromMatrixColumn(this.camera.matrix, 1);
        this.orbit.target.addScaledVector(right, -dx * scale).addScaledVector(up, dy * scale);
        const lim = this.geom.halfWidth_m + 1;
        this.orbit.target.x = Math.max(-lim, Math.min(lim, this.orbit.target.x));
        this.orbit.target.y = Math.max(0, Math.min(3, this.orbit.target.y));
        this.orbit.target.z = Math.max(-lim, Math.min(lim, this.orbit.target.z));
      } else {
        this.orbit.theta += dx * 0.006;
        this.orbit.phi = Math.min(1.52, Math.max(0.12, this.orbit.phi - dy * 0.006));
      }
      lx = e.clientX;
      ly = e.clientY;
    });
    this.canvas.addEventListener('contextmenu', (e) => e.preventDefault());
    this.canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.orbit.dist = Math.min(14, Math.max(1.2, this.orbit.dist * (1 + Math.sign(e.deltaY) * 0.1)));
    }, { passive: false });
  }

  resize(): void {
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    if (w === 0 || h === 0) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  render(): void {
    this.renderer.render(this.scene, this.camera);
  }
}

export const _unused = M_TO_IN;
