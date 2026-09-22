/**
 * A REAL TEAM'S CAD, DRIVEN BY THE SIMULATION.
 *
 * tools/cad2robot.py turns a robot STEP into a .glb whose meshes are named for the part each
 * one drives -- wheel0..3, roller, flywheel, hood, turret, body -- and writes a -index.json
 * beside it with each moving group's pivot and spin axis. This hangs those meshes off pivot
 * groups and hands back the same handles `Scene.buildChassis` does, so the update loop that
 * already turns the procedural robot's wheels turns these without knowing the difference.
 *
 * WHY A PUPPET AND NOT A PICTURE. A static mesh of someone's robot tells you what it looks
 * like. This tells you what it is DOING: the wheels turn at their own measured speed, the
 * intake roller stops when it jams, the flywheel spins at its real rpm. Those are the three
 * things you cannot read off a number fast enough while driving.
 *
 * WHAT IT CANNOT DO, and says so rather than faking it: none of these robots has the turret
 * this simulator's brain aims. Their shooters are bolted to the frame. So `turret` comes back
 * as an EMPTY group on a skin with no turret part -- the yaw is applied to nothing, the sight
 * line still shows where the shot is aimed, and nothing on screen claims a robot can do
 * something it cannot.
 */
import * as THREE from 'three';
import type { RobotSpec } from '@core/types.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';

/** The .glb files tools/cad2robot.py has written. Absent ones simply do not appear. */
const SKIN_URLS = import.meta.glob('../../../assets/robot-*.glb', {
  query: '?url',
  import: 'default',
  eager: true,
}) as Record<string, string>;
const SKIN_INDEX = import.meta.glob('../../../assets/robot-*-index.json', {
  import: 'default',
  eager: true,
}) as Record<string, RobotIndex>;

export interface RobotIndex {
  units: string;
  source: string;
  tris: number;
  bbox_m: [number, number, number];
  min_m: [number, number, number];
  max_m: [number, number, number];
  groups: Record<string, { parts: number; tris: number; pivot: [number, number, number]; axis: [number, number, number] }>;
}

const nameOf = (path: string) => /robot-(.+?)(?:-index)?\.(?:glb|json)$/.exec(path)?.[1] ?? path;

/** Which skins exist, as the UI's picker lists them. 'box' is the procedural robot. */
export function skinNames(): string[] {
  return ['box', ...Object.keys(SKIN_URLS).map(nameOf).sort()];
}

export interface Puppet {
  group: THREE.Group;
  turret: THREE.Group;
  hood: THREE.Mesh;
  flywheel: THREE.Mesh;
  roller: THREE.Group;
  wheels: THREE.Object3D[];
}

/** A stand-in for a part this robot does not have, so every handle is always real. */
const stub = () => new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({ visible: false }));

/**
 * Hang `mesh` off a group at its own pivot, so rotating the group turns the part about its
 * axle instead of about the robot's centre. The mesh keeps its world position; the group is
 * what moves.
 */
function pivoted(mesh: THREE.Object3D, pivot: [number, number, number]): THREE.Group {
  const g = new THREE.Group();
  g.position.set(pivot[0], pivot[1], pivot[2]);
  mesh.position.set(-pivot[0], -pivot[1], -pivot[2]);
  g.add(mesh);
  return g;
}

/**
 * Load a skin and rig it.
 *
 * `tint` colours the body so the two alliances stay apart at a glance -- these STEPs carry no
 * colour at all (checked: 0 parts of 247 in the Void export), so without this every robot on
 * the field is the same grey and you cannot tell yours from theirs while driving.
 */
export async function loadPuppet(name: string, spec: RobotSpec, tint: number): Promise<Puppet | null> {
  const key = Object.keys(SKIN_URLS).find((p) => nameOf(p) === name);
  if (!key) return null;
  const idx = SKIN_INDEX[key.replace('.glb', '-index.json')];
  const gltf = await new GLTFLoader().loadAsync(SKIN_URLS[key]);

  const group = new THREE.Group();
  const parts = new Map<string, THREE.Object3D>();
  gltf.scene.traverse((o) => {
    if ((o as THREE.Mesh).isMesh) parts.set(o.name, o);
  });

  // THE CAD SITS WHERE THE CAD SITS. The simulator's robot group is centred on the chassis
  // centre with the floor at -(H/2 + clearance), so the model is shifted to put its own
  // wheels on that floor and its own middle on that centre. Without this a robot is buried
  // in the tiles or floating over them, which is the first thing anyone notices.
  const c = spec.chassis;
  const mid = idx ? [(idx.min_m[0] + idx.max_m[0]) / 2, 0, (idx.min_m[2] + idx.max_m[2]) / 2] : [0, 0, 0];
  const floor = -(c.height_m / 2 + c.clearance_m);
  const shift = new THREE.Vector3(-mid[0], floor - (idx ? idx.min_m[1] : 0), -mid[2]);

  const inner = new THREE.Group();
  inner.position.copy(shift);
  group.add(inner);

  const body = parts.get('body');
  if (body) {
    (body as THREE.Mesh).material = new THREE.MeshStandardMaterial({
      color: tint, metalness: 0.55, roughness: 0.42, vertexColors: true,
    });
    inner.add(body);
  }

  const pivotOf = (g: string): [number, number, number] =>
    (idx?.groups[g]?.pivot as [number, number, number]) ?? [0, 0, 0];

  const wheels: THREE.Object3D[] = [];
  for (let i = 0; i < 4; i++) {
    const m = parts.get(`wheel${i}`);
    if (!m) continue;
    const g = pivoted(m, pivotOf(`wheel${i}`));
    inner.add(g);
    wheels.push(g);
  }

  const rollerMesh = parts.get('roller');
  const roller = rollerMesh ? pivoted(rollerMesh, pivotOf('roller')) : new THREE.Group();
  if (rollerMesh) inner.add(roller);

  const flyMesh = parts.get('flywheel') as THREE.Mesh | undefined;
  let flywheel: THREE.Mesh = stub();
  if (flyMesh) {
    inner.add(pivoted(flyMesh, pivotOf('flywheel')));
    flywheel = flyMesh;
  }

  const hoodMesh = parts.get('hood') as THREE.Mesh | undefined;
  let hood: THREE.Mesh = stub();
  if (hoodMesh) {
    inner.add(pivoted(hoodMesh, pivotOf('hood')));
    hood = hoodMesh;
  }

  // The turret group is what the sim yaws. A skin with a turret part gets it; a skin without
  // one gets an empty group, on purpose -- see the header.
  const turret = new THREE.Group();
  const turretMesh = parts.get('turret');
  if (turretMesh) {
    turret.position.set(...pivotOf('turret'));
    turretMesh.position.set(...(pivotOf('turret').map((v) => -v) as [number, number, number]));
    turret.add(turretMesh);
  }
  inner.add(turret);

  // Anything the converter did not recognise still has to be drawn, or a robot loses whole
  // subassemblies and looks half-built.
  for (const [n, o] of parts) {
    if (!o.parent || o.parent === gltf.scene) {
      if (!/^(body|wheel\d|roller|flywheel|hood|turret)$/.test(n)) inner.add(o);
    }
  }

  return { group, turret, hood, flywheel, roller, wheels };
}
