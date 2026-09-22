"""
Tessellate a ROBOT STEP into a PUPPET: render meshes grouped by the part each one drives.

    python tools/cad2robot.py "C:/path/Robot.step" assets/robot-<name>.glb [--fast] [--budget 140000]
    python tools/cad2robot.py "C:/path/Robot.step" --names        # what is in it, no meshing

The field pipeline (tools/cad2assets.py) knows what the field's parts ARE -- it has PLAN.md
Appendix A and matches AndyMark part numbers by name. A team's robot STEP has no such list, so
this matches the handful of names that MOVE and merges everything else into one body:

    wheel0..3   each drive wheel on its own, spinning about its own axle
    roller      the intake roller
    flywheel    the shooter wheel
    hood        the launch angle, if the robot has one
    turret      the yaw axis, if the robot has one
    body        every other part, welded into one mesh

A group is only worth separating if the simulator can drive it. Everything else is one mesh
because one mesh is one draw call, and a robot the renderer draws 60 times a second next to a
field and 40 balls cannot afford 300.

EACH MOVING GROUP CARRIES ITS PIVOT AND ITS AXIS, written into the -index.json beside the glb:
the centre of its own bounding box, and the axis it turns about, taken as the THINNEST
direction of that box (a wheel and a flywheel are discs; the thin way through a disc is the
axle). The renderer hangs the mesh off a pivot group there and turns it, which is what makes
this a puppet rather than a photograph.

Millimetres in, METRES out. Fasteners dropped -- they are most of the part count and none of
the silhouette. STEP colour used wherever the exporter wrote any, which so far is nowhere.
"""
import json
import re
import sys
import time

import numpy as np
import trimesh

from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorType
from OCP.Quantity import Quantity_Color

MM_TO_M = 0.001
FAST = "--fast" in sys.argv
NAMES_ONLY = "--names" in sys.argv
DEFLECTION = 6.0 if FAST else 2.5
MIN_VOL_MM3 = 400.0
SKIP = re.compile(
    r"screw|nut\b|washer|rivnut|rivet|cable tie|spacer|bearing|plug|standoff|shim|"
    r"retaining ring|shcs|bhcs|fhcs|set screw|dowel|zip tie|m3x|m4x|connector|pcb|magnet",
    re.I,
)
DEFAULT = (168, 176, 188)

# FIRST MATCH WINS, and `flywheel` is checked before `wheel` on purpose: every shooter wheel
# in these STEPs is called a flywheel, and a shooter wheel spinning as a drive wheel would be
# a puppet lying about what the robot is doing.
ROLES = [
    ("flywheel", re.compile(r"fly ?wheel|shooter wheel|launch(er)? wheel", re.I)),
    ("hood", re.compile(r"\bhood\b", re.I)),
    ("turret", re.compile(r"\bturret\b", re.I)),
    ("wheel", re.compile(r"mecanum|omni|traction wheel|\bwheel\b(?!.*intake)|grip ?force", re.I)),
    ("roller", re.compile(r"compliant|intake roller|\broller\b|surgical|stealth", re.I)),
]


def role_of(leaf: str):
    for name, rx in ROLES:
        if rx.search(leaf):
            return name
    return "body"


def name_of(lbl) -> str:
    n = TDataStd_Name()
    try:
        if lbl.FindAttribute(TDataStd_Name.GetID_s(), n):
            return n.Get().ToExtString()
    except Exception:
        pass
    return ""


def triangles(shape, deflection):
    """Tessellate a shape and return (vertices Nx3 mm, faces Mx3)."""
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.5, True)
    verts, faces = [], []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face_s(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            base = len(verts)
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                verts.append((p.X(), p.Y(), p.Z()))
            rev = face.Orientation() == 1   # TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                if rev:
                    b, c = c, b
                faces.append((base + a - 1, base + b - 1, base + c - 1))
        ex.Next()
    return verts, faces


def spin_axis(verts: np.ndarray):
    """A disc turns about the thin way through it: the shortest side of its bounding box."""
    ext = verts.max(axis=0) - verts.min(axis=0)
    i = int(np.argmin(ext))
    return [1.0 if k == i else 0.0 for k in range(3)]


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    step = args[0]
    out = args[1] if len(args) > 1 else ""
    bi = sys.argv.index("--budget") if "--budget" in sys.argv else -1
    budget = int(sys.argv[bi + 1]) if bi > 0 else 140_000
    t0 = time.time()
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    rdr = STEPCAFControl_Reader()
    rdr.SetNameMode(True)
    rdr.SetColorMode(True)
    assert rdr.ReadFile(step) == IFSelect_RetDone, "STEP read failed: " + step
    print("read (%.0fs)" % (time.time() - t0), flush=True)
    rdr.Transfer(doc)
    print("transfer ok (%.0fs)" % (time.time() - t0), flush=True)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    ctool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    def colour_of(label, shape):
        col = Quantity_Color()
        for ct in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen,
                   XCAFDoc_ColorType.XCAFDoc_ColorCurv):
            try:
                if ctool.GetColor(label, ct, col) or ctool.GetColor(shape, ct, col):
                    return (int(col.Red() * 255), int(col.Green() * 255), int(col.Blue() * 255))
            except Exception:
                pass
        return None

    parts = []            # (role, leaf, verts, faces, rgb)
    seen = {}
    stats = {"parts": 0, "skipped": 0, "coloured": 0}

    def walk(lbl, loc, path):
        nm = name_of(lbl)
        ref = TDF_Label()
        if tool.IsReference_s(lbl):
            tool.GetReferredShape_s(lbl, ref)
            here = loc.Multiplied(tool.GetLocation_s(lbl))
            target = ref
        else:
            here, target = loc, lbl
        p = path + [nm or name_of(target)]
        if tool.IsAssembly_s(target):
            comps = TDF_LabelSequence()
            tool.GetComponents_s(target, comps)
            for i in range(1, comps.Length() + 1):
                walk(comps.Value(i), here, p)
            return
        leaf = p[-1] if p else ""
        if NAMES_ONLY:
            seen[leaf] = seen.get(leaf, 0) + 1
            return
        if SKIP.search(leaf):
            stats["skipped"] += 1
            return
        shp = tool.GetShape_s(target)
        if shp.IsNull():
            return
        shp = shp.Moved(here)
        vol = 0.0
        ex = TopExp_Explorer(shp, TopAbs_SOLID)
        while ex.More():
            g = GProp_GProps()
            BRepGProp.VolumeProperties_s(ex.Current(), g)
            vol += g.Mass()
            ex.Next()
        if vol < MIN_VOL_MM3:
            stats["skipped"] += 1
            return
        v, f = triangles(shp, DEFLECTION)
        if not f:
            return
        rgb = colour_of(target, shp)
        if rgb:
            stats["coloured"] += 1
        parts.append((role_of(leaf), leaf, v, f, rgb or DEFAULT))
        stats["parts"] += 1

    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])

    if NAMES_ONLY:
        for leaf, n in sorted(seen.items(), key=lambda kv: -kv[1]):
            print("%4d  %-52s -> %s" % (n, leaf[:52], role_of(leaf)))
        print("%d distinct part names (%.0fs)" % (len(seen), time.time() - t0))
        return

    print("tessellated %d parts (%d with STEP colour), skipped %d (%.0fs)"
          % (stats["parts"], stats["coloured"], stats["skipped"], time.time() - t0), flush=True)

    # THE DRIVE WHEELS BECOME wheel0..3, EACH ITS OWN GROUP. Sorted by position so the order
    # is stable: the renderer binds snapshot wheel i to group wheel i, and a robot whose wheels
    # shuffled between conversions would spin the wrong corner.
    groups = {}
    wheels = [p for p in parts if p[0] == "wheel"]
    if wheels:
        keyed = []
        for p in wheels:
            c = np.asarray(p[2], dtype=np.float64).mean(axis=0)
            keyed.append((round(c[0], 1), round(c[1], 1), round(c[2], 1), p))
        keyed.sort(key=lambda k: (k[0], k[2], k[1]))
        for i, (_, _, _, p) in enumerate(keyed):
            groups.setdefault("wheel%d" % i if i < 4 else "body", []).append(p)
    for p in parts:
        if p[0] == "wheel":
            continue
        groups.setdefault(p[0], []).append(p)

    scene = trimesh.Scene()
    index = {}
    for group, pieces in groups.items():
        vs, fs, cs, off = [], [], [], 0
        for _role, _leaf, v, f, rgb in pieces:
            vs.extend(v)
            cs.extend([(rgb[0], rgb[1], rgb[2], 255)] * len(v))
            fs.extend([(a + off, b + off, c + off) for a, b, c in f])
            off += len(v)
        verts = np.asarray(vs, dtype=np.float64) * MM_TO_M
        mesh = trimesh.Trimesh(vertices=verts, faces=np.asarray(fs, dtype=np.int64),
                               vertex_colors=np.asarray(cs, dtype=np.uint8), process=False)
        # DECIMATE THE BODY, not the moving parts: the body is 90% of the triangles and none
        # of the motion, and a decimated wheel wobbles visibly as it turns.
        before = len(mesh.faces)
        if group == "body" and before > budget:
            mesh = mesh.simplify_quadric_decimation(face_count=budget)
        centre = (mesh.bounds[0] + mesh.bounds[1]) / 2
        index[group] = {
            "parts": len(pieces),
            "tris": int(len(mesh.faces)),
            "pivot": [round(float(x), 5) for x in centre],
            "axis": spin_axis(mesh.vertices),
        }
        scene.add_geometry(mesh, node_name=group, geom_name=group)
        print("  %-10s %4d parts  %8d tris%s" % (group, len(pieces), len(mesh.faces),
              "  (from %d)" % before if len(mesh.faces) != before else ""), flush=True)

    b = scene.bounds
    size = b[1] - b[0]
    total = sum(v["tris"] for v in index.values())
    scene.export(out)
    with open(out.rsplit(".", 1)[0] + "-index.json", "w", encoding="utf-8") as fh:
        json.dump({"units": "m", "source": step, "groups": index, "tris": total,
                   "bbox_m": [round(float(x), 4) for x in size],
                   "min_m": [round(float(x), 4) for x in b[0]],
                   "max_m": [round(float(x), 4) for x in b[1]]}, fh, indent=1)
    print("wrote %s  %d triangles, bbox %.3f x %.3f x %.3f m (%.0fs)"
          % (out, total, size[0], size[1], size[2], time.time() - t0))


if __name__ == "__main__":
    main()
