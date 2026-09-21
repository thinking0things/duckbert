"""Import neutral assembly STL exports from the v1.1 CAD folder.
Usage: python scripts/import-cad.py /path/to/out_none_v1.1
Requires numpy and trimesh for CAD-derived mass properties.
"""
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as E
import numpy as np
import trimesh

source = Path(sys.argv[1]).resolve()
assets = Path(__file__).resolve().parents[1] / "dist/assets"
manifest = json.loads((source / "manifest.json").read_text())
tree = E.parse(assets / "robot.xml")
root = tree.getroot()
root.set("model", "Duckbert v1.1 - 6 SG90")
root.find("compiler").set("meshdir", "meshes")
asset = root.find("asset")
asset.clear()
bodies = {b.get("name"): b for b in root.iter("body")}
origins = {}
def walk(body, parent=np.zeros(3)):
    origin = parent + np.fromstring(body.get("pos", "0 0 0"), sep=" ")
    origins[body.get("name")] = origin
    for child in body.findall("body"):
        walk(child, origin)
walk(bodies["torso"])
for body in bodies.values():
    for geom in list(body.findall("geom")):
        if geom.get("type") == "mesh":
            body.remove(geom)
fmt = lambda values: " ".join(f"{v:.10g}" for v in values)
# Linear RGB corresponding to sRGB #800020 (Bordeaux).
srgb = np.array([128, 0, 32]) / 255
linear = np.where(srgb <= .04045, srgb / 12.92, ((srgb + .055) / 1.055) ** 2.4)
rgba = fmt([*linear, 1])
items = {name: [] for name in bodies}
hardware = {p["name"]: p for p in manifest["hardware"]}
parts = {p["name"]: p for p in manifest["parts"] if p["name"] != "00_fit_coupon"}
hashes = {}
for name in [*parts, *hardware]:
    src = source / "assembly_meshes" / (name + ".stl")
    mesh = trimesh.load_mesh(src, process=True)
    assert mesh.is_watertight and mesh.volume > 0, name
    mesh.apply_scale(.001)
    if name in parts:
        if name.startswith(("01_", "02_", "03_", "15_")):
            owner = "torso"
        else:
            owner = name[-1] + "_" + {"04": "hip", "05": "thigh", "06": "shin", "07": "shin"}[name[:2]]
        mass = mesh.volume * 1240  # Solid PLA estimate, matching the CAD manifest.
    else:
        owner = hardware[name]["group"]
        mass = hardware[name].get("mass", .0004 if name.endswith("_stock_horn") else .009)
    items[owner].append((mass, mesh.center_mass, mesh.moment_inertia * mass / mesh.volume))
    shutil.copyfile(src, assets / "meshes" / src.name)
    hashes[src.name] = hashlib.sha256(src.read_bytes()).hexdigest()
    E.SubElement(asset, "mesh", name=name, file=src.name, scale=".001 .001 .001")
    E.SubElement(bodies[owner], "geom", type="mesh", mesh=name, pos=fmt(-origins[owner]),
                 rgba=rgba, contype="0", conaffinity="0", group="1", mass="0")
    if name in parts and not name.startswith("07_"):
        E.SubElement(bodies[owner], "geom", name=name + "_collision", type="mesh", mesh=name,
                     pos=fmt(-origins[owner]), contype="2", conaffinity="1", group="3",
                     rgba="0 0 0 0", friction=".9 .01 .001", mass="0")
# Wires and fasteners are an explicit estimate, not hidden mass in visual meshes.
wire_mass = .022
wire_size = np.array([.020, .020, .010])
wire_inertia = np.diag(wire_mass / 12 * (sum(wire_size**2) - wire_size**2))
items["torso"].append((wire_mass, np.array([-.010, 0, .1384]), wire_inertia))
for name, owned in items.items():
    mass = sum(p[0] for p in owned)
    com = sum(m*c for m, c, _ in owned) / mass
    inertia = np.zeros((3, 3))
    for m, c, i in owned:
        delta = c-com
        inertia += i + m * (np.dot(delta, delta)*np.eye(3)-np.outer(delta, delta))
    body = bodies[name]
    inertial = body.find("inertial")
    inertial.set("mass", f"{mass:.10g}")
    inertial.set("pos", fmt(com-origins[name]))
    inertial.set("fullinertia", fmt([inertia[0,0],inertia[1,1],inertia[2,2],inertia[0,1],inertia[0,2],inertia[1,2]]))
for joint in manifest["joints"]:
    name = joint["name"]
    root.find(f".//joint[@name='{name}']").set("range", fmt(joint["limits"]))
    root.find(f".//position[@joint='{name}']").set("ctrlrange", fmt(np.radians(joint["limits"])))
for side, sign in [("L",1),("R",-1)]:
    sole = np.array([.007, sign*.0305, .002]) - origins[side+"_shin"]
    root.find(f".//geom[@name='{side}_sole']").set("pos",fmt(sole))
    root.find(f".//site[@name='{side}_foot_touch']").set("pos",fmt(sole))
E.indent(root)
tree.write(assets / "robot.xml", encoding="unicode")
files = ["meshes/"+name for name in hashes]
for path in (assets / "meshes").glob("*.stl"):
    if path.name not in hashes:
        path.unlink()
(assets / "files.json").write_text(json.dumps(files, indent=2)+"\n")
provenance = {
    "model": "Duckbert v1.1",
    "cad_revision": "out_none_v1.1",
    "visual_meshes": "Unmodified neutral assembly_meshes STL exports from the v1.1 CAD folder",
    "mesh_sha256": hashes,
    "manifest_sha256": hashlib.sha256((source/"manifest.json").read_bytes()).hexdigest(),
    "model_sha256": hashlib.sha256((assets/"robot.xml").read_bytes()).hexdigest(),
    "physics": "CAD mesh mass properties, solid PLA density 1240 kg/m3, module mass estimates from the CAD manifest, plus 22 g of wiring/fasteners",
    "contact": "54 x 41 mm flat sole boxes centred at y = +/-30.5 mm; printed-part convex hulls collide with the ground",
    "colour": "Renderer offers Bordeaux or White/Orange/Cyan, with blue servos, black screen and white eyes in both palettes",
    "controllers": ["v11_walk"],
    "controller_source": "none_oled periodic controller, validated on the imported v1.1 model",
    "steering": "Hip swing amplitude asymmetry, 10 percent maximum, with slew-limited targets"
}
(assets / "provenance.json").write_text(json.dumps(provenance,indent=2)+"\n")
print(f"Imported {len(hashes)} meshes; total estimated mass {sum(p[0] for owned in items.values() for p in owned):.4f} kg")
