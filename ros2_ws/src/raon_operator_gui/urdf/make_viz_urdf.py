#!/usr/bin/env python3
"""Generate a mesh-free indy7_viz.urdf from the robot app's indy7.urdf.

WHY THIS EXISTS: indy7.urdf points every <mesh> at
/home/user/indy-ros2/install/indy_description/... — a path that exists on
nobody's machine, here or on the desktop. RViz would load the model, fail all
14 meshes and render nothing. The kinematics in that file are correct and are
what the RT controller solves against, so we keep them verbatim and only swap
the geometry: each link becomes one cylinder pointing at its child joint.

Result is a recognisable articulated Indy7 whose JOINT ANGLES are exact; the
link shells are approximate on purpose. Re-run after any kinematic edit:

    python3 urdf/make_viz_urdf.py \
        ~/RAON-RT-Revision/App/Indy7/indy7.urdf urdf/indy7_viz.urdf
"""

import math
import sys
import xml.etree.ElementTree as ET

# Taper: thick at the base, thin at the wrist. Cosmetic only.
RADII = [0.075, 0.065, 0.055, 0.045, 0.040, 0.035, 0.030]
MIN_LEN = 0.03          # a near-zero link still needs to be visible


def rpy_to_z(vx, vy, vz):
    """rpy that rotates +Z onto (vx,vy,vz). roll is free, so keep it 0."""
    n = math.sqrt(vx * vx + vy * vy + vz * vz)
    if n < 1e-9:
        return 0.0, 0.0, 0.0
    return 0.0, math.acos(max(-1.0, min(1.0, vz / n))), math.atan2(vy, vx)


def main(src, dst):
    tree = ET.parse(src)
    root = tree.getroot()

    # parent link -> child joint origin, in the parent link's own frame
    child_xyz = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link")
        origin = joint.find("origin")
        xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None
                                  else "0 0 0").split()]
        child_xyz[parent] = xyz

    # ros2_control is not URDF and only makes the parser noisy here
    for extra in root.findall("ros2_control"):
        root.remove(extra)

    for idx, link in enumerate(root.findall("link")):
        # broken mesh paths, and RViz never needs collision geometry
        for coll in link.findall("collision"):
            link.remove(coll)

        visual = link.find("visual")
        if visual is None:
            continue
        vx, vy, vz = child_xyz.get(link.get("name"), [0.0, 0.0, 0.0])
        length = max(MIN_LEN, math.sqrt(vx * vx + vy * vy + vz * vz))
        roll, pitch, yaw = rpy_to_z(vx, vy, vz)

        # cylinder is centred on its own origin, so sit it at the midpoint
        origin = visual.find("origin")
        if origin is None:
            origin = ET.SubElement(visual, "origin")
        origin.set("xyz", f"{vx / 2:.6f} {vy / 2:.6f} {vz / 2:.6f}")
        origin.set("rpy", f"{roll:.6f} {pitch:.6f} {yaw:.6f}")

        geom = visual.find("geometry")
        geom.clear()
        ET.SubElement(geom, "cylinder",
                      radius=f"{RADII[min(idx, len(RADII) - 1)]:.3f}",
                      length=f"{length:.6f}")

    tree.write(dst, encoding="unicode", xml_declaration=True)
    print(f"wrote {dst}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
