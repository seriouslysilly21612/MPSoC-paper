#!/usr/bin/env python3
"""Generate indy7_mesh.urdf — the robot app's URDF with WORKING mesh paths.

The robot app's indy7.urdf points every <mesh> at
/home/user/indy-ros2/install/... which exists on nobody's machine. The mesh
FILES themselves (Indy7_0..6.stl, neuromeka-robotics/indy-ros2
jazzy-indyDCP3, indy_description) are now vendored under this package at
meshes/indy7/visual/, so this script only rewrites each visual mesh path to
package://raon_operator_gui/... — resolvable via the ament index on any
machine where this package is installed.

Kinematics are byte-identical to what the RT controller solves against; only
paths change. Also dropped: <collision> (meshes not vendored — RViz never
looks at them) and <ros2_control> (not URDF; parser noise).

Re-run after any kinematic edit to the robot app's URDF:

    python3 urdf/make_mesh_urdf.py \
        ~/RAON-RT-Revision/App/Indy7/indy7.urdf urdf/indy7_mesh.urdf
"""

import posixpath
import sys
import xml.etree.ElementTree as ET

PKG_PREFIX = "package://raon_operator_gui/meshes/indy7/visual/"


def main(src, dst):
    tree = ET.parse(src)
    root = tree.getroot()

    for extra in root.findall("ros2_control"):
        root.remove(extra)

    n_meshes = 0
    for link in root.findall("link"):
        for coll in link.findall("collision"):
            link.remove(coll)
        for mesh in link.iter("mesh"):
            fname = posixpath.basename(mesh.get("filename", ""))
            if not fname:
                raise SystemExit(f"mesh without filename in {link.get('name')}")
            mesh.set("filename", PKG_PREFIX + fname)
            n_meshes += 1

    tree.write(dst, encoding="unicode", xml_declaration=True)
    print(f"wrote {dst} ({n_meshes} mesh refs -> {PKG_PREFIX})")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
