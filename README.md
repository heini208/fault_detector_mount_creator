# Fault detector mount creator

A small, standalone desktop editor for Spot sensor mounts. It imports an STL or
builds a mount from boxes, places it against a fixed gripper reference, and marks
the sensor tip and sensing direction. A single `.sensorhead` file preserves the
collision mesh, transforms, and editable source parts.

This first milestone runs independently of ROS. Integration with the sensor
registry, TF, and MoveIt belongs to `fault_detector_spot` and is not implemented
here.

## Run

Python 3.10 or newer, NumPy, PyQt5, and VTK are required. From this repository:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
mount-creator
```

If the dependencies are already installed, `python3 main.py` also works. The
desktop application needs a working graphical display and OpenGL. Development
checks use Python 3.10, NumPy 1.21, PyQt5 5.15, and VTK 9.1.

## First workflow

1. Choose **Add box** or **Import STL**. STL import asks for millimetres, metres,
   or inches; check the displayed part size before continuing.
2. Set box dimensions and each part's position and rotation. **Whole mount**
   places the complete assembly relative to Spot's hand reference.
3. Open **Sensor tip**. Pick a point on the mount or set a virtual point at the
   mount origin and enter its measured coordinates. Check the red sensing arrow,
   flip it if needed, and adjust the rotation. Local +X faces the inspected surface.
4. **Export head** writes a `.sensorhead` file. **Open head** restores its parts,
   dimensions, placement, and tip for further editing.

Moving the whole mount carries its tip. Editing the tip does not move the mesh.
Moving an individual part changes only that part, so check the tip again when
changing the assembly. Length controls use millimetres; file coordinates use
metres. Rotations in the UI use roll/pitch/yaw degrees; exported poses use xyzw
quaternions.

## Reference and current limits

The bundled reference uses the existing Spot `hand` datum. Stock lower-jaw parts
are translucent for context; reference meshes never enter the exported collision
mesh. The finger is shown in its closed reference pose. See
[asset provenance](mount_creator/assets/ASSET_SOURCES.md) for sources and transforms.

The exact physically removed jaw components have not been verified. Files carry
`mounting_template_verified: false`; they are development definitions, not an
approved runtime attachment profile. The eventual robot importer must require a
verified compatible template before activation.

This milestone provides numeric placement, STL import, boxes, tip picking, and
export/reopen. Cylinders, drag handles, guided landmark alignment, undo/redo, and
collision-envelope padding remain future work. Export uses the nominal mesh with
zero extra clearance. Imported meshes must be closed, consistently wound and
nondegenerate. Structural validation does not detect every self-intersection or
perform Boolean unions of overlapping parts.

Limits are 200,000 total collision triangles, 128 parts, 32 MiB per STL/archive,
and 128 MiB of expanded archive data. These are initial resource bounds, not
performance guarantees. File parsing and mesh validation run outside the UI
thread. A failed open or export preserves the previous document/file.

The 0.01 mm numerical round-trip test is separate from the physical 1 mm tip
accuracy target. Physical accuracy needs measured mounting references, assembly
repeatability, and validation on the real head.

## Code layout

| File | Responsibility |
| --- | --- |
| `geometry.py` | Rigid poses, box geometry, STL data, mesh checks. |
| `document.py` | The editable document and its parts. |
| `bundle.py` | Bounded, validated, atomic `.sensorhead` file operations. |
| `viewport.py` | VTK actors, camera, and surface picking. |
| `app.py` | Qt controls and explicit edits to the document. |

The code uses ordinary functions and dataclasses, one document owner, and direct
method calls. Keep units explicit, validate at file boundaries, and keep rendering
out of geometry calculations. Add an abstraction only when a concrete repeated
responsibility needs it; prefer a readable function over a framework or hierarchy.

## File format

`.sensorhead` is a ZIP containing `manifest.json` and `collision.stl`. The manifest
records schema version 1, template identity, units, the mesh hash, `hand_to_geometry`
and `hand_to_probe`. Mesh vertices are local to the geometry frame; placement is
applied once. The editor also writes `authoring.json` and normalized source STLs
to preserve editable parts. A runtime-only package without authoring data opens
as a single mesh. Inconsistent authoring and collision data are rejected.

The package has no sensor registry ID or acquisition settings. A later importer
will apply geometry and tip together to the sensor entry selected in the Spot UI.

## Offline checks

```bash
python3 -m unittest discover -s tests -v
```

Tests cover pose conventions, unit conversion, mesh validation, archive limits,
failed-write preservation, and export/reopen precision. UI tests use offscreen Qt
with a generic VTK render window, without rendering or connecting to a robot.
They check controls and scene data, not the desktop OpenGL rendering path.
