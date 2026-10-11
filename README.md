# Fault detector mount creator

A small, standalone desktop editor for Spot sensor mounts. It starts with a
bundled mount aligned to the gripper screw axes. Move or replace the STL, add
boxes or other meshes, and mark the sensor tip and sensing direction. A single `.sensorhead` file preserves the
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
desktop application needs a working graphical display and OpenGL. Press Ctrl+C
in its launch terminal to exit immediately, discarding unexported changes. Development
checks use Python 3.10, NumPy 1.21, PyQt5 5.15, and VTK 9.1.

## First workflow

1. Start from the preloaded **Starter mount**. **New** restores a fresh copy.
   Use **Remove selected part** to start empty, **Add box** to extend it, or
   **Import STL** to add another mesh. Import asks for millimetres, metres,
   or inches; check the displayed part size before continuing.
2. Set box dimensions and each part's position and rotation. **Whole mount**
   places the complete assembly relative to Spot's hand reference.
3. Open **Sensor tip**. Pick a point on the mount or set a virtual point at the
   mount origin and enter its measured coordinates. Check the red sensing arrow,
   flip it if needed, and adjust the rotation. Local +X faces the inspected surface.
4. **Export head** writes a `.sensorhead` file. **Open head** restores its parts,
   dimensions, placement, and tip for further editing.

The starter STL retains its original shape: its dimensions are read-only, but
its position and rotation are editable. To change its holes, wall thickness or
other built-in features, edit it in CAD and replace the mesh. Added boxes have
editable dimensions. The sensor tip starts unset; choose its actual sensing
point before exporting.

Moving the whole mount carries its tip. Editing the tip does not move the mesh.
Moving an individual part changes only that part, so check the tip again when
changing the assembly. Length controls use millimetres; file coordinates use
metres. Rotations in the UI use roll/pitch/yaw degrees; exported poses use xyzw
quaternions.

## Starter mount

The supplied Shapr3D STL is bundled unchanged (3,360 triangles), interpreted in
millimetres, and placed with a rigid transform. Its four screw axes match the
reference candidates within 0.046 mm in the model. The mesh remains in its source
frame; **Whole mount** holds the alignment to `hand`, applied once on display and
recorded in the export.

This numerical fit does not establish physical accuracy. The STL has a 78 mm
inside opening, while the reference accessory suggests 84 mm. Its plates overlap
retained jaw material in the current approximate preview. That material may be
part of the jaw being replaced; check the actual assembly before treating the
alignment as a calibrated datum. The STL is not stretched to conceal this
difference. Alignment details are in `assets/starter_mount.json`.

![Supplied starter mount aligned to the gripper](documentation/starter_mount_preview.png)

## Reference and current limits

The viewport uses the more detailed Spot visual meshes at the existing `hand`
datum. The forward lower-jaw extension is hidden by default; **Show lower-jaw
extension** restores it. The source mesh fuses the jaw and wrist, so the split is
an approximate display cut, not a verified disassembly boundary. The rear wrap
under the wrist remains visible. The finger is shown in its closed reference pose.

Four orange rings highlight mounting-hole axes fitted from a lower-jaw camera
accessory mesh. These are the four locations identified for the sensor mount;
their numerical positions remain model-derived. The stock mesh does not
establish precise circular bores there. The rings are lifted
1 mm outward for visibility without changing the stored candidate coordinates.
**Highlight candidate mounting holes** toggles the overlay. Neither the detailed
reference meshes nor the highlights enter exported collision geometry. See
[asset provenance](mount_creator/assets/ASSET_SOURCES.md) for sources and transforms.

![Detailed gripper preview with candidate mounting holes](documentation/detailed_mount_preview.png)

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
performance guarantees. User file parsing and mesh validation run outside the UI
thread; the small bundled starter is loaded when creating a new document. A failed open or export preserves the previous document/file.

The 0.01 mm numerical round-trip test is separate from the physical 1 mm tip
accuracy target. Physical accuracy needs measured mounting references, assembly
repeatability, and validation on the real head.

## Code layout

| File | Responsibility |
| --- | --- |
| `geometry.py` | Rigid poses, box geometry, STL data, mesh checks. |
| `document.py` | The editable document and its parts. |
| `starter.py` | A fresh starter document from the bundled STL and alignment. |
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
failed-write preservation, starter screw-axis alignment, and export/reopen precision. UI tests use offscreen Qt
with a generic VTK render window, without rendering or connecting to a robot.
They check controls and scene data, not the desktop OpenGL rendering path.
