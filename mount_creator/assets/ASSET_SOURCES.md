# Spot hand reference assets

These ten OBJ files are unchanged collision meshes from
[`bdaiinstitute/spot_ros2`](https://github.com/bdaiinstitute/spot_ros2), commit
`77160178c644bd3ce7175e0ab027ee8a8a529088`, under
`spot_description/spot_description/meshes/arm/collision/`.
The supplied package license is retained in `SPOT_DESCRIPTION_LICENSE`.
These simplified collision meshes are reference displays, not a measured CAD
model or a promise of millimetre accuracy.

Placement comes from the local `spot_moveit_config/config/spot.urdf`, captured in
`reference.urdf.fragment`. The `spot_moveit_config` checkout was at commit
`436a32557bc5c587c2c61f49cfd076a7eb6c4f2e`; the captured fragment is authoritative
if that checkout had local changes. No ROS installation or URDF parsing is
required at runtime.

All manifest translations and mesh coordinates use metres. Quaternions use
`[x, y, z, w]`. Each pose maps a mesh's original coordinates into `hand`.
The URDF places `hand` 0.19557 m along wrist +X with identical orientation:

- Wrist and stock jaw meshes: translation `[-0.19557, 0, 0]`.
- Moving finger meshes: translation `[-0.07812, 0, 0.01482]`, obtained from the
  finger joint origin `[0.11745, 0, 0.01482]` minus the hand offset.
- All rotations are identity. The moving finger is displayed at joint angle
  zero, the closed reference pose; its motion or swept volume is not represented.

The wrist has role `fixed`; `front_jaw`, `middle_jaw`, and `jaw_tooth` have role
`stock_jaw`; the six moving finger parts have role `finger`. The stock jaw is
ghosted placement context. These labels do not establish which physical parts a
particular mount replaces. That removable boundary remains unverified, so
`verified` is `false`. Do not infer planning-scene removal or collision exceptions
from the display roles. Reference geometry must never be exported as mount
collision geometry.

The SHA-256 `fingerprint` identifies this exact reference. Hash, in sequence:
UTF-8 `reference.urdf.fragment`, a NUL byte, its file bytes, a NUL byte; then, for
each mesh in manifest order, its UTF-8 filename, a NUL byte, its unchanged file
bytes, and a NUL byte. `id` and `revision` identify the versioned template.
