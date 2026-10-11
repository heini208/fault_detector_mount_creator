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

The original reference manifest gives the wrist role `fixed`; `front_jaw`,
`middle_jaw`, and `jaw_tooth` have role `stock_jaw`; the six moving finger parts
have role `finger`. These labels do not establish which physical parts a
particular mount replaces. That removable boundary remains unverified, so
`verified` is `false`. Do not infer planning-scene removal or collision exceptions
from the display roles. Reference geometry must never be exported as mount
collision geometry.

## Detailed viewport meshes and candidate holes

`preview.json` describes display-only assets, separately from the unchanged
`reference.json` template identity. The additional source meshes come from the
same repository and commit, under `spot_description/spot_description/meshes/arm/visual/`:

- `finger_visual.obj` is an unchanged copy of `arm_link_fngr.obj`, displayed at
  the same zero-angle finger transform listed above.
- `wrist_detailed.obj` and `jaw_extension_visual.obj` are clipped portions of
  `arm_link_wr1.obj`, in the original wrist coordinates. The source fuses the wrist
  and jaw. Triangles are split at wrist X = 0.146100 m, where the collision wrist
  ends and `middle_jaw` begins; no caps or holes are generated. The forward
  extension can be hidden, but the rear wrap under the wrist remains. This is an
  approximate preview split, not a physical removal model. Source hash, clipping
  criteria and surface areas are recorded in `visual_split_provenance.json`.
- `lower_jaw_mount_source.obj` is an unchanged copy of
  `arm_link_lower_jaw_realsense_mount.obj`, retained as evidence for the candidate
  hole overlay; it is not itself displayed as the stock wrist.

`mount_hole_candidates.json` records least-squares circle fits to the actual
accessory inner-side opening contours, including source vertex indices, radii,
fit residuals and its source hash. The accessory transform is defined in
`urdf/accessories.urdf.xacro`: wrist translation `[0.09, 0, -0.034]` m and rotation
Rz(pi/2). Therefore an accessory point maps to hand coordinates as
`[-0.10557 - y, x, z - 0.034]`. The candidate axes run along hand ±Y, with two
openings on each side.

The stock visual mesh has coarse, irregular features near those axes and cannot
independently establish precise screw centers. Small fitting residuals describe
agreement with the accessory triangles, not physical accuracy. The orange rings
are unverified reference annotations, offset outward by 1 mm for visibility; the
stored coordinates remain at the accessory's inner side planes. They cannot be
picked as sensor-tip surfaces and never enter exports. The jaw visibility and
hole overlay do not change the export's mounting-template identity.

The SHA-256 `fingerprint` identifies this exact reference. Hash, in sequence:
UTF-8 `reference.urdf.fragment`, a NUL byte, its file bytes, a NUL byte; then, for
each mesh in manifest order, its UTF-8 filename, a NUL byte, its unchanged file
bytes, and a NUL byte. `id` and `revision` identify the versioned template.

## Supplied starter mount

`starter_mount.stl` is an unchanged copy of the user-supplied
`shapr3d_export_2026-10-11_17h07m.stl` (SHA-256
`6c5eb185f4c6fc82b9ed6b783061994665fa2be703d2d3fb9730e5bbfefebdb6`).
It contains 3,360 triangles and is interpreted in millimetres. Bounds are
[0, -41, 0] to [158, 38, 85] mm. It passes the same structural mesh validation
as imported geometry; no repair, decimation, stretching or reshaping is applied.

`starter_mount.json` records the source units, hole measurements, rigid
`hand_to_geometry` transform and fit limitations. Two pairs of coaxial bores
have XY centers [7.5, 30.5] and [46.5, 23] mm, radius 2.1 mm, and axes along
source Z. The side plates occupy source Z [0, 3.5] and [81.5, 85] mm. A rigid
least-squares fit maps those four axes to the reference candidates in hand X/Z.
The mount is centered across hand Y at the midpoint of the reference side
planes (-0.508 mm); bore axes alone do not determine translation along Y.
Source X faces forward, source Y faces up, and source Z faces negative hand Y.
The resulting roll/pitch/yaw is [90, -1.17763687535, 0] degrees. The stored pose
uses metres and an xyzw quaternion. Runtime geometry stays in the original
source frame, converted to metres, with this pose stored once in the document.

Source hole pitch is 39.714607 mm, versus 39.628896 mm in the reference; the
largest perpendicular axis residual is 0.045796 mm. This is mesh agreement, not
physical calibration. The 78 mm source inner opening differs from the 84 mm
reference accessory opening. Under this placement, the starter plates overlap
the retained rear jaw surfaces in the approximate reference preview. Those
surfaces may also be removed in the actual assembly; the physical removal
boundary and fit still need verification. No geometry is stretched to resolve
that uncertainty.

The user identified all four highlighted locations as the normal sensor-mount
screw locations. This confirms the intended attachment, not a measured datum
or sensor-tip calibration. The tip remains unset in each new document. The
starter is ordinary editable document geometry and is included in exports;
the gray reference and orange annotations remain display-only.
