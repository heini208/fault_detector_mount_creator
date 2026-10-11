import tempfile
import unittest
import warnings
from pathlib import Path
from unittest.mock import patch

import numpy as np

from mount_creator.document import MAX_PARTS, Document, Part
from mount_creator.geometry import (
    MAX_TRIANGLES, Pose, box_mesh, direction_pose, mesh_from_stl, mesh_to_stl,
    read_stl, validate_mesh,
)


class PoseTests(unittest.TestCase):
    def test_composition_and_inverse_use_local_translation(self):
        parent = Pose.from_rpy((0.1, 0.2, 0.3), (0, 0, np.pi / 2))
        child = Pose((0.04, 0, 0))
        combined = parent.compose(child)
        np.testing.assert_allclose(combined.translation, (0.1, 0.24, 0.3), atol=1e-12)
        point = np.array((0.003, -0.002, 0.001))
        np.testing.assert_allclose(combined.apply(point), parent.apply(child.apply(point)), atol=1e-12)
        np.testing.assert_allclose(combined.inverse().apply(combined.apply(point)), point, atol=1e-12)

    def test_pose_round_trip_preserves_rotation_including_half_turns(self):
        for angles in ((0.3, -0.4, 2.1), (np.pi, 0, 0), (0, np.pi, 0),
                       (0, 0, np.pi), (0.4, np.pi / 2, 0.7), (0.4, -np.pi / 2, 0.7)):
            pose = Pose.from_rpy((0.012345678, -0.04, 0.09), angles)
            restored = Pose.from_dict(pose.to_dict())
            np.testing.assert_allclose(Pose.from_matrix(pose.matrix()).matrix(), pose.matrix(), atol=1e-12)
            np.testing.assert_allclose(Pose.from_rpy(restored.translation, restored.rpy()).matrix(), pose.matrix(), atol=1e-8)

    def test_direction_defines_positive_x_and_preserves_position(self):
        pose = direction_pose((0.01, -0.02, 0.03), (1, 2, 3))
        np.testing.assert_allclose(pose.matrix()[:3, 0], np.array((1, 2, 3)) / np.sqrt(14))
        np.testing.assert_allclose(pose.translation, (0.01, -0.02, 0.03))
        self.assertGreater(pose.matrix()[2, 2], 0)
        for direction in ((0, 0, 0), (0, 0, 1)):
            with self.assertRaises(ValueError):
                direction_pose((0, 0, 0), direction)

    def test_invalid_rigid_poses_are_rejected(self):
        for quaternion in ((0, 0, 0, 0), (0, 0, 0, 2), (0, 0, np.nan, 1)):
            with self.assertRaises(ValueError):
                Pose(quaternion=quaternion)
        for matrix in (np.diag((2, 1, 1, 1)), np.diag((-1, 1, 1, 1))):
            with self.assertRaises(ValueError):
                Pose.from_matrix(matrix)


class MeshTests(unittest.TestCase):
    def test_box_dimensions_closedness_and_disconnected_parts(self):
        box = box_mesh((0.02, 0.03, 0.04))
        np.testing.assert_allclose(np.ptp(box.reshape(-1, 3), axis=0), (0.02, 0.03, 0.04))
        validate_mesh(box)
        validate_mesh(np.concatenate((box, box + (0.1, 0, 0))))
        validate_mesh(np.concatenate((box, box + (0.02, 0, 0))))
        with self.assertRaises(ValueError):
            validate_mesh(box[:-1])

    def test_nonfinite_degenerate_and_inconsistently_wound_meshes_are_rejected(self):
        for defect in ("nan", "degenerate", "winding"):
            box = box_mesh((0.02, 0.03, 0.04))
            if defect == "nan":
                box[0, 0, 0] = np.nan
            elif defect == "degenerate":
                box[0, 0] = box[0, 1]
            else:
                box[0] = box[0, ::-1]
            with self.assertRaises(ValueError):
                validate_mesh(box)

    def test_zero_volume_component_is_rejected_even_beside_a_valid_box(self):
        triangle = np.array(((0, 0, 0), (0.01, 0, 0), (0, 0.01, 0)))
        flat_shell = np.array((triangle, triangle[::-1]))
        for mesh in (flat_shell, np.concatenate((flat_shell, box_mesh((0.02, 0.03, 0.04)) + 1))):
            with self.assertRaisesRegex(ValueError, "nonzero volume"):
                validate_mesh(mesh)
        # Separate closed components can have either winding orientation.
        box = box_mesh((0.02, 0.03, 0.04))
        validate_mesh(np.concatenate((box, box[:, ::-1] + 1)))

    def test_stl_float32_overflow_is_a_clean_validation_error(self):
        with warnings.catch_warnings():
            warnings.simplefilter("error")
            with self.assertRaisesRegex(ValueError, "float32 range"):
                mesh_to_stl(box_mesh((1e40, 1e40, 1e40)))

    def test_document_limits_are_checked_before_transforming_parts(self):
        repeated = Document(parts=[Part("Box")] * (MAX_PARTS + 1))
        oversized = Document(parts=[Part("Large mesh", kind="mesh", mesh=np.empty((MAX_TRIANGLES + 1, 3, 3)))])
        combined = Document(parts=[Part("Boxes", kind="mesh", mesh=np.empty((MAX_TRIANGLES - 1, 3, 3))), Part("Box")])
        for document in (repeated, oversized, combined):
            with patch.object(Part, "triangles", side_effect=AssertionError("Unexpected geometry allocation")):
                with self.assertRaises(ValueError):
                    document.collision_mesh()

    def test_rejected_addition_preserves_existing_parts(self):
        existing = Part("Existing box")
        document = Document(parts=[existing])
        oversized = Part("Large mesh", kind="mesh", mesh=np.empty((MAX_TRIANGLES, 3, 3)))
        with self.assertRaisesRegex(ValueError, "triangle"):
            document.add_part(oversized)
        self.assertEqual(len(document.parts), 1)
        self.assertIs(document.parts[0], existing)

    def test_stl_precision_and_explicit_source_units(self):
        mesh = Pose.from_rpy((0.023, -0.037, 0.11), (0.2, 0.3, 0.4)).apply(box_mesh((0.024, 0.031, 0.048)))
        recovered = mesh_from_stl(mesh_to_stl(mesh))
        self.assertLess(np.max(np.linalg.norm(recovered - mesh, axis=2)), 0.00001)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "millimetres.stl"
            path.write_bytes(mesh_to_stl(box_mesh((20, 30, 40))))
            np.testing.assert_allclose(read_stl(path, 0.001), box_mesh((0.02, 0.03, 0.04)))
            lines = ["solid box"]
            for triangle in box_mesh((20, 30, 40)):
                lines.extend(("facet normal 0 0 0", "outer loop"))
                lines.extend("vertex " + " ".join(map(str, vertex)) for vertex in triangle)
                lines.extend(("endloop", "endfacet"))
            lines.append("endsolid box")
            path.write_text("\n".join(lines))
            np.testing.assert_allclose(read_stl(path, 0.001), box_mesh((0.02, 0.03, 0.04)))

    def test_tip_edit_does_not_move_mesh_and_placement_is_applied_once(self):
        part = Part("Box", pose=Pose((0.02, 0, 0)))
        document = Document(parts=[part], placement=Pose.from_rpy((0.1, 0.2, 0.3), (0, 0, np.pi / 2)))
        geometry = document.collision_mesh().copy()
        document.tip = Pose((0.03, 0, 0))
        np.testing.assert_array_equal(document.collision_mesh(), geometry)
        np.testing.assert_allclose(document.hand_to_probe().translation, (0.1, 0.23, 0.3), atol=1e-12)
        np.testing.assert_allclose(np.mean(geometry, axis=(0, 1)), (0.02, 0, 0), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
