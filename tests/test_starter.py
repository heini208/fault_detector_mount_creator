"""The bundled mount keeps its source units and a single hand transform."""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from mount_creator import starter
from mount_creator.bundle import load_head, save_head
from mount_creator.geometry import Pose


class StarterTests(unittest.TestCase):
    def test_new_documents_have_independent_geometry_and_no_assumed_tip(self):
        first = starter.new_starter_document()
        second = starter.new_starter_document()
        expected = second.parts[0].mesh.copy()
        self.assertEqual(first.name, "Untitled head")
        self.assertIsNone(first.tip)
        self.assertEqual(first.parts[0].pose, Pose())
        first.parts[0].mesh[:] = 0
        first.parts.clear()
        np.testing.assert_array_equal(second.parts[0].mesh, expected)
        self.assertIsNone(second.tip)

    def test_source_dimensions_are_converted_from_millimetres(self):
        document = starter.new_starter_document()
        vertices = document.collision_mesh().reshape(-1, 3)
        np.testing.assert_allclose(vertices.min(axis=0), (0, -0.041, 0), atol=1e-8)
        np.testing.assert_allclose(vertices.max(axis=0), (0.158, 0.038, 0.085), atol=1e-8)

    def test_bore_axes_align_with_reference_hole_axes(self):
        document = starter.new_starter_document()
        assets = Path(starter.__file__).with_name("assets")
        reference = json.loads((assets / "mount_hole_candidates.json").read_text())
        source_axis_hand = document.placement.matrix()[:3, :3] @ (0, 0, 1)
        for hole in reference["holes"]:
            with self.subTest(hole=hole["id"]):
                x, y = (7.5, 30.5) if hole["id"].endswith("proximal") else (46.5, 23)
                z = 85 if hole["id"].startswith("negative_y") else 0
                center_hand = document.placement.apply(np.array((x, y, z)) * 0.001)
                axis_hand = np.array(hole["axis_hand"])
                difference = center_hand - hole["center_hand_m"]
                perpendicular = difference - np.dot(difference, axis_hand) * axis_hand
                self.assertLessEqual(np.linalg.norm(perpendicular), 0.00006)
                self.assertLess(np.linalg.norm(np.cross(source_axis_hand, axis_hand)), 1e-12)

    def test_export_preserves_source_geometry_and_hand_alignment(self):
        document = starter.new_starter_document()
        document.tip = Pose.from_rpy((0.15, 0.002, 0.05), (0.1, 0.2, 0.3))
        before_mesh = document.collision_mesh()
        before_world = document.placement.apply(before_mesh)
        before_tip = document.hand_to_probe()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "starter.sensorhead"
            save_head(path, document)
            loaded = load_head(path)
        self.assertEqual(len(loaded.parts), 1)
        self.assertEqual(loaded.parts[0].kind, "mesh")
        np.testing.assert_allclose(loaded.collision_mesh(), before_mesh, atol=1e-8)
        np.testing.assert_allclose(loaded.placement.apply(loaded.collision_mesh()),
                                   before_world, atol=1e-8)
        np.testing.assert_allclose(loaded.placement.matrix(), document.placement.matrix(), atol=1e-12)
        np.testing.assert_allclose(loaded.hand_to_probe().matrix(), before_tip.matrix(), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
