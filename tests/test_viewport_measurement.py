"""Check surface measurements and picking without an OpenGL context."""

import os
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch

os.environ["QT_QPA_PLATFORM"] = "offscreen"

import numpy as np
from PyQt5.QtWidgets import QApplication
from vtkmodules.qt import QVTKRenderWindowInteractor as qt_vtk
from vtkmodules.vtkRenderingAnnotation import vtkAxesActor
from vtkmodules.vtkRenderingCore import vtkActor, vtkBillboardTextActor3D
from vtkmodules.vtkRenderingOpenGL2 import vtkGenericOpenGLRenderWindow

from mount_creator.bundle import save_head
from mount_creator.document import Document, Part
from mount_creator.geometry import Pose, box_mesh
from mount_creator.viewport import Viewport


class SurfaceViewportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def setUp(self):
        with patch.object(qt_vtk, "vtkRenderWindow", vtkGenericOpenGLRenderWindow):
            self.viewport = Viewport()
        self.viewport.GetRenderWindow().SetSize(800, 600)
        self.part = Part("Box", pose=Pose.from_rpy((0.015, 0.006, 0.04), (0.3, 0.2, 0.7)))
        self.document = Document(parts=[self.part],
                                 placement=Pose.from_rpy((0.02, -0.1, 0.04), (0.7, 0.2, 0.1)))
        self.viewport.set_document(self.document)

    def tearDown(self):
        self.viewport.close()
        self.viewport.deleteLater()
        self.application.processEvents()

    def test_edit_axes_follow_named_frame_without_changing_document_or_export(self):
        self.document.tip = Pose((0.01, 0.02, 0.03))
        mesh_before = self.document.collision_mesh().copy()
        tip_before = self.document.hand_to_probe()
        axes = self.viewport.edit_axes
        frames = (("Hand", Pose()), ("Mount", self.document.placement),
                  ("Part", self.document.placement.compose(self.part.pose)))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "head.sensorhead"
            save_head(path, self.document)
            with zipfile.ZipFile(path) as archive:
                before = {name: archive.read(name) for name in archive.namelist()}
            for name, frame in frames:
                with self.subTest(frame=name):
                    self.viewport.set_edit_frame(frame, name)
                    actual = np.array([[axes.GetMatrix().GetElement(row, column)
                                        for column in range(4)] for row in range(4)])
                    np.testing.assert_allclose(actual, frame.matrix(), atol=1e-12)
                    self.assertEqual((axes.GetXAxisLabelText(), axes.GetYAxisLabelText(),
                                      axes.GetZAxisLabelText()),
                                     tuple(f"{name} {axis}" for axis in "XYZ"))
                    self.assertTrue(axes.GetVisibility())
                    self.assertFalse(axes.GetPickable())
            save_head(path, self.document)
            with zipfile.ZipFile(path) as archive:
                self.assertEqual(before, {name: archive.read(name) for name in archive.namelist()})
        np.testing.assert_array_equal(self.document.collision_mesh(), mesh_before)
        self.assertEqual(self.document.hand_to_probe(), tip_before)

    def test_surface_pick_uses_exact_triangle_and_both_part_and_mount_transforms(self):
        results = []
        self.viewport.surface_picked.connect(lambda *args: results.append(args))
        with patch("mount_creator.viewport.vtkCellPicker") as picker_type:
            picker = picker_type.return_value
            picker.Pick.return_value = True
            picker.GetActor.return_value = self.viewport.parts[0]
            picker.GetCellId.return_value = 2
            picker.GetPickPosition.return_value = (999, 999, 999)
            picker.GetPickNormal.return_value = (0, 0, 0)
            self.viewport.set_pick_mode("surface")
            self.viewport._pick(None, None)
            picker.GetPickPosition.assert_not_called()
            picker.GetPickNormal.assert_not_called()
        self.assertEqual(len(results), 1)
        part_index, cell_id, triangle = results[0]
        self.assertEqual((part_index, cell_id), (0, 2))
        expected = self.document.placement.apply(self.part.triangles()[2])
        np.testing.assert_allclose(triangle, expected, atol=1e-12)

    def test_both_pick_modes_include_all_authored_parts_and_exclude_annotations(self):
        self.document.parts.append(Part("Other box"))
        self.viewport.set_document(self.document)
        self.viewport.set_surface_measurement(0, 0)
        with patch("mount_creator.viewport.vtkCellPicker") as picker_type:
            picker = picker_type.return_value
            picker.Pick.return_value = False
            for mode in ("tip", "surface"):
                picker.reset_mock()
                self.viewport.set_pick_mode(mode)
                self.viewport._pick(None, None)
                picked_actors = [call.args[0] for call in picker.AddPickList.call_args_list]
                self.assertEqual(picked_actors, self.viewport.parts)
            self.assertTrue(all(not actor.GetPickable()
                                for actor in self.viewport.measurement_actors))
        with self.assertRaises(ValueError):
            self.viewport.set_pick_mode("reference")

    def test_invalid_surface_cell_reports_failure_without_emitting_a_surface(self):
        failures, results = [], []
        self.viewport.pick_failed.connect(failures.append)
        self.viewport.surface_picked.connect(lambda *args: results.append(args))
        with patch("mount_creator.viewport.vtkCellPicker") as picker_type:
            picker = picker_type.return_value
            picker.Pick.return_value = True
            picker.GetActor.return_value = self.viewport.parts[0]
            picker.GetCellId.return_value = -1
            self.viewport.set_pick_mode("surface")
            self.viewport._pick(None, None)
        self.assertEqual(results, [])
        self.assertEqual(len(failures), 1)

    def test_tip_pick_preserves_surface_hit_and_normal(self):
        results = []
        self.viewport.tip_picked.connect(lambda *args: results.append(args))
        with patch("mount_creator.viewport.vtkCellPicker") as picker_type:
            picker = picker_type.return_value
            picker.Pick.return_value = True
            picker.GetPickPosition.return_value = (0.0123456, 0.04, -0.03)
            picker.GetPickNormal.return_value = (0, 1, 0)
            self.viewport.set_pick_mode("tip")
            self.viewport._pick(None, None)
        self.assertEqual(len(results), 1)
        np.testing.assert_array_equal(results[0][0], (0.0123456, 0.04, -0.03))
        np.testing.assert_array_equal(results[0][1], (0, 1, 0))

    def test_highlight_covers_coplanar_face_and_retains_actor_transform(self):
        self.viewport.set_surface_measurement(0, 0)
        self.assertTrue(self.viewport.edit_axes.GetVisibility())
        overlay, = self.viewport.measurement_actors
        data = overlay.GetMapper().GetInput()
        self.assertEqual(data.GetNumberOfCells(), 2)
        world_points = np.array([overlay.GetMatrix().MultiplyPoint((*data.GetPoint(i), 1))[:3]
                                 for i in range(data.GetNumberOfPoints())])
        local_points = self.document.placement.compose(self.part.pose).inverse().apply(world_points)
        np.testing.assert_allclose(local_points[:, 2], -0.01, atol=1e-12)
        self.assertLess(overlay.GetProperty().GetOpacity(), 1)
        self.assertFalse(overlay.GetPickable())
        self.viewport.set_surface_measurement()
        self.assertEqual(self.viewport.measurement_actors, [])
        self.assertFalse(self.viewport.renderer.HasViewProp(overlay))

    def test_oblique_surface_ruler_follows_one_part_axis_and_leaves_geometry_unchanged(self):
        rotation = Pose.from_rpy((0, 0, 0), (0, 0, np.pi / 4))
        self.part.kind = "mesh"
        self.part.mesh = rotation.apply(box_mesh((0.02, 0.02, 0.02)))
        self.document.tip = Pose((0.02, 0.01, 0.08))
        self.viewport.set_document(self.document)
        before = self.document.collision_mesh().copy()
        tip_before = self.document.tip
        frame = self.document.placement.compose(self.part.pose)
        point = np.array((0, -np.sqrt(2) * 0.01, -0.01))
        normal = np.array((1, -1, 0)) / np.sqrt(2)
        tip = np.array((0.03, 0.04, 0.05))
        base = np.array((0.04 + np.sqrt(2) * 0.01, 0.04, 0.05))
        self.viewport.set_surface_measurement(0, 2, frame, point, normal, tip, 0)
        actors = self.viewport.measurement_actors
        axes = next(actor for actor in actors if isinstance(actor, vtkAxesActor))
        expected_axes = frame.compose(Pose(base)).matrix()
        actual_axes = [[axes.GetMatrix().GetElement(row, col) for col in range(4)]
                       for row in range(4)]
        np.testing.assert_allclose(actual_axes, expected_axes, atol=1e-12)
        self.assertEqual((axes.GetXAxisLabelText(), axes.GetYAxisLabelText(),
                          axes.GetZAxisLabelText()), ("Part X", "Part Y", "Part Z"))
        self.assertFalse(self.viewport.edit_axes.GetVisibility())
        self.viewport.set_edit_frame(Pose(), "Hand")
        self.assertFalse(self.viewport.edit_axes.GetVisibility())
        labels = [actor.GetInput() for actor in actors
                  if isinstance(actor, vtkBillboardTextActor3D)]
        self.assertEqual(labels, ["X -24.142 mm"])
        lines = self.dimension_lines()
        self.assertEqual(len(lines), 1)
        np.testing.assert_allclose(lines[0].GetPoint(0), frame.apply(base), atol=1e-8)
        np.testing.assert_allclose(lines[0].GetPoint(1), frame.apply(tip), atol=1e-8)
        self.assertAlmostEqual(np.dot(base - point, normal), 0)
        np.testing.assert_array_equal(self.document.collision_mesh(), before)
        self.assertEqual(self.document.tip, tip_before)
        self.assertTrue(all(not actor.GetPickable() for actor in actors))
        self.viewport.set_surface_measurement()
        self.assertTrue(self.viewport.edit_axes.GetVisibility())
        self.assertFalse(self.viewport.renderer.HasViewProp(axes))

    def test_zero_distance_has_a_label_without_a_degenerate_line(self):
        self.viewport.set_surface_measurement(0, 0, Pose(), (0, 0, 0), (0, 0, 1),
                                              (0.01, 0.02, 0), 2)
        labels = [actor.GetInput() for actor in self.viewport.measurement_actors
                  if isinstance(actor, vtkBillboardTextActor3D)]
        self.assertEqual(labels, ["Z +0.000 mm"])
        self.assertEqual(self.dimension_lines(), [])

    def dimension_lines(self):
        return [actor.GetMapper().GetInput() for actor in self.viewport.measurement_actors
                if isinstance(actor, vtkActor) and actor.GetMapper() is not None
                and actor.GetMapper().GetInput().GetNumberOfLines()]


if __name__ == "__main__":
    unittest.main()
