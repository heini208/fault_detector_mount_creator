"""Check sequential surface measurements with real controls, without rendering."""

import os
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch

os.environ["QT_QPA_PLATFORM"] = "offscreen"

import numpy as np
from PyQt5.QtWidgets import QApplication, QMessageBox
from vtkmodules.qt import QVTKRenderWindowInteractor as qt_vtk
from vtkmodules.vtkRenderingAnnotation import vtkAxesActor
from vtkmodules.vtkRenderingOpenGL2 import vtkGenericOpenGLRenderWindow

from mount_creator.app import MainWindow
from mount_creator.bundle import save_head
from mount_creator.document import Document, Part
from mount_creator.geometry import Pose, box_mesh, plane_from_triangle, surface_axis_distance


class MeasurementTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def setUp(self):
        with patch.object(qt_vtk, "vtkRenderWindow", vtkGenericOpenGLRenderWindow):
            self.window = MainWindow()
        self.window.tabs.setCurrentIndex(1)
        self.part_pose = Pose.from_rpy((0.03, -0.012, 0.023), (0.4, -0.3, 0.7))
        self.placement = Pose.from_rpy((-0.11, 0.02, 0.01), (-0.2, 0.5, -0.9))
        self.local_tip = np.array((0.01, 0.02, 0.03))
        self._load_document()

    def tearDown(self):
        self.window.dirty = False
        self.window.close()
        self.window.deleteLater()
        self.application.processEvents()

    def _load_document(self, with_tip=True, oblique=False):
        part = Part("Measured box", dimensions=(0.04, 0.03, 0.02), pose=self.part_pose)
        if oblique:
            rotation = Pose.from_rpy((0, 0, 0), (0.4, -0.3, 0.7))
            part = Part("Oblique mesh", kind="mesh", pose=self.part_pose,
                        mesh=rotation.apply(box_mesh(part.dimensions)))
        tip = (Pose.from_rpy(self.part_pose.apply(self.local_tip), (0.2, 0.1, -0.5))
               if with_tip else None)
        self.window._replace_document(Document(parts=[part, Part("Other box")],
                                               placement=self.placement, tip=tip))
        self.window.part_list.setCurrentRow(0)

    def _pick_surface(self, cell_id=3, part_index=0):
        part = self.window.document.parts[part_index]
        triangle = self.window.document.placement.apply(part.triangles()[cell_id])
        self.window._surface_picked(part_index, cell_id, triangle)

    def _distance(self):
        tip = self.window.reference_part.pose.inverse().apply(self.window.document.tip.translation)
        return surface_axis_distance(self.window.reference_point, self.window.reference_normal,
                                     tip, self.window.measure_axis.currentIndex())

    def test_tabs_show_only_the_axes_for_building_or_surface_measurement(self):
        self._pick_surface()
        before = self.window.document.hand_to_probe().matrix().copy()
        self.assertFalse(self.window.viewport.edit_axes.GetVisibility())
        self.window.tabs.setCurrentIndex(0)
        self.assertTrue(self.window.viewport.edit_axes.GetVisibility())
        self.assertEqual(self.window.viewport.measurement_actors, [])
        self.window.tabs.setCurrentIndex(1)
        self.assertFalse(self.window.viewport.edit_axes.GetVisibility())
        axes, = [actor for actor in self.window.viewport.measurement_actors
                 if isinstance(actor, vtkAxesActor)]
        self.assertEqual(axes.GetYAxisLabelText(), "Part Y")
        np.testing.assert_array_equal(self.window.document.hand_to_probe().matrix(), before)
        self.assertFalse(self.window.dirty)

    def test_pick_tip_first_then_reference_on_any_part_without_build_selection(self):
        self._load_document(with_tip=False)
        self.assertFalse(self.window.reference_button.isEnabled())
        self.assertFalse(self.window.measure_distance.isEnabled())
        self._pick_surface()
        self.assertIsNone(self.window.reference_part)
        triangle = self.placement.apply(self.window.document.parts[0].triangles()[3])
        _, normal = plane_from_triangle(triangle)
        self.window.pick_button.click()
        self.assertEqual(self.window.viewport.pick_mode, "tip")
        self.window._tip_picked(triangle.mean(axis=0), normal)
        np.testing.assert_allclose(self.window.document.hand_to_probe().translation,
                                   triangle.mean(axis=0), atol=1e-12)
        self.assertTrue(self.window.reference_button.isEnabled())
        self.assertIsNone(self.window.viewport.pick_mode)

        self.window.reference_button.click()
        self.assertEqual(self.window.viewport.pick_mode, "surface")
        self._pick_surface(part_index=1)
        self.assertEqual(self.window.part_list.currentRow(), 0)
        self.assertIs(self.window.reference_part, self.window.document.parts[1])
        self.assertIsNone(self.window.viewport.pick_mode)
        self.window.part_list.setCurrentRow(1)
        self.assertIs(self.window.reference_part, self.window.document.parts[1])

    def test_surface_and_axis_selection_are_read_only_and_do_not_change_exports(self):
        self._load_document(oblique=True)
        original = self.window.document.tip
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "head.sensorhead"
            save_head(path, self.window.document)
            with zipfile.ZipFile(path) as archive:
                before = {name: archive.read(name) for name in archive.namelist()}
            self._pick_surface()
            expected_axis = int(np.argmax(np.abs(self.window.reference_normal)))
            self.assertEqual(self.window.measure_axis.currentIndex(), expected_axis)
            for axis in range(3):
                self.window.measure_axis.setCurrentIndex(axis)
                self.assertAlmostEqual(self.window.measure_distance.value(), self._distance() * 1000,
                                       delta=0.000051)
            self._pick_surface(cell_id=1)
            self.assertEqual(self.window.document.tip, original)
            self.assertFalse(self.window.dirty)
            self.assertTrue(self.window.viewport.measurement_actors)
            self.assertTrue(all(not actor.GetPickable()
                                for actor in self.window.viewport.measurement_actors))
            save_head(path, self.window.document)
            with zipfile.ZipFile(path) as archive:
                self.assertEqual(before, {name: archive.read(name) for name in archive.namelist()})

    def test_distance_edit_moves_only_chosen_part_axis_and_preserves_orientation(self):
        self._load_document(oblique=True)
        self._pick_surface()
        self.window.measure_axis.setCurrentIndex(1)
        original = self.window.document.tip
        original_local = self.part_pose.inverse().apply(original.translation)
        mesh = self.window.document.collision_mesh().copy()
        for requested in (-12.3456, 0, 7.125):
            self.window.measure_distance.setValue(requested)
            local = self.part_pose.inverse().apply(self.window.document.tip.translation)
            np.testing.assert_allclose(local[[0, 2]], original_local[[0, 2]], atol=1e-12)
            self.assertAlmostEqual(self._distance(), requested / 1000, places=12)
            np.testing.assert_allclose(self.window.document.tip.quaternion, original.quaternion,
                                       atol=1e-15)
            np.testing.assert_allclose(self.window.document.hand_to_probe().translation,
                                       self.placement.apply(self.part_pose.apply(local)), atol=1e-12)
        np.testing.assert_array_equal(self.window.document.collision_mesh(), mesh)
        self.assertTrue(self.window.dirty)

    def test_sequential_faces_adjust_three_coordinates_and_disable_parallel_axes(self):
        # Box faces at local X=20, Y=15 and Z=10 mm.
        for cell_id, axis, distance in ((3, 0, -5), (4, 1, 2), (1, 2, 3)):
            self._pick_surface(cell_id)
            self.assertEqual(self.window.measure_axis.currentIndex(), axis)
            for candidate in range(3):
                self.assertEqual(self.window.measure_axis.model().item(candidate).isEnabled(),
                                 candidate == axis)
            self.window.measure_distance.setValue(distance)
        expected = np.array((0.015, 0.017, 0.013))
        np.testing.assert_allclose(self.part_pose.inverse().apply(self.window.document.tip.translation),
                                   expected, atol=1e-12)
        original = self.window.document.tip
        self.window.measure_axis.setCurrentIndex(0)  # Even a programmatic invalid selection is safe.
        self.assertFalse(self.window.measure_distance.isEnabled())
        self.assertEqual(self.window.document.tip, original)

    def test_repicked_tip_refreshes_distance_and_whole_mount_move_keeps_it(self):
        self._pick_surface()
        position = list(self.window.document.tip.translation)
        position[0] = 0.085
        self.window._tip_picked(self.placement.apply(position), np.array((0, 0, 1)))
        self.assertAlmostEqual(self.window.measure_distance.value(), self._distance() * 1000,
                               delta=0.000051)
        before = self._distance()
        original_tip = self.window.document.tip
        self.window.mount_pose.position.fields[0].setValue(120)
        self.window.mount_pose.rotation.fields[1].setValue(50)
        self.assertAlmostEqual(self._distance(), before, places=12)
        self.assertEqual(self.window.document.tip, original_tip)
        axes = next(actor for actor in self.window.viewport.measurement_actors
                    if isinstance(actor, vtkAxesActor))
        actual = np.array([[axes.GetUserMatrix().GetElement(row, column)
                            for column in range(4)] for row in range(4)])
        local_tip = self.part_pose.inverse().apply(original_tip.translation)
        base = local_tip.copy()
        base[self.window.measure_axis.currentIndex()] -= before
        frame = self.window.document.placement.compose(self.part_pose)
        np.testing.assert_allclose(actual, frame.compose(Pose(base)).matrix(), atol=1e-12)

    def test_geometry_changes_and_new_document_clear_stale_surface(self):
        actions = {
            "deletion": self.window._remove_part,
            "resize": lambda: self.window.dimensions.fields[0].setValue(45),
            "part movement": lambda: self.window.part_pose.position.fields[0].setValue(50),
            "new": self.window._new,
        }
        for name, action in actions.items():
            with self.subTest(action=name):
                self._load_document()
                self._pick_surface()
                with patch.object(QMessageBox, "question", return_value=QMessageBox.Discard):
                    action()
                self.assertIsNone(self.window.reference_part)
                self.assertIsNone(self.window.reference_point)
                self.assertIsNone(self.window.reference_normal)
                self.assertFalse(self.window.measure_distance.isEnabled())
                self.assertEqual(self.window.viewport.measurement_actors, [])

    def test_tip_and_reference_pick_modes_are_exclusive_and_cancellable(self):
        self.window.pick_button.click()
        self.assertEqual(self.window.viewport.pick_mode, "tip")
        self.window.reference_button.click()
        self.assertFalse(self.window.pick_button.isChecked())
        self.assertEqual(self.window.viewport.pick_mode, "surface")
        self.window.reference_button.click()
        self.assertIsNone(self.window.viewport.pick_mode)
        self.window.pick_button.click()
        self.assertFalse(self.window.reference_button.isChecked())
        self.assertEqual(self.window.viewport.pick_mode, "tip")


if __name__ == "__main__":
    unittest.main()
