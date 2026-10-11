"""Exercise real Qt controls and VTK scene data without a display or rendering."""

import os
import tempfile
import time
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch

os.environ["QT_QPA_PLATFORM"] = "offscreen"

import numpy as np
from PyQt5.QtWidgets import QApplication, QMessageBox
from vtkmodules.qt import QVTKRenderWindowInteractor as qt_vtk
from vtkmodules.vtkRenderingOpenGL2 import vtkGenericOpenGLRenderWindow

from mount_creator.app import MainWindow
from mount_creator.bundle import load_head, save_head
from mount_creator.document import Document, Part
from mount_creator.geometry import Pose


class EditorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def setUp(self):
        # Only replace the platform render window. Controls, actors and picking
        # data remain real; these tests deliberately never initialize OpenGL.
        with patch.object(qt_vtk, "vtkRenderWindow", vtkGenericOpenGLRenderWindow):
            self.window = MainWindow()

    def tearDown(self):
        self.window.dirty = False
        self.window.close()
        self.window.deleteLater()
        self.application.processEvents()

    def test_refresh_preserves_precision_and_large_loaded_coordinates(self):
        pose = Pose.from_rpy((3.123456789, -0.02, 0.04), (0.1, 0.2, 0.3))
        self.window._replace_document(Document(parts=[Part("Box")], placement=pose, tip=Pose()))
        self.assertEqual(self.window.document.placement, pose)
        self.assertFalse(self.window.dirty)
        self.assertGreater(self.window.mount_pose.position.fields[0].maximum(), 3123)
        self.window.mount_pose.position.fields[1].setValue(12)
        self.assertAlmostEqual(self.window.document.placement.translation[0], pose.translation[0], places=12)
        self.assertAlmostEqual(self.window.document.placement.translation[1], 0.012)

    def test_rotated_mount_pick_sets_local_tip_and_correct_world_direction(self):
        placement = Pose.from_rpy((0.2, -0.1, 0.3), (0.2, 0.3, 0.8))
        self.window._replace_document(Document(parts=[Part("Box")], placement=placement))
        point = placement.apply((0.01, 0.02, 0.03))
        normal = placement.matrix()[:3, 0]
        before = self.window.document.collision_mesh().copy()
        self.window._tip_picked(point, normal)
        np.testing.assert_allclose(self.window.document.tip.translation, (0.01, 0.02, 0.03))
        np.testing.assert_allclose(self.window.document.hand_to_probe().matrix()[:3, 0], normal)
        self.window._flip_tip()
        np.testing.assert_allclose(self.window.document.hand_to_probe().matrix()[:3, 0], -normal)
        np.testing.assert_array_equal(self.window.document.collision_mesh(), before)

    def test_part_edits_and_whole_mount_have_separate_effects(self):
        self.window._replace_document(Document())
        self.window._add_box()
        self.window._add_box()
        self.window._tip_at_origin()
        self.window.part_pose.position.fields[0].setValue(25)
        self.assertEqual(self.window.document.parts[0].pose.translation, (0, 0, 0))
        self.assertEqual(self.window.document.parts[1].pose.translation, (0.025, 0, 0))
        tip = self.window.document.tip
        mesh = self.window.document.collision_mesh().copy()
        self.window.mount_pose.position.fields[1].setValue(50)
        self.assertEqual(self.window.document.tip, tip)
        np.testing.assert_array_equal(self.window.document.collision_mesh(), mesh)
        np.testing.assert_allclose(self.window.document.hand_to_probe().translation, (0, 0.05, 0))
        self.assertEqual(len(self.window.viewport.parts), 2)
        self.assertEqual(len(self.window.viewport.markers), 2)

    def test_background_export_then_reopen(self):
        self.window._replace_document(Document())
        self.window._add_box()
        self.window._tip_at_origin()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "example.sensorhead"
            self.window._run_task("Exporting", lambda: save_head(path, self.window.document),
                                  self.window._exported)
            self._finish_worker()
            self.assertFalse(self.window.dirty)
            self.assertTrue(self.window.isEnabled())
            self.window._run_task("Opening", lambda: load_head(path), self.window._replace_document)
            self._finish_worker()
            self.assertEqual(self.window.document.parts[0].kind, "box")
            self.assertIsNotNone(self.window.document.tip)

    def test_oversized_assembly_is_rejected_before_changing_scene(self):
        self.window._replace_document(Document())
        with patch("mount_creator.document.MAX_PARTS", 1):
            self.window._add_box()
            with patch.object(QMessageBox, "warning") as warning:
                self.window._add_box()
                warning.assert_called_once()
        self.assertEqual(len(self.window.document.parts), 1)
        self.assertEqual(len(self.window.viewport.parts), 1)

    def test_reference_toggles_do_not_change_export_or_tip_picking(self):
        self.window._add_box()
        self.window._tip_at_origin()
        viewport = self.window.viewport
        self.assertEqual(len(viewport.hole_markers), 4)
        self.assertTrue(all(not actor.GetVisibility() for actor in viewport.reference_actors["lower_jaw"]))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "head.sensorhead"
            save_head(path, self.window.document)
            with zipfile.ZipFile(path) as archive:
                before = {name: archive.read(name) for name in archive.namelist()}
            self.window.show_lower_jaw.setChecked(True)
            self.window.show_holes.setChecked(False)
            self.assertTrue(all(actor.GetVisibility() for actor in viewport.reference_actors["lower_jaw"]))
            self.assertTrue(all(not actor.GetVisibility() for actor in viewport.hole_markers))
            self.assertTrue(all(not actor.GetPickable() for actor in viewport.hole_markers))
            save_head(path, self.window.document)
            with zipfile.ZipFile(path) as archive:
                after = {name: archive.read(name) for name in archive.namelist()}
            self.assertEqual(before, after)

    def test_failed_open_keeps_document_and_unsaved_state(self):
        self.window._add_box()
        document = self.window.document
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.sensorhead"
            path.write_bytes(b"not a zip")
            with patch.object(QMessageBox, "warning") as warning:
                self.window._run_task("Opening", lambda: load_head(path), self.window._replace_document)
                self._finish_worker()
                warning.assert_called_once()
        self.assertIs(self.window.document, document)
        self.assertTrue(self.window.dirty)
        self.assertTrue(self.window.isEnabled())

    def test_starter_is_removable_and_new_restores_it_without_a_tip(self):
        original = self.window.document
        mesh = original.parts[0].mesh.copy()
        self.assertEqual(len(original.parts), 1)
        self.assertEqual(original.parts[0].kind, "mesh")
        self.assertIsNone(original.tip)
        self.assertFalse(self.window.dimensions.isEnabled())
        self.assertTrue(self.window.part_pose.isEnabled())
        self.window.part_pose.position.fields[0].setValue(25)
        self.assertEqual(original.parts[0].pose.translation, (0.025, 0, 0))
        self.window._tip_at_origin()
        self.window._remove_part()
        self.assertEqual(self.window.document.parts, [])
        self.assertEqual(len(self.window.viewport.parts), 0)
        with patch.object(QMessageBox, "question", return_value=QMessageBox.Discard):
            self.window._new()
        restored = self.window.document
        self.assertIsNot(restored, original)
        self.assertEqual(len(restored.parts), 1)
        self.assertEqual(restored.parts[0].pose, Pose())
        np.testing.assert_array_equal(restored.parts[0].mesh, mesh)
        self.assertEqual(restored.placement, original.placement)
        self.assertIsNone(restored.tip)
        self.assertFalse(self.window.tip_pose.isEnabled())
        self.assertFalse(self.window.dirty)
        with patch.object(QMessageBox, "information") as information:
            self.window._export()
            information.assert_called_once()

    def _finish_worker(self):
        deadline = time.monotonic() + 10
        while self.window.worker is not None and time.monotonic() < deadline:
            self.application.processEvents()
            time.sleep(0.001)
        self.assertIsNone(self.window.worker, "File worker did not finish")


if __name__ == "__main__":
    unittest.main()
