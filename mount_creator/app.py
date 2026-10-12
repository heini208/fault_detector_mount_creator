"""A small Qt editor. UI values use millimetres and degrees; the model uses SI."""

import signal
import sys
import traceback
from functools import partial
from pathlib import Path

import numpy as np
from PyQt5.QtCore import QSignalBlocker, QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout,
    QGroupBox, QHBoxLayout, QInputDialog, QLabel, QLineEdit, QListWidget,
    QMainWindow, QMessageBox, QPushButton, QScrollArea, QTabWidget,
    QVBoxLayout, QWidget,
)

from .bundle import load_head, save_head
from .document import Part
from .geometry import (
    PLANE_AXIS_EPSILON, Pose, direction_pose, plane_from_triangle,
    point_at_surface_distance, read_stl, surface_axis_distance,
)
from .starter import new_starter_document
from .viewport import Viewport


class VectorFields(QWidget):
    """Three labelled numbers; setting a value never emits an edit."""

    changed = pyqtSignal()

    def __init__(self, labels=("X", "Y", "Z"), minimum=-2000, maximum=2000):
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.fields = []
        self._values = [0.0, 0.0, 0.0]
        for index, label in enumerate(labels):
            column = QVBoxLayout()
            column.addWidget(QLabel(label))
            field = QDoubleSpinBox()
            field.setRange(minimum, maximum)
            field.setDecimals(4)
            field.setSingleStep(1)
            field.setKeyboardTracking(False)
            field.valueChanged.connect(partial(self._edited, index))
            column.addWidget(field)
            layout.addLayout(column)
            self.fields.append(field)

    def values(self):
        return tuple(self._values)

    def _edited(self, index, value):
        self._values[index] = value
        self.changed.emit()

    def set_values(self, values):
        self._values = [float(value) for value in values]
        for field, value in zip(self.fields, values):
            with QSignalBlocker(field):
                field.setRange(min(field.minimum(), value), max(field.maximum(), value))
                field.setValue(float(value))


class PoseFields(QGroupBox):
    changed = pyqtSignal()

    def __init__(self, title):
        super().__init__(title)
        layout = QFormLayout(self)
        layout.setRowWrapPolicy(QFormLayout.WrapLongRows)
        self.position = VectorFields()
        self.rotation = VectorFields(("Roll", "Pitch", "Yaw"), -360, 360)
        layout.addRow("Position (mm)", self.position)
        layout.addRow("Rotation (°)", self.rotation)
        self.position.changed.connect(self.changed.emit)
        self.rotation.changed.connect(self.changed.emit)

    def pose(self):
        return Pose.from_rpy(np.array(self.position.values()) / 1000,
                             np.radians(self.rotation.values()))

    def set_pose(self, pose):
        self.position.set_values(np.array(pose.translation) * 1000)
        self.rotation.set_values(np.degrees(pose.rpy()))


class FileTask(QThread):
    """Keep bounded file/mesh work off the Qt thread."""

    succeeded = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, function, parent):
        super().__init__(parent)
        self.function = function

    def run(self):
        try:
            self.succeeded.emit(self.function())
        except (ValueError, OSError) as error:
            self.failed.emit(str(error))
        except Exception as error:
            # Unexpected failures retain their traceback for debugging.
            traceback.print_exc()
            self.failed.emit(str(error))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.document = new_starter_document()
        self.reference_part = None
        self.reference_point = None
        self.reference_normal = None
        self.reference_cell = -1
        self._axis_frame = "mount"
        self.dirty = False
        self.worker = None
        self.resize(1200, 800)
        self.setWindowTitle("Spot mount creator")

        toolbar = self.addToolBar("File")
        toolbar.setMovable(False)
        toolbar.addAction("New", self._new)
        toolbar.addAction("Open head…", self._open)
        toolbar.addAction("Export head…", self._export)
        toolbar.addSeparator()
        toolbar.addAction("Fit view", lambda: self.viewport.reset_view())

        controls = QWidget()
        panel = QVBoxLayout(controls)
        panel.addWidget(QLabel("Sensor mount"))
        self.name = QLineEdit(self.document.name)
        self.name.setPlaceholderText("Head name")
        self.name.textEdited.connect(self._rename)
        panel.addWidget(self.name)
        tabs = QTabWidget()
        self.tabs = tabs
        panel.addWidget(tabs)

        build = QWidget()
        build_layout = QVBoxLayout(build)
        instruction = QLabel("Move, extend or replace the starter mount.\n"
                             "Box dimensions are editable; STL meshes keep their shape.")
        instruction.setWordWrap(True)
        build_layout.addWidget(instruction)
        row = QHBoxLayout()
        for title, action in (("Add box", self._add_box), ("Import STL…", self._import_stl)):
            button = QPushButton(title)
            button.clicked.connect(action)
            row.addWidget(button)
        build_layout.addLayout(row)
        self.part_list = QListWidget()
        self.part_list.setMaximumHeight(130)
        self.part_list.currentRowChanged.connect(self._select_part)
        build_layout.addWidget(self.part_list)
        remove = QPushButton("Remove selected part")
        remove.clicked.connect(self._remove_part)
        build_layout.addWidget(remove)
        self.dimensions = VectorFields(minimum=0.001, maximum=2000)
        self.dimensions.changed.connect(self._resize_box)
        build_layout.addWidget(QLabel("Part size (mm; editable for boxes)"))
        build_layout.addWidget(self.dimensions)
        self.part_pose = PoseFields("Selected part · relative to mount")
        self.part_pose.changed.connect(self._move_part)
        build_layout.addWidget(self.part_pose)
        self.mount_pose = PoseFields("Whole mount · relative to hand")
        self.mount_pose.changed.connect(self._move_mount)
        build_layout.addWidget(self.mount_pose)
        build_layout.addStretch()
        tabs.addTab(build, "1 · Build mount")

        measurement = QGroupBox("Position tip from surfaces")
        measurement_layout = QVBoxLayout(measurement)
        help_text = QLabel("Click the initial tip position, then choose a reference face.\n"
                           "Edit its distance to the tip along X, Y or Z.\n"
                           "Repeat with another face to refine the placement.")
        help_text.setWordWrap(True)
        measurement_layout.addWidget(help_text)
        self.pick_button = QPushButton("1 · Pick sensor tip on mount")
        self.pick_button.setCheckable(True)
        self.pick_button.toggled.connect(self._picking_changed)
        measurement_layout.addWidget(self.pick_button)
        self.use_normal = QCheckBox("Use clicked tip face for sensing direction")
        self.use_normal.setChecked(True)
        measurement_layout.addWidget(self.use_normal)
        reference_row = QHBoxLayout()
        self.reference_button = QPushButton("2 · Pick reference surface")
        self.reference_button.setCheckable(True)
        self.reference_button.toggled.connect(
            lambda enabled: self._set_pick_mode("surface" if enabled else None))
        reference_row.addWidget(self.reference_button)
        clear = QPushButton("Clear reference")
        clear.clicked.connect(self._clear_measurement)
        reference_row.addWidget(clear)
        measurement_layout.addLayout(reference_row)
        self.reference_label = QLabel()
        self.reference_label.setWordWrap(True)
        measurement_layout.addWidget(self.reference_label)
        distance_label = QLabel("Signed distance from surface plane to tip (mm)")
        distance_label.setWordWrap(True)
        measurement_layout.addWidget(distance_label)
        distance_row = QHBoxLayout()
        self.measure_axis = QComboBox()
        self.measure_axis.addItems(("X (red)", "Y (green)", "Z (blue)"))
        self.measure_axis.currentIndexChanged.connect(self._show_measurement)
        distance_row.addWidget(self.measure_axis)
        self.measure_distance = QDoubleSpinBox()
        self.measure_distance.setRange(-2000, 2000)
        self.measure_distance.setDecimals(4)
        self.measure_distance.setSingleStep(1)
        self.measure_distance.setKeyboardTracking(False)
        self.measure_distance.setSuffix(" mm")
        self.measure_distance.valueChanged.connect(self._edit_surface_distance)
        distance_row.addWidget(self.measure_distance)
        measurement_layout.addLayout(distance_row)
        help_text = QLabel("Editing moves the tip immediately along only the chosen axis.\n"
                           "Positive follows the colored arrow; negative goes the other way.\n"
                           "Use flat faces for precise physical measurements.")
        help_text.setWordWrap(True)
        measurement_layout.addWidget(help_text)
        measure_page = QWidget()
        measure_layout = QVBoxLayout(measure_page)
        measure_layout.addWidget(measurement)
        direction_note = QLabel("The red arrow shows the sensing direction toward the inspected surface.")
        direction_note.setWordWrap(True)
        measure_layout.addWidget(direction_note)
        measure_layout.addStretch()
        tabs.addTab(measure_page, "2 · Sensor tip")

        self.show_lower_jaw = QCheckBox("Show lower-jaw extension")
        panel.addWidget(self.show_lower_jaw)
        self.show_holes = QCheckBox("Highlight candidate mounting holes")
        self.show_holes.setChecked(True)
        panel.addWidget(self.show_holes)
        note = QLabel("Orange rings mark unverified mounting-hole candidates.\n"
                      "The lower-jaw split is a visual approximation.\n"
                      "Starter alignment is model-derived; check mounting fit and tip.")
        note.setWordWrap(True)
        panel.addWidget(note)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(controls)
        scroll.setMinimumWidth(420)
        scroll.setMaximumWidth(500)
        self.viewport = Viewport()
        self.viewport.tip_picked.connect(self._tip_picked)
        self.viewport.surface_picked.connect(self._surface_picked)
        self.viewport.pick_failed.connect(self.statusBar().showMessage)
        self.show_lower_jaw.toggled.connect(self.viewport.set_lower_jaw_visible)
        self.show_holes.toggled.connect(self.viewport.set_holes_visible)
        central = QWidget()
        layout = QHBoxLayout(central)
        layout.addWidget(scroll)
        layout.addWidget(self.viewport, 1)
        self.setCentralWidget(central)
        tabs.currentChanged.connect(self._show_measurement)
        QApplication.instance().focusChanged.connect(self._focus_changed)
        self._refresh()
        self.viewport.reset_view()
        self.statusBar().showMessage("Orbit: left drag · Pan: middle drag · Zoom: wheel")

    def _selected_part(self):
        index = self.part_list.currentRow()
        return self.document.parts[index] if 0 <= index < len(self.document.parts) else None

    def _focus_changed(self, _previous, current):
        for control, frame in ((self.mount_pose, "hand"), (self.part_pose, "mount"),
                               (self.dimensions, "part")):
            if current is control or (current is not None and control.isAncestorOf(current)):
                self._axis_frame = frame
                self._show_measurement()
                return

    def _show_edit_axes(self):
        part = self._selected_part()
        if self._axis_frame == "hand" or part is None:
            frame, name = Pose(), "Hand"
        elif self._axis_frame == "part":
            frame, name = self.document.placement.compose(part.pose), "Part"
        else:
            frame, name = self.document.placement, "Mount"
        self.viewport.set_edit_frame(frame, name)

    def _refresh(self):
        index = self.part_list.currentRow()
        with QSignalBlocker(self.part_list):
            self.part_list.clear()
            self.part_list.addItems([part.name for part in self.document.parts])
            self.part_list.setCurrentRow(min(max(index, 0), len(self.document.parts) - 1))
        self.name.setText(self.document.name)
        self.mount_pose.set_pose(self.document.placement)
        self._select_part()

    def _select_part(self, index=None):
        self._axis_frame = "mount"
        part = self._selected_part()
        if self.reference_part is not None and not any(
                item is self.reference_part for item in self.document.parts):
            self._clear_measurement()
        self.part_pose.setEnabled(part is not None)
        self.dimensions.setEnabled(part is not None and part.kind == "box")
        if part is not None:
            self.part_pose.set_pose(part.pose)
            size = part.dimensions if part.kind == "box" else np.ptp(part.mesh.reshape(-1, 3), axis=0)
            self.dimensions.set_values(np.array(size) * 1000)
        self.viewport.set_document(self.document, self.part_list.currentRow())
        self._show_measurement()

    def _changed(self):
        self.dirty = True
        self.setWindowTitle("Spot mount creator · unsaved changes")
        self.viewport.set_document(self.document, self.part_list.currentRow())
        self._show_measurement()

    def _rename(self, name):
        self.document.name = name
        self._changed()

    def _add_part(self, part):
        try:
            self.document.add_part(part)
        except ValueError as error:
            QMessageBox.warning(self, "Cannot add part", str(error))
            return
        self._refresh()
        self.part_list.setCurrentRow(len(self.document.parts) - 1)
        self._changed()
        self.viewport.reset_view()

    def _add_box(self):
        self._add_part(Part(name=f"Box {len(self.document.parts) + 1}", dimensions=(0.04, 0.03, 0.02)))

    def _remove_part(self):
        if self._selected_part() is not None:
            del self.document.parts[self.part_list.currentRow()]
            self._refresh()
            self._changed()

    def _resize_box(self):
        part = self._selected_part()
        if part is not None and part.kind == "box":
            self._axis_frame = "part"
            self._clear_measurement()
            part.dimensions = tuple(np.array(self.dimensions.values()) / 1000)
            self._changed()

    def _move_part(self):
        part = self._selected_part()
        if part is not None:
            self._axis_frame = "mount"
            self._clear_measurement()
            part.pose = self.part_pose.pose()
            self._changed()

    def _move_mount(self):
        self._axis_frame = "hand"
        self.document.placement = self.mount_pose.pose()
        self._changed()

    def _picking_changed(self, enabled):
        self._set_pick_mode("tip" if enabled else None)

    def _set_pick_mode(self, mode):
        for button, name in ((self.pick_button, "tip"), (self.reference_button, "surface")):
            with QSignalBlocker(button):
                button.setChecked(mode == name)
        self.viewport.set_pick_mode(mode)
        messages = {
            "tip": "Click a mount surface to place the sensor tip.",
            "surface": "Click a reference face on any mount part.",
        }
        self.statusBar().showMessage(messages.get(mode, ""))

    def _clear_measurement(self):
        self.reference_part = None
        self.reference_point = None
        self.reference_normal = None
        self.reference_cell = -1
        self._set_pick_mode(None)
        self._show_measurement()

    def _surface_picked(self, part_index, cell_id, world_triangle):
        if self.document.tip is None:
            return
        part = self.document.parts[part_index]
        frame = self.document.placement.compose(part.pose)
        try:
            point, normal = plane_from_triangle(frame.inverse().apply(world_triangle))
        except ValueError as error:
            self.statusBar().showMessage(str(error))
            return
        self.reference_part = part
        self.reference_point = point
        self.reference_normal = normal
        self.reference_cell = cell_id
        with QSignalBlocker(self.measure_axis):
            self.measure_axis.setCurrentIndex(int(np.argmax(np.abs(normal))))
        self._set_pick_mode(None)
        self._show_measurement()

    def _show_measurement(self):
        self._show_edit_axes()
        has_tip = self.document.tip is not None
        self.reference_button.setEnabled(has_tip and bool(self.document.parts))
        enabled = has_tip and self.reference_part is not None
        self.measure_axis.setEnabled(enabled)
        self.measure_distance.setEnabled(enabled)
        if not enabled:
            self.reference_label.setText("Pick a reference surface." if has_tip else "Pick the sensor tip first.")
            with QSignalBlocker(self.measure_distance):
                self.measure_distance.setValue(0)
            self.viewport.set_surface_measurement()
            return
        for axis, component in enumerate(self.reference_normal):
            self.measure_axis.model().item(axis).setEnabled(bool(abs(component) > PLANE_AXIS_EPSILON))
        axis = self.measure_axis.currentIndex()
        tip = self.reference_part.pose.inverse().apply(self.document.tip.translation)
        try:
            distance = surface_axis_distance(self.reference_point, self.reference_normal, tip, axis)
        except ValueError as error:
            self.measure_distance.setEnabled(False)
            self.reference_label.setText(str(error))
            self.viewport.set_surface_measurement()
            return
        self.reference_label.setText(f"Reference: {self.reference_part.name}\n"
                                    "X/Y/Z follow this part's axes. Yellow marks the surface plane.")
        millimetres = distance * 1000
        with QSignalBlocker(self.measure_distance):
            self.measure_distance.setRange(min(-2000, millimetres), max(2000, millimetres))
            self.measure_distance.setValue(millimetres)
        part_index = next(index for index, part in enumerate(self.document.parts)
                          if part is self.reference_part)
        frame = self.document.placement.compose(self.reference_part.pose)
        if self.tabs.currentIndex() == 1:
            self.viewport.set_surface_measurement(part_index, self.reference_cell, frame,
                                                  self.reference_point, self.reference_normal, tip, axis)
        else:
            self.viewport.set_surface_measurement()

    def _edit_surface_distance(self, millimetres):
        if self.reference_part is None or self.document.tip is None:
            return
        tip = self.reference_part.pose.inverse().apply(self.document.tip.translation)
        try:
            point = point_at_surface_distance(self.reference_point, self.reference_normal,
                                              tip, self.measure_axis.currentIndex(), millimetres / 1000)
        except ValueError as error:
            self.statusBar().showMessage(str(error))
            return
        position = self.reference_part.pose.apply(point)
        self._set_tip(Pose(tuple(position), self.document.tip.quaternion))

    def _set_tip(self, pose):
        self.document.tip = pose
        self._set_pick_mode(None)
        self._changed()

    def _tip_picked(self, point, normal):
        inverse = self.document.placement.inverse()
        position = inverse.apply(point)
        if self.use_normal.isChecked():
            direction = inverse.matrix()[:3, :3] @ normal
            up = inverse.matrix()[:3, :3] @ np.array([0, 0, 1])
            if abs(np.dot(direction, up)) > 0.999:
                up = inverse.matrix()[:3, :3] @ np.array([0, 1, 0])
            self._set_tip(direction_pose(position, direction, up))
        else:
            previous = self.document.tip or Pose()
            self._set_tip(Pose(tuple(position), previous.quaternion))

    def _discard_changes(self):
        if not self.dirty:
            return True
        answer = QMessageBox.question(self, "Unsaved changes", "Discard unexported changes?",
                                      QMessageBox.Discard | QMessageBox.Cancel, QMessageBox.Cancel)
        return answer == QMessageBox.Discard

    def _new(self):
        if self._discard_changes():
            self._replace_document(new_starter_document())

    def _replace_document(self, document):
        self._clear_measurement()
        self.document = document
        self.dirty = False
        self.pick_button.setChecked(False)
        self._refresh()
        self.viewport.reset_view()
        self.setWindowTitle("Spot mount creator")

    def _run_task(self, description, function, on_success):
        self.setEnabled(False)
        self.statusBar().showMessage(description)
        self.worker = FileTask(function, self)
        self.worker.succeeded.connect(on_success)
        self.worker.failed.connect(lambda message: QMessageBox.warning(self, "File operation failed", message))
        self.worker.finished.connect(self._task_finished)
        self.worker.start()

    def _task_finished(self):
        self.setEnabled(True)
        self.worker.deleteLater()
        self.worker = None
        self.statusBar().showMessage("Ready")

    def _import_stl(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Import mount geometry", "", "STL meshes (*.stl)")
        if not filename:
            return
        unit, accepted = QInputDialog.getItem(self, "STL units", "Units used by this file:",
                                             ["Millimetres", "Metres", "Inches"], 0, False)
        if accepted:
            scale = {"Millimetres": 0.001, "Metres": 1.0, "Inches": 0.0254}[unit]
            self._run_task("Reading and checking mesh…", lambda: read_stl(Path(filename), scale),
                           lambda mesh: self._add_part(Part(name=Path(filename).stem, kind="mesh", mesh=mesh)))

    def _open(self):
        if not self._discard_changes():
            return
        filename, _ = QFileDialog.getOpenFileName(self, "Open sensor head", "", "Sensor heads (*.sensorhead)")
        if filename:
            self._run_task("Opening head…", lambda: load_head(Path(filename)), self._replace_document)

    def _export(self):
        if not self.document.parts or self.document.tip is None:
            QMessageBox.information(self, "Complete the head", "Add a mount part and set its sensor tip before exporting.")
            return
        filename, _ = QFileDialog.getSaveFileName(self, "Export sensor head", "head.sensorhead",
                                                 "Sensor heads (*.sensorhead)")
        if filename:
            path = Path(filename)
            if path.suffix != ".sensorhead":
                path = Path(str(path) + ".sensorhead")
                if path.exists():
                    answer = QMessageBox.question(self, "Replace file", f"Replace {path.name}?",
                                                  QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
                    if answer != QMessageBox.Yes:
                        return
            self._run_task("Validating and exporting head…", lambda: save_head(path, self.document),
                           self._exported)

    def _exported(self, result):
        self.dirty = False
        self.setWindowTitle("Spot mount creator")

    def closeEvent(self, event):
        if self.worker is not None or not self._discard_changes():
            event.ignore()
        else:
            self.viewport.Finalize()
            event.accept()


def main():
    # Let the OS stop the process even while Qt is waiting in its event loop.
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    application = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return application.exec_()
