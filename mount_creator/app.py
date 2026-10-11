"""A small Qt editor. UI values use millimetres and degrees; the model uses SI."""

import signal
import sys
import traceback
from functools import partial
from pathlib import Path

import numpy as np
from PyQt5.QtCore import QSignalBlocker, QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QDoubleSpinBox, QFileDialog, QFormLayout,
    QGroupBox, QHBoxLayout, QInputDialog, QLabel, QLineEdit, QListWidget,
    QMainWindow, QMessageBox, QPushButton, QScrollArea, QTabWidget,
    QVBoxLayout, QWidget,
)

from .bundle import load_head, save_head
from .document import Part
from .geometry import Pose, direction_pose, read_stl
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

        sensor = QWidget()
        sensor_layout = QVBoxLayout(sensor)
        instruction = QLabel("Pick the sensing point on your mount, or enter a virtual point.\n"
                             "The red arrow must point toward the surface being inspected.")
        instruction.setWordWrap(True)
        sensor_layout.addWidget(instruction)
        self.pick_button = QPushButton("Pick tip on mount")
        self.pick_button.setCheckable(True)
        self.pick_button.toggled.connect(self._picking_changed)
        sensor_layout.addWidget(self.pick_button)
        self.use_normal = QCheckBox("Use picked face for sensing direction")
        self.use_normal.setChecked(True)
        sensor_layout.addWidget(self.use_normal)
        origin = QPushButton("Set tip at mount origin")
        origin.clicked.connect(self._tip_at_origin)
        sensor_layout.addWidget(origin)
        self.tip_pose = PoseFields("Sensor tip · relative to mount")
        self.tip_pose.changed.connect(self._move_tip)
        sensor_layout.addWidget(self.tip_pose)
        flip = QPushButton("Flip sensing direction")
        flip.clicked.connect(self._flip_tip)
        sensor_layout.addWidget(flip)
        sensor_layout.addStretch()
        tabs.addTab(sensor, "2 · Sensor tip")

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
        scroll.setMaximumWidth(480)
        self.viewport = Viewport()
        self.viewport.tip_picked.connect(self._tip_picked)
        self.show_lower_jaw.toggled.connect(self.viewport.set_lower_jaw_visible)
        self.show_holes.toggled.connect(self.viewport.set_holes_visible)
        central = QWidget()
        layout = QHBoxLayout(central)
        layout.addWidget(scroll)
        layout.addWidget(self.viewport, 1)
        self.setCentralWidget(central)
        self._refresh()
        self.viewport.reset_view()
        self.statusBar().showMessage("Orbit: left drag · Pan: middle drag · Zoom: wheel")

    def _selected_part(self):
        index = self.part_list.currentRow()
        return self.document.parts[index] if 0 <= index < len(self.document.parts) else None

    def _refresh(self):
        index = self.part_list.currentRow()
        with QSignalBlocker(self.part_list):
            self.part_list.clear()
            self.part_list.addItems([part.name for part in self.document.parts])
            self.part_list.setCurrentRow(min(max(index, 0), len(self.document.parts) - 1))
        self.name.setText(self.document.name)
        self.mount_pose.set_pose(self.document.placement)
        self.tip_pose.setEnabled(self.document.tip is not None)
        self.tip_pose.set_pose(self.document.tip or Pose())
        self._select_part()

    def _select_part(self, index=None):
        part = self._selected_part()
        self.part_pose.setEnabled(part is not None)
        self.dimensions.setEnabled(part is not None and part.kind == "box")
        if part is not None:
            self.part_pose.set_pose(part.pose)
            size = part.dimensions if part.kind == "box" else np.ptp(part.mesh.reshape(-1, 3), axis=0)
            self.dimensions.set_values(np.array(size) * 1000)
        self.viewport.set_document(self.document, self.part_list.currentRow())

    def _changed(self):
        self.dirty = True
        self.setWindowTitle("Spot mount creator · unsaved changes")
        self.viewport.set_document(self.document, self.part_list.currentRow())

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
            part.dimensions = tuple(np.array(self.dimensions.values()) / 1000)
            self._changed()

    def _move_part(self):
        part = self._selected_part()
        if part is not None:
            part.pose = self.part_pose.pose()
            self._changed()

    def _move_mount(self):
        self.document.placement = self.mount_pose.pose()
        self._changed()

    def _picking_changed(self, enabled):
        self.viewport.picking = enabled
        self.statusBar().showMessage("Click a mount surface to set the tip." if enabled else "")

    def _set_tip(self, pose):
        self.document.tip = pose
        self.tip_pose.setEnabled(True)
        self.tip_pose.set_pose(pose)
        self.pick_button.setChecked(False)
        self._changed()

    def _tip_at_origin(self):
        self._set_tip(Pose())

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

    def _move_tip(self):
        if self.document.tip is not None:
            self.document.tip = self.tip_pose.pose()
            self._changed()

    def _flip_tip(self):
        if self.document.tip is not None:
            self._set_tip(self.document.tip.compose(Pose.from_rpy((0, 0, 0), (0, 0, np.pi))))

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
