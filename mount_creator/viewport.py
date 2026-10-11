"""VTK display and picking; the document owns all editable geometry."""

import json
from pathlib import Path

import numpy as np
from PyQt5.QtCore import pyqtSignal
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy
from vtkmodules.vtkCommonCore import vtkPoints
from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkPolyData
from vtkmodules.vtkCommonMath import vtkMatrix4x4
from vtkmodules.vtkFiltersSources import (
    vtkArrowSource, vtkLineSource, vtkRegularPolygonSource, vtkSphereSource,
)
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
from vtkmodules.vtkIOGeometry import vtkOBJReader
from vtkmodules.vtkRenderingAnnotation import vtkAxesActor
from vtkmodules.vtkRenderingCore import (
    vtkActor, vtkBillboardTextActor3D, vtkCellPicker, vtkPolyDataMapper, vtkRenderer,
)

# Importing these modules registers VTK's rendering backends.
import vtkmodules.vtkRenderingOpenGL2  # noqa: F401

from .geometry import Pose, surface_axis_distance


ASSETS = Path(__file__).parent / "assets"


def vtk_matrix(matrix):
    result = vtkMatrix4x4()
    result.DeepCopy(np.asarray(matrix, dtype=float).ravel())
    return result


def mesh_data(triangles):
    """Build a VTK mesh without retaining pointers to temporary NumPy arrays."""
    points = vtkPoints()
    points.SetData(numpy_to_vtk(triangles.reshape(-1, 3), deep=True))
    faces = np.column_stack((np.full(len(triangles), 3),
                             np.arange(len(triangles) * 3).reshape(-1, 3)))
    cells = vtkCellArray()
    cells.SetCells(len(triangles), numpy_to_vtkIdTypeArray(faces.astype(np.int64).ravel(), deep=True))
    result = vtkPolyData()
    result.SetPoints(points)
    result.SetPolys(cells)
    return result


def mesh_actor(data, color, opacity=1.0):
    mapper = vtkPolyDataMapper()
    mapper.SetInputData(data)
    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(*color)
    actor.GetProperty().SetOpacity(opacity)
    return actor


def axes_actor(length):
    actor = vtkAxesActor()
    actor.SetTotalLength(length, length, length)
    for caption in (actor.GetXAxisCaptionActor2D(), actor.GetYAxisCaptionActor2D(),
                    actor.GetZAxisCaptionActor2D()):
        # Keep labels small instead of scaling them with the viewport.
        caption.GetTextActor().SetTextScaleModeToNone()
        caption.GetCaptionTextProperty().SetFontSize(12)
    actor.PickableOff()
    return actor


class Viewport(QVTKRenderWindowInteractor):
    tip_picked = pyqtSignal(object, object)
    surface_picked = pyqtSignal(int, int, object)
    pick_failed = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.renderer = vtkRenderer()
        self.renderer.SetBackground(0.12, 0.15, 0.19)
        self.GetRenderWindow().AddRenderer(self.renderer)
        self.SetInteractorStyle(vtkInteractorStyleTrackballCamera())
        self.parts = []
        self.markers = []
        self.measurement_actors = []
        self.pick_mode = None
        self.AddObserver("LeftButtonPressEvent", self._pick)

        # Visual detail is independent of the export's mounting-template identity.
        preview = json.loads((ASSETS / "preview.json").read_text())
        self.reference_actors = {"fixed": [], "lower_jaw": [], "finger": []}
        for item in preview["meshes"]:
            reader = vtkOBJReader()
            reader.SetFileName(str(ASSETS / item["file"]))
            reader.Update()
            color = (0.45, 0.52, 0.58) if item["role"] == "finger" else (0.70, 0.72, 0.75)
            actor = mesh_actor(reader.GetOutput(), color)
            actor.SetUserMatrix(vtk_matrix(Pose(tuple(item["translation"]),
                                               tuple(item["quaternion"])).matrix()))
            actor.PickableOff()
            actor.SetVisibility(item["role"] != "lower_jaw")
            self.reference_actors[item["role"]].append(actor)
            self.renderer.AddActor(actor)

        candidates = json.loads((ASSETS / "mount_hole_candidates.json").read_text())
        self.hole_markers = []
        for hole in candidates["holes"]:
            ring = vtkRegularPolygonSource()
            ring.SetNumberOfSides(48)
            ring.SetRadius(hole["radius_m"])
            ring.SetNormal(*hole["axis_hand"])
            # Lift the highlight, not the stored datum, slightly off the surface.
            center = (np.array(hole["center_hand_m"])
                      + preview["hole_display_offset_m"] * np.array(hole["axis_hand"]))
            ring.SetCenter(*center)
            ring.GeneratePolygonOff()
            ring.Update()
            actor = mesh_actor(ring.GetOutput(), (1.0, 0.5, 0.05))
            actor.GetProperty().SetLineWidth(4)
            actor.GetProperty().LightingOff()
            actor.PickableOff()
            self.hole_markers.append(actor)
            self.renderer.AddActor(actor)

        axes = axes_actor(0.04)
        self.renderer.AddActor(axes)
        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(0.35, -0.45, 0.18)
        camera.SetFocalPoint(-0.08, 0, -0.01)
        camera.SetViewUp(0, 0, 1)
        self.renderer.ResetCamera()

    def showEvent(self, event):
        super().showEvent(event)
        self.Initialize()
        self.render()

    def render(self):
        if self.isVisible():
            self.GetRenderWindow().Render()

    def set_document(self, document, selected=-1):
        for actor in self.parts + self.markers:
            self.renderer.RemoveActor(actor)
        self.parts = []
        self.markers = []
        for index, part in enumerate(document.parts):
            color = (0.98, 0.68, 0.22) if index == selected else (0.25, 0.72, 0.86)
            actor = mesh_actor(mesh_data(part.triangles()), color)
            actor.SetUserMatrix(vtk_matrix(document.placement.matrix()))
            self.renderer.AddActor(actor)
            self.parts.append(actor)
        if document.tip is not None:
            tip = document.hand_to_probe()
            sphere = vtkSphereSource()
            sphere.SetRadius(0.002)
            sphere.SetCenter(*tip.translation)
            sphere.Update()
            point = mesh_actor(sphere.GetOutput(), (1.0, 0.3, 0.3))
            arrow = vtkArrowSource()  # The source points along local +X.
            arrow.Update()
            direction = mesh_actor(arrow.GetOutput(), (1.0, 0.3, 0.3))
            direction.SetUserMatrix(vtk_matrix(tip.matrix() @ np.diag([0.04, 0.04, 0.04, 1])))
            self.markers = [point, direction]
            for actor in self.markers:
                actor.PickableOff()
                self.renderer.AddActor(actor)
        self.renderer.ResetCameraClippingRange()
        self.render()

    def reset_view(self):
        self.renderer.ResetCamera()
        self.render()

    def set_lower_jaw_visible(self, visible):
        for actor in self.reference_actors["lower_jaw"]:
            actor.SetVisibility(visible)
        self.renderer.ResetCameraClippingRange()
        self.render()

    def set_holes_visible(self, visible):
        for actor in self.hole_markers:
            actor.SetVisibility(visible)
        self.render()

    def set_pick_mode(self, mode):
        if mode not in (None, "tip", "surface"):
            raise ValueError("Unknown picking mode.")
        self.pick_mode = mode

    def set_surface_measurement(self, part_index=-1, cell_id=-1, frame=None,
                                point=None, normal=None, tip=None, axis=0):
        """Highlight a face and measure along its part's local axes."""
        for actor in self.measurement_actors:
            self.renderer.RemoveActor(actor)
        self.measurement_actors = []
        if 0 <= part_index < len(self.parts):
            self._highlight_surface(self.parts[part_index], cell_id)
        if frame is not None and point is not None and normal is not None and tip is not None:
            tip = np.asarray(tip, dtype=float)
            distance = surface_axis_distance(point, normal, tip, axis)
            base = tip.copy()
            base[axis] -= distance
            self._measurement_point(frame.apply(base), (0.95, 0.95, 0.35))
            axes = axes_actor(0.015)
            axes.SetUserMatrix(vtk_matrix(frame.compose(Pose(base)).matrix()))
            self._add_measurement_actor(axes)
            colors = ((1.0, 0.35, 0.35), (0.4, 0.9, 0.4), (0.35, 0.65, 1.0))
            self._measurement_segment(frame.apply(base), frame.apply(tip),
                                      f"{'XYZ'[axis]} {distance * 1000:+.3f} mm", colors[axis])
        self.renderer.ResetCameraClippingRange()
        self.render()

    def _highlight_surface(self, source, cell_id):
        data = source.GetMapper().GetInput()
        if not 0 <= cell_id < data.GetNumberOfCells():
            return
        # mesh_data stores three independent points for every authored triangle.
        triangles = vtk_to_numpy(data.GetPoints().GetData()).reshape(-1, 3, 3)
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        lengths = np.linalg.norm(normals, axis=1)
        normals = np.divide(normals, lengths[:, None], out=np.zeros_like(normals),
                            where=lengths[:, None] > 0)
        normal = normals[cell_id]
        distances = (triangles - triangles[cell_id, 0]) @ normal
        coplanar = ((np.abs(normals @ normal) >= 1 - 1e-6)
                    & (np.max(np.abs(distances), axis=1) <= 1e-6))
        if not np.any(coplanar):
            return
        actor = mesh_actor(mesh_data(triangles[coplanar]), (1.0, 0.92, 0.2), 0.5)
        actor.SetUserMatrix(source.GetMatrix())
        actor.GetMapper().SetResolveCoincidentTopologyToPolygonOffset()
        actor.GetMapper().SetRelativeCoincidentTopologyPolygonOffsetParameters(-1, -1)
        actor.GetProperty().LightingOff()
        self._add_measurement_actor(actor)

    def _add_measurement_actor(self, actor):
        actor.PickableOff()
        self.measurement_actors.append(actor)
        self.renderer.AddActor(actor)

    def _measurement_point(self, point, color):
        sphere = vtkSphereSource()
        sphere.SetRadius(0.0013)
        sphere.SetCenter(*point)
        sphere.Update()
        self._add_measurement_actor(mesh_actor(sphere.GetOutput(), color))

    def _measurement_segment(self, start, end, text, color):
        if np.linalg.norm(end - start) > 1e-9:
            line = vtkLineSource()
            line.SetPoint1(*start)
            line.SetPoint2(*end)
            line.Update()
            actor = mesh_actor(line.GetOutput(), color)
            actor.GetProperty().SetLineWidth(3)
            actor.GetProperty().LightingOff()
            self._add_measurement_actor(actor)
        label = vtkBillboardTextActor3D()
        label.SetInput(text)
        label.SetPosition(*((start + end) / 2))
        label.SetDisplayOffset(5, 5)
        label.GetTextProperty().SetFontSize(15)
        label.GetTextProperty().SetColor(*color)
        self._add_measurement_actor(label)

    def _pick(self, caller, event):
        if self.pick_mode is None:
            return
        picker = vtkCellPicker()
        picker.SetTolerance(0.0005)
        picker.PickFromListOn()
        for actor in self.parts:
            picker.AddPickList(actor)
        x, y = self.GetEventPosition()
        if not picker.Pick(x, y, 0, self.renderer):
            self.pick_failed.emit("Click a visible surface of a mount part.")
            return
        if self.pick_mode == "tip":
            self.tip_picked.emit(np.array(picker.GetPickPosition()),
                                 np.array(picker.GetPickNormal()))
            return
        actor, cell_id = picker.GetActor(), picker.GetCellId()
        if actor not in self.parts or not 0 <= cell_id < actor.GetMapper().GetInput().GetNumberOfCells():
            self.pick_failed.emit("Click a visible surface of a mount part.")
            return
        points = actor.GetMapper().GetInput().GetCell(cell_id).GetPoints()
        triangle = np.array([actor.GetMatrix().MultiplyPoint((*points.GetPoint(index), 1))[:3]
                             for index in range(3)])
        self.surface_picked.emit(self.parts.index(actor), cell_id, triangle)
