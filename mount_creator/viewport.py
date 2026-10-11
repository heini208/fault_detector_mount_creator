"""VTK display and picking; the document owns all editable geometry."""

import json
from pathlib import Path

import numpy as np
from PyQt5.QtCore import pyqtSignal
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray
from vtkmodules.vtkCommonCore import vtkPoints
from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkPolyData
from vtkmodules.vtkCommonMath import vtkMatrix4x4
from vtkmodules.vtkFiltersSources import vtkArrowSource, vtkRegularPolygonSource, vtkSphereSource
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
from vtkmodules.vtkIOGeometry import vtkOBJReader
from vtkmodules.vtkRenderingAnnotation import vtkAxesActor
from vtkmodules.vtkRenderingCore import vtkActor, vtkCellPicker, vtkPolyDataMapper, vtkRenderer

# Importing these modules registers VTK's rendering backends.
import vtkmodules.vtkRenderingOpenGL2  # noqa: F401

from .geometry import Pose


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


class Viewport(QVTKRenderWindowInteractor):
    tip_picked = pyqtSignal(object, object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.renderer = vtkRenderer()
        self.renderer.SetBackground(0.12, 0.15, 0.19)
        self.GetRenderWindow().AddRenderer(self.renderer)
        self.SetInteractorStyle(vtkInteractorStyleTrackballCamera())
        self.parts = []
        self.markers = []
        self.picking = False
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

        axes = vtkAxesActor()
        axes.SetTotalLength(0.04, 0.04, 0.04)
        axes.PickableOff()
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

    def _pick(self, caller, event):
        if not self.picking:
            return
        picker = vtkCellPicker()
        picker.SetTolerance(0.0005)
        picker.PickFromListOn()
        for actor in self.parts:
            picker.AddPickList(actor)
        x, y = self.GetEventPosition()
        if picker.Pick(x, y, 0, self.renderer):
            self.tip_picked.emit(np.array(picker.GetPickPosition()),
                                 np.array(picker.GetPickNormal()))
