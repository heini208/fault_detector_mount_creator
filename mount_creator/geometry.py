"""Rigid transforms and triangle meshes, with all distances in metres."""

from dataclasses import dataclass
from pathlib import Path
import re
import struct

import numpy as np


MAX_TRIANGLES = 200_000
MAX_STL_BYTES = 32 * 1024 * 1024
STL_TRIANGLE_DTYPE = np.dtype([
    ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2"),
])


@dataclass(frozen=True)
class Pose:
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self):
        translation = np.asarray(self.translation, dtype=float)
        quaternion = np.asarray(self.quaternion, dtype=float)
        if translation.shape != (3,) or not np.isfinite(translation).all():
            raise ValueError("Translation must contain three finite coordinates.")
        if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
            raise ValueError("Quaternion must contain four finite xyzw values.")
        length = np.linalg.norm(quaternion)
        if not np.isclose(length, 1.0, rtol=0.0, atol=1e-6):
            raise ValueError("Quaternion must have unit length.")
        object.__setattr__(self, "translation", tuple(translation))
        object.__setattr__(self, "quaternion", tuple(quaternion / length))

    def matrix(self) -> np.ndarray:
        x, y, z, w = self.quaternion
        result = np.eye(4)
        result[:3, :3] = (
            (1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)),
            (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)),
            (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)),
        )
        result[:3, 3] = self.translation
        return result

    def inverse(self) -> "Pose":
        matrix = self.matrix()
        rotation = matrix[:3, :3].T
        result = np.eye(4)
        result[:3, :3] = rotation
        result[:3, 3] = -rotation @ matrix[:3, 3]
        return Pose.from_matrix(result)

    def compose(self, other: "Pose") -> "Pose":
        """Return self followed by the local transform other."""
        return Pose.from_matrix(self.matrix() @ other.matrix())

    def apply(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=float)
        if points.ndim == 0 or points.shape[-1] != 3:
            raise ValueError("Points must have three coordinates.")
        if not np.isfinite(points).all():
            raise ValueError("Points must be finite.")
        matrix = self.matrix()
        return points @ matrix[:3, :3].T + matrix[:3, 3]

    def rpy(self) -> tuple[float, float, float]:
        """Return roll, pitch and yaw in radians (Rz(yaw) Ry(pitch) Rx(roll))."""
        rotation = self.matrix()[:3, :3]
        pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
        if abs(np.cos(pitch)) > 1e-8:
            roll = np.arctan2(rotation[2, 1], rotation[2, 2])
            yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
        else:
            roll = 0.0
            yaw = np.arctan2(-rotation[0, 1], rotation[1, 1])
        return (float(roll), float(pitch), float(yaw))

    def to_dict(self) -> dict:
        return {"translation": list(self.translation), "quaternion": list(self.quaternion)}

    @classmethod
    def from_dict(cls, values: dict) -> "Pose":
        return cls(values["translation"], values["quaternion"])

    @classmethod
    def from_rpy(cls, translation, rpy):
        angles = np.asarray(rpy, dtype=float)
        if angles.shape != (3,) or not np.isfinite(angles).all():
            raise ValueError("Roll, pitch and yaw must be finite radians.")
        sr, sp, sy = np.sin(angles / 2)
        cr, cp, cy = np.cos(angles / 2)
        return cls(translation, (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ))

    @classmethod
    def from_matrix(cls, matrix):
        matrix = np.asarray(matrix, dtype=float)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError("Pose matrix must be a finite 4 by 4 array.")
        rotation = matrix[:3, :3]
        if (not np.allclose(matrix[3], (0, 0, 0, 1), rtol=0, atol=1e-9)
                or not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=1e-8)
                or not np.isclose(np.linalg.det(rotation), 1.0, rtol=0, atol=1e-8)):
            raise ValueError("Pose matrix must describe a rigid rotation and translation.")
        trace = np.trace(rotation)
        if trace > 0:
            scale = 2 * np.sqrt(1 + trace)
            quaternion = (
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                scale / 4,
            )
        else:
            # Choose the largest diagonal entry to avoid division near zero.
            index = int(np.argmax(np.diag(rotation)))
            next_index, last_index = (index + 1) % 3, (index + 2) % 3
            scale = 2 * np.sqrt(1 + rotation[index, index]
                                - rotation[next_index, next_index]
                                - rotation[last_index, last_index])
            quaternion = np.zeros(4)
            quaternion[index] = scale / 4
            quaternion[next_index] = (rotation[index, next_index] + rotation[next_index, index]) / scale
            quaternion[last_index] = (rotation[index, last_index] + rotation[last_index, index]) / scale
            quaternion[3] = (rotation[last_index, next_index] - rotation[next_index, last_index]) / scale
        return cls(matrix[:3, 3], quaternion)


def direction_pose(position, direction, up=(0, 0, 1)) -> Pose:
    """Orient local +X toward the surface and local +Z toward the chosen up."""
    direction = np.asarray(direction, dtype=float)
    up = np.asarray(up, dtype=float)
    if (direction.shape != (3,) or up.shape != (3,)
            or not np.isfinite(direction).all() or not np.isfinite(up).all()
            or np.linalg.norm(direction) == 0 or np.linalg.norm(up) == 0):
        raise ValueError("Direction and up must be nonzero finite vectors.")
    x_axis = direction / np.linalg.norm(direction)
    y_axis = np.cross(up / np.linalg.norm(up), x_axis)
    if np.linalg.norm(y_axis) < 1e-8:
        raise ValueError("Direction and up are parallel; choose another up axis.")
    y_axis /= np.linalg.norm(y_axis)
    matrix = np.eye(4)
    matrix[:3, :3] = np.column_stack((x_axis, y_axis, np.cross(x_axis, y_axis)))
    matrix[:3, 3] = position
    return Pose.from_matrix(matrix)


def box_mesh(dimensions: tuple[float, float, float]) -> np.ndarray:
    dimensions = np.asarray(dimensions, dtype=float)
    if (dimensions.shape != (3,) or not np.isfinite(dimensions).all()
            or (dimensions <= 0).any()):
        raise ValueError("Box dimensions must be three positive finite lengths.")
    vertices = np.array([
        (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
        (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
    ]) * dimensions / 2
    faces = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
    triangles = [(a, b, c) for a, b, c, d in faces]
    triangles += [(a, c, d) for a, b, c, d in faces]
    return vertices[np.array(triangles)]


def validate_mesh(triangles: np.ndarray) -> None:
    """Check closed, consistently wound, nonzero-volume components, not intersections."""
    triangles = np.asarray(triangles, dtype=float)
    if (triangles.ndim != 3 or triangles.shape[1:] != (3, 3)
            or not 0 < len(triangles) <= MAX_TRIANGLES):
        raise ValueError(f"Mesh must contain 1 to {MAX_TRIANGLES} triangles.")
    if not np.isfinite(triangles).all():
        raise ValueError("Mesh coordinates must be finite.")
    with np.errstate(over="ignore", invalid="ignore"):
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        areas = np.linalg.norm(normals, axis=1)
    if not np.isfinite(areas).all() or (areas == 0).any():
        raise ValueError("Mesh has degenerate triangles or coordinates too large.")
    vertices, vertex_ids = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = vertex_ids.reshape(-1, 3)
    edges = np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]))
    _, edge_ids = np.unique(np.sort(edges, axis=1), axis=0, return_inverse=True)
    directions = np.where(edges[:, 0] < edges[:, 1], 1, -1)
    # Closed parts may touch: allow multiple pairs of opposite directed edges.
    if (np.bincount(edge_ids, weights=directions) != 0).any():
        raise ValueError("Mesh must be closed with consistent triangle winding.")
    _check_component_volumes(triangles, vertices, faces)


def _check_component_volumes(triangles, vertices, faces):
    # Union-find groups triangles that share vertices without a graph dependency.
    parents = np.arange(len(vertices))

    def root(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    for a, b, c in faces:
        parent = root(a)
        parents[root(b)] = parent
        parents[root(c)] = parent
    roots = np.array([root(face[0]) for face in faces])
    _, component_ids = np.unique(roots, return_inverse=True)
    # A nearby origin reduces cancellation for shapes far from the document origin.
    local = triangles - vertices[roots, None, :]
    volumes = np.einsum("ij,ij->i", local[:, 0], np.cross(local[:, 1], local[:, 2])) / 6
    signed = np.bincount(component_ids, weights=volumes)
    magnitude = np.bincount(component_ids, weights=np.abs(volumes))
    if (np.abs(signed) <= magnitude * 1e-12).any():
        raise ValueError("Each mesh component must enclose nonzero volume.")


def mesh_to_stl(triangles: np.ndarray) -> bytes:
    triangles = np.asarray(triangles, dtype=float)
    if (np.abs(triangles) > np.finfo(np.float32).max).any():
        raise ValueError("Mesh coordinates exceed the float32 range supported by STL.")
    validate_mesh(triangles)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    records = np.zeros(len(triangles), dtype=STL_TRIANGLE_DTYPE)
    records["normal"] = normals
    records["vertices"] = triangles
    # STL stores float32 coordinates; verify rounding has not damaged the mesh.
    validate_mesh(records["vertices"])
    header = b"Sensor head collision mesh; coordinates in metres".ljust(80, b"\0")
    return header + struct.pack("<I", len(triangles)) + records.tobytes()


def mesh_from_stl(data: bytes) -> np.ndarray:
    """Read binary STL coordinates as metres; STL itself carries no unit label."""
    if not 84 <= len(data) <= MAX_STL_BYTES:
        raise ValueError("Binary STL is empty, truncated or larger than 32 MiB.")
    count = struct.unpack_from("<I", data, 80)[0]
    if count > MAX_TRIANGLES or len(data) != 84 + count * 50:
        raise ValueError("Binary STL triangle count or file length is invalid.")
    records = np.frombuffer(data, dtype=STL_TRIANGLE_DTYPE, offset=84, count=count)
    triangles = records["vertices"].astype(float)
    validate_mesh(triangles)
    return triangles


def read_stl(path: str | Path, scale: float) -> np.ndarray:
    """Load a source STL, multiplying its coordinates by an explicit unit scale."""
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("STL unit scale must be positive and finite.")
    with Path(path).open("rb") as source_file:
        data = source_file.read(MAX_STL_BYTES + 1)
    if len(data) > MAX_STL_BYTES:
        raise ValueError("STL exceeds the 32 MiB file limit.")
    if len(data) >= 84 and len(data) == 84 + struct.unpack_from("<I", data, 80)[0] * 50:
        triangles = mesh_from_stl(data)
    else:
        try:
            source = data.decode("ascii")
            vertices = re.findall(r"^\s*vertex\s+(\S+)\s+(\S+)\s+(\S+)\s*$", source, re.MULTILINE)
            triangles = np.asarray(vertices, dtype=float).reshape(-1, 3, 3)
        except (UnicodeDecodeError, ValueError) as error:
            raise ValueError("STL must contain binary or ASCII triangle data.") from error
        validate_mesh(triangles)
    with np.errstate(over="ignore", invalid="ignore"):
        triangles = triangles * scale
    validate_mesh(triangles)
    return triangles
