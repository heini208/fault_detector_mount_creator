"""The editable head document is the application's single source of geometry."""

from dataclasses import dataclass, field

import numpy as np

from .geometry import MAX_TRIANGLES, Pose, box_mesh


MAX_PARTS = 128


@dataclass
class Part:
    name: str
    kind: str = "box"
    dimensions: tuple[float, float, float] = (0.02, 0.02, 0.02)
    pose: Pose = field(default_factory=Pose)
    mesh: np.ndarray | None = None

    def triangles(self) -> np.ndarray:
        if self.kind == "box":
            triangles = box_mesh(self.dimensions)
        elif self.kind == "mesh" and self.mesh is not None:
            triangles = self.mesh
        else:
            raise ValueError("Part must be a box or contain an imported mesh.")
        return self.pose.apply(triangles)


def _check_part_limits(parts: list[Part]) -> None:
    if not parts:
        raise ValueError("Add at least one collision part before exporting.")
    if len(parts) > MAX_PARTS:
        raise ValueError(f"A document may contain at most {MAX_PARTS} parts.")
    triangle_count = 0
    for part in parts:
        if part.kind == "box":
            triangle_count += 12
        elif part.kind == "mesh" and part.mesh is not None:
            triangle_count += len(part.mesh)
        else:
            raise ValueError("Part must be a box or contain an imported mesh.")
        if triangle_count > MAX_TRIANGLES:
            raise ValueError(f"Combined collision mesh exceeds {MAX_TRIANGLES} triangles.")


@dataclass
class Document:
    name: str = "Untitled head"
    parts: list[Part] = field(default_factory=list)
    placement: Pose = field(default_factory=Pose)
    tip: Pose | None = None

    def add_part(self, part: Part) -> None:
        _check_part_limits([*self.parts, part])
        self.parts.append(part)

    def hand_to_probe(self) -> Pose:
        if self.tip is None:
            raise ValueError("Define the sensor tip before exporting.")
        return self.placement.compose(self.tip)

    def collision_mesh(self) -> np.ndarray:
        _check_part_limits(self.parts)
        return np.concatenate([part.triangles() for part in self.parts])
