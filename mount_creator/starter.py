"""Create an editable document from the bundled mount and its hand alignment."""

import json
from pathlib import Path

from .document import Document, Part
from .geometry import Pose, read_stl


def new_starter_document() -> Document:
    assets = Path(__file__).with_name("assets")
    alignment = json.loads((assets / "starter_mount.json").read_text())
    mesh = read_stl(assets / "starter_mount.stl", scale=0.001)
    return Document(
        parts=[Part("Starter mount", kind="mesh", mesh=mesh)],
        placement=Pose.from_dict(alignment["hand_to_geometry"]),
    )
