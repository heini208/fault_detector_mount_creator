"""Read and write the portable, ROS-independent .sensorhead archive."""

import hashlib
import json
import os
from pathlib import Path
import stat
import struct
import tempfile
import zipfile
import zlib

import numpy as np

from .document import MAX_PARTS, Document, Part
from .geometry import MAX_TRIANGLES, Pose, mesh_from_stl, mesh_to_stl, validate_mesh


MAX_FILE_BYTES = 32 * 1024 * 1024
MAX_EXPANDED_BYTES = 128 * 1024 * 1024
MAX_ENTRIES = 256


def _json_bytes(value):
    return json.dumps(value, indent=2, allow_nan=False).encode("utf-8")


def _reject_constant(value):
    raise ValueError(f"Invalid JSON number: {value}")


def _json_object(data):
    value = json.loads(data, parse_constant=_reject_constant)
    if not isinstance(value, dict):
        raise ValueError("Expected a JSON object.")
    return value


def _reference():
    path = Path(__file__).parent / "assets" / "reference.json"
    return _json_object(path.read_bytes())


def _asset(path, data):
    return {
        "path": path,
        "sha256": hashlib.sha256(data).hexdigest(),
        "byte_count": len(data),
    }


def _pose(value):
    if not isinstance(value, dict):
        raise ValueError("A transform must be a JSON object.")
    return Pose.from_dict(value)


def _name(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError("Names must be nonempty text.")
    return value


def save_head(path, document):
    """Atomically save collision geometry, tip pose and editable source parts."""
    if document.tip is None:
        raise ValueError("Set the sensor tip before exporting.")
    reference = _reference()
    files = {"collision.stl": mesh_to_stl(document.collision_mesh())}
    manifest = {
        "schema_version": 1,
        "display_name": _name(document.name),
        "reference_frame": "hand",
        "units": "m",
        "mounting_template_id": reference["id"],
        "mounting_template_revision": reference["revision"],
        "reference_model_fingerprint": reference["fingerprint"],
        "mounting_template_verified": reference["verified"],
        "hand_to_geometry": document.placement.to_dict(),
        "hand_to_probe": document.hand_to_probe().to_dict(),
        "collision_mesh": _asset("collision.stl", files["collision.stl"]),
        "clearance_m": 0,
        "configuration_method": "manual",
    }
    parts = []
    for index, part in enumerate(document.parts):
        item = {"name": _name(part.name), "kind": part.kind, "pose": part.pose.to_dict()}
        if part.kind == "mesh":
            source_path = f"source_{index}.stl"
            files[source_path] = mesh_to_stl(part.mesh)
            item["source_asset"] = _asset(source_path, files[source_path])
        else:
            item["dimensions"] = list(part.dimensions)
        parts.append(item)
    files["manifest.json"] = _json_bytes(manifest)
    files["authoring.json"] = _json_bytes({"schema_version": 1, "parts": parts})
    if len(files) > MAX_ENTRIES or sum(map(len, files.values())) > MAX_EXPANDED_BYTES:
        raise ValueError("The head contains too much data.")
    if any(len(data) > MAX_FILE_BYTES for data in files.values()):
        raise ValueError("An archive entry exceeds the 32 MiB limit.")

    destination = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=destination.parent, suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
        with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            for name, data in files.items():
                archive.writestr(name, data)
        if temporary.stat().st_size > MAX_FILE_BYTES:
            raise ValueError("The archive exceeds the 32 MiB limit.")
        os.replace(temporary, destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _check_archive(archive):
    entries = archive.infolist()
    names = [entry.filename for entry in entries]
    if len(entries) > MAX_ENTRIES or len(set(names)) != len(names):
        raise ValueError("The archive has too many entries or duplicate filenames.")
    if sum(entry.file_size for entry in entries) > MAX_EXPANDED_BYTES:
        raise ValueError("The expanded archive exceeds the 128 MiB limit.")
    for entry in entries:
        name = entry.filename
        if "\\" in name or ":" in name or any(part in ("", ".", "..") for part in name.split("/")):
            raise ValueError(f"Unsafe archive filename: {name!r}")
        if stat.S_ISLNK(entry.external_attr >> 16):
            raise ValueError("Archive entries cannot be symbolic links.")
        if entry.file_size > MAX_FILE_BYTES or entry.flag_bits & 1:
            raise ValueError("Archive entries must be unencrypted and at most 32 MiB.")


def _read_asset(archive, description, triangle_budget=MAX_TRIANGLES):
    if not isinstance(description, dict):
        raise ValueError("Missing mesh asset description.")
    name = description["path"]
    size = description["byte_count"]
    checksum = description["sha256"]
    if not isinstance(name, str) or type(size) is not int or not isinstance(checksum, str):
        raise ValueError("Invalid mesh asset description.")
    data = archive.read(name)
    if len(data) != size or hashlib.sha256(data).hexdigest() != checksum:
        raise ValueError(f"Mesh size or checksum does not match: {name}")
    if len(data) >= 84 and struct.unpack_from("<I", data, 80)[0] > triangle_budget:
        raise ValueError(f"Mesh sources exceed the {MAX_TRIANGLES:,} triangle limit.")
    return mesh_from_stl(data)


def _load(archive):
    _check_archive(archive)
    manifest = _json_object(archive.read("manifest.json"))
    reference = _reference()
    expected = {
        "schema_version": 1,
        "reference_frame": "hand",
        "units": "m",
        "mounting_template_id": reference["id"],
        "mounting_template_revision": reference["revision"],
        "reference_model_fingerprint": reference["fingerprint"],
        "mounting_template_verified": reference["verified"],
    }
    for key, value in expected.items():
        if type(manifest.get(key)) is not type(value) or manifest.get(key) != value:
            raise ValueError(f"Unsupported or incompatible {key}.")
    clearance = manifest.get("clearance_m")
    if type(clearance) not in (int, float) or not np.isfinite(clearance):
        raise ValueError("Clearance metadata must be a finite number.")
    if clearance != 0:
        raise ValueError("Nonzero clearance metadata is not supported by this editor version.")
    placement = _pose(manifest["hand_to_geometry"])
    tip = placement.inverse().compose(_pose(manifest["hand_to_probe"]))
    mesh = _read_asset(archive, manifest["collision_mesh"])
    name = _name(manifest["display_name"])
    parts = [Part(name=name, kind="mesh", mesh=mesh)]
    if "authoring.json" in archive.namelist():
        authoring = _json_object(archive.read("authoring.json"))
        if type(authoring.get("schema_version")) is not int or authoring["schema_version"] != 1:
            raise ValueError("Unsupported authoring schema version.")
        if not isinstance(authoring.get("parts"), list) or not 1 <= len(authoring["parts"]) <= MAX_PARTS:
            raise ValueError(f"Authoring data must contain 1 to {MAX_PARTS} parts.")
        parts = []
        triangle_count = 0
        for item in authoring["parts"]:
            if not isinstance(item, dict) or item.get("kind") not in ("box", "mesh"):
                raise ValueError("Unsupported part kind.")
            arguments = {"name": _name(item["name"]), "kind": item["kind"], "pose": _pose(item["pose"])}
            if item["kind"] == "mesh":
                arguments["mesh"] = _read_asset(archive, item["source_asset"], MAX_TRIANGLES - triangle_count)
                triangle_count += len(arguments["mesh"])
            else:
                arguments["dimensions"] = tuple(item["dimensions"])
                triangle_count += 12
            if triangle_count > MAX_TRIANGLES:
                raise ValueError(f"Parts exceed the {MAX_TRIANGLES:,} triangle limit.")
            parts.append(Part(**arguments))
    document = Document(name=name, parts=parts, placement=placement, tip=tip)
    rebuilt = document.collision_mesh()
    validate_mesh(rebuilt)
    if rebuilt.shape != mesh.shape or not np.allclose(rebuilt, mesh, rtol=2e-6, atol=1e-7):
        raise ValueError("Editable parts do not match the exported collision mesh.")
    return document


def load_head(path):
    """Validate and reopen an archive without extracting files to disk."""
    if Path(path).stat().st_size > MAX_FILE_BYTES:
        raise ValueError("The archive exceeds the 32 MiB limit.")
    try:
        with zipfile.ZipFile(path) as archive:
            return _load(archive)
    except (zipfile.BadZipFile, KeyError, TypeError, UnicodeError, OverflowError,
            IndexError, RuntimeError, NotImplementedError, zlib.error) as error:
        raise ValueError(f"Invalid sensor-head file: {error}") from error
