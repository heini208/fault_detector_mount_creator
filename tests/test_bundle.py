"""File-contract tests; no GUI, ROS installation or robot is needed."""

import hashlib
import json
from pathlib import Path
import stat
import tempfile
import unittest
from unittest.mock import patch
import warnings
import zipfile

import numpy as np

from mount_creator.bundle import load_head, save_head
from mount_creator.document import MAX_PARTS, Document, Part
from mount_creator.geometry import Pose, mesh_from_stl


class BundleTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "example.sensorhead"
        source = Part(name="source", dimensions=(0.011, 0.023, 0.037)).triangles()
        self.document = Document(
            name="Test head",
            parts=[
                Part(name="Base", dimensions=(0.1, 0.04, 0.015),
                     pose=Pose(translation=(0.004, 0.01, 0.03))),
                Part(name="Imported", kind="mesh", mesh=source,
                     pose=Pose(translation=(0.02, -0.01, 0.07),
                               quaternion=(0, 0, 0.382683432365, 0.923879532511))),
            ],
            placement=Pose(translation=(0.103123456, -0.002, 0.007),
                           quaternion=(0.258819045103, 0, 0, 0.965925826289)),
            tip=Pose(translation=(0.02, 0.003, 0.0756789),
                     quaternion=(0, 0.707106781187, 0, 0.707106781187)),
        )
        save_head(self.path, self.document)

    def files(self):
        with zipfile.ZipFile(self.path) as archive:
            return {name: archive.read(name) for name in archive.namelist()}

    def rewrite(self, files):
        with zipfile.ZipFile(self.path, "w") as archive:
            for name, data in files.items():
                archive.writestr(name, data)

    def change_manifest(self, key, value):
        files = self.files()
        manifest = json.loads(files["manifest.json"])
        manifest[key] = value
        files["manifest.json"] = json.dumps(manifest)
        self.rewrite(files)

    def test_round_trip_preserves_parts_placement_and_tip(self):
        loaded = load_head(self.path)
        self.assertEqual(loaded.name, self.document.name)
        self.assertEqual([part.kind for part in loaded.parts], ["box", "mesh"])
        np.testing.assert_allclose(loaded.collision_mesh(), self.document.collision_mesh(), atol=1e-8)
        np.testing.assert_allclose(loaded.placement.translation,
                                   self.document.placement.translation, atol=1e-12)
        before, after = self.document.hand_to_probe(), loaded.hand_to_probe()
        self.assertLess(np.linalg.norm(np.subtract(before.translation, after.translation)), 0.00001)
        self.assertAlmostEqual(abs(np.dot(before.quaternion, after.quaternion)), 1, places=12)
        self.assertNotIn("tip", json.loads(self.files()["authoring.json"]))

    def test_runtime_only_file_reopens_as_mesh(self):
        files = self.files()
        del files["authoring.json"]
        del files["source_1.stl"]
        self.rewrite(files)
        loaded = load_head(self.path)
        self.assertEqual(len(loaded.parts), 1)
        self.assertEqual(loaded.parts[0].kind, "mesh")
        np.testing.assert_allclose(loaded.collision_mesh(), self.document.collision_mesh(), atol=1e-8)
        np.testing.assert_allclose(loaded.hand_to_probe().translation,
                                   self.document.hand_to_probe().translation, atol=1e-12)

    def test_rejects_incompatible_manifest(self):
        original = self.path.read_bytes()
        for key, value in (("schema_version", 2), ("schema_version", True),
                           ("units", "mm"), ("reference_frame", "wrist"),
                           ("mounting_template_revision", 99),
                           ("reference_model_fingerprint", "wrong")):
            with self.subTest(key=key, value=value):
                self.path.write_bytes(original)
                self.change_manifest(key, value)
                with self.assertRaises(ValueError):
                    load_head(self.path)

    def test_rejects_mesh_with_wrong_hash(self):
        files = self.files()
        files["collision.stl"] += b"changed"
        self.rewrite(files)
        with self.assertRaisesRegex(ValueError, "checksum"):
            load_head(self.path)

    def test_rejects_malformed_mesh_even_with_correct_hash(self):
        files = self.files()
        files["collision.stl"] = b"not an STL mesh"
        manifest = json.loads(files["manifest.json"])
        manifest["collision_mesh"].update(byte_count=len(files["collision.stl"]),
            sha256=hashlib.sha256(files["collision.stl"]).hexdigest())
        files["manifest.json"] = json.dumps(manifest)
        self.rewrite(files)
        with self.assertRaises(ValueError):
            load_head(self.path)

    def test_rejects_authoring_that_disagrees_with_collision_mesh(self):
        files = self.files()
        authoring = json.loads(files["authoring.json"])
        authoring["parts"][0]["dimensions"][0] += 0.001
        files["authoring.json"] = json.dumps(authoring)
        self.rewrite(files)
        with self.assertRaisesRegex(ValueError, "do not match"):
            load_head(self.path)

    def test_rejects_repeated_mesh_sources_before_decoding_beyond_budget(self):
        files = self.files()
        authoring = json.loads(files["authoring.json"])
        authoring["parts"] = [authoring["parts"][1]] * 3
        files["authoring.json"] = json.dumps(authoring)
        self.rewrite(files)
        # Use a small budget so the fixture stays tiny: each source has 12 triangles.
        with patch("mount_creator.bundle.MAX_TRIANGLES", 24):
            with patch("mount_creator.bundle.mesh_from_stl", wraps=mesh_from_stl) as decode:
                with self.assertRaisesRegex(ValueError, "triangle limit"):
                    load_head(self.path)
        self.assertEqual(decode.call_count, 3)  # Runtime mesh and the first two sources.

    def test_rejects_excessive_box_parts_on_load_and_save(self):
        files = self.files()
        authoring = json.loads(files["authoring.json"])
        authoring["parts"] = [authoring["parts"][0]] * (MAX_PARTS + 1)
        files["authoring.json"] = json.dumps(authoring)
        self.rewrite(files)
        with self.assertRaisesRegex(ValueError, "parts"):
            load_head(self.path)
        original = self.path.read_bytes()
        self.document.parts = [self.document.parts[0]] * (MAX_PARTS + 1)
        with self.assertRaisesRegex(ValueError, "parts"):
            save_head(self.path, self.document)
        self.assertEqual(self.path.read_bytes(), original)

    def test_rejects_invalid_or_unsupported_clearance(self):
        original = self.path.read_bytes()
        for value in (False, True, "0", None, float("inf"), float("nan"), 0.001, -0.001):
            with self.subTest(value=value):
                self.path.write_bytes(original)
                self.change_manifest("clearance_m", value)
                with self.assertRaises(ValueError):
                    load_head(self.path)
        self.path.write_bytes(original)
        self.change_manifest("clearance_m", 0.001)
        with self.assertRaisesRegex(ValueError, "Nonzero clearance metadata is not supported"):
            load_head(self.path)

    def test_rejects_unsafe_archive_paths(self):
        original = self.files()
        for name in ("../escape.stl", "/absolute", "a\\b", "a//b", "C:/file"):
            with self.subTest(name=name):
                self.rewrite({**original, name: b"unused"})
                with self.assertRaisesRegex(ValueError, "Unsafe"):
                    load_head(self.path)

    def test_rejects_duplicate_entries(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            with zipfile.ZipFile(self.path, "a") as archive:
                archive.writestr("manifest.json", b"{}")
        with self.assertRaisesRegex(ValueError, "duplicate"):
            load_head(self.path)

    def test_rejects_symlinks(self):
        entry = zipfile.ZipInfo("linked.stl")
        entry.create_system = 3
        entry.external_attr = (stat.S_IFLNK | 0o777) << 16
        with zipfile.ZipFile(self.path, "a") as archive:
            archive.writestr(entry, "collision.stl")
        with self.assertRaisesRegex(ValueError, "symbolic"):
            load_head(self.path)

    def test_invalid_export_preserves_previous_file(self):
        original = self.path.read_bytes()
        self.document.tip = None
        with self.assertRaisesRegex(ValueError, "tip"):
            save_head(self.path, self.document)
        self.assertEqual(self.path.read_bytes(), original)

    def test_failed_replace_preserves_previous_file_and_removes_temporary(self):
        original = self.path.read_bytes()
        with patch("mount_creator.bundle.os.replace", side_effect=OSError("disk error")):
            with self.assertRaises(OSError):
                save_head(self.path, self.document)
        self.assertEqual(self.path.read_bytes(), original)
        self.assertEqual(list(Path(self.directory.name).iterdir()), [self.path])

    def test_rejects_non_archive(self):
        self.path.write_text("not a ZIP archive")
        with self.assertRaises(ValueError):
            load_head(self.path)


if __name__ == "__main__":
    unittest.main()
