"""Distances from exact mesh planes, independent of display or robot frames."""

import unittest

import numpy as np

from mount_creator.geometry import (
    PLANE_AXIS_EPSILON, plane_from_triangle, point_at_surface_distance,
    surface_axis_distance,
)


class SurfaceGeometryTests(unittest.TestCase):
    def test_triangle_plane_uses_vertices_and_rejects_invalid_triangles(self):
        triangle = np.array(((0.02, 0.03, 0.04), (0.03, 0.03, 0.03),
                             (0.02, 0.04, 0.02)))
        point, normal = plane_from_triangle(triangle)
        np.testing.assert_array_equal(point, triangle[0])
        np.testing.assert_allclose(normal, np.array((1, 2, 1)) / np.sqrt(6))
        point[0] = 123
        self.assertEqual(triangle[0, 0], 0.02)
        for invalid in (triangle[:2], np.full((3, 3), np.nan), np.zeros((3, 3)),
                        ((0, 0, 0), (1, 2, 3), (2, 4, 6))):
            with self.subTest(triangle=invalid), self.assertRaises(ValueError):
                plane_from_triangle(invalid)

    def test_oblique_plane_distances_follow_each_axis_and_ignore_winding(self):
        point = np.array((0.01, -0.02, 0.03))
        tip = point + (0.006, -0.004, 0.008)
        normal = np.array((2, -3, 4))
        # n dot (tip - point) = 0.056; divide by each axis component.
        for axis, expected in enumerate((0.028, -0.056 / 3, 0.014)):
            for scale in (1, -1, 7):
                self.assertAlmostEqual(surface_axis_distance(point, normal * scale, tip, axis),
                                       expected)
            # Changing the chosen point within the same plane changes nothing.
            moved_point = point + (0.003, 0.002, 0)
            self.assertAlmostEqual(surface_axis_distance(moved_point, normal, tip, axis), expected)

    def test_requested_signed_distance_changes_only_one_coordinate(self):
        point = np.array((0.01, -0.02, 0.03))
        normal = np.array((2, -3, 4))
        tip = np.array((0.03, 0.05, -0.01))
        original = tip.copy()
        for axis in range(3):
            for requested in (-0.0123456, 0, 0.0076543):
                with self.subTest(axis=axis, distance=requested):
                    changed = point_at_surface_distance(point, normal, tip, axis, requested)
                    unchanged_axes = np.arange(3) != axis
                    np.testing.assert_array_equal(changed[unchanged_axes], tip[unchanged_axes])
                    self.assertAlmostEqual(surface_axis_distance(point, normal, changed, axis),
                                           requested)
                    intersection = changed.copy()
                    intersection[axis] -= requested
                    self.assertAlmostEqual(np.dot(normal, intersection - point), 0)
        np.testing.assert_array_equal(tip, original)

    def test_parallel_threshold_uses_unit_normal_and_rejects_invalid_input(self):
        for normal in ((0, 1, 0), (PLANE_AXIS_EPSILON / 2, 1, 0),
                       (PLANE_AXIS_EPSILON * 5, 10, 0)):
            with self.subTest(normal=normal), self.assertRaisesRegex(ValueError, "parallel"):
                surface_axis_distance((0, 0, 0), normal, (1, 2, 3), 0)
        self.assertTrue(np.isfinite(surface_axis_distance(
            (0, 0, 0), (PLANE_AXIS_EPSILON * 2, 1, 0), (1, 2, 3), 0)))
        valid = [(0, 0, 0), (1, 0, 0), (1, 2, 3), 0]
        for index, invalid in ((0, (0, 0)), (1, (0, 0, 0)), (1, (np.inf, 0, 0)),
                               (2, (np.nan, 0, 0)), (3, -1), (3, 3), (3, 1.0), (3, True)):
            args = valid.copy()
            args[index] = invalid
            with self.subTest(index=index, invalid=invalid), self.assertRaises(ValueError):
                surface_axis_distance(*args)
        for distance in (np.nan, np.inf):
            with self.assertRaises(ValueError):
                point_at_surface_distance(*valid, distance)


if __name__ == "__main__":
    unittest.main()
