from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = (
    Path(__file__).resolve().parents[2]
    / "ros2_ws"
    / "src"
    / "amr_controller"
)
sys.path.insert(0, str(PACKAGE_ROOT))

from amr_controller.map_utils import build_occupancy_grid  # noqa: E402


class MapUtilsTest(unittest.TestCase):
    def test_border_is_occupied_and_interior_is_free(self):
        data = build_occupancy_grid(5, 4, 1)

        self.assertEqual(len(data), 20)
        self.assertEqual(data[0:5], [100, 100, 100, 100, 100])
        self.assertEqual(data[5:10], [100, 0, 0, 0, 100])
        self.assertEqual(data[10:15], [100, 0, 0, 0, 100])
        self.assertEqual(data[15:20], [100, 100, 100, 100, 100])

    def test_multiple_border_cells(self):
        data = build_occupancy_grid(6, 6, 2)
        self.assertEqual(data[2 * 6 + 2], 0)
        self.assertEqual(data[1 * 6 + 3], 100)

    def test_invalid_dimensions_raise(self):
        with self.assertRaises(ValueError):
            build_occupancy_grid(0, 4, 1)
        with self.assertRaises(ValueError):
            build_occupancy_grid(4, 4, 2)


if __name__ == "__main__":
    unittest.main()

