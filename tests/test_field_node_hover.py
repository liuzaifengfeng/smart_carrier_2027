"""地图节点悬停区域与中心值。"""
import unittest

from upper_computer import FIELD_NODE_RADIUS_MM, field_node_at


class FieldNodeHoverTests(unittest.TestCase):
    def test_center_and_edge_are_inside_node_region(self):
        self.assertEqual(field_node_at(240.0, 240.0), (0, 240.0, 240.0))
        self.assertEqual(
            field_node_at(240.0 + FIELD_NODE_RADIUS_MM, 240.0),
            (0, 240.0, 240.0),
        )

    def test_position_outside_node_regions_has_no_center(self):
        self.assertIsNone(field_node_at(480.0, 480.0))


if __name__ == "__main__":
    unittest.main()
