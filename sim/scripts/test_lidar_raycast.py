"""Regression for a scan origin inside the robot's LiDAR housing."""
import math
from types import SimpleNamespace
import unittest

from lidar_raycast import LidarRaycaster, due_ray_end


class Query:
    def __init__(self, hits):
        self.hits = hits

    def raycast_all(self, origin, direction, max_distance, callback):
        for path, distance in self.hits:
            if distance <= max_distance:
                if not callback(SimpleNamespace(collision=path, distance=distance)):
                    break


class LidarTests(unittest.TestCase):
    def test_batched_rolling_acquisition_matches_scalar_at_boundaries(self):
        count, period = 3240, 1/10.0
        for start in (0., period, 17. * period):
            for ray in (0, 1, 143, 1000, 3239):
                boundary = start + ray*period/(count-1)
                for elapsed in (math.nextafter(boundary, -math.inf), boundary,
                                math.nextafter(boundary, math.inf)):
                    expected = 0
                    while expected < count and start + expected*period/(count-1) <= elapsed:
                        expected += 1
                    self.assertEqual(due_ray_end(elapsed, start, 0, count, period), expected)

    def test_nearest_wall_behind_housing_with_unsorted_hits(self):
        query = Query([('/World/AMR/base/lidar', 0.), ('/World/GMap/far_wall', 9.),
                       ('/World/AMR', .02), ('/World/GMap/near_wall', 2.5)])
        lidar = LidarRaycaster(query)
        self.assertEqual(lidar.cast(None, None, 30.), 2.5)
        query.hits = [('/World/AMR/base/lidar', 0.)]
        self.assertTrue(math.isinf(lidar.cast(None, None, 30.)))

    def test_other_robot_and_max_range(self):
        lidar = LidarRaycaster(Query([('/World/AMR_other/body', 1.2),
                                     ('/World/GMap/wall', 31.)]))
        self.assertEqual(lidar.cast(None, None, 30.), 1.2)
        self.assertTrue(math.isinf(lidar.cast(None, None, 1.)))


if __name__ == '__main__':
    unittest.main()
