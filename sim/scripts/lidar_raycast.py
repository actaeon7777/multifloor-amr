"""PhysX LiDAR rays that ignore the sensor's own robot collision geometry."""
import math


def due_ray_end(elapsed, scan_start, index, count, period):
    """Exclusive end of the rays due at this step, matching scalar acquisition."""
    end = min(count, max(index, math.floor((elapsed-scan_start)*(count-1)/period)+1))
    # Preserve the original floating-point boundary test at exact sample times.
    while end < count and scan_start + end*period/(count-1) <= elapsed:
        end += 1
    while end > index and scan_start + (end-1)*period/(count-1) > elapsed:
        end -= 1
    return end


class LidarRaycaster:
    backend = 'physx_scene_query_python'

    def __init__(self, query, robot_path='/World/AMR'):
        self.query = query
        self.robot_path = robot_path.rstrip('/')
        self.robot_prefix = self.robot_path + '/'
        self.distance = math.inf

    def _report_hit(self, hit):
        path = str(hit.collision)
        if path != self.robot_path and not path.startswith(self.robot_prefix):
            self.distance = min(self.distance, float(hit.distance))
        return True

    def cast(self, origin, direction, max_distance):
        self.distance = math.inf
        # The optical origin is inside the LiDAR housing. A closest-only query
        # stops at that housing and loses the wall behind it. PhysX reports all
        # hits in unspecified order, so retain the nearest non-robot collider.
        self.query.raycast_all(origin, direction, max_distance, self._report_hit)
        return self.distance

    def cast_batch(self, origin, directions, max_distance):
        import carb
        ray_origin = carb.Float3(*map(float, origin))
        return [self.cast(ray_origin, carb.Float3(*map(float, direction)), max_distance)
                for direction in directions]

    def close(self):
        self.query = None
