"""Boundary and reversal checks independent of Isaac Sim."""
import unittest
from drive_controller import WheelCommandLimiter


class CommandTests(unittest.TestCase):
    def make_limiter(self):
        return WheelCommandLimiter({"command_limit_rad_s": 12, "command_acceleration_rad_s2": 4,
                                    "command_deceleration_rad_s2": 6})

    def test_saturation_preserves_curvature(self):
        limiter = self.make_limiter()
        limiter.command(100, -50)
        for _ in range(1000):
            output = limiter.step(0.005)
            self.assertLessEqual(max(map(abs, output)), 12)
        self.assertEqual(output, (12, -6))

    def test_braking_and_reversal_split_at_zero(self):
        limiter = self.make_limiter()
        limiter.command(4, 4)
        self.assertEqual(limiter.step(1), (4, 4))
        limiter.command(-4, -4)
        # 2/3 s braking, then 1/3 s accelerating in the opposite direction.
        for value in limiter.step(1):
            self.assertAlmostEqual(value, -4 / 3)
        limiter.command(0, 0)
        self.assertEqual(limiter.step(1), (0, 0))

    def test_variable_steps_and_invalid_commands(self):
        limiter = self.make_limiter()
        previous = (0, 0)
        for target in ((100, -100), (-100, 100), (0, 0)):
            limiter.command(*target)
            for dt in [0.003, 0.007, 0.02] * 100:
                output = limiter.step(dt)
                self.assertTrue(all(abs(a - b) <= 6 * dt + 1e-10 for a, b in zip(output, previous)))
                previous = output
        for bad in (float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                limiter.command(bad, 0)
        for bad in (0, -1, float('nan')):
            with self.assertRaises(ValueError):
                limiter.step(bad)
        limiter.reset()
        self.assertEqual(limiter.output, (0, 0))
        self.assertEqual(limiter.requested, (0, 0))


if __name__ == '__main__':
    unittest.main()
