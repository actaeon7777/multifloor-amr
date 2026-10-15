"""The AMR command path: SI wheel commands, common speed clamp and per-step ramp."""
import math


class WheelCommandLimiter:
    def __init__(self, drive):
        self.speed_limit = drive["command_limit_rad_s"]
        self.acceleration = drive["command_acceleration_rad_s2"]
        self.deceleration = drive["command_deceleration_rad_s2"]
        if not all(math.isfinite(v) and v > 0 for v in (self.speed_limit, self.acceleration, self.deceleration)):
            raise ValueError("Wheel limits must be finite and positive")
        self.reset()

    def reset(self):
        self.requested = (0.0, 0.0)
        self.output = (0.0, 0.0)

    def command(self, left, right):
        if not all(math.isfinite(v) for v in (left, right)):
            raise ValueError("Wheel commands must be finite")
        # Scale both together so saturation preserves the requested curvature.
        scale = max(1.0, abs(left) / self.speed_limit, abs(right) / self.speed_limit)
        self.requested = (left / scale, right / scale)

    def step(self, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Physics dt must be finite and positive")
        result = []
        for current, target in zip(self.output, self.requested):
            remaining = dt
            if current * target < 0:
                time_to_zero = abs(current) / self.deceleration
                if remaining < time_to_zero:
                    result.append(current - math.copysign(self.deceleration * remaining, current))
                    continue
                remaining -= time_to_zero
                current = 0.0
            rate = self.acceleration if abs(target) > abs(current) else self.deceleration
            change = max(-rate * remaining, min(rate * remaining, target - current))
            result.append(current + change)
        self.output = tuple(result)
        return self.output


class AMRDriveController:
    """Use command_wheels/command_twist; direct USD/tensor writes bypass this controller."""
    def __init__(self, articulation_path, config):
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

        self.path = articulation_path
        self.config = config
        self.limiter = WheelCommandLimiter(config["drive"])
        self.robot = None
        self.dof_indices = None
        self.history = None
        self._callbacks = [
            SimulationManager.register_callback(self._step, SimulationEvent.PHYSICS_PRE_STEP),
            SimulationManager.register_callback(self._stop, SimulationEvent.SIMULATION_STOPPED),
        ]

    def command_wheels(self, left, right):
        self.limiter.command(left, right)

    def command_twist(self, linear_m_s, angular_rad_s):
        geometry = self.config["kinematics"]
        r, half_track = geometry["wheel_radius_m"], geometry["wheel_distance_m"] / 2
        self.command_wheels((linear_m_s - angular_rad_s * half_track) / r,
                            (linear_m_s + angular_rad_s * half_track) / r)

    def _step(self, dt, context):
        if self.robot is None:
            from isaacsim.core.experimental.prims import Articulation
            self.robot = Articulation(self.path)
            self.dof_indices = self.robot.get_dof_indices(["left_drive_wheel_joint", "right_drive_wheel_joint"])
        wheels = self.limiter.step(dt)
        self.robot.set_dof_velocity_targets([wheels], dof_indices=self.dof_indices)
        if self.history is not None:
            self.history.append((float(dt), *wheels))

    def _stop(self, event):
        self.robot = None
        self.dof_indices = None
        self.limiter.reset()

    def close(self):
        from isaacsim.core.simulation_manager import SimulationManager
        for uid in self._callbacks:
            SimulationManager.deregister_callback(uid)
        self._callbacks.clear()
