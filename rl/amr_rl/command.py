"""Batched AMR command path: policy action -> (v, w) -> wheel targets, matching the real stack.

Mirrors, per environment:
  * control_mux: clamp |v|, |w| to max_linear/angular_speed, then scale (v, w) together so
    neither wheel exceeds max_wheel_rev_s (curvature preserved).
  * sim/scripts/drive_controller.py WheelCommandLimiter: per-physics-step wheel ramp with
    separate acceleration/deceleration and decelerate-through-zero on reversal.
"""
import math

import torch


class TwistToWheels:
    def __init__(self, wheel_radius, wheel_distance, max_wheel_rad_s, max_linear, max_angular):
        self.r = wheel_radius
        self.half_track = wheel_distance / 2
        self.max_wheel = max_wheel_rad_s
        self.max_linear = max_linear
        self.max_angular = max_angular

    def __call__(self, v, w):
        v = v.clamp(-self.max_linear, self.max_linear)
        w = w.clamp(-self.max_angular, self.max_angular)
        left = (v - w * self.half_track) / self.r
        right = (v + w * self.half_track) / self.r
        scale = torch.maximum(torch.ones_like(left), torch.maximum(left.abs(), right.abs()) / self.max_wheel)
        return torch.stack((left / scale, right / scale), dim=-1)


class WheelRamp:
    """Batched WheelCommandLimiter.step; requested targets are already speed-clamped."""

    def __init__(self, num_envs, acceleration, deceleration, device):
        if not all(math.isfinite(x) and x > 0 for x in (acceleration, deceleration)):
            raise ValueError("Wheel ramp limits must be finite and positive")
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.output = torch.zeros(num_envs, 2, device=device)
        self.requested = torch.zeros(num_envs, 2, device=device)

    def reset(self, env_ids):
        self.output[env_ids] = 0.0
        self.requested[env_ids] = 0.0

    def step(self, dt):
        current, target = self.output, self.requested
        remaining = torch.full_like(current, dt)
        reversing = current * target < 0
        time_to_zero = current.abs() / self.deceleration
        # Reversal not finished this step: keep braking toward zero.
        braking = reversing & (remaining < time_to_zero)
        braked = current - torch.sign(current) * self.deceleration * remaining
        # Reversal finished within the step: start from zero with the leftover time.
        crossed = reversing & ~braking
        remaining = torch.where(crossed, remaining - time_to_zero, remaining)
        current = torch.where(crossed, torch.zeros_like(current), current)
        accelerating = target.abs() > current.abs()
        rate = self.deceleration + (self.acceleration - self.deceleration) * accelerating.to(current.dtype)
        change = torch.clamp(target - current, -rate * remaining, rate * remaining)
        self.output = torch.where(braking, braked, current + change)
        return self.output
