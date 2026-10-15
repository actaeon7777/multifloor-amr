"""Install keyboard controls for the generated fourth-floor doors in Isaac Sim.

Run this file once from Isaac Sim's Script Editor after opening
``capstone_amr_4_floor.usda``. Press O to toggle all steel doors and P to
toggle the elevator door. Re-running the file safely replaces the old
keyboard subscription.
"""

from __future__ import annotations

import asyncio

import carb
import omni.appwindow
import omni.kit.app
import omni.usd
from pxr import Gf


class MapDoorController:
    def __init__(self, duration: float = 1.0) -> None:
        self.duration = duration
        self._task: asyncio.Task | None = None
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._subscription = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard_event
        )

    def shutdown(self) -> None:
        if self._subscription is not None:
            self._input.unsubscribe_to_keyboard_events(
                self._keyboard, self._subscription
            )
            self._subscription = None
        if self._task is not None and not self._task.done():
            self._task.cancel()

    def _on_keyboard_event(self, event: carb.input.KeyboardEvent, *_) -> bool:
        if event.type != carb.input.KeyboardEventType.KEY_PRESS:
            return False
        if event.input == carb.input.KeyboardInput.O:
            self.toggle("steel")
            return True
        if event.input == carb.input.KeyboardInput.P:
            self.toggle("elevator")
            return True
        return False

    def toggle(self, door_type: str) -> None:
        if self._task is not None and not self._task.done():
            carb.log_warn("A map door animation is already running")
            return
        self._task = asyncio.ensure_future(self._animate(door_type))

    async def _animate(self, door_type: str) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            carb.log_error("No USD stage is open")
            return

        door_prims = [
            prim
            for prim in stage.Traverse()
            if prim.GetAttribute("door:type").Get() == door_type
        ]
        if not door_prims:
            carb.log_error(f"No {door_type} doors found in the current stage")
            return

        should_open = any(
            not bool(prim.GetAttribute("door:isOpen").Get()) for prim in door_prims
        )
        motions = []
        for prim in door_prims:
            if door_type == "steel":
                attribute = prim.GetAttribute("xformOp:rotateZ:doorMotion")
                target_name = "door:openAngle" if should_open else "door:closedAngle"
                start = float(attribute.Get())
                target = float(prim.GetAttribute(target_name).Get())
                motions.append((attribute, start, target, False))
            else:
                attribute = prim.GetAttribute("xformOp:translate")
                target_name = (
                    "door:openPosition" if should_open else "door:closedPosition"
                )
                start = Gf.Vec3d(attribute.Get())
                target = Gf.Vec3d(prim.GetAttribute(target_name).Get())
                motions.append((attribute, start, target, True))

        application = omni.kit.app.get_app()
        frame_count = max(1, round(self.duration * 60))
        for frame in range(1, frame_count + 1):
            fraction = frame / frame_count
            smooth = fraction * fraction * (3.0 - 2.0 * fraction)
            for attribute, start, target, is_vector in motions:
                value = start + (target - start) * smooth
                attribute.Set(Gf.Vec3d(value) if is_vector else value)
            await application.next_update_async()

        for prim in door_prims:
            prim.GetAttribute("door:isOpen").Set(should_open)
        state = "opened" if should_open else "closed"
        carb.log_info(f"Map {door_type} doors {state}")


try:
    MAP_DOOR_CONTROLLER.shutdown()
except (NameError, AttributeError):
    pass

MAP_DOOR_CONTROLLER = MapDoorController()
carb.log_info("Map door controls installed: O=steel doors, P=elevator door")

