"""robosuite's MjviewerRenderer with replay keys for a standalone MjSim.

In robosuite 1.5.2, add_keypress_callback() stores the callback but update()
does not pass it to MuJoCo. Initialize the handle with that callback here;
frame updates and closing use the original renderer implementation.
"""

from collections.abc import Callable
from contextlib import nullcontext
from types import SimpleNamespace

from mujoco import viewer as mujoco_viewer
from robosuite.renderers.viewer import MjviewerRenderer
from robosuite.utils.binding_utils import MjSim


class RobosuiteViewer(MjviewerRenderer):
    """Use robosuite's renderer with the demo's existing simulation and R key."""

    def __init__(self, sim: MjSim, key_callback: Callable[[int], None]):
        # MjviewerRenderer only needs env.sim; a full robot environment is unnecessary.
        camera = {"lookat": [0, 0, 1.18], "distance": 0.9,
                  "azimuth": 135, "elevation": -25}
        super().__init__(env=SimpleNamespace(sim=sim), camera_id=-1, cam_config=camera)
        self.add_keypress_callback(key_callback)
        self.viewer = mujoco_viewer.launch_passive(
            sim.model._model, sim.data._data,
            key_callback=key_callback, show_left_ui=False, show_right_ui=False,
        )
        try:
            with self.viewer.lock():
                # Match robosuite's default: show visual geoms, hide collision geoms.
                self.viewer.opt.geomgroup[0] = 0
                self.viewer.cam.lookat[:] = camera["lookat"]
                self.viewer.cam.distance = camera["distance"]
                self.viewer.cam.azimuth = camera["azimuth"]
                self.viewer.cam.elevation = camera["elevation"]
                self.viewer.cam.type = 0  # Free camera.
            self.render()
        except BaseException:
            self.close()
            raise

    def render(self) -> None:
        # The upstream render() is empty; update() performs the actual synchronization.
        if self.is_running():
            super().update()

    def is_running(self) -> bool:
        return self.viewer is not None and self.viewer.is_running()

    def lock(self):
        return self.viewer.lock() if self.viewer is not None else nullcontext()
