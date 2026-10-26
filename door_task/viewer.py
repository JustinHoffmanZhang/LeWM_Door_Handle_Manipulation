"""Single-threaded MuJoCo renderer with project-owned GLFW input callbacks.

The native simulate UI handles keys before launch_passive's Python callback.
This window renders through mjv / mjr directly and never installs that UI.
"""

import os
import sys

import glfw
import mujoco
import numpy as np


KEY_ACTIONS = {
    glfw.KEY_W: (0, 1), glfw.KEY_S: (0, -1),
    glfw.KEY_A: (1, 1), glfw.KEY_D: (1, -1),
    glfw.KEY_Q: (2, 1), glfw.KEY_E: (2, -1),
    glfw.KEY_I: (3, 1), glfw.KEY_K: (3, -1),
    glfw.KEY_J: (4, 1), glfw.KEY_L: (4, -1),
    glfw.KEY_U: (5, 1), glfw.KEY_O: (5, -1),
}


class RobotKeyboard:
    """Held motion keys plus edge-triggered gripper / reset commands."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.pressed = set()
        self.pending = set()
        self.gripper = -1.0
        self.reset_requested = False

    def clear_motion(self):
        self.pressed.clear()
        self.pending.clear()

    def on_key(self, key, event, mods=0):
        if event == glfw.RELEASE:
            self.pressed.discard(key)
            return
        # Ignore OS key repeat and modified OS / desktop shortcuts.
        if event != glfw.PRESS or mods & (glfw.MOD_CONTROL | glfw.MOD_ALT | glfw.MOD_SUPER):
            return
        if key in KEY_ACTIONS:
            self.pressed.add(key)
            self.pending.add(key)  # Preserve a quick tap between control steps.
        elif key == glfw.KEY_SPACE:
            self.gripper *= -1
        elif key == glfw.KEY_R:
            self.reset_requested = True
            self.clear_motion()

    def action(self):
        result = np.zeros(7)
        for key in self.pressed | self.pending:
            axis, direction = KEY_ACTIONS[key]
            # 5 mm / 0.05 rad per 20 Hz control step before controller tracking.
            result[axis] += direction * 0.1
        self.pending.clear()
        result[-1] = self.gripper
        return result


class DoorViewer:
    """One robot-control window; camera interaction is mouse-only."""

    def __init__(self, model, data):
        self.model, self.data = model, data
        self.controls = RobotKeyboard()
        self.window = None
        self.context = None
        self.status = ""
        if not glfw.init():
            raise RuntimeError("Cannot initialize GLFW. Run from a desktop terminal or use --headless.")
        try:
            glfw.default_window_hints()
            # Match robosuite's offscreen API when both contexts share this
            # thread. Mixing an active EGL context with GLX causes BadAccess.
            if sys.platform.startswith("linux") and os.environ.get("MUJOCO_GL") == "egl":
                glfw.window_hint(glfw.CONTEXT_CREATION_API, glfw.EGL_CONTEXT_API)
            self.window = glfw.create_window(1200, 800, "Door task - Panda robot controls", None, None)
            if not self.window:
                raise RuntimeError("Cannot create the door-task OpenGL window.")
            glfw.make_context_current(self.window)
            glfw.swap_interval(1)
            self.cam = mujoco.MjvCamera()
            self.cam.lookat[:] = [-0.25, -0.15, 1.0]
            self.cam.distance = 1.8
            self.cam.azimuth = 215
            self.cam.elevation = -25
            self.opt = mujoco.MjvOption()
            self.opt.geomgroup[0] = 0
            self.scene = mujoco.MjvScene(model, maxgeom=10000)
            self.context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
            self._cursor = glfw.get_cursor_pos(self.window)
            glfw.set_key_callback(self.window, self._on_key)
            glfw.set_window_focus_callback(self.window, self._on_focus)
            glfw.set_cursor_pos_callback(self.window, self._on_cursor)
            glfw.set_scroll_callback(self.window, self._on_scroll)
        except BaseException:
            self.close()
            raise

    def _on_key(self, window, key, scancode, event, mods):
        if key == glfw.KEY_ESCAPE and event == glfw.PRESS:
            glfw.set_window_should_close(window, True)
        else:
            self.controls.on_key(key, event, mods)

    def _on_focus(self, window, focused):
        if not focused:
            self.controls.clear_motion()

    def _on_cursor(self, window, x, y):
        dx, dy = x - self._cursor[0], y - self._cursor[1]
        self._cursor = (x, y)
        height = max(1, glfw.get_window_size(window)[1])
        if glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS:
            motion = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        elif glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS:
            motion = mujoco.mjtMouse.mjMOUSE_MOVE_V
        elif glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS:
            motion = mujoco.mjtMouse.mjMOUSE_ZOOM
        else:
            return
        mujoco.mjv_moveCamera(self.model, motion, dx / height, dy / height, self.scene, self.cam)

    def _on_scroll(self, window, xoffset, yoffset):
        mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM,
                             0, -0.05 * yoffset, self.scene, self.cam)

    def poll_events(self):
        glfw.poll_events()

    def is_running(self):
        return self.window is not None and not glfw.window_should_close(self.window)

    def render(self):
        if not self.is_running():
            return
        glfw.make_context_current(self.window)
        width, height = glfw.get_framebuffer_size(self.window)
        if not width or not height:
            return
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                              mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        mujoco.mjr_render(viewport, self.scene, self.context)
        mujoco.mjr_overlay(
            mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, viewport,
            "ROBOT (hold to move)\nMove world XYZ\nRotate world XYZ\nGripper\nReset / Exit\nCamera",
            "\nW/S  A/D  Q/E\nI/K  J/L  U/O\nSpace\nR / Esc\nLeft drag: orbit | Right drag: pan | Wheel: zoom",
            self.context,
        )
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                           viewport, self.status, "", self.context)
        glfw.swap_buffers(self.window)

    def close(self):
        if self.window is not None:
            glfw.make_context_current(self.window)
            if self.context is not None:
                self.context.free()
                self.context = None
            glfw.destroy_window(self.window)
            self.window = None
        # robosuite may own another GLFW offscreen context in this process;
        # destroy only our window instead of terminating GLFW globally.
