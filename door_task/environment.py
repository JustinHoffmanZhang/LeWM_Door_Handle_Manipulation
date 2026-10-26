"""A fixed, reproducible door task built on robosuite 1.5.2 / MuJoCo 3.3.0."""

from pathlib import Path

import mujoco
import numpy as np
from robosuite.controllers import load_composite_controller_config
from robosuite.environments.manipulation.door import Door
from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.arenas import TableArena
from robosuite.models.objects import DoorObject, MujocoXMLObject
from robosuite.models.tasks import ManipulationTask
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import UniformRandomSampler


TASK_STATE_FIELDS = (
    "door_angle", "door_velocity", "handle_angle", "handle_velocity",
    "left_finger_contacts", "right_finger_contacts", "latch_frame_contacts",
    "latch_normal_force", "handle_turned", "door_released", "door_moving", "success",
)


class LeverDoorObject(DoorObject):
    def __init__(self, name="Door"):
        # Reuse DoorObject's named-site interface with our project-owned MJCF.
        MujocoXMLObject.__init__(
            self, str(Path(__file__).parent / "assets" / "lever_door.xml"),
            name=name, joints=None, obj_type="all", duplicate_collision_geoms=True,
        )
        self.door_body = self.naming_prefix + "door"
        self.frame_body = self.naming_prefix + "frame"
        self.latch_body = self.naming_prefix + "latch"
        self.hinge_joint = self.naming_prefix + "hinge"
        self.lock = True


class DoorHandleEnv(Door):
    """Seven-dimensional OSC action; both door joints remain passive.

    Contact-end is an observed event, not an oracle for unlocking. The
    door_released flag means the door has passed the latch's closed-door stop.
    """

    def __init__(self, *, camera_obs=False, seed=0, horizon=1000,
                 randomize_placement=False, success_angle=np.pi / 3,
                 success_hold_seconds=0.25):
        if not 0.05 < success_angle < np.pi / 2:
            raise ValueError("success_angle must be between 0.05 and pi/2 radians")
        if horizon < 1 or success_hold_seconds <= 0:
            raise ValueError("horizon and success_hold_seconds must be positive")
        self.randomize_placement = randomize_placement
        self.success_angle = float(success_angle)
        self.success_hold_steps = max(1, int(np.ceil(success_hold_seconds * 20)))
        self._goal_steps = 0
        self._succeeded = False
        self._previous_state = None
        self._contact_flags = None
        self._physics_events = []
        config = load_composite_controller_config(robot="Panda")
        # Specify the coordinate frame instead of relying on version defaults.
        config["body_parts"]["right"]["input_ref_frame"] = "world"
        super().__init__(
            robots="Panda", controller_configs=config, initialization_noise=None,
            use_latch=True, use_object_obs=True, use_camera_obs=camera_obs,
            has_renderer=False, has_offscreen_renderer=camera_obs,
            camera_names="agentview", camera_heights=256, camera_widths=256,
            control_freq=20, horizon=horizon, reward_shaping=True,
            hard_reset=False, seed=seed,
        )

    def _load_model(self):
        ManipulationEnv._load_model(self)
        robot = self.robots[0].robot_model
        robot.set_base_xpos(robot.base_xpos_offset["table"](self.table_full_size[0]))
        arena = TableArena(table_full_size=self.table_full_size, table_offset=self.table_offset)
        arena.set_origin([0, 0, 0])
        # Keep the complete robot, door, and handle in the dataset camera.
        camera_pos = np.array([1.2, 0.9, 1.8])
        camera_z = camera_pos - np.array([-0.25, -0.15, 1.0])
        camera_z /= np.linalg.norm(camera_z)
        camera_x = np.cross([0, 0, 1], camera_z)
        camera_x /= np.linalg.norm(camera_x)
        camera_y = np.cross(camera_z, camera_x)
        camera_quat = np.empty(4)
        mujoco.mju_mat2Quat(camera_quat, np.column_stack((camera_x, camera_y, camera_z)).ravel())
        arena.set_camera("agentview", pos=camera_pos, quat=camera_quat)
        self.door = LeverDoorObject()
        self.placement_initializer = UniformRandomSampler(
            name="DoorPlacement", mujoco_objects=self.door,
            x_range=[0.07, 0.09] if self.randomize_placement else [0.08, 0.08],
            y_range=[-0.01, 0.01] if self.randomize_placement else [0, 0],
            rotation=(-np.pi / 2 - 0.15, -np.pi / 2 + 0.15)
            if self.randomize_placement else -np.pi / 2,
            rotation_axis="z", ensure_object_boundary_in_range=False,
            ensure_valid_placement=True, reference_pos=self.table_offset, rng=self.rng,
        )
        self.model = ManipulationTask(arena, [robot], self.door)

    def _setup_references(self):
        super()._setup_references()
        model = self.sim.model
        self.hinge_dof = model.get_joint_qvel_addr(self.door.joints[0])
        self.handle_dof = model.get_joint_qvel_addr(self.door.joints[1])
        ids = lambda names: {model.geom_name2id(name) for name in names}
        prefix = self.door.naming_prefix
        self._handle_geoms = ids([prefix + "handle", prefix + "handle_base"])
        self._latch_geoms = ids([prefix + "latch", prefix + "latch_tip"])
        self._frame_geoms = ids([prefix + "l_frame", prefix + "r_frame", prefix + "top_frame"])
        gripper = self.robots[0].gripper["right"]
        self._left_geoms = ids(gripper.important_geoms["left_finger"])
        self._right_geoms = ids(gripper.important_geoms["right_finger"])

    def _reset_internal(self):
        super()._reset_internal()
        self._goal_steps = 0
        self._succeeded = False
        self._previous_state = None
        self._contact_flags = None
        self._physics_events = []

    def reset(self):
        obs = super().reset()
        self._previous_state = self.get_task_state()
        self._record_contact_events()
        return obs

    def _setup_observables(self):
        observables = super()._setup_observables()

        @sensor(modality="task")
        def task_state(obs_cache):
            state = self.get_task_state()
            return np.array([state[key] for key in TASK_STATE_FIELDS], dtype=np.float64)

        observables["task_state"] = Observable(
            name="task_state", sensor=task_state, sampling_rate=self.control_freq,
        )
        return observables

    def _contact_state(self, include_forces=True):
        data = self.sim.data
        state = {
            "left_finger_contacts": 0, "right_finger_contacts": 0,
            "latch_frame_contacts": 0, "latch_normal_force": 0.0,
        }
        force = np.zeros(6)
        for index, contact in enumerate(data.contact[:data.ncon]):
            if contact.efc_address < 0:
                continue
            a, b = int(contact.geom1), int(contact.geom2)

            def between(first, second):
                return (a in first and b in second) or (b in first and a in second)

            for name, geoms in (("left_finger_contacts", self._left_geoms),
                                ("right_finger_contacts", self._right_geoms)):
                if between(geoms, self._handle_geoms):
                    state[name] += 1
            if between(self._latch_geoms, self._frame_geoms):
                state["latch_frame_contacts"] += 1
                if include_forces:
                    mujoco.mj_contactForce(self.sim.model._model, data._data, index, force)
                    state["latch_normal_force"] += max(0.0, float(force[0]))
        return state

    def _record_contact_events(self):
        state = self._contact_state(include_forces=False)
        flags = {key: bool(state[key]) for key in (
            "left_finger_contacts", "right_finger_contacts", "latch_frame_contacts",
        )}
        if self._contact_flags is not None:
            for key, active in flags.items():
                if active != self._contact_flags[key]:
                    self._physics_events.append({
                        "time": float(self.sim.data.time), "contact": key,
                        "event": "started" if active else "ended",
                    })
        self._contact_flags = flags

    def _pre_action(self, action, policy_step=False):
        if policy_step:
            self._physics_events = []
        # robosuite calls this after collision detection at every 0.002 s step.
        self._record_contact_events()
        super()._pre_action(action, policy_step=policy_step)

    def get_task_state(self):
        """Ground truth diagnostics at the current physics time, in SI units."""
        data = self.sim.data
        state = {
            "time": float(data.time),
            "door_angle": float(data.qpos[self.hinge_qpos_addr]),
            "door_velocity": float(data.qvel[self.hinge_dof]),
            "handle_angle": float(data.qpos[self.handle_qpos_addr]),
            "handle_velocity": float(data.qvel[self.handle_dof]),
            **self._contact_state(),
        }
        state.update(
            handle_turned=abs(state["handle_angle"]) >= 1.25,
            door_released=state["door_angle"] > 0.05,
            door_moving=state["door_angle"] > 0.04 and abs(state["door_velocity"]) > 0.02,
            success=self._succeeded,
        )
        return state

    def _post_action(self, action):
        # Refresh contacts / sensors after the final integration substep.
        self.sim.forward()
        self._record_contact_events()
        state = self.get_task_state()
        self._goal_steps = self._goal_steps + 1 if state["door_angle"] >= self.success_angle else 0
        self._succeeded |= self._goal_steps >= self.success_hold_steps
        state["success"] = self._succeeded
        previous = self._previous_state or state
        events = []
        for key in ("handle_turned", "door_released", "door_moving", "success"):
            if state[key] and not previous[key]:
                events.append("door_motion_started" if key == "door_moving" else key)
        for key, label in (("latch_frame_contacts", "latch_contact"),
                           ("left_finger_contacts", "left_contact"),
                           ("right_finger_contacts", "right_contact")):
            if bool(state[key]) != bool(previous[key]):
                events.append(label + ("_started" if state[key] else "_ended"))
        self._previous_state = state.copy()
        reward, done, info = super()._post_action(action)
        self._update_observables(force=True)
        info.update(task_state=state, events=events, physics_events=list(self._physics_events),
                    success=self._succeeded)
        return reward, done, info

    def _check_success(self):
        return self._succeeded

    def step(self, action):
        action = np.asarray(action, dtype=np.float64)
        low, high = self.action_spec
        if action.shape != low.shape or not np.isfinite(action).all():
            raise ValueError(f"Expected a finite action of shape {low.shape}")
        if np.any(action < low) or np.any(action > high):
            raise ValueError("Action must be within env.action_spec (normally [-1, 1])")
        result = super().step(action)
        if not np.isfinite(self.sim.data.qpos).all() or not np.isfinite(self.sim.data.qvel).all():
            raise RuntimeError("Non-finite simulation state")
        return result


def make_env(**kwargs):
    return DoorHandleEnv(**kwargs)
