"""Validate physics and task interfaces; fixture torques are NOT robot demonstrations."""

import json
from pathlib import Path

import mujoco
import numpy as np

from .environment import TASK_STATE_FIELDS, make_env


def check_fixture():
    """Isolate the exact project MJCF to test the mechanism without robot interference."""
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).parent / "assets" / "lever_door.xml"))
    assert model.nv == 2 and model.nu == 0, "Door and handle must both be passive"
    hinge = model.joint("hinge").id
    handle = model.joint("latch_joint").id
    hq, lq = model.jnt_qposadr[[hinge, handle]]
    hd, ld = model.jnt_dofadr[[hinge, handle]]
    results = {}
    for name, target in (("locked", 0.0), ("turned", 1.4)):
        data = mujoco.MjData(model)
        data.qpos[lq] = target
        mujoco.mj_forward(model, data)
        peak, contact_steps = 0.0, 0
        for _ in range(round(6.0 / model.opt.timestep)):
            # Diagnostic external torques applied to the isolated fixture only.
            data.qfrc_applied[hd] = 0.5
            data.qfrc_applied[ld] = 20 * (target - data.qpos[lq]) - data.qvel[ld]
            mujoco.mj_step(model, data)
            assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
            peak = max(peak, float(data.qpos[hq]))
            contact_steps += int(data.ncon > 0)
        results[name] = {"max_door_angle_rad": peak, "final_door_angle_rad": float(data.qpos[hq]),
                         "final_handle_angle_rad": float(data.qpos[lq]), "contact_steps": contact_steps}
    assert results["locked"]["max_door_angle_rad"] < 0.05, "Latch did not block the door"
    assert results["locked"]["contact_steps"] > 0, "No physical latch contact"
    assert results["turned"]["final_door_angle_rad"] > np.pi / 3, "Turned handle did not release the door"
    data = mujoco.MjData(model)
    data.qpos[lq] = 1.0
    for _ in range(round(6.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert abs(data.qpos[lq]) < 0.2, "Handle return spring did not restore the handle"
    results["spring_return_angle_rad"] = float(data.qpos[lq])
    results["diagnostic_only"] = True
    return results


def check_environment():
    env = make_env(horizon=200)
    try:
        obs = env.reset()
        initial_qpos = env.sim.data.qpos.copy()
        initial_handle = obs["handle_pos"].copy()
        assert env.action_dim == 7
        assert obs["task_state"].shape == (len(TASK_STATE_FIELDS),)
        assert not env.get_task_state()["success"]
        action = np.zeros(env.action_dim)
        action[-1] = -1
        for index in range(200):
            obs, _, done, info = env.step(action)
            assert done == (index == 199)
            np.testing.assert_allclose(obs["task_state"], [info["task_state"][key] for key in TASK_STATE_FIELDS])
            assert not info["success"]
        idle_state = env.get_task_state()
        assert abs(idle_state["door_angle"]) < 0.02
        obs = env.reset()
        np.testing.assert_allclose(env.sim.data.qpos, initial_qpos, atol=1e-12)
        np.testing.assert_allclose(obs["handle_pos"], initial_handle, atol=1e-12)
        assert env.sim.data.time == 0 and not env.done
        assert env._physics_events == []
        for invalid in (np.full(7, np.nan), np.zeros(6), np.full(7, 2.0)):
            try:
                env.step(invalid)
            except ValueError:
                pass
            else:
                raise AssertionError("Invalid action was accepted")
        assert env.sim.data.time == 0

        start_eef = obs["robot0_eef_pos"].copy()
        action[0] = 0.2
        for _ in range(5):
            obs, _, _, _ = env.step(action)
        action[0] = 0
        for _ in range(10):
            obs, _, _, _ = env.step(action)
        eef_delta = obs["robot0_eef_pos"] - start_eef
        assert eef_delta[0] > 0.005, "World-X OSC action did not move the end effector"
        # Panda's two finger joints share an axis and have opposite signed ranges.
        open_width = float(obs["robot0_gripper_qpos"][0] - obs["robot0_gripper_qpos"][1])
        action[-1] = 1
        for _ in range(20):
            obs, _, _, _ = env.step(action)
        closed_width = float(obs["robot0_gripper_qpos"][0] - obs["robot0_gripper_qpos"][1])
        assert closed_width < open_width - 0.01, "Gripper did not close"
        env.reset()
        action[-1] = -1

        # Verify the dwell-time criterion and its reset using a prepared open state.
        # This is a criterion check, not a robot-generated successful episode.
        env.sim.data.qpos[env.hinge_qpos_addr] = 1.2
        env.sim.forward()
        for index in range(env.success_hold_steps):
            obs, _, _, info = env.step(action)
            assert info["success"] == (index == env.success_hold_steps - 1)
        assert "success" in info["events"]
        obs = env.reset()
        assert not env.get_task_state()["success"] and obs["task_state"][-1] == 0

        # Exercise actual latch contact through the environment's physics-step hooks.
        # External torque here is diagnostic, never part of the action interface.
        contact_events = []
        env.sim.data.qfrc_applied[env.hinge_dof] = 0.5
        for _ in range(40):
            start = float(env.sim.data.time)
            _, _, _, info = env.step(action)
            for event in info["physics_events"]:
                assert start - 1e-9 <= event["time"] <= env.sim.data.time + 1e-9
            contact_events.extend(info["physics_events"])
        assert any(event["contact"] == "latch_frame_contacts" and event["event"] == "started"
                   for event in contact_events)
        assert env.get_task_state()["latch_frame_contacts"] > 0
        assert env.get_task_state()["latch_normal_force"] > 0
        env.reset()
        assert env._physics_events == [] and np.all(env.sim.data.qfrc_applied == 0)
        return {"idle_steps": 200, "idle_final_state": idle_state,
                "reset_reproducible": True, "invalid_actions_rejected": True,
                "success_hold_and_reset": True, "physics_contact_events": contact_events,
                "world_x_action_eef_delta_m": eef_delta.tolist(),
                "gripper_open_width_m": open_width, "gripper_closed_width_m": closed_width}
    finally:
        env.close()


def main():
    report = {"fixture": check_fixture(), "environment": check_environment(), "passed": True}
    output = Path(__file__).resolve().parents[1] / "outputs" / "door_task" / "validation.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"PASS: {output}")


if __name__ == "__main__":
    main()
