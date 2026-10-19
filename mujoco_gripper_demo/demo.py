"""Official robosuite gripper interaction demo using robosuite's MjviewerRenderer.

Source: robosuite v1.5.2, robosuite/demos/demo_gripper_interaction.py.
The unchanged source and MIT license are in upstream/. Scene geometry,
actuator settings, gravity compensation, and the four-phase plan are retained.
Additions: renderer replay keys, finite cycles, lift checks, CSV and plot outputs.
This example uses a standalone RethinkGripper, not a complete robot arm.
"""

import argparse
from contextlib import nullcontext
import csv
from importlib.metadata import version
import json
from pathlib import Path
from threading import Event
import time
import xml.etree.ElementTree as ET

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from robosuite.models import MujocoWorldBase
from robosuite.models.arenas.table_arena import TableArena
from robosuite.models.grippers import RethinkGripper
from robosuite.utils.binding_utils import MjSim
from robosuite.utils.mjcf_utils import new_actuator, new_joint

from robosuite_compat import BoxObject
from robosuite_viewer import RobosuiteViewer


SOURCE = "https://github.com/ARISE-Initiative/robosuite/blob/v1.5.2/robosuite/demos/demo_gripper_interaction.py"
TABLE_Z = 1.1
BOX_HALF_HEIGHT = 0.02
STEPS_PER_PHASE = 500
PHASES = (("open", False, False), ("lower", True, False),
          ("grasp", True, True), ("lift", False, True))


def build_scene() -> tuple[MjSim, RethinkGripper, BoxObject]:
    """Build the scene using the official demo's model and actuator parameters."""
    world = MujocoWorldBase()
    arena = TableArena(table_full_size=(0.4, 0.4, 0.05),
                       table_offset=(0, 0, TABLE_Z), has_legs=False)
    world.merge(arena)

    gripper = RethinkGripper()
    base = ET.Element("body", name="gripper_base", pos="0 0 1.3", quat="0 0 1 0")
    base.append(new_joint(name="gripper_z_joint", type="slide", axis="0 0 1", damping="50"))
    world.worldbody.append(base)
    world.merge(gripper, merge_body="gripper_base")
    world.actuator.append(new_actuator(joint="gripper_z_joint", act_type="position",
                                      name="gripper_z", kp="500"))

    box = BoxObject(name="box", size=[0.02, 0.02, BOX_HALF_HEIGHT],
                    rgba=[1, 0, 0, 1], friction=[1, 0.005, 0.0001])
    box.get_obj().set("pos", "0 0 1.11")
    world.worldbody.append(box.get_obj())
    for name, pos, color in (("x_ref", "0.2 0 1.105", [0, 1, 0, 1]),
                             ("y_ref", "0 0.2 1.105", [0, 0, 1, 1])):
        reference = BoxObject(name=name, size=[0.01, 0.01, 0.01],
                              rgba=color, obj_type="visual", joints=None).get_obj()
        reference.set("pos", pos)
        world.worldbody.append(reference)
    return MjSim(world.get_model(mode="mujoco")), gripper, box


def save_outputs(output_dir, rows, results, timestep, completed, requested):
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "trajectory.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s", "cycle", "phase", "box_x_m", "box_y_m", "box_z_m",
                         "box_bottom_clearance_m", "gripper_box_contacts"])
        writer.writerows(rows)
    success = completed and len(results) == requested and all(r["success"] for r in results)
    summary = {
        "source": SOURCE,
        "adaptation": "robosuite MjviewerRenderer with replay keys, finite cycles, measurements",
        "versions": {name: version(name) for name in ("robosuite", "mujoco", "numpy")},
        "gripper": "RethinkGripper", "full_robot_arm": False,
        "physics_timestep_s": timestep, "steps_per_phase": STEPS_PER_PHASE,
        "cycles_requested": requested, "cycles_completed": len(results),
        "completed": completed, "success": success,
        "success_criterion": "During the last 0.2 s of each lift phase, the box bottom stays > 0.02 m above the table and in contact with the gripper.",
        "cycles": results,
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    samples = np.asarray([[r[0], r[6]] for r in rows])
    fig, ax = plt.subplots(figsize=(9, 4), constrained_layout=True)
    ax.plot(samples[:, 0], 100 * samples[:, 1], label="Measured box bottom clearance")
    ax.axhline(2, color="tab:red", linestyle="--", label="Lift threshold: 2 cm")
    phase_duration = STEPS_PER_PHASE * timestep
    for cycle in range(requested):
        for i, (name, _, _) in enumerate(PHASES):
            start = (cycle * len(PHASES) + i) * phase_duration
            if start > samples[-1, 0]:
                break
            if name in ("grasp", "lift"):
                ax.axvspan(start, min(start + phase_duration, samples[-1, 0]),
                           color="tab:green" if name == "lift" else "tab:orange", alpha=0.12)
            ax.text(start + phase_duration / 2, 1.01, name,
                    transform=ax.get_xaxis_transform(), ha="center", fontsize=9)
    ax.set(xlabel="Simulation time (s)", ylabel="Box bottom above table (cm)",
           xlim=(0, samples[-1, 0]))
    ax.set_title("Official robosuite gripper interaction demo", pad=26)
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left")
    fig.savefig(output_dir / "box_lift_height.png", dpi=160)
    plt.close(fig)
    print(f"{'PASS' if success else 'INCOMPLETE / FAIL'}: {len(results)}/{requested} cycles completed.", flush=True)
    print(f"Outputs: {output_dir.resolve()}", flush=True)
    return success


def run_episode(sim, gripper, box, cycles, viewer, replay_requested):
    """Reset all physics data and execute one batch without recreating the viewer."""
    with viewer.lock() if viewer is not None else nullcontext():
        sim.reset()
        sim.forward()
    if viewer is not None:
        viewer.render()
    box_id = sim.model.body_name2id(box.root_body)
    jaw_geoms = {sim.model.geom_name2id(name) for name in gripper.contact_geoms}
    gravity_indexes = [sim.model.get_joint_qvel_addr("gripper_z_joint")]
    z_actuator = sim.model.actuator_name2id("gripper_z")
    jaw_actuators = [sim.model.actuator_name2id(name) for name in gripper.actuators]
    timestep = float(sim.model.opt.timestep)
    render_stride = max(1, round(1.0 / (20 * timestep)))
    hold_steps = max(1, round(0.2 / timestep))
    rows, results, lift_tail = [], [], []
    total_steps = cycles * len(PHASES) * STEPS_PER_PHASE
    frame_start = time.perf_counter()
    for step in range(total_steps):
        if replay_requested.is_set() or (viewer is not None and not viewer.is_running()):
            break
        phase_index = (step // STEPS_PER_PHASE) % len(PHASES)
        phase, low, closed = PHASES[phase_index]
        cycle = step // (len(PHASES) * STEPS_PER_PHASE) + 1
        if step % STEPS_PER_PHASE == 0:
            print(f"Cycle {cycle}/{cycles}: {phase}", flush=True)
            if phase == "lift":
                lift_tail = []
        with viewer.lock() if viewer is not None else nullcontext():
            # Same commands and gravity compensation order as the official demo.
            sim.data.ctrl[z_actuator] = 0.07 if low else -0.02
            sim.data.ctrl[jaw_actuators] = [0.020833, -0.020833] if closed else [-0.0115, 0.0115]
            sim.step()
            sim.data.qfrc_applied[gravity_indexes] = sim.data.qfrc_bias[gravity_indexes]
            sim.forward()
            position = sim.data.body_xpos[box_id].copy()
            # Compute the lowest corner even if the box tilts.
            rotation = sim.data.body_xmat[box_id].reshape(3, 3)
            vertical_half_extent = BOX_HALF_HEIGHT * np.abs(rotation[2]).sum()
            clearance = float(position[2] - vertical_half_extent - TABLE_Z)
            contact_count = 0
            for contact in sim.data.contact[:sim.data.ncon]:
                a, b = int(contact.geom1), int(contact.geom2)
                if ((a in jaw_geoms and sim.model.geom_bodyid[b] == box_id)
                        or (b in jaw_geoms and sim.model.geom_bodyid[a] == box_id)):
                    contact_count += 1
            rows.append([float(sim.data.time), cycle, phase, *position.tolist(), clearance, contact_count])
        if phase == "lift" and step % STEPS_PER_PHASE >= STEPS_PER_PHASE - hold_steps:
            lift_tail.append((clearance, contact_count))
        if (step + 1) % (len(PHASES) * STEPS_PER_PHASE) == 0:
            minimum = min(value[0] for value in lift_tail)
            contacts_held = all(value[1] > 0 for value in lift_tail)
            success = minimum > 0.02 and contacts_held
            results.append({"cycle": cycle, "success": success,
                            "final_box_bottom_clearance_m": clearance,
                            "minimum_clearance_during_final_hold_m": minimum,
                            "gripper_contact_throughout_final_hold": contacts_held})
            print(f"  {'PASS' if success else 'FAIL'}: held clearance >= {minimum * 100:.2f} cm", flush=True)
        if viewer is not None and ((step + 1) % render_stride == 0 or step == total_steps - 1):
            viewer.render()
            time.sleep(max(0.0, render_stride * timestep - (time.perf_counter() - frame_start)))
            frame_start = time.perf_counter()
    return rows, results, timestep


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run without opening a window.")
    parser.add_argument("--cycles", type=int, default=2, help="Number of open/lower/grasp/lift cycles.")
    window_mode = parser.add_mutually_exclusive_group()
    window_mode.add_argument("--hold", dest="hold", action="store_true",
                             help="Keep the viewer open for R-key replay (default).")
    window_mode.add_argument("--auto-close", dest="hold", action="store_false",
                             help="Close the viewer after saving results.")
    parser.set_defaults(hold=True)
    parser.add_argument("--output-dir", type=Path,
                        default=Path(__file__).resolve().parent / "outputs")
    args = parser.parse_args()
    if args.cycles < 1:
        parser.error("--cycles must be at least 1")

    sim, gripper, box = build_scene()
    viewer = None
    replay_requested = Event()

    def on_key(keycode):
        # Viewer callbacks run on its UI thread. Reset only on the main thread.
        if keycode in (ord("R"), ord("r")):
            replay_requested.set()

    passed = False
    try:
        sim.forward()
        if not args.headless:
            viewer = RobosuiteViewer(sim, key_callback=on_key)
            print("Viewer: robosuite MjviewerRenderer (R-key replay enabled).", flush=True)
            print("Click the viewer to focus it. Press R to restart the demo at any time.", flush=True)

        while True:
            replay_requested.clear()
            rows, results, timestep = run_episode(
                sim, gripper, box, args.cycles, viewer, replay_requested
            )
            if replay_requested.is_set():
                print("Restarting the demo from its initial state...", flush=True)
                continue
            total_steps = args.cycles * len(PHASES) * STEPS_PER_PHASE
            completed = len(rows) == total_steps
            passed = save_outputs(args.output_dir, rows, results, timestep, completed, args.cycles) if rows else False
            if viewer is None or not args.hold or not viewer.is_running():
                break
            print("Results saved. Press R to replay, or close the viewer to exit.", flush=True)
            while viewer.is_running() and not replay_requested.is_set():
                viewer.render()
                time.sleep(0.05)
            if not viewer.is_running():
                break
            print("Restarting the demo from its initial state...", flush=True)
    finally:
        if viewer is not None:
            viewer.close()
        sim.free()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
