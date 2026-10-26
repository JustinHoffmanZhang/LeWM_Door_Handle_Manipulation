"""Interactive scene inspection and headless simulation, not an opening policy."""

import argparse
import csv
from importlib.metadata import version
import json
from pathlib import Path
import time

import numpy as np

from .environment import TASK_STATE_FIELDS, make_env


OUTPUT_DIR = Path(__file__).resolve().parents[1] / "outputs" / "door_task"


def save_camera(obs, path):
    import robosuite.macros as macros
    from PIL import Image

    pixels = obs["agentview_image"]
    if macros.IMAGE_CONVENTION == "opengl":
        pixels = pixels[::-1]
    Image.fromarray(pixels).save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run without a window.")
    parser.add_argument("--steps", type=int, default=1000, help="Control steps per episode.")
    parser.add_argument("--camera", action="store_true", help="Enable RGB observations and save preview.png.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--randomize", action="store_true", help="Randomize door placement on reset.")
    parser.add_argument("--auto-close", action="store_true", help="Close the window at the episode horizon.")
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--steps must be at least 1")

    env = make_env(camera_obs=args.camera, seed=args.seed, horizon=args.steps,
                   randomize_placement=args.randomize)
    viewer = None
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "versions": {name: version(name) for name in ("mujoco", "robosuite", "numpy")},
        "seed": args.seed, "randomize_placement": args.randomize,
        "mode": "idle_headless" if args.headless else "keyboard",
        "control_freq_hz": env.control_freq,
        "physics_timestep_s": float(env.sim.model.opt.timestep),
        "action_dim": env.action_dim, "action_frame": "world",
        "success_angle_rad": env.success_angle, "success_hold_steps": env.success_hold_steps,
        "steps_recorded": 0, "episodes_started": 1, "success_observed": False,
        "finished_without_error": False,
        "note": "Environment inspection only; no autonomous door-opening policy.",
    }
    try:
        obs = env.reset()
        if args.camera:
            save_camera(obs, args.output_dir / "preview.png")
        if not args.headless:
            from .viewer import DoorViewer

            viewer = DoorViewer(env.sim.model._model, env.sim.data._data)
            viewer.render()

        print("Panda + lever handle + mechanically latched door ready.", flush=True)
        print("No autonomous policy: use the keyboard to move the robot.", flush=True)
        if viewer is not None:
            print("World XYZ: W/S A/D Q/E | rotation XYZ: I/K J/L U/O", flush=True)
            print("Hold to move; release to stop. Space: gripper | R: reset | Esc: exit", flush=True)
            print("Mouse: left drag orbits, right drag pans, wheel zooms.", flush=True)
        print(f"Outputs: {args.output_dir.resolve()}", flush=True)
        episode, step, done = 0, 0, False
        with (args.output_dir / "state_trace.csv").open("w", newline="", encoding="utf-8") as stream, \
                (args.output_dir / "contact_events.jsonl").open("w", encoding="utf-8") as events_file:
            fields = ["episode", "step", "time", *TASK_STATE_FIELDS,
                      *(f"action_{i}" for i in range(env.action_dim)), "reward", "events"]
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            while viewer is None or viewer.is_running():
                started = time.monotonic()
                action = np.zeros(env.action_dim)
                action[-1] = -1.0
                reset_requested = False
                if viewer is not None:
                    viewer.poll_events()
                    if not viewer.is_running():
                        break
                    reset_requested = viewer.controls.reset_requested
                    if reset_requested:
                        viewer.controls.reset()
                    action = viewer.controls.action()
                if reset_requested:
                    obs = env.reset()
                    episode += 1
                    step, done = 0, False
                    summary["episodes_started"] += 1
                    print(f"Reset: episode {episode}", flush=True)
                if not done:
                    obs, reward, done, info = env.step(action)
                    step += 1
                    state = info["task_state"]
                    for event in info["physics_events"]:
                        events_file.write(json.dumps({"episode": episode, "step": step, **event}) + "\n")
                    writer.writerow({
                        "episode": episode, "step": step, **state,
                        **{f"action_{i}": float(value) for i, value in enumerate(action)},
                        "reward": reward, "events": "|".join(info["events"]),
                    })
                    summary["steps_recorded"] += 1
                    summary["success_observed"] |= info["success"]
                    if info["events"]:
                        print(f"t={state['time']:.2f}s: {', '.join(info['events'])}", flush=True)
                    if done:
                        stream.flush()
                        events_file.flush()
                        print(f"Episode ended: door={np.degrees(state['door_angle']):.1f} deg, "
                              f"success={info['success']}. R resets the scene.", flush=True)
                if viewer is not None:
                    viewer.status = (
                        f"Episode {episode} | Step {step}/{args.steps} | "
                        f"Door {np.degrees(state['door_angle']):.1f} deg | "
                        f"Handle {np.degrees(state['handle_angle']):.1f} deg | "
                        f"Gripper {'OPEN' if viewer.controls.gripper < 0 else 'CLOSE'}"
                        + (" | Episode ended - press R" if done else "")
                    )
                    viewer.render()
                if done and (args.headless or args.auto_close):
                    break
                if viewer is not None:
                    time.sleep(max(0.0, 1 / env.control_freq - (time.monotonic() - started)))
        summary["finished_without_error"] = True
    finally:
        summary["final_state"] = env.get_task_state()
        (args.output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8",
        )
        if viewer is not None:
            viewer.close()
        env.close()
    print(f"Finished {summary['steps_recorded']} steps; results saved.", flush=True)


if __name__ == "__main__":
    main()
