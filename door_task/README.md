# 门把手操作环境

使用 MuJoCo 3.3.0、robosuite 1.5.2 和 Python 3.10，建立可供脚本策略和 LeWM 使用的 `reset()` / `step(action)` 环境。
当前实现环境、键盘操作、状态与接触事件记录；自动抓握开门策略、训练数据集、LeWM 训练和 CEM 规划尚未接入。

## 场景与任务

场景由 Panda 七自由度机械臂、双指夹爪、桌面、固定门框、门板、杠杆式把手及旋转锁舌组成。
门板和把手都是被动关节。锁舌通过与门框的物理接触阻挡开门，把手转动后锁舌可以绕开门框；代码没有通过角度阈值切换锁定约束，也没有给门和把手添加执行器。

| 参数 | 当前设置 |
| --- | --- |
| 门板尺寸 | 0.44 × 0.04 × 0.58 m |
| 门板质量 | 约 2.43 kg，沿用上游参数 |
| 门铰链范围 | 0～90°，MuJoCo 软约束可能产生少量越界 |
| 把手关节范围 | 约 −90～90°，带回位弹簧、摩擦和阻尼 |
| 物理步长 | 0.002 s，500 Hz |
| 策略频率 | 20 Hz，每次动作推进 25 个物理步 |
| 默认初始状态 | 固定门的位置、方向和机器人姿态 |
| 成功条件 | 连续 5 个控制步采样到门角度 ≥60° |
| 回合长度 | 默认 1000 个控制步，即 50 s |

成功标记在本回合内保持为真，`reset()` 后清零；`done` 表示达到回合时长，不会因成功而提前结束。
这是基于 robosuite 门机构的简化台架模型，尚未按真实门锁标定，来源和差异见 [NOTICE.md](NOTICE.md)。

## 启动

在外层工作区 `~/projects/LeWM_Door_Handle`：

```bash
./LeWM_Door_Handle_Manipulation/run_door.sh
```

启动脚本位于内层项目目录，自动使用同目录中的 `.venv/bin/python`，并为本次命令清空 ROS 的 `PYTHONPATH`。
无需在 `.venv/` 或 `.venv/bin/` 中放置业务脚本。

无窗口检查与相机预览：

```bash
./LeWM_Door_Handle_Manipulation/run_door.sh --headless --steps 200
MUJOCO_GL=egl ./LeWM_Door_Handle_Manipulation/run_door.sh --headless --camera --steps 20
```

若单独使用内层项目，在 `LeWM_Door_Handle_Manipulation` 目录运行：

```bash
PYTHONPATH= .venv/bin/python door_demo.py
PYTHONPATH= .venv/bin/python -m door_task.check
```

相机离屏渲染需要可用的 OpenGL 后端；本机已验证 `MUJOCO_GL=egl`。不启用 `--camera` 的无窗口运行不需要图形显示服务。
打开交互窗口需要在有桌面显示权限的终端运行；若报 `X11: Failed to open display`，可先用 `--headless` 检查物理环境。
依赖沿用项目已有环境，版本固定文件位于 `mujoco_gripper_demo/requirements.txt`。

## 键盘操作

交互窗口使用项目自己的 GLFW 输入处理和 MuJoCo 渲染接口。原生 `simulate` UI 未加载，字母键和空格只执行下表中的任务动作，不会同时切换渲染、暂停或触发原生重置。
先点击窗口获得焦点。按住移动键连续移动，松开停止发送位姿增量；短按移动一次。窗口失去焦点时清除移动按键状态。
每控制步发送 0.005 m 平移或 0.05 rad 旋转的目标增量，实际运动取决于控制器跟踪和接触约束。
坐标轴为**世界坐标系**，不是屏幕方向。

| 按键 | 操作 |
| --- | --- |
| W / S | 沿世界 X 正 / 负方向移动 |
| A / D | 沿世界 Y 正 / 负方向移动 |
| Q / E | 沿世界 Z 正 / 负方向移动 |
| I / K | 绕世界 X 正 / 负方向旋转 |
| J / L | 绕世界 Y 正 / 负方向旋转 |
| U / O | 绕世界 Z 正 / 负方向旋转 |
| 空格 | 切换夹爪张开 / 闭合，长按不会反复切换 |
| R | 重置整个回合，包括物理状态、控制器和事件历史 |
| Esc / 关闭窗口 | 保存已有记录并退出 |
| 鼠标左键拖动 | 旋转视角 |
| 鼠标右键拖动 | 平移视角 |
| 鼠标滚轮 / 中键拖动 | 缩放视角 |

默认每个回合结束后保持窗口，按 R 开始下一回合。`--auto-close` 可在回合结束时自动关闭。
没有移动输入时保持末端目标；夹爪保持当前开合指令，初始为张开。环境不会自行执行抓握开门。
窗口直接显示键位和状态提示。重置任务使用 R；Ctrl / Alt / Super 组合快捷键不会发送机械臂动作。

在内层项目目录可运行无窗口键盘回归检查：

```bash
PYTHONPATH= .venv/bin/python -m unittest door_task.test_viewer
```

## 策略接口

在内层项目目录导入：

```python
import numpy as np
from door_task import make_env

env = make_env(camera_obs=True, seed=0)
try:
    obs = env.reset()
    action = np.zeros(7)
    action[-1] = -1.0
    next_obs, reward, done, info = env.step(action)
    image = next_obs["agentview_image"]  # uint8，256 × 256 × 3
    state = info["task_state"]
    events = info["physics_events"]
finally:
    env.close()
```

七维动作依次为 `[dx, dy, dz, drx, dry, drz, gripper]`，范围均为 `[-1, 1]`。
`OSC_POSE` 将平移缩放至每步最大 0.05 m、轴角旋转分量缩放至 0.5 rad；旋转分量不是欧拉角。
夹爪指令 −1 表示张开，+1 表示闭合。控制器显式使用世界坐标系。

观测包含机器人关节位置、速度、末端位置和四元数、夹爪位置，以及 `handle_pos`、`handle_qpos`、`hinge_qpos`。
`task_state` 的字段顺序由 `environment.TASK_STATE_FIELDS` 给出，包含门和把手的角度 / 角速度、双指接触数、锁舌接触数和法向力、阶段标签及成功标记。
`info["task_state"]` 提供同一组数据的命名字典及仿真时间。角度单位为 rad，角速度为 rad/s，法向力为 N。

默认图像遵循 robosuite 的 OpenGL 行方向；需要通常的上到下图像时使用 `image[::-1].copy()`，或统一设置 robosuite 的图像约定。
预览 PNG 已自动做方向转换。物体真值和接触状态是调试 / 评估信息，视觉 LeWM 实验应显式选择图像与机器人自身状态作为模型输入，避免把评估标签混入输入。

设置 `make_env(randomize_placement=True, seed=...)` 或使用 `--randomize --seed 3`，可在每次重置时对门的位置、朝向做小范围随机化。
相同种子对应相同随机序列；默认关闭随机化，便于先调试操作策略。

## 接触与阶段标签

`info["physics_events"]` 在每个物理步检测左指、右指与把手，以及锁舌与门框的接触开始 / 结束，附带仿真时间戳。
这里的接触指 MuJoCo 中进入约束求解的几何接触，不是基于距离或把手角度猜测的抓握成功。
接触力只在控制步结束时作为状态输出；当前没有保存每个物理步的力曲线。

`info["events"]` 是控制步上的阶段和接触变化。主要标签：

| 标签 | 定义与限制 |
| --- | --- |
| `handle_turned` | 把手角度绝对值 ≥1.25 rad，是可调的阶段阈值，不能单独证明解锁 |
| `door_released` | 门角度 >0.05 rad，超过本模型在检查力矩下的锁舌阻挡区间 |
| `door_motion_started` | 门角度 >0.04 rad 且角速度绝对值 >0.02 rad/s 时的上升沿 |
| `success` | 达到开门角度并满足连续控制步数量 |

闭门但未施加拉力时，锁舌可能没有接触门框。因此“没有接触”或“接触结束”都不能独立作为解锁真值。
研究接触状态突变时，应结合物理步事件、把手角度与门运动，而不是把这些阶段阈值当成真实门锁传感器。
左、右指同时接触只是接触证据，不等同于已经稳定抓住把手。

## 输出与验证

默认结果保存到 `outputs/door_task/`，重复运行覆盖同名文件，可用 `--output-dir` 指定独立实验目录。

| 文件 | 内容 |
| --- | --- |
| `state_trace.csv` | 每个控制步的动作、执行后状态、奖励和阶段事件，包含回合编号 |
| `contact_events.jsonl` | 物理步级接触事件，包含回合编号、控制步编号和事件时间 |
| `summary.json` | 版本、频率、种子、运行状态、成功标记及最终状态 |
| `preview.png` | 使用 `--camera` 时保存的初始 RGB 图像 |
| `validation.json` | 运行 `python -m door_task.check` 后生成的检查报告 |

状态 CSV 和预览图用于环境检查，尚不是包含完整 `(obs_t, action_t, obs_t+1)` 图像序列的训练数据集。
下一步应在此接口上实现有反馈的脚本策略，再保存对齐的图像、动作、下一帧与事件标签。

检查程序验证：完整场景稳定步进、固定初始状态可重现、末端沿世界 X 方向移动、夹爪闭合、非法动作拒绝、成功保持时间及重置、物理步接触事件和法向力。
它还直接加载同一份门 MJCF，对隔离的门机构施加诊断力矩，比较未转把手 / 已转把手时的开门角度，并验证回位弹簧。
诊断直接设置把手初始角度、施加外力；成功条件测试直接准备开门状态。这些检查不代表机械臂已经学会或通过脚本完成开门，不可作为操作示范数据。

实现参考：[robosuite 自定义环境](https://robosuite.ai/docs/tutorials/add_environment.html)、[MuJoCo 接触模型](https://mujoco.readthedocs.io/en/3.3.0/computation/index.html#collision-detection)。
