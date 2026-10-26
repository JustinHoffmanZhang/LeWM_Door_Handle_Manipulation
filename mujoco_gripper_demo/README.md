# robosuite 官方夹爪抓取演示

自动完成“张开 → 下降 → 闭合抓取 → 抬起方块”，默认演示两轮。此目录包含运行入口、安装脚本、依赖和官方原版源码，可独立于其他实验目录使用。

## 安装与运行

### Linux

当前 Linux 环境位于项目根目录 `LeWM_Door_Handle_Manipulation/.venv/`，使用 Python 3.10、MuJoCo 3.3.0 和 robosuite 1.5.2。在项目根目录运行：

```bash
source .venv/bin/activate
PYTHONPATH= python mujoco_gripper_demo/demo.py --headless  # 无窗口验证抓取
PYTHONPATH= python mujoco_gripper_demo/demo.py             # 打开图形窗口，按 R 重播
```

当前终端加载了 ROS Jazzy 的 Python 3.12 路径；`PYTHONPATH=` 仅在本次命令中清空外部包路径，避免混入 Python 3.10 环境。

需要重新安装演示依赖时，在已激活的环境中执行 `PYTHONPATH= python -m pip install -r mujoco_gripper_demo/requirements.txt`。

### Windows

在项目根目录的 PowerShell 中执行一次安装（需要 `uv`）：

```powershell
powershell -ExecutionPolicy Bypass -File .\mujoco_gripper_demo\setup.ps1
```

使用 robosuite 的 `MjviewerRenderer` 打开图形窗口并运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\mujoco_gripper_demo\run.ps1
```

脚本优先复用项目 `.python/` 中的 Python 3.10，在本目录创建独立的 `.venv/`，依赖按 `requirements.lock` 安装。也可用 `setup.ps1 -Python 'Python 3.10 路径'` 指定解释器；已有依赖缓存时，可用 `setup.ps1 -Offline` 离线安装。无需 CUDA。

演示结束后先保存结果，再保留窗口供截图。**点击窗口使其获得焦点后，按 `R` 可以从初始状态重新演示，无需关闭窗口或重启程序**；演示过程中也可按 `R` 立即重新开始。每次重播会重置物理状态、时间、动作阶段和采样记录；完成后更新同名输出文件。关闭窗口退出。

窗口由 `robosuite_viewer.py` 中的 `RobosuiteViewer` 管理，它继承 robosuite 官方 `MjviewerRenderer`。场景仍由 `build_scene()` 创建，共享同一个 `MjSim`，由官方 renderer 的 `update()` 更新画面并由 `close()` 清理窗口。robosuite 1.5.2 没有将键盘回调传给底层 viewer，因此本地扩展补充窗口初始化时的回调注册，让 `R` 重播继续可用。

这个后端底层仍使用 MuJoCo passive viewer，外观与此前接近。按 robosuite 的默认设置隐藏左右面板；按 `Tab` 可显示左侧面板，按 `Shift+Tab` 可显示右侧面板。

若希望演示后自动关闭，用 `run.ps1 -AutoClose`。无窗口运行用 `run.ps1 -Headless`；可用 `-Cycles 3` 修改轮数。每轮包含四阶段，每阶段 500 个物理步；步长 0.002 秒时两轮共 8 秒仿真时间。

窗口中的 **Reload** 在 passive viewer 模式下不可用；仿真由本脚本管理，重播使用 `R`。原生 **Reset** 按钮只重置物理数据，不会重置 Python 动作序列和采样计数，因此也不能代替完整重播。相关机制见 [MuJoCo passive viewer 文档](https://mujoco.readthedocs.io/en/3.3.0/python.html#passive-viewer)。

也可以直接运行 Python 文件：

```powershell
.\mujoco_gripper_demo\.venv\Scripts\python.exe .\mujoco_gripper_demo\demo.py --hold
```

在 VS Code 中选择解释器 `mujoco_gripper_demo/.venv/Scripts/python.exe` 后，可直接调试 `demo.py`。直接运行 Python 时也默认保留窗口并支持 `R` 重播；`--hold` 可省略，若要结束后自动关闭可传 `--auto-close`。

## 文件

| 文件 | 用途 |
| --- | --- |
| `demo.py` | 使用 robosuite viewer 的抓取演示，默认打开窗口 |
| `robosuite_viewer.py` | 扩展官方 MjviewerRenderer，接入独立场景并支持 R 键重播 |
| `robosuite_compat.py` / `.pyi` | 导出原版 BoxObject，并为 Pylance 补全可选 joints 参数和 XML 返回类型 |
| `setup.ps1` | 创建环境并安装依赖 |
| `run.ps1` | 运行入口，默认演示后保留窗口 |
| `requirements.txt` / `requirements.lock` | 直接依赖 / 完整依赖版本 |
| `upstream/demo_gripper_interaction.py` | 从官方 robosuite 1.5.2 安装包原样复制的示例 |
| `upstream/LICENSE` | 上游许可 |

生成结果统一位于本目录的 `outputs/`：

| 输出 | 内容 |
| --- | --- |
| `trajectory.csv` | 各物理步的时间、阶段、方块位置、离桌面高度和接触数量 |
| `box_lift_height.png` | 方块最低点离桌面高度曲线 |
| `summary.json` | 依赖版本、完成轮数、成功判定与各轮抬升高度 |

成功条件：每轮抬升阶段最后 0.2 秒，方块最低点始终离桌面超过 2 厘米，并保持与夹爪接触。最低点计算考虑方块姿态。所有轮次完成并通过检查才输出 `PASS`、返回退出码 0；提前关闭窗口会保存已有记录并返回失败状态。重复运行会更新同名结果。

## 来源与汇报

- [robosuite 官方教学文档](https://robosuite.ai/docs/demos.html#gripper-interaction-procedural-generation)
- [官方 v1.5.2 源码](https://github.com/ARISE-Initiative/robosuite/blob/v1.5.2/robosuite/demos/demo_gripper_interaction.py)

保留官方场景、执行器参数、重力补偿和四阶段动作序列，本地接入 robosuite 的 `MjviewerRenderer` 并增加重播、有限轮次、记录和成功检查。该例使用**独立 Rethink 双指夹爪与竖直滑动关节**，展示预设序列控制下的接触抓取。

可按实际运行结果汇报：

> 复现 robosuite 官方夹爪交互示例，完成程序化场景构建、夹爪执行器控制和“下降—闭合抓取—抬升”流程。记录物体轨迹、接触数量和抬升高度曲线，并按持续悬空与接触条件验证抓取成功。

实际成功轮数和抬升高度以 `outputs/summary.json` 为准。
