# LEWM 机械臂开门研究

基于 LeWorldModel（LEWM）开展机械臂操作门把手与开门任务的研究。

项目首先在 MuJoCo 中搭建包含门板、杠杆式把手与末端执行器的仿真环境，通过脚本化策略采集操作轨迹，训练 LeWM 的编码器与预测器，在潜在空间中学习动作条件下的状态转移规律，并利用 CEM 规划实现闭环控制。关键研究问题包括接触状态突变（把手解锁、门开始移动）的预测能力，以及 SIGReg 正则化对潜在状态分离度的影响。

目前已实现 [门把手操作环境](door_task/README.md)：Panda 机械臂、双指夹爪、门板、杠杆式把手与物理锁舌，支持键盘操作、RGB 观测和接触事件记录。脚本开门策略、数据集采集与模型训练将在此环境上继续接入。

在本目录运行：

```bash
PYTHONPATH= .venv/bin/python door_demo.py                  # 交互窗口
PYTHONPATH= .venv/bin/python door_demo.py --headless --steps 200
PYTHONPATH= .venv/bin/python -m door_task.check            # 物理与接口检查
```

若当前位于外层工作区 `~/projects/LeWM_Door_Handle`，运行 `./LeWM_Door_Handle_Manipulation/run_door.sh`，脚本会选择正确的解释器路径。在本目录可直接运行 `./run_door.sh`。

官方夹爪抓取演示及运行说明见 [mujoco_gripper_demo](mujoco_gripper_demo/README.md)。
