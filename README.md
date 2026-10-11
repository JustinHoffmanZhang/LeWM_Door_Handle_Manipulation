# LEWM 机械臂开门研究

基于 LeWorldModel（LEWM）开展机械臂操作门把手与开门任务的研究。

## 代码来源

- 官方仓库：https://github.com/lucas-maes/le-wm
- 上游代码位于 `lewm/`，以 Git 子模块管理；主仓库记录使用的上游提交。
- 上游安装、训练与评估说明见 [lewm/README.md](lewm/README.md)。

## 获取完整项目

```powershell
git clone --recurse-submodules <本项目的远程仓库地址>
```

如果已经克隆主仓库：

```powershell
git submodule update --init --recursive
```

## 开发约定

项目级研究代码、配置与笔记放在主仓库中。若需要修改 LEWM，在 `lewm/` 中创建开发分支并提交改动，再在主仓库中提交更新后的子模块指针。

数据、模型权重和实验输出不纳入 Git。当前仅完成仓库初始化和上游代码克隆；运行环境、数据与开门任务适配尚未配置。
