# Pittsburgh 矿洞的 Fortress 场景入口

`pittsburgh_fortress.launch.py` 复用现有 SMB 机器人、桥接、控制器、里程计、建图、规划和 Action 执行链。默认启用原传感器头的双激光雷达，加载现有 `upstream_planning.yaml`；自主规划默认关闭，`planning:=true` 才创建规划与跟踪节点。原 `edgar_fortress.launch.py` 的世界、出生点、紧凑传感器及规划默认值保持不变。

场景使用保留的 `pittsburgh_mine.dae`，模型显式放在 `(-22, 92.5, 0)`，旋转为零，未增加地板。原 `model.sdf` 和 Classic world 的模型定义均使用这个位置，但 Classic world 的 `<state>` 另存了 `(-22.4884, 286.186, 0)`；这里采用模型定义的位置，不复现该保存状态。

已验证的 ignition-common 4.8.1 原生网格加载结果保留了 DAE 的三角形坐标及绕序，因此没有按文件中的 `Y_UP` 再旋转模型。加上显式平移后，网格边界为 `[-22, -234.744, 0]` 至 `[230.329, 92.5, 2.75088] m`。独立配置的规划范围为 `[-25, -240, -2]` 至 `[235, 95, 6] m`，包围该网格；范围内的空间仍须由传感器实际观测。默认增益计算使用 VLP16 的 `20 m` 范围，双雷达建图使用 `0.5–50 m` 积分范围，与已迁移的 SMB 原传感器配置一致。ROS 2 的机器人碰撞包络、地面投影和 `0.1 m` TSDF 适配继续沿用。

出生点 `(0, 0, 0.5)`、偏航零来自保留的 ROS 1 `gbplanner/launch/smb/smb_sim.launch`。原生运行 `results/pittsburgh_port/scene2.json` 已确认矿洞实际世界位姿、机器人落位和四轮与矿洞的接触：稳定后 base 高度约 `0.41193 m`，支撑面约 `0.17193 m`。稳定窗口内四轮各记录500条接触；机身接触传感器已配置，未记录到机身接触事件。两部雷达各积分19帧，收到183902个已知体素。该静止检查没有发送运动命令或规划请求。

完成安装后，可启动场景：

```bash
ros2 launch gbplanner_gazebo pittsburgh_fortress.launch.py planning:=false contacts:=true
```

`spawn_x`、`spawn_y`、`spawn_z`、`spawn_yaw` 可显式调整，现有 `mapping`、`sensor_profile`、`renderer` 等参数由共用启动文件接收。

`autonomy1` 已完成监督轮驱建图：前行3.216 m、倒车2.712 m、停在 x≈0.504 m，双雷达共积分698帧。但当前位姿检查返回 `terrain_support_missing`，自主请求数为零。保存地图显示当前位置上方自由空间已被观测，地面零交叉所需的邻体素缺失；真实轮胎接触不能代替规划地图的地面支持。

这个停靠点位于当前行程的地面观测盲区：VLP16 离地0.9337 m、最低扫描角−15°，最近地面水平距离约3.485 m；OS1 朝前下倾斜，初始位置在 y=0 的近侧地面边界约 x=0.991 m。原定 x=0.5 m 并不适合直接沿用为本场景的任务起点。后续从既有往返走廊内选择已实际观测到支持的位置，仍须通过同一机体、地面和路径检查后才能设置任务 home。连续视场计算只解释几何可见性，不保证离散回波、遮挡或 TSDF 支持；`autonomy1` 的失败保持原样。

当前任务起点采用监督建图返回 x=1.8 m，实际停在约1.804 m。`autonomy2` 通过地图检查，但搜索跟踪暴露出原追踪曲率公式的直线转向偏置。SMB 现在显式使用 `signed_geometric` 转向模式，保留原线速度、门限和终端要求；`autonomy3` 已完成双层搜索 Action，终点位置／偏航误差为0.039034 m／1.41e-6 rad。

完整探索与返航仍未通过。`autonomy3` 的局部路线在终端转向期间发生横滑，实测位置接回路线的校验线段触及6个未观测体素后停车。停车后的地面投影使机体检查范围降低约16.16 mm，接触同一未知层，重规划在采样前返回 `ground_root_unknown`。具体保存地图、真实动作和校验路径的区别见 `results/pittsburgh_port/autonomy3_diagnosis/analysis.md`。物理网格中没有相应障碍，也不能代替真实观测把这些体素填成 Free。
