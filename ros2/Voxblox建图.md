# Voxblox TSDF 建图

更新：2026 年 10 月 31 日。

## 实现

新增 [voxblox_core](src/voxblox_core/) 与 [gbplanner_voxblox](src/gbplanner_voxblox/) 两个 ament 包。前者以 NTNU Voxblox 固定版本的 C++ 核心构建，后者接收 ROS 2 点云与采样时刻 TF 或变换话题，可选原版 ICP 校正，再调用 TSDF 积分器，把测得的距离场按原 GBPlanner 规则分类，送入已有规划器。全程无需 ROS 1 节点或 ros1_bridge。

原 GBPlanner 的 packages_https.rosinstall 指向 ntnu-arl/voxblox 的 dev/noetic 分支。此次解析并固定为 0ee88c26c203a3fe88e348dbbb552ab431a35352；minkindr 头文件固定为 564f12639a8447d4d3e5e7707851424302941056。这些依赖是本次解析的分支版本，不声称与原 GBPlanner 提交同期。归档 SHA-256、逐文件哈希与许可证保存在 [provenance.json](src/voxblox_core/provenance.json) 及 vendor 目录；101 个依赖文件保持原文。

原版核心的 catkin 构建被独立 ament 包替代，系统 Protobuf 生成 Block/Layer 消息。构建出现原始 protobuf_utils.cc 使用旧 ByteSize() API 的弃用警告，实际链接和测试通过；保留原文件，未为消除警告改写上游算法。vendor 源码保持原文；本轮发现 RayCaster 的六处 `abs(direction)<0` 永不成立，使轴向射线只遍历起点，构建副本已将其改为 `==0`，六方向遍历及真实自由空间积分通过。原库的 ESDF、网格和文件 I/O 已接入 ROS 2，使用结果见 [地图保存与距离场](地图保存与距离场.md)。

## 数据流与规则

points → 非零时间戳与 XYZ 格式检查 → 采样时刻的传感器位姿 → 可选 ICP 校正 → 原版 FastTsdfIntegrator → TSDF 层 → 距离/权重分类 → voxel_map → 规划器接受地图 → 路径碰撞检查与执行。

- 体素缺失或权重小于 1e-6：未知。无效数值也按未知处理。
- distance ≤ occupancy_factor × voxel_size + 1e-6：占据；负距离仍为占据。
- 其余已观测体素：空闲。未观测体素不会被填为空闲。
- 原版世界栅格以零点对齐；适配器要求输出原点与分辨率整格对齐，否则启动失败。
- 重置同时重新建立 TSDF 层和积分器，清除 Fast 积分器内部的跨帧缓存，发布空地图并让规划器清空对应全局图。
- 完整保留入射点的传感器坐标，由原版积分器执行位姿变换。超距回波按原版规则清空到最大射程；不会把超距端点写成障碍。

voxel_map 是已有固定范围占据查询接口。tsdf_voxels 另行发布 x、y、z、distance、weight，便于核对实际距离值与融合权重。mapper_status 给出 backend、扫描计数、已知格和分配块数。原始 TSDF 层采用稀疏块，可超出输出窗口；已接入 `max_block_distance_from_body`，以每次成功扫描的原始传感器位置裁掉远处块；默认不限制距离。

## 当前参数

| 项 | 小房间验证 | 原 RMF 配置 |
| --- | ---: | ---: |
| 体素边长 / m | 0.25 | 0.20 |
| 截断距离 / m | 0.75 | 0.60 |
| 每块每边体素数 | 16 | 16 |
| 最小/最大射程 / m | 0.1 / 6 | 0.5 / 50 |
| 方法、积分顺序 | fast、mixed（可设 sorted） | fast、sorted |
| 最大权重 | 200 | 200 |
| 清空射线权重 | 0.01 | 0.01 |
| 占据距离系数 | 1 | 1 |

小房间参数与原配置分别记录。历史实验使用硬编码 sorted；当前已恢复原积分器的 mixed 默认，显式 `tsdf.integration_order_mode=sorted` 可重现旧选择。当前测试使用单线程积分来固定积分顺序，便于复位后复现；不据此评价原版多线程性能。机器人尺寸为 0.4 m 立方体，输出窗口为 10×10×3 m，共 19,200 格。它是测试场景，不是原 RMF 实机参数文件。

位姿默认使用 `use_tf_transforms=true`。设为 `false` 后接收 `transform` 话题，以 `timestamp_tolerance_sec=0.001` 严格容差查找首个匹配，否则在两帧之间按原版 log/exp 插值，不外推。`sensor_config` 可提供 `T_B_D`、`T_B_C` 两个按行排列的 16 项矩阵及各自 `invert_*`；合成位姿为 `T_G_D * inverse(T_B_D) * T_B_C`。此模式要求普通和已启用的自由空间输入共用一个传感器坐标系及 `T_B_C`，变换消息的坐标系字符串沿用原版忽略行为。

`enable_icp` 默认 `false`，`accumulate_icp_corrections` 默认 `true`。参数文件可设置原 `icp_min_match_ratio`、`icp_subsample_keep_ratio`、`icp_mini_batch_size`、`icp_refine_roll_pitch`、`icp_inital_translation_weighting`、`icp_inital_rotation_weighting`，保留 `inital` 拼写；新增 `icp_num_threads` 默认硬件并发数，取不到时用 1。积分使用校正位姿，清空球使用原位姿，零成功批次仍正常积分。校正话题和 `map→icp_corrected→pose_corrected` 两段 TF 均使用点云采集时间；两子坐标系名可配置，reset/load 沿用原版保留累计校正。

## 验证

10 项 C++ 测试覆盖原始分类阈值、墙面正负距离、障碍后未知空间、旋转和平移、重复观测权重、超距射线、复位、非法参数与非有限点，以及 ESDF 规划清空球的内外层语义。测试中平面前后得到真实 TSDF 距离，不是只检查接口是否返回成功。

ROS 2 点云联调使用 6,480 个合成激光回波，首帧得到 10,135 个已知格、24 个分配块；规划路线约 3.5265 m。逐体素核对 tsdf_voxels 的距离与权重，并按原阈值重新计算分类；路径另做机器人扫掠体和场景墙面碰撞检查。缺失 TF、零时间戳、错误坐标系、截断数据和字段重叠均被拒绝。重置后地图清空，重新积分相同点云得到相同路线。

开发容器的首轮持续探索完成 21 条轨迹，行程 26.7570 m，已知格 10,135→19,113，314 帧点云无拒绝并返回起点。记录位于 [voxblox_execution/summary.json](results/voxblox_execution/summary.json)。该运行使用按墙钟插值的测试消费者，无飞行动力学；零返航误差只表示测试坐标回到起点。

2026 年 10 月 26 日 TSDF 阶段全量回归：只读源码、禁网新容器中八包从零构建（约 1 分 29 秒）；58 项 C++ 与 13 项 Python，共 71 项实际单元用例通过，colcon 77 含 6 项包装。原有接口、续航、剩余路径和两套地图闭环均通过。TSDF 本轮执行 15 条轨迹、行进 20.5800 m，236 帧扫描无拒绝，已知格 10,135→19,200/19,200 并返航。

新容器证据：[完整回归日志](results/clean_image/validation.log)、[点云建图结果](results/clean_image/results/voxblox_mapping_integration.json)、[逐体素距离/权重](results/clean_image/results/voxblox_mapping_integration.csv)、[持续探索结果](results/clean_image/results/voxblox_execution/summary.json)。

![首帧距离、权重与规划分类](results/clean_image/results/voxblox_mapping_integration.png)

![新容器中的探索轨迹与执行速率](results/clean_image/results/voxblox_execution/execution.png)

全新容器回归结果与开发容器的数字分开记录。已知格包含空闲格和占据格，不是可达空间覆盖率。不同后端对表面的分类与膨胀不同，异步扫描时序也不同，不能仅凭路程或已知格数判断探索效率优劣。

10 月 31 日新增 Mesh 原生流程 26 项、变换话题流程 13 项通过，包含部分地图导入后的定时网格重建。ICP 核心 2 项和原生 11 项通过：289 点解析平面的误差由 0.12 m 降为 0.000214 m，校正实际用于积分。RayCaster 的 4 项定向测试及修正后的 10 项 TSDF 后端检查通过；轴向自由空间射线由原先 1 格变为 18 格。结果见 `results/voxblox_icp.json`、`results/voxblox_ray_caster_core.xml`。这些是合成几何与真实 ROS 2 节点验证，没有进行 RViz GUI 或真实机器人配准实验。

## 运行

在已构建的 ROS 2 工作空间中：

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch gbplanner_voxblox mapping.launch.py

完整控制链路：

    ros2 launch gbplanner_voxblox mapping.launch.py global_planning:=true background:=true control:=true
    ros2 launch gbplanner_execution execution.launch.py

完整集成测试会自行启动所需节点：

    ROS_DOMAIN_ID=65 ros2 run gbplanner_local mapping_demo.py --backend voxblox --start-node --output results/voxblox_mapping_integration.json
    ROS_DOMAIN_ID=66 ros2 run gbplanner_execution execution_system_demo.py --backend voxblox --start-node --output results/voxblox_execution

原 gbplanner_local 启动文件仍使用三态射线建图适配器，Voxblox 使用以上明确的入口。ESDF、地图保存/加载、持久增量网格和 PLY 导出已接入；多传感器输入见[多传感器](多传感器.md)。变换话题模式使用 `use_tf_transforms:=false`，ICP 使用 `enable_icp:=true`，两者均需外部位姿输入。


## 原生地图交换与距离可视化

`voxblox_msgs` 保留原 Block/Layer 与 Mesh 定义，支持地图层交换、`mesh` 增量块及迟到订阅者的完整网格。表面 RGB、完整 TSDF/ESDF 距离云和占据立方体默认不自动发布：`publish_pointclouds_on_update=true` 随规划快照发布，否则可用 `publish_pointclouds=true` 在脏网格定时更新后发布；服务 `publish_pointclouds` 强制发布，reset 强制清除旧产品。切片／可通行云仍由各自开关随快照发布，具体参数见[地图保存与距离场](地图保存与距离场.md)。输入最小间隔按采集时间严格比较，普通与自由空间各自计时；缺位姿的云保留队列，20 ms 定时重试，变换话题到达也触发重试。


## 积分器配置补齐

新增参数可通过节点参数或 `sensor_config` 文件配置；去掉 `tsdf.` 即为对应原版名字，显式的新名字优先。

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `tsdf.use_const_weight` | 未指定时为 `!use_weight_dropoff` | 截断带内恒定权重；保留已有关闭 dropoff 的兼容行为 |
| `tsdf.use_symmetric_weight_dropoff` | false | 零交叉两侧对称衰减，优先于 const 和普通 dropoff |
| `tsdf.weight_ray_by_range` | false | 按传感器 z 深度平方倒数加权，不是欧氏射程平方倒数 |
| `tsdf.allow_clear` | true | 超距普通回波转为清空射线；显式自由空间点云不受限制 |
| `tsdf.integration_order_mode` | mixed | mixed／sorted；此前本地写死 sorted |
| `tsdf.start_voxel_subsampling_factor` | 2.0 | Fast 的端点去重格细分倍数 |
| `tsdf.max_consecutive_ray_collisions` | 0 | Fast 允许的连续重复体素数 |
| `tsdf.clear_checks_every_n_frames` | 1 | Fast 近似去重集合清理周期 |
| `tsdf.max_integration_time_s` | float 最大值 | Fast 每帧时限；0 跳过实际融合 |
| `tsdf.anti_grazing` | false | Merged 避免其他射线擦过本帧有效表面端点 |

三个截断带权重开关按 symmetric→const→dropoff 取首个开启项，全部显式关闭会在启动时拒绝。深度加权开启时，传感器 z=0 等无法产生有效权重的点连同颜色一起跳过。分辨率、截断距离、最大权重、射程、carving、method 和清空系数也已支持原参数别名；已有本地数值默认保留。Fast 要求清理间隔至少1、碰撞阈值非负、细分倍数正有限、时间有限且非负。原 Fast 类内静态 reset_counter 保留，多个 Fast 实例在同进程中仍共享该计数。

7 项新增核心测试、受默认顺序影响的现有 10 项后端测试通过。原生三节点 14 项检查通过：同一体素距离为 0.375 m 时，恒定权重为 1，对称衰减再乘深度权重为 0.144444；关闭 allow_clear 后拒绝超距普通回波但仍接受显式自由空间；Fast 零时限三次均无分配块。见 [TSDF 参数验证](results/voxblox_tsdf_configuration.json)。

## RMF 的 ESDF 更新周期（2026-11-07）

原 `EsdfServer` 默认用 ROS 时间每秒增量更新一次 ESDF，RMF 的原配置没有覆盖该默认值。此前 ROS 2 后端在每次点云积分后立即更新 ESDF，把独立的派生距离场计算放进了每帧建图关键路径。RMF 的规划占据来自 TSDF，ESDF 主要提供距离场和可通行区域产品。

现在 RMF 配置显式设置 `update_esdf_every_n_sec: 1.0`，TSDF 仍逐帧积分并发布；ESDF 在同一执行器的 ROS 时钟定时回调中消费累计的 `kEsdf` 更新标记。没有新增并发写图，也没有关闭 ESDF。通用后端 `esdf_update_on_scan` 默认为 true；节点周期参数 0 保留本 ROS 2 适配器原来的同步模式，**不是**上游参数 0 的关闭定时器语义。周期模式目前只用于 TSDF 规划且不启用机器人清空球的配置。

ESDF 点云、切片和可通行区域使用其实际覆盖的最后输入时间。定时回调不发布新的规划观测或推进观测编号；状态分别报告已接收输入序号、已完成 ESDF 序号和待更新状态。保存或显式请求完整图层时先增量补完 ESDF；加载、导入和重置会刷新产品并清除旧采集时间，文件和 Layer 消息没有采集时间时以 0 表示未知。已知区域清空导致同步重建时，也单独刷新 ESDF 产品。

同批修复了原积分器在多 worker 下对 block 更新 bitset 的无锁写，以及首次射线读取未初始化 block 缓存索引的问题。补丁只作用于带原 SHA256 校验的构建树副本，vendor 文件未改。RMF 仍为 `fast / sorted / 0.2 m / 1 thread`；Fast 的多线程去重顺序差异和同进程多实例静态计数问题不在本批范围。

真实固定路线结果见 `results/rmf_mapping_throughput/fixed2_esdf_periodic.json`。一次原 30 点路线完整执行、原生接地和停桨通过，终点误差 0.005893 m，进程正常退出。落地查询时已积分 446 帧，TF 等待队列丢弃 1 帧、pending 0；跟踪日志末尾又完成了第 447 帧，二者测点不同。44 次 ESDF 更新耗时共 6.939 秒；447 次 TSDF 积分共 5.528 秒，点过滤共 0.041 秒。它们是本次实测阶段成本，不是与旧运行逐帧对应的加速比。

观察器收到最后一条规划地图通知时，仿真时间减该地图采集时间为 0.696 秒；最后里程计采集时间减这张地图采集时间为 0.710 秒。此前同一源路线 `collinear_execution/fixed3` 的第一个指标为 10.544 秒，落地积分 260 帧、队列丢弃 118 帧。两个物理场景独立初始化，本次还增加 ESDF 产品观察订阅，不是相同传感器流的严格 A/B。原路线、碰撞判定和轨迹配置未变；本次恢复的是派生距离场的调度方式。

此前 `fixed1` 把整图发布周期设为 0.5 秒仍落后约 10.44 秒，已经试过并撤回，不能再作为“未试候选”。本批保持整图逐帧发布。当前 195 次规划端完整地图解析平均 0.1558 秒、接收处理平均 0.2092 秒；该次建图积压显著改善，但一次运行不足以证明持续稳定。0.4 秒是基础提前规划量，并非地图年龄的硬阈值。下一步应处理规划端整图解析/接收成本；本批不把单次固定路线通过写成完整迁移、自主任务稳定率或全场景零丢帧。

保存、加载与ROS时钟协议另见 `results/rmf_mapping_throughput/periodic_protocol1.json`：一次18项通过，使用小点云和受控时钟，没有启动物理仿真。节点保存服务负责flush；直接调用延迟模式后端的独立程序，保存一致的TSDF/ESDF对之前也应显式调用 `updateEsdf()`。


## 稀疏快照存储与完整发布频率（2026-11-07）

规划端每次接收完整快照，旧实现为每个体素单独分配哈希节点，替换最后一个旧图引用时再逐个释放。`SparseVoxelMap` 现在以连续Entry数组和32位slot索引存储体素及4³自由位图。精确键比较、负坐标、完整几何与TSDF插值保持；删除采用末元素补位和探测链后移，不积累墓碑。快照深拷贝与不可变多读者保留，不能复用仍被验证线程持有的旧图内存。

同输入核对使用真实RMF revision131的448001格地图，每版固定三轮记录导入和最后引用释放。896002个输入与规范中心位置的查询及原路径/所有段结果hash全等，原失败仍是第6段path_not_free。导入中位121.633→91.049ms；每轮导入＋释放中位145.080→91.051ms。详见 `results/rmf_mapping_throughput/parse_storage/comparison.json`。

真实fixed3仅采用连续表后，旧图替换平均51.387→0.236ms，完整callback209.227→170.629ms，但mapper同次也变慢，末次地图通知age10.154s。fixed4进一步显式`map.snapshot_publish_period: 0.2`，保留全部点云积分和1秒ESDF周期，最多5Hz构造完整快照；原30点物理路线/降落通过，终点5.920mm。落地430积分、TF初始帧丢弃1、等待队列0；完整trace432次积分。末次通知age2.006s，末odom−map2.140s，后段仍增长，不能写成积压已解决。

`planner_trigger_lead_time: 0.4`只定义基础提前规划时间，实际自适应lead另计规划与交接耗时，并非map_age<0.4的准入条件。当前应分别检查延迟趋势、任务结束后等待新观测和真实提前交接。物理各次独立初始化，前后结果只作描述，不能把所有耗时变化归因于单项存储优化。
