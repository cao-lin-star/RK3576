# RK3576 足浴桶自动探索监督包

本包只负责第一阶段自动建图的**任务监督**。前沿选择由固定版本的
`explore_lite` 完成，SLAM、Nav2、双雷达启动及速度仲裁仍由各自专用包负责。

## 第一阶段数据分工

- 高位 RPLIDAR C1：发布 `/scan_high`，作为 slam_toolbox 唯一建图扫描源。
- 低位 RPLIDAR C1：过滤掉扫描车壳的 225° 后进入 Nav2
  全局/局部动态障碍层；不进入第一阶段 SLAM 扫描匹配。
- `explore_lite`：从 `/map` 和 `/map_updates` 提取前沿，通过 Nav2
  `NavigateToPose` 逐个探索。
- `exploration_supervisor`：高位 `/scan_high`、`/map`、`/odom` 都持续新鲜后才
  发布恢复命令；运行中任一输入过期即暂停、请求取消导航并在自动命令话题发零速。

低位雷达不会直接改变SLAM保存的静态地图，但它进入Nav2代价地图后会阻止规划器
穿过沙发底座等低障碍。由于前沿仍来自高位雷达的静态占据栅格，某些被低位障碍
完全挡住的前沿可能被反复尝试，首版必须有人监护。

## 固定依赖

根工作区的 `dependencies.repos` 将 `m-explore-ros2` 固定为：

```text
326cf8a0b487c34246bb8f3326afbcd69576dc60
```

该提交包含Humble支持、暂停/恢复、完成状态以及预占目标误拉黑修复。上游没有
Ubuntu二进制包，因此部署时必须导入源码，不能使用浮动的 `main`：

```bash
cd /home/sky/rk3576_footbath_ws
vcs import src < dependencies.repos
rosdep install --from-paths src --ignore-src -r -y --skip-keys ydlidar
colcon build --symlink-install --packages-up-to \
  rk3576_footbath_exploration
```

如果不使用多机器人地图合并，不需要启动 `map_merge` 包。

## 启动组合

在最终bringup中，顺序应为：

1. STM32、TF和两台雷达；
2. 高位雷达驱动及低位135°过滤器；
3. slam_toolbox；
4. Nav2的 `navigation_launch.py`（不启动AMCL和静态map_server）；
5. 本包 `exploration.launch.py`。

单独启动本包：

```bash
source /home/sky/rk3576_footbath_ws/install/setup.bash
ros2 launch rk3576_footbath_exploration exploration.launch.py
```

本launch不会替你启动硬件、SLAM或Nav2。缺少 `/scan_high`、`/map`、`/odom` 时，
监督器保持暂停并持续输出自动通道零速；60秒仍不健康会锁存启动超时，之后必须
人工调用 `/exploration/start`。

`explore_lite`自身在构造时会先启动，本launch先启动监督器并延迟1秒启动探索器，
让暂停话题有时间完成DDS发现。这不是速度安全边界：最终命令链仍必须设置独立的
自动限速器，将Nav2线速度硬限制为 `0.20 m/s`，再进入手动优先的命令仲裁器。

监督器仅在 `RUNNING` 状态每0.20秒向 `/safety/auto_motion_lease` 发布一次 `true`；
其他状态以及节点关闭时发布 `false`。自动建图限速器应使用 `require_lease:=true`，
并在0.50秒收不到新租约时闭锁为零速。`/explore/resume`只控制前沿探索器，不能
作为底盘运动授权。

## 操作接口

```bash
# 健康输入恢复后，人工清除故障并继续
ros2 service call /exploration/start std_srvs/srv/Trigger '{}'

# 暂停探索、取消NavigateToPose并在自动通道发零速
ros2 service call /exploration/stop std_srvs/srv/Trigger '{}'

# 手动保存当前栅格地图和slam_toolbox姿态图
ros2 service call /exploration/save_map std_srvs/srv/Trigger '{}'

# 检查状态
ros2 topic echo /diagnostics
ros2 topic echo /explore/status
```

默认保存前缀为：

```text
/home/sky/rk3576_footbath_ws/maps/footbath_auto_%Y%m%d_%H%M%S
```

探索完成或达到30分钟上限时，监督器先暂停和取消导航，再调用
`/slam_toolbox/save_map`；栅格成功后调用 `/slam_toolbox/serialize_map`。
服务是异步的，`/exploration/save_map` 返回“accepted”只表示请求已提交，最终结果
以日志和诊断中的 `save_state` 为准。`return_to_init` 默认关闭。

## 手动接管要求

本包不拥有PS2、键盘、Foxglove的命令仲裁。最终系统必须保证：

- Nav2命令：专用自动话题，三层限制不超过 `0.20 m/s`；
- 手动命令：独立话题，可按项目要求限制为 `1.00 m/s`；
- 手动通道优先且有超时；
- 人工接管时调用 `/exploration/stop`，恢复必须显式调用 `/exploration/start`；
- F407的500 ms通信硬停和ToF/超声安全层继续独立工作。

## 测试边界

当前代码可以进行构建、lint、纯逻辑单元测试和ROS接口测试，但尚未在实车完成：

- Nav2目标取消时序；
- 两台相同C1的稳定udev别名；
- 自动限速与手动接管仲裁；
- 地图/姿态图写盘；
- 真实家庭环境前沿耗尽判定。

首次实测必须轮子悬空，然后在有人监护、低速、无玻璃风险的封闭小区域进行。
