# 智能足浴桶 RK3576 本地建图与导航

面向野火鲁班猫 3（RK3576）、Ubuntu 22.04 与 ROS 2 Humble。雷达驱动、SLAM、Nav2、速度安全链和 STM32 串口桥均在 RK3576 本机运行；Windows 电脑承担 SSH、Foxglove、键盘遥控和文件管理。

Linux 运行工作空间为 `/home/sky/rk3576_footbath_ws`。本目录是纳入 Git 管理的部署源镜像，部署时同步到 Linux ext4 工作空间后再构建。

独立 Git 仓库：`https://github.com/cao-lin-star/RK3576.git`。Windows/WSL 只保存和检查源码，正式 `build/install/log` 必须在 RK3576 的 ARM64 Linux ext4 工作空间生成。

- 跨板和全外设接线：[`docs/RK3576与STM32F407接线表.md`](docs/RK3576与STM32F407接线表.md)
- RK 编译、部署和 eMMC 镜像烧录：[`docs/RK3576编译与系统镜像烧录流程.md`](docs/RK3576编译与系统镜像烧录流程.md)
- 实际运行几何/安装参数：`src/rk3576_footbath_bringup/config/hardware.yaml`
- F407 固件仓库：`https://github.com/cao-lin-star/F407_controll.git`

## 第一阶段功能边界

```text
高位 RPLIDAR C1 /scan_high
  ├─ SLAM Toolbox / Cartographer 建图（唯一激光输入）
  ├─ AMCL 定位（唯一激光输入）
  └─ Nav2 全局、局部障碍层

低位 RPLIDAR C1 /scan_low_raw
  └─ 角域滤波，仅保留物理正前方 135°（原始 LaserScan 跨 ±180°）
       └─ /scan_low_front
            └─ Nav2 全局、局部障碍层（不进入 SLAM 或 AMCL）

前超声波 /range/ultrasonic
  ├─ F407 近距硬停（代码已启用，待实车验收）
  ├─ Nav2 局部 RangeSensorLayer
  └─ 与两台雷达比较，发布疑似玻璃诊断
```

低位雷达其余 225° 会扫描到车壳。`low_lidar_angular_filter` 保持 LaserScan 角度索引和时间戳不变：先把有效扇区外的 range/intensity 替换为 NaN，再按雷达位姿剔除落在 `0.225 m` 车体自滤波半径内的近距点。第一阶段不合并两份 LaserScan。

## 已确认几何

全部坐标相对 `base_footprint`，遵守 REP-103：X 向前、Y 向左、Z 向上。

| 项目 | 坐标/尺寸 |
|---|---|
| 圆形车体最大外廓 | 半径 0.22 m，高 0.50 m |
| `base_link` | `(0, 0, 0.25)` |
| 高位 C1 | `(0.10, 0, 0.50)`，外观箭头朝前；驱动坐标补偿 yaw=π |
| 低位 C1 | `(0.10, 0, 0.15)`，外观箭头朝前；驱动坐标补偿 yaw=π |
| MPU6050 | `(0, 0, 0.195)`，X 前/Y 左；X/Y 待安装复核 |
| 左 ToF | `(0.155563, +0.155563, 0.15)`，左前 45° |
| 右 ToF | `(0.155563, -0.155563, 0.15)`，右前 45° |
| 正前超声波 | `(0.135, 0, 0.24)`，水平朝前 |

ToF 下倾角仍需按最终支架实测；当前 TF 的 pitch=0 只是占位。F407 源码已设置 `CLIFF_SAFETY_ENABLE=1`，但左右地面基线、真实台阶、断线和倒退恢复仍未完成整车验收，因此 RK 阶段合同元数据继续保持 `cliff_hard_stop_enabled=false`，不能把“固件已启用”理解为“系统已验收”。

## 包结构

- `rk3576_footbath_description`：整车和传感器 xacro、静态 TF。
- `rk3576_footbath_base`：UART v1 协议、重连、500 ms 超时 STOP、里程计、IMU 和 Range 消息。
- `rk3576_footbath_lidar`：双 RPLIDAR C1 启动及低位 135° 角域滤波。
- `rk3576_footbath_slam`：默认 SLAM Toolbox，并保留 Cartographer 2D。
- `rk3576_footbath_navigation`：Nav2、AMCL、双雷达障碍源和前超声 RangeSensorLayer。
- `rk3576_footbath_safety`：自动命令 0.20 m/s 硬限速、短租约、手动优先仲裁和疑似玻璃诊断。
- `m_explore_ros2/explore`、`m_explore_ros2/explore_lite_msgs`：供应商化的前沿探索节点和状态接口，固定上游提交及归档校验见 `src/m_explore_ros2/UPSTREAM_VERSION.md`。`map_merge` 未纳入。
- `rk3576_footbath_exploration`：explore_lite 健康门控、暂停/停止、30 分钟超时和地图保存监督。
- `rk3576_footbath_bringup`：硬件、手动建图、自动建图和已有地图导航的一键启动。
- `rk3576_footbath_remote`：SSH、Tailscale、Foxglove、远程诊断和受控运动测试。
- `rk3576_footbath_mobile`：G806P 现场 Wi-Fi 手机建图/导航页面和单会话 PIN；手机心跳只约束手动遥控，自动建图/导航由 RK 本地监督器维持。

## 安装、构建与测试

```bash
cd /home/sky/rk3576_footbath_ws
sudo ./scripts/install_dependencies.sh
vcs import src --skip-existing < dependencies.repos
./scripts/build.sh
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 scripts/validate_phase1_contract.py
```

`m-explore-ros2` 已按固定提交直接纳入 `src/m_explore_ros2`，不再由 `vcs import` 重复下载；`dependencies.repos` 只固定外部 `sllidar_ros2`。两者都不得改成浮动分支。

## 两台同型号 C1 的稳定命名

不能依赖 `/dev/ttyUSB0` 和 `/dev/ttyUSB1`。分别只插一台雷达并记录：

```bash
udevadm info --query=property --name=/dev/ttyUSB0 | \
  grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH'
```

本车两只CP2102N已确认具有不同序列号，正式规则位于 `deploy/udev/99-footbath-lidars.rules`。若以后更换适配器且序列号相同或为空，才改用 `ID_PATH` 并按模板 `deploy/udev/99-footbath-serial.rules.example` 重新生成规则。安装并验证：

```bash
sudo install -m 0644 deploy/udev/99-footbath-lidars.rules /etc/udev/rules.d/99-footbath-lidars.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/footbath_lidar_high /dev/footbath_lidar_low /dev/footbath_stm32
```

`sky` 必须属于 `dialout` 组；修改用户组后要重新登录。

## F407正式UART链路

fireDAP 只保留 SWD 烧录/调试，不再承载串口。正式链路改为：

| F407 | LubanCat-3IO |
|---|---|
| PA9 / USART1_TX | 物理3脚 / UART3_RX_M0 |
| PA10 / USART1_RX | 物理5脚 / UART3_TX_M0 |
| GND | 物理6脚 / GND |

不要互接两板3.3 V或5 V。先启用 `rk3576-lubancat-uart3-m0-overlay.dtbo`，并禁用冲突的 `i2c7-m1`、`uart3-m1`/RS485-1。重启后应出现 `/dev/ttyS3`。专用规则 `deploy/udev/99-footbath-uart3-m0.rules` 将它稳定命名为 `/dev/footbath_stm32`：

```bash
test -e /boot/dtb/overlay/rk3576-lubancat-uart3-m0-overlay.dtbo
grep -E 'uart3|i2c7|rs485' /boot/uEnv/uEnv.txt
ls -l /dev/ttyS3
sudo ./scripts/install_systemd.sh
readlink -f /dev/footbath_stm32
```

如果板端曾安装过 fireDAP/USB-TTL 的旧 `footbath_stm32` 规则，必须先找出并移除旧规则，不能让两个设备争用同一符号链接。USART1是AA55二进制协议口，不要用minicom或echo向它发送ASCII文本。

## 分项启动和检查

只启动车体描述和两台雷达，不连接 STM32：

```bash
ros2 launch rk3576_footbath_bringup hardware.launch.py start_base:=false
```

只启动高位或低位：

```bash
ros2 launch rk3576_footbath_bringup hardware.launch.py \
  start_base:=false start_lidar_low:=false
ros2 launch rk3576_footbath_bringup hardware.launch.py \
  start_base:=false start_lidar_high:=false
```

检查话题和 TF：

```bash
ros2 topic hz /scan_high
ros2 topic hz /scan_low_raw
ros2 topic hz /scan_low_front
ros2 run tf2_ros tf2_echo base_footprint laser_high_frame
ros2 run tf2_ros tf2_echo base_footprint laser_low_frame
```

期望 TF 平移分别为 `(0.10, 0, 0.50)` 和 `(0.10, 0, 0.15)`，两者 yaw 均为 `π`；`/scan_low_front` 只有原始扫描角 `[112.5°, 180°] ∪ [-180°, -112.5°]` 可出现有限距离，其余应为 NaN。

## 手动建图

```bash
ros2 launch rk3576_footbath_bringup mapping.launch.py
```

SLAM Toolbox 只订阅 `/scan_high`。低位雷达不写入静态地图；它只在 Nav2 运行时补盲。三种手动方式保留：

1. PS2：由 F407 直接控制，R1 为死人开关、SELECT 强停；ROS 只接收里程计。
2. 键盘：推荐先现场布置急停，再使用 `footbath_safe_drive` 的短时命令；通用键盘节点必须 remap 到 `/cmd_vel_manual`。
3. Foxglove：bridge 只开放 `/cmd_vel_foxglove`；门控节点在 root 短租约有效时转发到 `/cmd_vel_manual`，断流 0.30 s 归零。门控服务可以常驻，非转发状态不占用 `/cmd_vel_manual`；运动服务和参数默认仍禁用。

F407 UART4/RK 通用底盘绝对上限仍为 1.00 m/s；PS2 与手机手动线速度上限为 0.30 m/s；仓库自带的远程安全驾驶工具仍额外限制为 0.20 m/s。

保存手动地图：

```bash
./scripts/save_map.sh \
  /home/sky/rk3576_footbath_ws/src/rk3576_footbath_navigation/maps/footbath_site
```

## 自动建图

启动自动建图前必须停止所有持续发布 `/cmd_vel_manual` 或 `/cmd_vel_foxglove` 的键盘/Foxglove 控制源，否则手动优先仲裁会压住自动通道；Foxglove 门控服务本身可以常驻。

```bash
ros2 launch rk3576_footbath_bringup auto_mapping.launch.py \
  headless:=true start_explorer:=true
```

自动链路固定为：

```text
Nav2 cmd_vel_nav
  → velocity_smoother（≤0.20 m/s）
  → /cmd_vel
  → auto_cmd_vel_limiter（≤0.20 m/s、租约 0.50 s）
  → /cmd_vel_auto_limited
  → command_mux（自动再次限制 ≤0.20 m/s）
  → /cmd_vel_selected
  → STM32 串口桥
```

探索监督器只有在 `/scan_high`、`/map`、`/odom` 都持续新鲜时才发放运动租约。故障、停止、前沿耗尽、超过 30 分钟或监督器退出都会撤销租约并输出零速。

```bash
ros2 topic echo /diagnostics
ros2 topic echo /explore/status
ros2 topic echo /safety/auto_motion_lease
ros2 topic echo /cmd_vel_auto_limited
ros2 topic echo /cmd_vel_selected
ros2 service call /exploration/stop std_srvs/srv/Trigger '{}'
ros2 service call /exploration/start std_srvs/srv/Trigger '{}'
ros2 service call /exploration/save_map std_srvs/srv/Trigger '{}'
```

默认地图前缀是 `/home/sky/rk3576_footbath_ws/maps/footbath_auto_时间戳`，完成或超时时保存栅格地图和 SLAM posegraph；`return_to_init=false`。

## 已有地图导航

```bash
ros2 launch rk3576_footbath_bringup navigation.launch.py \
  map:=/home/sky/rk3576_footbath_ws/src/rk3576_footbath_navigation/maps/footbath_site.yaml
```

已有地图导航同样经过 0.20 m/s 限速器，但不要求 explore supervisor 租约。Nav2 的 local/global obstacle layer 同时使用 `/scan_high` 和 `/scan_low_front`，前超声只进入 local costmap。

## 疑似玻璃诊断

- `/safety/suspected_glass`：超声检测到近障而两台雷达正前方均无对应距离回波时置位。
- `/safety/suspected_glass_valid`：三路输入均新鲜时才为 true。
- 该结果只作提示，不直接控制电机，也不能保证识别所有玻璃。

## 上车前最低验收

1. 架空驱动轮，准备可断 24 V 主电源的物理急停。
2. 确认三个稳定设备名、双雷达频率、TF、`/odom` 和 `/range/ultrasonic`。
3. 确认串口桥唯一订阅 `/cmd_vel_selected`，自动命令全过程绝对值不超过 0.20 m/s。
4. 杀死探索监督器，确认 0.50 s 内 `/cmd_vel_selected` 和 F407 目标速度归零。
5. 实测轮径、轮距、死区、PID、IMU、ToF 平地基线和有载刹停距离后，才允许无人值守探索。

当前正前超声 F407 硬停代码已启用（0.20 m 停、0.28 m 清），但仍需实车验证盲区和误触发；ToF 台阶联锁保持禁用，等待下倾角和地面基线标定。
