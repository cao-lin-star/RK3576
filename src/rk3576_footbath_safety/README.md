# RK3576 足浴桶速度安全链

本包提供软件层的自动速度限制、手动/自动命令仲裁。它不能替代
STM32F407的500 ms通信硬停、ToF台阶联锁、超声波停车或物理急停。

## 速度话题链

```text
Nav2 /cmd_vel ──> auto_cmd_vel_limiter ──> /cmd_vel_auto_limited ─┐
                                                                  ├─> command_mux
键盘/Foxglove汇总后的 /cmd_vel_manual（PS2由F407直控） ─────────────────────────┘
command_mux ──> /cmd_vel_selected ──> STM32 serial bridge
```

要求STM32桥只订阅 `/cmd_vel_selected`。任何发布者直接连接桥输入都会绕过本安全链。
同一时间只允许一个 ROS 手动控制源占用 `/cmd_vel_manual`；command mux 发现发布者数大于 1 时立即输出零速并上报 ERROR。Foxglove 门控服务可以常驻，它只在租约有效且收到新鲜命令时动态创建输出发布者；切换到键盘或远程安全驾驶工具前停止 Foxglove 持续发布并等待最多 0.30 s 释放即可，无需停止服务。

### 自动运动租约

`exploration_supervisor`只在RUNNING状态按监督周期向
`/safety/auto_motion_lease`发布 `true`，其他状态和关闭时发布 `false`。
自动限速器要求租约值为true且消息年龄不超过0.50秒，否则持续发布零速。这个短期
租约独立于 `/explore/resume`：后者只暂停或恢复前沿探索，不能直接授权底盘运动。

自动通道有两次0.20 m/s限制：限速器第一次限制，command_mux再次限制。手动通道
最大线速度1.00 m/s。两个输入超过各自0.30秒没有新命令后失效；手动新鲜时优先，
其次自动，否则 `/cmd_vel_selected` 以20 Hz持续发布零速。NaN和Inf不会被转发。

启动自动建图前必须停止所有持续发布 `/cmd_vel_manual` 或 `/cmd_vel_foxglove` 的键盘/Foxglove 控制源，否则手动优先仲裁会压住自动通道；Foxglove 门控服务本身可以常驻。

## 启动

```bash
source /home/sky/rk3576_footbath_ws/install/setup.bash
ros2 launch rk3576_footbath_safety safety.launch.py \
  start_auto_limiter:=true \
  auto_limiter_require_lease:=true \
  start_command_mux:=true
```

launch中的限速器和仲裁器默认关闭，必须由最终bringup显式开启，并同时把STM32桥的
输入改为 `/cmd_vel_selected`。在这两项完成以前不能声称自动限速链已接通。

## 近距离障碍

2026-09-23起不再运行任何玻璃分类节点，也不再发布疑似玻璃虚拟Range。
左右12cm停车在F407执行；自动任务由exploration中的hazard_recovery使用停车后的新回波确认位置，
确认后在已有hazard层记忆小范围障碍。本包只负责速度限幅、仲裁和恢复通道。
普通导航和返航也由监督器持有租约，不应通过关闭租约绕开该监督器。

## 验收检查

```bash
ros2 topic info -v /cmd_vel
ros2 topic info -v /cmd_vel_auto_limited
ros2 topic info -v /cmd_vel_manual
ros2 topic info -v /cmd_vel_selected
ros2 topic echo /diagnostics
```

预期：Nav2发布 `/cmd_vel`；限速器是其订阅者并唯一发布auto_limited；mux订阅manual
和auto_limited并唯一发布selected；STM32桥只订阅selected。当前实现经过包级测试，
尚未完成实车带载验收。

## 普通导航的租约模式

自动建图必须使用 `auto_limiter_require_lease:=true`。普通已有地图导航为了不依赖探索
监督器，可以显式使用 `auto_limiter_require_lease:=false`；此时0.20 m/s限幅和0.30秒
命令watchdog仍然生效，只跳过租约检查。