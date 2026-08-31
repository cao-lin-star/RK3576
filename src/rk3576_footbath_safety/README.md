# RK3576 足浴桶速度安全链与玻璃诊断

本包提供软件层的自动速度限制、手动/自动命令仲裁和疑似玻璃诊断。它不能替代
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
  start_glass_monitor:=true \
  start_auto_limiter:=true \
  auto_limiter_require_lease:=true \
  start_command_mux:=true
```

launch中的限速器和仲裁器默认关闭，必须由最终bringup显式开启，并同时把STM32桥的
输入改为 `/cmd_vel_selected`。在这两项完成以前不能声称自动限速链已接通。

## 疑似玻璃诊断

监视器比较正前方超声波距离与高、低雷达前方扇区最近点。只有三路数据均在0.50秒
内更新时，`/safety/suspected_glass_valid`才为true。当超声波在有效距离内、并且
两个雷达最近距离都不满足：

```text
abs(lidar_distance - ultrasonic_distance) <= 0.15 m
```

连续3次后，`/safety/suspected_glass`置true。结果及valid均为可靠、transient-local
Bool，方便晚加入的界面获得最新状态。

该算法**仅用于诊断**，不能作为玻璃防撞或自由空间判定：超声波波束宽，雷达只取
扇区最近点，没有目标方位和时间同步；玻璃斜入射仍可能同时漏检。valid=false表示
数据未知，绝不能将此时的suspected=false解释为“确认没有玻璃”。

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