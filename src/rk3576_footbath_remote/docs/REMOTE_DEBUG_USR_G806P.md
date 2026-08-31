# 鲁班猫 3 + USR-G806p 远程调试部署

## 1. 设计边界

远程入口只使用标准 OpenSSH。ROS 2、RPLIDAR C1、SLAM 与 Nav2 全部继续在 RK3576 本机运行；电脑通过 SSH 在 RK 上执行 `ros2` 命令，不让 DDS 跨越 Wi-Fi、4G 或公网。

默认策略：

| 项目 | 值 | 原因 |
|---|---:|---|
| G806p LAN 地址 | `192.168.8.1/24` | 避开常见的 `192.168.1.0/24` 上级网络冲突 |
| RK3576 LAN 固定租约 | `192.168.8.10` | 由 G806p 按 RK 网口 MAC 固定分配 |
| SSH | TCP 22，仅 LAN/Tailscale | 不做 WAN 端口转发，不启用 DMZ |
| 远程虚拟网络 | Tailscale | RK 主动出站建链，适合 4G/运营商 NAT |
| `ROS_DOMAIN_ID` | `0` | 与现有系统服务一致 |
| DDS 跨网暴露 | 禁止 | `FOOTBATH_EXPOSE_ROS_DDS=0` |
| 远程运动控制 | 禁止 | `FOOTBATH_ALLOW_REMOTE_MOTION=0` |
| 调试负载 | `nice 15`、短超时、无 rosbag | 避免与 SLAM/Nav2 抢占 CPU 和磁盘 I/O |

`footbath_remote_status` 和 `footbath_collect_debug` 均不发布话题、不改参数、不重启服务、不写地图，也不发送 `/cmd_vel`。

## 2. 物理连接与端口分配

| 鲁班猫 3 接口 | 连接 | 说明 |
|---|---|---|
| 网口 1 | G806p 的 LAN 口 | RK 唯一默认网关；不要接 WAN 口 |
| 网口 2 | 留空 | 可日后配置为救援直连口，但不得设置默认网关/DNS |
| 40Pin 物理3/5/6脚 | F407 USART1 TX/RX/GND | UART3_M0；`/dev/ttyS3` → `/dev/footbath_stm32` |
| USB 2.0 #1 | fireDAP | 只接SWD烧录/调试，TX/RX不接 |
| USB 2.0 #2 | 独立USB-TTL（可选） | 只接F407 UART4 PC10/PC11，ASCII调参 |
| USB 3.0 #1 | 高位 RPLIDAR C1 | 使用 `/dev/footbath_lidar_high`；物理接口贴“高位”标签 |
| USB 3.0 #2 | 低位 RPLIDAR C1 | 使用 `/dev/footbath_lidar_low`；物理接口贴“低位”标签 |

两台C1是USB2设备，插USB3口可以正常工作。上述分配使四个USB口分别用于两台雷达、fireDAP和可选UART4调试，不再需要为F407正式通信占一个USB口。不要把两台雷达接到无源Hub。G806p 的有线 LAN 为百兆，足够 SSH、日志和文件上传；点云处理、SLAM 与 Nav2 数据流保持在 RK 本机，不依赖路由器吞吐量。

## 3. 首次台架部署

### 3.1 配置 G806p

首次操作先保持电脑连接 G806p Wi-Fi，暂时不要接电机动力电源。

1. 打开 `http://192.168.1.1`，使用机身/说明书默认信息首次登录。
2. 立即修改 Web 管理密码、Wi-Fi 名称和 Wi-Fi 密码。
3. 将 LAN IPv4 改为 `192.168.8.1`、掩码 `255.255.255.0`，保存并重连 Wi-Fi。
4. 重新打开 `http://192.168.8.1`，开启 LAN DHCP，建议地址池 `192.168.8.100`～`192.168.8.200`。
5. 鲁班猫接入 G806p LAN 后，在 DHCP 租约页面找到其有线网口 MAC，添加静态租约 `192.168.8.10`。
6. 不配置端口转发，不配置 DMZ，不把 22、7400～7600 等端口暴露到 WAN/4G。

### 3.2 确认 RK 网络

在 RK 本地终端执行：

```bash
ip -br link
ip -br address
ip route
```

识别两个真实网口名后，只让接 G806p 的网口通过 DHCP 获得默认路由。第二网口先保持断开。重启一次，确认地址仍为 `192.168.8.10`，并执行：

```bash
ping -c 3 192.168.8.1
ping -c 3 1.1.1.1
getent hosts tailscale.com
```

前两项分别验证路由器和互联网，最后一项验证 DNS。

### 3.3 从 Windows 复制并构建包

PowerShell：

```powershell
Set-Location G:\work\ROS2\RK3576_overlay\src
scp -r .\rk3576_footbath_remote sky@192.168.8.10:/home/sky/rk3576_footbath_ws/src/
ssh sky@192.168.8.10
```

RK 终端：

```bash
cd /home/sky/rk3576_footbath_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rk3576_footbath_remote
source install/setup.bash
```

该包不包含 ROS 节点，也不修改现有 mapping/navigation launch。

### 3.4 配置 SSH 公钥

Windows PowerShell 生成专用密钥：

```powershell
ssh-keygen -t ed25519 -f $env:USERPROFILE\.ssh\id_ed25519_footbath -C footbath-rk
scp $env:USERPROFILE\.ssh\id_ed25519_footbath.pub sky@192.168.8.10:/tmp/footbath.pub
```

先在 RK 上仅安装公钥和工具，不立即关闭密码登录：

```bash
cd /home/sky/rk3576_footbath_ws
sudo bash src/rk3576_footbath_remote/scripts/install_remote_debug.sh \
  --user sky --public-key /tmp/footbath.pub
```

保持当前会话不关，在 Windows 第二个窗口测试：

```powershell
ssh -i $env:USERPROFILE\.ssh\id_ed25519_footbath sky@192.168.8.10
```

确认成功后，回到 RK 执行一次加固：

```bash
sudo bash src/rk3576_footbath_remote/scripts/install_remote_debug.sh \
  --user sky --public-key /tmp/footbath.pub --harden
```

把 `deploy/windows_ssh_config.example` 中两段内容合并进 Windows 的 `$env:USERPROFILE\.ssh\config`，以后局域网可直接执行 `ssh footbath-rk-lan`。

## 4. 跨互联网部署

### 4.1 RK 安装 Tailscale

按 Tailscale 官方 Linux 安装页在 RK 安装稳定版，然后：

```bash
sudo tailscale up --hostname=lubancat3-footbath
tailscale status
tailscale ip -4
```

Windows 安装 Tailscale 并登录同一账号。验证：

```powershell
tailscale ping lubancat3-footbath
ssh footbath-rk-remote
```

这里使用的是“Tailscale 网络上的普通 OpenSSH”，不需要启用 Tailscale SSH，也不需要 G806p 公网 IP、DDNS 或端口映射。

### 4.2 路由器移到小车上

1. G806p 使用独立、稳定的合规电源；鲁班猫使用开发板要求的稳定电源。电机 24 V 原始电源不可直接给开发板供电。
2. G806p 插入并启用可用的 SIM/上网链路，确认 4G 状态与 DNS 正常。
3. 鲁班猫网口 1 接 G806p LAN，保持静态租约 `192.168.8.10`；网口 2 留空。
4. 先启动 G806p，网络稳定后再启动 RK。确认 `tailscale status` 与 `systemctl is-active footbath-mobile.service`。
5. 电脑可使用任何互联网连接，登录同一 tailnet 后执行 `ssh footbath-rk-remote`。

## 5. 日常远程调试

```bash
# 快速只读状态
footbath_remote_status

# 增加每个 ROS 话题数秒的频率采样
footbath_remote_status --deep

# 生成低优先级、无凭据、无地图、无 rosbag 的诊断包
footbath_collect_debug

# 查看导航服务日志，不改变服务状态
journalctl -u footbath-mobile.service -f
```

把诊断包拉回 Windows：

```powershell
scp footbath-rk-remote:/home/sky/footbath_debug/footbath-debug_*.tar.gz .
```

不要把远程 SSH 链路作为安全相关的实时遥控通道。底盘最终硬停仍由 STM32 的 500 ms 超时保护执行；需要远程运动测试时，现场必须有人、车轮离地或有急停，并单独明确授权测试命令。

### 5.1 Foxglove 手动建图（受控入口）

首次安装必须同时显式启用可视化和运动工具：

```bash
sudo bash install_remote_debug.sh --user sky --public-key /path/key.pub \
  --enable-visualization --enable-motion-tools
```

每次使用都需要现场确认物理急停，然后只发放短租约：

```bash
sudo footbath_motion_arm arm --minutes 15 \
  --i-confirm-onsite-emergency-stop
systemctl is-active footbath-foxglove.service
```

Windows 建立 SSH 隧道；现场用 `footbath-rk-lan`，异地用
`footbath-rk-remote`：

```powershell
ssh -N -L 8765:127.0.0.1:8765 footbath-rk-lan
```

Foxglove 连接 `ws://localhost:8765`。唯一允许客户端发布的话题是
`/cmd_vel_foxglove`，消息类型为 `geometry_msgs/msg/Twist`；门控节点会在租约有效、
消息有限且 0.30 秒内持续更新时，将其限制后转发到 `/cmd_vel_manual`。建议以
10～20 Hz 持续发布，首轮只用 0.05～0.10 m/s。网页不能发布服务、修改参数或直接
写 `/cmd_vel_selected`。

同一时间只允许一个 ROS 手动控制源。Foxglove bridge 和门控服务可以常驻；切换到键盘或 `footbath_safe_drive` 前，只需停止 Foxglove 持续发布（或撤销租约）并等待最多 0.30 s，让门控释放 `/cmd_vel_manual`。使用 Foxglove 前必须退出其他 `/cmd_vel_manual` 发布者。门控或 command mux 检测到第二个发布者时会立即零速、停止转发并报警。

启动自动建图前必须停止所有持续发布 `/cmd_vel_manual` 或 `/cmd_vel_foxglove` 的键盘/Foxglove 控制源，否则手动优先仲裁会压住自动通道；Foxglove bridge 和门控服务本身可以常驻。

结束时先发布零速/停止发布，再立即撤销租约：

```bash
sudo footbath_motion_arm disarm
```

即使客户端异常断开，Foxglove 门控、command mux 和 F407 仍分别在 0.30 s、
0.30 s 和 0.50 s 量级归零；这些软件保护不能替代现场急停。
## 6. 验收表

| 验收项 | 命令/动作 | 通过标准 |
|---|---|---|
| 固定 LAN 地址 | 重启 G806p 和 RK 两次 | RK 每次均为 `192.168.8.10` |
| 本地 SSH | `ssh footbath-rk-lan` | 公钥登录成功 |
| 密码加固 | `sshd -T` | `passwordauthentication no` |
| 远程 SSH | 手机热点/异地网络执行 `ssh footbath-rk-remote` | 无端口映射也能登录 |
| ROS 服务未受影响 | `systemctl is-active footbath-mobile.service` | `active` |
| 调试工具只读 | 运行 status/bundle 前后比较节点与服务 | 无节点重启、无 `/cmd_vel` 发布 |
| 建图回归 | 本地完成一次 10～15 分钟建图 | 无新增丢帧、导航服务无异常重启 |
| 断网回归 | 建图时断开 G806p 外网 2 分钟 | SLAM/Nav2 继续，本地底盘保护正常 |

## 7. 故障恢复

- Tailscale 不通但在现场：连接 G806p Wi-Fi，执行 `ssh footbath-rk-lan`。
- SSH 配置失误：用 RK 的 HDMI/串口本地登录，移走 `/etc/ssh/sshd_config.d/60-footbath-remote.conf` 的最新文件或恢复脚本生成的 `.bak.*`，运行 `sudo sshd -t` 后重启 SSH。
- 4G 下显示 relay：对终端调试通常可接受；不要因此打开 DMZ。用 `tailscale ping` 和 `tailscale netcheck` 确认链路类型。
- 第二网口启用后网络漂移：删除该网口的默认网关和 DNS，只保留救援静态地址。
