# rk3576_footbath_remote

该包为鲁班猫 3/RK3576 提供独立的 SSH 远程诊断入口，不启动 ROS 节点，也不修改 SLAM、Nav2 或底盘通信。

- `footbath_remote_status`：有时间上限的只读系统/ROS 状态。
- `footbath_collect_debug`：低 I/O 诊断归档，不含 rosbag、地图和凭据。
- `install_remote_debug.sh`：安装 OpenSSH、公钥及命令；只有显式 `--harden` 才关闭密码登录。
- `docs/REMOTE_DEBUG_USR_G806P.md`：USR-G806p、RK3576 与 Windows 的完整部署和验收步骤。

构建：

```bash
cd /home/sky/rk3576_footbath_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rk3576_footbath_remote
```
