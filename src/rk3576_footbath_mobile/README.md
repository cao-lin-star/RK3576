# RK3576 Footbath Mobile Gateway

Local G806P Wi-Fi phone UI; deliberately not a generic ROS or shell bridge.

## Safety contract

- HTTP clients accepted only from 192.168.8.x or loopback.
- PBKDF2 PIN, one 15-minute session, 1 s browser heartbeat; ordinary mapping/idle tolerates 2.5 s jitter and automatic modes tolerate 3.0 s.
- Manual output is released after 0.30 s without refresh; auto-mapping/navigation remain supervised locally on the RK and are not cancelled by a brief phone disconnect.
- Manual mapping uses /cmd_vel_manual only while held, <=0.30 m/s, and refuses another manual publisher.
- Manual and automatic mapping default to /scan_mapping_fused; the phone UI can explicitly fall back to high-only /scan_high before launch. Navigation localization remains high-lidar-only.
- Goals require navigation mode, initial pose, fresh scan/map/odom and 0.35 m known-free clearance.
- Automatic motion remains on existing <=0.20 m/s limiter, command mux and F407 chain.

## Build and install

```bash
cd /home/sky/rk3576_footbath_ws
colcon build --symlink-install --packages-select rk3576_footbath_mobile
source install/setup.bash
sudo bash install/rk3576_footbath_mobile/share/rk3576_footbath_mobile/scripts/install_mobile_gateway.sh sky
```

Re-running the installer preserves the existing PIN. Pass `--reset-pin` as the second argument only when an intentional PIN reset is required.

Join G806P Wi-Fi and open `http://192.168.8.10:8080`.
## Map interaction contract
- The map preserves aspect ratio and supports zoom in/out, 15-degree left/right rotation, view reset, wheel zoom, and drag-to-pan when pose picking is inactive.

- Initial pose and navigation goal are selected by pressing the map, dragging along the desired heading, and releasing; a drag shorter than 8 cm uses the numeric yaw fallback.
- An initial-pose request stays pending until a newer `/amcl_pose` message confirms localization, or fails after 8 s.
- Navigation reports goal acceptance, distance remaining, ETA, recovery count, and the real action result (`succeeded`, `canceled`, or `aborted`).
- Blue, green, and red arrows show the requested initial pose, requested goal pose, and current robot pose respectively.
