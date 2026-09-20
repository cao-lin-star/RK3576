#!/usr/bin/env bash
# Existing LubanCat/Humble installation only. Never flashes firmware or commands motion.
set -Eeuo pipefail
upload=${1:?upload directory}; commit=${2:?commit}; archive_hash=${3:?hash}; helper_hash=${4:?helper hash}
[[ $EUID == 0 && $commit =~ ^[0-9a-f]{40}$ ]]
[[ $upload =~ ^/home/sky/footbath-upload-[0-9]{8}-[0-9]{6}-[0-9a-f]{8}$ ]]
[[ $(uname -m) == aarch64 && -f /opt/ros/humble/setup.bash ]]
exec 9>/run/lock/footbath-deploy.lock
flock -n 9 || { echo 'Another deployment is active'; exit 1; }
printf '%s  %s\n' "$archive_hash" "$upload/source.tar.gz" "$helper_hash" "$upload/deploy.sh" | sha256sum -c -
ws=/home/sky/rk3576_footbath_ws
[[ -d $ws && ! -L $ws && $(realpath "$ws") == "$ws" ]]
# Do not move filesystems or nested mounts.
if findmnt -rn -o TARGET | grep -Eq '^/home/sky/rk3576_footbath_ws(/|$)'; then
  echo 'Workspace contains mount points; stop and inspect'; exit 1
fi
for unit in footbath-mobile.service footbath-foxglove.service rk3576-footbath.service; do
  [[ $(systemctl show "$unit" -p LoadState --value) == loaded ]]
done
[[ -r /etc/footbath/mobile.env && -f /usr/local/bin/footbath_visualization_bridge ]]
configured_ws=$(sed -n 's/^FOOTBATH_WS=//p' /etc/footbath/mobile.env | tr -d '\r' | tail -n 1)
[[ -z $configured_ws || $configured_ws == "$ws" ]] || { echo 'FOOTBATH_WS points elsewhere; inspect configuration first'; exit 1; }
# Reject camera-specific/custom startup paths rather than replacing system configuration.
systemctl show footbath-mobile.service -p ExecStart --value | grep -Fq "$ws/install/rk3576_footbath_mobile/share/rk3576_footbath_mobile/scripts/footbath_mobile_start"
grep -q '^ReadWritePaths=' /etc/systemd/system/footbath-mobile.service
[[ $(df -Pk /home/sky | awk 'NR==2 {print $4}') -ge 4194304 ]] || { echo 'Need at least 4 GiB free'; exit 1; }
stage=$(mktemp -d /home/sky/footbath-stage-XXXXXX)
chown sky:sky "$stage"; chmod 755 "$stage"
sudo -u sky tar -xzf "$upload/source.tar.gz" -C "$stage"
[[ -f $stage/dependencies.repos && -d $stage/src/rk3576_footbath_mobile ]]
driver_commit=34300099fadfc772965962dec837bf436706188f
grep -Fq "$driver_commit" "$stage/dependencies.repos" || { echo 'Dependency pin changed; update deploy helper'; exit 1; }
mkdir -p "$stage/src/sllidar_ros2"; chown sky:sky "$stage/src/sllidar_ros2"
if sudo -u sky git -C "$ws/src/sllidar_ros2" cat-file -e "$driver_commit^{commit}" 2>/dev/null; then
  sudo -u sky git clone --no-hardlinks --no-checkout "$ws/src/sllidar_ros2" "$stage/src/sllidar_ros2"
  sudo -u sky git -C "$stage/src/sllidar_ros2" checkout --detach "$driver_commit"
else
  sudo -u sky git clone https://github.com/Slamtec/sllidar_ros2.git "$stage/src/sllidar_ros2"
  sudo -u sky git -C "$stage/src/sllidar_ros2" checkout --detach "$driver_commit"
fi
backup="${ws}_before_$(basename "$upload")"
[[ ! -e $backup ]]
swapped=0
rollback() {
  rc=$?
  trap - EXIT
  if (( rc != 0 && swapped == 1 )); then
    echo 'Deployment failed; restoring previous workspace. Services remain STOPPED.'
    systemctl stop footbath-mobile.service footbath-foxglove.service rk3576-footbath.service || true
    failed="${ws}_failed_$(basename "$upload")"
    if [[ ! -e $failed ]] && mv -T "$ws" "$failed" && mv -T "$backup" "$ws"; then
      echo "ROLLBACK COMPLETE. Failed build/logs: $failed"
    else
      echo "ROLLBACK INCOMPLETE. Do not start services; inspect $ws and $backup"
    fi
  fi
  exit "$rc"
}
trap rollback EXIT
systemctl stop footbath-mobile.service footbath-foxglove.service rk3576-footbath.service
# Separate camera or manually started ROS programs must be stopped by their owner.
if pgrep -af '[/]opt/ros/humble/.*(ros2|controller_server|planner_server)|[/]home/sky/.*(camera|sllidar|footbath.*install)|[g]st-launch|[l]ibcamera|[r]picam' ; then
  echo 'Other ROS/camera processes remain; workspace unchanged. Stop them and retry.'; exit 1
fi
mv -T "$ws" "$backup"
if ! mv -T "$stage" "$ws"; then mv -T "$backup" "$ws"; exit 1; fi
swapped=1
mkdir -p "$ws/maps"
if [[ -d $backup/maps ]]; then cp -a "$backup/maps/." "$ws/maps/"; fi
# Preserve legacy map files too; do not copy old program files.
old_maps="$backup/src/rk3576_footbath_navigation/maps"
if [[ -d $old_maps ]]; then
  mkdir -p "$ws/src/rk3576_footbath_navigation/maps"
  find "$old_maps" -maxdepth 1 -type f ! -name README.md -exec cp -p -t "$ws/src/rk3576_footbath_navigation/maps" -- {} +
fi
chown -R sky:sky "$ws"
sudo -u sky env -i HOME=/home/sky USER=sky PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
  bash --noprofile --norc -c 'set -eo pipefail; cd /home/sky/rk3576_footbath_ws; source /opt/ros/humble/setup.bash; colcon build --symlink-install --executor sequential --event-handlers console_cohesion+'
# Validate every source shell entrypoint, including files without extensions.
while IFS= read -r -d '' script; do
  if head -n 1 "$script" | grep -Eq '^#!.*(bash|/sh)'; then
    if LC_ALL=C grep -q $'\r' "$script"; then echo "CRLF detected: $script"; exit 1; fi
    bash -n "$script"
  fi
done < <(find "$ws/scripts" "$ws/src" -type f -path '*/scripts/*' -print0)
printf 'commit=%s\ndriver_commit=%s\ndeployed_utc=%s\nprevious_workspace=%s\n' "$commit" "$driver_commit" "$(date -u +%FT%TZ)" "$backup" > "$ws/DEPLOYED_VERSION.txt"
systemctl reset-failed footbath-mobile.service footbath-foxglove.service
systemctl start footbath-mobile.service footbath-foxglove.service
mobile_pid=$(systemctl show footbath-mobile.service -p MainPID --value)
fox_pid=$(systemctl show footbath-foxglove.service -p MainPID --value)
for attempt in {1..10}; do
  sleep 2
  systemctl is-active --quiet footbath-mobile.service
  systemctl is-active --quiet footbath-foxglove.service
  [[ $mobile_pid != 0 && $mobile_pid == "$(systemctl show footbath-mobile.service -p MainPID --value)" ]]
  [[ $fox_pid != 0 && $fox_pid == "$(systemctl show footbath-foxglove.service -p MainPID --value)" ]]
done
swapped=0
echo "DEPLOYMENT OK: $commit"
echo "Previous workspace retained: $backup"
echo 'Gateway/bridge stayed active for 20 seconds. Sensor, mapping and movement acceptance still required.'
