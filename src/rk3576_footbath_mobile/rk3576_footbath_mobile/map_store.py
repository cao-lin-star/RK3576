"""Safe filesystem-backed map catalog for the mobile gateway."""
from __future__ import annotations

import os
import json
import math
import hashlib
import yaml
import re
import threading
import time
from pathlib import Path

_MAP_NAME = re.compile(r"^[0-9A-Za-z_\-\u4e00-\u9fff]{1,64}$")
_IMAGE_LINE = re.compile(r"^(\s*image\s*:\s*).*$", re.MULTILINE)


class MapStore:
    def __init__(self, roots):
        self.roots = tuple(Path(root).resolve() for root in roots)
        self._lock = threading.RLock()
        self._paths_cache = []
        self._cache_at = 0.0

    def _invalidate(self):
        self._paths_cache = []
        self._cache_at = 0.0

    def checked_yaml(self, raw_path):
        candidate = Path(str(raw_path)).resolve()
        if candidate.suffix.lower() != ".yaml" or candidate.parent not in self.roots:
            raise ValueError("map path is not allowed")
        if not candidate.is_file():
            raise ValueError("map YAML does not exist")
        return candidate

    def paths(self):
        with self._lock:
            now = time.monotonic()
            if now - self._cache_at < 5.0:
                return list(self._paths_cache)
            result = []
            for root in self.roots:
                if root.is_dir():
                    result.extend(str(path.resolve()) for path in root.glob("*.yaml"))
            self._paths_cache = sorted(set(result))
            self._cache_at = now
            return list(self._paths_cache)

    def _image_path(self, yaml_path):
        try:
            text = yaml_path.read_text(encoding="utf-8")
        except OSError:
            return None
        for line in text.splitlines():
            if line.lstrip().startswith("image:"):
                value = line.split(":", 1)[1].strip().strip("'\"")
                image = (yaml_path.parent / value).resolve()
                if image.parent != yaml_path.parent:
                    return None
                if image.suffix.lower() not in (".pgm", ".png", ".jpg", ".jpeg"):
                    return None
                return image
        return None

    def _group(self, yaml_path):
        files = [yaml_path]
        image = self._image_path(yaml_path)
        if image is not None and image.is_file():
            files.append(image)
        prefix = yaml_path.with_suffix("")
        for suffix in (".posegraph.data", ".posegraph.posegraph", ".home.json"):
            companion = Path(str(prefix) + suffix)
            if companion.is_file():
                files.append(companion)
        return files, image

    def home(self, raw_path):
        yaml_path = self.checked_yaml(raw_path)
        try:
            sidecar = yaml_path.with_suffix('.home.json')
            if sidecar.is_symlink() or sidecar.stat().st_size > 8192:
                return None
            data = json.loads(sidecar.read_text(encoding='utf-8'))
            pose = data['pose']
            if data['version'] != 1 or data['frame_id'] != 'map':
                return None
            if not all(isinstance(pose[k], (int,float)) and math.isfinite(pose[k]) for k in ('x','y','yaw')):
                return None
            image = self._image_path(yaml_path)
            grid = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
            if data['geometry'] != dict(resolution=grid['resolution'], origin=grid['origin']):
                return None
            if image is None or hashlib.sha256(image.read_bytes()).hexdigest() != data['image_sha256']:
                return None
            return {k:float(pose[k]) for k in ('x','y','yaw')}
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
            return None

    def entries(self, active_path=""):
        with self._lock:
            result = []
            active = str(Path(active_path).resolve()) if active_path else ""
            for raw in self.paths():
                yaml_path = self.checked_yaml(raw)
                files, image = self._group(yaml_path)
                stats = [path.stat() for path in files if path.is_file()]
                root_index = self.roots.index(yaml_path.parent)
                result.append({
                    "path": str(yaml_path),
                    "name": yaml_path.stem,
                    "source": "自动建图" if root_index == 0 else "手动建图/导航",
                    "modified_ms": int(max(stat.st_mtime for stat in stats) * 1000),
                    "size_bytes": sum(stat.st_size for stat in stats),
                    "complete": image is not None and image.is_file(),
                    "has_posegraph": any(path.name.endswith((".posegraph.data", ".posegraph.posegraph")) for path in files),
                    "active": str(yaml_path) == active,
                })
            return sorted(result, key=lambda item: item["modified_ms"], reverse=True)

    def rename(self, raw_path, requested_name):
        with self._lock:
            yaml_path = self.checked_yaml(raw_path)
            name = str(requested_name).strip()
            if name.lower().endswith(".yaml"):
                name = name[:-5]
            if not _MAP_NAME.fullmatch(name):
                raise ValueError("名称仅允许1到64个中文、字母、数字、短横线或下划线")
            if name == yaml_path.stem:
                return str(yaml_path)
            files, image = self._group(yaml_path)
            if image is None or not image.is_file():
                raise ValueError("地图图像缺失，不能安全重命名")
            target_yaml = yaml_path.with_name(name + ".yaml")
            target_image = image.with_name(name + image.suffix.lower())
            moves = []
            for source in files:
                if source == yaml_path:
                    continue
                if source == image:
                    target = target_image
                elif source.name.endswith(".posegraph.data"):
                    target = yaml_path.with_name(name + ".posegraph.data")
                elif source.name.endswith(".posegraph.posegraph"):
                    target = yaml_path.with_name(name + ".posegraph.posegraph")
                elif source.name.endswith('.home.json'):
                    target = yaml_path.with_name(name + '.home.json')
                else:
                    continue
                moves.append((source, target))
            targets = [target_yaml] + [target for _, target in moves]
            if any(target.exists() for target in targets):
                raise ValueError("目标名称已存在")
            old_text = yaml_path.read_text(encoding="utf-8")
            if not _IMAGE_LINE.search(old_text):
                raise ValueError("地图 YAML 缺少 image 字段")
            new_text = _IMAGE_LINE.sub(lambda match: match.group(1) + target_image.name, old_text, count=1)
            temp_yaml = yaml_path.with_name(f".{name}.yaml.tmp-{os.getpid()}")
            moved = []
            try:
                temp_yaml.write_text(new_text, encoding="utf-8")
                for source, target in moves:
                    source.rename(target)
                    moved.append((source, target))
                os.replace(temp_yaml, target_yaml)
                yaml_path.unlink()
            except Exception:
                if temp_yaml.exists():
                    temp_yaml.unlink()
                for source, target in reversed(moved):
                    if target.exists() and not source.exists():
                        target.rename(source)
                if target_yaml.exists() and not yaml_path.exists():
                    target_yaml.rename(yaml_path)
                raise
            self._invalidate()
            return str(target_yaml)

    def trash(self, raw_path):
        with self._lock:
            yaml_path = self.checked_yaml(raw_path)
            files, _ = self._group(yaml_path)
            trash_root = yaml_path.parent / ".trash"
            trash_root.mkdir(mode=0o700, parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            destination = trash_root / f"{stamp}_{yaml_path.stem}"
            suffix = 1
            while destination.exists():
                destination = trash_root / f"{stamp}_{yaml_path.stem}_{suffix}"
                suffix += 1
            destination.mkdir(mode=0o700)
            moved = []
            try:
                for source in files:
                    target = destination / source.name
                    source.rename(target)
                    moved.append((source, target))
            except Exception:
                for source, target in reversed(moved):
                    if target.exists() and not source.exists():
                        target.rename(source)
                destination.rmdir()
                raise
            self._invalidate()
            return str(destination)
    @staticmethod
    def _pgm_token(data, index):
        size = len(data)
        while index < size:
            if data[index] in b" \t\r\n":
                index += 1
                continue
            if data[index] == 35:
                while index < size and data[index] not in b"\r\n":
                    index += 1
                continue
            break
        start = index
        while index < size and data[index] not in b" \t\r\n#":
            index += 1
        if start == index:
            raise ValueError("invalid PGM header")
        return data[start:index], index

    def preview_data(self, raw_path):
        with self._lock:
            yaml_path = self.checked_yaml(raw_path)
            _, image = self._group(yaml_path)
            if image is None or not image.is_file():
                raise ValueError("地图图像缺失")
            if image.suffix.lower() != ".pgm":
                raise ValueError("当前仅支持PGM地图预览")
            raw = image.read_bytes()
            magic, index = self._pgm_token(raw, 0)
            width_token, index = self._pgm_token(raw, index)
            height_token, index = self._pgm_token(raw, index)
            max_token, index = self._pgm_token(raw, index)
            try:
                width, height, maximum = int(width_token), int(height_token), int(max_token)
            except ValueError as error:
                raise ValueError("invalid PGM dimensions") from error
            if width <= 0 or height <= 0 or width * height > 20_000_000:
                raise ValueError("PGM dimensions are not allowed")
            if maximum <= 0 or maximum > 255:
                raise ValueError("only 8-bit PGM maps are supported")
            pixel_count = width * height
            if magic == b"P5":
                if index >= len(raw) or raw[index] not in b" \t\r\n":
                    raise ValueError("invalid PGM binary separator")
                if raw[index:index + 2] == b"\r\n":
                    index += 2
                else:
                    index += 1
                pixels = list(raw[index:index + pixel_count])
                if len(pixels) != pixel_count:
                    raise ValueError("truncated PGM image")
            elif magic == b"P2":
                pixels = []
                while len(pixels) < pixel_count:
                    token, index = self._pgm_token(raw, index)
                    try:
                        pixels.append(int(token))
                    except ValueError as error:
                        raise ValueError("invalid PGM pixel") from error
            else:
                raise ValueError("unsupported PGM format")
            unknown = round(205 * maximum / 255)
            occupancy = []
            for row in range(height - 1, -1, -1):
                for pixel in pixels[row * width:(row + 1) * width]:
                    occupancy.append(-1 if abs(pixel - unknown) <= 1 else round((maximum - pixel) * 100 / maximum))
            return occupancy, width, height
