"""Persist the session start, bound to the saved occupancy image."""
import hashlib
import json
import math
import os
from pathlib import Path
import time
import yaml


def save_home(prefix, pose):
    if pose is None:
        raise ValueError('尚未记录有效建图起点')
    p, q = pose.pose.position, pose.pose.orientation
    data = dict(x=p.x, y=p.y,
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    if not all(math.isfinite(v) for v in data.values()):
        raise ValueError('invalid mapping home')
    image = Path(prefix+'.pgm')
    target = Path(prefix+'.home.json')
    grid = yaml.safe_load(Path(prefix+'.yaml').read_text(encoding='utf-8'))
    payload = dict(version=1, frame_id='map', pose=data, saved_at=time.time(),
                   geometry=dict(resolution=grid['resolution'], origin=grid['origin']),
                   image_sha256=hashlib.sha256(image.read_bytes()).hexdigest())
    temporary = target.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
    os.replace(temporary, target)
