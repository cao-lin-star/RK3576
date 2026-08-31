"""Pure validation, authentication and map rendering helpers."""
from __future__ import annotations
import hashlib, hmac, math, secrets, struct, zlib
ALLOWED_MODES = {"idle", "mapping", "auto_mapping", "navigation"}
def clamp_command(linear: float, angular: float) -> tuple[float, float]:
    if not math.isfinite(linear) or not math.isfinite(angular): raise ValueError("non-finite velocity")
    return max(-0.30, min(0.30, linear)), max(-0.80, min(0.80, angular))
def valid_pose(x: float, y: float, yaw: float) -> bool:
    return all(math.isfinite(v) for v in (x,y,yaw)) and abs(x)<=100.0 and abs(y)<=100.0
def hash_pin(pin: str, salt_hex: str, rounds: int=200_000) -> str:
    if len(pin)<6: raise ValueError("PIN must contain at least 6 characters")
    return hashlib.pbkdf2_hmac("sha256",pin.encode(),bytes.fromhex(salt_hex),rounds).hex()
def verify_pin(pin: str, salt_hex: str, expected_hex: str) -> bool:
    try: actual=hash_pin(pin,salt_hex)
    except (ValueError,TypeError): return False
    return hmac.compare_digest(actual,expected_hex)
def new_session_token() -> str: return secrets.token_urlsafe(32)
def map_cell(data,width,height,resolution,origin_x,origin_y,x,y):
    if width<=0 or height<=0 or resolution<=0: return None
    cx=int(math.floor((x-origin_x)/resolution)); cy=int(math.floor((y-origin_y)/resolution))
    return (cx,cy) if 0<=cx<width and 0<=cy<height else None
def target_is_clear(data,width,height,resolution,origin_x,origin_y,x,y,clearance_m=0.35):
    cell=map_cell(data,width,height,resolution,origin_x,origin_y,x,y)
    if cell is None: return False
    cx,cy=cell; radius=max(1,int(math.ceil(clearance_m/resolution)))
    for yy in range(cy-radius,cy+radius+1):
      for xx in range(cx-radius,cx+radius+1):
        if xx<0 or yy<0 or xx>=width or yy>=height: return False
        if (xx-cx)**2+(yy-cy)**2>radius**2: continue
        value=int(data[yy*width+xx])
        if value<0 or value>=50: return False
    return True
def occupancy_png(data,width,height):
    if width<=0 or height<=0 or len(data)!=width*height: raise ValueError("invalid occupancy grid")
    raw=bytearray()
    for y in range(height-1,-1,-1):
      raw.append(0)
      for value in data[y*width:(y+1)*width]:
        raw.append(205 if value<0 else max(0,min(255,255-int(value)*255//100)))
    def chunk(kind,payload):
      return struct.pack(">I",len(payload))+kind+payload+struct.pack(">I",zlib.crc32(kind+payload)&0xffffffff)
    return b"\x89PNG\r\n\x1a\n"+chunk(b"IHDR",struct.pack(">IIBBBBB",width,height,8,0,0,0,0))+chunk(b"IDAT",zlib.compress(bytes(raw),6))+chunk(b"IEND",b"")
