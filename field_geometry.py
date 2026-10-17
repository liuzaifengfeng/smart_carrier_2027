"""上位机和固件共享 field_geometry.def，避免只画障碍却不参与规划。"""
import math
from pathlib import Path
import re


def _load():
    bounds = None
    rectangles = []
    for line in (Path(__file__).parent/"src/field_geometry.def").read_text(encoding="utf-8").splitlines():
        line = line.split("//", 1)[0].strip()
        if not line:
            continue
        match = re.fullmatch(r"(FIELD_BOUNDS|FIELD_OBSTACLE)\(([\d.,\s]+)\)", line)
        if not match:
            raise ValueError("共享场地几何格式无效")
        values = tuple(float(v.strip()) for v in match[2].split(","))
        if match[1] == "FIELD_BOUNDS" and len(values) == 2:
            bounds = values
        elif match[1] == "FIELD_OBSTACLE" and len(values) == 4:
            rectangles.append(values)
        else:
            raise ValueError("共享场地几何字段数无效")
    if bounds is None or len(rectangles) != 4:
        raise ValueError("共享场地几何不完整")
    return *bounds, tuple(rectangles)


FIELD_SIZE, BODY_SIZE, OBSTACLES = _load()
GEOMETRY_SIGNATURE = 2166136261
for _v in (FIELD_SIZE, BODY_SIZE, *(v for r in OBSTACLES for v in r)):
    GEOMETRY_SIGNATURE = ((GEOMETRY_SIGNATURE ^ int(round(_v*1000)))*16777619) & 0xffffffff


def check_pose(x, y, heading, margin=20):
    angle = math.radians(heading)
    fx, fy, rx, ry = math.cos(angle), math.sin(angle), math.sin(angle), -math.cos(angle)
    half = BODY_SIZE/2
    extent = half*(abs(fx)+abs(rx))
    if (min(x-extent, y-extent) < margin-.001
            or max(x+extent, y+extent) > FIELD_SIZE-margin+.001):
        return "BOUNDARY"
    for left, right, bottom, top in OBSTACLES:
        dx, dy = x-(left+right)/2, y-(bottom+top)/2
        hx, hy = (right-left)/2+margin, (top-bottom)/2+margin
        separated = False
        for ux, uy in ((1, 0), (0, 1), (fx, fy), (rx, ry)):
            body = half*(abs(ux*fx+uy*fy)+abs(ux*rx+uy*ry))
            obstacle = hx*abs(ux)+hy*abs(uy)
            if abs(dx*ux+dy*uy) >= body+obstacle-.001:
                separated = True
                break
        if not separated:
            return "OBSTACLE"
    return None
