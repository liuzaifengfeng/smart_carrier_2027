"""独立试跑混合轨迹；与 src/hybrid_route.cpp 使用相同几何规则。"""
import math
from dataclasses import dataclass, field, replace
from continuous_trial import TrialPlan, angle_delta, adjacent
from field_geometry import check_pose, GEOMETRY_SIGNATURE


def road_heading(current, direction):
    forward = math.degrees(math.atan2(direction[1], direction[0])) % 360
    back = (forward + 180) % 360
    return back if abs(angle_delta(current, back)) < abs(angle_delta(current, forward)) - .01 else forward


def stopped_headings(directions, start, end):
    candidates = []
    for x, y in directions:
        forward = math.degrees(math.atan2(y, x)) % 360
        candidates.append((forward, (forward+180) % 360))
    costs = [[0., 0.] for _ in candidates]
    for i in range(len(candidates)-1, -1, -1):
        for k, heading in enumerate(candidates[i]):
            costs[i][k] = (abs(angle_delta(heading, end)) if i == len(candidates)-1 else
                           min(abs(angle_delta(heading, h))+costs[i+1][j]
                               for j, h in enumerate(candidates[i+1])))
    heads, current = [], start
    for i, pair in enumerate(candidates):
        a, b = (abs(angle_delta(current, h)) for h in pair)
        ca, cb = a+costs[i][0], b+costs[i][1]
        k = int(cb < ca-.01 or (abs(cb-ca) <= .01 and b < a-.01))
        current = pair[k]
        heads.append(current)
    return heads


@dataclass
class Phase:
    start: tuple
    end: tuple
    h0: float
    h1: float
    kind: str = "LINE"
    offset: float = 0
    length: float = 0
    c1: tuple = (0, 0)
    c2: tuple = (0, 0)
    lut: list = field(default_factory=list)

    def cubic(self, t):
        u = 1-t
        return tuple(a*u**3 + b*3*u*u*t + c*3*u*t*t + d*t**3
                     for a, b, c, d in zip(self.start, self.c1, self.c2, self.end))

    def at(self, distance):
        local = min(self.length, max(0, distance-self.offset))
        if self.kind == "ROTATE":
            return self.start
        if self.kind == "LINE":
            return tuple(a+(b-a)*local/self.length for a, b in zip(self.start, self.end))
        i = 1
        while i < 64 and self.lut[i] < local:
            i += 1
        span = self.lut[i]-self.lut[i-1]
        return self.cubic((i-1+((local-self.lut[i-1])/span if span else 0))/64)

    def heading(self, local):
        u = min(1, max(0, local/self.length)) if self.length else 0
        return (self.h0+angle_delta(self.h0, self.h1)*u**3*(10-15*u+6*u*u)) % 360

    def samples(self):
        count = max(1, math.ceil(max(self.length/2, abs(angle_delta(self.h0, self.h1))*1.875)))
        for i in range(count+1):
            u = i/count
            x, y = self.at(self.offset+self.length*u)
            heading = ((self.h0+angle_delta(self.h0, self.h1)*u) % 360
                       if self.kind == "ROTATE" else self.heading(self.length*u))
            yield self.offset+self.length*u, x, y, heading


def outside_curve(p, incoming, outgoing, trim, outset, h0, h1):
    start = tuple(v-d*trim for v, d in zip(p, incoming))
    end = tuple(v+d*trim for v, d in zip(p, outgoing))
    handle = (4*trim+8*outset)/3
    phase = Phase(start, end, h0, h1, "CUBIC",
                  c1=tuple(v+d*handle for v, d in zip(start, incoming)),
                  c2=tuple(v-d*handle for v, d in zip(end, outgoing)), lut=[0.])
    previous = start
    for i in range(1, 65):
        point = phase.cubic(i/64)
        phase.length += math.dist(point, previous)
        phase.lut.append(phase.length)
        previous = point
    return phase


def valid_phase(phase, margin):
    return next((error for _, x, y, h in phase.samples() if (error := check_pose(x, y, h, margin))), None)


class HybridPlan(TrialPlan):
    def __init__(self, nodes, positions, config, auto_end=False):
        config.validate()
        if config.policy not in (1, 2):
            raise ValueError("请选择自动混合或全程停转策略")
        if not 2 <= len(nodes) <= 25 or any(not isinstance(n, int) or not 0 <= n < 25 for n in nodes):
            raise ValueError("需要2～25个节点，编号0～24")
        if any(not adjacent(a, b) for a, b in zip(nodes, nodes[1:])):
            raise ValueError("相邻输入节点必须在网格上相邻")
        self.nodes, self.radii, self.positions, self.config = tuple(nodes), (0,)*len(nodes), dict(positions), config
        self.segments, self.preview, self.length = [], [], 0.
        points = [tuple(map(float, positions[n])) for n in nodes]
        if any(not math.isfinite(v) for p in points for v in p):
            raise ValueError("地图坐标必须为有限数")
        directions, distances = [], []
        for a, b in zip(points, points[1:]):
            d = math.dist(a, b)
            if d < 20:
                raise ValueError("节点间距不足20mm")
            distances.append(d)
            directions.append(tuple((v-u)/d for u, v in zip(a, b)))
        heads = (stopped_headings(directions, config.start, config.end)
                 if config.policy == 2 and not auto_end else [road_heading(config.start, directions[0])])
        if abs(angle_delta(config.start, heads[0])) > .05:
            self._append(Phase(points[0], points[0], config.start, heads[0], "ROTATE"), "起点原地转向空间不足")
        curves = {}
        for i in range(1, len(nodes)-1):
            if config.policy != 2 or auto_end:
                heads.append(road_heading(heads[-1], directions[i]))
            dot = sum(a*b for a, b in zip(directions[i-1], directions[i]))
            if dot > .9999:
                continue
            if abs(dot) > .001 and dot > -.9999:
                raise ValueError("仅支持直线、90°转弯及原路返回")
            x, y = points[i]
            if config.policy == 1 and (x < 550 or x > 1850) and (y < 550 or y > 1850) and abs(dot) < .001:
                trim = min(180, (distances[i-1]-20)*.45, (distances[i]-20)*.45)
                if trim >= 30:
                    headings = (heads[i], (heads[i]+180) % 360)
                    for outset in (config.outset, 80, 100, 40, 20, 0):
                        # 同一外绕量同时验证正向/倒退出弯，不能因单一姿态失败直接停转。
                        for heading in headings:
                            candidate = outside_curve(points[i], directions[i-1], directions[i], trim, outset, heads[i-1], heading)
                            if not valid_phase(candidate, config.margin):
                                curves[i] = candidate
                                heads[i] = heading
                                break
                        if i in curves:
                            break
        cursor = points[0]
        for edge in range(len(nodes)-1):
            i = edge+1
            curve = curves.get(i)
            entry = curve.start if curve else points[i]
            distance = math.dist(cursor, entry)
            if distance > .001:
                self._append(Phase(cursor, entry, heads[edge], heads[edge], length=distance), "直线路段")
            if i+1 < len(nodes):
                if curve:
                    self._append(curve, "外围外绕")
                    cursor = curve.end
                else:
                    if sum(a*b for a, b in zip(directions[edge], directions[i])) < .9999:
                        self._append(Phase(points[i], points[i], heads[edge], heads[i], "ROTATE"), f"节点{nodes[i]}原地转向空间不足")
                    cursor = points[i]
        if auto_end:
            self.config = config = replace(config, end=heads[-1])
        self._append(Phase(points[-1], points[-1], heads[-1], config.end, "ROTATE"), "终点转向空间不足")
        if math.ceil(self.length/5)+1 > 4096:
            raise ValueError("路径长度超出固件速度规划容量")

    def _append(self, phase, label):
        phase.offset = self.length
        error = valid_phase(phase, self.config.margin)
        if error:
            raise ValueError(f"{label}：车体{'越界' if error == 'BOUNDARY' else '侵入黄色禁入区'}，含{self.config.margin:g}mm余量；请改路径或朝向")
        self.segments.append(phase)
        self.length += phase.length
        self.preview.extend(phase.samples())

    def heading(self, s):
        for phase in self.segments:
            if phase.length and s < phase.offset+phase.length-.0001:
                return phase.heading(s-phase.offset)
        return next(p for p in reversed(self.segments) if p.length).h1

    def at(self, s):
        for phase in self.segments:
            if phase.length and s < phase.offset+phase.length-.0001:
                return phase.at(s)
        return self.segments[-1].end

    def commands(self, job_id, build_frame):
        frames = super().commands(job_id, build_frame)
        c = self.config
        frames.insert(1, build_frame("CMD", "NAV", "TRIAL_POLICY", job_id, c.policy,
                                     f"{c.outset:.6g}", f"{c.margin:.6g}", GEOMETRY_SIGNATURE))
        return frames
