"""按单次开机日志汇总主阶段与作业耗时，不从 ACK 或目标位置推断完成。

用法：python scripts/summarize_motion_log.py run.log > timing.md
支持原始串口帧及上位机带 [RX] 前缀的日志；时间均取设备开机毫秒。
PHASE/FLAG 的编号 0 为主流程，RUN_START/RUN_RESULT 定义正式比赛计时边界。
RESULT 为运动作业结果，采用附件计划的 编号,时间,结果,总耗时 字段。
区间并集用于区分累计作业耗时与实际覆盖时间；重叠量不能直接宣称为优化收益。
SPAN 记录路径、观察及视觉分项；STEP 记录机械臂进入/退出。缺失结束事件不补造耗时。
区间建模参考 https://github.com/google/perfetto/blob/main/docs/instrumentation/track-events.md
仅参考模型，未复制实现或引入依赖。
"""

import argparse
from dataclasses import dataclass
from pathlib import Path
import re


@dataclass(frozen=True)
class Event:
    kind: str
    job: str
    time: int
    fields: tuple[str, ...]


def parse_events(lines):
    sizes = {"PHASE": 1, "FLAG": 2, "RUN_START": 0, "RUN_RESULT": 2, "RESULT": 2,
             "SPAN": 2, "STEP": 3, "ERROR": 1}
    events = []
    for number, line in enumerate(lines, 1):
        for match in re.finditer(r"\{EVT,MOTION,([^{}]+)\}", line):
            parts = match.group(1).split(",")
            kind = parts[0]
            if kind not in sizes:
                continue
            if len(parts) != 3 + sizes[kind]:
                raise ValueError(f"第 {number} 行 MOTION 字段数量错误")
            time = int(parts[2])
            if time < 0:
                raise ValueError(f"第 {number} 行时间为负数")
            events.append(Event(kind, parts[1], time, tuple(parts[3:])))
    return sorted(events, key=lambda event: event.time)


def union_ms(intervals):
    """合并重叠和嵌套区间，避免把并行执行的时间重复计入总量。"""
    total, covered_until = 0, 0
    for start, end in sorted(intervals):
        total += max(0, end - max(start, covered_until))
        covered_until = max(covered_until, end)
    return total


def summarize(events):
    starts = [e for e in events if e.kind == "RUN_START" and e.job == "0"]
    results = [e for e in events if e.kind == "RUN_RESULT" and e.job == "0"]
    if len(starts) > 1 or len(results) > 1:
        raise ValueError("请按单次比赛拆分日志，不要合并多次开机或多场比赛")
    start = starts[0].time if starts else None
    end = results[0].time if results else None
    status, total = "INCOMPLETE", None
    if start is not None and end is not None:
        total = int(results[0].fields[1])
        if total < 0 or end - start != total:
            raise ValueError("比赛总耗时与设备起止时间不一致")
        status = results[0].fields[0]

    phases = [e for e in events if e.kind == "PHASE" and e.job == "0"]
    rows = []
    for index, phase in enumerate(phases):
        finish = phases[index + 1].time if index + 1 < len(phases) else end
        begin = max(phase.time, start) if start is not None else phase.time
        if end is not None:
            finish = min(finish, end) if finish is not None else end
        if finish is not None and finish <= begin:
            continue
        rows.append((phase.fields[0], begin, finish, None if finish is None else finish - begin))

    # 作业各自的墙钟耗时可相互重叠，不能把它们相加作为比赛耗时。
    jobs = []
    for event in events:
        if event.kind == "RESULT" and event.job != "0":
            duration = int(event.fields[1])
            if duration < 0 or duration > event.time:
                raise ValueError("作业耗时无效；日志需来自同一次开机且 millis 未回绕")
            jobs.append((event.time - duration, event.time))
    serial = sum(finish - begin for begin, finish in jobs)
    wall = union_ms(jobs)
    opened, components, unfinished = {}, {}, []
    for event in events:
        if event.kind not in ("SPAN", "STEP"):
            continue
        name = event.fields[0] if event.kind == "SPAN" else "STEP_" + event.fields[1]
        key = (event.kind, event.job, event.fields[0])
        edge = event.fields[-1]
        if edge == "BEGIN":
            if key in opened:
                unfinished.append((event.job, name, opened[key]))
            opened[key] = event.time
        elif edge == "END" and key in opened:
            begin = opened.pop(key)
            finish = event.time
            if start is not None:
                begin = max(begin, start)
            if end is not None:
                finish = min(finish, end)
            if finish >= begin:
                components.setdefault(name, []).append((begin, finish))
    unfinished.extend((key[1], key[2], begin) for key, begin in opened.items())
    component_rows = [(name, len(intervals), sum(b-a for a, b in intervals), union_ms(intervals))
                      for name, intervals in sorted(components.items())]
    return dict(status=status, total_ms=total, phases=rows, jobs=len(jobs),
                job_sum_ms=serial, job_wall_ms=wall, job_overlap_ms=serial - wall,
                components=component_rows, unfinished=unfinished)


def render(report):
    total = "未知（未获得完整起止事件）" if report["total_ms"] is None else f'{report["total_ms"]} ms'
    lines = [f'比赛结果：{report["status"]}；正式耗时：{total}。', "",
             "| 主阶段 | 开始 ms | 结束 ms | 耗时 ms |", "| --- | ---: | ---: | ---: |"]
    for name, begin, end, duration in report["phases"]:
        lines.append(f'| {name} | {begin} | {end if end is not None else "未结束"} | '
                     f'{duration if duration is not None else "未知"} |')
    lines += ["", f'已结束作业：{report["jobs"]}；累计耗时：{report["job_sum_ms"]} ms；'
              f'区间并集：{report["job_wall_ms"]} ms；重叠量：{report["job_overlap_ms"]} ms。',
              "重叠量仅描述日志中的并行关系，不代表相对原流程的节省量。",
              "阶段时间为主任务观察边界；暂停后的末段及缺失事件仍需补测。", "",
              "| 分项 | 次数 | 累计 ms | 区间并集 ms |", "| --- | ---: | ---: | ---: |"]
    lines.extend(f"| {name} | {count} | {total} | {wall} |" for name, count, total, wall in report["components"])
    lines += ["", "分项包含嵌套和并行关系，不可跨行相加作为比赛总耗时。",
              f'缺少结束事件的分项：{len(report["unfinished"])}；这些分项不补算耗时。']
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="一次开机、一次比赛的 UTF-8 串口日志")
    args = parser.parse_args()
    try:
        with args.log.open(encoding="utf-8-sig") as stream:
            report = summarize(parse_events(stream))
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(render(report), end="")


if __name__ == "__main__":
    main()
