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
from datetime import datetime
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


def summarize(events, *, observation_start=None, terminal_time=None, strict=True):
    starts = [e for e in events if e.kind == "RUN_START" and e.job == "0"]
    results = [e for e in events if e.kind == "RUN_RESULT" and e.job == "0"]
    if len(starts) > 1 or len(results) > 1:
        raise ValueError("请按单次比赛拆分日志，不要合并多次开机或多场比赛")
    start = starts[0].time if starts else None
    end = results[0].time if results else terminal_time
    status, total = "INCOMPLETE", None
    if start is not None and results:
        elapsed = results[0].fields[1]
        if elapsed.isdecimal() and end - start == int(elapsed):
            total = int(elapsed)
            status = results[0].fields[0]
        elif strict:
            raise ValueError("比赛总耗时与设备起止时间不一致")
    if observation_start is not None:
        start = observation_start

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
    intervals = []
    for event in events:
        if event.kind == "RESULT" and event.job != "0":
            elapsed = event.fields[1]
            if not elapsed.isdecimal() or int(elapsed) > event.time:
                if strict:
                    raise ValueError("作业耗时无效；日志需来自同一次开机且 millis 未回绕")
                continue
            duration = int(elapsed)
            begin = event.time - duration
            finish = event.time
            if start is not None:
                begin = max(begin, start)
            if end is not None:
                finish = min(finish, end)
            if finish >= begin:
                jobs.append((begin, finish))
                intervals.append(("RESULT", event.fields[0], event.job, begin, finish))
    serial = sum(finish - begin for begin, finish in jobs)
    wall = union_ms(jobs)
    opened, components, unfinished, unmatched_ends = {}, {}, [], []
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
                intervals.append((event.kind, name, event.job, begin, finish))
        elif edge == "END":
            unmatched_ends.append((event.job, name, event.time))
    unfinished.extend((key[1], key[2], begin) for key, begin in opened.items())
    component_rows = [(name, len(intervals), sum(b-a for a, b in intervals), union_ms(intervals))
                      for name, intervals in sorted(components.items())]
    return dict(status=status, total_ms=total, phases=rows, jobs=len(jobs),
                job_sum_ms=serial, job_wall_ms=wall, job_overlap_ms=serial - wall,
                components=component_rows, unfinished=unfinished, unmatched_ends=unmatched_ends,
                intervals=intervals)


MAIN_PHASES = (
    "READ_TASK", "GRAB_ROUND1", "PLACE_COARSE1", "PLACE_TEMP1",
    "GRAB_ROUND2", "PLACE_COARSE2", "STACK_TEMP2", "RETURN_HOME",
)
MOTION_FRAME = re.compile(r"\{EVT,MOTION,[^{}\r\n]+\}")
START_SIGNAL = re.compile(r"\{RSP,SYS,(?:RELEASE|START),OK\}|^\[StartPanel\] button=PRESS\b.*\bstart=REQUESTED$")
FAILED_PHASE = re.compile(r"\{EVT,MOTION,PHASE,0,\d+,[A-Z_]+_FAILED\}")


def archive_logs(logs):
    """Validate the wire representation shared by snapshots and automatic capture."""
    if not isinstance(logs, list) or not logs or any(
        not isinstance(entry, dict) or
        not all(isinstance(entry.get(field), str) for field in ("time", "direction", "text"))
        for entry in logs
    ):
        raise ValueError("请提供带时间、方向和正文的日志")
    return logs


def snapshot_sections(logs):
    """Keep separate task timings when a 60-second snapshot includes multiple runs."""
    section = []
    release_seen = formal_seen = ended = False
    for log in logs:
        line = log["text"] if log["direction"] == "RX" else ""
        release = "{RSP,SYS,RELEASE,OK}" in line
        start = "{EVT,MOTION,RUN_START,0," in line
        result = "{EVT,MOTION,RUN_RESULT,0," in line
        if ((release and (release_seen or formal_seen or ended))
                or (START_SIGNAL.search(line) and ended)
                or (start and (formal_seen or ended)) or (result and ended and not formal_seen)):
            yield section
            section = []
            release_seen = formal_seen = ended = False
        section.append(log)
        release_seen |= release
        formal_seen |= start
        ended |= result or bool(FAILED_PHASE.search(line))
        if result:
            formal_seen = False
    if section:
        yield section


def analyze_archive(frames=(), logs=(), end_reason=None):
    """Analyze the observed portion without estimating missing formal boundaries."""
    if logs:
        frames = [frame for log in logs if log["direction"] == "RX"
                  for frame in MOTION_FRAME.findall(log["text"])]
    events, valid_frames = [], []
    for frame in frames:
        if not isinstance(frame, str) or not MOTION_FRAME.fullmatch(frame):
            continue
        try:
            parsed = parse_events([frame])
        except ValueError:
            continue
        if parsed:
            events.extend(parsed)
            valid_frames.append(frame)

    events.sort(key=lambda event: event.time)
    start = events[0].time if events else 0
    failure = next((event for event in reversed(events) if event.job == "0" and
                    event.kind == "PHASE" and event.fields[0].endswith("_FAILED")), None)
    result = next((event for event in reversed(events)
                   if event.job == "0" and event.kind == "RUN_RESULT"), None)
    terminal = failure or next((event for event in reversed(events)
                               if event.job == "0" and event.kind == "PHASE" and event.fields[0] == "DONE"), None)
    report = summarize(events, observation_start=start, terminal_time=terminal.time if terminal else None,
                       strict=False)
    total = report["total_ms"]
    phases = [(name, begin - start, None if end is None else end - start, duration)
              for name, begin, end, duration in report["phases"]
              if name != "DONE" and not name.endswith("_FAILED")]
    if events and events[0].kind == "RUN_START" and phases and phases[0][0] == "READ_TASK":
        name, begin, end, duration = phases[0]
        phases[0] = (name, 0, end, duration + begin if duration is not None else None)
    status = (result.fields[0] if result else "FAILED" if failure or end_reason == "FAILED" else "DONE" if terminal else
              end_reason if end_reason in ("SILENT", "DISCONNECTED", "RESTARTED") else "INCOMPLETE")
    reason = failure.fields[0] if failure else ""
    end = result or terminal
    timeline_ms = max(0, (end or events[-1]).time - start) if events else 0
    observed_ms = timeline_ms
    if logs:
        try:
            first, last = (datetime.fromisoformat(item["time"].replace("Z", "+00:00"))
                           for item in (logs[0], logs[-1]))
            observed_ms = max(0, round((last - first).total_seconds() * 1000))
        except (ValueError, TypeError):
            pass

    missing = []
    if total is None:
        missing.append("正式起止事件缺失或时间不一致，正式耗时未知")
    if not events:
        missing.append("没有可计算阶段或作业耗时的任务事件")
    if len(frames) > len(valid_frames):
        missing.append(f"跳过 {len(frames) - len(valid_frames)} 条非计时或无效任务事件")
    logged_phases = {name for name, *_ in phases}
    if total is not None and status == "DONE":
        missing.extend(f"缺少阶段 {name}" for name in MAIN_PHASES if name not in logged_phases)
    if any(row[3] is None for row in phases):
        missing.append("最后的业务阶段缺少结束事件")
    if report["unfinished"]:
        missing.append(f'{len(report["unfinished"])} 个动作区间缺少结束事件')
    if report["unmatched_ends"]:
        missing.append(f'{len(report["unmatched_ends"])} 个动作区间缺少开始事件')
    return dict(
        total_ms=total, observed_ms=observed_ms, timeline_ms=timeline_ms or observed_ms,
        status=status, reason=reason, phases=phases, missing=missing,
        intervals=[(kind, name, job, begin-start, end-start)
                   for kind, name, job, begin, end in report["intervals"]],
        components=report["components"], job_sum_ms=report["job_sum_ms"],
        job_wall_ms=report["job_wall_ms"], frames=valid_frames, logs=logs,
    )


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
