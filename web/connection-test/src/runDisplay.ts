import type { Interval, RunDetail } from './cloudRuns'

const phaseLabels: Record<string, string> = {
  READ_TASK: '读取任务', GRAB_ROUND1: '第一轮抓取', PLACE_COARSE1: '第一轮粗加工',
  PLACE_TEMP1: '第一轮暂存', GRAB_ROUND2: '第二轮抓取', PLACE_COARSE2: '第二轮粗加工',
  STACK_TEMP2: '第二轮码垛', RETURN_HOME: '返航归位', DONE: '结束',
}

const spanLabels: Record<string, string> = {
  NAVIGATION: '导航', ARM_PROGRAM: '机械臂', OBSERVATION: '观察姿态', VISION: '视觉对齐',
  VISION_ALIGN: '视觉对齐',
  CHASSIS_TRANSLATION: '底盘平移', CHASSIS_TURN: '底盘转向',
  PATH_WAIT: '等待路径', SCAN_APPROACH: '扫码接近', SCAN_SEARCH: '扫码搜索',
}

export const phases = Object.keys(phaseLabels).slice(0, -1)
const statusLabels: Record<string, string> = {
  WAIT_START: '启动准备', SCAN_FAILED: '扫码失败', ROUTE_FAILED: '导航失败', ALIGN_FAILED: '对齐失败',
  TIMEOUT_FAILED: '任务超时', TRANSFER_FAILED: '搬运失败',
}
export const phaseName = (name: string) => phaseLabels[name] ?? statusLabels[name] ?? name
export const activityName = (name: string) => spanLabels[name] ?? name
  .replace(/^VISION_WAIT_/, '视觉会话 · ')
  .replace(/^VISION_FIRST_FRAME_/, '等待视觉首帧 · ')
  .replace(/^STEP_/, '')
export const seconds = (value: number) => (value / 1000).toFixed(3) + ' s'
export const duration = (value: number | undefined) => value === undefined ? '缺数据' : seconds(value)
export const delta = (a: number | undefined, b: number | undefined) =>
  a === undefined || b === undefined ? '缺数据' : (b > a ? '+' : '') + seconds(b - a)
export const date = (value: string) => new Date(value).toLocaleString('zh-CN', { hour12: false })

export function phaseActivities(run: RunDetail, phase: string) {
  const row = run.phases.find(([name]) => name === phase)
  const groups = new Map<string, [number, number][]>()
  if (!row) return new Map<string, { sum: number; coverage: number }>()
  for (const [kind, name, , start, end] of run.intervals) {
    if (kind === 'RESULT') continue
    const begin = Math.max(start, row[1])
    const finish = Math.min(end, row[2])
    if (finish <= begin) continue
    const group = groups.get(name) ?? []
    group.push([begin, finish])
    groups.set(name, group)
  }
  return new Map(Array.from(groups, ([name, intervals]) => {
    let sum = 0
    let coverage = 0
    let coveredUntil = 0
    for (const [begin, end] of intervals.sort((a, b) => a[0] - b[0])) {
      sum += end - begin
      coverage += Math.max(0, end - Math.max(begin, coveredUntil))
      coveredUntil = Math.max(coveredUntil, end)
    }
    return [name, { sum, coverage }]
  }))
}

export function lanes(intervals: Interval[]) {
  const rows: Interval[][] = []
  for (const interval of [...intervals].sort((a, b) => a[3] - b[3])) {
    const row = rows.find(items => items.at(-1)![4] <= interval[3])
    if (row) row.push(interval)
    else rows.push([interval])
  }
  return rows
}

export function actionStats(run: RunDetail, kind: string, matches: (name: string) => boolean) {
  const intervals = run.intervals.filter(row => row[0] === kind && matches(row[1]))
    .sort((a, b) => a[3] - b[3])
  if (!intervals.length) return
  let sum = 0
  let coverage = 0
  let coveredUntil = 0
  for (const [, , , start, end] of intervals) {
    sum += end - start
    coverage += Math.max(0, end - Math.max(start, coveredUntil))
    coveredUntil = Math.max(coveredUntil, end)
  }
  return { count: intervals.length, sum, coverage }
}
