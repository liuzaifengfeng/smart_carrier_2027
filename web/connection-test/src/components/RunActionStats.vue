<script setup lang="ts">
import { computed } from 'vue'
import type { RunDetail } from '../cloudRuns'
import { actionStats, activityName, delta, duration } from '../runDisplay'

const props = defineProps<{ runs: RunDetail[] }>()
const groups = [
  ['底盘平移', 'CHASSIS_TRANSLATION'],
  ['底盘转向', 'CHASSIS_TURN'],
  ['视觉会话', 'VISION_WAIT_'],
  ['等待视觉首帧', 'VISION_FIRST_FRAME_'],
  ['机械臂程序', 'ARM_PROGRAM'],
  ['等待路径', 'PATH_WAIT'],
] as const

const summary = computed(() => groups.map(([label, code]) => ({
  label,
  stats: props.runs.map(run => actionStats(run, 'SPAN', name => code.endsWith('_') ? name.startsWith(code) : name === code)),
})))

const sections = computed(() => [
  { label: '视觉分项', kind: 'SPAN', matches: (name: string) => name.startsWith('VISION_') },
  { label: '机械臂步骤', kind: 'STEP', matches: () => true },
  { label: '其他动作区间', kind: 'SPAN', matches: (name: string) =>
    !name.startsWith('VISION_') && !groups.some(([, code]) => name === code) },
].map(({ label, kind, matches }) => {
  const names = new Set(props.runs.flatMap(run => run.intervals
    .filter(row => row[0] === kind && matches(row[1])).map(row => row[1])))
  const rows = Array.from(names, name => ({
    label: activityName(name),
    stats: props.runs.map(run => actionStats(run, kind, value => value === name)),
  }))
  return { label, rows: rows.sort((a, b) => (b.stats[0]?.coverage ?? b.stats[1]?.coverage ?? 0)
    - (a.stats[0]?.coverage ?? a.stats[1]?.coverage ?? 0)) }
}))
</script>

<template>
  <section class="action-stats">
    <h3>整场动作统计</h3>
    <p class="stats-note">覆盖时间会合并同类动作的重叠区间。视觉会话包含首帧等待，各行可能重叠。平移和转向按底盘已记录的区间统计。</p>
    <div class="table-scroll">
      <table class="stats-table">
        <thead>
          <tr><th scope="col" rowspan="2">动作类别</th><th scope="colgroup" colspan="3">场次 A</th><th v-if="runs.length > 1" scope="colgroup" colspan="3">场次 B</th><th v-if="runs.length > 1" scope="col" rowspan="2">覆盖 B − A</th></tr>
          <tr><template v-for="run in runs" :key="run.id"><th scope="col">次数</th><th scope="col">累计</th><th scope="col">覆盖</th></template></tr>
        </thead>
        <tbody>
          <tr v-for="row in summary" :key="row.label">
            <th scope="row">{{ row.label }}</th>
            <template v-for="(stat, i) in row.stats" :key="i"><td>{{ stat?.count ?? '缺数据' }}</td><td>{{ duration(stat?.sum) }}</td><td>{{ duration(stat?.coverage) }}</td></template>
            <td v-if="runs.length > 1">{{ delta(row.stats[0]?.coverage, row.stats[1]?.coverage) }}</td>
          </tr>
        </tbody>
      </table>
    </div>
    <details v-for="section in sections" v-show="section.rows.length" :key="section.label" class="stats-details">
      <summary>{{ section.label }}（{{ section.rows.length }} 类）</summary>
      <div class="table-scroll">
        <table :aria-label="section.label">
          <thead>
            <tr><th scope="col">动作</th><th scope="col">A 次数 / 累计 / 覆盖</th><th v-if="runs.length > 1" scope="col">B 次数 / 累计 / 覆盖</th><th v-if="runs.length > 1" scope="col">覆盖 B − A</th></tr>
          </thead>
          <tbody><tr v-for="row in section.rows" :key="row.label">
            <th scope="row">{{ row.label }}</th>
            <td v-for="(stat, i) in row.stats" :key="i">{{ stat ? `${stat.count} / ${duration(stat.sum)} / ${duration(stat.coverage)}` : '缺数据' }}</td>
            <td v-if="runs.length > 1">{{ delta(row.stats[0]?.coverage, row.stats[1]?.coverage) }}</td>
          </tr></tbody>
        </table>
      </div>
    </details>
  </section>
</template>
