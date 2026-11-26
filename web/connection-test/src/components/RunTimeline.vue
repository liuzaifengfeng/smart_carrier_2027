<script setup lang="ts">
import { computed, ref } from 'vue'
import type { Interval, RunDetail } from '../cloudRuns'
import { activityName, date, lanes, phaseName, runStatus, runTiming, seconds } from '../runDisplay'

const props = defineProps<{ run: RunDetail; label: string; start: number; end: number; phase: string; steps: boolean }>()
const emit = defineEmits<{ focus: [phase: string] }>()
const selected = ref<Interval>()
const phase = computed(() => props.run.phases.find(([name]) => name === props.phase))
const scale = computed(() => props.end - props.start)
const observedEnd = computed(() => props.run.timeline_ms ?? props.run.total_ms ?? 0)
const isOpen = ([kind, name, , start]: Interval) => kind === 'PHASE' &&
  props.run.phases.some(row => row[0] === name && row[1] === start && row[2] === null)
const tracks = computed(() => {
  const stages: Interval[] = props.run.phases
    .filter(([name]) => !props.phase || name === props.phase)
    .map(([name, start, end]) => ['PHASE', name, '0', start, end ?? observedEnd.value])
  const tracks = [{ label: '业务阶段', rows: lanes(stages) }]
  if (!phase.value && props.run.phases.length) return tracks
  const [, start, end] = phase.value ?? ['', props.start, props.end]
  const intervals = props.run.intervals.filter(row => row[3] < (end ?? observedEnd.value) && row[4] > start)
  tracks.push({ label: '动作区间', rows: lanes(intervals.filter(([kind]) => kind === 'SPAN')) })
  if (props.steps || !props.run.phases.length) {
    tracks.push({ label: '作业', rows: lanes(intervals.filter(([kind]) => kind === 'RESULT')) })
    tracks.push({ label: '机械臂步骤', rows: lanes(intervals.filter(([kind]) => kind === 'STEP')) })
  }
  return tracks
})
function position(interval: Interval) {
  const start = Math.max(props.start, phase.value?.[1] ?? props.start, interval[3])
  const end = Math.min(props.end, phase.value?.[2] ?? props.end, interval[4])
  return { left: (start - props.start) / scale.value * 100, width: (end - start) / scale.value * 100 }
}
function name([kind, title, job]: Interval) {
  if (kind === 'PHASE') return phaseName(title)
  if (kind === 'RESULT') return `作业 #${job} · ${title}`
  return activityName(title)
}
function description(interval: Interval) {
  const [, , , start, end] = interval
  if (isOpen(interval)) {
    return `${name(interval)} · 从 ${seconds(start)} 开始 · 缺少结束记录`
  }
  return `${name(interval)} · ${seconds(start)} 至 ${seconds(end)} · 耗时 ${seconds(end - start)}`
}
</script>

<template>
  <section class="timeline-run" :class="{ baseline: label === 'A' }">
    <h3>{{ label }} · {{ date(run.captured_at) }} <strong>{{ runStatus(run) }} · {{ runTiming(run) }}</strong></h3>
    <div v-for="track in tracks" :key="track.label" class="timeline-row">
      <span>{{ track.label }}</span>
      <div class="timeline-grid">
        <div v-if="!track.rows.length" class="lane muted">无记录</div>
        <div v-for="(row, index) in track.rows" :key="index" class="lane">
          <button v-for="(interval, i) in row" :key="i" type="button" class="time-bar"
            :class="[interval[0].toLowerCase(), { selected: selected === interval, 'open-phase': isOpen(interval) }]"
            :style="{ left: position(interval).left + '%', width: position(interval).width + '%' }"
            :title="description(interval)" :aria-label="description(interval)"
            @click="interval[0] === 'PHASE' ? emit('focus', interval[1]) : selected = interval">
            <span v-if="interval[0] === 'PHASE' || position(interval).width >= 12" class="bar-label">{{ name(interval) }}</span>
          </button>
        </div>
      </div>
    </div>
    <p v-if="phase" class="inspection" aria-live="polite">{{ selected ? description(selected) : '点选动作查看起止和耗时' }}</p>
  </section>
</template>
