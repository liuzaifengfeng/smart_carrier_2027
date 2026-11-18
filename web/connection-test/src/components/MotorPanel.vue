<script setup lang="ts">
import { computed, onUnmounted, ref, watch } from 'vue'
import type { Controller } from '../useController'
import { frame } from '../commands'
import { download } from '../python'
const props = defineProps<{ controller: Controller }>()
const motor = ref(7)
const pid = ref(['', '', ''])
const drive = ref(['', '', '', '', '', '', ''])
const save = ref(false)
const mode = ref(2)
const settings = ref({ a: 0, b: 90, speed: 250, ramp: 100, period: 20, duration: 3000, cycles: 2 })
const state = ref('尚未读取')
const active = ref(false)
const records = ref<{ target: number; start: number; pid: string[]; trace: string; result: string; samples: number[][] }[]>([])
const selected = ref(0)
const chartMode = ref('位置')
const tail = ref(false)
let pending: number[] = []
let phase = ''
let startMs = 0
let current: typeof records.value[number] | undefined
const labels = ['最大输出电压 mV', '开环工作电流 mA', '堵转最大电流 mA', '堵转检测转速 RPM', '堵转检测电流 mA', '堵转检测时间 ms', '到达窗口 °']
const command = (action: string, values: (number | string)[] = []) => props.controller.send(frame('EMM', action, [motor.value, ...values]))
function stop() {
  const wasActive = active.value
  active.value = false; pending = []; phase = ''
  if (wasActive && props.controller.connected.value) void props.controller.send('{CMD,NAV,STOP}')
}
function next() {
  const target = pending.shift()
  if (target === undefined) { active.value = false; state.value = '往返试验完成'; return }
  phase = 'command'
  const s = settings.value
  void props.controller.send(frame('EMM', motor.value === 7 ? 'TURRET_TRIAL' : 'AXIS_TRIAL',
    [...(motor.value === 7 ? [] : [motor.value]), target, s.speed, s.ramp, s.period, s.duration]))
}
function start() {
  const s = settings.value
  if (!Number.isInteger(s.cycles) || s.cycles < 1) { state.value = '往返次数须为正整数'; return }
  pending = [s.a, ...Array.from({ length: s.cycles }, () => [s.b, s.a]).flat()]
  active.value = true; phase = 'pid'; state.value = '等待 PID 实际回读'
  void command('PID_SET', [...pid.value, 0])
}
const remove = props.controller.onFrame(fields => {
  let f = fields
  if (f[0] === 'RSP' && f[1] === 'EMM') {
    if (f[3] === 'ERR') { state.value = f.join(','); stop(); return }
    if (f[4] === String(motor.value)) {
      if (f[2]?.startsWith('PID_')) {
        const actual = f.slice(5)
        const matches = actual.every((value, i) => Number(value) === Number(pid.value[i]))
        pid.value = actual
        if (active.value && phase === 'pid' && f[2] === 'PID_SET') {
          if (matches) next()
          else { state.value = 'PID 实际回读不一致'; stop() }
        }
      }
      if (f[2]?.startsWith('DRIVE_')) drive.value = f.slice(5).map((value, i) => i === 6 ? String(Number(value) / 10) : value)
      state.value = f.join(' / ')
    }
  }
  if (!active.value) return
  if (f[0] === 'EVT' && ((f[1] === 'TURRET' && motor.value !== 7) || (f[1] === 'AXIS' && motor.value === 7))) return
  if (f[0] === 'EVT' && f[1] === 'AXIS') {
    if (f[3] !== String(motor.value)) return
    f = [...f.slice(0, 3), ...f.slice(4)]
  }
  if (f[0] === 'EVT' && ['AXIS', 'TURRET'].includes(f[1]!)) {
    if (f[2] === 'COMMAND') {
      current = { target: Number(f[5]), start: Number(f[6]), pid: [...pid.value], trace: f[3]!, result: '', samples: [] }
      records.value.push(current); current = records.value.at(-1)!; selected.value = records.value.length - 1
      startMs = Number(f[4]); phase = 'samples'
    }
    if (f[2] === 'SAMPLE' && current?.trace === f[3]) current.samples.push([(Number(f[4]) - startMs) >>> 0, Number(f[6]), ...f.slice(7).map(Number)])
    if (f[2] === 'RESULT' && current?.trace === f[3]) current.result = f[5]!
  }
  if (f[0] === 'EVT' && f[1] === 'EMM' && f[2] === (motor.value === 7 ? 'TURRET_TRIAL' : 'AXIS_TRIAL')) {
    if (motor.value !== 7 && f[4] !== String(motor.value)) return
    if (f[3] === 'DONE' && current?.result === 'OBSERVED') next()
    else stop()
  }
  if (f[0] === 'RSP' && f[1] === 'SYS' && f[2] === 'RELEASE') stop()
})
watch(motor, () => { stop(); pid.value = ['', '', '']; drive.value = labels.map(() => ''); state.value = '请读取所选电机'; settings.value.a = motor.value === 5 ? 80 : 0; settings.value.b = motor.value === 7 ? 90 : motor.value === 5 ? 120 : 50 })
watch(() => props.controller.connected.value, connected => { if (!connected) stop() })
onUnmounted(() => { stop(); remove() })
const chart = computed(() => {
  const record = records.value[selected.value]
  if (!record?.samples.length) return ''
  const samples = tail.value ? record.samples.filter(s => s[0]! >= record.samples.at(-1)![0]! - 500) : record.samples
  const values = samples.map(s => chartMode.value === '转速' ? s[2]! : chartMode.value === '误差' ? s[1]! - record.target : s[1]!)
  const min = Math.min(...values); const span = Math.max(...values) - min || 1
  const first = samples[0]![0]!; const duration = samples.at(-1)![0]! - first || 1
  return samples.map((s, i) => (20 + (s[0]! - first) / duration * 460) + ',' + (180 - (values[i]! - min) / span * 150)).join(' ')
})
function metrics(record: typeof records.value[number]) {
  if (!record.samples.length) return '尚无采样'
  const sign = record.target >= record.start ? 1 : -1
  const over = Math.max(0, ...record.samples.map(s => sign * (s[1]! - record.target)))
  const tail = record.samples.filter(s => s[0]! >= record.samples.at(-1)![0]! - 500).map(s => s[1]!)
  const lastBad = record.samples.reduce((last, s, i) => Math.abs(s[1]! - record.target) > 1 ? i : last, -1)
  const stable = record.samples[lastBad + 1]?.[0]
  const intervals = record.samples.slice(1).map((s, i) => s[0]! - record.samples[i]![0]!)
  return '过冲 ' + over.toFixed(3) + ' · 稳定 ms ' + (stable !== undefined && record.samples.at(-1)![0]! - stable >= 500 ? stable : '未稳定')
    + ' · 末段峰峰 ' + (Math.max(...tail) - Math.min(...tail)).toFixed(3)
    + ' · 末误差 ' + (record.samples.at(-1)![1]! - record.target).toFixed(3)
    + ' · 平均间隔 ms ' + (intervals.length ? intervals.reduce((a,b) => a+b,0) / intervals.length : 0).toFixed(1)
}
function exportCsv() {
  const rows = ['record,target,pid,trace,result,elapsed_ms,position,rpm,flags,extra,encoder,position_counts,query_ms']
  records.value.forEach((r, i) => r.samples.forEach(s => rows.push([i, r.target, r.pid.join('/'), r.trace, r.result, ...s].join(','))))
  download('motor-trials.csv', rows.join('\n'), 'text/csv')
}
</script>
<template>
  <label>EMM 地址（1～4 底盘，5 升降，6 伸出，7 机械臂转台）<select v-model.number="motor" :disabled="active"><option v-for="i in 7" :key="i">{{ i }}</option></select></label>
  <fieldset><legend>驱动器位置环 PID</legend>
    <label v-for="(name, i) in ['Kp', 'Ki', 'Kd']" :key="name">{{ name }}<input v-model="pid[i]" :disabled="active"></label>
    <label><input v-model="save" type="checkbox">写入 Flash</label>
    <button :disabled="!controller.canControl.value || active" @click="command('PID_GET')">读取 PID</button>
    <button :disabled="!controller.canControl.value || active" @click="command('PID_SET', [...pid, Number(save)])">写入并回读</button>
  </fieldset>
  <fieldset><legend>驱动参数</legend>
    <label v-for="(name, i) in labels" :key="name">{{ name }}<input v-model="drive[i]"></label>
    <button :disabled="!controller.canControl.value || active" @click="command('DRIVE_GET')">读取驱动参数</button>
    <button :disabled="!controller.canControl.value || active" @click="command('DRIVE_SET', [...drive.map((value, i) => i === 6 ? Number(value) * 10 : value), Number(save)])">写入并回读</button>
  </fieldset>
  <fieldset><legend>状态与模式</legend>
    <label>脉冲输入模式<select v-model.number="mode"><option v-for="(name, i) in ['关闭', '开环', '闭环', '限位 / 到位']" :key="i" :value="i">{{ name }}</option></select></label>
    <button :disabled="!controller.canControl.value" @click="command('MODE_SET', [mode, Number(save)])">应用模式</button>
    <button :disabled="!controller.canControl.value" @click="command('STATE_GET')">读取状态</button>
    <button :disabled="!controller.canControl.value" @click="command('ENABLE', [1])">使能</button>
    <button :disabled="!controller.canControl.value" @click="command('ENABLE', [0])">失能</button>
    <button :disabled="!controller.canControl.value" @click="command('CLEAR_STALL')">解除堵转保护</button>
  </fieldset>
  <fieldset v-if="motor >= 5"><legend>半自动往返 PID 试验</legend>
    <label v-for="(label, key) in {a:'位置 A',b:'位置 B',speed: motor === 7 ? '速度 °/s' : '速度 RPM',ramp:'加减速 ms',period:'采样周期 ms',duration:'观察时长 ms',cycles:'往返次数'}" :key="key">{{ label }}<input v-model.number="settings[key]" type="number" :disabled="active"></label>
    <button :disabled="!controller.canControl.value || active" @click="start">写入 RAM 并开始试验</button>
    <button :disabled="!active" @click="stop">停止试验</button><button @click="exportCsv">导出 CSV</button>
    <label>曲线<select v-model="chartMode"><option>位置</option><option>误差</option><option>转速</option></select></label>
    <label><input v-model="tail" type="checkbox">末 0.5 秒</label>
    <svg viewBox="0 0 500 210" aria-label="电机实测曲线"><polyline :points="chart" fill="none" stroke="#2563eb" stroke-width="2" /></svg>
    <button v-for="(record, i) in records" :key="i" @click="selected = i">{{ record.pid.join('/') }} → {{ record.target }} · {{ metrics(record) }}</button>
  </fieldset>
  <p role="status">{{ state }}</p>
</template>
