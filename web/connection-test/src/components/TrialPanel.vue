<script setup lang="ts">
import { onUnmounted, ref, watch } from 'vue'
import type { Controller } from '../useController'
import { frame } from '../commands'
import { download, python } from '../python'
const props = defineProps<{ controller: Controller; nodes: number[][]; mapReady: boolean }>()
const emit = defineEmits<{ preview: [points: number[][]] }>()
const path = ref('11-12-17')
const config = ref({ start: 180, end: 270, rpm: 80, accel: 200, lateral_accel: 300, lateral_ppm: 13.3, forward_ppm: 13.6, kp: 2, omega: 45, correction: 20, policy: 2, outset: 60, margin: 20 })
const autoEnd = ref(true)
const active = ref(false)
const planning = ref(false)
const status = ref('先读取完整固件参数地图，再预览试跑')
const samples = ref<string[][]>([])
let commands: string[] = []
let job = 0
let expected: string[] = []
let timer: ReturnType<typeof setInterval> | undefined
let ackTimer: ReturnType<typeof setTimeout> | undefined
let lastStatus = 0
function clear() {
  clearInterval(timer); clearTimeout(ackTimer); active.value = false; commands = []
}
function stop() {
  clear()
  void props.controller.send(frame('NAV', 'TRIAL_STOP', [job]))
  status.value = '已请求停车，等待终态'
}
async function preview() {
  planning.value = true
  emit('preview', [])
  try {
    if (!props.mapReady) throw new Error('请完整读取固件参数地图')
    const py = await python()
    job = Math.floor(Math.random() * 9999999) + 1
    py.globals.set('plan_input', JSON.stringify({
      nodes: path.value.split(/[-,，\s]+/).map(Number),
      positions: Object.fromEntries(props.nodes.map(([i, x, y]) => [String(i), [x, y]])),
      config: config.value, auto: autoEnd.value, job,
    }))
    const result = JSON.parse(await py.runPythonAsync(`from hybrid_plan import HybridPlan
from continuous_trial import TrialConfig
data = json.loads(plan_input)
plan = HybridPlan(data['nodes'], {int(k): tuple(v) for k,v in data['positions'].items()}, TrialConfig(**data['config']), auto_end=data['auto'])
def build_frame(*parts):
    return '{' + ','.join(map(str, parts)) + '}'
json.dumps({'commands': plan.commands(data['job'], build_frame), 'preview': plan.preview, 'length': plan.length})
`))
    emit('preview', result.preview)
    commands = result.commands
    status.value = '预览通过 · ' + result.length.toFixed(1) + ' mm'
    return true
  } catch (error) { commands = []; status.value = String(error); return false }
  finally { planning.value = false }
}
function next() {
  const command = commands.shift()
  if (!command) return
  expected = command.slice(1, -1).split(',')
  void props.controller.send(command)
  clearTimeout(ackTimer)
  ackTimer = setTimeout(() => { stop(); status.value = '上传应答超时，已请求停止' }, 2000)
}
async function start() {
  if (!await preview()) return
  active.value = true; samples.value = []
  next()
}
function calibrate() {
  const node = props.nodes.find(n => n[0] === Number(path.value.split(/[-,]/)[0]))
  if (node) void props.controller.send(frame('POSE', 'SET', [node[1]!, node[2]!, config.value.start]))
}
const remove = props.controller.onFrame(f => {
  if (f[1] !== 'NAV') return
  if (f[2] === 'TRIAL_RESULT' && Number(f[3]) === job) {
    clear(); status.value = f.join(' / ')
  }
  if (f[2] === 'TRIAL_SAMPLE' && Number(f[3]) === job) samples.value.push(f)
  if (!active.value || f[0] !== 'RSP') return
  if (f[3] === 'ERR' && f[2]?.startsWith('TRIAL_')) { status.value = f.join(' / '); clear(); return }
  if (f[2] === 'TRIAL_STATUS' && Number(f[4]) === job) {
    lastStatus = performance.now(); status.value = f.join(' / ')
    if (['DONE', 'FAILED', 'IDLE'].includes(f[5]!)) clear()
  }
  if (f[2] === expected[2] && f[3] === 'ACK' && Number(f[4]) === job
    && (f[2] !== 'TRIAL_NODE' || f[5] === expected[4])) {
    clearTimeout(ackTimer)
    if (f[2] === 'TRIAL_START') {
      lastStatus = performance.now()
      timer = setInterval(() => {
        if (performance.now() - lastStatus > 2000) { stop(); status.value = '状态超过 2 秒未更新，已请求停止'; return }
        void props.controller.send(frame('NAV', 'TRIAL_STATUS', [job]))
      }, 500)
    } else next()
  }
})
watch(() => props.controller.connected.value, connected => { if (!connected) clear() })
onUnmounted(() => { if (active.value) stop(); remove(); clear() })
</script>
<template>
  <fieldset :disabled="active || planning">
    <legend>独立连续试跑</legend>
    <label>节点路径<input v-model="path"></label>
    <label>策略<select v-model.number="config.policy"><option :value="1">自动混合</option><option :value="2">全程停转</option></select></label>
    <label>起始地图航向<input v-model.number="config.start" type="number"></label>
    <label><input v-model="autoEnd" type="checkbox">沿末段自动确定终点朝向</label>
    <label v-if="!autoEnd">终点航向<input v-model.number="config.end" type="number"></label>
    <label>轮速 RPM<input v-model.number="config.rpm" type="number"></label>
    <details><summary>高级参数</summary>
      <label v-for="(name, key) in {accel:'平移加速度',lateral_accel:'横向加速度',lateral_ppm:'横移脉冲/mm',forward_ppm:'前进脉冲/mm',kp:'航向 Kp',omega:'总角速度',correction:'纠偏角速度',outset:'外绕量 mm',margin:'余量 mm'}" :key="key">{{ name }}<input v-model.number="config[key]" type="number"></label>
    </details>
    <button :disabled="!controller.canControl.value || !mapReady" @click="calibrate">已摆到首节点，校准起点</button>
    <button @click="preview">预览路径</button>
    <button :disabled="!controller.canControl.value" @click="start">上传并开始试跑</button>
  </fieldset>
  <button :disabled="!active" @click="stop">停车 / 取消</button>
  <button @click="download('trial-samples.csv', 'kind,category,event,id,s,total,reference,heading,error,rpm1,rpm2,rpm3,rpm4,cycle_ms\n' + samples.map(row => row.join(',')).join('\n'), 'text/csv')">导出采样 CSV</button>
  <p role="status">{{ planning ? '正在加载并计算路径' : status }}</p>
</template>
