<script setup lang="ts">
import { computed, ref } from 'vue'
import type { Controller } from '../useController'
import type { useParameters } from '../parameters'
import { download, python } from '../python'
import { frame } from '../commands'
const props = defineProps<{ controller: Controller; model: ReturnType<typeof useParameters>; actions: boolean }>()
const search = ref('')
const group = ref('')
const source = ref('')
const note = ref('')
const stage = ref('全部参数')
const stageNames = ['全部参数','开局与扫码','圆盘抓取（两轮）','粗加工（两轮）','暂存第一层','暂存第二层','返航归位','机械臂公共参数','底盘导航与标定']
const stageIndexes = ref<number[] | null>(null)
const testAction = ref('试选中机械臂轴')
const selectedName = ref('')
const color = ref(1)
const cargo = ref(1)
const move = ref(['0','0','0','80'])
async function filterStage() {
  try {
    const py = await python()
    py.globals.set('filter_data', JSON.stringify({ stage: stage.value, parameters: props.model.parameters.value }))
    stageIndexes.value = JSON.parse(py.runPython("from desktop_helpers import parameter_matches_stage\ndata = json.loads(filter_data)\njson.dumps([p['index'] for p in data['parameters'] if parameter_matches_stage(p['name'], data['stage'])])"))
  } catch (error) { note.value = String(error) }
}
async function test() {
  note.value = ''
  try {
    const py = await python()
    py.globals.set('test_data', JSON.stringify({ action: testAction.value, stage: stage.value,
      selected: selectedName.value, color: color.value, cargo: cargo.value, move: move.value,
      values: Object.fromEntries(props.model.parameters.value.map(p => [p.name, Number(props.model.edits.value[p.index] ?? p.value)])) }))
    const line = py.runPython("from desktop_helpers import build_parameter_test_command\ndata = json.loads(test_data)\nbuild_parameter_test_command(data['action'], data['stage'], data['selected'], data['values'], color=data['color'], cargo=data['cargo'], move=data['move'])") as string
    props.model.write(() => { void props.controller.send(line) })
  } catch (error) { note.value = String(error) }
}
const groups = computed(() => [...new Set(props.model.parameters.value.filter(p => p.name.startsWith('动作组延时/')).map(p => p.name.split('/').slice(1, -1).join('/')))])
const visible = computed(() => props.model.parameters.value.filter(p => {
  if (props.actions && !p.name.startsWith('动作组延时/' + (group.value ? group.value + '/' : ''))) return false
  if (!props.actions && stage.value !== '全部参数' && stageIndexes.value && !stageIndexes.value.includes(p.index)) return false
  return p.name.includes(search.value)
}))
function exportConfig() {
  const groups: Record<string, { parameters: Record<string, unknown> }> = {}
  for (const p of props.model.parameters.value) {
    if (p.name.startsWith('地图/') || p.name.startsWith('动作组延时/')) continue
    const category = p.name.split('/')[0]!
    groups[category] ??= { parameters: {} }
    const value = Number(props.model.edits.value[p.index] ?? p.value)
    groups[category].parameters[p.name] = { value: Number.isFinite(value) ? value : null, minimum: p.min, maximum: p.max, integer: p.integer }
  }
  download('robot_parameters.json', JSON.stringify({ schema_version: 1, groups }, null, 2), 'application/json')
}
async function importFile(event: Event) {
  const file = (event.target as HTMLInputElement).files?.[0]
  if (!file) return
  try {
    const data = JSON.parse(await file.text())
    if (data.schema_version !== 1) throw new Error('参数版本不支持')
    const edits: Record<number, string> = {}
    const hasTrayPose = props.model.parameters.value.some(p => p.name === '搬运/待操作区/高度(mm)')
    for (const group of Object.values(data.groups) as { parameters: Record<string, { value: number | null }> }[]) {
      for (const [name, entry] of Object.entries(group.parameters)) {
        if (name.startsWith('地图/') || entry.value === null) continue
        if (hasTrayPose && /^搬运\/载物台[1-3]\/(高度\(mm\)|伸出\(mm\)|转台\(deg\))$/.test(name)) continue
        const parameter = props.model.parameters.value.find(p => p.name === name)
        if (!parameter) throw new Error('固件缺少参数 ' + name)
        edits[parameter.index] = String(entry.value)
      }
    }
    Object.assign(props.model.edits.value, edits)
    note.value = '已导入本地编辑，发送后回读确认'
  } catch (error) { note.value = String(error) }
}
async function readSource(event: Event) {
  const file = (event.target as HTMLInputElement).files?.[0]
  if (file) source.value = await file.text()
}
async function saveSource() {
  try {
    const py = await python()
    const values = Object.fromEntries(visible.value.map(p => [p.name, Number(props.model.edits.value[p.index] ?? p.value)]))
    py.FS.writeFile('/carrier/material_transfer.cpp', source.value)
    py.globals.set('delay_json', JSON.stringify(values))
    py.runPython("from action_group_config import save_action_group_delays\nsave_action_group_delays(json.loads(delay_json), '/carrier/material_transfer.cpp')")
    download('material_transfer.cpp', py.FS.readFile('/carrier/material_transfer.cpp', { encoding: 'utf8' }))
    note.value = '已生成更新后的 C++ 文件，替换工程文件后重新编译烧录'
  } catch (error) { note.value = String(error) }
}
function trial() {
  const forward = group.value.match(/^载物台(\d)到工作区(\d)$/)
  const reverse = group.value.match(/^工作区(\d)到载物台(\d)$/)
  const match = forward ?? reverse
  if (!match) return
  props.model.write(() => { void props.controller.send(frame('ARM', forward ? 'TRANSFER' : 'RETRIEVE', [match[1]!, match[2]!])) })
}
</script>
<template>
  <div class="actions">
    <button :disabled="!controller.canControl.value || model.busy.value" @click="model.read()">读取小车参数</button>
    <button :disabled="!controller.canControl.value || model.busy.value || !model.parameters.value.length" @click="model.write()">发送修改并回读</button>
    <button :disabled="model.busy.value" @click="model.edits.value = {}">撤销编辑</button>
    <button v-if="model.busy.value" @click="model.cancel">取消等待</button>
  </div>
  <p role="status">{{ model.status.value }} {{ note }}</p>
  <template v-if="actions">
    <label>动作组<select v-model="group"><option value="">全部</option><option v-for="name in groups" :key="name">{{ name }}</option></select></label>
    <button :disabled="!controller.canControl.value || model.busy.value || !/到/.test(group)" @click="trial">应用并试跑当前组</button>
    <label>读取工程 material_transfer.cpp<input type="file" accept=".cpp" @change="readSource"></label>
    <button :disabled="!source || !visible.length || model.busy.value || Object.keys(model.edits.value).length > 0" @click="saveSource">下载当前组更新后的 C++</button>
    <p>修改发送并回读确认后，可下载更新的源码文件。</p>
  </template>
  <template v-else>
    <label>比赛阶段<select v-model="stage" @change="filterStage"><option v-for="name in stageNames" :key="name">{{ name }}</option></select></label>
    <label>导入 JSON<input type="file" accept=".json" :disabled="!model.directoryLoaded.value || model.busy.value" @change="importFile"></label>
    <button :disabled="!model.directoryLoaded.value" @click="exportConfig">导出烧录默认参数 JSON</button>
    <button :disabled="!controller.canControl.value || model.busy.value || !model.parameters.value.length" @click="model.write(() => controller.send('{CMD,SYS,RELEASE}'))">发送并进入主流程</button>
    <fieldset><legend>当前阶段试动作</legend>
      <label>试动作<select v-model="testAction"><option v-for="name in ['试选中机械臂轴','圆盘抓取一件','三件放到工作区','三件码垛第二层','三件从工作区取回','开启视觉定位','底盘小距离移动','开局机械臂姿态']" :key="name">{{ name }}</option></select></label>
      <label v-if="testAction === '试选中机械臂轴'">位置参数<select v-model="selectedName"><option v-for="p in visible" :key="p.index" :value="p.name">{{ p.name }}</option></select></label>
      <template v-if="testAction === '圆盘抓取一件'"><label>颜色<input v-model.number="color" type="number"></label><label>载物台<input v-model.number="cargo" type="number"></label></template>
      <template v-if="testAction === '底盘小距离移动'"><label v-for="(name,i) in ['X mm','Y mm','转角 °','轮速 RPM']" :key="name">{{ name }}<input v-model="move[i]"></label></template>
      <button :disabled="!controller.canControl.value || model.busy.value || !model.parameters.value.length" @click="test">应用参数并试动作</button>
      <button :disabled="!controller.canControl.value" @click="controller.send('{CMD,VISION,ALIGN_STOP}')">停止视觉对齐</button>
    </fieldset>
  </template>
  <label>搜索参数<input v-model="search"></label>
  <div class="table-wrap"><table>
    <thead><tr><th>名称</th><th>设备回读</th><th>新值</th></tr></thead>
    <tbody><tr v-for="p in visible" :key="p.index">
      <td>{{ p.name }}<small>{{ p.min }} ～ {{ p.max }} {{ p.integer ? '整数' : '' }}</small></td>
      <td>{{ p.value }}</td>
      <td><input v-model="model.edits.value[p.index]" :placeholder="p.value" :disabled="model.busy.value" :aria-label="p.name"></td>
    </tr></tbody>
  </table></div>
</template>
