<script setup lang="ts">
import { computed, ref, watch } from 'vue'
import { useController } from './useController'
import { useCloudCapture } from './cloudRuns'
import { usePwa } from './pwa'
import { useParameters } from './parameters'
import { frame } from './commands'
import CommandPanel from './components/CommandPanel.vue'
import ParametersPanel from './components/ParametersPanel.vue'
import MotorPanel from './components/MotorPanel.vue'
import TrayPanel from './components/TrayPanel.vue'
import TrialPanel from './components/TrialPanel.vue'
import KeyboardPanel from './components/KeyboardPanel.vue'
import FieldMap from './components/FieldMap.vue'
import BatteryPanel from './components/BatteryPanel.vue'
import './desktop.css'
const controller = useController()
const { status: archiveStatus, pending, upload } = useCloudCapture(controller)
const model = useParameters(controller)
const { connected, busy, error, device, logs, snapshots, handshake, canControl, connect, disconnect, hello, send, exportLogs } = controller
const { installPrompt, offlineStatus, waiting, updating, install, update } = usePwa()
const baud = ref(115200)
const supportsSerial = Boolean(navigator.serial)
const brightness = ref(0)
const frequency = ref(2000)
const command = ref('{CMD,POSE,GET}')
const page = ref('姿态')
const tabs = ['姿态', '底盘', 'EMM 电机', '旋转料盘', '机械臂', '键盘控制', '日志', '视觉', '参数', '动作组调试', '连续试跑', 'LED 调光']
const target = ref([600, 600, 0])
const targetVisible = ref(false)
const preview = ref<number[][]>([])
const pose = computed(() => (snapshots.value['POSE/GET'] ?? []).map(Number))
const display = ref<Record<string, string>>({})
controller.onFrame(f => {
  if (f[0] === 'RSP' && f[1] === 'SYS' && f[2] === 'REBOOT' && f[3] === 'ACK') void disconnect()
  if (f[0] === 'EVT' && f[1] === 'DISPLAY') display.value[f[2]!] = f.slice(3).join(' / ')
  if (f[0] === 'EVT' && f[1] === 'VISION') display.value.VISION = f.slice(2).join(' / ')
})
watch(connected, value => {
  if (!value) {
    model.cancel()
    model.parameters.value = []
    model.edits.value = {}
    preview.value = []
  }
})
const mapReady = computed(() => Array.from({ length: 25 }, (_, i) => ['X', 'Y'].every(axis =>
  model.parameters.value.some(p => p.name === `地图/节点${i}/${axis}(mm)`))).every(Boolean))
const nodes = computed(() => Array.from({ length: 25 }, (_, i) => {
  const row = Math.floor(i / 5); const col = row % 2 ? 4 - i % 5 : i % 5
  return [i, ...['X', 'Y'].map((axis, index) => Number(model.parameters.value.find(p => p.name === `地图/节点${i}/${axis}(mm)`)?.value
    ?? (index === 0 ? [240,720,1200,1680,2050][col] : 240 + row * 480)))]
}))
function gotoTarget() {
  const [x, y, heading] = pose.value
  if (x === undefined || y === undefined || heading === undefined) return
  targetVisible.value = true
  const radians = heading * Math.PI / 180
  const dx = target.value[0]! - x; const dy = target.value[1]! - y
  const angle = ((target.value[2]! - heading + 180) % 360 + 360) % 360 - 180
  void send(frame('POSE', 'GOTO_REL', [dx * Math.cos(radians) + dy * Math.sin(radians), -dx * Math.sin(radians) + dy * Math.cos(radians), angle]))
}
function pickTarget(x: number, y: number) {
  target.value[0] = x
  target.value[1] = y
  targetVisible.value = true
}
</script>
<template>
  <main class="desktop">
    <h1>智能搬运车调试上位机</h1>
    <fieldset class="connection"><legend>USB 串口连接</legend>
      <div class="actions">
        <label>波特率<input v-model.number="baud" type="number" :disabled="connected || busy"></label>
        <template v-if="!connected">
          <button v-if="supportsSerial" :disabled="busy || updating" @click="connect(baud, 'serial')">连接串口（COM）</button>
          <button :disabled="busy || updating" @click="connect(baud)">连接 USB 设备</button>
        </template>
        <button v-else @click="disconnect">断开</button>
        <button :disabled="!connected" @click="hello">重新握手</button>
        <span role="status">{{ handshake }} {{ device }}</span>
        <button :disabled="!canControl" @click="send('{CMD,SYS,RELEASE}')">切换到 Release</button>
        <button :disabled="!canControl" @click="send('{CMD,POSE,GET}')">查询小车 / 机械臂位姿</button>
        <button :disabled="!canControl" @click="send('{CMD,IMU,GET}')">查询陀螺仪姿态</button>
        <button :disabled="!canControl" @click="send('{CMD,IMU,HEADING}')">查询地图航向</button>
        <div class="connection-tools">
          <BatteryPanel :controller="controller" />
          <button :disabled="!canControl" @click="send('{CMD,SYS,REBOOT}')">重启主控</button>
        </div>
      </div>
      <div class="connection-status">
        <span role="status">云端归档 · {{ archiveStatus }}{{ pending ? `（待传 ${pending} 场）` : '' }}</span>
        <span><a href="./runs.html" target="_blank" rel="noopener">查看云端运行日志</a>
          <button v-if="pending" @click="upload">重试上传</button>
        </span>
        <span>任务码 {{ display.TASK_CODE ?? '尚未收到' }}</span>
        <span>调试信息 {{ display.DEBUG ?? '等待小车信息' }}</span>
        <span>视觉请求 {{ display.VISION ?? '尚无请求' }}</span>
        <span>H30 Pitch / Roll / Yaw / 年龄 ms {{ snapshots['IMU/GET']?.join(' / ') ?? '尚未查询' }}</span>
        <span>地图航向 / 年龄 ms {{ snapshots['IMU/HEADING']?.join(' / ') ?? '尚未查询' }}</span>
      </div>
      <p v-if="error" class="error">{{ error }}</p>
    </fieldset>
    <div class="desktop-columns">
      <section class="map-section">
        <FieldMap :pose="pose" :target="targetVisible ? target : []" :nodes="nodes" :preview="preview" @target="pickTarget" />
        <p>蓝色为理想位置，橙色为目标。点击场地可选取目标坐标。</p>
        <p>回读 X / Y / 航向 / 高度 / 伸出 / 转台 / 夹爪 {{ snapshots['POSE/GET']?.join(' / ') ?? '尚未回读' }}</p>
        <fieldset v-if="page === '机械臂'"><legend>机械臂姿态示意</legend>
          <svg viewBox="0 0 450 220" aria-label="机械臂侧视图">
            <path d="M40 190H410 M70 190V20" stroke="#59636e" fill="none" stroke-width="8" />
            <path :d="'M70 ' + (185-(pose[3]??0)*0.8) + 'H' + (170+(pose[4]??0))" stroke="#e5b529" fill="none" stroke-width="14" />
            <text x="100" y="210">高度 {{ pose[3] ?? '—' }} mm · 伸出 {{ pose[4] ?? '—' }} mm</text>
          </svg>
        </fieldset>
      </section>
      <section class="notebook">
        <nav aria-label="上位机功能"><button v-for="tab in tabs" :key="tab" :aria-pressed="page === tab" @click="page = tab">{{ tab }}</button></nav>
        <div class="tab-content">
          <fieldset v-if="page === '姿态'" class="map-target"><legend>地图目标预览</legend>
            <div class="actions"><label v-for="(name, i) in ['X mm','Y mm','航向 °']" :key="name">{{ name }}<input v-model.number="target[i]" type="number"></label></div>
            <button @click="targetVisible = true">在地图上预览目标</button>
            <button :disabled="!canControl || pose.length < 3" @click="gotoTarget">发送绝对目标姿态</button>
          </fieldset>
          <CommandPanel :page="page" :controller="controller" />
          <div v-show="page === 'EMM 电机'"><MotorPanel :controller="controller" /></div>
          <div v-show="page === '旋转料盘'"><TrayPanel :controller="controller" /></div>
          <KeyboardPanel v-if="page === '键盘控制'" :controller="controller" />
          <div v-show="page === '参数' || page === '动作组调试'"><ParametersPanel :controller="controller" :model="model" :actions="page === '动作组调试'" /></div>
          <div v-show="page === '连续试跑'"><TrialPanel :controller="controller" :nodes="nodes" :map-ready="mapReady" @preview="preview = $event" /></div>
          <fieldset v-if="page === 'LED 调光'"><legend>LED 亮度与频率 · GPIO5</legend>
            <label>亮度 %<input v-model.number="brightness" type="range" min="0" max="100">{{ brightness }} %</label>
            <button :disabled="!canControl" @click="send(frame('LED','SET',[brightness]))">应用亮度</button>
            <button :disabled="!canControl" @click="send('{CMD,LED,SET,0}')">关闭 LED</button>
            <button :disabled="!canControl" @click="send('{CMD,LED,GET}')">读取设置</button>
            <label>频率 Hz<input v-model.number="frequency" type="number"></label>
            <button :disabled="!canControl" @click="send(frame('LED','FREQ',[frequency]))">应用频率</button>
            <p>设备回读 · {{ snapshots['LED/GET']?.join(' % / ') ?? '尚未读取' }} Hz</p>
          </fieldset>
          <section v-show="page === '日志'">
            <form @submit.prevent="send(command)"><label>原始命令<input v-model="command"></label><button :disabled="!canControl">发送原始命令</button></form>
            <button @click="logs = []">清空日志</button><button @click="exportLogs">导出 JSON</button>
            <div class="console" role="log"><div v-for="(entry, i) in logs" :key="i"><time>{{ entry.time.slice(11,23) }} UTC</time> <b>{{ entry.direction }}</b><pre>{{ entry.text }}</pre></div></div>
          </section>
          <p class="last-response">最新消息 {{ logs.at(-1)?.text ?? '等待设备消息' }}</p>
        </div>
      </section>
    </div>
    <footer>
      <div><a href="./">连接兼容性测试</a> · 协议 ACK 表示接令，实际执行结果查看后续事件。</div>
      <div v-if="offlineStatus" class="actions">
        <span role="status">{{ offlineStatus }}</span>
        <button v-if="installPrompt" @click="install">安装应用</button>
        <button v-if="waiting" :disabled="connected || busy || updating" @click="update">
          {{ connected || busy ? '有新版本，断开 USB 后更新' : updating ? '正在更新' : '更新到新版本' }}
        </button>
      </div>
    </footer>
  </main>
</template>
