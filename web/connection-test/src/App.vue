<script setup lang="ts">
import { computed, ref, shallowRef } from 'vue'
import { describeUsb, inspectGatt, openBluetooth, openSerial, openUsbSerial, parseServices, sendBluetooth, subscribe } from './transports'
import type { Link } from './transports'

const methods = [
  { id: 'serial', label: 'Web Serial', route: 'USB 虚拟串口', available: !!navigator.serial },
  { id: 'bluetooth', label: 'Web Bluetooth', route: 'BLE GATT 透传', available: !!navigator.bluetooth },
  { id: 'usb', label: 'Web USB', route: 'USB 串口收发', available: !!navigator.usb },
] as const
type Method = typeof methods[number]['id']
type Result = {
  connected: boolean
  roundTrips: number[]
  device?: unknown
  error?: string
}

const selection = ref<Method>('serial')
const method = computed(() => methods.find(item => item.id === selection.value)!)
const results = ref<Record<Method, Result>>({
  serial: { connected: false, roundTrips: [] },
  bluetooth: { connected: false, roundTrips: [] },
  usb: { connected: false, roundTrips: [] },
})
const current = computed(() => results.value[selection.value])
const active = ref(false)
const busy = ref(false)
const sending = ref(false)
const baud = ref(115200)
const serviceInput = ref('')
const gatt = shallowRef<Awaited<ReturnType<typeof inspectGatt>>>([])
const tx = ref('')
const rx = ref('')
const listening = ref(false)
const authorizedUsb = ref<ReturnType<typeof describeUsb>[]>([])
const command = ref('{CMD,LED,GET}')
const probeCommand = '{CMD,LED,GET}'
const pending = ref<number | null>(null)
const logs = ref<{ time: string; method: Method; direction: string; message: string }[]>([])
const decoder = new TextDecoder()
let receiveBuffer = ''
let link: Link | undefined
let release: (() => Promise<void>) | undefined
let stopListening: (() => Promise<void>) | undefined

const characteristics = computed(() => gatt.value.flatMap(service =>
  service.characteristics.map(characteristic => ({
    key: `${service.uuid}/${characteristic.uuid}`,
    uuid: characteristic.uuid,
    characteristic,
    properties: characteristic.properties,
  })),
))
const writable = computed(() => characteristics.value.filter(item =>
  item.properties.write || item.properties.writeWithoutResponse,
))
const notifiable = computed(() => characteristics.value.filter(item =>
  item.properties.notify || item.properties.indicate,
))
const platform = /Android/i.test(navigator.userAgent) ? 'Android'
  : /Windows/i.test(navigator.userAgent) ? 'Windows'
  : /Linux/i.test(navigator.userAgent) ? 'Linux'
  : /Macintosh/i.test(navigator.userAgent) ? 'macOS' : '其他系统'
const browser = (navigator.userAgent.match(/Edg(?:A|iOS)?\/[\d.]+/)
  ?? navigator.userAgent.match(/(?:Chrome|Firefox|Version)\/[\d.]+/))?.[0]
  ?.replace(/^EdgA?\//, 'Edge ').replace(/^Version\//, 'Safari ').replace('/', ' ') ?? '未知浏览器'
const environment = { platform, browser, secureContext: isSecureContext, userAgent: navigator.userAgent }
const canSend = computed(() => active.value && (selection.value === 'serial' || selection.value === 'usb'
  || (selection.value === 'bluetooth' && listening.value && !!tx.value)))
const recommendation = computed(() => {
  if (results.value.serial.roundTrips.length) return '优先使用 Web Serial，已收到固件查询回包。'
  if (results.value.usb.roundTrips.length) return 'Web USB 串口已收到固件查询回包。'
  if (results.value.bluetooth.roundTrips.length) return '可使用 Web Bluetooth，已收到固件查询回包。'
  if (methods[0].available) return '先测试 Web Serial。打开端口后，发送 LED 查询验证板端回包。'
  if (methods[1].available) return '可尝试 Web Bluetooth。连接后，用 GATT 通知接收固件查询回包。'
  if (methods[2].available) return '可尝试 Web USB 串口，连接后发送 LED 查询验证回包。'
  return '当前浏览器未提供这三项 API。可在同一设备的 Chromium 浏览器中复测。'
})

function log(direction: string, message: string) {
  logs.value.push({ time: new Date().toISOString(), method: selection.value, direction, message })
}

function reportError(error: unknown) {
  const message = error instanceof Error ? `${error.name}: ${error.message}` : String(error)
  current.value.error = message
  log('错误', message)
}

function receive(bytes: Uint8Array) {
  const chunk = decoder.decode(bytes, { stream: true })
  log('RX', chunk)
  const lines = (receiveBuffer + chunk).split('\n')
  receiveBuffer = lines.pop()!
  for (const line of lines) {
    if (pending.value !== null && /^\{RSP,LED,GET,OK,\d+,\d+\}$/.test(line.trim())) {
      const elapsed = Math.round((performance.now() - pending.value) * 10) / 10
      current.value.roundTrips.push(elapsed)
      pending.value = null
      log('RTT', `${elapsed} ms`)
    }
  }
}

async function disconnect() {
  busy.value = true
  pending.value = null
  try {
    await release?.()
  } catch (error) {
    reportError(error)
  } finally {
    active.value = false
    release = undefined
    link = undefined
    stopListening = undefined
    listening.value = false
    tx.value = ''
    rx.value = ''
    gatt.value = []
    busy.value = false
  }
}

function lost(error?: unknown) {
  if (error) reportError(error)
  if (!busy.value) {
    log('状态', '设备已断开')
    void disconnect()
  }
}

async function connect() {
  busy.value = true
  current.value.error = undefined
  receiveBuffer = ''
  decoder.decode()
  try {
    if (selection.value === 'serial') {
      const opened = await openSerial(baud.value, receive, lost)
      link = opened.link
      release = () => opened.link.close()
      current.value.roundTrips = []
      current.value.device = { ...opened.info, baudRate: baud.value, framing: '8N1' }
    } else if (selection.value === 'bluetooth') {
      const requested = parseServices(serviceInput.value)
      const { device, server, close } = await openBluetooth(requested, lost)
      release = close
      current.value.roundTrips = []
      current.value.device = { name: device.name, requested }
      current.value.connected = active.value = true
      const services = await inspectGatt(server, requested)
      if (!server.connected) {
        await disconnect()
        throw new Error('GATT 已断开。')
      }
      gatt.value = services
      current.value.device = {
        name: device.name, requested,
        services: services.map(service => ({
          uuid: service.uuid,
          characteristics: service.characteristics.map(({ uuid, properties: p }) => ({
            uuid, read: p.read, write: p.write, writeWithoutResponse: p.writeWithoutResponse,
            notify: p.notify, indicate: p.indicate,
          })),
        })),
      }
    } else {
      const opened = await openUsbSerial(baud.value, receive, lost)
      link = opened.link
      release = () => opened.link.close()
      current.value.roundTrips = []
      current.value.device = opened.info
    }
    current.value.connected = active.value = true
    log('状态', '设备已连接')
  } catch (error) {
    reportError(error)
  } finally {
    busy.value = false
  }
}

async function enumerateUsb() {
  try {
    authorizedUsb.value = (await navigator.usb.getDevices()).map(describeUsb)
    log('状态', `已授权 USB 设备 ${authorizedUsb.value.length} 台`)
  } catch (error) {
    reportError(error)
  }
}

async function toggleNotifications() {
  busy.value = true
  try {
    if (listening.value) {
      await stopListening?.()
      pending.value = null
      listening.value = false
    } else {
      const characteristic = notifiable.value.find(item => item.key === rx.value)!.characteristic
      stopListening = await subscribe(characteristic, receive)
      listening.value = true
    }
  } catch (error) {
    reportError(error)
  } finally {
    busy.value = false
    if (characteristics.value.some(item => !item.characteristic.service.device.gatt?.connected)) lost()
  }
}

async function send(message: string, probe = false) {
  sending.value = true
  const payload = message + '\n'
  try {
    log('TX', payload)
    const bytes = new TextEncoder().encode(payload)
    if (probe) pending.value = performance.now()
    if (selection.value === 'serial' || selection.value === 'usb') {
      await link!.send(bytes)
    } else {
      const characteristic = writable.value.find(item => item.key === tx.value)!.characteristic
      for (let offset = 0; offset < bytes.length; offset += 20) {
        await sendBluetooth(characteristic, bytes.slice(offset, offset + 20))
      }
    }
  } catch (error) {
    if (probe) pending.value = null
    reportError(error)
  } finally {
    sending.value = false
  }
}

function exportReport() {
  const report = {
    exportedAt: new Date().toISOString(), environment,
    methods: methods.map(({ id, available }) => ({ id, available, ...results.value[id] })),
    authorizedUsb: authorizedUsb.value, recommendation: recommendation.value, logs: logs.value,
  }
  const url = URL.createObjectURL(new Blob([JSON.stringify(report, null, 2)], { type: 'application/json' }))
  const download = document.createElement('a')
  download.href = url
  download.download = 'ch9143-connection-report.json'
  download.click()
  setTimeout(() => URL.revokeObjectURL(url), 0)
}
</script>

<template>
  <main>
    <header class="top">
      <span class="brand">SMART CARRIER <b>/</b> 2027</span>
      <button @click="exportReport">导出 JSON</button>
    </header>
    <div class="intro">
      <h1>CH9143 连接测试</h1>
      <p>在当前浏览器验证设备访问和 ESP32-S3 串口回包。</p>
      <div class="environment">
        <span>{{ platform }} · {{ browser }}</span>
        <span :class="{ passed: environment.secureContext }">{{ environment.secureContext ? '安全上下文' : '需要 HTTPS 或 localhost' }}</span>
      </div>
    </div>

    <nav aria-label="连接方式" class="methods">
      <button v-for="item in methods" :key="item.id" :aria-pressed="selection === item.id"
        :disabled="(active || busy || sending) && selection !== item.id" @click="selection = item.id">
        <strong>{{ item.label }}</strong>
        <span>{{ item.route }}</span>
        <span :class="{ passed: item.available }">{{ item.available ? 'API 可用' : 'API 不可用' }}</span>
      </button>
    </nav>

    <div class="grid">
      <section class="panel">
        <div class="heading">
          <h2>{{ method.label }}</h2>
          <span :class="{ passed: active }" role="status">{{ busy ? '处理中' : active ? '已连接' : '未连接' }}</span>
        </div>
        <div class="checks" aria-label="验证结果">
          <span :class="{ passed: method.available }">API · {{ method.available ? '可用' : '不可用' }}</span>
          <span :class="{ passed: current.connected }">设备 · {{ current.connected ? '连接通过' : '待验证' }}</span>
          <span :class="{ passed: current.roundTrips.length }">
            回包 · {{ pending !== null ? '等待中' : current.roundTrips.length ? '通过' : '待验证' }}
          </span>
        </div>
        <p v-if="!method.available">接口可用性以当前浏览器为准。Android 是否暴露 USB 串口取决于浏览器和系统驱动。</p>
        <label v-if="selection === 'serial' || selection === 'usb'">
          波特率（8N1）
          <input v-model.number="baud" type="number" :disabled="active || busy">
        </label>
        <template v-else-if="selection === 'bluetooth'">
          <label>
            GATT 服务 UUID
            <input v-model="serviceInput" :disabled="active || busy" placeholder="16 位或完整 UUID，可填多个">
          </label>
          <p>留空可以测试 GATT 连接。探测特征值时，请在选设备前填写服务 UUID。可用系统 BLE 工具读取 UUID。</p>
        </template>
        <p v-if="selection === 'usb'">选择 CDC 串口设备，连接后可发送命令并查看回包。</p>

        <div class="actions">
          <button v-if="!active" class="primary" :disabled="!method.available || busy" @click="connect">
            {{ selection === 'serial' ? '选择并打开串口' : selection === 'bluetooth' ? '查找并连接 BLE 设备' : '选择并打开 USB 设备' }}
          </button>
          <button v-else :disabled="busy || sending" @click="disconnect">断开连接</button>
          <button v-if="selection === 'usb'" :disabled="!method.available || busy" @click="enumerateUsb">枚举已授权设备</button>
        </div>
        <p v-if="current.error" class="error" role="status">{{ current.error }}</p>

        <template v-if="active && gatt.length">
          <label>发送特征值
            <select v-model="tx" :disabled="busy || sending">
              <option value="">选择可写特征值</option>
              <option v-for="item in writable" :key="item.key" :value="item.key">{{ item.key }}</option>
            </select>
          </label>
          <label>接收特征值
            <select v-model="rx" :disabled="busy || listening">
              <option value="">选择支持通知或指示的特征值</option>
              <option v-for="item in notifiable" :key="item.key" :value="item.key">{{ item.key }}</option>
            </select>
          </label>
          <button :disabled="busy || sending || !rx" @click="toggleNotifications">{{ listening ? '停止接收' : '订阅通知' }}</button>
        </template>

        <details v-if="current.device">
          <summary>设备与接口详情</summary>
          <pre>{{ JSON.stringify(current.device, null, 2) }}</pre>
        </details>
        <details v-if="selection === 'usb' && authorizedUsb.length" open>
          <summary>已授权 USB 设备（{{ authorizedUsb.length }}）</summary>
          <pre>{{ JSON.stringify(authorizedUsb, null, 2) }}</pre>
        </details>

        <template v-if="true">
          <div class="probe">
            <h3>固件回包测试</h3>
            <p>发送 <code>{CMD,LED,GET}</code> 读取 PWM 当前设置。收到匹配的应答后才记录往返延迟。</p>
            <div class="actions">
              <button class="primary" :disabled="!canSend || busy || sending || pending !== null" @click="send(probeCommand, true)">查询 LED</button>
              <button v-if="pending !== null" :disabled="sending" @click="pending = null">停止等待</button>
              <span role="status">
                {{ pending !== null ? '等待回包' : current.roundTrips.length ? `${current.roundTrips.at(-1)} ms · ${current.roundTrips.length} 次成功` : '尚无测量结果' }}
              </span>
            </div>
          </div>
          <form class="command" @submit.prevent="send(command)">
            <label>自定义命令（自动加 LF）<input v-model="command" spellcheck="false"></label>
            <button :disabled="!canSend || busy || sending || pending !== null || !command.trim()">发送</button>
          </form>
        </template>
      </section>

      <section class="panel log">
        <div class="heading"><h2>通信日志</h2><button :disabled="!logs.length" @click="logs = []">清空</button></div>
        <div class="console" role="log" aria-label="通信日志">
          <p v-if="!logs.length">连接后在此查看收发数据。</p>
          <div v-for="(entry, index) in logs" :key="index" class="entry">
            <div><time>{{ entry.time.slice(11, 23) }} UTC</time> <b>{{ entry.direction }}</b> <span>{{ entry.method }}</span></div>
            <pre>{{ entry.message }}</pre>
          </div>
        </div>
      </section>
    </div>
    <section class="conclusion">
      <h2>测试结论</h2>
      <p aria-live="polite">{{ recommendation }}</p>
    </section>
    <footer>设备信息和通信日志仅在当前页面处理。</footer>
  </main>
</template>
