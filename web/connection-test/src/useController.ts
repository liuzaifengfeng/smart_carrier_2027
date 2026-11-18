import { computed, ref } from 'vue'
import { openSerial, openUsbSerial, type Link } from './transports'

export function useController() {
  const connected = ref(false)
  const ready = ref(false)
  const busy = ref(false)
  const error = ref('')
  const device = ref('')
  const logs = ref<{ time: string; direction: string; text: string }[]>([])
  const snapshots = ref<Record<string, string[]>>({})
  const handshake = ref('未连接')
  let link: Link | undefined
  let decoder = new TextDecoder()
  let buffer = ''
  let writes = Promise.resolve()
  let session = 0
  const listeners = new Set<(fields: string[]) => void>()
  const disconnectListeners = new Set<() => void>()
  const lastFrame = ref<string[]>([])
  function onFrame(callback: (fields: string[]) => void) {
    listeners.add(callback)
    return () => listeners.delete(callback)
  }

  function log(direction: string, text: string) {
    logs.value.push({ time: new Date().toISOString(), direction, text })
  }
  function failure(reason: unknown) {
    error.value = String(reason)
    log('错误', error.value)
  }
  async function disconnect() {
    session++
    for (const listener of disconnectListeners) listener()
    const previous = link
    link = undefined
    connected.value = ready.value = false
    handshake.value = '未连接'
    try { await previous?.close() } catch (reason) { failure(reason) }
  }
  function receive(bytes: Uint8Array) {
    buffer += decoder.decode(bytes, { stream: true })
    const lines = buffer.split('\n')
    buffer = lines.pop()!
    for (const raw of lines) {
      const line = raw.trim()
      if (!line) continue
      log('RX', line)
      if (!line.startsWith('{') || !line.endsWith('}')) continue
      const fields = line.slice(1, -1).split(',')
      lastFrame.value = fields
      for (const listener of listeners) listener(fields)
      const [kind, category, action, status, ...values] = fields
      if (kind === 'RSP' && category === 'SYS' && action === 'HELLO') {
        ready.value = status === 'OK' && values[0] === '2'
        handshake.value = ready.value ? '协议 v2 已确认' : '握手未通过，请检查固件版本'
      }
      if (kind === 'RSP' && ['OK', 'READBACK', 'READBACK_MISMATCH'].includes(status)) {
        snapshots.value[category + '/' + action] = values
        if (category === 'LED') snapshots.value['LED/GET'] = values
      }
      if (kind === 'RSP' && status === 'ERR' && category !== 'BAT') error.value = line
      if (kind === 'EVT' && category === 'NAV' && action === 'TRIAL_RESULT' && fields.length === 9) {
        snapshots.value['POSE/GET'] = [...fields.slice(6), ...(snapshots.value['POSE/GET']?.slice(3) ?? ['0','0','0','0'])]
      }
    }
  }
  async function send(command: string) {
    const currentLink = link
    if (!currentLink) return
    const ticket = session
    const task = writes.then(async () => {
      if (session !== ticket) return
      log('TX', command)
      await currentLink.send(new TextEncoder().encode(command + '\n'))
    })
    writes = task.catch(failure)
    await writes
  }
  async function hello() {
    ready.value = false
    handshake.value = '等待协议 v2 应答'
    await send('{CMD,SYS,HELLO}')
  }
  async function connect(baudRate: number, transport: 'usb' | 'serial' = 'usb') {
    busy.value = true
    error.value = ''
    snapshots.value = {}
    buffer = ''
    decoder = new TextDecoder()
    const ticket = ++session
    try {
      const open = transport === 'serial' ? openSerial : openUsbSerial
      const opened = await open(baudRate, bytes => {
        if (ticket === session) receive(bytes)
      }, reason => {
        if (ticket !== session) return
        if (reason) failure(reason)
        void disconnect()
      })
      if (ticket !== session) { await opened.link.close(); return }
      link = opened.link
      writes = Promise.resolve()
      device.value = 'product' in opened.info ? opened.info.product ?? 'CDC 串口设备' : '系统串口（Web Serial）'
      connected.value = true
      await hello()
    } catch (reason) { failure(reason) }
    finally { busy.value = false }
  }
  function exportLogs() {
    const url = URL.createObjectURL(new Blob([JSON.stringify({
      device: device.value, handshake: handshake.value, snapshots: snapshots.value, logs: logs.value,
    }, null, 2)], { type: 'application/json' }))
    const anchor = document.createElement('a')
    anchor.href = url
    anchor.download = 'carrier-console-log.json'
    anchor.click()
    setTimeout(() => URL.revokeObjectURL(url), 0)
  }
  return { connected, ready, busy, error, device, logs, snapshots, handshake,
    canControl: computed(() => connected.value && ready.value && !busy.value),
    connect, disconnect, hello, send, exportLogs, onFrame, lastFrame,
    onDisconnect(callback: () => void) {
      disconnectListeners.add(callback)
      return () => disconnectListeners.delete(callback)
    } }
}

export type Controller = ReturnType<typeof useController>
