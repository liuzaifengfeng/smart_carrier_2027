import { ref } from 'vue'
import type { Controller } from './useController'
import { frame } from './commands'
export interface Parameter { index: number; name: string; value: string; min: number; max: number; integer: boolean }
export function useParameters(controller: Controller) {
  const parameters = ref<Parameter[]>([])
  const edits = ref<Record<number, string>>({})
  const status = ref('请读取小车参数')
  const busy = ref(false)
  let sequence = 0
  let received: Parameter[] = []
  let queue: { action: string; values: (string | number)[] }[] = []
  let expected: { action: string; values: (string | number)[] } | undefined
  let verifying = false
  let afterRead: (() => void) | undefined
  function read(verify = false) {
    sequence++
    busy.value = true
    verifying = verify
    received = []
    status.value = '正在读取参数'
    void controller.send(frame('CFG', 'GET', [sequence]))
  }
  function next() {
    expected = queue.shift()
    if (!expected) { read(true); return }
    void controller.send(frame('CFG', expected.action, [sequence, ...expected.values]))
  }
  function write(callback?: () => void) {
    for (const parameter of parameters.value) {
      if (!(parameter.index in edits.value)) continue
      const value = Number(edits.value[parameter.index])
      if (!Number.isFinite(value) || value < parameter.min || value > parameter.max || (parameter.integer && !Number.isInteger(value))) {
        status.value = parameter.name + ' 超出固件参数范围'
        return
      }
    }
    afterRead = callback
    busy.value = true
    sequence++
    queue = [{ action: 'BEGIN', values: [] },
      ...Object.entries(edits.value).map(([index, value]) => ({ action: 'SET', values: [index, value] })),
      { action: 'COMMIT', values: [] }]
    status.value = '正在写入，等待逐项确认'
    next()
  }
  controller.onFrame(fields => {
    if (!busy.value || fields[0] !== 'RSP' || fields[1] !== 'CFG' || Number(fields[4]) !== sequence) return
    const [, , action, result] = fields
    if (result === 'ERR') { status.value = fields.join(','); busy.value = false; afterRead = undefined; return }
    if (action === 'GET' && result === 'VALUE') {
      received.push({ index: Number(fields[5]), name: fields[6]!, value: fields[7]!,
        min: Number(fields[8]), max: Number(fields[9]), integer: fields[10] === '1' })
    }
    if (action === 'GET' && result === 'END') {
      busy.value = false
      if (received.length !== Number(fields[5]) || new Set(received.map(p => p.index)).size !== received.length
        || received.some(p => p.index < 0 || p.index >= received.length)) {
        status.value = '参数回读不完整，请重读'; afterRead = undefined; return
      }
      parameters.value = received
      if (verifying && Object.entries(edits.value).some(([index, value]) => {
        const actual = Number(received.find(p => p.index === Number(index))?.value)
        return !Number.isFinite(actual) || Math.abs(actual - Number(value)) > Math.max(1e-8, Math.abs(Number(value)) * 1e-6)
      })) {
        status.value = '回读与请求不一致，已保留编辑'; afterRead = undefined; return
      }
      edits.value = {}
      status.value = '已完整回读 ' + received.length + ' 项'
      afterRead?.()
      afterRead = undefined
    }
    if (result === 'OK' && action === expected?.action && (action !== 'SET' || fields[5] === String(expected.values[0]))) next()
  })
  function cancel() {
    sequence++; busy.value = false; queue = []; afterRead = undefined; status.value = '已取消当前参数操作，可重新读取'
  }
  return { parameters, edits, status, busy, read, write, cancel }
}
