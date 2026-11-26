import { ref } from 'vue'
import type { Controller } from './useController'
import { frame } from './commands'
export interface Parameter { index: number; name: string; value: string; min: number; max: number; integer: boolean }
const ringNames = {
  RING_X: '圆环纠偏/X死区(px)',
  RING_Y: '圆环纠偏/Y死区(px)',
  RING_TIMEOUT: '圆环纠偏/超时(ms)',
} as const
type Selector = number | keyof typeof ringNames

export function useParameters(controller: Controller) {
  const parameters = ref<Parameter[]>([])
  const edits = ref<Record<number, string>>({})
  const status = ref('请读取小车参数')
  const busy = ref(false)
  const directoryLoaded = ref(false)
  let sequence = 0
  const received = new Map<number, Parameter>()
  let targets: Selector[] = []
  let selector: Selector | undefined
  let matched = false
  let retried = false
  let fullRead = false
  let queue: { action: string; values: (string | number)[] }[] = []
  let expected: { action: string; values: (string | number)[] } | undefined
  let verifying = false
  let preserveEdits = false
  let written: [string, string][] = []
  let afterRead: (() => void) | undefined
  function request(target?: Selector) {
    sequence++
    selector = target
    matched = false
    void controller.send(frame('CFG', 'GET', target === undefined ? [sequence] : [sequence, target]))
  }
  function startRead(verify: boolean, keepEdits: boolean, selectors: Selector[] = []) {
    busy.value = true
    verifying = verify
    preserveEdits = keepEdits
    received.clear()
    fullRead = !selectors.length
    targets = [...selectors]
    retried = false
    status.value = fullRead ? '正在读取参数' : '正在回读指定参数'
    request(targets.shift())
  }
  function read() { startRead(false, false) }
  function readDeadzones() { startRead(false, true, ['RING_X', 'RING_Y']) }
  function readRingTimeout() { startRead(false, true, ['RING_TIMEOUT']) }
  function finishRead() {
    busy.value = false
    const merged = new Map(fullRead ? [] : parameters.value.map(p => [p.index, p] as const))
    for (const p of received.values()) merged.set(p.index, p)
    parameters.value = [...merged.values()].sort((a, b) => a.index - b.index)
    if (fullRead) directoryLoaded.value = true
    if (verifying && written.some(([index, value]) => {
      const actual = Number(received.get(Number(index))?.value)
      return !Number.isFinite(actual) || Math.abs(actual - Number(value)) > Math.max(1e-8, Math.abs(Number(value)) * 1e-6)
    })) {
      status.value = '回读与请求不一致，已保留编辑'; afterRead = undefined; return
    }
    if (verifying) {
      for (const [index] of written) delete edits.value[Number(index)]
    } else if (!preserveEdits) edits.value = {}
    status.value = '已回读 ' + received.size + ' 项'
    afterRead?.()
    afterRead = undefined
  }
  function next() {
    expected = queue.shift()
    if (!expected) { startRead(true, true, written.map(([index]) => Number(index))); return }
    void controller.send(frame('CFG', expected.action, [sequence, ...expected.values]))
  }
  function write(callback?: () => void, indexes?: readonly number[]) {
    const changes: [string, string][] = []
    for (const parameter of parameters.value) {
      const text = edits.value[parameter.index]?.trim()
      if (!text || (indexes && !indexes.includes(parameter.index))) continue
      const value = Number(text)
      if (value === Number(parameter.value)) continue
      if (!Number.isFinite(value) || value < parameter.min || value > parameter.max || (parameter.integer && !Number.isInteger(value))) {
        status.value = parameter.name + ' 超出固件参数范围'
        return
      }
      changes.push([String(parameter.index), text])
    }
    if (!changes.length) {
      status.value = '没有需要发送的参数修改'
      callback?.()
      return
    }
    afterRead = callback
    written = changes
    busy.value = true
    sequence++
    queue = [{ action: 'BEGIN', values: [] },
      ...changes.map(([index, value]) => ({ action: 'SET', values: [index, value] })),
      { action: 'COMMIT', values: [] }]
    status.value = '正在写入，等待逐项确认'
    next()
  }
  controller.onFrame(fields => {
    if (!busy.value || fields[0] !== 'RSP' || fields[1] !== 'CFG' || Number(fields[4]) !== sequence) return
    const [, , action, result] = fields
    if (result === 'ERR') { status.value = fields.join(','); busy.value = false; afterRead = undefined; return }
    if (action === 'GET' && result === 'VALUE') {
      const [, , , , , rawIndex, name, value, minimum, maximum, integer] = fields
      const index = Number(rawIndex), min = Number(minimum), max = Number(maximum)
      if (fields.length !== 11 || !name || !value || !minimum || !maximum
        || !Number.isSafeInteger(index) || index < 0 || ![Number(value), min, max].every(Number.isFinite)
        || (integer !== '0' && integer !== '1')) return
      if (selector !== undefined && (typeof selector === 'number' ? index !== selector : name !== ringNames[selector])) return
      received.set(index, { index, name, value, min, max, integer: integer === '1' })
      matched = true
    }
    if (action === 'GET' && result === 'END') {
      const count = Number(fields[5])
      if (selector === undefined) {
        if (!Number.isSafeInteger(count) || count < 0) {
          status.value = '参数目录数量无效，请重读'; busy.value = false; afterRead = undefined; return
        }
        for (const index of received.keys()) if (index >= count) received.delete(index)
        targets = Array.from({ length: count }, (_, index) => index).filter(index => !received.has(index))
        if (targets.length) {
          status.value = '正在补读 ' + targets.length + ' 项缺失参数'
          request(targets.shift())
          return
        }
      } else if (!matched || count !== 1) {
        if (!retried) {
          retried = true
          request(selector)
          return
        }
        status.value = '参数补读不完整 ' + selector
        busy.value = false
        afterRead = undefined
        return
      }
      if (targets.length) {
        retried = false
        request(targets.shift())
        return
      }
      finishRead()
    }
    if (result === 'OK' && action === expected?.action && (action !== 'SET' || fields[5] === String(expected.values[0]))) next()
  })
  function cancel() {
    sequence++; busy.value = false; queue = []; afterRead = undefined; status.value = '已取消当前参数操作，可重新读取'
  }
  return { parameters, edits, status, busy, directoryLoaded, read, readDeadzones, readRingTimeout, write, cancel }
}
