import { onMounted, onUnmounted, ref } from 'vue'
import type { Controller } from './useController'

export interface PendingRun {
  id: string
  captured_at: string
  frames: string[]
}

export type Phase = [name: string, start: number, end: number, duration: number]
export type Interval = [kind: string, name: string, job: string, start: number, end: number]

export interface RunOutline {
  id: string
  captured_at: string
  total_ms: number
  phases: Phase[]
  missing: string[]
  note?: string
}

export interface RunDetail extends RunOutline {
  intervals: Interval[]
  components: [name: string, count: number, sum: number, coverage: number][]
  job_sum_ms: number
  job_wall_ms: number
  frames: string[]
}

export class CloudError extends Error {
  constructor(public status: number, message: string) {
    super(message)
  }
}

const eventFields: Record<string, number> = {
  RUN_START: 5, RUN_RESULT: 7, PHASE: 6, FLAG: 7,
  SPAN: 7, STEP: 8, RESULT: 7, ERROR: 6,
}

export async function prepareRunUpload(run: PendingRun): Promise<PendingRun> {
  const frames = run.frames.filter(line => {
    if (!/^\{EVT,MOTION,[^{}\r\n]+\}$/.test(line)) return false
    const fields = line.slice(1, -1).split(',')
    return fields.length === eventFields[fields[2]!] &&
      /^\d+$/.test(fields[3]!) && /^\d+$/.test(fields[4]!)
  })
  if (frames[0] !== run.frames[0] || frames.at(-1) !== run.frames.at(-1) ||
      !frames[0]?.startsWith('{EVT,MOTION,RUN_START,0,') ||
      !frames.at(-1)?.startsWith('{EVT,MOTION,RUN_RESULT,0,')) {
    throw new Error('正式运行起止记录不完整')
  }
  // 同一份有效帧在手工补录、自动上传和重传时使用相同的归档编号。
  const hash = new Uint8Array(await crypto.subtle.digest('SHA-256',
    new TextEncoder().encode(JSON.stringify(frames))))
  hash[6] = (hash[6]! & 0x0f) | 0x50
  hash[8] = (hash[8]! & 0x3f) | 0x80
  const hex = Array.from(hash.subarray(0, 16), byte => byte.toString(16).padStart(2, '0')).join('')
  const id = [hex.slice(0, 8), hex.slice(8, 12), hex.slice(12, 16), hex.slice(16, 20), hex.slice(20)].join('-')
  return { ...run, id, frames }
}

export async function cloudRequest<T>(path: string, body?: object): Promise<T> {
  const response = await fetch('/api/' + path, {
    cache: 'no-store',
    ...(body && { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }),
  })
  if (!response.ok) {
    const result = await response.json().catch(() => ({}))
    throw new CloudError(response.status, result.error ?? `云端请求失败（${response.status}）`)
  }
  return response.json()
}

function openQueue(): Promise<IDBDatabase> {
  return new Promise((resolve, reject) => {
    const request = indexedDB.open('carrier-cloud-runs', 1)
    request.onupgradeneeded = () => request.result.createObjectStore('pending', { keyPath: 'id' })
    request.onsuccess = () => resolve(request.result)
    request.onerror = () => reject(request.error)
  })
}

async function queueOperation<T>(mode: IDBTransactionMode, operation: (store: IDBObjectStore) => IDBRequest<T>) {
  const db = await openQueue()
  try {
    return await new Promise<T>((resolve, reject) => {
      const transaction = db.transaction('pending', mode)
      const request = operation(transaction.objectStore('pending'))
      transaction.oncomplete = () => resolve(request.result)
      transaction.onabort = () => reject(transaction.error)
    })
  } finally {
    db.close()
  }
}

export function createRunCapture(finished: (run: PendingRun) => void) {
  let active: PendingRun | undefined
  const reset = () => { active = undefined }

  function frame(fields: string[]) {
    const [kind, category, action, job, , result] = fields
    if (kind !== 'EVT' || category !== 'MOTION') return
    if (!['RUN_START', 'RUN_RESULT', 'PHASE', 'FLAG', 'SPAN', 'STEP', 'RESULT', 'ERROR'].includes(action!)) return
    if (action === 'RUN_START' && job === '0') {
      active = { id: crypto.randomUUID(), captured_at: new Date().toISOString(), frames: [] }
    }
    if (!active) return
    active.frames.push('{' + fields.join(',') + '}')
    if (action === 'RUN_RESULT' && job === '0') {
      const completed = active
      reset()
      if (result === 'DONE') finished(completed)
    }
  }
  return { frame, reset }
}

export function useCloudCapture(controller: Controller) {
  const status = ref('等待正式运行')
  const pending = ref(0)
  let uploading = false
  let requested = false
  async function upload() {
    requested = true
    if (uploading) return
    uploading = true
    try {
      while (requested) {
        requested = false
        const runs = await queueOperation('readonly', store => store.getAll()) as PendingRun[]
        pending.value = runs.length
        for (const run of runs) {
          status.value = '正在上传运行日志'
          await cloudRequest('runs', await prepareRunUpload(run))
          await queueOperation('readwrite', store => store.delete(run.id))
          pending.value--
          status.value = '已归档到云端'
        }
      }
    } catch (reason) {
      status.value = reason instanceof CloudError && reason.status === 401
        ? '登录队内账号后补传'
        : pending.value
          ? '上传失败，日志仍在本机待传 · ' + (reason instanceof Error ? reason.message : String(reason))
          : '无法读取本机待传日志'
    } finally {
      uploading = false
      if (requested) void upload()
    }
  }

  const capture = createRunCapture(run => {
    status.value = '正在保存完赛日志'
    void queueOperation('readwrite', store => store.put(run))
      .then(upload)
      .catch(() => { status.value = '完赛日志写入本机失败' })
  })
  const unsubscribe = controller.onFrame(capture.frame)
  const unsubscribeDisconnect = controller.onDisconnect(capture.reset)
  const channel = new BroadcastChannel('carrier-cloud-auth')
  channel.onmessage = () => { void upload() }
  onMounted(() => {
    window.addEventListener('online', upload)
    void upload()
  })
  onUnmounted(() => {
    unsubscribe()
    unsubscribeDisconnect()
    channel.close()
    window.removeEventListener('online', upload)
  })
  return { status, pending, upload }
}
