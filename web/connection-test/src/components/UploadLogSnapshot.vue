<script setup lang="ts">
import { shallowRef, ref } from 'vue'
import { captureRecentLogs, CloudError, cloudRequest, type LogSnapshot } from '../cloudRuns'
import type { ConsoleLogEntry } from '../useController'

const props = defineProps<{ logs: ConsoleLogEntry[]; device: string }>()
const dialog = ref<HTMLDialogElement>()
const snapshot = shallowRef<ReturnType<typeof captureRecentLogs>>()
const note = ref('')
const busy = ref(false)
const needsLogin = ref(false)
const error = ref('')
const status = ref('')

function open() {
  const captured = captureRecentLogs(props.logs, props.device)
  status.value = ''
  if (!captured.logs.length) {
    status.value = '最近 60 秒没有日志'
    return
  }
  snapshot.value = captured
  note.value = error.value = ''
  needsLogin.value = false
  dialog.value?.showModal()
}

async function upload() {
  if (!snapshot.value || busy.value) return
  busy.value = true
  error.value = ''
  needsLogin.value = false
  try {
    const saved = await cloudRequest<LogSnapshot>('runs/snapshots', { ...snapshot.value, note: note.value.trim() })
    status.value = `已上传 ${saved.count} 条日志`
    dialog.value?.close()
  } catch (reason) {
    needsLogin.value = reason instanceof CloudError && reason.status === 401
    error.value = reason instanceof Error ? reason.message : String(reason)
  } finally {
    busy.value = false
  }
}
</script>

<template>
  <button type="button" @click="open">上传最近 60 秒日志</button>
  <span v-if="status" role="status">{{ status }} <a v-if="status.startsWith('已上传')" href="./runs.html#log-snapshots" target="_blank" rel="noopener">查看云端日志</a></span>
  <dialog ref="dialog" class="log-upload" aria-labelledby="log-upload-title" @cancel="busy && $event.preventDefault()">
    <form @submit.prevent="upload">
      <h2 id="log-upload-title">上传最近 60 秒日志</h2>
      <p>已选取点击前的 {{ snapshot?.logs.length }} 条日志。</p>
      <label>备注（选填）<textarea v-model="note" autofocus rows="3" :readonly="busy"></textarea></label>
      <p v-if="error" class="error" role="alert">
        {{ error }}
        <a v-if="needsLogin" href="./runs.html" target="_blank" rel="noopener">队内登录</a>
      </p>
      <div class="actions">
        <button type="submit" :disabled="busy">{{ busy ? '正在上传…' : '确认上传' }}</button>
        <button type="button" :disabled="busy" @click="dialog?.close()">取消</button>
      </div>
    </form>
  </dialog>
</template>

<style scoped>
.log-upload { width: min(28rem, calc(100% - 2rem)); box-sizing: border-box; border: 1px solid var(--border); }
.log-upload::backdrop { background: rgb(32 40 48 / 48%); }
.log-upload h2 { font-size: 1.1rem; margin: 0 0 0.5rem; }
.log-upload label { display: grid; gap: 0.3rem; }
.log-upload textarea { box-sizing: border-box; width: 100%; padding: 0.4rem; font: inherit; resize: vertical; }
</style>
