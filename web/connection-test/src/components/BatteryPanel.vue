<script setup lang="ts">
import { onUnmounted, ref, watch } from 'vue'
import type { Controller } from '../useController'

const props = defineProps<{ controller: Controller }>()
const voltage = ref<number>()
const threshold = ref(0)
const draft = ref<number | ''>()
const alarm = ref(false)
const unavailable = ref(false)
const message = ref('')
const editing = ref(false)
const volts = (mv: number) => (mv / 1000).toFixed(2)

const unsubscribe = props.controller.onFrame(fields => {
  const [kind, category, action, status, a, b, low] = fields
  if (kind !== 'RSP' || category !== 'BAT') return
  if (status === 'OK') {
    voltage.value = Number(a)
    const nextThreshold = Number(b)
    if (!editing.value && (draft.value === undefined || action === 'SET' || threshold.value !== nextThreshold)) {
      draft.value = nextThreshold / 1000
    }
    threshold.value = nextThreshold
    alarm.value = low === '1'
    if (action === 'SET') message.value = ''
  } else if (a === 'UNKNOWN_ACTION') {
    unavailable.value = true
  } else if (action === 'SET') {
    message.value = a === 'STORAGE' ? '写入小车失败' : '报警电压设置失败'
  }
})

watch(props.controller.ready, (ready, _, cleanup) => {
  voltage.value = undefined
  draft.value = undefined
  editing.value = false
  alarm.value = unavailable.value = false
  message.value = ''
  if (!ready) return
  const update = () => {
    if (!unavailable.value) void props.controller.send('{CMD,BAT,GET}')
  }
  update()
  const timer = window.setInterval(update, 30_000)
  cleanup(() => window.clearInterval(timer))
}, { immediate: true })
onUnmounted(unsubscribe)

function setThreshold(event: FocusEvent) {
  editing.value = false
  const input = event.currentTarget as HTMLInputElement
  if (!input.validity.valid || typeof draft.value !== 'number') {
    draft.value = threshold.value / 1000
    return
  }
  const millivolts = Math.round(draft.value * 1000)
  if (millivolts === threshold.value) return
  message.value = ''
  void props.controller.send(`{CMD,BAT,SET,${millivolts}}`)
}

function finish(event: KeyboardEvent) {
  if (event.isComposing) return
  const input = event.currentTarget as HTMLInputElement
  input.blur()
}
</script>

<template>
  <section class="battery-panel" aria-label="电池电压">
      <span>电池</span>
      <strong>{{ voltage === undefined ? '—' : volts(voltage) + ' V' }}</strong>
      <label v-if="voltage !== undefined">报警
        <input v-model.number="draft" aria-label="报警电压" type="number" inputmode="decimal" step="0.001" min="0" required
          title="点击编辑，按 Enter 或移开焦点保存；0 为关闭" @focus="editing = true" @blur="setThreshold" @keydown.enter="finish">
        V
      </label>
      <span v-if="alarm" class="error" role="status">低压报警</span>
      <span v-if="unavailable" title="当前固件未提供电压读数">未支持</span>
      <span v-if="message" class="error" role="status">{{ message }}</span>
  </section>
</template>

<style scoped>
.battery-panel, .battery-panel label { display: flex; align-items: center; gap: 0.35rem; white-space: nowrap; }
.battery-panel strong { font-variant-numeric: tabular-nums; }
.battery-panel input { width: 4.2rem; padding-inline: 4px; appearance: textfield; }
.battery-panel input::-webkit-inner-spin-button { appearance: none; }
</style>
