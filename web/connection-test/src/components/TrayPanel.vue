<script setup lang="ts">
import { onUnmounted, ref, watch } from 'vue'
import { frame } from '../commands'
import type { Controller } from '../useController'

const props = defineProps<{ controller: Controller }>()
const angle = ref(0)
const interval = ref(0)
const power = ref(0)
const status = ref('')
const pending = ref(false)

function rotate() {
  pending.value = true
  status.value = '等待主控应答'
  void props.controller.send(frame('TRAY', 'ROTATE', [angle.value, interval.value, power.value]))
}

function zero() {
  pending.value = true
  status.value = '等待读取料盘零点'
  void props.controller.send(frame('TRAY', 'ZERO'))
}

const remove = props.controller.onFrame(([type, category, action, result, reason, , multi]) => {
  if (category !== 'TRAY') return
  if (type === 'RSP' && action === 'ZERO' && result === 'READBACK') {
    pending.value = false
    status.value = '料盘设零后回读 · 多圈 ' + multi + '°'
    return
  }
  if (type === 'RSP' && result === 'ACK') {
    status.value = '料盘正在旋转'
  } else if (type === 'RSP' && result === 'ERR') {
    pending.value = false
    status.value = '指令未受理 · ' + reason
  } else if (type === 'EVT') {
    pending.value = false
    status.value = result === 'DONE' ? '料盘已到位' : '旋转未完成'
  }
})
watch(props.controller.connected, connected => {
  if (!connected) {
    pending.value = false
    status.value = ''
  }
})
onUnmounted(remove)
</script>

<template>
  <form @submit.prevent="rotate">
    <fieldset>
      <legend>旋转料盘</legend>
      <label>目标绝对角度（°）
        <input v-model.number="angle" type="number" required min="-368640" max="368640" step="0.1">
      </label>
      <label>运动时长（ms，0 为尽快到达）
        <input v-model.number="interval" type="number" required min="0" max="4294967295" step="1">
      </label>
      <label>功率限制（mW，0 为舵机默认）
        <input v-model.number="power" type="number" required min="0" max="65535" step="1">
      </label>
      <button type="submit" :disabled="!controller.canControl.value || pending">旋转</button>
      <button type="button" :disabled="!controller.canControl.value || pending" @click="zero">将当前位置设为料盘零点</button>
      <output role="status">{{ status }}</output>
    </fieldset>
  </form>
</template>
