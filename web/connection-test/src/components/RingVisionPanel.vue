<script setup lang="ts">
import { ref, watch } from 'vue'
import { frame } from '../commands'
import type { Controller } from '../useController'

const { controller } = defineProps<{ controller: Controller }>()
const gains = ['Kp', 'Ki', 'Kd']
const axes = ['X → 转台（角度）', 'Y → 伸缩（毫米）']
const ringPosition = ref(1)
const values = ref(['0.06', '0', '0', '0.05', '0', '0'])
const lastReadback = ref('')
for (const action of ['RING_AXIS_PID_GET', 'RING_AXIS_PID_SET']) {
  watch(() => controller.snapshots.value['VISION/' + action], reply => {
    if (reply?.length === axes.length * gains.length) {
      values.value = [...reply]
      lastReadback.value = '设备回读 · X ' + reply.slice(0, 3).join(' / ')
        + ' · Y ' + reply.slice(3).join(' / ')
    }
  }, { immediate: true })
}
</script>

<template>
  <fieldset>
    <legend>圆环纠偏视觉</legend>
    <div class="actions">
      <label>目标圆环号
        <select v-model="ringPosition">
          <option v-for="position in 3" :key="position" :value="position">{{ position }}</option>
        </select>
      </label>
      <button :disabled="!controller.canControl.value" @click="controller.send(frame('VISION', 'START_REQUEST', ['RING_CORRECTION', ringPosition]))">开启圆环视觉</button>
      <button :disabled="!controller.canControl.value" @click="controller.send(frame('VISION', 'STOP_REQUEST', ['RING_CORRECTION']))">结束圆环视觉</button>
    </div>
    <form @submit.prevent="controller.send(frame('VISION', 'RING_AXIS_PID_SET', values))">
      <fieldset v-for="(axis, group) in axes" :key="axis">
        <legend>{{ axis }}</legend>
        <div class="actions">
          <label v-for="(gain, index) in gains" :key="gain">{{ gain }}
            <input v-model="values[group * gains.length + index]" type="number" step="any" required>
          </label>
        </div>
      </fieldset>
      <button :disabled="!controller.canControl.value">应用纠偏参数</button>
      <button type="button" :disabled="!controller.canControl.value" @click="controller.send(frame('VISION', 'RING_AXIS_PID_GET'))">读取纠偏参数</button>
    </form>
    <output aria-live="polite">{{ lastReadback }}</output>
  </fieldset>
</template>
