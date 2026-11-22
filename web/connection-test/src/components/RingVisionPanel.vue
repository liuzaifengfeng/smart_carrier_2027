<script setup lang="ts">
import { computed, ref, watch } from 'vue'
import { frame } from '../commands'
import type { useParameters } from '../parameters'
import type { Controller } from '../useController'

const { controller, model, showVision } = defineProps<{
  controller: Controller
  model: ReturnType<typeof useParameters>
  showVision: boolean
}>()
const gains = ['Kp', 'Ki', 'Kd']
const axes = ['X → 转台（角度）', 'Y → 伸缩（毫米）']
const ringPosition = ref(1)
const values = ref(['0.0427', '0.01', '0', '0.033', '0.007', '0'])
const lastReadback = ref('')
const deadzones = computed(() => model.parameters.value.filter(p =>
  p.name === '圆环纠偏/X死区(px)' || p.name === '圆环纠偏/Y死区(px)'))
const deadzoneIndexes = computed(() => deadzones.value.map(p => p.index))
const hasDeadzoneEdits = computed(() => deadzoneIndexes.value.some(index => index in model.edits.value))
const timeout = computed(() => model.parameters.value.find(p => p.name === '圆环纠偏/超时(ms)'))
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
    <legend>{{ showVision ? '圆环纠偏视觉' : '圆环纠偏调参' }}</legend>
    <div v-if="showVision" class="actions">
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
      <button :disabled="!controller.canControl.value">应用 PID 增益</button>
      <button type="button" :disabled="!controller.canControl.value" @click="controller.send(frame('VISION', 'RING_AXIS_PID_GET'))">读取 PID 增益</button>
    </form>
    <output aria-live="polite">{{ lastReadback }}</output>
    <fieldset v-if="!showVision">
      <legend>圆环纠偏死区和超时</legend>
      <div class="actions">
        <label v-for="p in deadzones" :key="p.index">{{ p.name.split('/')[1] }}
          <input v-model="model.edits.value[p.index]" type="number" step="any"
            :min="p.min" :max="p.max" :placeholder="p.value" :disabled="model.busy.value">
        </label>
        <button :disabled="!controller.canControl.value || model.busy.value"
          @click="model.readDeadzones()">读取死区</button>
        <button :disabled="!controller.canControl.value || model.busy.value || !hasDeadzoneEdits"
          @click="model.write(undefined, deadzoneIndexes)">应用死区并回读</button>
      </div>
      <div class="actions">
        <label v-if="timeout">超时（ms，0 为跳过视觉纠偏）
          <input v-model="model.edits.value[timeout.index]" type="number" step="1"
            :min="timeout.min" :max="timeout.max" :placeholder="timeout.value" :disabled="model.busy.value">
        </label>
        <button :disabled="!controller.canControl.value || model.busy.value"
          @click="model.readRingTimeout()">读取超时</button>
        <button v-if="timeout" :disabled="!controller.canControl.value || model.busy.value || !(timeout.index in model.edits.value)"
          @click="model.write(undefined, [timeout.index])">应用超时并回读</button>
      </div>
      <output aria-live="polite">{{ model.status.value }}</output>
    </fieldset>
  </fieldset>
</template>
