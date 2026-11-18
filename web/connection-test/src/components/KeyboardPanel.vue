<script setup lang="ts">
import { onMounted, onUnmounted, ref } from 'vue'
import type { Controller } from '../useController'
import { frame } from '../commands'
const props = defineProps<{ controller: Controller }>()
const speed = ref(80)
const enabled = ref(false)
const pressed = new Set<string>()
const directions: Record<string, number> = { w: 0, s: 1, a: 2, d: 3, q: 4, e: 5 }
function drive(key: string) {
  if (!props.controller.canControl.value) return
  if (pressed.size) void props.controller.send(frame('CHASSIS', 'MOVE', [0, 0, 1]))
  pressed.add(key)
  void props.controller.send(frame('CHASSIS', 'MOVE', [directions[key]!, speed.value, 0]))
}
function release(key?: string) {
  if (key) pressed.delete(key); else pressed.clear()
  if (props.controller.connected.value) void props.controller.send(frame('CHASSIS', 'MOVE', [0, 0, 1]))
  const remaining = [...pressed].at(-1)
  if (remaining) void props.controller.send(frame('CHASSIS', 'MOVE', [directions[remaining]!, speed.value, 0]))
}
function down(event: KeyboardEvent) {
  if (!enabled.value || event.repeat || (event.target as HTMLElement).matches('input,textarea,select')) return
  const key = event.key.toLowerCase()
  if ((key === 'q' || key === 'e') && !event.shiftKey) {
    event.preventDefault()
    release()
    void props.controller.send(frame('POSE', 'GOTO_REL', [0, 0, key === 'q' ? 90 : -90]))
    return
  }
  if (key in directions) { event.preventDefault(); drive(key) }
  if (event.code === 'ShiftLeft') speed.value = Math.min(300, speed.value + 10)
  if (event.code === 'ShiftRight') speed.value = Math.max(10, speed.value - 10)
}
function up(event: KeyboardEvent) {
  if (pressed.has(event.key.toLowerCase())) release(event.key.toLowerCase())
}
function blur() { if (pressed.size) release() }
onMounted(() => { window.addEventListener('keydown', down); window.addEventListener('keyup', up); window.addEventListener('blur', blur) })
onUnmounted(() => { blur(); window.removeEventListener('keydown', down); window.removeEventListener('keyup', up); window.removeEventListener('blur', blur) })
</script>
<template>
  <label><input v-model="enabled" type="checkbox" @change="blur">启用键盘遥控</label>
  <label>速度 RPM<input v-model.number="speed" type="number" min="10" max="300"></label>
  <p>WASD 平移，Q/E 旋转 90°，Shift + Q/E 微调旋转，连续运动松开停车。左 Shift 提速，右 Shift 降速。</p>
  <div class="actions"><button v-for="(name, key) in {w:'前进 W',s:'后退 S',a:'左移 A',d:'右移 D',q:'左转 Q',e:'右转 E'}" :key="key"
    :disabled="!controller.canControl.value" @pointerdown="(event) => { (event.target as HTMLElement).setPointerCapture(event.pointerId); drive(key) }"
    @pointerup="release(key)" @pointercancel="release(key)" @contextmenu.prevent>{{ name }}</button></div>
  <button :disabled="!controller.connected.value" @click="release()">停止</button>
  <button :disabled="!controller.canControl.value" @click="controller.send(frame('POSE', 'GOTO_REL', [0,0,90]))">左转 90°</button>
  <button :disabled="!controller.canControl.value" @click="controller.send(frame('POSE', 'GOTO_REL', [0,0,-90]))">右转 90°</button>
</template>
