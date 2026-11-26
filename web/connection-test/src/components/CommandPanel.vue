<script setup lang="ts">
import { reactive } from 'vue'
import { frame, operations } from '../commands'
import type { Controller } from '../useController'
defineProps<{ page: string; controller: Controller }>()
const values = reactive(Object.fromEntries(Object.values(operations).flat().map(op =>
  [op.category + '/' + op.action, op.fields.map(field => field[1])])))
</script>
<template>
  <fieldset v-for="op in operations[page]" :key="op.action">
    <legend>{{ op.title }}</legend>
    <div v-if="op.category === 'ARM' && op.action === 'ENABLE'" class="actions">
      <button type="button" :disabled="!controller.canControl.value"
        @click="controller.send(frame(op.category, op.action, [1]))">使能</button>
      <button type="button" :disabled="!controller.canControl.value"
        @click="controller.send(frame(op.category, op.action, [0]))">失能</button>
      <output>{{ controller.snapshots.value['ARM/ENABLE']?.join(' / ') }}</output>
    </div>
    <form v-else @submit.prevent="controller.send(frame(op.category, op.action, values[op.category + '/' + op.action]))">
      <label v-for="(field, index) in op.fields" :key="index">{{ field[0] }}
        <select v-if="field[2]" v-model="values[op.category + '/' + op.action]![index]">
          <option v-for="(label, value) in field[2]" :key="value" :value="value">{{ label }}</option>
        </select>
        <input v-else v-model="values[op.category + '/' + op.action]![index]" required>
      </label>
      <button :disabled="!controller.canControl.value">发送</button>
      <output>{{ controller.snapshots.value[op.category + '/' + op.action]?.join(' / ') }}</output>
    </form>
  </fieldset>
</template>
