<script setup lang="ts">
import { ref, watch } from 'vue'
import { cloudRequest } from '../cloudRuns'

const props = defineProps<{ runId: string; note: string }>()
const emit = defineEmits<{ saved: [note: string] }>()
const draft = ref(props.note)
const busy = ref(false)
const error = ref('')
watch(() => props.note, note => { draft.value = note })

async function save() {
  const note = draft.value.trim()
  if (busy.value || note === props.note) return
  busy.value = true
  error.value = ''
  try {
    const saved = await cloudRequest<{ note: string }>(`runs/${props.runId}/note`, { note })
    draft.value = saved.note
    emit('saved', saved.note)
  } catch (reason) {
    error.value = reason instanceof Error ? reason.message : String(reason)
  } finally {
    busy.value = false
  }
}

function finish(event: KeyboardEvent) {
  if (event.isComposing) return
  event.preventDefault()
  const field = event.currentTarget as HTMLTextAreaElement
  field.blur()
}
</script>

<template>
  <div class="note-field">
    <textarea v-model="draft" rows="1" aria-label="场次备注" placeholder="添加备注" :readonly="busy"
      @blur="save" @keydown.enter="finish"></textarea>
    <span v-if="error" class="error" role="alert">{{ error }}</span>
  </div>
</template>
