import { ref, shallowRef } from 'vue'

interface InstallPrompt extends Event {
  prompt(): Promise<{ outcome: 'accepted' | 'dismissed' }>
}

export function usePwa() {
  const installPrompt = shallowRef<InstallPrompt>()
  const offlineStatus = ref('')
  const waiting = shallowRef<ServiceWorker | null>(null)
  const updating = ref(false)
  const enabled = import.meta.env.PROD && 'serviceWorker' in navigator

  if (enabled) {
    window.addEventListener('beforeinstallprompt', event => {
      event.preventDefault()
      installPrompt.value = event as InstallPrompt
    })
    window.addEventListener('appinstalled', () => { installPrompt.value = undefined })
    navigator.serviceWorker.addEventListener('controllerchange', () => {
      if (updating.value) location.reload()
    })

    offlineStatus.value = '正在准备离线资源'
    void navigator.serviceWorker.register('./sw.js', { updateViaCache: 'none' }).then(registration => {
      waiting.value = registration.waiting
      if (registration.active) offlineStatus.value = '离线资源已就绪'

      function watchInstall() {
        const worker = registration.installing
        worker?.addEventListener('statechange', () => {
          if (worker.state === 'installed') waiting.value = registration.waiting
          if (worker.state === 'redundant' && !registration.active) {
            offlineStatus.value = '离线资源准备失败'
          }
        })
      }
      watchInstall()
      registration.addEventListener('updatefound', watchInstall)
      void navigator.serviceWorker.ready.then(() => { offlineStatus.value = '离线资源已就绪' })
      window.addEventListener('focus', () => { void registration.update().catch(() => {}) })
    }).catch(() => { offlineStatus.value = '离线资源准备失败' })
  }

  function install() {
    const prompt = installPrompt.value
    installPrompt.value = undefined
    void prompt?.prompt()
  }

  function update() {
    if (!waiting.value) return
    updating.value = true
    waiting.value.postMessage('activate-update')
  }

  return { installPrompt, offlineStatus, waiting, updating, install, update }
}
