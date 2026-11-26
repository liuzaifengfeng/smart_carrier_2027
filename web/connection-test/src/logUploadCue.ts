let context: AudioContext | undefined

export function enableLogUploadCue() {
  if (!window.AudioContext) return
  context ??= new AudioContext()
  if (context.state === 'suspended') void context.resume().catch(() => {})
}

function play(context: AudioContext) {
  const time = context.currentTime
  const tone = context.createOscillator()
  const volume = context.createGain()
  tone.frequency.value = 880
  volume.gain.setValueAtTime(0, time)
  volume.gain.linearRampToValueAtTime(0.13, time + 0.008)
  volume.gain.exponentialRampToValueAtTime(0.001, time + 0.12)
  tone.connect(volume).connect(context.destination)
  tone.start(time)
  tone.stop(time + 0.12)
}

export function playLogUploadCue() {
  const audio = context
  if (!audio) return
  if (audio.state === 'running') play(audio)
  else void audio.resume().then(() => play(audio)).catch(() => {})
}
