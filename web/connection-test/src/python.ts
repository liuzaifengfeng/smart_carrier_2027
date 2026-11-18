import type { PyodideInterface } from 'pyodide'
let runtime: Promise<PyodideInterface> | undefined
export function python() {
  runtime ??= (async () => {
    const origin = new URL('./pyodide/', window.location.href).href
    const module = await import(/* @vite-ignore */ origin + 'loader.js')
    const py: PyodideInterface = await module.loadPyodide({ indexURL: origin })
    py.FS.mkdirTree('/carrier/src')
    for (const file of ['hybrid_plan.py', 'continuous_trial.py', 'field_geometry.py', 'action_group_config.py', 'desktop_helpers.py', 'src/field_geometry.def']) {
      const response = await fetch(new URL('./python/' + file, window.location.href))
      if (!response.ok) throw new Error('规划模块加载失败 ' + file)
      py.FS.writeFile('/carrier/' + file, await response.text())
    }
    py.runPython("import sys, json\nsys.path.insert(0, '/carrier')")
    return py
  })()
  return runtime
}
export function download(name: string, content: string, type = 'text/plain') {
  const url = URL.createObjectURL(new Blob([content], { type }))
  const anchor = document.createElement('a')
  anchor.href = url
  anchor.download = name
  anchor.click()
  setTimeout(() => URL.revokeObjectURL(url), 0)
}
