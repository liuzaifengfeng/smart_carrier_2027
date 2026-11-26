import { readFileSync } from 'node:fs'
import vue from '@vitejs/plugin-vue'
import { defineConfig } from 'vite'

const chassis = readFileSync(new URL('../../src/chassis.cpp', import.meta.url), 'utf8')
const geometry = readFileSync(new URL('../../src/field_geometry.def', import.meta.url), 'utf8')
const positions = chassis.match(/NODE_POSITIONS\[FIELD_NODE_COUNT\]\s*=\s*\{([\s\S]*?)\};/)?.[1] ?? ''
const nodes = Array.from(positions.matchAll(/\{\s*([\d.]+)f?,\s*([\d.]+)f?\s*\}/g),
  ([, x, y], id) => [id, Number(x), Number(y)])
const bounds = geometry.match(/FIELD_BOUNDS\(\s*(\d+),\s*(\d+)\s*\)/)
if (nodes.length !== 25 || !bounds) throw new Error('Cannot read firmware field map defaults')

export default defineConfig({
  plugins: [vue()],
  define: {
    __FIRMWARE_FIELD_MAP__: JSON.stringify({ nodes, fieldSize: Number(bounds[1]), bodyWidth: Number(bounds[2]) }),
  },
  base: './',
  server: { proxy: { '/api': 'http://127.0.0.1:8765' } },
  preview: { proxy: { '/api': 'http://127.0.0.1:8765' } },
  build: { rollupOptions: { input: ['index.html', 'console.html', 'runs.html'] } },
})
