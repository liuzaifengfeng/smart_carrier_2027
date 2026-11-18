import vue from '@vitejs/plugin-vue'
import { defineConfig } from 'vite'

export default defineConfig({
  plugins: [vue()],
  base: './',
  server: { proxy: { '/api': 'http://127.0.0.1:8765' } },
  preview: { proxy: { '/api': 'http://127.0.0.1:8765' } },
  build: { rollupOptions: { input: ['index.html', 'console.html', 'runs.html'] } },
})
