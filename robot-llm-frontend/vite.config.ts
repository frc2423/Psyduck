import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [
    react({
      babel: {
        plugins: [['babel-plugin-react-compiler']],
      },
    }),
  ],
  server: {
    proxy: {
      // The Python bridge (`uv run robot-llm-server`) listens on :8000.
      '/ws': { target: 'ws://127.0.0.1:8000', ws: true },
      '/api': { target: 'http://127.0.0.1:8000' },
    },
  },
})
