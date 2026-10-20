import { defineConfig } from 'vite';

// Cross-origin isolation enables SharedArrayBuffer (zero-copy snapshots from the physics worker)
// and high-resolution timers. Production hosts must send the same headers (see public/_headers).
const isolation = {
  'Cross-Origin-Opener-Policy': 'same-origin',
  'Cross-Origin-Embedder-Policy': 'require-corp',
};

export default defineConfig({
  server: { headers: isolation },
  preview: { headers: isolation },
  worker: { format: 'es' },
  build: {
    target: 'es2023',
    assetsInlineLimit: 0,
    // three.js is large and changes rarely: give it its own long-cached chunk
    chunkSizeWarningLimit: 1200,
    rolldownOptions: {
      output: {
        codeSplitting: { groups: [{ name: 'three', test: /node_modules[\\/]three/ }] },
      },
    },
  },
});
