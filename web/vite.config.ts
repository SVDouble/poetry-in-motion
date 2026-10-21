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
  },
});
