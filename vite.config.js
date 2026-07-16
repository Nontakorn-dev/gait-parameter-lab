import { defineConfig } from 'vite';
import { resolve } from 'node:path';

export default defineConfig({
  server: {
    port: 5180,
    // Web Bluetooth requires a secure context (HTTPS) on real devices/phones.
    // On localhost, Chrome treats it as secure automatically.
    host: true,
  },
  build: {
    rollupOptions: {
      input: {
        main: resolve(__dirname, 'index.html'),
        debug: resolve(__dirname, 'debug/index.html'),
        validate: resolve(__dirname, 'validate/index.html'),
      },
    },
  },
});
