import { defineConfig } from 'vite';

export default defineConfig({
  base: './',
  build: { target: 'es2022' },
  // The MuJoCo package locates its .wasm relative to its own module; don't pre-bundle it.
  optimizeDeps: { exclude: ['@mujoco/mujoco'], esbuildOptions: { target: 'es2022' } },
});
