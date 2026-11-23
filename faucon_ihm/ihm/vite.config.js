import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react()],

  // Base path pour le build statique (servi depuis la racine)
  base: '/',

  server: {
    // Dev : écoute sur tout le LAN
    host: '0.0.0.0',
    port: 5173,
  },

  build: {
    // Output dans ihm/dist/ (copié par CMakeLists dans le package ROS2)
    outDir: 'dist',
    sourcemap: false,
    rollupOptions: {
      output: {
        // Chunking pour optimiser le chargement
        manualChunks: {
          react: ['react', 'react-dom'],
        },
      },
    },
  },
})
