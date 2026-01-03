import { defineConfig, loadEnv } from 'vite'
import react from '@vitejs/plugin-react'
import path from 'path'

export default defineConfig(({ mode }) => {
    const env = loadEnv(mode, process.cwd(), '')
    const DEV_BACKEND_TARGET = env.VITE_DEV_BACKEND_TARGET || 'http://localhost:8082'

    return {
        plugins: [react()],
        resolve: {
            alias: {
                '@': path.resolve(__dirname, './src')
            },
        },
        server: {
            host: '0.0.0.0',
            port: 5173,
            strictPort: true,
            proxy: {
                '/api': {
                    target: 'http://localhost:8082',
                    changeOrigin: true,
                    secure: false,
                },
                '/ws': {
                    target: 'http://localhost:8082',
                    changeOrigin: true,
                    ws: true,
                    secure: false,
                },
                '/rosbridge': {
                    target: 'ws://localhost:9091',
                    changeOrigin: true,
                    ws: true,
                    secure: false,
                },
            },
        },
        define: {
            'process.env': {},
            global: 'globalThis'
        },
        optimizeDeps: {
            include: [
                'react',
                'react-dom',
                'three',
                'roslib',
                '@tanstack/react-query'
            ]
        },
        build: {
            outDir: 'dist',
            sourcemap: true,
            rollupOptions: {
                output: {
                    manualChunks: {
                        'react-vendor': ['react', 'react-dom', 'react-router-dom'],
                        'ui-vendor': ['framer-motion', 'lucide-react'],
                        'api-vendor': ['axios', '@tanstack/react-query', 'roslib']
                    },
                },
            },
        },
    }
})