// frontend/src/App.jsx
import React, { Suspense, lazy } from 'react'
import { createBrowserRouter, RouterProvider, Navigate, Outlet } from 'react-router-dom'
import { AnimatePresence } from 'framer-motion'
import LoadingSpinner from './components/ui/LoadingSpinner'
import Layout from './components/layout/Layout'

// Lazy pages
const Dashboard = lazy(() => import('./pages/Dashboard'))
const Simulator = lazy(() => import('./pages/Simulator'))
const Examples = lazy(() => import('./pages/Examples'))
const Maps = lazy(() => import('./pages/Maps'))
const Navigation = lazy(() => import('./pages/Navigation'))
const Settings = lazy(() => import('./pages/Settings'))
const Turtlesim = lazy(() => import('./pages/Turtlesim'))
const SoArm101 = lazy(() => import('./pages/SoArm101'))
const SO101Simulator = lazy(() => import('./pages/SO101Simulator'))
const NotFound = lazy(() => import('./pages/NotFound'))

const PageLoader = () => (
    <div className="flex items-center justify-center min-h-screen bg-gray-950">
        <LoadingSpinner size="large" />
    </div>
)

// Wrapper for suspense loading
const SuspenseWrapper = ({ children }) => (
    <Suspense fallback={<PageLoader />}>
        {children}
    </Suspense>
)

// Create router with future flags enabled
const router = createBrowserRouter(
    [
        {
            path: '/',
            element: <Layout />,
            children: [
                { index: true, element: <Navigate to="/dashboard" replace /> },
                { path: 'dashboard', element: <SuspenseWrapper><Dashboard /></SuspenseWrapper> },
                { path: 'simulator', element: <SuspenseWrapper><Simulator /></SuspenseWrapper> },
                { path: 'examples', element: <SuspenseWrapper><Examples /></SuspenseWrapper> },
                { path: 'maps', element: <SuspenseWrapper><Maps /></SuspenseWrapper> },
                { path: 'navigation', element: <SuspenseWrapper><Navigation /></SuspenseWrapper> },
                { path: 'settings', element: <SuspenseWrapper><Settings /></SuspenseWrapper> },
                { path: 'turtlesim', element: <SuspenseWrapper><Turtlesim /></SuspenseWrapper> },
                { path: 'so-arm-101', element: <SuspenseWrapper><SoArm101 /></SuspenseWrapper> },
                { path: 'so101-simulator', element: <SuspenseWrapper><SO101Simulator /></SuspenseWrapper> },
                { path: '*', element: <SuspenseWrapper><NotFound /></SuspenseWrapper> },
            ],
        },
    ],
    {
        future: {
            v7_startTransition: true,
            v7_relativeSplatPath: true,
        },
    }
)

export default function App() {
    return (
        <AnimatePresence mode="wait">
            <RouterProvider router={router} />
        </AnimatePresence>
    )
}
