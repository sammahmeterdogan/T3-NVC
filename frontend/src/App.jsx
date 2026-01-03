// frontend/src/App.jsx
import React, { Suspense, lazy } from 'react'
import { BrowserRouter as Router, Routes, Route, Navigate } from 'react-router-dom'
import { AnimatePresence } from 'framer-motion'
import LoadingSpinner from './components/ui/LoadingSpinner'
import Layout from './components/layout/Layout'

// Lazy pages
const Dashboard = lazy(() => import('./pages/Dashboard'))
const Simulator = lazy(() => import('./pages/Simulator'))
const Examples  = lazy(() => import('./pages/Examples'))
const Maps      = lazy(() => import('./pages/Maps'))
const Settings  = lazy(() => import('./pages/Settings'))
const Turtlesim = lazy(() => import('./pages/Turtlesim'))
const NotFound  = lazy(() => import('./pages/NotFound'))

const PageLoader = () => (
    <div className="flex items-center justify-center min-h-screen bg-gray-950">
        <LoadingSpinner size="large" />
    </div>
)

export default function App() {
    return (
        <Router>
            <AnimatePresence mode="wait">
                <Suspense fallback={<PageLoader />}>
                    <Routes>
                        <Route path="/" element={<Layout />}>
                            <Route index element={<Navigate to="/dashboard" replace />} />
                            <Route path="dashboard" element={<Dashboard />} />
                            <Route path="simulator" element={<Simulator />} />
                            <Route path="examples"  element={<Examples />} />
                            <Route path="maps"      element={<Maps />} />
                            <Route path="settings"  element={<Settings />} />
                            <Route path="turtlesim" element={<Turtlesim />} />
                            <Route path="*"         element={<NotFound />} />
                        </Route>
                    </Routes>
                </Suspense>
            </AnimatePresence>
        </Router>
    )
}
