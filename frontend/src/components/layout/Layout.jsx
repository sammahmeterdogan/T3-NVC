import React, { useState } from 'react'
import { Outlet } from 'react-router-dom'
import Sidebar from './Sidebar'
import Topbar from './Topbar'
import LiveTerminal from '../common/LiveTerminal'

const Layout = () => {
    const [sidebarCollapsed, setSidebarCollapsed] = useState(false)

    return (
        <div className="flex h-screen bg-gray-950 overflow-hidden relative">
            {/* Sidebar */}
            <Sidebar isCollapsed={sidebarCollapsed} setIsCollapsed={setSidebarCollapsed} />

            {/* Main Content Area */}
            <div className="flex-1 flex flex-col overflow-hidden">
                {/* Topbar */}
                <Topbar />

                {/* Page Content */}
                <main className="flex-1 overflow-auto p-6 bg-gradient-to-br from-gray-950 via-gray-900 to-gray-950">
                    <div className="max-w-[1920px] mx-auto h-full">
                        <Outlet />
                    </div>
                </main>
            </div>

            {/* Global System Terminal - her şeyin üzerinde durur */}
            <LiveTerminal />
        </div>
    )
}

export default Layout