import React, { useEffect, useState } from 'react'
import { useQuery } from '@tanstack/react-query'
import { visualizationAPI } from '../../services/api'

// RViz noVNC iframe-based visualization
const RvizPanel = () => {
    const [rvizUrl, setRvizUrl] = useState(null)
    const [isLoading, setIsLoading] = useState(true)
    const [error, setError] = useState(null)

    const { data: rvizData, isLoading: isFetchingUrl } = useQuery({
        queryKey: ['rviz-url'],
        queryFn: visualizationAPI.getRvizUrl,
        retry: 3,
        retryDelay: 1000,
    })

    useEffect(() => {
        if (rvizData?.url) {
            setRvizUrl(rvizData.url)
            setIsLoading(false)
        } else if (isFetchingUrl) {
            setIsLoading(true)
            setError(null)
        } else if (rvizData && !rvizData.url) {
            setError('RViz URL not available')
            setIsLoading(false)
        }
    }, [rvizData, isFetchingUrl])

    return (
        <div className="w-full h-full bg-gray-950 flex items-center justify-center relative">
            {isLoading && (
                <div className="absolute inset-0 bg-gray-900/80 flex items-center justify-center z-10">
                    <div className="text-center">
                        <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-500 mx-auto mb-4"></div>
                        <p className="text-white text-sm">Loading RViz...</p>
                        <p className="text-gray-400 text-xs mt-1">Connecting to visualization server</p>
                    </div>
                </div>
            )}

            {error && (
                <div className="absolute inset-0 bg-gray-900/80 flex items-center justify-center z-10">
                    <div className="text-center">
                        <p className="text-red-400 text-sm mb-2">Error loading RViz</p>
                        <p className="text-gray-400 text-xs">{error}</p>
                    </div>
                </div>
            )}

            {rvizUrl && !isLoading && !error && (
                <iframe
                    src={rvizUrl}
                    className="w-full h-full border-0"
                    title="RViz Visualization"
                    allow="clipboard-read; clipboard-write"
                />
            )}
        </div>
    )
}

export default RvizPanel