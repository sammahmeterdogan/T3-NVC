import React, { useState } from 'react'
import { motion } from 'framer-motion'
import { useNavigate } from 'react-router-dom'
import { useQuery, useMutation } from '@tanstack/react-query'
import toast from 'react-hot-toast'
import {
    Play,
    Info,
    Clock,
    Cpu,
    Filter,
    Search,
    ChevronRight,
    Gamepad2,
    Map,
    Navigation,
    Shield,
    Users,
    Route,
    GitBranch,
    Target,
    Grid3x3,
    BookOpen,
    Video,
    Code,
    Eye,
    Bug
} from 'lucide-react'
import PageContainer from '../components/layout/PageContainer'
import { examplesAPI, simulationAPI } from '../services/api'
import { EXAMPLES_DATA } from '../data/examples'

const getIconForCategory = (category) => {
    const icons = {
        BASIC: Gamepad2,
        MAPPING: Map,
        NAVIGATION: Navigation,
        AUTONOMOUS: Shield,
        VISION: Users,
        CONTROL: Target,
        PERCEPTION: Eye,
        DEBUG: Bug,
        ADVANCED: Grid3x3
    }
    return icons[category] || Cpu
}

const getDifficultyBadge = (difficulty) => {
    const styles = {
        EASY: 'bg-green-900/50 text-green-400 border-green-800',
        MEDIUM: 'bg-yellow-900/50 text-yellow-400 border-yellow-800',
        HARD: 'bg-red-900/50 text-red-400 border-red-800'
    }
    return styles[difficulty] || styles.MEDIUM
}

const Examples = () => {
    const navigate = useNavigate()
    const [searchQuery, setSearchQuery] = useState('')
    const [selectedCategory, setSelectedCategory] = useState('ALL')
    const [selectedDifficulty, setSelectedDifficulty] = useState('ALL')

    // Use local EXAMPLES_DATA as fallback, merge with API data if available
    const { data: apiExamples, isLoading } = useQuery({
        queryKey: ['examples'],
        queryFn: async () => {
            try {
                return await examplesAPI.list()
            } catch (error) {
                console.warn('API examples not available, using local data')
                return EXAMPLES_DATA
            }
        },
        initialData: EXAMPLES_DATA
    })

    const launchExample = useMutation({
        mutationFn: (exampleId) => examplesAPI.launch(exampleId),
        onSuccess: () => {
            toast.success('Example launched successfully')
            navigate('/simulator')
        },
        onError: () => {
            toast.error('Failed to launch example')
        }
    })

    const categories = ['ALL', 'BASIC', 'MAPPING', 'NAVIGATION', 'AUTONOMOUS', 'PERCEPTION', 'DEBUG', 'CONTROL', 'ADVANCED']
    const difficulties = ['ALL', 'EASY', 'MEDIUM', 'HARD']

    const examples = apiExamples || EXAMPLES_DATA

    const filteredExamples = examples?.filter(example => {
        const matchesSearch = example.title.toLowerCase().includes(searchQuery.toLowerCase()) ||
            example.description.toLowerCase().includes(searchQuery.toLowerCase())
        const matchesCategory = selectedCategory === 'ALL' || example.category === selectedCategory
        const matchesDifficulty = selectedDifficulty === 'ALL' || example.difficulty === selectedDifficulty

        return matchesSearch && matchesCategory && matchesDifficulty
    })

    return (
        <PageContainer
            title="Example Scenarios"
            description="Pre-configured TurtleBot3 examples and demonstrations"
        >
            {/* Filters */}
            <div className="bg-gray-900 border border-gray-800 rounded-xl p-4 mb-6">
                <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
                    {/* Search */}
                    <div className="relative">
                        <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-4 h-4 text-gray-500" />
                        <input
                            type="text"
                            placeholder="Search examples..."
                            value={searchQuery}
                            onChange={(e) => setSearchQuery(e.target.value)}
                            className="w-full pl-10 pr-4 py-2 bg-gray-800 border border-gray-700 rounded-lg text-sm text-gray-300 placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-primary-500"
                        />
                    </div>

                    {/* Category Filter */}
                    <div className="flex items-center gap-2">
                        <Filter className="w-4 h-4 text-gray-500" />
                        <select
                            value={selectedCategory}
                            onChange={(e) => setSelectedCategory(e.target.value)}
                            className="flex-1 px-3 py-2 bg-gray-800 border border-gray-700 rounded-lg text-sm text-gray-300 focus:outline-none focus:ring-2 focus:ring-primary-500"
                        >
                            {categories.map(cat => (
                                <option key={cat} value={cat}>{cat}</option>
                            ))}
                        </select>
                    </div>

                    {/* Difficulty Filter */}
                    <div className="flex items-center gap-2">
                        <Clock className="w-4 h-4 text-gray-500" />
                        <select
                            value={selectedDifficulty}
                            onChange={(e) => setSelectedDifficulty(e.target.value)}
                            className="flex-1 px-3 py-2 bg-gray-800 border border-gray-700 rounded-lg text-sm text-gray-300 focus:outline-none focus:ring-2 focus:ring-primary-500"
                        >
                            {difficulties.map(diff => (
                                <option key={diff} value={diff}>{diff}</option>
                            ))}
                        </select>
                    </div>
                </div>
            </div>

            {/* Examples Grid */}
            {isLoading ? (
                <div className="flex items-center justify-center h-64">
                    <div className="text-center">
                        <Cpu className="w-12 h-12 text-gray-600 mx-auto mb-4 animate-spin" />
                        <p className="text-gray-400">Loading examples...</p>
                    </div>
                </div>
            ) : (
                <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                    {filteredExamples?.map((example, index) => {
                        const Icon = getIconForCategory(example.category)

                        return (
                            <motion.div
                                key={example.id}
                                initial={{ opacity: 0, y: 20 }}
                                animate={{ opacity: 1, y: 0 }}
                                transition={{ delay: index * 0.05 }}
                                whileHover={{ y: -4 }}
                                className="bg-gray-900 border border-gray-800 rounded-xl overflow-hidden hover:border-gray-700 transition-all"
                            >
                                {/* Card Header */}
                                <div className="p-4 border-b border-gray-800">
                                    <div className="flex items-start justify-between mb-3">
                                        <div className="p-2 bg-primary-500/10 rounded-lg">
                                            <Icon className="w-6 h-6 text-primary-400" />
                                        </div>
                                        <span className={`px-2 py-1 text-xs rounded-full border ${getDifficultyBadge(example.difficulty)}`}>
                      {example.difficulty}
                    </span>
                                    </div>

                                    <h3 className="text-white font-semibold mb-1">{example.title}</h3>
                                    <p className="text-gray-400 text-sm line-clamp-2">{example.description}</p>
                                </div>

                                {/* Card Body */}
                                <div className="p-4">
                                    {/* Category Badge */}
                                    <div className="flex items-center gap-2 mb-4">
                    <span className="px-2 py-1 bg-gray-800 text-gray-300 text-xs rounded">
                      {example.category}
                    </span>
                                        {example.enabled && (
                                            <span className="px-2 py-1 bg-green-900/50 text-green-400 text-xs rounded">
                        Available
                      </span>
                                        )}
                                    </div>

                                    {/* Action Buttons */}
                                    <div className="flex gap-2">
                                        <div className="relative flex-1 group">
                                            <motion.button
                                                whileTap={{ scale: example.enabled ? 0.95 : 1 }}
                                                onClick={() => example.enabled && launchExample.mutate(example.id)}
                                                disabled={!example.enabled || launchExample.isPending}
                                                className="w-full flex items-center justify-center gap-2 px-3 py-2 bg-primary-600 hover:bg-primary-700 disabled:bg-gray-700 disabled:cursor-not-allowed text-white text-sm rounded-lg transition-colors"
                                            >
                                                <Play className="w-4 h-4" />
                                                Launch
                                            </motion.button>
                                            {!example.enabled && (
                                                <div className="absolute bottom-full left-1/2 -translate-x-1/2 mb-2 px-2 py-1 bg-gray-800 text-gray-300 text-xs rounded whitespace-nowrap opacity-0 group-hover:opacity-100 transition-opacity pointer-events-none">
                                                    Coming soon
                                                </div>
                                            )}
                                        </div>

                                        <motion.button
                                            whileTap={{ scale: 0.95 }}
                                            className="p-2 bg-gray-800 hover:bg-gray-700 text-gray-400 rounded-lg transition-colors"
                                            title="View Details"
                                        >
                                            <Info className="w-4 h-4" />
                                        </motion.button>
                                    </div>
                                </div>

                                {/* Card Footer - Quick Actions */}
                                <div className="px-4 pb-4 flex items-center gap-2">
                                    {example.links?.docs && (
                                        <a
                                            href={example.links.docs}
                                            target="_blank"
                                            rel="noopener noreferrer"
                                            className="flex items-center gap-1 text-xs text-gray-500 hover:text-primary-400 transition-colors"
                                            title="Official Documentation"
                                        >
                                            <BookOpen className="w-3 h-3" />
                                            Docs
                                        </a>
                                    )}
                                    {example.links?.docs && example.links?.tutorial && (
                                        <span className="text-gray-700">•</span>
                                    )}
                                    {example.links?.tutorial && (
                                        <a
                                            href={example.links.tutorial}
                                            target="_blank"
                                            rel="noopener noreferrer"
                                            className="flex items-center gap-1 text-xs text-gray-500 hover:text-primary-400 transition-colors"
                                            title="Tutorial"
                                        >
                                            <Video className="w-3 h-3" />
                                            Tutorial
                                        </a>
                                    )}
                                    {example.links?.tutorial && example.links?.code && (
                                        <span className="text-gray-700">•</span>
                                    )}
                                    {example.links?.code && (
                                        <a
                                            href={example.links.code}
                                            target="_blank"
                                            rel="noopener noreferrer"
                                            className="flex items-center gap-1 text-xs text-gray-500 hover:text-primary-400 transition-colors"
                                            title="Source Code"
                                        >
                                            <Code className="w-3 h-3" />
                                            Code
                                        </a>
                                    )}
                                </div>
                            </motion.div>
                        )
                    })}
                </div>
            )}

            {filteredExamples?.length === 0 && (
                <div className="flex flex-col items-center justify-center h-64 text-center">
                    <Search className="w-12 h-12 text-gray-600 mb-4" />
                    <p className="text-gray-400 mb-2">No examples found</p>
                    <p className="text-gray-500 text-sm">Try adjusting your filters</p>
                </div>
            )}
        </PageContainer>
    )
}

export default Examples