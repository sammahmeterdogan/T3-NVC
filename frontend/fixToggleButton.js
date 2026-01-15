// Quick fix script for FK/IK toggle button
// This shows the exact replacement needed

console.log('='.repeat(60))
console.log('FK/IK Toggle Button Fix')
console.log('='.repeat(60))
console.log('\nFile: frontend/src/pages/SoArm101.jsx')
console.log('Lines: 593-605')
console.log('\nREPLACE THIS:')
console.log('─'.repeat(60))
console.log(`
                                    <div className="flex items-center justify-between mb-3">
                                        <span className="text-gray-400 text-xs font-medium">Joint Control Mode</span>
                                        <button
                                            type="button"
                                            onClick={() =>
                                                setJointControlMode((prev) => (prev === 'basic' ? 'advanced' : 'basic'))
                                            }
                                            className="text-xs text-gray-500 hover:text-primary-400 transition-colors flex items-center gap-1">
                                            {jointControlMode === 'basic' ? 'Advanced' : 'Basic'}
                                            <Settings className="w-3 h-3" />
                                        </button>
                                    </div>
`)
console.log('\nWITH THIS:')
console.log('─'.repeat(60))
console.log(`
                                    <div className="flex items-center justify-between mb-3">
                                        <span className="text-gray-400 text-xs font-medium">Joint Control Mode</span>
                                        <div className="flex bg-gray-800 rounded-lg p-1 border border-gray-700">
                                            <button
                                                type="button"
                                                onClick={() => handleJointControlModeSwitch('FK')}
                                                className={\`flex items-center gap-1.5 px-2 py-1 text-xs font-medium rounded transition-colors \${jointControlMode === 'FK' ? 'bg-gray-700 text-white' : 'text-gray-400 hover:text-white'}\`}
                                            >
                                                <Sliders className="w-3 h-3" />
                                                FK
                                            </button>
                                            <button
                                                type="button"
                                                onClick={() => handleJointControlModeSwitch('IK')}
                                                className={\`flex items-center gap-1.5 px-2 py-1 text-xs font-medium rounded transition-colors \${jointControlMode === 'IK' ? 'bg-blue-600 text-white' : 'text-gray-400 hover:text-white'}\`}
                                            >
                                                <Target className="w-3 h-3" />
                                                IK
                                            </button>
                                        </div>
                                    </div>
`)
console.log('\n' + '='.repeat(60))
console.log('Manual fix required - automated replacement failed due to whitespace')
console.log('='.repeat(60))
