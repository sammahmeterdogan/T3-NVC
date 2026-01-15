import re

# Read the file
with open('src/pages/SoArm101.jsx', 'r', encoding='utf-8') as f:
    content = f.read()

# Define the old pattern (the button we want to replace)
old_pattern = r'''                                        <button
                                            type="button"
                                            onClick=\{\(\) =>
                                                setJointControlMode\(\(prev\) => \(prev === 'basic' \? 'advanced' : 'basic'\)\)
                                            \}
                                            className="text-xs text-gray-500 hover:text-primary-400 transition-colors flex items-center gap-1">
                                            \{jointControlMode === 'basic' \? 'Advanced' : 'Basic'\}
                                            <Settings className="w-3 h-3" />
                                        </button>'''

# Define the new toggle
new_toggle = '''                                        <div className="flex bg-gray-800 rounded-lg p-1 border border-gray-700">
                                            <button
                                                type="button"
                                                onClick={() => handleJointControlModeSwitch('FK')}
                                                className={`flex items-center gap-1.5 px-2 py-1 text-xs font-medium rounded transition-colors ${jointControlMode === 'FK' ? 'bg-gray-700 text-white' : 'text-gray-400 hover:text-white'}`}
                                            >
                                                <Sliders className="w-3 h-3" />
                                                FK
                                            </button>
                                            <button
                                                type="button"
                                                onClick={() => handleJointControlModeSwitch('IK')}
                                                className={`flex items-center gap-1.5 px-2 py-1 text-xs font-medium rounded transition-colors ${jointControlMode === 'IK' ? 'bg-blue-600 text-white' : 'text-gray-400 hover:text-white'}`}
                                            >
                                                <Target className="w-3 h-3" />
                                                IK
                                            </button>
                                        </div>'''

# Try simple string replacement first
if "jointControlMode === 'basic' ? 'Advanced' : 'Basic'" in content:
    # Find the section and replace it
    lines = content.split('\n')
    new_lines = []
    i = 0
    while i < len(lines):
        if "jointControlMode === 'basic' ? 'Advanced' : 'Basic'" in lines[i]:
            # Found the line, now find the start and end of the button
            start = i
            while start > 0 and '<button' not in lines[start]:
                start -= 1
            end = i
            while end < len(lines) and '</button>' not in lines[end]:
                end += 1
            
            # Replace the entire button section
            indent = ' ' * 40  # Match the indentation
            new_lines.append(indent + '<div className="flex bg-gray-800 rounded-lg p-1 border border-gray-700">')
            new_lines.append(indent + '    <button')
            new_lines.append(indent + '        type="button"')
            new_lines.append(indent + '        onClick={() => handleJointControlModeSwitch(\'FK\')}')
            new_lines.append(indent + '        className={`flex items-center gap-1.5 px-2 py-1 text-xs font-medium rounded transition-colors ${jointControlMode === \'FK\' ? \'bg-gray-700 text-white\' : \'text-gray-400 hover:text-white\'}`}')
            new_lines.append(indent + '    >')
            new_lines.append(indent + '        <Sliders className="w-3 h-3" />')
            new_lines.append(indent + '        FK')
            new_lines.append(indent + '    </button>')
            new_lines.append(indent + '    <button')
            new_lines.append(indent + '        type="button"')
            new_lines.append(indent + '        onClick={() => handleJointControlModeSwitch(\'IK\')}')
            new_lines.append(indent + '        className={`flex items-center gap-1.5 px-2 py-1 text-xs font-medium rounded transition-colors ${jointControlMode === \'IK\' ? \'bg-blue-600 text-white\' : \'text-gray-400 hover:text-white\'}`}')
            new_lines.append(indent + '    >')
            new_lines.append(indent + '        <Target className="w-3 h-3" />')
            new_lines.append(indent + '        IK')
            new_lines.append(indent + '    </button>')
            new_lines.append(indent + '</div>')
            
            i = end + 1
        else:
            new_lines.append(lines[i])
            i += 1
    
    # Write back
    with open('src/pages/SoArm101.jsx', 'w', encoding='utf-8') as f:
        f.write('\n'.join(new_lines))
    
    print("✅ Successfully replaced toggle button!")
    print("FK/IK mode toggle is now visible in the UI")
else:
    print("❌ Pattern not found in file")
