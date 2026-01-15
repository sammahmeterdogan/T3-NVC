import re

# Read the file
with open('src/pages/SoArm101.jsx', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Find and fix the malformed section (lines 595-601 need to be removed)
# They contain an unclosed button tag
new_lines = []
skip_until = -1

for i, line in enumerate(lines):
    line_num = i + 1
    
    # Skip lines 595-601 (the malformed button opening)
    if 595 <= line_num <= 601:
        continue
    
    new_lines.append(line)

# Write back
with open('src/pages/SoArm101.jsx', 'w', encoding='utf-8') as f:
    f.writelines(new_lines)

print("✅ Fixed malformed JSX!")
print("Removed unclosed button tag (lines 595-601)")
