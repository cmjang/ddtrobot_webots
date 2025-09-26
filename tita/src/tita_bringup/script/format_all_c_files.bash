#!/bin/bash

# Usage: format_files.sh <path>
# This script finds and formats .hpp, .h, .cpp, .c, .cc files using ament_uncrustify.
# It excludes directories named "build", "install", and "log".
# Usage example: bash /path/to/directory

if [ $# -ne 1 ]; then
    echo "Usage: $0 <path>"
    echo "This script finds and formats .hpp, .h, .cpp, .c, .cc files using ament_uncrustify."
    echo "It excludes directories named 'build', 'install', and 'log'."
    echo "Usage example: $0 /path/to/directory"
    exit 1
fi

# Find and format .hpp, .h, .cpp, .c, .cc files using ament_uncrustify

# Find all relevant file types and store them in an array
file_types=(".hpp" ".h" ".cpp" ".c" ".cc")

# Exclude directories named "build", "install", and "log"
exclude_dirs=("build" "install" "log")

# Function to format files using ament_uncrustify
format_files() {
    ament_clang_format --reformat "$@"
}

export -f format_files

echo "Formatting......"

# Find files of the current type, excluding specified directories, and format them using xargs
for type in "${file_types[@]}"; do
    find $1 -type d \( -name "${exclude_dirs[0]}" -o -name "${exclude_dirs[1]}" -o -name "${exclude_dirs[2]}" \) -prune -false -o -type f -name "*$type" | xargs -P 4 -I {} bash -c 'format_files "$@"' _ {} | grep -v 'No code style divergence' | grep -v "No problems found"
done

echo "Formatting complete."

