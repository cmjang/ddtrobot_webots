#!/bin/bash

# Usage: run_checks.sh <path>
# This script runs ament_copyright, ament_cppcheck, ament_cpplint, ament_lint_cmake, and ament_xmllint checks.
# Usage example: ./run_checks.sh /path/to/directory

if [ $# -ne 1 ]; then
    echo "Usage: $0 <path>"
    echo "This script runs ament_copyright, ament_cppcheck, ament_cpplint, ament_lint_cmake, and ament_xmllint checks."
    echo "Usage example: $0 /path/to/directory"
    exit 1
fi

exclude_dirs=("build" "install" "log")

cpp_file_types=(".cpp" ".c" ".h" ".cc" ".hpp")

echo "##################  Run ament_copyright"

thread_number=10

execute_copyright() {
    ament_copyright "$@"
}
export -f execute_copyright

for type in "${cpp_file_types[@]}"; do
    find $1 -type d \( -name "${exclude_dirs[0]}" -o -name "${exclude_dirs[1]}" -o -name "${exclude_dirs[2]}" \) -prune -false -o -type f -name "*$type" | xargs -P $thread_number -I {} bash -c 'execute_copyright "$@"' _ {}
done

echo "##################  Run ament_cppcheck"

execute_cppcheck() {
    ament_cppcheck "$@"
}
export -f execute_cppcheck

for type in "${cpp_file_types[@]}"; do
    find $1 -type d \( -name "${exclude_dirs[0]}" -o -name "${exclude_dirs[1]}" -o -name "${exclude_dirs[2]}" \) -prune -false -o  -type f -name "*$type" | xargs -P $thread_number -I {} bash -c 'execute_cppcheck "$@"' _ {}
done

echo "##################  Run ament_cpplint"
execute_cpplint() {
    ament_cpplint "$@"
}
export -f execute_cpplint
for type in "${cpp_file_types[@]}"; do
    find $1 -type d \( -name "${exclude_dirs[0]}" -o -name "${exclude_dirs[1]}" -o -name "${exclude_dirs[2]}" \) -prune -false -o  -type f -name "*$type" | xargs -P $thread_number -I {} bash -c 'execute_cpplint "$@" | grep -v "Done"| grep -v "Using"| grep -v "^$"' _ {}
done
echo "##################  Run ament_lint_cmake"
cmake_file_types=(".txt")

execute_lint_cmake() {
    ament_lint_cmake "$@"
}
export -f execute_lint_cmake

for type in "${cmake_file_types[@]}"; do
    find $1 -type d \( -name "${exclude_dirs[0]}" -o -name "${exclude_dirs[1]}" -o -name "${exclude_dirs[2]}" \) -prune -false -o  -type f -name "*$type" | xargs -P $thread_number -I {} bash -c 'execute_lint_cmake "$@" | grep -v "^$"' _ {}
done
echo "##################  Run ament_xmllint"

xml_file_types=(".xml")

execute_xmllint() {
    ament_xmllint "$@"
}
export -f execute_xmllint

for type in "${xml_file_types[@]}"; do
    find $1 -type d \( -name "${exclude_dirs[0]}" -o -name "${exclude_dirs[1]}" -o -name "${exclude_dirs[2]}" \) -prune -false -o  -type f -name "*$type" | xargs -P $thread_number -I {} bash -c 'execute_xmllint "$@" |grep -v "is valid" | grep -v "^$"' _ {}
done

echo "All checks complete."

bash format_all_c_files.bash $1
