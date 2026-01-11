#!/bin/bash
# Script to run all pytest tests while avoiding ROS ament dependencies
# and module import conflicts

set -e

# Disable ROS pytest plugins that cause conflicts
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

# Use python or python3 depending on availability
if command -v python &> /dev/null; then
    PYTHON_CMD=python
elif command -v python3 &> /dev/null; then
    PYTHON_CMD=python3
else
    echo "Error: Neither python nor python3 found"
    exit 1
fi

# Find all actual test files (excluding ROS linter tests)
test_files=$(find workspace/src -name "test_*.py" -type f | \
    grep -v test_copyright.py | \
    grep -v test_flake8.py | \
    grep -v test_pep257.py)

# Run pytest on each test file individually to avoid import conflicts
# This is necessary because multiple test directories have files with the same names
echo "Running pytest on individual test files..."
for test_file in $test_files; do
    echo "Testing: $test_file"
    $PYTHON_CMD -m pytest "$test_file" -v --tb=short || exit 1
done

echo "All tests passed!"
