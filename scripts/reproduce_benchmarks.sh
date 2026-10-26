#!/usr/bin/env bash
# ==============================================================================
# Multi-Sensor Fusion SLAM & Dynamic Navigation Benchmark Reproduction Script
# IEEE Transactions on Robotics / ICRA Open-Source Artifact
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "========================================================================"
echo " [START] SmartCar AMR Degeneracy-Aware SLAM & Navigation Benchmarks"
echo " Workspace: ${WORKSPACE_ROOT}"
echo " Date: $(date)"
echo "========================================================================"

cd "${WORKSPACE_ROOT}"

# 1. Source ROS 2 Humble Environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo ">>> [1/5] Sourcing ROS 2 Humble environment..."
    source /opt/ros/humble/setup.bash
else
    echo ">>> [WARNING] /opt/ros/humble/setup.bash not found. Using current PATH."
fi

# 2. Build Workspace Packages
echo ">>> [2/5] Building ROS 2 packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 3. Execute Complete Unit Test Suite
echo ">>> [3/5] Running GoogleTest test suite..."
colcon test
colcon test-result --verbose

# 4. Generate Trajectories, Quantitative Benchmarks & Figures
echo ">>> [4/5] Executing quantitative statistical benchmarks..."
python3 evaluate_trajectories.py
python3 benchmark_evaluation.py
python3 generate_paper_figures.py
python3 generate_academic_figures.py

# 5. Compile Publication-Grade IEEE LaTeX Paper
echo ">>> [5/5] Compiling IEEE Transactions LaTeX Paper to PDF..."
if which pdflatex >/dev/null 2>&1; then
    cd "${WORKSPACE_ROOT}/paper"
    pdflatex -interaction=nonstopmode paper.tex >/dev/null 2>&1
    bibtex paper >/dev/null 2>&1
    pdflatex -interaction=nonstopmode paper.tex >/dev/null 2>&1
    pdflatex -interaction=nonstopmode paper.tex >/dev/null 2>&1
    echo "  [OK] Successfully compiled: ${WORKSPACE_ROOT}/paper/paper.pdf"
    cd "${WORKSPACE_ROOT}"
else
    echo "  [SKIP] pdflatex not found. Skipping PDF generation."
fi

echo "========================================================================"
echo " [COMPLETE] All benchmarks, tests, figures, and paper compiled cleanly!"
echo " - Paper PDF:    paper/paper.pdf"
echo " - Paper Source: paper/paper.tex & paper/PAPER.md"
echo " - Slides (HTML):slides/presentation.html"
echo " - Video Demo:   simulation_60s.mp4"
echo "========================================================================"
