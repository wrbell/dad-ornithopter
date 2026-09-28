#!/bin/bash
# MCO-1D Quick Launch: Simulation Mode
# Runs the 50 Hz flight computer executive in software-in-the-loop (SITL) mode

echo "========================================="
echo "   MCO-1D Flight Computer — Simulation   "
echo "========================================="
export PYTHONPATH="firmware:$PYTHONPATH"
python3 firmware/mco_main.py --sim "$@"
