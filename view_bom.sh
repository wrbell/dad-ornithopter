#!/usr/bin/env bash
# MCO-1D Mark 1 — View & Update Bill of Materials (BOM)
# Automatically checks pricing, regenerates docs/mark1/MCO-1D_Mark1_Bill_of_Materials.md,
# and displays the current procurement report.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/scripts/mco_bom_tool.py" --view
