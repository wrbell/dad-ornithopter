# MCO-1D Metachronal Cascade Ornithopter — Engineering Makefile
# Delegates standard workflows to ./mco (scripts/mco_cli.py)

.PHONY: help test tc01 sim gs bom cad cad-verify slice-plate docs doctor clean

help:
	@echo "======================================================================"
	@echo "  MCO-1D Metachronal Cascade Ornithopter — Standard Repo Workflows"
	@echo "======================================================================"
	@echo "  make test         Run full unit test verification suite (60+ suites)"
	@echo "  make tc01         Run automated TC-01 ground bench qualification"
	@echo "  make sim          Launch 50 Hz SITL flight computer simulation"
	@echo "  make gs           Launch Ground Station server & Gemini copilot"
	@echo "  make bom          Verify vendor pricing, MTOW & update BOM Markdown"
	@echo "  make cad-verify   Verify all CDR 3D print STLs via PrusaSlicer CLI"
	@echo "  make cad          Regenerate & package Wing Root CDR 3D print files"
	@echo "  make slice-plate  Slice PLATE_01 in PA12-CF with PrusaSlicer 2.9.6"
	@echo "  make docs         Regenerate all PDF installation & calibration guides"
	@echo "  make doctor       Audit repo cleanliness, markdown links & CAD/MCP"
	@echo "  make clean        Clean Python caches and temporary sliced output"
	@echo "======================================================================"

test:
	./mco test

tc01:
	./mco test --tc01

sim:
	./mco sim

gs:
	./mco gs

bom:
	./mco bom --view

cad-verify:
	./mco cad verify

cad:
	./mco cad package

slice-plate:
	./mco slice cad/print_package_wing_root/PLATE_01_PA12CF_ALL_STRUCTURAL_PARTS.stl --profile pa12cf_structural

docs:
	./mco docs all

doctor:
	./mco doctor

clean:
	rm -rf __pycache__ .pytest_cache firmware/__pycache__ scripts/__pycache__ tests/__pycache__ cad/sliced_output
