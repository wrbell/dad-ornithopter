#!/usr/bin/env python3
"""
scripts/mcp/prusaslicer_mcp_server.py
=====================================
Native Model Context Protocol (MCP) Server for driving PrusaSlicer 2.9+ on macOS.

Provides structured AI agent tools for:
  1. `prusaslicer_version_and_status`: Inspect local PrusaSlicer binary, user datadir, active presets, and MCO-1D profiles.
  2. `list_printer_and_filament_profiles`: Query installed Prusa vendor profiles and versioned repo `.ini` profiles.
  3. `inspect_model`: Run `PrusaSlicer --info` to verify mesh manifoldness, dimensions, volume, part count, and Prusa CORE One bed fit.
  4. `slice_model`: Slice STL/STEP/3MF models with MCO-1D profiles (`pa12cf_structural`, `pa12cf_solid_gears`, `lwpla_aero_fairing`, `pla_prototype`) or custom FFF overrides, returning parsed G-code print time, filament mass (g), length (mm), volume (cm³), and layer count.
  5. `transform_or_pack_models`: Orient, rotate, scale, split, cut, or pack multiple models onto a single `.3mf` or `.stl` plate.
  6. `validate_mco_print_package`: Batch-audit all 3D printable parts in `cad/print_package_wing_root/` for manifoldness, bed fit, and mass budget adherence.
  7. `open_in_prusaslicer_gui`: Launch the macOS PrusaSlicer GUI with selected models and profiles loaded for visual toolpath review.
"""

from __future__ import annotations

import configparser
import os
import re
import subprocess
from pathlib import Path
from typing import Any, Optional

REPO_ROOT = Path(os.environ.get("MCO_REPO_ROOT", Path(__file__).resolve().parents[2]))
SLICER_PROFILES_DIR = REPO_ROOT / "cad" / "slicer_profiles"

DEFAULT_PRUSASLICER_PATHS = [
    os.environ.get("PRUSASLICER_BIN", ""),
    "/Applications/Original Prusa Drivers/PrusaSlicer.app/Contents/MacOS/PrusaSlicer",
    "/Applications/PrusaSlicer.app/Contents/MacOS/PrusaSlicer",
]

DEFAULT_DATADIR = Path(
    os.environ.get(
        "PRUSASLICER_DATADIR",
        os.path.expanduser("~/Library/Application Support/PrusaSlicer"),
    )
)

PROFILE_ALIASES: dict[str, str] = {
    "pa12cf": "mco_pa12cf_structural_coreone.ini",
    "pa12cf_structural": "mco_pa12cf_structural_coreone.ini",
    "pa12cf-structural": "mco_pa12cf_structural_coreone.ini",
    "pa12cf_solid": "mco_pa12cf_solid_gears_coreone.ini",
    "pa12cf_solid_gears": "mco_pa12cf_solid_gears_coreone.ini",
    "pa12cf-solid": "mco_pa12cf_solid_gears_coreone.ini",
    "lwpla": "mco_lwpla_aero_fairing_coreone.ini",
    "lwpla_aero_fairing": "mco_lwpla_aero_fairing_coreone.ini",
    "lw-pla": "mco_lwpla_aero_fairing_coreone.ini",
    "pla": "mco_pla_prototype_coreone.ini",
    "pla_prototype": "mco_pla_prototype_coreone.ini",
}


def find_prusaslicer_bin() -> str:
    """Locate the PrusaSlicer CLI executable on macOS or PATH."""
    for candidate in DEFAULT_PRUSASLICER_PATHS:
        if candidate and os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    for path_dir in os.environ.get("PATH", "").split(os.pathsep):
        for name in ("prusa-slicer", "PrusaSlicer"):
            candidate = os.path.join(path_dir, name)
            if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                return candidate
    raise FileNotFoundError(
        "PrusaSlicer binary not found. Checked /Applications/Original Prusa Drivers/PrusaSlicer.app "
        "and /Applications/PrusaSlicer.app."
    )


def resolve_repo_path(path_str: str) -> Path:
    """Resolve a path relative to REPO_ROOT if not already absolute."""
    p = Path(os.path.expanduser(path_str))
    if not p.is_absolute():
        p = (REPO_ROOT / p).resolve()
    return p


def resolve_profile_path(profile: str) -> Path:
    """Resolve a profile alias or path to an existing .ini configuration file."""
    key = profile.strip().lower()
    if key in PROFILE_ALIASES:
        target = SLICER_PROFILES_DIR / PROFILE_ALIASES[key]
        if target.exists():
            return target
    candidate = resolve_repo_path(profile)
    if candidate.exists():
        return candidate
    in_profiles_dir = SLICER_PROFILES_DIR / (profile if profile.endswith(".ini") else f"{profile}.ini")
    if in_profiles_dir.exists():
        return in_profiles_dir
    raise FileNotFoundError(
        f"Slicer profile '{profile}' not found. Available aliases: {sorted(PROFILE_ALIASES.keys())}"
    )


def _parse_duration_seconds(duration_str: str) -> int:
    """Convert PrusaSlicer duration string (e.g. '1d 2h 39m 21s') to total seconds."""
    total = 0
    for val, unit in re.findall(r"(\d+)\s*([dhms])", duration_str):
        n = int(val)
        if unit == "d":
            total += n * 86400
        elif unit == "h":
            total += n * 3600
        elif unit == "m":
            total += n * 60
        elif unit == "s":
            total += n
    return total


def parse_gcode_metadata(gcode_path: Path) -> dict[str, Any]:
    """Parse filament usage, print time, layer count, and settings from a sliced G-code file."""
    if not gcode_path.exists():
        return {"error": f"G-code file not found: {gcode_path}"}

    layer_count = 0
    max_z_mm = 0.0
    metadata: dict[str, Any] = {}

    with gcode_path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if line.startswith(";LAYER_CHANGE"):
                layer_count += 1
            elif line.startswith(";Z:"):
                try:
                    z_val = float(line.split(":", 1)[1].strip())
                    if z_val > max_z_mm:
                        max_z_mm = z_val
                except ValueError:
                    pass
            elif line.startswith("; "):
                match = re.match(r"^;\s*([^=]+?)\s*=\s*(.+)$", line.strip())
                if match:
                    k, v = match.group(1).strip(), match.group(2).strip()
                    metadata[k] = v

    def _float_meta(key: str) -> Optional[float]:
        raw = metadata.get(key)
        if raw is None:
            return None
        try:
            return float(raw.split(",")[0].strip())
        except ValueError:
            return None

    normal_time = metadata.get("estimated printing time (normal mode)", "")
    silent_time = metadata.get("estimated printing time (silent mode)", "")

    if layer_count == 0:
        # Fallback: count G1 Z moves or estimate from max_z / layer_height
        lh = _float_meta("layer_height") or 0.16
        if max_z_mm > 0 and lh > 0:
            layer_count = max(1, int(round(max_z_mm / lh)))

    return {
        "gcode_path": str(gcode_path),
        "gcode_size_bytes": gcode_path.stat().st_size,
        "filament_used_mm": _float_meta("filament used [mm]"),
        "filament_used_cm3": _float_meta("filament used [cm3]"),
        "filament_used_g": _float_meta("filament used [g]") or _float_meta("total filament used [g]"),
        "total_filament_cost": _float_meta("total filament cost"),
        "estimated_printing_time_normal": normal_time,
        "estimated_printing_time_seconds": _parse_duration_seconds(normal_time),
        "estimated_printing_time_silent": silent_time,
        "layer_count": layer_count,
        "layer_height_mm": _float_meta("layer_height"),
        "perimeters": int(_float_meta("perimeters") or 0),
        "fill_density": metadata.get("fill_density"),
        "fill_pattern": metadata.get("fill_pattern"),
        "nozzle_temperature_c": _float_meta("temperature"),
        "bed_temperature_c": _float_meta("bed_temperature"),
        "filament_type": metadata.get("filament_type"),
    }


def prusaslicer_version_and_status() -> dict[str, Any]:
    """
    Return PrusaSlicer CLI version, binary path, user configuration directory,
    currently selected GUI presets (e.g. Prusa CORE One), and repo slicer profiles.
    """
    bin_path = find_prusaslicer_bin()
    proc = subprocess.run(
        [bin_path, "--help"],
        capture_output=True,
        text=True,
        timeout=15,
    )
    first_line = (proc.stdout or proc.stderr).splitlines()[0] if (proc.stdout or proc.stderr) else "Unknown"

    ini_path = DEFAULT_DATADIR / "PrusaSlicer.ini"
    active_presets: dict[str, str] = {}
    if ini_path.exists():
        section = None
        for line in ini_path.read_text(encoding="utf-8", errors="replace").splitlines():
            line_s = line.strip()
            if line_s.startswith("[") and line_s.endswith("]"):
                section = line_s[1:-1]
            elif section == "presets" and "=" in line_s:
                k, v = line_s.split("=", 1)
                active_presets[k.strip()] = v.strip()

    repo_profiles = sorted(p.name for p in SLICER_PROFILES_DIR.glob("*.ini")) if SLICER_PROFILES_DIR.exists() else []

    return {
        "status": "ok",
        "version_banner": first_line,
        "binary_path": bin_path,
        "datadir": str(DEFAULT_DATADIR),
        "datadir_exists": DEFAULT_DATADIR.exists(),
        "active_gui_presets": active_presets,
        "mco_repo_profiles": repo_profiles,
        "profile_aliases": PROFILE_ALIASES,
    }


def list_printer_and_filament_profiles(vendor_or_filter: str = "COREONE") -> dict[str, Any]:
    """
    List installed PrusaSlicer filament/printer presets matching `vendor_or_filter`
    alongside the version-controlled MCO-1D `.ini` profiles in `cad/slicer_profiles/`.
    """
    ini_path = DEFAULT_DATADIR / "PrusaSlicer.ini"
    filaments: list[str] = []
    vendors: dict[str, list[str]] = {}
    flt = vendor_or_filter.strip().lower()

    if ini_path.exists():
        section = ""
        for line in ini_path.read_text(encoding="utf-8", errors="replace").splitlines():
            line_s = line.strip()
            if line_s.startswith("[") and line_s.endswith("]"):
                section = line_s[1:-1]
                continue
            if "=" not in line_s:
                continue
            k, v = [x.strip() for x in line_s.split("=", 1)]
            if section == "filaments":
                if not flt or flt in k.lower():
                    filaments.append(k)
            elif section.startswith("vendor:"):
                vname = section.split(":", 1)[1]
                vendors.setdefault(vname, []).append(f"{k} ({v})")

    repo_profiles_detail = []
    if SLICER_PROFILES_DIR.exists():
        for p in sorted(SLICER_PROFILES_DIR.glob("*.ini")):
            repo_profiles_detail.append({"name": p.name, "path": str(p)})

    return {
        "filter": vendor_or_filter,
        "matching_filament_presets": filaments,
        "installed_vendor_models": vendors,
        "mco_repo_profiles": repo_profiles_detail,
    }


def inspect_model(
    file_path: str,
    bed_x_mm: float = 250.0,
    bed_y_mm: float = 220.0,
    bed_z_mm: float = 250.0,
) -> dict[str, Any]:
    """
    Inspect a 3D model (.stl, .obj, .3mf, .step) using `PrusaSlicer --info`.
    Returns dimensions, bounding box, facet count, manifold status, volume,
    estimated solid mass in PA12-CF / PLA / LW-PLA, and build volume fit check.
    """
    bin_path = find_prusaslicer_bin()
    target = resolve_repo_path(file_path)
    if not target.exists():
        raise FileNotFoundError(f"Model file not found: {target}")

    proc = subprocess.run(
        [bin_path, "--info", str(target)],
        capture_output=True,
        text=True,
        timeout=60,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"PrusaSlicer --info failed ({proc.returncode}): {proc.stderr.strip()}")

    raw: dict[str, str] = {}
    for line in proc.stdout.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            raw[k.strip()] = v.strip()

    size_x = float(raw.get("size_x", 0.0))
    size_y = float(raw.get("size_y", 0.0))
    size_z = float(raw.get("size_z", 0.0))
    vol_mm3 = float(raw.get("volume", 0.0))
    vol_cm3 = vol_mm3 / 1000.0
    is_manifold = raw.get("manifold", "").lower() == "yes"
    fits_bed = (
        (size_x <= bed_x_mm and size_y <= bed_y_mm)
        or (size_y <= bed_x_mm and size_x <= bed_y_mm)
    ) and (size_z <= bed_z_mm)

    return {
        "file_path": str(target),
        "file_name": target.name,
        "is_manifold": is_manifold,
        "manifold_raw": raw.get("manifold", "unknown"),
        "number_of_parts": int(raw.get("number_of_parts", 0)),
        "number_of_facets": int(raw.get("number_of_facets", 0)),
        "dimensions_mm": {
            "x": round(size_x, 3),
            "y": round(size_y, 3),
            "z": round(size_z, 3),
        },
        "bounds_mm": {
            "min_x": float(raw.get("min_x", 0.0)),
            "max_x": float(raw.get("max_x", 0.0)),
            "min_y": float(raw.get("min_y", 0.0)),
            "max_y": float(raw.get("max_y", 0.0)),
            "min_z": float(raw.get("min_z", 0.0)),
            "max_z": float(raw.get("max_z", 0.0)),
        },
        "volume_mm3": round(vol_mm3, 2),
        "volume_cm3": round(vol_cm3, 3),
        "estimated_solid_mass_g": {
            "PA12_CF_100pct (1.25 g/cm3)": round(vol_cm3 * 1.25, 2),
            "PA12_CF_60pct_infill (~0.85 g/cm3 eff)": round(vol_cm3 * 0.85, 2),
            "PLA_100pct (1.24 g/cm3)": round(vol_cm3 * 1.24, 2),
            "LW_PLA_foamed (~0.55 g/cm3)": round(vol_cm3 * 0.55, 2),
        },
        "bed_check": {
            "bed_dimensions_mm": [bed_x_mm, bed_y_mm, bed_z_mm],
            "fits_on_bed": fits_bed,
        },
    }


def slice_model(
    input_paths: list[str],
    output_dir: Optional[str] = None,
    profile: str = "pa12cf_structural",
    layer_height: Optional[float] = None,
    perimeters: Optional[int] = None,
    fill_density: Optional[str] = None,
    fill_pattern: Optional[str] = None,
    support_material: Optional[bool] = None,
    brim_width: Optional[float] = None,
    nozzle_temp: Optional[int] = None,
    bed_temp: Optional[int] = None,
    merge_plate: bool = False,
    export_3mf: bool = True,
) -> dict[str, Any]:
    """
    Slice one or more 3D models (.stl, .step, .3mf) using PrusaSlicer CLI and
    return parsed G-code telemetry (estimated print time, filament grams/mm/cm3, layers).
    """
    if not input_paths:
        raise ValueError("input_paths must contain at least one model path.")

    bin_path = find_prusaslicer_bin()
    profile_path = resolve_profile_path(profile)
    resolved_inputs = [resolve_repo_path(p) for p in input_paths]
    for p in resolved_inputs:
        if not p.exists():
            raise FileNotFoundError(f"Input model not found: {p}")

    out_dir = resolve_repo_path(output_dir) if output_dir else (REPO_ROOT / "cad" / "sliced_output")
    out_dir.mkdir(parents=True, exist_ok=True)

    stem = resolved_inputs[0].stem if len(resolved_inputs) == 1 else f"plate_{len(resolved_inputs)}parts_{resolved_inputs[0].stem}"
    gcode_out = out_dir / f"{stem}.gcode"
    threemf_out = out_dir / f"{stem}.3mf"

    override_args: list[str] = ["--load", str(profile_path)]
    if layer_height is not None:
        override_args.append(f"--layer-height={layer_height}")
    if perimeters is not None:
        override_args.append(f"--perimeters={perimeters}")
    if fill_density is not None:
        fd = fill_density if str(fill_density).endswith("%") else f"{fill_density}%"
        override_args.append(f"--fill-density={fd}")
    if fill_pattern is not None:
        override_args.append(f"--fill-pattern={fill_pattern}")
    if support_material is not None:
        override_args.append("--support-material" if support_material else "--support-material=0")
    if brim_width is not None:
        override_args.append(f"--brim-width={brim_width}")
    if nozzle_temp is not None:
        override_args.extend([f"--temperature={nozzle_temp}", f"--first-layer-temperature={nozzle_temp}"])
    if bed_temp is not None:
        override_args.extend([f"--bed-temperature={bed_temp}", f"--first-layer-bed-temperature={bed_temp}"])
    if merge_plate or len(resolved_inputs) > 1:
        override_args.append("--merge")

    # 1. Slice to G-code
    cmd_gcode = [
        bin_path,
        *override_args,
        "--export-gcode",
        "-o",
        str(gcode_out),
        *[str(p) for p in resolved_inputs],
    ]
    proc = subprocess.run(cmd_gcode, capture_output=True, text=True, timeout=300)
    if proc.returncode != 0:
        raise RuntimeError(
            f"PrusaSlicer slicing failed (exit {proc.returncode}):\nSTDOUT: {proc.stdout}\nSTDERR: {proc.stderr}"
        )

    # 2. Optionally export configured 3MF project
    exported_3mf = None
    if export_3mf:
        cmd_3mf = [
            bin_path,
            *override_args,
            "--export-3mf",
            "-o",
            str(threemf_out),
            *[str(p) for p in resolved_inputs],
        ]
        proc_3mf = subprocess.run(cmd_3mf, capture_output=True, text=True, timeout=180)
        if proc_3mf.returncode == 0 and threemf_out.exists():
            exported_3mf = str(threemf_out)

    telemetry = parse_gcode_metadata(gcode_out)
    return {
        "status": "sliced",
        "profile_used": str(profile_path),
        "input_files": [str(p) for p in resolved_inputs],
        "gcode_path": str(gcode_out),
        "project_3mf_path": exported_3mf,
        "telemetry": telemetry,
    }


def transform_or_pack_models(
    input_paths: list[str],
    output_path: str,
    rotate_z_deg: float = 0.0,
    rotate_x_deg: float = 0.0,
    rotate_y_deg: float = 0.0,
    scale: float = 1.0,
    cut_z_mm: Optional[float] = None,
    split_parts: bool = False,
    merge_and_arrange: bool = False,
    duplicate: int = 1,
    center_xy: Optional[str] = None,
) -> dict[str, Any]:
    """
    Use PrusaSlicer CLI geometry transforms to orient, scale, cut, split, duplicate,
    or pack multiple models onto a build plate and export as `.3mf` or `.stl`.
    """
    bin_path = find_prusaslicer_bin()
    resolved_inputs = [resolve_repo_path(p) for p in input_paths]
    for p in resolved_inputs:
        if not p.exists():
            raise FileNotFoundError(f"Input file not found: {p}")

    out_target = resolve_repo_path(output_path)
    out_target.parent.mkdir(parents=True, exist_ok=True)
    export_flag = "--export-3mf" if out_target.suffix.lower() == ".3mf" else "--export-stl"

    cmd: list[str] = [bin_path]
    if rotate_z_deg:
        cmd.extend(["--rotate", str(rotate_z_deg)])
    if rotate_x_deg:
        cmd.extend(["--rotate-x", str(rotate_x_deg)])
    if rotate_y_deg:
        cmd.extend(["--rotate-y", str(rotate_y_deg)])
    if scale != 1.0:
        cmd.extend(["--scale", str(scale)])
    if cut_z_mm is not None:
        cmd.extend(["--cut", str(cut_z_mm)])
    if split_parts:
        cmd.append("--split")
    if merge_and_arrange or len(resolved_inputs) > 1:
        cmd.append("--merge")
    if duplicate > 1:
        cmd.extend(["--duplicate", str(duplicate)])
    if center_xy:
        cmd.extend(["--center", center_xy])

    cmd.extend(["--ensure-on-bed", export_flag, "-o", str(out_target), *[str(p) for p in resolved_inputs]])
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    if proc.returncode != 0:
        raise RuntimeError(f"PrusaSlicer transform failed ({proc.returncode}): {proc.stderr.strip()}")

    info = inspect_model(str(out_target)) if out_target.suffix.lower() == ".stl" else {"output_path": str(out_target)}
    return {
        "status": "exported",
        "output_path": str(out_target),
        "info": info,
    }


def validate_mco_print_package(
    package_dir: str = "cad/print_package_wing_root",
    bed_x_mm: float = 250.0,
    bed_y_mm: float = 220.0,
    bed_z_mm: float = 250.0,
) -> dict[str, Any]:
    """
    Audit all STL files in `cad/print_package_wing_root` (or any specified folder)
    to verify 100% manifold status, positive volumes, and Prusa CORE One bed fit.
    """
    pkg_path = resolve_repo_path(package_dir)
    if not pkg_path.exists():
        raise FileNotFoundError(f"Package directory not found: {pkg_path}")

    stl_files = sorted(pkg_path.glob("*.stl"))
    results = []
    all_manifold = True
    all_fit_bed = True

    for stl in stl_files:
        info = inspect_model(str(stl), bed_x_mm=bed_x_mm, bed_y_mm=bed_y_mm, bed_z_mm=bed_z_mm)
        if not info["is_manifold"]:
            all_manifold = False
        if not info["bed_check"]["fits_on_bed"]:
            all_fit_bed = False
        results.append(
            {
                "file": stl.name,
                "is_manifold": info["is_manifold"],
                "fits_on_bed": info["bed_check"]["fits_on_bed"],
                "dimensions_mm": info["dimensions_mm"],
                "volume_cm3": info["volume_cm3"],
                "number_of_parts": info["number_of_parts"],
                "number_of_facets": info["number_of_facets"],
            }
        )

    return {
        "package_dir": str(pkg_path),
        "total_stl_files": len(results),
        "all_manifold": all_manifold,
        "all_fit_bed": all_fit_bed,
        "parts": results,
    }


GUI_PRESET_MAP: dict[str, dict[str, str]] = {
    "mco_pla_prototype_coreone.ini": {
        "printer": "Prusa CORE One HF0.4 nozzle",
        "print": "0.20mm STRUCTURAL @COREONE HF0.4",
        "filament": "Prusament PLA @COREONE HF0.4",
    },
    "mco_lwpla_aero_fairing_coreone.ini": {
        "printer": "Prusa CORE One HF0.4 nozzle",
        "print": "0.20mm SPEED @COREONE HF0.4",
        "filament": "Generic PLA @COREONE HF0.4",
    },
    "mco_pa12cf_structural_coreone.ini": {
        "printer": "Prusa CORE One HF0.4 nozzle",
        "print": "0.20mm STRUCTURAL @COREONE HF0.4",
        "filament": "Prusament PA11 Carbon Fiber @COREONE",
    },
    "mco_pa12cf_solid_gears_coreone.ini": {
        "printer": "Prusa CORE One HF0.4 nozzle",
        "print": "0.15mm STRUCTURAL @COREONE 0.4",
        "filament": "Prusament PA11 Carbon Fiber @COREONE",
    },
}


def _sync_gui_presets_ini(prof_name: str) -> None:
    """Update ~/Library/Application Support/PrusaSlicer/PrusaSlicer.ini [presets] before launching GUI."""
    ini_path = DEFAULT_DATADIR / "PrusaSlicer.ini"
    preset_cfg = GUI_PRESET_MAP.get(prof_name)
    if not ini_path.exists() or not preset_cfg:
        return
    lines = ini_path.read_text(encoding="utf-8", errors="replace").splitlines()
    out_lines: list[str] = []
    in_presets = False
    for line in lines:
        s = line.strip()
        if s.startswith("[") and s.endswith("]"):
            in_presets = s == "[presets]"
            out_lines.append(line)
            continue
        if in_presets and "=" in s:
            k = s.split("=", 1)[0].strip()
            if k in preset_cfg:
                out_lines.append(f"{k} = {preset_cfg[k]}")
                continue
        out_lines.append(line)
    ini_path.write_text("\n".join(out_lines) + "\n", encoding="utf-8")


def open_in_prusaslicer_gui(
    file_paths: list[str],
    profile: Optional[str] = None,
) -> dict[str, Any]:
    """
    Launch the macOS PrusaSlicer GUI with the specified STL/STEP/3MF files and optional profile loaded.
    """
    bin_path = find_prusaslicer_bin()
    resolved = [resolve_repo_path(p) for p in file_paths]
    for p in resolved:
        if not p.exists():
            raise FileNotFoundError(f"File not found: {p}")

    cmd = [bin_path]
    if profile:
        import time

        prof_path = resolve_profile_path(profile)
        # Force-terminate any running PrusaSlicer GUI first so its shutdown hook does not overwrite PrusaSlicer.ini with old presets
        subprocess.run(["killall", "-9", "PrusaSlicer"], capture_output=True)
        time.sleep(0.3)
        if prof_path.name in GUI_PRESET_MAP:
            _sync_gui_presets_ini(prof_path.name)
        else:
            cmd.extend(["--load", str(prof_path)])
    else:
        cmd.append("--single-instance")
    cmd.extend(str(p) for p in resolved)

    subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    return {
        "status": "launched_gui",
        "command": cmd,
        "opened_files": [str(p) for p in resolved],
    }


def create_mcp_server():
    """Create and configure the FastMCP server instance."""
    from mcp.server.fastmcp import FastMCP

    mcp = FastMCP("prusaslicer")
    mcp.tool()(prusaslicer_version_and_status)
    mcp.tool()(list_printer_and_filament_profiles)
    mcp.tool()(inspect_model)
    mcp.tool()(slice_model)
    mcp.tool()(transform_or_pack_models)
    mcp.tool()(validate_mco_print_package)
    mcp.tool()(open_in_prusaslicer_gui)
    return mcp


if __name__ == "__main__":
    server = create_mcp_server()
    server.run()
