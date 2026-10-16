# MCO-1 Master Fasteners, Bolts & Brass Inserts Mass Budget

This document establishes the **authoritative, item-by-item hardware inventory and mass budget** for every machine screw, bolt, nut, lock washer, and brass heat-set insert across the entire physical MCO-1 aircraft build.

---

## 1. Executive Summary: Total Hardware Mass Budget

Across the entire airframe, drivetrain, actuation suite, and electronics bay, the fastener and threaded insert inventory totals:

$$\mathbf{201.0\text{ g}}\quad (0.201\text{ kg} \text{ / } 0.443\text{ lb})\quad \text{across}\quad \mathbf{400\text{ individual hardware pieces}}$$

| Hardware Category | Total Quantity | Total Mass | Primary Material | Function & Location |
| :--- | :---: | :---: | :--- | :--- |
| **Structural Machine Screws & Bolts** | **201 pieces** | **$167.4\text{ g}$** | Grade 12.9 Alloy Steel / Ti | Clamps, split collars, motor faces, servo brackets |
| **Brass Heat-Set Threaded Inserts** | **114 pieces** | **$27.1\text{ g}$** | C3604 High-Lead Brass | Heat-staked into 3D-printed parts (M2.0, M2.5, M3) |
| **Nuts, Washers & Retaining Hardware**| **85 pieces** | **$6.5\text{ g}$** | Grade 8 Steel / Nylon | Nyloc locknuts, spring washers, standoffs, magnets |
| **TOTAL HARDWARE INVENTORY** | **400 pieces** | **$\mathbf{201.0\text{ g}}$** | — | **$5.5\%$ of aircraft flying AUW ($3.64\text{--}3.73\text{ kg}$)** |

---

## 2. Itemized Hardware Inventory by Subsystem

### A. Compact Delta Frame Clamps & Skid ($17.5\text{ g}$)

| Fastener Description | Size / Length | Qty | Unit Mass | Subtotal | Location / Assembly Note |
| :--- | :---: | :---: | :---: | :---: | :--- |
| Socket Head Cap Screw | M3 $\times$ 16mm | 4 | $1.15\text{ g}$ | **$4.6\text{ g}$** | Corner 1 & Corner 2 tube pinch bolts |
| Socket Head Cap Screw | M3 $\times$ 20mm | 5 | $1.35\text{ g}$ | **$6.75\text{ g}$** | Corner 3 dorsal pinch (1x) & Main Gear Saddle (4x) |
| Steel Nyloc Locknut | M3 standard | 9 | $0.50\text{ g}$ | **$4.5\text{ g}$** | Vibration-proof retention across all clamp ears |
| Hardened Flat Washer | M3 DIN 125 | 9 | $0.12\text{ g}$ | **$1.08\text{ g}$** | Under bolt heads to prevent FDM surface galling |
| Brass Heat-Set Insert | M3 $\times 4.0\text{ mm}$ | 2 | $0.28\text{ g}$ | **$0.56\text{ g}$** | Heat-staked into Corner 3 base pad for tailskid |
| **Subsystem Subtotal** | — | **29 pcs** | — | **$17.49\text{ g}$** | — |

---

### B. Modular Wing Station Collars on Spine ($69.0\text{ g}$)

The 4 tandem wing stations feature 8 modular split-clamp collars clamping the ⌀20mm upper carbon spine tube:

| Fastener Description | Size / Length | Qty | Unit Mass | Subtotal | Location / Assembly Note |
| :--- | :---: | :---: | :---: | :---: | :--- |
| Socket Head Cap Screw | M3 $\times$ 16mm | 16 | $1.15\text{ g}$ | **$18.4\text{ g}$** | Split collar spine pinch bolts (2 per collar) |
| Steel Nyloc Locknut | M3 standard | 16 | $0.50\text{ g}$ | **$8.0\text{ g}$** | Clamping locknuts torqued to $2.2\text{ N}\cdot\text{m}$ |
| Socket Head Cap Screw | M3 $\times$ 12mm | 32 | $0.95\text{ g}$ | **$30.4\text{ g}$** | Flange mount pattern (4 per collar at $Y = \pm 35\text{ mm}$) |
| Brass Heat-Set Insert | M3 $\times 5.7\text{ mm}$ | 32 | $0.38\text{ g}$ | **$12.16\text{ g}$** | Staked into 3D-printed wing root brackets |
| **Subsystem Subtotal** | — | **96 pcs** | — | **$68.96\text{ g}$** | — |

*(Note: Grade 5 Titanium ⌀6.0mm pivot pins `HW-02` [8x 6.4g = 51.2g] and MF106ZZ bearings `HW-01` [16x 2.4g = 38.4g] are tracked in primary structural mechanics).*

---

### C. Flapping Brushless Motors ($26.6\text{ g}$)

Mounts the 8x GoolRC 5010-14 360KV high-torque motors to the airframe brackets:

| Fastener Description | Size / Length | Qty | Unit Mass | Subtotal | Location / Assembly Note |
| :--- | :---: | :---: | :---: | :---: | :--- |
| Button Head Cap Screw | M3 $\times$ 8mm | 32 | $0.75\text{ g}$ | **$24.0\text{ g}$** | Motor stator baseplate mounting (4 per motor) |
| Spring Lock Washer | M3 DIN 127 | 32 | $0.08\text{ g}$ | **$2.56\text{ g}$** | Critical anti-loosening under $50\text{ Hz}$ motor vibration |
| **Subsystem Subtotal** | — | **64 pcs** | — | **$26.56\text{ g}$** | Must be installed with medium Loctite 243 |

---

### D. Kinematic Capstans & Sector Wheels ($11.9\text{ g}$)

| Fastener Description | Size / Length | Qty | Unit Mass | Subtotal | Location / Assembly Note |
| :--- | :---: | :---: | :---: | :---: | :--- |
| Cup-Point Set Screw (Grub) | M3 $\times$ 4mm | 8 | $0.18\text{ g}$ | **$1.44\text{ g}$** | Locks PETG helical drum onto ⌀4.0mm D-shaft flat |
| Brass Heat-Set Insert | M3 $\times 4.0\text{ mm}$ | 8 | $0.28\text{ g}$ | **$2.24\text{ g}$** | Radial thread insert embedded in PETG motor drum |
| Button Head Cap Screw | M3 $\times$ 8mm | 8 | $0.75\text{ g}$ | **$6.0\text{ g}$** | Dyneema SK99 line termination pinch clamp |
| Brass Heat-Set Insert | M3 $\times 4.0\text{ mm}$ | 8 | $0.28\text{ g}$ | **$2.24\text{ g}$** | Embedded in 10:1 sector wheel rim for cable clamp |
| **Subsystem Subtotal** | — | **32 pcs** | — | **$11.92\text{ g}$** | — |

---

### E. Morphing Servos & Linkages ($41.3\text{ g}$)

Secures the 8x AGFRC B44BLS sweep servos and 8x AGFRC B24CLM micro pitch servos:

| Fastener Description | Size / Length | Qty | Unit Mass | Subtotal | Location / Assembly Note |
| :--- | :---: | :---: | :---: | :---: | :--- |
| Socket Head Cap Screw | M2.5 $\times$ 10mm | 32 | $0.52\text{ g}$ | **$16.64\text{ g}$** | B44BLS sweep servo mounting ears (4 per servo) |
| Brass Heat-Set Insert | M2.5 $\times 4.0\text{ mm}$ | 32 | $0.19\text{ g}$ | **$6.08\text{ g}$** | Staked into 3D-printed servo tray ribs |
| Horn Clamp Cap Screw | M2.5 $\times$ 8mm | 8 | $0.42\text{ g}$ | **$3.36\text{ g}$** | Pinches titanium servo horns onto 25T splines |
| Socket Head Cap Screw | M2.0 $\times$ 8mm | 32 | $0.30\text{ g}$ | **$9.60\text{ g}$** | B24CLM pitch servo mounting ears (4 per servo) |
| Brass Heat-Set Insert | M2.0 $\times 3.0\text{ mm}$ | 32 | $0.12\text{ g}$ | **$3.84\text{ g}$** | Staked into micro pitch servo brackets |
| Central Horn Retaining Screw | M2.0 $\times$ 6mm | 8 | $0.22\text{ g}$ | **$1.76\text{ g}$** | Fastens pitch horn to micro servo output shaft |
| **Subsystem Subtotal** | — | **112 pcs** | — | **$41.28\text{ g}$** | — |

---

### F. Teardrop Electronics Pod & Access Hatch ($14.5\text{ g}$)

| Fastener Description | Size / Length | Qty | Unit Mass | Subtotal | Location / Assembly Note |
| :--- | :---: | :---: | :---: | :---: | :--- |
| Keel Clamp Cap Screw | M3 $\times$ 20mm | 3 | $1.35\text{ g}$ | **$4.05\text{ g}$** | Lower pod saddle pinch bolts through ⌀12mm keel |
| Steel Nyloc Locknut | M3 standard | 3 | $0.50\text{ g}$ | **$1.50\text{ g}$** | Retaining nuts for keel saddles |
| Knurled Thumbscrew / Pin | M3 $\times$ 15mm | 1 | $2.50\text{ g}$ | **$2.50\text{ g}$** | Top dorsal ridge mechanical backup canopy latch |
| N52 Neodymium Disc Magnets | $8\text{ mm} \times 3\text{ mm}$ | 4 | $1.10\text{ g}$ | **$4.40\text{ g}$** | Aft canopy quick-release pairs ($>22\text{ N}$ pull) |
| Nylon M2.5 Standoffs & Screws | M2.5 $\times$ 6mm | 8 | $0.25\text{ g}$ | **$2.00\text{ g}$** | Isolates Pixhawk 4 Mini and Pi 4B from tub floor |
| **Subsystem Subtotal** | — | **19 pcs** | — | **$14.45\text{ g}$** | — |

---

### G. Landing Gear & Undercarriage Hardware ($20.3\text{ g}$)

| Fastener Description | Size / Length | Qty | Unit Mass | Subtotal | Location / Assembly Note |
| :--- | :---: | :---: | :---: | :---: | :--- |
| Carbon Arch Mount Screw | M3 $\times$ 16mm | 2 | $1.15\text{ g}$ | **$2.30\text{ g}$** | Secures NOBRIM CF arch to keel saddle clamp |
| Steel Nyloc Locknut | M3 standard | 2 | $0.50\text{ g}$ | **$1.00\text{ g}$** | Locknuts under saddle clamp pad |
| Stepped Axles & Hardware Kit | M5 stepped steel | 2 | $8.50\text{ g}$ | **$17.00\text{ g}$** | Stepped axles, M5 locknuts, flange washers |
| **Subsystem Subtotal** | — | **6 pcs** | — | **$20.30\text{ g}$** | (Included in NOBRIM gear kit) |

---

## 3. Summary of Brass Heat-Set Inserts (Ruthex / CNC Style)

Heat-set inserts provide vibration-proof reusable threads in 3D-printed thermoplastics (PETG, PA-CF, Tough PLA). The total build requires **114 inserts ($27.12\text{ g}$)**:

| Thread Size | Outer Diameter | Length | Quantity | Unit Mass | Total Mass | Subsystem Usage |
| :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **M3 Standard** | ⌀4.6 mm | $5.7\text{ mm}$ | **32** | $0.38\text{ g}$ | **$12.16\text{ g}$** | Wing collar mounting brackets |
| **M3 Short** | ⌀4.6 mm | $4.0\text{ mm}$ | **18** | $0.28\text{ g}$ | **$5.04\text{ g}$** | Drums (8x), sectors (8x), tailskid (2x) |
| **M2.5 Standard**| ⌀3.6 mm | $4.0\text{ mm}$ | **32** | $0.19\text{ g}$ | **$6.08\text{ g}$** | B44BLS sweep servo tray mounting |
| **M2.0 Micro** | ⌀3.2 mm | $3.0\text{ mm}$ | **32** | $0.12\text{ g}$ | **$3.84\text{ g}$** | B24CLM pitch servo bracket mounting |
| **TOTAL INSERTS** | — | — | **114** | — | **$\mathbf{27.12\text{ g}}$** | — |

---

## 4. Potential Mass-Shaving Options (Fastener Optimization)

If further weight reduction is required, substituting alternative hardware materials can yield direct savings:

1. **Grade 5 Titanium (Ti-6Al-4V) Screw Substitution**:
   * Titanium density is $4.43\text{ g/cm}^3$ vs Alloy Steel $7.85\text{ g/cm}^3$ (a **$43.6\%$ weight reduction**).
   * Swapping the 60 primary M3 structural screws (collars, clamps, motor mounts) to Titanium saves **$\approx 32.5\text{ g}$**.
2. **7075-T6 Aluminum Motor Screws**:
   * The 32x M3 $\times$ 8mm motor mounting screws experience shear loads $< 8\text{ N}$ per screw. Swapping to black-anodized 7075-T6 aluminum screws ($0.26\text{ g}$ ea vs $0.75\text{ g}$ steel) saves **$\mathbf{15.7\text{ g}}$**.
3. **Shortened M3 Inserts**:
   * Replacing the 32 standard $5.7\text{ mm}$ M3 inserts with short $3.0\text{ mm}$ inserts ($0.20\text{ g}$) saves **$\mathbf{5.8\text{ g}}$**.

> [!TIP]
> **Fastener Lightweighting Potential**:
> Upgrading to an optimized Titanium/Aluminum fastener kit reduces total hardware mass from **$201.0\text{ g} \longrightarrow \mathbf{147.0\text{ g}}$**, shaving an additional **$54.0\text{ g}$ ($0.12\text{ lb}$)** off the airframe.
