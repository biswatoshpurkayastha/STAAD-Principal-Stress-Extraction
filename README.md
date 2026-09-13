# Principal Stress Visualization for STAAD.Pro Members

## Overview

STAAD.Pro provides member stress results such as axial stress, bending stress, and shear stress in the member local coordinate system. While these results are suitable for many conventional design applications, they do not directly provide visualization of principal stresses and the associated stress trajectories.

For certain structural components such as reinforced-concrete deep beams, transfer girders, pile caps, corbels, and disturbed regions (D-regions), understanding the principal stress field can provide valuable insight into stress flow and load-transfer mechanisms.

This Python-based tool post-processes STAAD.Pro member stress results and generates principal stress visualizations to aid engineering interpretation.

---

## Why Principal Stresses?

In many structures, cracking and failure do not necessarily occur along the member axis.

Principal stress visualization can help engineers:

- Understand stress-flow patterns
- Visualize compression-strut development
- Study tension-field behavior
- Interpret possible crack orientations
- Assess potential failure-plane directions
- Identify stress-concentration regions

For reinforced concrete members, principal tensile stress trajectories may provide useful insight into likely crack-development patterns and load-transfer paths.

---

## Features

The application can generate:

- Major Principal Stress (σ₁)
- Minor Principal Stress (σ₂)
- Principal Stress Directions
- Principal Stress Trajectories
- Maximum Shear Stress
- Maximum Shear Stress Directions
- Von Mises Stress Contours
- Interactive Stress Visualization
- Zoom and Pan Functions
- Stress Value Inspection

---

## Typical Applications

- Deep Concrete Beams
- Transfer Girders
- Corbels
- Pile Caps
- D-Regions
- Stress Flow Visualization
- Failure Plane Assessment
- Structural Engineering Education
- Research Applications

---

## Requirements

### Software

- STAAD.Pro
- Python 3.x

### Python Libraries

Install the required libraries:

```bash
pip install numpy scipy matplotlib pandas pyvista PySide6
