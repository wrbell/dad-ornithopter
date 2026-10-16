#!/usr/bin/env python3
"""
scripts/render_topological_collar.py
====================================
Renders high-resolution multi-view engineering visualization of the
topologically optimized bilateral wing collar using VTK and Pillow.
"""

import os
import vtk
from PIL import Image

OUT_DIR = "cad/manufactured"
STL_PATH = os.path.join(OUT_DIR, "mco_topological_wing_collar_v1.stl")

def render_vtk_view(stl_path, view="iso", img_size=(1200, 900)):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(stl_path)
    reader.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.20, 0.55, 0.85) # High-tech precision blue
    actor.GetProperty().SetSpecular(0.4)
    actor.GetProperty().SetSpecularPower(25.0)
    actor.GetProperty().SetAmbient(0.25)
    actor.GetProperty().SetDiffuse(0.75)

    ren = vtk.vtkRenderer()
    ren.AddActor(actor)
    ren.SetBackground(0.08, 0.11, 0.17) # Dark slate background (#141c2b)

    ren_win = vtk.vtkRenderWindow()
    ren_win.SetOffScreenRendering(1)
    ren_win.AddRenderer(ren)
    ren_win.SetSize(img_size[0], img_size[1])

    cam = ren.GetActiveCamera()
    ren.ResetCamera()
    fp = cam.GetFocalPoint()

    if view == "iso":
        cam.SetPosition(fp[0] + 120, fp[1] - 140, fp[2] + 90)
        cam.SetViewUp(0, 0, 1)
    elif view == "front":
        cam.SetPosition(fp[0], fp[1], fp[2] + 180) # Looking down or along X
        cam.SetViewUp(0, 1, 0)
    elif view == "top":
        cam.SetPosition(fp[0] + 180, fp[1], fp[2])
        cam.SetViewUp(0, 0, 1)
    elif view == "side":
        cam.SetPosition(fp[0], fp[1] - 180, fp[2])
        cam.SetViewUp(0, 0, 1)

    ren.ResetCameraClippingRange()
    cam.Zoom(1.15)
    ren_win.Render()

    w2if = vtk.vtkWindowToImageFilter()
    w2if.SetInput(ren_win)
    w2if.Update()

    writer = vtk.vtkPNGWriter()
    temp_png = f"/tmp/collar_{view}.png"
    writer.SetFileName(temp_png)
    writer.SetInputConnection(w2if.GetOutputPort())
    writer.Write()
    return temp_png

print("Rendering Topological Collar Views...")
img_iso_path   = render_vtk_view(STL_PATH, view="iso")
img_front_path = render_vtk_view(STL_PATH, view="front")
img_top_path   = render_vtk_view(STL_PATH, view="top")
img_side_path  = render_vtk_view(STL_PATH, view="side")

# Combine into a 2x2 4-view gallery
im_iso   = Image.open(img_iso_path)
im_front = Image.open(img_front_path)
im_top   = Image.open(img_top_path)
im_side  = Image.open(img_side_path)

W, H = im_iso.size
composite = Image.new("RGB", (W * 2, H * 2), "#0f172a")
composite.paste(im_iso,   (0, 0))
composite.paste(im_front, (W, 0))
composite.paste(im_top,   (0, H))
composite.paste(im_side,  (W, H))

out_png = os.path.join(OUT_DIR, "mco_topological_collar_4view.png")
composite.save(out_png, quality=95)
print(f"✓ Saved 4-view composite render to {out_png}")
