#!/usr/bin/env python3
"""
scripts/convert_organic_to_step.py
==================================
Converts the organic blended continuous STL models into exact OpenCASCADE B-Rep AP214 STEP files.
"""

import sys, os
from OCP.RWStl import RWStl
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Compound
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace
from OCP.STEPControl import STEPControl_Writer, STEPControl_StepModelType
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static

def stl_to_step(stl_path, step_path):
    print(f"Reading {stl_path} ...")
    mesh = RWStl.ReadFile_s(stl_path)
    if mesh is None:
        raise RuntimeError(f"Failed to read STL from {stl_path}")
    
    n_tri = mesh.NbTriangles()
    print(f"Loaded mesh with {n_tri:,} triangles. Building compound of planar faces...")
    
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    
    for i in range(1, n_tri + 1):
        tri = mesh.Triangle(i)
        n1, n2, n3 = tri.Get()
        p1 = mesh.Node(n1)
        p2 = mesh.Node(n2)
        p3 = mesh.Node(n3)
        
        poly = BRepBuilderAPI_MakePolygon(p1, p2, p3, True)
        if poly.IsDone():
            face_maker = BRepBuilderAPI_MakeFace(poly.Wire())
            if face_maker.IsDone():
                builder.Add(compound, face_maker.Face())
                
    print(f"Transferring B-Rep to STEP format...")
    Interface_Static.SetCVal_s("write.step.schema", "AP214")
    writer = STEPControl_Writer()
    status = writer.Transfer(compound, STEPControl_StepModelType.STEPControl_AsIs)
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP transfer failed with status {status}")
    
    os.makedirs(os.path.dirname(step_path), exist_ok=True)
    res = writer.Write(step_path)
    if res != IFSelect_RetDone:
        raise RuntimeError("Failed to write STEP file")
    
    size_mb = os.path.getsize(step_path) / (1024.0 * 1024.0)
    print(f"  [OK] Generated {step_path} ({size_mb:.2f} MB, {n_tri:,} B-Rep faces)")

if __name__ == '__main__':
    stls = [
        ('cad/manufactured/s1223_bird_of_prey_organic_port.stl',
         'cad/manufactured/s1223_bird_of_prey_organic_port.step'),
        ('cad/manufactured/s1223_bird_of_prey_organic_starboard.stl',
         'cad/manufactured/s1223_bird_of_prey_organic_starboard.step'),
    ]
    for in_stl, out_step in stls:
        stl_to_step(in_stl, out_step)
