# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.cascade as cascade
from OCC.Core  import TopoDS

print ("Example: Load a STEP file, generated by a CAD")

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')


#  Create the simulation system and add items
sys = chrono.ChSystemNSC()

# Load a STEP file, containing a mechanism. The demo STEP file has been
# created using a 3D CAD (in this case, SolidEdge v.18).

# Create the ChCascadeDoc, a container that loads the STEP model
# and manages its subassembles
mydoc = cascade.ChCascadeDoc()

# load the STEP model using this command:
load_ok = mydoc.Load_STEP(chrono.GetChronoDataFile('cascade/assembly.stp'))  

# print the contained shapes
#mydoc.Dump(chrono.GetLog())

# In most CAD systems the Y axis is horizontal, but we want it vertical.
# So define a root transformation for rotating all the imported objects.
rotation1 = chrono.ChQuaternionD()
rotation1.Q_from_AngAxis(-chrono.CH_C_PI / 2, chrono.ChVectorD(1, 0, 0))  # 1: rotate 90° on X axis
rotation2 = chrono.ChQuaternionD()
rotation2.Q_from_AngAxis(chrono.CH_C_PI, chrono.ChVectorD(0, 1, 0))  # 2: rotate 180° on vertical Y axis
tot_rotation = chrono.ChQuaternionD()
tot_rotation = rotation2 % rotation1     # rotate on 1 then on 2, using quaternion product
root_frame = chrono.ChFrameMovingD(chrono.ChVectorD(0, 0, 0), tot_rotation)

# Retrieve some sub shapes from the loaded model, using
# the GetNamedShape() function, that can use path/subpath/subsubpath/part
# syntax and * or ? wildcards, etc.
rigidBody1 = 0
rigidBody2 = 0

if load_ok: 
    
    shape1 = TopoDS.TopoDS_Shape()
    if (mydoc.GetNamedShape(shape1, "Assem1/body1")):
        body1 = cascade.ChCascadeBodyEasy(shape1, 1000) # density
        body1.SetBodyFixed(True) 
        sys.Add(body1)
        
        # Move the body as for global displacement/rotation (also body1 %= root_frame; )
        body1.ConcatenatePreTransformation(root_frame);
        
        rigidBody1 = body1;
    else:
        print("Warning. Desired object not found in document \n")

    shape2 = TopoDS.TopoDS_Shape()
    if (mydoc.GetNamedShape(shape2, "Assem1/body2")): 
        body2 = cascade.ChCascadeBodyEasy(shape2, 1000) # density        
        sys.Add(body2)
        
        # Move the body as for global displacement/rotation  (also mbody2 %= root_frame; )
        body2.ConcatenatePreTransformation(root_frame);

        rigidBody2 = body2;
    else:
        print("Warning. Desired object not found in document \n")

else:
    print("Warning. Desired STEP file could not be opened/parsed \n")

# Create a revolute joint between the two parts
# as in a pendulum. We assume we already know in advance
# the aboslute position of the joint (ex. we used measuring tools in the 3D CAD)

measured_joint_pos_mm = chrono.ChVectorD(0, 48, 120);

scale = 1. / 1000.  # because we use meters instead of mm

joint_pos = chrono.ChVectorD(root_frame.TransformPointLocalToParent(measured_joint_pos_mm * scale)) 

if (rigidBody1 and rigidBody2):
    link = chrono.ChLinkLockRevolute()
    link.Initialize(rigidBody1, rigidBody2, chrono.ChCoordsysD(joint_pos));
    sys.Add(link);

# Create a large cube as a floor.
floor = chrono.ChBodyEasyBox(1, 0.2, 1, 1000)
floor.SetPos(chrono.ChVectorD(0,-0.3,0))
floor.SetBodyFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/blue.png'))
sys.Add(floor)

#  Create an Irrlicht application to visualize the system
vis = chronoirr.ChVisualSystemIrrlicht()
sys.SetVisualSystem(vis)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Test')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.2,0.2,-0.2))
vis.AddTypicalLights()

#  Run the simulation
while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    sys.DoStepDynamics(0.01)

