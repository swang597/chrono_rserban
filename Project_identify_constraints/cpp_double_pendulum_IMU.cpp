// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// A very simple example that can be used as template project for
// a Chrono::Engine simulator with 3D view.
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include <fstream>
#include "chrono_thirdparty/filesystem/path.h"

// Shu: for sensor
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"


// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);
    
    // Create a Chrono physical system
    ChSystemNSC sys;


    // Pendulum example ------------------------------------

    // 1 - Create a floor that is fixed (that is used also to represent the absolute reference)

    auto floorBody = std::make_shared<ChBodyEasyBox>(10, 2, 10,  // x, y, z dimensions
                                                     3000,       // density
                                                     true,       // create visualization asset
                                                     false       // no collision geometry
                                                     );
    floorBody->SetPos(ChVector<>(0, -2, 0));
    floorBody->SetBodyFixed(true);

    sys.Add(floorBody);

    // 2 - Create the first pendulum

    auto pendulumBody1 = std::make_shared<ChBodyEasyBox>(0.1,2,0.1, //0.5, 2, 0.5,  // x, y, z dimensions
                                                         3000,         // density
                                                         true,         // create visualization asset
                                                         false         // no collision geometry
                                                         );
    pendulumBody1->SetPos(ChVector<>(0, 3, 0));
    pendulumBody1->SetPos_dt(ChVector<>(0, 0, 1));

    sys.Add(pendulumBody1);

    // 3 - Create a motor to drive the first pendulum in 3D space
    // Define an arbitrary axis of rotation in 3D space (e.g., around the vector (1, 1, 1))
    ChVector<> rotation_axis(1, 0, 0);
    rotation_axis.Normalize(); // Ensure the axis is a unit vector
    auto motor = std::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(floorBody, pendulumBody1, ChFrame<>(ChVector<>(0, 4, 0), Q_from_AngAxis(CH_C_PI / 4, rotation_axis))); // Rotate around the arbitrary axis
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 2)); // Constant speed of 0.1 * 90 degrees per second
    sys.AddLink(motor);

    // // 3 - Create a motor to drive the first pendulum
    // auto motor = std::make_shared<ChLinkMotorRotationSpeed>();
    // motor->Initialize(floorBody, pendulumBody1, ChFrame<>(ChVector<>(0, 4, 0), Q_from_AngAxis(CH_C_PI_2, VECT_Z)));
    // motor->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 2)); // Constant speed of 90 degrees per second
    // sys.AddLink(motor);

    // 3 - Create a spherical constraint for the first pendulum

    auto sphericalLink1 =
        std::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false);  // x,y,z,Rx,Ry,Rz constrains
    ChFrame<> link_position_abs1(ChVector<>(0, 4, 0));

    sphericalLink1->Initialize(pendulumBody1,        // the 1st body to connect
                               floorBody,            // the 2nd body to connect
                               false,                // the two following frames are in absolute, not relative, coords.
                               link_position_abs1,   // the link reference attached to 1st body
                               link_position_abs1);  // the link reference attached to 2nd body

    sys.Add(sphericalLink1);

    // 4 - Create the second pendulum

    auto pendulumBody2 = std::make_shared<ChBodyEasyBox>(0.1, 2, 0.1,  // x, y, z dimensions
                                                         3000,         // density
                                                         true,         // create visualization asset
                                                         false         // no collision geometry
                                                         );
    pendulumBody2->SetPos(ChVector<>(0, 1, 0));

    sys.Add(pendulumBody2);

    // 5 - Create a spherical constraint for the second pendulum

    auto sphericalLink2 =
        std::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false);  // x,y,z,Rx,Ry,Rz constrains
    ChFrame<> link_position_abs2(ChVector<>(0, 2, 0));

    sphericalLink2->Initialize(pendulumBody2,        // the 1st body to connect
                               pendulumBody1,        // the 2nd body to connect
                               false,                // the two following frames are in absolute, not relative, coords.
                               link_position_abs2,   // the link reference attached to 1st body
                               link_position_abs2);  // the link reference attached to 2nd body

    sys.Add(sphericalLink2);

    // Optionally, set color and/or texture for visual assets
    pendulumBody1->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.5f, 0.25f));
    pendulumBody2->GetVisualShape(0)->SetColor(ChColor(0.5f, 0.2f, 0.25f));
    floorBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/checker1.png"), 2, 2);

    // 6 - Create the Irrlicht visualization system
    ChVisualSystemIrrlicht vis;
    vis.SetWindowSize(800, 600);
    vis.SetWindowTitle("Double Pendulum Simulation");
    vis.Initialize();
    vis.AddLogo();
    vis.AddSkyBox();
    vis.AddTypicalLights();
    vis.AddCamera(ChVector<>(2, 2, -5), ChVector<>(0, 1, 0));
    vis.AttachSystem(&sys);
    // Add global coordinate system to the visualization
    // vis.RenderCOGFrames(1.0); // The parameter is the scale of the axes

    const std::string out_dir = GetChronoOutputPath() + "Double_pendulum_3D/";
    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Open output files for writing coordinates and rotation matrices
    std::ofstream pos_file1(out_dir + "pendulum1_positions.txt");
    std::ofstream rot_file1(out_dir + "pendulum1_rotations.txt");
    std::ofstream pos_file2(out_dir + "pendulum2_positions.txt");
    std::ofstream rot_file2(out_dir + "pendulum2_rotations.txt");

    // 7 - Simulation loop
    ChRealtimeStepTimer realtime_timer;
    double step_size = 5e-3;
    double num_steps = 5000;
    double time;
    // while (vis.Run()) {
    for (int istep = 0; istep < num_steps; istep++) {
        // Render scene
        vis.BeginScene();
        vis.Render();
        vis.RenderCOGFrames(1.0);
        vis.RenderFrame(ChFrame<>(), 10);
        vis.EndScene();
        

        // Perform the integration step
        sys.DoStepDynamics(step_size);
        
        // Write positions and rotations to files
        time = sys.GetChTime();
        ChVector<> pos1 = pendulumBody1->GetPos();
        ChMatrix33<> rot1 = pendulumBody1->GetRot();

        pos_file1 << time << " " << pos1.x() << " " << pos1.y() << " " << pos1.z() << "\n";
        rot_file1 << time << " "
                  << rot1(0, 0) << " " << rot1(0, 1) << " " << rot1(0, 2) << " "
                  << rot1(1, 0) << " " << rot1(1, 1) << " " << rot1(1, 2) << " "
                  << rot1(2, 0) << " " << rot1(2, 1) << " " << rot1(2, 2) << "\n";

        ChVector<> pos2 = pendulumBody2->GetPos();
        ChMatrix33<> rot2 = pendulumBody2->GetRot();

        pos_file2 << time << " " << pos2.x() << " " << pos2.y() << " " << pos2.z() << "\n";
        rot_file2 << time << " " 
                  << rot2(0, 0) << " " << rot2(0, 1) << " " << rot2(0, 2) << " "
                  << rot2(1, 0) << " " << rot2(1, 1) << " " << rot2(1, 2) << " "
                  << rot2(2, 0) << " " << rot2(2, 1) << " " << rot2(2, 2) << "\n";

        // Spin in place to maintain soft real-time
        realtime_timer.Spin(step_size);
    }

    return 0;
}
