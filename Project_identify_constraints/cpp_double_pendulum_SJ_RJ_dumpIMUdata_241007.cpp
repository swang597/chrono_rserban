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
    pendulumBody1->SetPos_dt(ChVector<>(2, 0, 5));
    // pendulumBody1->SetWvel_par(ChVector<>(CH_C_PI / 2, 0.0, 0.0));  // Angular velocity in rad/s (90 deg/s) around z-axis

    sys.Add(pendulumBody1);

    // 3 - Create a motor to drive the first pendulum in 3D space
    // Define an arbitrary axis of rotation in 3D space (e.g., around the vector (1, 1, 1))
    // ChVector<> rotation_axis(1, 0, 0);
    // rotation_axis.Normalize(); // Ensure the axis is a unit vector
    // auto motor = std::make_shared<ChLinkMotorRotationSpeed>();
    // motor->Initialize(floorBody, pendulumBody1, 
    //     ChFrame<>(ChVector<>(0, 4, 0), 
    //     Q_from_AngAxis(0, rotation_axis))); // Rotate around the arbitrary axis

    // auto speed_function = chrono_types::make_shared<ChFunction_Sine>(
    // 0.0,                   // Start time
    // 0.5,                   // Frequency (Hz)
    // CH_C_PI / 4            // Amplitude (rad/s)
    // );
    // // motor->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 2)); // Constant speed of 0.1 * 90 degrees per second
    // motor->SetSpeedFunction(speed_function); // Constant speed of 0.1 * 90 degrees per second
    // sys.AddLink(motor);

    // 4 - Create a spherical constraint for the first pendulum
    // Create a spherical constraint for the first pendulum
    auto sphericalLink1 = std::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false);  // x,y,z,Rx,Ry,Rz constrains
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
    // Add initial linear velocity (e.g., moving along the x-axis)
    double body2_vz = 10.0;
    pendulumBody2->SetPos_dt(ChVector<>(0.0, body2_vz, 0.0));  // Velocity in m/s along x-axis
    // Add initial angular velocity (e.g., rotating around the z-axis)
    // pendulumBody2->SetWvel_par(ChVector<>(0.0, CH_C_PI / 2, 0.0));  // Angular velocity in rad/s (90 deg/s) around z-axis

    sys.Add(pendulumBody2);

    // 5 - Create a revolute joint
    auto revoluteLink2 = std::make_shared<ChLinkLockRevolute>();  // Revolute joint constrains x, y, z, and allows rotation about a single axis (usually z)

    // Define the position of the revolute joint in absolute coordinates
    // Initialize the revolute joint (rotation around z-axis)
    // revoluteLink2->Initialize(pendulumBody2,        // the 1st body to connect
    //                         pendulumBody1,            // the 2nd body to connect
    //                         ChCoordsys<>(ChVector<>(0, 2, 0),   // Joint position
    //                         Q_from_AngAxis(CH_C_PI_2, ChVector<>(0, 0, 1))));  // Orientation of the joint axis (rotation around z-axis)
    
    // // Initialize the revolute joint (rotation around arbitrary axis)
    // auto axis_revolute = ChVector<>(1, 1, 1);  // Define the axis of rotation
    auto axis_revolute = ChVector<>(0, 0, 1);  // Define the axis of rotation
    axis_revolute.Normalize();            // Normalize the axis
    revoluteLink2->Initialize(pendulumBody2,        // the 1st body to connect
                            pendulumBody1,            // the 2nd body to connect
                            ChCoordsys<>(ChVector<>(0, 2, 0),   // Joint position
                            Q_from_AngAxis(CH_C_PI_2, axis_revolute)));  // Orientation of the joint axis (rotation around z-axis)

    // Add the revolute joint to the system
    sys.Add(revoluteLink2);

    // 5 - Create a spherical constraint for the second pendulum

    // auto sphericalLink2 =
    //     std::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false);  // x,y,z,Rx,Ry,Rz constrains
    // ChFrame<> link_position_abs2(ChVector<>(0, 2, 0));

    // sphericalLink2->Initialize(pendulumBody2,        // the 1st body to connect
    //                            pendulumBody1,        // the 2nd body to connect
    //                            false,                // the two following frames are in absolute, not relative, coords.
    //                            link_position_abs2,   // the link reference attached to 1st body
    //                            link_position_abs2);  // the link reference attached to 2nd body

    // sys.Add(sphericalLink2);

    // Optionally, set color and/or texture for visual assets
    pendulumBody1->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.5f, 0.25f));
    pendulumBody2->GetVisualShape(0)->SetColor(ChColor(0.5f, 0.2f, 0.25f));
    // floorBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/checker1.png"), 2, 2);

    // 6 - Create the Irrlicht visualization system
    ChVisualSystemIrrlicht vis;
    vis.SetWindowSize(800, 600);
    vis.SetWindowTitle("Double Pendulum Simulation");
    vis.Initialize();
    vis.AddLogo();
    vis.AddSkyBox();
    vis.AddTypicalLights();
    vis.AddCamera(ChVector<>(10, 4, -2), ChVector<>(0, 1, 0));
    vis.AttachSystem(&sys);
    // Add global coordinate system to the visualization
    // vis.RenderCOGFrames(1.0); // The parameter is the scale of the axes

    double step_size = 1e-2;// 1e-4;
    double num_steps = int(50 / step_size);
    const std::string out_dir = GetChronoOutputPath() 
        + "Double_pendulum_SJ_RJ_dt" +std::to_string(step_size) 
        + "_vnew1_RJaxis001/";
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
    
    
    double time;
    // while (vis.Run()) {
    for (int istep = 0; istep < num_steps; istep++) {
        if(istep % 1000 == 0){
            std::cout << "Time: " << sys.GetChTime() << std::endl;
        }
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
        ChVector<> pos1_dt = pendulumBody1->GetPos_dt();
        ChVector<> pos1_dtdt = pendulumBody1->GetPos_dtdt();
        ChMatrix33<> rot1 = pendulumBody1->GetRot();
        ChVector<> rot1_wvel = pendulumBody1->GetWvel_loc();
        ChVector<> rot1_wacc = pendulumBody1->GetWacc_loc();
        ChVector<> rot1_wvel_par = pendulumBody1->GetWvel_par();
        ChVector<> rot1_wacc_par = pendulumBody1->GetWacc_par();
        
        ChVector<> pos2 = pendulumBody2->GetPos();
        ChVector<> pos2_dt = pendulumBody2->GetPos_dt();
        ChVector<> pos2_dtdt = pendulumBody2->GetPos_dtdt();
        ChMatrix33<> rot2 = pendulumBody2->GetRot();
        ChVector<> rot2_wvel = pendulumBody2->GetWvel_loc();
        ChVector<> rot2_wacc = pendulumBody2->GetWacc_loc();
        ChVector<> rot2_wvel_par = pendulumBody2->GetWvel_par();
        ChVector<> rot2_wacc_par = pendulumBody2->GetWacc_par();

        // if(istep % 100 == 0){
            pos_file1 << time << " " << pos1.x() << " " << pos1.y() << " " << pos1.z() 
                        << " " << pos1_dt.x() << " " << pos1_dt.y() << " " << pos1_dt.z() 
                        << " " << pos1_dtdt.x() << " " << pos1_dtdt.y() << " " << pos1_dtdt.z() 
                        << "\n";
            rot_file1 << time << " "
                    << rot1(0, 0) << " " << rot1(0, 1) << " " << rot1(0, 2) << " "
                    << rot1(1, 0) << " " << rot1(1, 1) << " " << rot1(1, 2) << " "
                    << rot1(2, 0) << " " << rot1(2, 1) << " " << rot1(2, 2) 
                    << " " << rot1_wvel.x() << " " << rot1_wvel.y() << " " << rot1_wvel.z() 
                    << " " << rot1_wacc.x() << " " << rot1_wacc.y() << " " << rot1_wacc.z() 
                        << " " << rot1_wvel_par.x() << " " << rot1_wvel_par.y() << " " << rot1_wvel_par.z()
                        << " " << rot1_wacc_par.x() << " " << rot1_wacc_par.y() << " " << rot1_wacc_par.z()
                    << "\n";

            pos_file2 << time << " " << pos2.x() << " " << pos2.y() << " " << pos2.z() 
                            << " " << pos2_dt.x() << " " << pos2_dt.y() << " " << pos2_dt.z()
                            << " " << pos2_dtdt.x() << " " << pos2_dtdt.y() << " " << pos2_dtdt.z()
                            << "\n";
            rot_file2 << time << " " 
                    << rot2(0, 0) << " " << rot2(0, 1) << " " << rot2(0, 2) << " "
                    << rot2(1, 0) << " " << rot2(1, 1) << " " << rot2(1, 2) << " "
                    << rot2(2, 0) << " " << rot2(2, 1) << " " << rot2(2, 2) 
                    << " " << rot2_wvel.x() << " " << rot2_wvel.y() << " " << rot2_wvel.z()
                    << " " << rot2_wacc.x() << " " << rot2_wacc.y() << " " << rot2_wacc.z()
                        << " " << rot2_wvel_par.x() << " " << rot2_wvel_par.y() << " " << rot2_wvel_par.z()
                        << " " << rot2_wacc_par.x() << " " << rot2_wacc_par.y() << " " << rot2_wacc_par.z()
                    << "\n";
        // }
        
        // Spin in place to maintain soft real-time
        realtime_timer.Spin(step_size);
    }

    return 0;
}
