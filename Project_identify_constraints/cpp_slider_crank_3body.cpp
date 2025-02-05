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
// Authors: Alessandro Tasora
// =============================================================================
//
//  Demo code about
//
//  - constraints and 'motor' objects
//  - using IRRLICHT as a realtime 3D viewer of a slider-crank mechanism
//    simulated with Chrono::Engine.
//  - using the real-time step.
//
// This is just a possible method of integration of Chrono::Engine + Irrlicht;
// many others are possible.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"


#include <fstream>
#include "chrono_thirdparty/filesystem/path.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    //
    // HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO...
    //

    // 1- Create a Chrono physical system: all bodies and constraints
    //    will be handled by this ChSystemNSC object.

    ChSystemNSC sys;

    // 2- Create the rigid bodies of the slider-crank mechanical system
    //   (a crank, a rod, a truss), maybe setting position/mass/inertias of
    //   their center of mass (COG) etc.

    // ..the truss
    auto my_body_A = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_A);
    my_body_A->SetBodyFixed(true);  // truss does not move!
    my_body_A->SetName("Ground-Truss");

    // ..the crank
    auto my_body_B = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_B);
    my_body_B->SetPos(ChVector<>(1, 0, 0));  // position of COG of crank
    my_body_B->SetMass(2);
    my_body_B->SetName("Crank");

    // ..the rod
    auto my_body_C = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_C);
    my_body_C->SetPos(ChVector<>(4, 0, 0));  // position of COG of rod
    my_body_C->SetMass(3);
    my_body_C->SetName("Rod");

    // 3- Create constraints: the mechanical joints between the rigid bodies.

    // .. a revolute joint between crank and rod
    auto my_link_BC = chrono_types::make_shared<ChLinkLockRevolute>();
    my_link_BC->SetName("RevJointCrankRod");
    my_link_BC->Initialize(my_body_B, my_body_C, ChCoordsys<>(ChVector<>(2, 0, 0)));
    sys.AddLink(my_link_BC);

    // .. a slider joint between rod and truss
    auto my_link_CA = chrono_types::make_shared<ChLinkLockPointLine>();
    my_link_CA->SetName("TransJointRodGround");
    my_link_CA->Initialize(my_body_C, my_body_A, ChCoordsys<>(ChVector<>(6, 0, 0)));
    sys.AddLink(my_link_CA);

    // .. a motor between crank and truss
    auto my_link_AB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_link_AB->Initialize(my_body_A, my_body_B, ChFrame<>(ChVector<>(0, 0, 0)));
    my_link_AB->SetName("RotationalMotor");
    sys.AddLink(my_link_AB);
    auto my_speed_function = chrono_types::make_shared<ChFunction_Const>(CH_C_PI);  // speed w=3.145 rad/sec
    my_link_AB->SetSpeedFunction(my_speed_function);

    // 4- Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Simple slider-crank example");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 0, -6));
    vis->AddTypicalLights();

    // Simulation loop
    const std::string out_dir = GetChronoOutputPath() + "Slider_crank/";
    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Open output files for writing coordinates and rotation matrices
    std::ofstream pos_file1(out_dir + "body1_positions.txt");
    std::ofstream rot_file1(out_dir + "body1_rotations.txt");
    std::ofstream pos_file2(out_dir + "body2_positions.txt");
    std::ofstream rot_file2(out_dir + "body2_rotations.txt");
    std::ofstream pos_file3(out_dir + "body3_positions.txt");
    std::ofstream rot_file3(out_dir + "body3_rotations.txt");


    // Timer for enforcing soft real-time
    ChRealtimeStepTimer realtime_timer;
    double time_step = 0.01;
    double num_steps = 5000;
    double time;
    
    // bool removed = false;

    // while (vis->Run()) {
    for (int istep = 0; istep < num_steps; istep++) {
        // Irrlicht must prepare frame to draw
        vis->BeginScene();

        // Irrlicht now draws simple lines in 3D world representing a
        // skeleton of the mechanism, in this instant:
        //
        // .. draw items belonging to Irrlicht scene, if any
        vis->Render();
        // .. draw a grid
        tools::drawGrid(vis.get(), 0.5, 0.5);
        // .. draw GUI items belonging to Irrlicht screen, if any
        vis->GetGUIEnvironment()->drawAll();

        // .. draw the rod (from joint BC to joint CA)
        tools::drawSegment(vis.get(), my_link_BC->GetMarker1()->GetAbsCoord().pos,
                           my_link_CA->GetMarker1()->GetAbsCoord().pos, ChColor(0, 1, 0));
        // .. draw the crank (from joint AB to joint BC)
        tools::drawSegment(vis.get(), my_link_AB->GetLinkAbsoluteCoords().pos,
                           my_link_BC->GetMarker1()->GetAbsCoord().pos, ChColor(1, 0, 0));
        // .. draw a small circle at crank origin
        tools::drawCircle(vis.get(), 0.1, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));

        /* test: delete a link after 10 seconds
        if (sys.GetChTime() >10 && (!removed))
        {
                sys.RemoveLink(my_link_AB);
                removed = true;
        }*/

        // ADVANCE SYSTEM STATE BY ONE STEP
        sys.DoStepDynamics(time_step);

        // Write positions and rotations to files
        time = sys.GetChTime();
        ChVector<> pos1 = my_body_A->GetPos();
        ChMatrix33<> rot1 = my_body_A->GetRot();

        pos_file1 << time << " " << pos1.x() << " " << pos1.y() << " " << pos1.z() << "\n";
        rot_file1 << time << " "
                  << rot1(0, 0) << " " << rot1(0, 1) << " " << rot1(0, 2) << " "
                  << rot1(1, 0) << " " << rot1(1, 1) << " " << rot1(1, 2) << " "
                  << rot1(2, 0) << " " << rot1(2, 1) << " " << rot1(2, 2) << "\n";

        ChVector<> pos2 = my_body_B->GetPos();
        ChMatrix33<> rot2 = my_body_B->GetRot();

        pos_file2 << time << " " << pos2.x() << " " << pos2.y() << " " << pos2.z() << "\n";
        rot_file2 << time << " " 
                  << rot2(0, 0) << " " << rot2(0, 1) << " " << rot2(0, 2) << " "
                  << rot2(1, 0) << " " << rot2(1, 1) << " " << rot2(1, 2) << " "
                  << rot2(2, 0) << " " << rot2(2, 1) << " " << rot2(2, 2) << "\n";

        ChVector<> pos3 = my_body_C->GetPos();
        ChMatrix33<> rot3 = my_body_C->GetRot();

        pos_file3 << time << " " << pos3.x() << " " << pos3.y() << " " << pos3.z() << "\n";
        rot_file3 << time << " "
                  << rot3(0, 0) << " " << rot3(0, 1) << " " << rot3(0, 2) << " "
                  << rot3(1, 0) << " " << rot3(1, 1) << " " << rot3(1, 2) << " "
                  << rot3(2, 0) << " " << rot3(2, 1) << " " << rot3(2, 2) << "\n";



        // Enforce soft real-time
        realtime_timer.Spin(time_step);

        // Irrlicht must finish drawing the frame
        vis->EndScene();
    }

    return 0;
}
