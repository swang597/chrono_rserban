#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
// #include "chrono/physics/ChLinkLockRevolute.h"
// #include "chrono/physics/ChLinkLockPointLine.h"
// #include "chrono/physics/ChLinkLockPrismatic.h"  // Include prismatic joint
// #include "chrono/geometry/ChTriangleMeshConnected.h"  // Needed for drawing with triangle mesh
// #include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include <fstream>
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Chrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono physical system
    ChSystemNSC sys;

    // Create the bodies: truss (ground), crank, rod, and slider
    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetBodyFixed(true);  // Ground body
    truss->SetName("Ground-Truss");
    sys.AddBody(truss);

    auto rail = chrono_types::make_shared<ChBoxShape>(12, 0.1, 0.1);
    truss->AddVisualShape(rail, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));

    auto crank = chrono_types::make_shared<ChBody>();
    crank->SetPos(ChVector<>(1, 0, 0));  // Position of COG of crank
    crank->SetMass(2);
    crank->SetName("Crank");
    sys.AddBody(crank);

    auto rod = chrono_types::make_shared<ChBody>();
    rod->SetPos(ChVector<>(4, 0, 0));  // Position of COG of rod
    rod->SetMass(3);
    rod->SetName("Rod");
    sys.AddBody(rod);

    auto slider = chrono_types::make_shared<ChBody>();
    slider->SetPos(ChVector<>(6, 0, 0));  // Position of COG of slider
    slider->SetMass(1);
    slider->SetName("Slider");
    sys.AddBody(slider);

    // Create constraints: revolute joint (crank-rod), prismatic joint (slider-truss), and motor (crank-truss)
    auto joint_crank_rod = chrono_types::make_shared<ChLinkLockRevolute>();
    joint_crank_rod->Initialize(crank, rod, ChCoordsys<>(ChVector<>(2, 0, 0)));
    joint_crank_rod->SetName("RevJointCrankRod");
    sys.AddLink(joint_crank_rod);

    auto joint_rod_slider = chrono_types::make_shared<ChLinkLockRevolute>();
    joint_rod_slider->Initialize(rod, slider, ChCoordsys<>(ChVector<>(6, 0, 0)));
    joint_rod_slider->SetName("RevJointCrankRod");
    sys.AddLink(joint_rod_slider);

    // auto joint_slider_truss = chrono_types::make_shared<ChLinkLockPrismatic>();
    // joint_slider_truss->Initialize(slider, truss, ChCoordsys<>(ChVector<>(6, 0, 0)));
    // joint_slider_truss->SetName("TransJointSliderGround");
    // sys.AddLink(joint_slider_truss);

    // auto joint_slider_truss = chrono_types::make_shared<ChLinkLockPrismatic>();
    // joint_slider_truss->Initialize(slider, truss, ChCoordsys<>(ChVector<>(6, 0, 0)));
    // joint_slider_truss->SetName("TransJointSliderGround");
    // sys.AddLink(joint_slider_truss);

    // Create prismatic joints between ground and sliders
    auto joint_slider_truss = chrono_types::make_shared<ChLinkLockPrismatic>();
    joint_slider_truss->Initialize(slider, truss, ChCoordsys<>(ChVector<>(6, 0, 0), Q_from_AngY(CH_C_PI_2)));
    joint_slider_truss->SetName("TransJointSliderGround");
    sys.AddLink(joint_slider_truss);

    auto motor_crank_truss = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor_crank_truss->Initialize(truss, crank, ChFrame<>(ChVector<>(0 , 0, 0)));
    motor_crank_truss->SetName("RotationalMotor");
    sys.AddLink(motor_crank_truss);
    // auto speed_function = chrono_types::make_shared<ChFunction_Const>(CH_C_PI);  // Speed w=3.145 rad/sec
    // motor_crank_truss->SetSpeedFunction(speed_function);
    // Set up the sinusoidal speed function: w(t) = sin(0.1 * time)
    auto speed_function = chrono_types::make_shared<chrono::ChFunction_Sine>(0.0, 0.1, 10.0);  // offset=0, frequency=0.1, amplitude=1.0
    motor_crank_truss->SetSpeedFunction(speed_function);

    // 4- Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Simple slider-crank example");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 0, -10));
    vis->AddTypicalLights();

    double time_step = 1e-3;
    double num_steps = int(20 / time_step);
    double time;
    
    // Simulation loop
    const std::string out_dir = GetChronoOutputPath() + "Slider_crank_IMU_dt"+std::to_string(time_step)+"/";
    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Open output files for writing coordinates and rotation matrices
    std::ofstream pos_file1(out_dir + "crank_positions.txt");
    std::ofstream rot_file1(out_dir + "crank_rotations.txt");
    std::ofstream pos_file2(out_dir + "rod_positions.txt");
    std::ofstream rot_file2(out_dir + "rod_rotations.txt");
    std::ofstream pos_file3(out_dir + "slider_positions.txt");
    std::ofstream rot_file3(out_dir + "slider_rotations.txt");
    std::ofstream joint12_file(out_dir + "crank_rod_position.txt");

    // Timer for enforcing soft real-time
    ChRealtimeStepTimer realtime_timer;
    
    // bool removed = false;

    // while (vis->Run()) {
    for (int istep = 0; istep < num_steps; istep++) {
        if(istep % int(num_steps*0.1) == 0)
            std::cout << "Time: " << sys.GetChTime() << std::endl;
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
        tools::drawSegment(vis.get(), joint_crank_rod->GetMarker1()->GetAbsCoord().pos,
                           joint_slider_truss->GetMarker1()->GetAbsCoord().pos, ChColor(0, 1, 0));
        // .. draw the crank (from joint AB to joint BC)
        tools::drawSegment(vis.get(), motor_crank_truss->GetLinkAbsoluteCoords().pos,
                           joint_crank_rod->GetMarker1()->GetAbsCoord().pos, ChColor(1, 0, 0));
        // .. draw a small circle at crank origin
        tools::drawCircle(vis.get(), 0.1, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
        // .. draw a small square at slider origin
        tools::drawCircle(vis.get(), 0.1, ChCoordsys<>(ChVector<>(6, 0, 0), Q_from_AngX(CH_C_PI_2)));

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
        ChVector<> pos1 = crank->GetPos();
        ChVector<> pos1_dt = crank->GetPos_dt();
        ChVector<> pos1_dtdt = crank->GetPos_dtdt();
        ChMatrix33<> rot1 = crank->GetRot();
        ChVector<> rot1_wvel = crank->GetWvel_loc();
        ChVector<> rot1_wacc = crank->GetWacc_loc();
        // joint_crank_rod position, velocity, acceleration
        ChVector<> joint12_pos = joint_crank_rod->GetMarker1()->GetAbsCoord().pos;
        ChVector<> joint12_vel = joint_crank_rod->GetMarker1()->GetAbsCoord_dt().pos;
        ChVector<> joint12_acc = joint_crank_rod->GetMarker1()->GetAbsCoord_dtdt().pos;

        // joint_rod_slider position
        ChVector<> joint23_pos = joint_rod_slider->GetMarker1()->GetAbsCoord().pos;
        
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
                  << "\n";

        ChVector<> pos2 = rod->GetPos();
        ChVector<> pos2_dt = rod->GetPos_dt();
        ChVector<> pos2_dtdt = rod->GetPos_dtdt();
        ChMatrix33<> rot2 = rod->GetRot();
        ChVector<> rot2_wvel = rod->GetWvel_loc();
        ChVector<> rot2_wacc = rod->GetWacc_loc();

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
                  << "\n";

        ChVector<> pos3 = slider->GetPos();
        ChVector<> pos3_dt = slider->GetPos_dt();
        ChVector<> pos3_dtdt = slider->GetPos_dtdt();
        ChMatrix33<> rot3 = slider->GetRot();
        ChVector<> rot3_wvel = slider->GetWvel_loc();
        ChVector<> rot3_wacc = slider->GetWacc_loc();

        pos_file3 << time << " " << pos3.x() << " " << pos3.y() << " " << pos3.z() 
                    << " " << pos3_dt.x() << " " << pos3_dt.y() << " " << pos3_dt.z()
                    << " " << pos3_dtdt.x() << " " << pos3_dtdt.y() << " " << pos3_dtdt.z()
                    << "\n";
        rot_file3 << time << " "
                  << rot3(0, 0) << " " << rot3(0, 1) << " " << rot3(0, 2) << " "
                  << rot3(1, 0) << " " << rot3(1, 1) << " " << rot3(1, 2) << " "
                  << rot3(2, 0) << " " << rot3(2, 1) << " " << rot3(2, 2) 
                  << " " << rot3_wvel.x() << " " << rot3_wvel.y() << " " << rot3_wvel.z()
                  << " " << rot3_wacc.x() << " " << rot3_wacc.y() << " " << rot3_wacc.z()
                  << "\n";



        // Enforce soft real-time
        realtime_timer.Spin(time_step);

        // Irrlicht must finish drawing the frame
        vis->EndScene();
    }

    return 0;
    // // Create the Irrlicht visualization system
    // ChIrrApp application(&sys, L"Slider-Crank Mechanism", core::dimension2d<u32>(800, 600), false, true);
    // application.AddTypicalLights();
    // application.AddTypicalCamera(core::vector3df(0, 3, -6));

    // // Simulation loop
    // while (application.GetDevice()->run()) {
    //     // Advance simulation by one step
    //     application.BeginScene();
    //     application.DrawAll();
    //     application.DoStep();
    //     application.EndScene();
    // }

    // return 0;
}
