// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Luning Bakke
// =============================================================================
// Demo to show Iris Rover operated on Rigid Terrain
// This Demo includes operation to spawn an Iris rover, control wheel speed
// =============================================================================
// TODO: different wheel types that uses different meshes

#include "Iris.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_thirdparty/filesystem/path.h"
// #include <vector>

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

using namespace chrono;
using namespace chrono::iris;

// -----------------------------------------------------------------------------

// Define Iris rover wheel type
IrisWheelType wheel_type = IrisWheelType::RealWheel;


// -----------------------------------------------------------------------------
// contact material, define friction coefficienn, cor, etc.
std::shared_ptr<ChMaterialSurface> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.1f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    SetChronoDataPath("/home/swang597/Documents/Research/chrono_fork_rserban/build/data/");
    // SetDataPath("/home/swang597/Documents/Research/chrono_fork_radu/build/data/vehicle/");
    bool flag_vis = true; //false; //
    bool flag_save_vedio = true; //false; //
    // Simulation time step
    double time_step = 1e-3;

    double num_steps = 1e5;
    int save_vedio_fps = int(num_steps/100.0);
    int idx_vedio = 0;
    int num_dump_steps = 100;

    // Output directory
    const std::string out_dir = GetChronoOutputPath() + "Iris_dt" + std::to_string(time_step) + "/";
    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create the Chrono system with gravity in the negative Z direction
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    sys.SetCollisionSystemType(collision::ChCollisionSystemType::BULLET);
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // Create the ground.
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(30, 30, 1, 1000, true, true, ground_mat);
    ground->SetPos(ChVector<>(0, 0, -0.5));
    ground->SetBodyFixed(true);
    ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 60, 45);
    sys.Add(ground);

    // Construct a Iris rover and the asociated driver
    // anguler velocity reaches 0.4 rad/s after 1 second of simulation
    auto driver = chrono_types::make_shared<IrisSpeedDriver>(1.0, 0.4f);

    Iris iris(&sys, wheel_type);
    iris.SetSpeedDriver(driver);
    iris.SetChassisFixed(false);
    iris.SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));

    iris.SetChassisVisualization(true);

    // Initialize the rover at position (0, 0, 0.14)
    iris.Initialize(ChFrame<>(ChVector<>(0, 0, 0.14), QUNIT));

    std::cout << "iris total mass: " << iris.GetRoverMass() << std::endl;
    std::cout << "  chassis:        " << iris.GetChassis()->GetBody()->GetMass() << std::endl;
    std::cout << "  wheel:          " << iris.GetWheel(IrisWheelID::LF)->GetBody()->GetMass() << std::endl;
    std::cout << std::endl;

    // Open output files for writing coordinates and rotation matrices
    std::ofstream fp_pos_chassis(out_dir + "chassis_pos.txt");
    std::ofstream fp_rot_chassis(out_dir + "chassis_rot.txt");
    std::vector<std::ofstream> fp_pos_wheels;
    std::vector<std::ofstream> fp_rot_wheels;
    for (int i = 0; i < 4; i++) {
        fp_pos_wheels.push_back(std::ofstream(out_dir + "wheel_" + std::to_string(i) + "_pos.txt"));
        fp_rot_wheels.push_back(std::ofstream(out_dir + "wheel_" + std::to_string(i) + "_rot.txt"));
    }
    // std::ofstream pos_file2(out_dir + "rod_positions.txt");
    // std::ofstream rot_file2(out_dir + "rod_rotations.txt");
    // std::ofstream pos_file3(out_dir + "slider_positions.txt");
    // std::ofstream rot_file3(out_dir + "slider_rotations.txt");

    // Create the run-time visualization interface
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    if(flag_vis){
        vis->AttachSystem(&sys);
        vis->SetCameraVertical(CameraVerticalDir::Z);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("Iris Rover on Rigid Terrain");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        // vis->AddCamera(ChVector<>(3, 3, 1));
        vis->AddCamera(ChVector<>(1, 1, 1));
        vis->AddTypicalLights();
        // vis->EnableContactDrawing(ContactsDrawMode::CONTACT_DISTANCES);
        vis->EnableShadows();
    }
    
    // ChVector<> pos_chassis;
    // ChVector<> pos_wheel;
    // ChMatrix33<> rot_chassis;
    // ChMatrix33<> rot_wheel;
    // Simulation loop
    // while (vis->Run()) {
    for (int istep = 0; istep < num_steps; istep++) {
        auto& loc = iris.GetChassis()->GetBody()->GetPos();
        if(flag_vis){
            vis->UpdateCamera(loc + ChVector<>(0.5, 0.5, 0.2), loc + ChVector<>(0, 0.0, 0));
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        if(flag_vis && flag_save_vedio  && istep % save_vedio_fps == 0){ //
            std::string imgName = out_dir + "/img_" + std::to_string(idx_vedio) + ".jpg";
            vis->WriteImageToFile(imgName);
            idx_vedio++;
        }
        
        // Set current steering angle
        double time = iris.GetSystem()->GetChTime();
        // Update the rover controls
        iris.Update();

        // std::cout << sys.GetChTime() << ", " << iris.GetChassisVel() << ", " << std::endl;
        sys.DoStepDynamics(time_step);
        if (istep % num_dump_steps == 0) {
            // std::cout << "time: " << time << ", chassis pos: " << iris.GetChassis()->GetBody()->GetPos() << std::endl;
            
            ChVector<> pos_chassis = iris.GetChassis()->GetBody()->GetPos();
            ChMatrix33<> rot_chassis = iris.GetChassis()->GetBody()->GetRot();
            // std::cout << "time: " << time << ", chassis pos: " << pos_chassis << std::endl;
            // std::cout << "time: " << time << ", chassis rot: " << rot_chassis << std::endl;

            fp_pos_chassis << time << " " << pos_chassis.x() << " " << pos_chassis.y() << " " << pos_chassis.z() << "\n";
            fp_rot_chassis << time << " "
                            << rot_chassis(0, 0) << " " << rot_chassis(0, 1) << " " << rot_chassis(0, 2) << " "
                            << rot_chassis(1, 0) << " " << rot_chassis(1, 1) << " " << rot_chassis(1, 2) << " "
                            << rot_chassis(2, 0) << " " << rot_chassis(2, 1) << " " << rot_chassis(2, 2) << "\n";
            // std::cout << "time: " << time << ", chassis pos: " << iris.GetChassis()->GetBody()->GetPos() << std::endl; 
            for (int i = 0; i < 4; i++) {
                ChVector<> pos_wheel = iris.GetWheel(static_cast<IrisWheelID>(i))->GetBody()->GetPos();
                ChMatrix33<> rot_wheel = iris.GetWheel(static_cast<IrisWheelID>(i))->GetBody()->GetRot();
                fp_pos_wheels[i] << time << " " << pos_wheel.x() << " " << pos_wheel.y() << " " << pos_wheel.z() << "\n";
                fp_rot_wheels[i] << time << " "
                                << rot_wheel(0, 0) << " " << rot_wheel(0, 1) << " " << rot_wheel(0, 2) << " "
                                << rot_wheel(1, 0) << " " << rot_wheel(1, 1) << " " << rot_wheel(1, 2) << " "
                                << rot_wheel(2, 0) << " " << rot_wheel(2, 1) << " " << rot_wheel(2, 2) << "\n";
            }
            // std::cout << "time: " << time << ", wheel pos: " << iris.GetWheel(static_cast<IrisWheelID>(0))->GetBody()->GetPos() << std::endl;
        }


    }

    return 0;
}