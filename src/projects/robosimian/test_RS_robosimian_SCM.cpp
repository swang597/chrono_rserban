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
// Authors: Radu Serban
// =============================================================================
//
// RoboSimian on rigid terrain
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <vector>

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "driver_cb.h"
#include "irrlicht_app.h"
#include "robosimian.h"

using namespace chrono;
using namespace chrono::collision;

double time_step = 1e-3;

// Drop the robot on SCM terrain
bool drop = true;

// Phase durations
double duration_pose = 1.0;          // Interval to assume initial pose
double duration_settle_robot = 0.5;  // Interval to allow robot settling on terrain
double duration_sim = 10;            // Duration of actual locomotion simulation

// Output frequencies
double output_fps = 100;
double render_fps = 60;

// Output directories
const std::string out_dir = "../ROBOSIMIAN_SCM";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string img_dir = out_dir + "/IMG";

// POV-Ray and/or IMG output
bool data_output = true;
bool povray_output = false;
bool image_output = false;

// =============================================================================

void CreateCamera(irrlicht::ChIrrApp& application,
                  const irr::core::vector3df& position,
                  const irr::core::vector3df& target) {
    irrlicht::RTSCamera* camera =
        new irrlicht::RTSCamera(application.GetDevice(), application.GetSceneManager()->getRootSceneNode(),
                                application.GetSceneManager(), -1, -160.0f, 1.0f, 0.003f);

    camera->setPosition(position);
    camera->setTarget(target);
    camera->setUpVector(irr::core::vector3df(0, 0, 1));
    camera->setNearValue(0.1f);
    camera->setMinZoom(0.6f);
}

// =============================================================================

void SetContactProperties(robosimian::RoboSimian* robot) {
    assert(robot->GetSystem()->GetContactMethod() == ChMaterialSurface::SMC);

    float friction = 0.4f;
    float Y = 1e7f;
    float cr = 0.0f;

    robot->GetSledBody()->GetMaterialSurfaceSMC()->SetFriction(friction);
    robot->GetWheelBody(robosimian::LimbID::FR)->GetMaterialSurfaceSMC()->SetFriction(friction);
    robot->GetWheelBody(robosimian::LimbID::FL)->GetMaterialSurfaceSMC()->SetFriction(friction);
    robot->GetWheelBody(robosimian::LimbID::RR)->GetMaterialSurfaceSMC()->SetFriction(friction);
    robot->GetWheelBody(robosimian::LimbID::RL)->GetMaterialSurfaceSMC()->SetFriction(friction);

    robot->GetSledBody()->GetMaterialSurfaceSMC()->SetYoungModulus(Y);
    robot->GetWheelBody(robosimian::LimbID::FR)->GetMaterialSurfaceSMC()->SetYoungModulus(Y);
    robot->GetWheelBody(robosimian::LimbID::FL)->GetMaterialSurfaceSMC()->SetYoungModulus(Y);
    robot->GetWheelBody(robosimian::LimbID::RR)->GetMaterialSurfaceSMC()->SetYoungModulus(Y);
    robot->GetWheelBody(robosimian::LimbID::RL)->GetMaterialSurfaceSMC()->SetYoungModulus(Y);

    robot->GetSledBody()->GetMaterialSurfaceSMC()->SetRestitution(cr);
    robot->GetWheelBody(robosimian::LimbID::FR)->GetMaterialSurfaceSMC()->SetRestitution(cr);
    robot->GetWheelBody(robosimian::LimbID::FL)->GetMaterialSurfaceSMC()->SetRestitution(cr);
    robot->GetWheelBody(robosimian::LimbID::RR)->GetMaterialSurfaceSMC()->SetRestitution(cr);
    robot->GetWheelBody(robosimian::LimbID::RL)->GetMaterialSurfaceSMC()->SetRestitution(cr);
}

std::shared_ptr<vehicle::SCMDeformableTerrain> CreateTerrain(ChSystemSMC* sys,
                                                             double length,
                                                             double width,
                                                             double height,
                                                             double offset) {
    // Deformable terrain properties (LETE sand)
    double Kphi = 5301e3;    // Bekker Kphi
    double Kc = 102e3;       // Bekker Kc
    double n = 0.793;        // Bekker n exponent
    double coh = 1.3e3;      // Mohr cohesive limit (Pa)
    double phi = 31.1;       // Mohr friction limit (degrees)
    double K = 1.2e-2;       // Janosi shear coefficient (m)
    double E_elastic = 2e8;  // Elastic stiffness (Pa/m), before plastic yeld
    double damping = 3e4;    // Damping coefficient (Pa*s/m)

    // Initial number of divisions per unit (m)
    double factor = 12;

    // Mesh divisions
    int ndivX = (int)std::ceil(length * factor);
    int ndivY = (int)std::ceil(width * factor);

    auto terrain = std::make_shared<vehicle::SCMDeformableTerrain>(sys);
    terrain->SetPlane(ChCoordsys<>(ChVector<>(offset, 0, 0), Q_from_AngX(CH_C_PI_2)));
    terrain->SetSoilParametersSCM(Kphi, Kc, n, coh, phi, K, E_elastic, damping);
    terrain->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.15);
    terrain->Initialize(height, length, width, ndivX, ndivY);
    terrain->SetAutomaticRefinement(true);
    terrain->SetAutomaticRefinementResolution(0.02);

    // Enable moving patch feature
    ////terrain.EnableMovingPatch(m113.GetChassisBody(), ChVector<>(-2, 0, 0), 6.5, 3.5);

    return terrain;
}

// =============================================================================

int main(int argc, char* argv[]) {
    // ------------
    // Timed events
    // ------------

    double time_create_terrain = duration_pose;                       // create terrain after robot assumes initial pose
    double time_start = time_create_terrain + duration_settle_robot;  // start actual simulation after robot settling
    double time_end = time_start + duration_sim;                      // end simulation after specified duration

    // -------------
    // Create system
    // -------------

    ChSystemSMC my_sys;

    my_sys.SetMaxItersSolverSpeed(200);
    if (my_sys.GetContactMethod() == ChMaterialSurface::NSC)
        my_sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));
    ////my_sys.Set_G_acc(ChVector<double>(0, 0, 0));

    // -----------------------
    // Create RoboSimian robot
    // -----------------------

    robosimian::RoboSimian robot(&my_sys, true, true);

    // Set output directory

    robot.SetOutputDirectory(out_dir);

    // Set actuation mode for wheel motors

    ////robot.SetMotorActuationMode(robosimian::ActuationMode::ANGLE);

    // Control collisions (default: true for sled and wheels only)

    ////robot.SetCollide(robosimian::CollisionFlags::NONE);
    ////robot.SetCollide(robosimian::CollisionFlags::ALL);
    ////robot.SetCollide(robosimian::CollisionFlags::LIMBS);
    ////robot.SetCollide(robosimian::CollisionFlags::CHASSIS | robosimian::CollisionFlags::WHEELS);

    // Set visualization modes (default: all COLLISION)

    ////robot.SetVisualizationTypeChassis(robosimian::VisualizationType::MESH);
    ////robot.SetVisualizationTypeLimb(robosimian::FL, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimb(robosimian::FR, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimb(robosimian::RL, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimb(robosimian::RR, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimbs(robosimian::VisualizationType::NONE);
    ////robot.SetVisualizationTypeChassis(robosimian::VisualizationType::MESH);
    ////robot.SetVisualizationTypeSled(robosimian::VisualizationType::MESH);
    ////robot.SetVisualizationTypeLimbs(robosimian::VisualizationType::MESH);

    // Initialize Robosimian robot

    ////robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI)));

    // -----------------------------------
    // Create a driver and attach to robot
    // -----------------------------------

    ////auto driver = std::make_shared<robosimian::Driver>(
    ////    "",                                                           // start input file
    ////    GetChronoDataFile("robosimian/actuation/walking_cycle.txt"),  // cycle input file
    ////    "",                                                           // stop input file
    ////    true);
    ////auto driver = std::make_shared<robosimian::Driver>(
    ////    GetChronoDataFile("robosimian/actuation/sculling_start.txt"),   // start input file
    ////    GetChronoDataFile("robosimian/actuation/sculling_cycle2.txt"),  // cycle input file
    ////    GetChronoDataFile("robosimian/actuation/sculling_stop.txt"),    // stop input file
    ////    true);
    ////auto driver = std::make_shared<robosimian::Driver>(
    ////    GetChronoDataFile("robosimian/actuation/inchworming_start.txt"),  // start input file
    ////    GetChronoDataFile("robosimian/actuation/inchworming_cycle.txt"),  // cycle input file
    ////    GetChronoDataFile("robosimian/actuation/inchworming_stop.txt"),   // stop input file
    ////    true);
    auto driver = std::make_shared<robosimian::Driver>(
        GetChronoDataFile("robosimian/actuation/driving_start.txt"),  // start input file
        GetChronoDataFile("robosimian/actuation/driving_cycle.txt"),  // cycle input file
        GetChronoDataFile("robosimian/actuation/driving_stop.txt"),   // stop input file
        true);

    robosimian::RobotDriverCallback cbk(&robot);
    driver->RegisterPhaseChangeCallback(&cbk);

    driver->SetTimeOffsets(duration_pose, duration_settle_robot);
    robot.SetDriver(driver);

    // -------------------------------
    // Create the visualization window
    // -------------------------------

    robosimian::RobotIrrApp application(&robot, driver.get(), L"RoboSimian - SCM terrain",
                                        irr::core::dimension2d<irr::u32>(800, 600));
    irrlicht::ChIrrWizard::add_typical_Logo(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Lights(application.GetDevice(), irr::core::vector3df(100.f, 100.f, 100.f),
                                              irr::core::vector3df(100.f, -100.f, 80.f));
    irrlicht::ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(1, -2.75f, 0.2f),
                                              irr::core::vector3df(1, 0, 0));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // -----------------------------
    // Initialize output directories
    // -----------------------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }
    if (image_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // ---------------------------------
    // Run simulation for specified time
    // ---------------------------------

    int output_steps = (int)std::ceil((1.0 / output_fps) / time_step);
    int render_steps = (int)std::ceil((1.0 / render_fps) / time_step);
    int sim_frame = 0;
    int output_frame = 0;
    int render_frame = 0;

    std::shared_ptr<vehicle::SCMDeformableTerrain> terrain;
    bool terrain_created = false;

    while (application.GetDevice()->run()) {
        if (drop && !terrain_created && my_sys.GetChTime() > time_create_terrain) {
            // Set terrain height
            double z = robot.GetWheelPos(robosimian::FR).z() - 0.15;

            // Rigid terrain parameters
            double length = 8;
            double width = 2;

            // Create terrain
            terrain = CreateTerrain(&my_sys, length, width, z, length / 4);
            SetContactProperties(&robot);

            application.AssetBindAll();
            application.AssetUpdateAll();

            // Release robot
            robot.GetChassis()->GetBody()->SetBodyFixed(false);

            terrain_created = true;
        }

        application.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        application.DrawAll();

        if (data_output && sim_frame % output_steps == 0) {
            robot.Output();
        }

        // Output POV-Ray date and/or snapshot images
        if (sim_frame % render_steps == 0) {
            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%04d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(&my_sys, filename);
            }
            if (image_output) {
                char filename[100];
                sprintf(filename, "%s/img_%04d.jpg", img_dir.c_str(), render_frame + 1);
                irr::video::IImage* image = application.GetVideoDriver()->createScreenShot();
                if (image) {
                    application.GetVideoDriver()->writeImageToFile(image, filename);
                    image->drop();
                }
            }

            render_frame++;
        }

        robot.DoStepDynamics(time_step);

        sim_frame++;

        application.EndScene();
    }

    std::cout << "avg. speed: " << cbk.GetAvgSpeed() << std::endl;

    return 0;
}
