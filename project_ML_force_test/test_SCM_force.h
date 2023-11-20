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
// Common settings for SCM_force_test programs
//
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using std::cout;
using std::endl;

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::viper;

// -----------------------------------------------------------------------------

// const std::string out_dir = GetChronoOutputPath() + "SCM_FORCE_TEST";
std::string out_dir = GetChronoOutputPath() + "SCM_FORCE_TEST";

const std::string SCM_filename = out_dir + "/SCM_forces.txt";

int num_steps = 10000; //30000;
double time_step = 5e-4; // used for gen training data
//5e-4;

// -----------------------------------------------------------------------------

std::shared_ptr<Viper> CreateViper(ChSystem& sys) {
    auto viper = chrono_types::make_shared<Viper>(&sys, ViperWheelType::RealWheel);

    double time_ramp_Wy = 0.1, Wy = 10;
    auto driver = chrono_types::make_shared<ViperSpeedDriver>(time_ramp_Wy, Wy);
    viper->SetDriver(driver);

    // viper->Initialize(ChFrame<>(ChVector<>(-5, 0, -0.2), QUNIT));
    viper->Initialize(ChFrame<>(ChVector<>(10, 2, 0.25), QUNIT));

    return viper;
}

std::shared_ptr<Viper> CreateViper(ChSystem& sys, ViperWheelType wheel_type, double time_ramp_Wy=0.1, double Wy=10,
    ChVector<> viper_init_loc=ChVector<>(10, 2, 0.25)) {
    // auto viper = chrono_types::make_shared<Viper>(&sys, ViperWheelType::RealWheel);
    std::shared_ptr<Viper> viper; // Declare viper outside the switch
    switch (wheel_type) {
        case ViperWheelType::RealWheel:
            viper = chrono_types::make_shared<Viper>(&sys, ViperWheelType::RealWheel);
            break;
        case ViperWheelType::SimpleWheel:
            viper = chrono_types::make_shared<Viper>(&sys, ViperWheelType::SimpleWheel);
            break;
        case ViperWheelType::CylWheel:
            viper = chrono_types::make_shared<Viper>(&sys, ViperWheelType::CylWheel);
            break;
    }

    auto driver = chrono_types::make_shared<ViperSpeedDriver>(time_ramp_Wy, Wy);
    viper->SetDriver(driver);

    // viper->Initialize(ChFrame<>(ChVector<>(-5, 0, -0.2), QUNIT));
    viper->Initialize(ChFrame<>(viper_init_loc, QUNIT));

    return viper;
}

std::shared_ptr<Viper> CreateViper(ChSystem& sys, ViperWheelType wheel_type, std::shared_ptr<chrono::viper::ViperDriver> driver,
    ChVector<> viper_init_loc=ChVector<>(10, 2, 0.25)) {
    // auto viper = chrono_types::make_shared<Viper>(&sys, ViperWheelType::RealWheel);
    std::shared_ptr<Viper> viper; // Declare viper outside the switch
    switch (wheel_type) {
        case ViperWheelType::RealWheel:
            viper = chrono_types::make_shared<Viper>(&sys, ViperWheelType::RealWheel);
            break;
        case ViperWheelType::SimpleWheel:
            viper = chrono_types::make_shared<Viper>(&sys, ViperWheelType::SimpleWheel);
            break;
        case ViperWheelType::CylWheel:
            viper = chrono_types::make_shared<Viper>(&sys, ViperWheelType::CylWheel);
            break;
    }

    // auto driver = chrono_types::make_shared<ViperSpeedDriver>(time_ramp_Wy, Wy);
    viper->SetDriver(driver);

    // viper->Initialize(ChFrame<>(ChVector<>(-5, 0, -0.2), QUNIT));
    viper->Initialize(ChFrame<>(viper_init_loc, QUNIT));

    return viper;
}

std::shared_ptr<vehicle::SCMTerrain> CreateTerrain(double resolution, bool enable_bulldozing, ChSystem& sys) {
    auto terrain = chrono_types::make_shared<vehicle::SCMTerrain>(&sys);

    // Displace/rotate the terrain reference plane
    terrain->SetPlane(ChCoordsys<>(ChVector<>(0, 0, -0.5)));

    // Initialize a rectangular pacth
    double length = 14;
    double width = 4;
    terrain->Initialize(length, width, resolution);

    // Set the soil terramechanical parameters
    terrain->SetSoilParameters(0.82e6,   // Bekker Kphi
                               0.14e4,   // Bekker Kc
                               1.0,      // Bekker n exponent
                               0.017e4,  // Mohr cohesive limit (Pa)
                               35.0,     // Mohr friction limit (degrees)
                               1.78e-2,  // Janosi shear coefficient (m)
                               2e8,      // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                               3e4       // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    // Set up bulldozing factors
    if (enable_bulldozing) {
        terrain->EnableBulldozing(true);  // inflate soil at the border of the rut
        terrain->SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    }

    // Set some visualization parameters
    terrain->SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 20000);
    terrain->SetMeshWireframe(true);

    return terrain;
}

std::shared_ptr<vehicle::SCMTerrain> CreateTerrain(double resolution, bool enable_bulldozing, ChSystem& sys,
    const std::string& heightmap_file, double sizeX, double sizeY, double hMin, double hMax, 
    double terrain_initX, double terrain_initH) {

    auto terrain = chrono_types::make_shared<vehicle::SCMTerrain>(&sys);

    // Displace/rotate the terrain reference plane
    ChVector<> location(terrain_initX, 0, terrain_initH);
    terrain->SetPlane(ChCoordsys<>(location));

    // Initialize terrain from a bmp file
    terrain->Initialize(heightmap_file, sizeX, sizeY, hMin, hMax, resolution);
    
    // Set the soil terramechanical parameters
    terrain->SetSoilParameters(0.82e6,   // Bekker Kphi
                               0.14e4,   // Bekker Kc
                               1.0,      // Bekker n exponent
                               0.017e4,  // Mohr cohesive limit (Pa)
                               35.0,     // Mohr friction limit (degrees)
                               1.78e-2,  // Janosi shear coefficient (m)
                               2e8,      // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                               3e4       // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    // Set up bulldozing factors
    if (enable_bulldozing) {
        terrain->EnableBulldozing(true);  // inflate soil at the border of the rut
        terrain->SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    }

    // Set some visualization parameters
    terrain->SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 20000);
    terrain->SetMeshWireframe(true);

    return terrain;
}

std::shared_ptr<ChVisualSystem> CreateVisualization(ChVisualSystem::Type vis_type, bool add_grid, ChSystem& sys, ChVector<> cam_loc=ChVector<>(1.0, 2.0, 1.4)) {
    // Create the run-time visualization interface
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Viper SCM test (SAVE)");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(cam_loc, VNULL);
            vis_irr->AddTypicalLights();
            if (add_grid)
                vis_irr->AddGrid(0.2, 0.2, 80, 20, ChCoordsys<>(ChVector<>(0, 0, -0.5), QUNIT));
            vis_irr->EnableShadows();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowSize(800, 600);
            vis_vsg->SetWindowTitle("Viper SCM test (SAVE)");
            vis_vsg->AddCamera(ChVector<>(1.0, 2.0, 1.4), VNULL);
            if (add_grid)
                vis_vsg->AddGrid(0.2, 0.2, 80, 20, ChCoordsys<>(ChVector<>(0, 0, -0.5), QUNIT));
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    return vis;
}