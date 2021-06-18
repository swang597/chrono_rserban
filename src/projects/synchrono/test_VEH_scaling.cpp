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
// Author: Radu Serban
// =============================================================================
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <cstdio>
#include <cmath>
#include <vector>

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

#undef CHRONO_IRRLICHT

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
using namespace chrono::irrlicht;
#endif

using std::cout;
using std::endl;

// =============================================================================

double terrainLength = 500;  // size in X direction
double terrainWidth = 500;   // size in Y direction
double delta = 0.05;         // SCM grid spacing

// Simulation run time
double end_time = 20;

// Simulation step size
double step_size = 2e-3;

// Number of threads
int nthreads = 8;

// Better conserve mass by displacing soil to the sides of a rut
const bool bulldozing = false;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------
    // Create HMMWV vehicle
    // --------------------
    ChVector<> init_loc(0, 2, 0.5);
    ChQuaternion<> init_rot = Q_from_AngZ(0);

    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChContactMethod::SMC);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(init_loc, init_rot));
    my_hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
    my_hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    my_hmmwv.SetTireType(TireModelType::RIGID);
    my_hmmwv.SetTireStepSize(step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::MESH);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::MESH);
    my_hmmwv.SetTireVisualizationType(VisualizationType::MESH);

    // -----------------------------------------------------------
    // Set tire contact material, contact model, and visualization
    // -----------------------------------------------------------
    auto wheel_material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    wheel_material->SetFriction(0.8f);
    wheel_material->SetYoungModulus(1.0e6f);
    wheel_material->SetRestitution(0.1f);

    my_hmmwv.SetTireVisualizationType(VisualizationType::MESH);

    // --------------------
    // Create driver system
    // --------------------

    auto path = StraightLinePath(init_loc, init_loc + ChVector<>(terrainLength, 0, 0), 0);
    ChPathFollowerDriver driver(my_hmmwv.GetVehicle(), path, "Box path", 10);
    driver.Initialize();

    // Reasonable defaults for the underlying PID
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.GetSteeringController().SetGains(0.4, 0.1, 0.2);
    driver.GetSteeringController().SetLookAheadDistance(2);

    // ------------------
    // Create the terrain
    // ------------------
    ChSystem* system = my_hmmwv.GetSystem();
    system->SetNumThreads(nthreads);

#ifdef CHRONO_IRRLICHT
    SCMDeformableTerrain terrain(system, true);
#else
    SCMDeformableTerrain terrain(system, false);
#endif
    terrain.SetSoilParameters(2e6,   // Bekker Kphi
                              0,     // Bekker Kc
                              1.1,   // Bekker n exponent
                              0,     // Mohr cohesive limit (Pa)
                              30,    // Mohr friction limit (degrees)
                              0.01,  // Janosi shear coefficient (m)
                              2e8,   // Elastic stiffness (Pa/m), before plastic yield
                              3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    if (bulldozing) {
        terrain.EnableBulldozing(true);       // inflate soil at the border of the rut
        terrain.SetBulldozingParameters(55,   // angle of friction for erosion of displaced material at rut border
                                        0.8,  // displaced material vs downward pressed material.
                                        5,    // number of erosion refinements per timestep
                                        10);  // number of concentric vertex selections subject to erosion
    }

    // Optionally, enable moving patch feature (single patch around vehicle chassis)
    terrain.AddMovingPatch(my_hmmwv.GetChassisBody(), ChVector<>(0, 0, 0), ChVector<>(5, 3, 1));

    // Optionally, enable moving patch feature (multiple patches around each wheel)
    ////for (auto& axle : my_hmmwv.GetVehicle().GetAxles()) {
    ////    terrain.AddMovingPatch(axle->m_wheels[0]->GetSpindle(), ChVector<>(0, 0, 0), ChVector<>(1, 0.5, 1));
    ////    terrain.AddMovingPatch(axle->m_wheels[1]->GetSpindle(), ChVector<>(0, 0, 0), ChVector<>(1, 0.5, 1));
    ////}

    terrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.1);

    terrain.Initialize(terrainLength, terrainWidth, delta);

#ifdef CHRONO_IRRLICHT
    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------
    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), L"HMMWV Deformable Soil Demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();
#endif

    // ---------------
    // Simulation loop
    // ---------------
    std::cout << "Total vehicle mass: " << my_hmmwv.GetTotalMass() << std::endl;

    // Solver settings.
    system->SetSolverMaxIterations(50);

    // Initialize simulation frame counter
    int step_number = 0;
    double time = 0;

    ChTimer<> timer;
    timer.start();

#ifdef CHRONO_IRRLICHT
    while (app.GetDevice()->run()) {
#else
    while (time < end_time) {
#endif
        time = system->GetChTime();

#ifdef CHRONO_IRRLICHT
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        tools::drawColorbar(0, 0.1, "Sinkage", app.GetDevice(), 30);
        app.EndScene();
#endif

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
#ifdef CHRONO_IRRLICHT
        app.Synchronize("", driver_inputs);
#endif

        // Advance dynamics
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
#ifdef CHRONO_IRRLICHT
        app.Advance(step_size);
#endif  // CHRONO_IRRLICHT

        // Increment frame number
        step_number++;
    }

    timer.stop();

    std::cout << "elapsed: " << timer() << std::endl;
    std::cout << "RTF: " << (timer() / end_time) << std::endl;
    std::cout << "\nSCM stats for last step:" << std::endl;
    terrain.PrintStepStatistics(std::cout);

    return 0;
}
