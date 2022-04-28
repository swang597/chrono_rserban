// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
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
// Polaris acceleration test.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

int main(int argc, char* argv[]) {
    double length = 400;

    // Create the vehicle system
    auto contact_method = ChContactMethod::SMC;
    WheeledVehicle vehicle(vehicle::GetDataFile("mrzr/vehicle/MRZR.json"), contact_method);
    vehicle.Initialize(ChCoordsys<>(ChVector<>(-length / 2 + 5, 0, 0.2), QUNIT));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    ////auto powertrain = ReadPowertrainJSON(vehicle::GetDataFile("mrzr/powertrain/MRZR_SimplePowertrain.json"));
    auto powertrain = ReadPowertrainJSON(vehicle::GetDataFile("mrzr/powertrain/MRZR_SimpleMapPowertrain.json"));
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile("mrzr/tire/MRZR_TMeasyTire.json"));
            vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    // Create the terrain
    RigidTerrain terrain(vehicle.GetSystem());
    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    patch_mat->SetYoungModulus(2e7f);
    patch_mat->SetPoissonRatio(0.3f);
    auto patch = terrain.AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), length, 5);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 5);
    terrain.Initialize();

    // Create the straight path and the driver system
    auto path = StraightLinePath(ChVector<>(-length / 2, 0, 0.5), ChVector<>(length / 2, 0, 0.5), 1);
    ChPathFollowerDriver driver(vehicle, path, "my_path", 1000.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&vehicle, L"Polaris acceleration test");
    app.AddTypicalLights();
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 5.0, 0.5);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Running average of vehicle speed
    utils::ChRunningAverage speed_filter(500);
    double last_speed = -1;

    // Record vehicle speed
    ChFunction_Recorder speed_recorder;

    // Initialize simulation frame counter and simulation time
    double step_size = 1e-3;
    int step_number = 0;
    double time = 0;
    bool done = false;

    // Simulation loop
    ChTimer<> timer;
    timer.start();
    while (app.GetDevice()->run()) {
        time = vehicle.GetSystem()->GetChTime();

        double speed = speed_filter.Add(vehicle.GetSpeed());
        if (!done) {
            speed_recorder.AddPoint(time, speed);
            if (time > 6 && std::abs((speed - last_speed) / step_size) < 2e-4) {
                done = true;
                timer.stop();
                std::cout << "Simulation time: " << timer() << std::endl;
                std::cout << "Maximum speed: " << speed << std::endl;
#ifdef CHRONO_POSTPROCESS
                postprocess::ChGnuPlot gplot;
                gplot.SetGrid();
                gplot.SetLabelX("time (s)");
                gplot.SetLabelY("speed (m/s)");
                gplot.Plot(speed_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
#endif
            }
        }
        last_speed = speed;

        // End simulation
        if (time >= 100)
            break;

        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        if (done) {
            driver_inputs.m_throttle = 0.1;
            driver_inputs.m_braking = 0.8;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        app.Synchronize("Acceleration test", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;

        app.EndScene();
    }

    return 0;
}
