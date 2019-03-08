// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Test influence of terrain friction
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================
// Problem parameters

// Contact method type
ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::SMC;

// Type of tire model (RIGID, TMEASY, FIALA, PAC89, or PACEJKA)
TireModelType tire_model = TireModelType::TMEASY;

// Terrain friction coefficients
float friction_1 = 0.6f;
float friction_2 = 0.9f;

// Simulation step size, end time
double step_size = 2e-3;
double tire_step_size = 1e-3;

// Output
bool output = true;
double out_fps = 50;
const std::string out_dir = "../HMMWV_FRICTION";

// =============================================================================

int main(int argc, char* argv[]) {
    // Chrono system
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));
    sys.SetMaxItersSolverSpeed(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // Create the terrain (will contain 2 side-by-side patches)
    RigidTerrain terrain(&sys);

    // Create and initialize the first vehicle
    HMMWV_Full hmmwv_1(&sys);
    hmmwv_1.SetInitPosition(ChCoordsys<>(ChVector<>(-90, -5.5, 1.0), QUNIT));
    hmmwv_1.SetPowertrainType(PowertrainModelType::SHAFTS);
    hmmwv_1.SetDriveType(DrivelineType::RWD);
    hmmwv_1.SetTireType(tire_model);
    hmmwv_1.SetTireStepSize(tire_step_size);
    hmmwv_1.SetVehicleStepSize(step_size);
    hmmwv_1.Initialize();
    hmmwv_1.SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv_1.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_1.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_1.SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv_1.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    auto patch_1 = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, -5.5, -0.1), QUNIT), ChVector<>(200, 10, 0.2));
    patch_1->SetContactFrictionCoefficient(friction_1);
    patch_1->SetContactRestitutionCoefficient(0.01f);
    patch_1->SetContactMaterialProperties(2e7f, 0.3f);
    patch_1->SetColor(ChColor(0.8f, 0.8f, 1.0f));
    patch_1->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 10);

    auto path_1 = StraightLinePath(ChVector<>(-90, -5.5, 0.1), ChVector<>(+90, -5.5, 0.1));
    ChPathFollowerDriver driver_1(hmmwv_1.GetVehicle(), path_1, "path_1", 10.0);
    driver_1.GetSteeringController().SetLookAheadDistance(5);
    driver_1.GetSteeringController().SetGains(0.5, 0, 0);
    driver_1.GetSpeedController().SetGains(0.4, 0, 0);
    driver_1.Initialize();

    // Create and initialize the first vehicle
    HMMWV_Full hmmwv_2(&sys);
    hmmwv_2.SetInitPosition(ChCoordsys<>(ChVector<>(-90, +5.5, 1.0), QUNIT));
    hmmwv_2.SetPowertrainType(PowertrainModelType::SHAFTS);
    hmmwv_2.SetDriveType(DrivelineType::RWD);
    hmmwv_2.SetTireType(tire_model);
    hmmwv_2.SetTireStepSize(tire_step_size);
    hmmwv_2.SetVehicleStepSize(step_size);
    hmmwv_2.Initialize();
    hmmwv_2.SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv_2.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_2.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_2.SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv_2.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    auto patch_2 = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, +5.5, -0.1), QUNIT), ChVector<>(200, 10, 0.2));
    patch_2->SetContactFrictionCoefficient(friction_2);
    patch_2->SetContactRestitutionCoefficient(0.01f);
    patch_2->SetContactMaterialProperties(2e7f, 0.3f);
    patch_2->SetColor(ChColor(1.0f, 0.8f, 0.8f));
    patch_2->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 10);

    auto path_2 = StraightLinePath(ChVector<>(-90, +5.5, 0.1), ChVector<>(+90, +5.5, 0.1));
    ChPathFollowerDriver driver_2(hmmwv_2.GetVehicle(), path_2, "path_2", 0.0);
    driver_2.GetSteeringController().SetLookAheadDistance(5);
    driver_2.GetSteeringController().SetGains(0.5, 0, 0);
    driver_2.GetSpeedController().SetGains(0.4, 0, 0);
    driver_2.Initialize();

    terrain.Initialize();

    std::string modelname;
    switch (tire_model) {
        case TireModelType::TMEASY:
            modelname = "tmeasy";
            break;
        case TireModelType::FIALA:
            modelname = "fiala";
            break;
        case TireModelType::PAC89:
            modelname = "pac89";
            break;
        case TireModelType::PACEJKA:
            modelname = "pacejka";
            break;
        case TireModelType::RIGID:
            modelname = "rigid";
            break;
    }

    // Create the vehicle Irrlicht interface (associated with 2nd vehicle)
    ChWheeledVehicleIrrApp app(&hmmwv_2.GetVehicle(), &hmmwv_2.GetPowertrain(), L"Terrain friction test");
    app.SetSkyBox();
    app.AddTypicalLogo();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, .75), 6.0, 0.5);
    app.SetChaseCameraState(utils::ChChaseCamera::Track);
    app.SetChaseCameraPosition(ChVector<>(-50, -10, 2.0));
    app.SetTimestep(step_size);

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Initialize output
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    // ---------------
    // Simulation loop
    // ---------------

    double out_step_size = 1 / out_fps;
    int out_steps = (int)std::ceil(out_step_size / step_size);
    int sim_frame = 0;

    while (app.GetDevice()->run()) {
        double time = sys.GetChTime();

        double veh_pos_1 = hmmwv_1.GetVehicle().GetVehiclePos().x();
        double veh_pos_2 = hmmwv_2.GetVehicle().GetVehiclePos().x();

        // Extract output
        if (output && sim_frame % out_steps == 0) {
            double veh_speed_1 = hmmwv_1.GetVehicle().GetVehicleSpeed();
            double fr_omega_1 = hmmwv_1.GetVehicle().GetWheelOmega(FRONT_RIGHT);
            double rr_omega_1 = hmmwv_1.GetVehicle().GetWheelOmega(REAR_RIGHT);

            double veh_speed_2 = hmmwv_2.GetVehicle().GetVehicleSpeed();
            double fr_omega_2 = hmmwv_2.GetVehicle().GetWheelOmega(FRONT_RIGHT);
            double rr_omega_2 = hmmwv_2.GetVehicle().GetWheelOmega(REAR_RIGHT);

            csv << time;
            csv << veh_pos_1 << veh_speed_1 << fr_omega_1 << rr_omega_1;
            csv << veh_pos_2 << veh_speed_2 << fr_omega_2 << rr_omega_2;
            csv << std::endl;
        }

        // Stop befoe end of terrain patches
        if (veh_pos_1 > 80 || veh_pos_2 > 80) {
            break;
        }

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        double throttle_input = 0;
        if (time > 1 && time < 2)
            throttle_input = 0.75 * (time - 1);
        else if (time > 2)
            throttle_input = 0.75;

        // Collect output data from modules (for inter-module communication)
        double steering_input_1 = driver_1.GetSteering();
        double throttle_input_1 = throttle_input;
        double braking_input_1 = 0;

        double steering_input_2 = driver_2.GetSteering();
        double throttle_input_2 = throttle_input;
        double braking_input_2 = 0;

        // Update modules (process inputs from other modules)
        driver_1.Synchronize(time);
        driver_2.Synchronize(time);
        hmmwv_1.Synchronize(time, steering_input_1, braking_input_1, throttle_input_1, terrain);
        hmmwv_2.Synchronize(time, steering_input_2, braking_input_2, throttle_input_2, terrain);
        terrain.Synchronize(time);
        app.Synchronize(modelname, steering_input_1, throttle_input_1, braking_input_1);

        // Advance simulation for one timestep for all modules.
        driver_1.Advance(step_size);
        driver_2.Advance(step_size);
        hmmwv_1.Advance(step_size);
        hmmwv_2.Advance(step_size);
        terrain.Advance(step_size);
        app.Advance(step_size);

        // Advance state of entire system (containing both vehicles)
        sys.DoStepDynamics(step_size);

        // Increment simulation frame number
        sim_frame++;
    }

    if (output) {
        csv.write_to_file(out_dir + "/" + modelname + ".out");
    }

    return 0;
}
