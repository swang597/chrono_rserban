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

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/m113/M113_SimplePowertrain.h"
#include "chrono_models/vehicle/m113/M113_Vehicle.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

// =============================================================================
// Problem parameters

// Terrain friction coefficients
float friction_1 = 0.1f;
float friction_2 = 0.9f;

// Simulation step size, end time
double step_size = 1e-3;

// Output
bool output = false;
double out_fps = 10;
const std::string out_dir = "../M113_FRICTION";

// =============================================================================

int main(int argc, char* argv[]) {
    // Chrono system
    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));
    sys.SetMaxItersSolverSpeed(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // Create the terrain (will contain 2 side-by-side patches)
    RigidTerrain terrain(&sys);

    // Create and initialize the first vehicle
    M113_Vehicle vehicle_1(false, TrackShoeType::SINGLE_PIN, &sys, ChassisCollisionType::NONE);
    vehicle_1.Initialize(ChCoordsys<>(ChVector<>(-90.0, -5.5, 1.0), QUNIT));
    vehicle_1.SetChassisVisualizationType(VisualizationType::NONE);
    vehicle_1.SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_1.SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_1.SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_1.SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_1.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    M113_SimplePowertrain powertrain_1("Powertrain1");
    powertrain_1.Initialize(vehicle_1.GetChassisBody(), vehicle_1.GetDriveshaft());

    auto patch_1 = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, -5.5, -0.1), QUNIT), ChVector<>(200, 10, 0.2));
    patch_1->SetContactFrictionCoefficient(friction_1);
    patch_1->SetContactRestitutionCoefficient(0.01f);
    patch_1->SetContactMaterialProperties(2e7f, 0.3f);
    patch_1->SetColor(ChColor(0.8f, 0.8f, 1.0f));
    patch_1->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 10);

    auto path_1 = StraightLinePath(ChVector<>(-90, -5.5, 0.1), ChVector<>(+90, -5.5, 0.1));
    ChPathFollowerDriver driver_1(vehicle_1, path_1, "path_1", 10.0);
    driver_1.GetSteeringController().SetLookAheadDistance(5);
    driver_1.GetSteeringController().SetGains(0.5, 0, 0);
    driver_1.GetSpeedController().SetGains(0.4, 0, 0);
    driver_1.Initialize();

    // Create and initialize the first vehicle
    M113_Vehicle vehicle_2(false, TrackShoeType::SINGLE_PIN, &sys, ChassisCollisionType::NONE);
    vehicle_2.Initialize(ChCoordsys<>(ChVector<>(-90.0, +5.5, 1.0), QUNIT));
    vehicle_2.SetChassisVisualizationType(VisualizationType::NONE);
    vehicle_2.SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_2.SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_2.SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_2.SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_2.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    M113_SimplePowertrain powertrain_2("Powertrain2");
    powertrain_2.Initialize(vehicle_2.GetChassisBody(), vehicle_2.GetDriveshaft());

    auto patch_2 = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, +5.5, -0.1), QUNIT), ChVector<>(200, 10, 0.2));
    patch_2->SetContactFrictionCoefficient(friction_2);
    patch_2->SetContactRestitutionCoefficient(0.01f);
    patch_2->SetContactMaterialProperties(2e7f, 0.3f);
    patch_2->SetColor(ChColor(1.0f, 0.8f, 0.8f));
    patch_2->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 10);

    auto path_2 = StraightLinePath(ChVector<>(-90, +5.5, 0.1), ChVector<>(+90, +5.5, 0.1));
    ChPathFollowerDriver driver_2(vehicle_2, path_2, "path_2", 0.0);
    driver_2.GetSteeringController().SetLookAheadDistance(5);
    driver_2.GetSteeringController().SetGains(0.5, 0, 0);
    driver_2.GetSpeedController().SetGains(0.4, 0, 0);
    driver_2.Initialize();

    terrain.Initialize();

    // Create the vehicle Irrlicht interface (associated with 2nd vehicle)
    ChTrackedVehicleIrrApp app(&vehicle_2, &powertrain_2, L"Terrain friction test");
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

    // Inter-module communication data
    BodyStates shoe_states_left_1(vehicle_1.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right_1(vehicle_1.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left_1(vehicle_1.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right_1(vehicle_1.GetNumTrackShoes(RIGHT));
    double driveshaft_speed_1;
    double powertrain_torque_1;

    BodyStates shoe_states_left_2(vehicle_2.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right_2(vehicle_2.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left_2(vehicle_2.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right_2(vehicle_2.GetNumTrackShoes(RIGHT));
    double driveshaft_speed_2;
    double powertrain_torque_2;

    // Output interval
    double out_step_size = 1 / out_fps;
    int out_steps = (int)std::ceil(out_step_size / step_size);
    int sim_frame = 0;

    while (app.GetDevice()->run()) {
        double time = sys.GetChTime();

        double veh_pos_1 = vehicle_1.GetVehiclePos().x();
        double veh_pos_2 = vehicle_2.GetVehiclePos().x();
        double veh_speed_1 = vehicle_1.GetVehicleSpeed();
        double veh_speed_2 = vehicle_2.GetVehicleSpeed();

        // Extract output
        if (output && sim_frame % out_steps == 0) {
            csv << time;
            csv << veh_pos_1 << veh_speed_1;
            csv << veh_pos_2 << veh_speed_2;
            csv << std::endl;

            std::cout << veh_pos_1 << "   " << veh_pos_2 << std::endl;
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

        powertrain_torque_1 = powertrain_1.GetOutputTorque();
        driveshaft_speed_1 = vehicle_1.GetDriveshaftSpeed();
        vehicle_1.GetTrackShoeStates(LEFT, shoe_states_left_1);
        vehicle_1.GetTrackShoeStates(RIGHT, shoe_states_right_1);

        powertrain_torque_2 = powertrain_2.GetOutputTorque();
        driveshaft_speed_2 = vehicle_2.GetDriveshaftSpeed();
        vehicle_2.GetTrackShoeStates(LEFT, shoe_states_left_2);
        vehicle_2.GetTrackShoeStates(RIGHT, shoe_states_right_2);

        // Update modules (process inputs from other modules)
        driver_1.Synchronize(time);
        driver_2.Synchronize(time);
        powertrain_1.Synchronize(time, throttle_input_1, driveshaft_speed_1);
        powertrain_2.Synchronize(time, throttle_input_2, driveshaft_speed_2);
        vehicle_1.Synchronize(time, steering_input_1, braking_input_1, powertrain_torque_1, shoe_forces_left_1,
                              shoe_forces_right_1);
        vehicle_2.Synchronize(time, steering_input_2, braking_input_2, powertrain_torque_2, shoe_forces_left_2,
                              shoe_forces_right_2);
        terrain.Synchronize(time);
        app.Synchronize("", steering_input_2, throttle_input_2, braking_input_2);

        // Advance simulation for one timestep for all modules.
        driver_1.Advance(step_size);
        driver_2.Advance(step_size);
        powertrain_1.Advance(step_size);
        powertrain_2.Advance(step_size);
        vehicle_1.Advance(step_size);
        vehicle_2.Advance(step_size);
        terrain.Advance(step_size);
        app.Advance(step_size);

        // Advance state of entire system (containing both vehicles)
        sys.DoStepDynamics(step_size);

        // Increment simulation frame number
        sim_frame++;
    }

    if (output) {
        csv.write_to_file(out_dir + "/m113.out");
    }

    return 0;
}
