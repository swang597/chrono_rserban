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
// Main driver function for a vehicle specified through JSON files.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_FialaTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac89Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac02Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_RigidTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_TMeasyTire.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Vehicle JSON specification
////std::string vehicle_json = "hmmwv/vehicle/HMMWV_Vehicle.json";
std::string vehicle_json = "generic/vehicle/Vehicle_RigidRigidSuspension.json";

// Powertrain JSON specification
bool add_powertrain = false;
std::string powertrain_json = "hmmwv/powertrain/HMMWV_ShaftsPowertrain.json";

// Type of tire model (RIGID, RIGID_MESH, PACEJKA, LUGRE, FIALA, PAC89, PAC02, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

enum class TerrainType { FLAT, RIGID };
TerrainType terrain_type = TerrainType::RIGID;

// Terrain length (X direction)
double terrainLength = 400.0;

// Lane direction
double yaw_angle = 0 * CH_C_DEG_TO_RAD;

// Initial chassis velocity if driveline disconnected
bool disconnect_driveline = true;
double init_vel = 10;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = 1e-3;

// =============================================================================
void CreateTires(WheeledVehicle& vehicle, VisualizationType tire_vis);

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    if (terrain_type == TerrainType::FLAT &&
        (tire_model == TireModelType::RIGID || tire_model == TireModelType::RIGID_MESH)) {
        std::cout << "Flat terrain incompatible with a rigid tire model!" << std::endl;
        return 1;
    }

    ChQuaternion<> yaw_rot = Q_from_AngZ(yaw_angle);
    ChCoordsys<> patch_sys(VNULL, yaw_rot);
    ChVector<> init_loc = patch_sys.TransformPointLocalToParent(ChVector<>(-terrainLength / 2 + 5, 0, 0.7));
    ChVector<> path_start = patch_sys.TransformPointLocalToParent(ChVector<>(-terrainLength / 2, 0, 0.5));
    ChVector<> path_end = patch_sys.TransformPointLocalToParent(ChVector<>(+terrainLength / 2, 0, 0.5));

    // Create the vehicle system
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_json), ChContactMethod::SMC);
    if (disconnect_driveline) {
        vehicle.Initialize(ChCoordsys<>(init_loc, yaw_rot), init_vel);
        vehicle.DisconnectDriveline();
    } else {
        vehicle.Initialize(ChCoordsys<>(init_loc, yaw_rot));    
    }
    vehicle.DisconnectDriveline();
    vehicle.GetChassis()->SetFixed(false);

    vehicle.SetChassisVisualizationType(VisualizationType::NONE);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::NONE);

    // Create and initialize the powertrain system
    if (add_powertrain) {
        auto powertrain = ReadPowertrainJSON(vehicle::GetDataFile(powertrain_json));
        vehicle.InitializePowertrain(powertrain);
    }

    // Create and initialize the tires
    auto tire_vis = (tire_model == TireModelType::RIGID_MESH ? VisualizationType::MESH : VisualizationType::PRIMITIVES);
    CreateTires(vehicle, tire_vis);

    // Containing system
    auto system = vehicle.GetSystem();

    // Create the terrain
    std::shared_ptr<ChTerrain> terrain;
    switch (terrain_type) {
        case TerrainType::RIGID:
        default: {
            auto rigid_terrain = chrono_types::make_shared<RigidTerrain>(system);
            auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            patch_mat->SetFriction(0.9f);
            patch_mat->SetRestitution(0.01f);
            patch_mat->SetYoungModulus(2e7f);
            patch_mat->SetPoissonRatio(0.3f);
            auto patch = rigid_terrain->AddPatch(patch_mat, ChCoordsys<>(ChVector<>(0), yaw_rot), terrainLength, 5);
            patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 5);
            rigid_terrain->Initialize();
            terrain = rigid_terrain;
            break;
        }
        case TerrainType::FLAT: {
            auto flat_terrain = chrono_types::make_shared<FlatTerrain>(0, 0.9f);
            terrain = flat_terrain;
            break;
        }
    }

    // Create the straight path and the driver system
    auto path = StraightLinePath(path_start, path_end, 1);
    ChPathFollowerDriver driver(vehicle, path, "my_path", 1000.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Create Irrilicht visualization
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Vehicle acceleration - JSON specification");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vehicle.SetVisualSystem(vis);

    vehicle.LogSubsystemTypes();

    // Simulation loop
    int step_number = 0;
    double time = 0;

    ChTimer<> timer;
    timer.start();
    while (vis->Run()) {
        time = system->GetChTime();

        // End simulation
        if (time >= 15)
            break;

        vis->BeginScene();
        vis->DrawAll();

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain->Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, *terrain);
        vis->Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain->Advance(step_size);
        vehicle.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;

        // Draw a coordinate system aligned with the world frame
        irrlicht::tools::drawCoordsys(vis.get(), ChCoordsys<>(vehicle.GetPos(), QUNIT), 2);

        vis->EndScene();
    }

    std::cout << "Final vehicle speed: " << vehicle.GetSpeed() << std::endl;

    return 0;
}

// ----------------------------------------------------------------------

void CreateTires(WheeledVehicle& vehicle, VisualizationType tire_vis) {
    switch (tire_model) {
        case TireModelType::RIGID:
        case TireModelType::RIGID_MESH: {
            bool use_mesh = (tire_model == TireModelType::RIGID_MESH);

            auto tire_FL = chrono_types::make_shared<HMMWV_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<HMMWV_RigidTire>("FR", use_mesh);
            auto tire_RL = chrono_types::make_shared<HMMWV_RigidTire>("RL", use_mesh);
            auto tire_RR = chrono_types::make_shared<HMMWV_RigidTire>("RR", use_mesh);

            vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[RIGHT], tire_vis);
            vehicle.InitializeTire(tire_RL, vehicle.GetAxle(1)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_RR, vehicle.GetAxle(1)->m_wheels[RIGHT], tire_vis);

            break;
        }
        case TireModelType::FIALA: {
            auto tire_FL = chrono_types::make_shared<HMMWV_FialaTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_FialaTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_FialaTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_FialaTire>("RR");

            vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[RIGHT], tire_vis);
            vehicle.InitializeTire(tire_RL, vehicle.GetAxle(1)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_RR, vehicle.GetAxle(1)->m_wheels[RIGHT], tire_vis);

            break;
        }
        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<HMMWV_TMeasyTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_TMeasyTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_TMeasyTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_TMeasyTire>("RR");

            vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[RIGHT], tire_vis);
            vehicle.InitializeTire(tire_RL, vehicle.GetAxle(1)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_RR, vehicle.GetAxle(1)->m_wheels[RIGHT], tire_vis);

            break;
        }
        case TireModelType::PAC89: {
            auto tire_FL = chrono_types::make_shared<HMMWV_Pac89Tire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_Pac89Tire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_Pac89Tire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_Pac89Tire>("RR");

            vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[RIGHT], tire_vis);
            vehicle.InitializeTire(tire_RL, vehicle.GetAxle(1)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_RR, vehicle.GetAxle(1)->m_wheels[RIGHT], tire_vis);

            break;
        }
        case TireModelType::PAC02: {
            auto tire_FL = chrono_types::make_shared<HMMWV_Pac02Tire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_Pac02Tire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_Pac02Tire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_Pac02Tire>("RR");

            vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[RIGHT], tire_vis);
            vehicle.InitializeTire(tire_RL, vehicle.GetAxle(1)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_RR, vehicle.GetAxle(1)->m_wheels[RIGHT], tire_vis);

            break;
        }
        default:
            break;
    }

    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            wheel->GetTire()->SetCollisionType(ChTire::CollisionType::SINGLE_POINT);
            if (tire_step_size > 0)
                wheel->GetTire()->SetStepsize(tire_step_size);
        }
    }
}