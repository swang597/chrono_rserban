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
// Double lane change.  Tire kinematics check.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Type of tire model (FIALA, PACEJKA, PAC89, or TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Simulation step size
double step_size = 2e-3;
double tire_step_size = 1e-3;

// Debug logging
double debug_fps = 50;

// Output directories
const std::string out_dir = "DLC_TIRE_TEST";

// =============================================================================

int main(int argc, char* argv[]) {
    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChMaterialSurface::SMC);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector<>(-75, 0, 0.5), QUNIT));
    my_hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
    my_hmmwv.SetDriveType(DrivelineType::RWD);
    my_hmmwv.SetSteeringType(SteeringType::PITMAN_ARM);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.SetVehicleStepSize(step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Get handle to FR tire
    std::string outfile = "tire.out";
    ChTire* FR_tire = my_hmmwv.GetTire(FRONT_RIGHT);
    ChTMeasyTire* FR_TMeasy = nullptr;
    ChFialaTire* FR_Fiala = nullptr;
    ChPac89Tire* FR_Pac89 = nullptr;
    ChPacejkaTire* FR_Pacejka = nullptr;
    switch (tire_model) {
        case TireModelType::TMEASY:
            FR_TMeasy = static_cast<ChTMeasyTire*>(FR_tire);
            outfile = "tmeasy.out";
            break;
        case TireModelType::FIALA:
            FR_Fiala = static_cast<ChFialaTire*>(FR_tire);
            outfile = "fiala.out";
            break;
        case TireModelType::PAC89:
            FR_Pac89 = static_cast<ChPac89Tire*>(FR_tire);
            outfile = "pac89.out";
            break;
        case TireModelType::PACEJKA:
            FR_Pacejka = static_cast<ChPacejkaTire*>(FR_tire);
            outfile = "pacejka.out";
            break;
    }

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());
    auto patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, -5), QUNIT), ChVector<>(200, 20, 10));
    patch->SetContactFrictionCoefficient(0.8f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(1, 1, 1));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 20);
    terrain.Initialize();

    // Parameterized ISO double lane change (to left)
    auto path = DoubleLaneChangePath(ChVector<>(-75, 0, 0.1), 13.5, 4.0, 11.0, 50.0, true);
    auto npoints = path->getNumPoints();
    double x_max = path->getPoint(npoints-1).x();

    // Parameterized NATO double lane change (to right)
    ////auto path = DoubleLaneChangePath(ChVector<>(-125, 0, 0.1), 28.93, 3.6105, 25.0, 50.0, false);

    // Create the vehicle Irrlicht application
    ChVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"DLC test");
    app.SetHUDLocation(500, 20);
    app.SetSkyBox();
    app.AddTypicalLogo();
    app.AddTypicalLights(irr::core::vector3df(-150.f, 0.f, 200.f), irr::core::vector3df(-150.f, 0.f, 200.f), 100, 100);
    app.AddTypicalLights(irr::core::vector3df(150.f, 0.f, 200.f), irr::core::vector3df(150.0f, 0.f, 200.f), 100, 100);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);

    app.SetTimestep(step_size);

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // Create the driver system
    ChPathFollowerDriver driver(my_hmmwv.GetVehicle(), path, "my_path", 10.0);
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(0.8, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();

    // -----------------
    // Initialize output
    // -----------------
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    // ---------------
    // Simulation loop
    // ---------------

    // Number of simulation steps between miscellaneous events
    double debug_step_size = 1 / debug_fps;
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;
    int sim_frame = 0;

    while (app.GetDevice()->run()) {
        // Extract system state
        double time = my_hmmwv.GetSystem()->GetChTime();

        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();

        if (my_hmmwv.GetVehicle().GetVehiclePos().x() > x_max - 20) {
            throttle_input = 0;
            steering_input = 0;
            braking_input = 1;
        }

        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Debug logging
        if (sim_frame % debug_steps == 0) {
            csv << time;
            csv << FR_tire->GetSlipAngle() << FR_tire->GetCamberAngle() << FR_tire->GetLongitudinalSlip();
            switch (tire_model) {
                case TireModelType::TMEASY:
                    csv << FR_TMeasy->GetSlipAngle_internal() << FR_TMeasy->GetCamberAngle_internal()
                        << FR_TMeasy->GetLongitudinalSlip_internal();
                    break;
                case TireModelType::FIALA:
                    csv << FR_Fiala->GetSlipAngle_internal() << FR_Fiala->GetCamberAngle_internal()
                        << FR_Fiala->GetLongitudinalSlip_internal();
                    break;
                case TireModelType::PAC89:
                    csv << FR_Pac89->GetSlipAngle_internal() << FR_Pac89->GetCamberAngle_internal()
                        << FR_Pac89->GetLongitudinalSlip_internal();
                    break;
                case TireModelType::PACEJKA:
                    csv << FR_Pacejka->GetSlipAngle_internal() << FR_Pacejka->GetCamberAngle_internal()
                        << FR_Pacejka->GetLongitudinalSlip_internal();
                    break;
            }
            csv << std::endl;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize(outfile, steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        app.Advance(step_size);

        // Increment simulation frame number
        sim_frame++;

        app.EndScene();
    }

    csv.write_to_file(out_dir + "/" + outfile);

    return 0;
}
