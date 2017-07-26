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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// Sample test program for sequential WVP simulation.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
// All units SI.
//
// =============================================================================

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_models/vehicle/wvp/WVP_FollowerDataDriver.h"

#include "chrono_models/vehicle/wvp/WVP.h"

#include <chrono>
#include <thread>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::wvp;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);

ChVector<>gravity(0,2.80604,-9.353468);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::NONE;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

// Type of tire model (RIGID, FIALA)
TireModelType tire_model = TireModelType::RIGID;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = step_size;

// Simulation end time
double tend = 15;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

//vehicle driver inputs
// Desired vehicle speed (m/s)
double mph_to_ms = 0.44704;
double target_speed = 20*mph_to_ms;

std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("wvp/SteeringController.json");
std::string speed_controller_file("wvp/SpeedController.json");

std::string steering_input_file("wvp/30Per Side Slope - Left Side Down Serpentine.csv");

// =============================================================================

int main(int argc, char* argv[]) {
    // --------------
    // Create systems
    // --------------

    // Create the vehicle, set parameters, and initialize
    WVP wvp;
    wvp.SetChassisFixed(false);
    wvp.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    wvp.SetTireType(tire_model);
    wvp.SetTireStepSize(tire_step_size);
    wvp.Initialize();

    wvp.SetChassisVisualizationType(chassis_vis_type);
    wvp.SetSuspensionVisualizationType(suspension_vis_type);
    wvp.SetSteeringVisualizationType(steering_vis_type);
    wvp.SetWheelVisualizationType(wheel_vis_type);
    wvp.SetTireVisualizationType(tire_vis_type);

    std::cout << "Total vehicle mass: " << wvp.GetVehicle().GetVehicleMass() << std::endl;

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(wvp.GetSystem());
    terrain.SetContactFrictionCoefficient(0.9f);
    terrain.SetContactRestitutionCoefficient(0.01f);
    terrain.SetContactMaterialProperties(2e7f, 0.3f);
    terrain.SetColor(ChColor(0.8f, 0.8f, 0.5f));
    terrain.SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize(0, 1000, 200);

    //set gravity to 30 percent side slope
    wvp.GetSystem()->Set_G_acc(gravity);

    //create the driver
    // auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    // ChPathFollowerDriver driver(wvp.GetVehicle(), vehicle::GetDataFile(steering_controller_file),
    //                             vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed, false);
    
    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    WVP_FollowerDataDriver driver(wvp.GetVehicle(),
            vehicle::GetDataFile(steering_controller_file),
            vehicle::GetDataFile(speed_controller_file), path, "side_slope_slalom",
            target_speed, vehicle::GetDataFile(steering_input_file), 1, 2, 5.0
            );

    driver.Initialize();

    // -------------------------------------
    // Create the vehicle Irrlicht interface
    // Create the driver system
    // -------------------------------------

    ChWheeledVehicleIrrApp app(&wvp.GetVehicle(), &wvp.GetPowertrain(), L"WVP sequential test");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    /*app.SetTimestep(step_size);*/
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create the interactive driver system
    /*ChIrrGuiDriver driver(app);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    driver.Initialize();*/

    // ---------------
    // Simulation loop
    // ---------------

    std::cout<<"data at: "<<vehicle::GetDataFile(steering_controller_file)<<std::endl;
    std::cout<<"data at: "<<vehicle::GetDataFile(speed_controller_file)<<std::endl;
    std::cout<<"data at: "<<vehicle::GetDataFile(path_file)<<std::endl;

    int render_steps = (int)std::ceil(render_step_size / step_size);
    int step_number = 0;


    while (app.GetDevice()->run()) {
        double time = wvp.GetSystem()->GetChTime();

        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();
        }
        // std::cout<<"drawing scene\n";
        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();

        // std::cout<<"about to Synchronize\n";
        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        wvp.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize("Follower driver", steering_input, throttle_input, braking_input);

        // std::cout<<"about to advance\n";
        // Advance simulation for one timestep for all modules

        driver.Advance(step_size);
        terrain.Advance(step_size);
        wvp.Advance(step_size);
        app.Advance(step_size);

        // std::cout<<"steerting input: "<<steering_input<<std::endl;

        // Increment frame number
        step_number++;
    }

    return 0;
}
