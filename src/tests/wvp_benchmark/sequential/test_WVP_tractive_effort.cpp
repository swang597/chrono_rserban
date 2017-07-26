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

#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

#include "chrono_models/vehicle/wvp/WVP.h"

#include <chrono>
#include <thread>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::wvp;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0,0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);

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
double target_speed = 1000;

std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("wvp/SteeringController.json");
std::string speed_controller_file("wvp/SpeedController.json");

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

    // Deformable terrain properties (LETE sand)
    double Kphi = 5301e3;
    double Kc = 102e3;
    double n = 0.793;
    double c = 1.3e3;
    double phi = 31.1;
    double K = 1.2e-2;
    double E_elastic = 2e8;
    double damping = 3e4;

    float mu = 0.8f;
    float restitution = 0.01f;
    float E = 2e7f;
    float nu = 0.3f;
    double depth = 10;
    double factor = 10;

    auto terrain = new SCMDeformableTerrain(wvp.GetSystem());
    terrain->SetPlane(ChCoordsys<>(VNULL, Q_from_AngX(CH_C_PI_2)));
    terrain->SetSoilParametersSCM(Kphi,Kc,n,c,phi,K,E_elastic,damping);
    terrain->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE_YIELD,0,30000.2);
    terrain->Initialize(terrainHeight,terrainLength,terrainWidth,terrainLength * factor, terrainWidth*factor);

    //create the driver
    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(wvp.GetVehicle(), vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed, false);
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

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

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

    std::cout << "Vehicle COM: " << wvp.GetVehicle().GetVehicleCOMPos().x() <<", "<<wvp.GetVehicle().GetVehicleCOMPos().y() <<", "<<wvp.GetVehicle().GetVehicleCOMPos().z() << std::endl;
    std::cout<< "Front Susp COM x: "<<wvp.GetVehicle().GetSuspension(0)->GetCOMPos().x()<<std::endl;
    std::cout<< "Rear Susp COM x: "<<wvp.GetVehicle().GetSuspension(1)->GetCOMPos().x()<<std::endl;
    std::cout<< "Steering COM x: "<<wvp.GetVehicle().GetSteering(0)->GetCOMPos().x()<<std::endl;
    std::cout<< "Wheel0 COM x: "<<wvp.GetVehicle().GetWheel(0)->GetCOMPos().x()<<std::endl;
    std::cout<< "Wheel1 COM x: "<<wvp.GetVehicle().GetWheel(1)->GetCOMPos().x()<<std::endl;
    std::cout<< "Wheel2 COM x: "<<wvp.GetVehicle().GetWheel(2)->GetCOMPos().x()<<std::endl;
    std::cout<< "Wheel3 COM x: "<<wvp.GetVehicle().GetWheel(3)->GetCOMPos().x()<<std::endl;


    while (app.GetDevice()->run()) {
        double time = wvp.GetSystem()->GetChTime();

        //path visualization
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        // std::cout<<"Target:\t"<<(irr::f32)pT.x()<<",\t "<<(irr::f32)pT.y()<<",\t "<<(irr::f32)pT.z()<<std::endl;
        // std::cout<<"Vehicle:\t"<<wvp.GetVehicle().GetChassisBody()->GetPos().x()
        //   <<",\t "<<wvp.GetVehicle().GetChassisBody()->GetPos().y()<<",\t "
        //   <<wvp.GetVehicle().GetChassisBody()->GetPos().z()<<std::endl;

        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();
        }
        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        wvp.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize("Follower driver", steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules

        driver.Advance(step_size);
        terrain.Advance(step_size);
        wvp.Advance(step_size);
        app.Advance(step_size);


        //check engine vs wheel speed
        /*std::cout<<"Engine Speed|"<<wvp.GetPowertrain().GetMotorSpeed()*(30.0/3.14159)
          <<"|vehicle speed|"<<wvp.GetVehicle().GetVehicleSpeed()
          <<"|vehicle gear|"<<wvp.GetPowertrain().GetCurrentTransmissionGear()
          <<"|time|"<<wvp.GetSystem()->GetChTime()
          <<"|throttle|"<<throttle_input
          <<std::endl;*/

        /*for(int i=0;i<4;i++){*/
          /*std::cout<<"wheel:"<<i<< "|Z Force:|" << wvp.GetTire(i)->GetTireForce(true).force.z()<<"|";*/
        /*}*/
        /*std::cout<<std::endl;*/

        /*std::cout<<"Vehicle COM|"<<wvp.GetVehicle().GetVehicleCOMPos().x()<<"|"
          <<wvp.GetVehicle().GetVehicleCOMPos().y()<<"|"
          <<(wvp.GetVehicle().GetVehicleCOMPos().z()-0.548)*/
          /*std::cout<<std::endl;*/

        /*std::cout<<"COM: "<< wvp.GetChassis()->GetCOMPos().x() <<", "<<wvp.GetChassis()->GetCOMPos().y()<<", "<<wvp.GetChassis()->GetCOMPos().z()<< std::endl;*/

        // Increment frame number
        step_number++;
    }

    return 0;
}
