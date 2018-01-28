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

#include "chrono_models/vehicle/wvp/WVP.h"

#include <chrono>
#include <thread>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::wvp;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::NONE;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

// Type of tire model (RIGID, FIALA, PAC89)
TireModelType tire_model = TireModelType::RIGID;

// Type of steering model (PITMAN_ARM or PITMAN_ARM_SHAFTS)
SteeringType steering_model = SteeringType::PITMAN_ARM;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = step_size;

// Simulation end time
double tend = 15;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

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
    wvp.setSteeringType(steering_model);
    wvp.SetInitFwdVel(0.0);
    wvp.Initialize();

    wvp.SetChassisVisualizationType(chassis_vis_type);
    wvp.SetSuspensionVisualizationType(suspension_vis_type);
    wvp.SetSteeringVisualizationType(steering_vis_type);
    wvp.SetWheelVisualizationType(wheel_vis_type);
    wvp.SetTireVisualizationType(tire_vis_type);

    {
        auto suspF = std::static_pointer_cast<WVP_DoubleWishboneFront>(wvp.GetVehicle().GetSuspension(0));
        auto springFL = suspF->GetSpring(VehicleSide::LEFT);
        auto shockFL = suspF->GetShock(VehicleSide::RIGHT);

        std::cout << "Spring rest length front: " << springFL->GetSpringRestLength() << std::endl;
        std::cout << "Shock rest length front:  " << shockFL->GetSpringRestLength() << std::endl;
    }
    {
        auto suspR = std::static_pointer_cast<WVP_DoubleWishboneRear>(wvp.GetVehicle().GetSuspension(1));
        auto springRL = suspR->GetSpring(VehicleSide::LEFT);
        auto shockRL = suspR->GetShock(VehicleSide::RIGHT);

        std::cout << "Spring rest length rear: " << springRL->GetSpringRestLength() << std::endl;
        std::cout << "Shock rest length rear:  " << shockRL->GetSpringRestLength() << std::endl;
    }

    std::cout << "Vehicle mass:               " << wvp.GetVehicle().GetVehicleMass() << std::endl;
    std::cout << "Vehicle mass (with tires):  " << wvp.GetTotalMass() << std::endl;

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(wvp.GetSystem());
    auto patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, -5), QUNIT), ChVector<>(600, 600, 10));
    patch->SetContactFrictionCoefficient(0.1f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // -------------------------------------
    // Create the vehicle Irrlicht interface
    // Create the driver system
    // -------------------------------------

    ChWheeledVehicleIrrApp app(&wvp.GetVehicle(), &wvp.GetPowertrain(), L"WVP sequential test");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create the interactive driver system
    ChIrrGuiDriver driver(app);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

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

        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        wvp.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        wvp.Advance(step_size);
        app.Advance(step_size);
        // ChQuaternion<> qW1= wvp.GetVehicle().GetWheelRot(1);
        // ChQuaternion<> qW0= wvp.GetVehicle().GetWheelRot(0);
        // ChQuaternion<> qV= wvp.GetVehicle().GetVehicleRot();
        // std::cout<<"Wheel0|"<<(qW0.Q_to_NasaAngles().z()*180/CH_C_PI) - (qV.Q_to_NasaAngles().z()*180/CH_C_PI)
        //     <<"\t|Wheel1|"<<(qW1.Q_to_NasaAngles().z()*180/CH_C_PI) - (qV.Q_to_NasaAngles().z()*180/CH_C_PI)<<std::endl;


        auto susp0 = std::static_pointer_cast<ChDoubleWishbone>(wvp.GetVehicle().GetSuspension(0));
        auto susp1 = std::static_pointer_cast<ChDoubleWishbone>(wvp.GetVehicle().GetSuspension(1));

        // std::cout<<"SpringDef0|"<<susp0->GetSpringLength(LEFT)
        // // <<"|force0|"<<susp0->GetSpringForce(LEFT);
        // <<"|SpringDef1|"<<susp0->GetSpringLength(RIGHT)
        // // <<"|force1|"<<susp0->GetSpringForce(RIGHT);
        // <<"|SpringDef2|"<<susp1->GetSpringLength(LEFT)
        // // <<"|force2|"<<susp1->GetSpringForce(LEFT);
        // <<"|SpringDef3|"<<susp1->GetSpringLength(RIGHT)
        // // <<"|force3|"<<susp1->GetSpringForce(RIGHT)
        // <<std::endl;

        ChQuaternion<> q= wvp.GetVehicle().GetVehicleRot();
        std::cout<<"attitude|"<<q.Q_to_NasaAngles().x()<<"|bank|"<<q.Q_to_NasaAngles().y()<<"|heading|"<<q.Q_to_NasaAngles().z()<<std::endl;


        // std::cout<<wvp.GetVehicle().GetWheelPos(0).x()<<"|"<<wvp.GetVehicle().GetWheelPos(0).y()<<"|"<<wvp.GetVehicle().GetWheelPos(0).z()<<"|||"
        //     <<wvp.GetVehicle().GetWheelPos(2).x()<<"|"<<wvp.GetVehicle().GetWheelPos(2).y()<<"|"<<wvp.GetVehicle().GetWheelPos(2).z()<<std::endl;

        //log suspension locations
        /*std::cout<<"|wheel pos|"<<wvp.GetVehicle().GetSuspension(0)->GetSpindle(LEFT)->GetPos().x()-wvp.GetVehicle().GetVehiclePos().x()
          <<"|"<<wvp.GetVehicle().GetSuspension(0)->GetSpindle(LEFT)->GetPos().y()-wvp.GetVehicle().GetVehiclePos().y()
          <<"|"<<wvp.GetVehicle().GetSuspension(0)->GetSpindle(LEFT)->GetPos().z()-wvp.GetVehicle().GetVehiclePos().z()
          <<std::endl;*/



        /*wvp.LogHardpointLocations();*/

        //check engine vs wheel speed
        /*std::cout<<"Engine Speed|"<<wvp.GetPowertrain().GetMotorSpeed()*(30.0/3.14159)
          <<"|vehicle speed|"<<wvp.GetVehicle().GetVehicleSpeed()
          <<"|vehicle gear|"<<wvp.GetPowertrain().GetCurrentTransmissionGear()
          <<"|time|"<<wvp.GetSystem()->GetChTime()
          <<"|throttle|"<<throttle_input
          <<std::endl;*/

        /*for(int i=0;i<4;i++){*/
          /*std::cout<<"wheel:"<<i<< "|Z Force:|" << wvp.GetTire(i)->ReportTireForce(&terrain).force.z()<<"|";*/
        /*}*/
        /*std::cout<<std::endl;*/

        // std::cout<<"Vehicle COM|"<<wvp.GetVehicle().GetVehicleCOMPos().x()-wvp.GetVehicle().GetVehiclePos().x()<<"|"
        //   <<wvp.GetVehicle().GetVehicleCOMPos().y()-wvp.GetVehicle().GetVehiclePos().y()<<"|"
        //   <<(wvp.GetVehicle().GetVehicleCOMPos().z()-wvp.GetVehicle().GetVehiclePos().z());
        // std::cout<<std::endl;

        /*std::cout<<"COM: "<< wvp.GetChassis()->GetCOMPos().x() <<", "<<wvp.GetChassis()->GetCOMPos().y()<<", "<<wvp.GetChassis()->GetCOMPos().z()<< std::endl;*/

        // Increment frame number
        step_number++;
    }

    return 0;
}
