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

//#define USE_IRRLICHT


#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#ifdef USE_IRRLICHT
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#endif

#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_models/vehicle/wvp/WVP_FollowerDataDriver.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/core/ChFileutils.h"

#include "chrono_models/vehicle/wvp/WVP.h"

#include <chrono>
#include <thread>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::wvp;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0,0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::NONE;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

// Type of tire model (RIGID, FIALA, PAC89)
TireModelType tire_model = TireModelType::PAC89;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step sizes
double step_size =1e-4;
double tire_step_size = step_size;

// Simulation end time
double tend = 15;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50
double output_step_size = 1e-2;

//output directory
const std::string out_dir = "../WVP_DLC_PAVED";
const std::string pov_dir = out_dir + "/POVRAY";
bool povray_output = false;
bool data_output = true;

//vehicle driver inputs
// Desired vehicle speed (m/s)
double mph_to_ms = 0.44704;
double target_speed = 35*mph_to_ms;

// std::string path_file("paths/NATO_double_lane_change.txt");
std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("wvp/SteeringController.json");
std::string speed_controller_file("wvp/SpeedController.json");

std::string steering_input_file("wvp/DLC_Paved_RtL_35mph.csv");
std::string steering_input_fileRtL48("wvp/DLC_Paved_RtL_48mph.csv");
std::string steering_input_fileLtR35("wvp/DLC_Paved_LtR_35mph.csv");
std::string steering_input_fileLtR48("wvp/DLC_Paved_LtR_48mph.csv");

std::string output_file_name("DLC_Paved_RtL_35");

bool LtR = false;

// =============================================================================

int main(int argc, char* argv[]) {
    //send in args: filename time speed LtR(or RtL)
    //read from args
    if(argc > 2){
        std::cout<<argv[0]<<"|"<<argv[1]<<"|"<<argv[2]<<"|"<<argv[3]<<std::endl;
        tend = atof(argv[1]);
        target_speed = atof(argv[2]);
        if(atof(argv[3])==0){
            LtR = true;
        }
        else if(atof(argv[3])==1){ //0 = LTR, 1=RtL
            LtR = false;
        }

        //set the steering input file accordingly
        if(LtR && target_speed==35){
            steering_input_file = steering_input_fileLtR35;
            output_file_name = "DLC_Paved_LtR_35";
            std::cout<<"LtR 35mph"<<std::endl;
        }
        else if(LtR && target_speed==48){
            steering_input_file = steering_input_fileLtR48;
            output_file_name = "DLC_Paved_LtR_48";
            std::cout<<"LtR 48mph"<<std::endl;
        }
        else if(!LtR && target_speed==48){
            steering_input_file = steering_input_fileRtL48;
            output_file_name = "DLC_Paved_RtL_48";
            std::cout<<"RtL 48mph"<<std::endl;
        }
        else{
            std::cout<<"RtL 35mph"<<std::endl;
        }
        target_speed = target_speed*mph_to_ms;
    }
    else{
        std::cout<<"RtL 35mph"<<std::endl;
    }

    

    // if an additional parameter given, run at that speed instead
    // if(argc > 3){
    //     target_speed = atof(argv[4])*mph_to_ms;
    //     std::cout<<"Target Speed"<<target_speed<<std::endl;
    //     output_file_name += "_closed_" + std::to_string((int)target_speed);
    // }

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
    auto patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, -5), QUNIT), ChVector<>(1000, 30, 10));
    patch->SetContactFrictionCoefficient(0.8f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 20, 20);
    terrain.Initialize();

    //create the driver -> path follower
    /*auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(wvp.GetVehicle(), vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed, false);*/

    //create the path follower/data follower combined driver
    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    //driver for open loop controller
    WVP_FollowerDataDriver driver(wvp.GetVehicle(),
            vehicle::GetDataFile(steering_controller_file),
            vehicle::GetDataFile(speed_controller_file), path, "double_lane_change",
            target_speed, vehicle::GetDataFile(steering_input_file), 1, 3, 20.0
            );

    //for closed loop, can just not switch to data for now
    // WVP_FollowerDataDriver driver(wvp.GetVehicle(),
    //         vehicle::GetDataFile(steering_controller_file),
    //         vehicle::GetDataFile(speed_controller_file), path, "double_lane_change",
    //         target_speed, vehicle::GetDataFile(steering_input_file), 1, 3, 800.0
    //         );


    driver.Initialize();


#ifdef USE_IRRLICHT
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
#endif
    
    // -------------
    // Prepare output
    // -------------

    if (data_output || povray_output) {
        if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    if (povray_output) {
        if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        driver.ExportPathPovray(out_dir);
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil(output_step_size / step_size);

    // ---------------
    // Simulation loop
    // ---------------

    std::cout<<"data at: "<<vehicle::GetDataFile(steering_controller_file)<<std::endl;
    std::cout<<"data at: "<<vehicle::GetDataFile(speed_controller_file)<<std::endl;
    std::cout<<"data at: "<<vehicle::GetDataFile(path_file)<<std::endl;
    std::cout<<"data at: "<<vehicle::GetDataFile(steering_input_file)<<std::endl;

    int step_number = 0;
    int render_frame = 0;

    double time = 0;

    //output headings for the saved data file
    csv <<"time"<<"Steering Angle"<<"vehicle Speed" ;

    csv <<"Chassis Attitude"<<"Chassis Bank"<<"Chassis Heading";
    csv <<"Chassis Omega X"<<"Chassis Omega Y"<<"Chassis Omega Z";

    csv <<"Lateral Acceleration"<<"W0 long"<<"W0 lat"<<"W0 vert"<<"W1 long"<<"W1 lat"<<"W1 vert";
    csv <<"W2 long"<<"W2 lat"<<"W2 vert"<<"W3 long"<<"W3 lat"<<"W3 vert";
    csv <<"W0 Pos x"<<"W0 Pos Y"<<"W0 Pos Z"<<"W1 Pos x"<<"W1 Pos Y"<<"W1 Pos Z";
    csv <<"W2 Pos x"<<"W2 Pos Y"<<"W2 Pos Z"<<"W3 Pos x"<<"W3 Pos Y"<<"W3 Pos Z"<< std::endl;


#ifdef USE_IRRLICHT
    while (app.GetDevice()->run()) {
        

        //path visualization
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        /*std::cout<<"Target:\t"<<(irr::f32)pT.x()<<",\t "<<(irr::f32)pT.y()<<",\t "<<(irr::f32)pT.z()<<std::endl;
        std::cout<<"Vehicle:\t"<<wvp.GetVehicle().GetChassisBody()->GetPos().x()
          <<",\t "<<wvp.GetVehicle().GetChassisBody()->GetPos().y()<<",\t "
          <<wvp.GetVehicle().GetChassisBody()->GetPos().z()<<std::endl;*/

        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();
        }
#else
    while(wvp.GetSystem()->GetChTime() < tend){
#endif

        time = wvp.GetSystem()->GetChTime();
        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        wvp.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
#ifdef USE_IRRLICHT
        app.Synchronize("Follower driver", steering_input, throttle_input, braking_input);
#endif

        // Advance simulation for one timestep for all modules

        driver.Advance(step_size);
        terrain.Advance(step_size);
        wvp.Advance(step_size);
#ifdef USE_IRRLICHT
        app.Advance(step_size);
#endif

        if (povray_output && step_number % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteShapesPovray(wvp.GetSystem(), filename);
            render_frame++;

        }

        if(data_output && step_number % output_steps == 0){
            //output time to check simulation is running
            std::cout<<time<<std::endl;

            csv <<time<<steering_input<<wvp.GetVehicle().GetVehicleSpeed();
            ChQuaternion<> q= wvp.GetVehicle().GetVehicleRot();
            csv << q.Q_to_NasaAngles();
            csv <<wvp.GetChassisBody()->GetWvel_loc();



            csv << wvp.GetVehicle().GetVehicleAcceleration(wvp.GetVehicle().GetChassis()->GetCOMPos()).y();

            for(int i=0;i<4;i++){
                csv << wvp.GetTire(i)->ReportTireForce(&terrain).force;
            }

            for(int i=0;i<4;i++){
                csv << wvp.GetVehicle().GetWheelPos(i);

            }
            csv << std::endl;




        }







        // std::cout<<time<<std::endl;
        // Increment frame number
        step_number++;
    }

    if (data_output) {
        csv.write_to_file(out_dir + "/" + output_file_name + ".dat");
    }

    return 0;
}
