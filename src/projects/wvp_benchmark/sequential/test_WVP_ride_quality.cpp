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

#define USE_IRRLICHT

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#ifdef USE_IRRLICHT
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#endif
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_models/vehicle/wvp/WVP.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <chrono>
#include <thread>
#include <math.h>

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

// Type of tire model (RIGID, FIALA, PAC89)
TireModelType tire_model = TireModelType::PAC89;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

ChVector<> vehCOM(-1.933, 0.014, 0.495);

// Simulation step sizes
double step_size = 1e-4;
double tire_step_size = step_size;

// Simulation end time
double tend = 1;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50
double output_step_size = 1e-2;

//vehicle driver inputs
// Desired vehicle speed (m/s)
double mph_to_ms = 0.44704;
double target_speed = 5*mph_to_ms;

//output directory
const std::string out_dir = "../WVP_QUALITY";
const std::string pov_dir = out_dir + "/POVRAY";
bool povray_output = false;
bool data_output = true;


std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("wvp/SteeringController.json");
std::string speed_controller_file("wvp/SpeedController.json");

std::string rnd_10("wvp/obj/rnd_10.obj");
std::string rnd_14("wvp/obj/rnd_14.obj");
std::string rnd_24("wvp/obj/rnd_24.obj");
std::string rnd_36("wvp/obj/rnd_36.obj");

std::string output_file_name("quality");

std::string terrainFile = rnd_10;

// =============================================================================

int main(int argc, char* argv[]) {

    //read in argument as simulation duration
    if(argc > 2){
      std::cout<<"ARGS: "<<argv[1]<<"|"<<argv[2]<<"|"<<argv[3]<<std::endl;
      tend = atof(argv[1]);
      std::cout<<tend<<std::endl;
      target_speed = atof(argv[2]);
      int terrainCode = (int)atof(argv[3]);
      switch(terrainCode){
        case 1:
          terrainFile = rnd_10;
          break;
        case 2:
          terrainFile = rnd_14;
          break;
        case 3:
          terrainFile = rnd_24;
          break;
        case 4:
          terrainFile = rnd_36;
          break;
        default:
          std::cout<<"Invalid Terrain Code - (1-4)";
      }
      output_file_name += "_" + std::to_string((int)target_speed);
      output_file_name += "_" + std::to_string((int)terrainCode);
      target_speed = target_speed*mph_to_ms;

    }

    // --------------
    // Create systems
    // --------------

    // Create the vehicle, set parameters, and initialize
    WVP wvp;
    wvp.SetContactMethod(ChContactMethod::NSC);
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

    std::cout << "Vehicle mass:               " << wvp.GetVehicle().GetMass() << std::endl;

    // ------------------
    // Create the terrain
    // ------------------
    double swept_radius = 0.01;

    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    RigidTerrain terrain(wvp.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile(terrainFile), swept_radius);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 12, 12);
    terrain.Initialize();


    //create the driver
    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(wvp.GetVehicle(), vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed);
    driver.Initialize();

    // -------------------------------------
    // Create the vehicle Irrlicht interface
    // Create the driver system
    // -------------------------------------

#ifdef USE_IRRLICHT
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("WVP sequential test");
    vis->SetChaseCamera(trackPoint, 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&wvp.GetVehicle());

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

#endif

    // -------------
    // Prepare output
    // -------------

    if (data_output || povray_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
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



    csv << "time";
    csv << "throttle";
    csv << "MotorSpeed";
    csv << "CurrentTransmissionGear";
    for(int i=0;i<4;i++){
        csv << "WheelTorque";
    }
    for(int i=0;i<4;i++){
        csv << "WheelAngVelX";
        csv << "WheelAngVelY";
        csv << "WheelAngVelZ";
    }
    csv << "VehicleSpeed";
    csv << "VehicleDriverAccelerationX";
    csv << "VehicleDriverAccelerationY";
    csv << "VehicleDriverAccelerationZ";

    csv << "VehicleCOMAccelerationX";
    csv << "VehicleCOMAccelerationY";
    csv << "VehicleCOMAccelerationZ";

    for(int i=0;i<4;i++){
        csv << "TireForce";
    }
    csv << "EngineTorque";

    csv <<"W0 Pos x"<<"W0 Pos Y"<<"W0 Pos Z"<<"W1 Pos x"<<"W1 Pos Y"<<"W1 Pos Z";
    csv << "W2 Pos x"
        << "W2 Pos Y"
        << "W2 Pos Z"
        << "W3 Pos x"
        << "W3 Pos Y"
        << "W3 Pos Z";

    for (auto& axle : wvp.GetVehicle().GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            csv << wheel->GetPos();
        }
    }

    csv << std::endl;

    // ---------------
    // Simulation loop
    // ---------------

    std::cout<<"data at: "<<vehicle::GetDataFile(steering_controller_file)<<std::endl;
    std::cout<<"data at: "<<vehicle::GetDataFile(speed_controller_file)<<std::endl;
    std::cout<<"data at: "<<vehicle::GetDataFile(path_file)<<std::endl;

    int step_number = 0;
    int render_frame = 0;

    double time = 0;

#ifdef USE_IRRLICHT
    while (vis->Run()) {
#else
    while(time < tend){
#endif
        time = wvp.GetSystem()->GetChTime();

#ifdef USE_IRRLICHT
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
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

#endif

        if (povray_output && step_number % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteVisualizationAssets(wvp.GetSystem(), filename);
            render_frame++;
        }

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        wvp.Synchronize(time, driver_inputs, terrain);

#ifdef USE_IRRLICHT
        vis->Synchronize(time, driver_inputs);
#endif

        // Advance simulation for one timestep for all modules

        driver.Advance(step_size);
        terrain.Advance(step_size);
        wvp.Advance(step_size);

#ifdef USE_IRRLICHT
        vis->Advance(step_size);
#endif

        if (data_output && step_number % output_steps == 0) {
            std::cout << time << std::endl;
            csv << time;
            csv << driver_inputs.m_throttle;
            csv << wvp.GetVehicle().GetEngine()->GetMotorSpeed();
            csv << wvp.GetVehicle().GetTransmission()->GetCurrentGear();
            for (int axle = 0; axle < 2; axle++) {
                csv << wvp.GetVehicle().GetDriveline()->GetSpindleTorque(axle, LEFT);
                csv << wvp.GetVehicle().GetDriveline()->GetSpindleTorque(axle, RIGHT);
            }
            for (int axle = 0; axle < 2; axle++) {
                csv << wvp.GetVehicle().GetSpindleAngVel(axle, LEFT);
                csv << wvp.GetVehicle().GetSpindleAngVel(axle, RIGHT);
            }
            csv << wvp.GetVehicle().GetSpeed();
            csv << wvp.GetVehicle().GetPointAcceleration(wvp.GetVehicle().GetChassis()->GetLocalDriverCoordsys().pos);

            csv << wvp.GetVehicle().GetPointAcceleration(vehCOM);

            for (auto& axle : wvp.GetVehicle().GetAxles()) {
                for (auto& wheel : axle->GetWheels()) {
                    csv << wheel->GetTire()->ReportTireForce(&terrain).force;
                }
            }

            csv << wvp.GetVehicle().GetEngine()->GetOutputMotorshaftTorque();

            csv << std::endl;
        }

        // Increment frame number
        step_number++;
    }

    if (data_output) {
        csv.write_to_file(out_dir + "/" + output_file_name + ".dat");
    }

    return 0;
}
