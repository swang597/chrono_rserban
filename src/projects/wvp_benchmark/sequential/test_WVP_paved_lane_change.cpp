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
#endif

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_models/vehicle/wvp/WVP_FollowerDataDriver.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_models/vehicle/wvp/WVP.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <chrono>
#include <thread>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::wvp;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0,0, 0.75);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::NONE;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

// Type of tire model (RIGID, FIALA, PAC89)
TireModelType tire_model = TireModelType::PAC89;

// Type of steering model (PITMAN_ARM or PITMAN_ARM_SHAFTS)
SteeringTypeWV steering_model = SteeringTypeWV::PITMAN_ARM;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step sizes
double step_size =1e-4;
double tire_step_size = step_size;

// Simulation end time
double tend = 15;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50
double output_step_size = 1e-3;

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
    wvp.SetContactMethod(ChContactMethod::NSC);
    wvp.SetChassisFixed(false);
	wvp.SetInitPosition(ChCoordsys<>(initLoc, initRot));
	wvp.SetTireType(tire_model);
    wvp.setSteeringType(steering_model);
	wvp.SetTireStepSize(tire_step_size);
	wvp.SetInitFwdVel(0.0);
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

    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch_mat->SetFriction(0.8f);
    patch_mat->SetRestitution(0.01f);
    RigidTerrain terrain(wvp.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 1000, 30);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 20, 20);
    terrain.Initialize();


#ifdef USE_IRRLICHT
	// -------------------------------------
	// Create the vehicle Irrlicht interface
	// Create the driver system
	// -------------------------------------

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

    csv << "time (s)";
    csv << "steering wheel angle (deg)";
    csv << "vehicle speed (m/s)";
    
    csv << "roll angle (deg)";
    csv << "roll rate (deg/s)";
    csv << "pitch angle (deg)";
    csv << "pitch rate (deg/s)";
    csv << "yaw angle (deg)";
    csv << "yaw rate (deg/s)";
    csv << "lateral acceleration (g)";
    csv << "longitudinal tire force FL (N)";
    csv << "longitudinal tire force FR (N)";
    csv << "longitudinal tire force RL (N)";
    csv << "longitudinal tire force RR (N)";
    csv << "lateral tire force FL (N)";
    csv << "lateral tire force FR (N)";
    csv << "lateral tire force RL (N)";
    csv << "lateral tire force RR (N)";
    csv << "vertical tire force FL (N)";
    csv << "vertical tire force FR (N)";
    csv << "vertical tire force RL (N)";
    csv << "vertical tire force RR (N)";
    csv << "strut length FL (m)";
    csv << "strut length FR (m)";
    csv << "strut length RL (m)";
    csv << "strut length RR (m)";
    csv << "wheel x position FL (m)";
    csv << "wheel y position FL (m)";
    csv << "wheel x position FR (m)";
    csv << "wheel y position FR (m)";
    csv << "wheel x position RL (m)";
    csv << "wheel y position RL (m)";
    csv << "wheel x position RR (m)";
    csv << "wheel y position RR (m)";
    csv << std::endl;
    //output headings for the saved data file
//    csv <<"time"<<"Steering Angle"<<"vehicle Speed" ;

//    csv <<"Chassis Attitude"<<"Chassis Bank"<<"Chassis Heading";
//    csv <<"Chassis Omega X"<<"Chassis Omega Y"<<"Chassis Omega Z";

//    csv <<"Lateral Acceleration"<<"W0 long"<<"W0 lat"<<"W0 vert"<<"W1 long"<<"W1 lat"<<"W1 vert";
//    csv <<"W2 long"<<"W2 lat"<<"W2 vert"<<"W3 long"<<"W3 lat"<<"W3 vert";
//    csv <<"W0 Pos x"<<"W0 Pos Y"<<"W0 Pos Z"<<"W1 Pos x"<<"W1 Pos Y"<<"W1 Pos Z";
//    csv <<"W2 Pos x"<<"W2 Pos Y"<<"W2 Pos Z"<<"W3 Pos x"<<"W3 Pos Y"<<"W3 Pos Z"<< std::endl;


#ifdef USE_IRRLICHT
    while (vis->Run()) {


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
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }
#else
    while(wvp.GetSystem()->GetChTime() < tend){
#endif

        time = wvp.GetSystem()->GetChTime();

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

        if (povray_output && step_number % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteVisualizationAssets(wvp.GetSystem(), filename);
            render_frame++;

        }

        if(data_output && step_number % output_steps == 0){
            //output time to check simulation is running
            std::cout<<time<<std::endl;


            auto susp0 = std::static_pointer_cast<ChDoubleWishbone>(wvp.GetVehicle().GetSuspension(0));
            auto susp1 = std::static_pointer_cast<ChDoubleWishbone>(wvp.GetVehicle().GetSuspension(1));
            double def0 = susp0->GetSpringLength(LEFT)-.779;
            double def1 = susp0->GetSpringLength(RIGHT)-.779;
            double def2 = susp1->GetSpringLength(LEFT)-.831;
            double def3 = susp1->GetSpringLength(RIGHT)-.831;

            if(def0 > .124 || def0 < -.169){
                std::cout<<"Strut 0 BUMP STOP ENGAGED!\n";
            }
            if(def1 > .124 || def1 < -.169){
                std::cout<<"Strut 1 BUMP STOP ENGAGED!\n";
            }
            if(def2 > .124 || def2 < -.169){
                std::cout<<"Strut 2 BUMP STOP ENGAGED!\n";
            }
            if(def3 > .124 || def3 < -.169){
                std::cout<<"Strut 3 BUMP STOP ENGAGED!\n";
            }

            csv << time;    //time
            csv << driver_inputs.m_steering/-.001282; //steering wheel angle in degrees
            csv << wvp.GetVehicle().GetSpeed();  //vehicle speed m/s
            ChQuaternion<> q= wvp.GetVehicle().GetRot();
            csv << q.Q_to_Euler123().x()*180.0/CH_C_PI;   //roll angle in deg
            csv << wvp.GetChassisBody()->GetWvel_loc().x()*180.0/CH_C_PI; //roll rate deg
            csv << q.Q_to_Euler123().y()*180.0/CH_C_PI;   //pitch angle deg
            csv << wvp.GetChassisBody()->GetWvel_loc().y()*180.0/CH_C_PI; //pitch rate deg
            csv << q.Q_to_Euler123().z()*180.0/CH_C_PI;   //yaw angle deg
            csv << wvp.GetChassisBody()->GetWvel_loc().z()*180.0/CH_C_PI; //yaw rate deg
            csv << wvp.GetVehicle().GetPointAcceleration({-2.070, .01, .495}).y() / 9.81;  // lateral acc. in g's

            std::vector<ChVector<>> tire_forces;
            for (auto& axle : wvp.GetVehicle().GetAxles()) {
                for (auto& wheel : axle->GetWheels()) {
                    tire_forces.push_back(wheel->GetTire()->ReportTireForce(&terrain).force);
                }
            }

            for (int i = 0; i < 4; i++) {
                csv << tire_forces[i].x();  // longitudinal tire forces FL, FR, RL, RR
            }
            for (int i = 0; i < 4; i++) {
                csv << tire_forces[i].y();  // lateral tire forces FL, FR, RL, RR
            }
            for (int i = 0; i < 4; i++) {
                csv << tire_forces[i].z();  // vertical tire forces FL, FR, RL, RR
            }

            csv << susp0->GetSpringLength(LEFT);//individual strut lengths m
            csv << susp0->GetSpringLength(RIGHT);//individual strut lengths m
            csv << susp1->GetSpringLength(LEFT);//individual strut lengths m
            csv << susp1->GetSpringLength(RIGHT);//individual strut lengths m

            for (auto& axle : wvp.GetVehicle().GetAxles()) {
                for (auto& wheel : axle->GetWheels()) {
                    csv << wheel->GetPos().x();  // individual wheel paths x m
                    csv << wheel->GetPos().y();  // individual wheel paths y m
                }
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
