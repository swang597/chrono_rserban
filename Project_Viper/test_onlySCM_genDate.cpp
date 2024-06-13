// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Shu Wang, Radu Serban
// =============================================================================
//
// Viper SCM test - apply forces from disk
//
// =============================================================================

#include "test_SCM_force.h"
#include "TorchModelRunner.hpp"
#include "Heightmap.hpp"
#include <filesystem>
#include <c10/core/Device.h>
#include <c10/core/DeviceType.h>
#include <torch/torch.h>
#include <iostream>
#include <map>
#include "cnpy.h"

//Shu
#include <algorithm>
#include <torch/torch.h>
#include <chrono> // for time profiling

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SaveHeightmap(ChTerrain* terrain, double pos_x, double pos_y, double heightmap_grid, 
    double heightmap_cutoff_x_backward, double heightmap_cutoff_x_forward, double heightmap_cutoff_y_left, double heightmap_cutoff_y_right,
    std::string HMfilename, int istep_pos, int istep_cur, double dt, int nx = 96, int ny = 72){
        // std::cout << "Save heightmap Pat time: " << istep_pos * dt << ", Tat " << istep_cur * dt << std::endl;
        std::vector<double> xVec, yVec;
        // Populate xVec and yVec with values
        for (double x = pos_x - heightmap_cutoff_x_backward; 
                x < pos_x + heightmap_cutoff_x_forward; 
                x += heightmap_grid) {
            xVec.push_back(x);
        }

        for (double y = pos_y - heightmap_cutoff_y_left; 
                y < pos_y + heightmap_cutoff_y_right; 
                y += heightmap_grid) {
            yVec.push_back(y);
        }

        while(xVec.size() > nx){
            xVec.pop_back();
            // std::cout << "xVec.size()=" << xVec.size() << std::endl;
        }
        while(yVec.size() > ny){
            yVec.pop_back();
            std::cout << "yVec.size()=" << yVec.size() << std::endl;
        }   

        // Create a 2D Eigen matrix for the heightmap
        Eigen::MatrixXd hmap_matrix(xVec.size(), yVec.size());

        // Populate the matrix with heights
        for (size_t ix = 0; ix < xVec.size(); ++ix) {
            for (size_t iy = 0; iy < yVec.size(); ++iy) {
                // Assuming GetHeight takes a vector and returns the height.
                hmap_matrix(ix, iy) = terrain->GetHeight(ChVector<>(xVec[ix], yVec[iy], 10));
            }
        }

        // Save the matrix to a file (assuming Eigen's IO)
        std::ofstream file_HM(HMfilename, std::ios::trunc);

        // std::ofstream file(heightmapName.str());
        if (file_HM.is_open()) {
            file_HM << hmap_matrix << std::endl;
            file_HM.close();
        } else {
            std::cerr << "Unable to open file: " << HMfilename << std::endl;
        }
}
// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    SetChronoDataPath("/home/swang597/Documents/Research/chrono_fork_rserban/build/data/");
    SetDataPath("/home/swang597/Documents/Research/chrono_fork_rserban/build/data/vehicle/");
    
    if (argc < 10) {
        std::cout << "Wrong argv.\nUsage: " << argv[0] 
        << " <dt> <terrain_grid> <time_tot> <dt_dump> <terrain_initX> <terrain_initH> <chassis_density>" << std::endl;
        return 1;
    }
    double dt = std::stod(argv[1]); // 1e-4;
    double terrain_grid = std::stod(argv[2]); // 0.1;
    double time_tot = std::stod(argv[3]); // 3;
    double dt_dump = std::stod(argv[4]); // 1e-4;
    double terrain_initX = std::stod(argv[5]); 
    double torque_wheel = std::stod(argv[6]);
    double chassis_density = std::stod(argv[7]); // 1000;
    double SCM_ML_switch = std::stod(argv[8]);
    double ang_vel_wheel = std::stod(argv[9]);
    double terrain_initY = 0.0; //0.3; // Default value by ChTireTestRig.cpp
    double terrain_initH = -1.0; 
    // generate initial terrain from bmp file
    double terrain_sizeX = 50, terrain_sizeY = 1;
    double terrain_hMin = 0, terrain_hMax = 0.05; //0.1; 
    std::string heightmap_file = "/home/swang597/Documents/Research/chrono_fork_radu/build/data/vehicle/terrain/height_maps/terrain_heightmap_smooth_horizontal_terrGrid0.005_wx50_wy1.bmp";
    double heightmap_grid = terrain_grid;
    double heightmap_cutoff_y_left = 0.18; //0.15; // cylinder wheel width 0.128*2
    double heightmap_cutoff_y_right = 0.18; //0.15; 
    double heightmap_cutoff_x_backward = 0.21; //0.24; // cylinder wheel radius 0.208
    double heightmap_cutoff_x_forward = 0.27; //0.18;

    float wheel_radius = 0.208, wheel_width = 0.256;

    // bool flag_heightmap_save = false; //true;
    bool flag_vis = true; //false; //
    bool flag_save_vedio = true; //false;

    // double time_delay = sqrt(2 * std::abs(terrain_initH) / 9.81) + 0.5; //sqrt(2*abs(H)/9.81) + 0.5
    // time_tot += time_delay;
    int num_steps = int(time_tot/dt);
    int save_vedio_fps = int(num_steps/100.0);
    int idx_vedio = 0;

    // Output directory MUST exist
    // if (!filesystem::path(out_dir).exists() || !filesystem::path(out_dir).is_directory()){
    //     std::filesystem::create_directory(out_dir);
    // }
    // if (!filesystem::path(out_dir).exists() || !filesystem::path(out_dir).is_directory()) {
    //     cout << "Data directory does NOT exist!" << endl;
    //     return 1;
    // }
    // utils::CSV_writer csv(" ");

    // Create the Chrono system (Z up)
    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Create the rover
    // auto viper = CreateViper(sys);
    ViperWheelType wheel_type = ViperWheelType::CylWheel;
    // auto driver = chrono_types::make_shared<ViperDCMotorControl>();

    // Default driver fixed Wy
    double time_ramp_Wy = 2, Wy = 10 * CH_C_RPM_TO_RPS;
    auto driver = chrono_types::make_shared<ViperSpeedDriver>(time_ramp_Wy, Wy);
    
    // double torque_wheel = 100;
    // auto driver = chrono_types::make_shared<ViperDCMotorControl>(torque_wheel, ang_vel_wheel);

    ChVector<> viper_init_loc=ChVector<>(-5, 0, -0.5); //(10, 2, 0.25)
    
    // auto viper = CreateViper(sys, wheel_type, driver,viper_init_loc);

    // double chassis_density = 100;
    auto viper = CreateViper(sys, wheel_type, driver, chassis_density, viper_init_loc);

    // Cache wheel bodies
    std::vector<std::shared_ptr<ChBodyAuxRef>> wheels{
        viper->GetWheel(ViperWheelID::V_LF)->GetBody(),  //
        viper->GetWheel(ViperWheelID::V_RF)->GetBody(),  //
        viper->GetWheel(ViperWheelID::V_LB)->GetBody(),  //
        viper->GetWheel(ViperWheelID::V_RB)->GetBody()   //
    };

    float wx=50, wy=4.0, delta=heightmap_grid; //0.005;
    
    // Create the SCM deformable terrain
    bool enable_bulldozing = false; // Enable/disable bulldozing effects
    bool enable_moving_patch = true; // Enable/disable moving patch feature
    // Create the flat terrain ----------------------------------------------
    auto terrain_SCM = CreateTerrain(delta, enable_bulldozing, sys);
    // Heightmap HM = Heightmap::init_flat(wx, wy, delta);
    // std::string output_folderpath = out_dir + "_HMflat_SCM2MLSwitch" +
    //     std::to_string(SCM_ML_switch) + "_dt" + std::to_string(dt) + 
    //     "_torque" + std::to_string(torque_wheel) + "_density" + std::to_string(chassis_density) + 
    //     "_wy" + std::to_string(ang_vel_wheel) + "_refreshNR_240516/";
    
    std::string output_folderpath = out_dir + "_HMflat_SCM2MLSwitch" +
        std::to_string(SCM_ML_switch) + "_dt" + std::to_string(dt) + 
        "_density" + std::to_string(chassis_density) + 
        "_fixWy_refreshNR_240517/";
    
    // Create the bumpy terrain ----------------------------------------------
    // auto terrain_SCM = CreateTerrain(delta, enable_bulldozing, sys, 
    //         heightmap_file, wx, wy, terrain_hMin, terrain_hMax, terrain_initX, terrain_initH);    
    // // Heightmap HM = Heightmap::init_bmp(heightmap_file, wx, wy, terrain_hMin, terrain_hMax, delta);
    // std::string output_folderpath = out_dir + "_HMbmp_terrain_hMax"+std::to_string(terrain_hMax)+"_SCM2MLSwitch" + std::to_string(SCM_ML_switch) + "_dt" + std::to_string(dt) + "_240331/";
    
    std::cout << "output_folderpath=" << output_folderpath << std::endl;
    
    terrain_SCM->SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.20); // set sinkage plot range
    terrain_SCM->SetMLSwitchTime(SCM_ML_switch);
    // auto terrain_SCM = CreateTerrain(0.1, enable_bulldozing, sys);
    // Add moving patches for each wheel
    if (enable_moving_patch) {
        double wheel_range = 0.5;
        ChVector<> size(0.5, 2 * wheel_range, 2 * wheel_range);
       
        terrain_SCM->AddMovingPatch(wheels[0], VNULL, size);
        terrain_SCM->AddMovingPatch(wheels[1], VNULL, size);
        terrain_SCM->AddMovingPatch(wheels[2], VNULL, size);
        terrain_SCM->AddMovingPatch(wheels[3], VNULL, size);
    }
    
    //make output folder
    if(!std::filesystem::exists(output_folderpath))
        std::filesystem::create_directory(output_folderpath);

    std::string folderpath_normlized = "/home/swang597/Documents/Research/chrono_fork_radu/project_TireTestRig/build_SCM_argVx/DEMO_OUTPUT/Dataset_train_96by72_2phase_varVx_I00V0_I01F0_240330/";
    
    std::string model_path = "/home/swang597/Documents/Research/Project_heightmap/Code/Pytorch_cpp_model/Model/Model_2stepNN_iDT1_img96by72_varKsize3333_varVx_I00V0_I01F0_240330/";
    std::string model_path_F;
    model_path_F =   model_path + "modelF_cpu.pt";
    TorchModelRunner model_runner_F(model_path_F);
    std::cout << "Load NN model done." << std::endl;

    // Create the run-time visualization interface
    // auto vis = CreateVisualization(vis_type, true, sys); //Add grid
    auto vis = CreateVisualization(vis_type, false, sys);

    // Open I/O files
    std::ofstream SCM_forces(output_folderpath + "SCM_force_saved.txt", std::ios::trunc);
    std::ofstream ROVER_states(output_folderpath + "ROVER_states_applied.txt", std::ios::trunc);
    std::string HMfilename_SCM;
    // Simulation loop
    ChVector<> wheelContFList_F;
    ChVector<> wheelContFList_M;

    double time, remainder;
    for (int istep = 0; istep < num_steps; istep++) {
        if (istep % 100 == 0)
            cout << "Time: " << sys.GetChTime() << endl;
        
        time = sys.GetChTime();
        remainder = std::fmod(time, dt_dump);
        if (remainder < 1e-6 || (dt_dump - remainder) < 1e-6) {
            // save heightmap 
            for(int iwheel = 0; iwheel < 4; iwheel++) {
                ChVector<> wheel_pos = wheels[iwheel]->GetPos();
                HMfilename_SCM = output_folderpath + "hmap_SCM_wheel" + std::to_string(iwheel) + "_Tat" + std::to_string(time) + ".txt";
                SaveHeightmap(terrain_SCM.get(), wheel_pos[0], wheel_pos[1], heightmap_grid, 
                heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left,
                heightmap_cutoff_y_right, HMfilename_SCM, istep, istep, dt);
                // std::cout << "Save heightmap:" << HMfilename_SCM << std::endl;
                // HM_SCM = GetSCMHM(terrain_SCM.get(), wheel_pos[0], wheel_pos[1], heightmap_grid, 
                //          heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left,heightmap_cutoff_y_right);
                // inI_1chan = torch::from_blob(HM_SCM.data(), {1, 1, HM_SCM.rows(), HM_SCM.cols()}, torch::kFloat32);
                // terrain_ML->UpdateHM(wheel_pos, inI_1chan);   
            }
        }

        if(flag_vis){
            #if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
                vis->BeginScene();
                vis->SetCameraTarget(viper->GetChassis()->GetPos());
                vis->Render();
                vis->EndScene();
            #endif
        }

        // std::cout << "line498" << std::endl;
        // Advance system dynamics
        sys.DoStepDynamics(dt);
        // std::cout << "line501" << std::endl;
        viper->Update();
        // std::cout << "line502" << std::endl;
        // double time = sys.GetChTime();
        // std::cout << "line504" << std::endl;
        // Save SCM terrain forces that were applied during the *previous* step
        // if ((istep+1) % 20 == 0){
        // double remainder = std::fmod(time, dt_dump);
        if (remainder < 1e-6 || (dt_dump - remainder) < 1e-6) {
            SCM_forces << time << "    ";
            for (int i = 0; i < 4; i++) {
                terrain_SCM->GetContactForceBody(wheels[i], wheelContFList_F, wheelContFList_M);
                SCM_forces << std::setprecision(10) << wheelContFList_F << "  "
                        << wheelContFList_M << "    ";
            }
            SCM_forces << endl;

        // Save vehicle states
        
            ROVER_states << time << "   ";
            for (int i = 0; i < 4; i++) {
                ROVER_states << wheels[i]->GetPos() << "   " << wheels[i]->GetPos_dt() << "   " << wheels[i]->GetWvel_par()
                            << "   ";
            }
            ROVER_states << endl;
        }

        if(flag_vis && flag_save_vedio && istep % save_vedio_fps == 0){
            std::string imgName = output_folderpath + "img_" + std::to_string(idx_vedio) + ".jpg";
            vis->WriteImageToFile(imgName);
            idx_vedio++;
        }
    }
    // std::cout << "line513" << std::endl;

    SCM_forces.close();
    ROVER_states.close();
    // std::cout << "line517" << std::endl;
    return 0;
}
