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
// Viper SCM test - save terrain forces to disk
//
// =============================================================================

#include <limits>
#include <iomanip>

#include "test_SCM_force.h"
// #include "chrono_vehicle/terrain/SCMTerrain.h" //Shu added

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Enable/disable bulldozing effects
bool enable_bulldozing = false;

// Enable/disable moving patch feature
bool enable_moving_patch = true;

// -----------------------------------------------------------------------------            
void SaveHeightmap(chrono::vehicle::ChTerrain* terrain, int iwheel, double pos_x, double pos_y, double heightmap_grid, 
    double heightmap_cutoff_x_backward, double heightmap_cutoff_x_forward, double heightmap_cutoff_y_left, double heightmap_cutoff_y_right,
    std::string out_dir, int istep_pos, int istep_cur, double dt){
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
        std::string HMfilename = out_dir + "/hmap_wheel"+std::to_string(iwheel) + "_Pat" + std::to_string(istep_pos * dt) + "_Tat" + std::to_string(istep_cur * dt) + ".txt";
        std::ofstream file_HM(HMfilename, std::ios::trunc);

        // std::ofstream file(heightmapName.str());
        if (file_HM.is_open()) {
            file_HM << hmap_matrix << std::endl;
            file_HM.close();
        } else {
            std::cerr << "Unable to open file: " << HMfilename << std::endl;
        }

}
// // -----------------------------------------------------------------------------            
// void SaveHeightmap(chrono::vehicle::ChTerrain* terrain, int iwheel, ChVector<>& origin, ChQuaternion<>& alignment, double heightmap_grid, 
//     int nx_half, int ny_half, std::string out_dir, int istep_pos, int istep_cur, double dt){
//         // double heightmap_grid_half = 0.5*heightmap_grid;
//         int nx = static_cast<int>(round(nx_half*2));
//         int ny = static_cast<int>(round(ny_half*2));

//         // Create a 2D Eigen matrix for the heightmap
//         Eigen::MatrixXd hmap_matrix(nx, ny);
//         ChVector<> pos_abs, pos_loc;
//         // for(int ix = -nx_half; ix < nx_half; ix++){
//             // for(int iy = -ny_half; iy < ny_half; iy++){
//         for(int ix = 0; ix < nx; ix++){
//             for(int iy = 0; iy < ny; iy++){
//                 pos_loc = ChVector<>((ix-nx_half+0.5)*heightmap_grid, (iy-ny_half+0.5)*heightmap_grid, 0);
//                 pos_abs = ChTransform<>::TransformLocalToParent(pos_loc, origin, alignment);
//                 hmap_matrix(ix, iy) = terrain->GetHeight(ChVector<>(pos_abs[0], pos_abs[1], 10));
//             }

//         }

//         // Save the matrix to a file (assuming Eigen's IO)
//         std::string HMfilename = out_dir + "/hmap_wheel"+std::to_string(iwheel) + "_Pat" + std::to_string(istep_pos * dt) + "_Tat" + std::to_string(istep_cur * dt) + ".txt";
//         std::ofstream file_HM(HMfilename, std::ios::trunc);

//         // std::ofstream file(heightmapName.str());
//         if (file_HM.is_open()) {
//             file_HM << hmap_matrix << std::endl;
//             file_HM.close();
//         } else {
//             std::cerr << "Unable to open file: " << HMfilename << std::endl;
//         }

// }
// -----------------------------------------------------------------------------            
ChVector<> rotatePointInXYPlane(const ChVector<>& point, double angle) {
    ChVector<> point_rot;
    point_rot[0] = point[0] * std::cos(angle) - point[1] * std::sin(angle);
    point_rot[1] = point[0] * std::sin(angle) + point[1] * std::cos(angle);
    return point_rot;
}

void SaveHeightmap(chrono::vehicle::ChTerrain* terrain, int iwheel, ChVector<>& origin, ChQuaternion<>& alignment, double heightmap_grid, 
    int nx_half, int ny_half, std::string out_dir, int istep_pos, int istep_cur, double dt){
        // double heightmap_grid_half = 0.5*heightmap_grid;
        int nx = static_cast<int>(round(nx_half*2));
        int ny = static_cast<int>(round(ny_half*2));
        // getRotationAxis
        double rot_y;
        double angle = 0.0;
        // angle = std::acos(alignment[2]/(std::sqrt(alignment[1]*alignment[1] + alignment[2]*alignment[2])));
        double length = std::sqrt(alignment[1]*alignment[1] + alignment[2]*alignment[2]); // sqrt(x*x+y*y)
        if(length < 1e-8){
            rot_y = 0.0;
            angle = 0.0;
        }else{
            // explain: cos(angle) = V1*V2/(|V1|*|V2|) = V1*V2 = V1x*V2x + V1y*V2y + V1z*V2z = 0 + V1y*V2y + 0 = V1y*1
            rot_y = abs(alignment[2]/length);
            angle = std::acos(rot_y);
        }
        if(iwheel == 0){
            std::cout << "wheel0: rot_y: " << rot_y << ", angle: " << angle <<", degree" << (angle * 180 /3.14)<< std::endl;
        }
        // Create a 2D Eigen matrix for the heightmap
        Eigen::MatrixXd hmap_matrix(nx, ny);
        ChVector<> pos_abs, pos_grid;
        for(int ix = 0; ix < nx; ix++){
            for(int iy = 0; iy < ny; iy++){
                pos_grid = ChVector<>((ix-nx_half+0.5)*heightmap_grid, (iy-ny_half+0.5)*heightmap_grid, 0);
                pos_abs = rotatePointInXYPlane(pos_grid, angle) + origin;
                hmap_matrix(ix, iy) = terrain->GetHeight(ChVector<>(pos_abs[0], pos_abs[1], 10));

                // if(ix==0 && iy == 0){
                //     std::cout << "(0,0):pos_grid: " << pos_grid << ",pos_abs:" << pos_abs<< std::endl;
                // }else if (ix == 0 && iy == ny - 1)
                // {
                //     std::cout << "(0,ny-1):pos_grid: " << pos_grid << ",pos_abs:" << pos_abs<< std::endl;
                // }
                
            }
        }

        // Save the matrix to a file (assuming Eigen's IO)
        std::string HMfilename = out_dir + "/hmap_wheel"+std::to_string(iwheel) + "_Pat" + std::to_string(istep_pos * dt) + "_Tat" + std::to_string(istep_cur * dt) + ".txt";
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
void SaveHeightmap(chrono::vehicle::ChTerrain* terrain, int iwheel, ChVector<>& origin, ChQuaternion<>& alignment, double heightmap_grid, 
    double heightmap_cutoff_x_backward, double heightmap_cutoff_x_forward, double heightmap_cutoff_y_left, double heightmap_cutoff_y_right,
    std::string out_dir, int istep_pos, int istep_cur, double dt){
        int nx = static_cast<int>(round((heightmap_cutoff_x_backward + heightmap_cutoff_x_forward) / heightmap_grid));
        int ny = static_cast<int>(round((heightmap_cutoff_y_left + heightmap_cutoff_y_right) / heightmap_grid));
        int nx_half = static_cast<int>(round(nx *0.5));
        int ny_half = static_cast<int>(round(ny *0.5));
        double heightmap_grid_half = 0.5*heightmap_grid;
        // Create a 2D Eigen matrix for the heightmap
        Eigen::MatrixXd hmap_matrix(nx, ny);
        ChVector<> pos_abs, pos_loc;
        for(int ix = -nx_half; ix < nx_half; ix++){
            for(int iy = -ny_half; iy < ny_half; iy++){
                pos_loc = ChVector<>(ix*heightmap_grid + heightmap_grid_half, iy*heightmap_grid + heightmap_grid_half, 0);
                pos_abs = ChTransform<>::TransformLocalToParent(pos_loc, origin, alignment);
                hmap_matrix(ix, iy) = terrain->GetHeight(ChVector<>(pos_abs[0], pos_abs[1], 10));
            }

        }

        // Save the matrix to a file (assuming Eigen's IO)
        std::string HMfilename = out_dir + "/hmap_wheel"+std::to_string(iwheel) + "_Pat" + std::to_string(istep_pos * dt) + "_Tat" + std::to_string(istep_cur * dt) + ".txt";
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
    
    SetChronoDataPath("/home/swang597/Documents/Research/chrono_fork_radu/build/data/");
    // SetDataPath("/home/swang597/Documents/Research/chrono_fork_radu/build/data/vehicle/");
    
    
    if (argc != 8) {
        std::cout << "Wrong argv.\nUsage: " << argv[0] 
        << " <dt> <terrain_grid> <time_tot> <dt_HM> <viper_initX> <viper_initH> <flag_save_locHM>" << std::endl;
        return 1;
    }
    double dt = std::stod(argv[1]); // 1e-4;
    double terrain_grid = std::stod(argv[2]); // 0.1;
    // generate initial terrain from bmp file
    double terrain_sizeX = 50, terrain_sizeY = 2;
    double terrain_hMin = 0, terrain_hMax = 0.1; 
    std::string heightmap_file = "/home/swang597/Documents/Research/chrono_fork_radu/build/data/vehicle/terrain/height_maps/terrain_heightmap_smooth_horizontal_terrGrid0.005_wx50_wy1.bmp";
    // std::string heightmap_file = "/home/swang597/Documents/Research/build_chrono/data/vehicle/terrain/height_maps/wave_heightmap_5000-100.bmp";

    double time_tot = std::stod(argv[3]); // 3;
    double dt_HM = std::stod(argv[4]); // 1e-4;
    double terrain_initX = 0; 
    double terrain_initH = 0;
    double viper_initX = std::stod(argv[5]); 
    double viper_initH = std::stod(argv[6]);
    int flag_save_locHM = std::stoi(argv[7]); // 0: global CS, 1: local CS
    out_dir = GetChronoOutputPath() + "SCM_FORCE_TEST_saveLocHM" + std::to_string(flag_save_locHM);// + "_dt" + std::to_string(dt) + "_dtHM" + std::to_string(dt_HM) + "_viperX" + std::to_string(viper_initX) + "_viperH" + std::to_string(viper_initH) + "_terrainGrid" + std::to_string(terrain_grid) + "_timeTot" + std::to_string(time_tot);
    double heightmap_grid = terrain_grid;
    double heightmap_cutoff_y_left = 0.2; // 0.15; // cylinder wheel width 0.128*2
    double heightmap_cutoff_y_right = 0.2; //0.15; 
    double heightmap_cutoff_x_backward = 0.3;// 0.25; //0.21; // cylinder wheel radius 0.208
    double heightmap_cutoff_x_forward = 0.3; //0.25; //0.21;

    int nx = static_cast<int>(round((heightmap_cutoff_x_backward + heightmap_cutoff_x_forward) / heightmap_grid));
    int ny = static_cast<int>(round((heightmap_cutoff_y_left + heightmap_cutoff_y_right) / heightmap_grid));
    int nx_half = static_cast<int>(round(nx *0.5));
    int ny_half = static_cast<int>(round(ny *0.5));
    std::cout << "nx_half: " << nx_half << ", ny_half: " << ny_half << std::endl;

    bool flag_heightmap_save = true;
    bool flag_vis = false;

    int num_steps = int(time_tot/dt);
    int ndt_HM = int(dt_HM/dt);
    std::cout << "num_steps: " << num_steps << ", ndt_HM:" << ndt_HM << std::endl;
    
    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }
    utils::CSV_writer csv(" ");

    // Create the Chrono system (Z up)
    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Create the rover
    // auto viper = CreateViper(sys);
    ChVector<> viper_init_loc(viper_initX, 0, viper_initH);
    double time_ramp_Wy = 0.5, Wy = 10;
    std::shared_ptr<chrono::viper::ViperDriver> driver = chrono_types::make_shared<ViperSpeedDriver>(time_ramp_Wy, Wy);
    auto viper = CreateViper(sys, ViperWheelType::CylWheel, driver, viper_init_loc);

    // Cache wheel bodies
    std::vector<std::shared_ptr<ViperWheel>> viperwheels{
        viper->GetWheel(ViperWheelID::V_LF),  //
        viper->GetWheel(ViperWheelID::V_RF),  //
        viper->GetWheel(ViperWheelID::V_LB),  //
        viper->GetWheel(ViperWheelID::V_RB)   //
    };
    std::vector<std::shared_ptr<ChBodyAuxRef>> wheels{
        viper->GetWheel(ViperWheelID::V_LF)->GetBody(),  //
        viper->GetWheel(ViperWheelID::V_RF)->GetBody(),  //
        viper->GetWheel(ViperWheelID::V_LB)->GetBody(),  //
        viper->GetWheel(ViperWheelID::V_RB)->GetBody()   //
    };

    // Create the SCM deformable terrain
    // auto terrain = CreateTerrain(terrain_grid, enable_bulldozing, sys);
    auto terrain = CreateTerrain(terrain_grid, enable_bulldozing, sys,
        heightmap_file, terrain_sizeX, terrain_sizeY,
         terrain_hMin, terrain_hMax, terrain_initX, terrain_initH);
    // std::shared_ptr<vehicle::SCMTerrain> CreateTerrain(double resolution, bool enable_bulldozing, ChSystem& sys,
    // const std::string& heightmap_file, double sizeX, double sizeY, double hMin, double hMax, 
    // double terrain_initX, double terrain_initH) {

    // Add moving patches for each wheel
    if (enable_moving_patch) {
        double wheel_range = 0.5;
        ChVector<> size(0.5, 2 * wheel_range, 2 * wheel_range);
       
        terrain->AddMovingPatch(wheels[0], VNULL, size);
        terrain->AddMovingPatch(wheels[1], VNULL, size);
        terrain->AddMovingPatch(wheels[2], VNULL, size);
        terrain->AddMovingPatch(wheels[3], VNULL, size);
    }

    // Create the run-time visualization interface
    ChVector<> cam_loc(viper_initX + 2, 2, viper_initH+1);
    auto vis = CreateVisualization(vis_type, false, sys, cam_loc);

    // Open I/O files
    std::ofstream SCM_forces(SCM_filename, std::ios::trunc);
    std::ofstream ROVER_states(out_dir + "/ROVER_states_saved.txt", std::ios::trunc);
    std::ofstream ROVER_states_rot(out_dir + "/wheels_states_rot_saved.txt", std::ios::trunc);
    // Simulation loop
    ChVector<> wheelContFList_F, wheelContFList_M, origin;
    ChQuaternion<> alignment;
    std::vector<ChVector<>> origin_pre(4);
    std::vector<ChQuaternion<>> alignment_pre(4);
    double pos_xy_pre[4][2];
    
    // *********************************************
    // main simulation loop
    // *********************************************
    for (int istep = 0; istep < num_steps; istep++) {
        if (istep % 10 == 0)
            cout << "Time: " << sys.GetChTime() << endl;

#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->SetCameraTarget(viper->GetChassis()->GetPos());
        vis->Render();
        vis->EndScene();
#endif

        // Time at *beginning* of step
        double time = sys.GetChTime();
        if(time > 0.2 && time < 0.2+30*dt){
            driver->SetSteering(1,ViperWheelID::V_LF);
            driver->SetSteering(1,ViperWheelID::V_RF);
        }
           
        // Advance system dynamics
        sys.DoStepDynamics(dt);
        viper->Update();

        // Save SCM terrain forces that were applied during the *previous* step
        SCM_forces << time << "    ";
        for (int i = 0; i < 4; i++) {
            terrain->GetContactForceBody(wheels[i], wheelContFList_F, wheelContFList_M);
            SCM_forces << std::setprecision(20) << wheelContFList_F << "  "
                       << wheelContFList_M << "    ";
        }
        SCM_forces << endl;

        // Save vehicle states
        ROVER_states << time << "   ";
        for (int i = 0; i < 4; i++) {
            ROVER_states << viperwheels[i]->GetPos() << "   " << viperwheels[i]->GetRot() 
                << "   " << viperwheels[i]->GetLinVel() << "   " << viperwheels[i]->GetAngVel() 
                         << "   ";
        }
        ROVER_states << endl;

        // Save heightmap
        if(flag_heightmap_save && istep % ndt_HM == 0){
            for (int i = 0; i < 4; i++) {

                // *** Save in global CS. ***
                if(flag_save_locHM == 0){
                    SaveHeightmap(terrain.get(), i, pos_xy_pre[i][0], pos_xy_pre[i][1], heightmap_grid, 
                        heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left, 
                        heightmap_cutoff_y_right, out_dir, istep - ndt_HM, istep, dt);
                    
                    SaveHeightmap(terrain.get(), i, wheels[i]->GetPos()[0], wheels[i]->GetPos()[1], heightmap_grid, 
                        heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left,
                        heightmap_cutoff_y_right, out_dir, istep, istep, dt);
                    pos_xy_pre[i][0] = wheels[i]->GetPos()[0];
                    pos_xy_pre[i][1] = wheels[i]->GetPos()[1];

                }else{    
                    // *** Save in local CS. *** 
                    // Save current terrain
                    origin = ChVector<>(wheels[i]->GetPos()[0], wheels[i]->GetPos()[1], 0);
                    alignment = viperwheels[i]->GetRot();
                    SaveHeightmap(terrain.get(), i, origin, alignment, heightmap_grid, 
                        nx_half, ny_half, out_dir, istep, istep, dt);                                
                    
                    // Save pre terrain
                    SaveHeightmap(terrain.get(), i,  origin_pre[i], alignment_pre[i], heightmap_grid, 
                        nx_half, ny_half, out_dir, istep- ndt_HM, istep, dt);
                        
                    origin_pre[i]  = origin;
                    alignment_pre[i] = alignment;
                    
                    if(i == 0){
                        std::cout << "alignment" << alignment << std::endl;
                    }
                }
                
            }
            
        }

    }

    SCM_forces.close();
    ROVER_states.close();
    ROVER_states_rot.close();

    return 0;
}
