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
// Function to write output
bool write_output(const std::string& filename, const torch::Tensor& matrix, bool append = false) {
    if (append == false && std::filesystem::exists(filename)) {
        std::filesystem::remove(filename);
    }
    std::ofstream m_fp_I(filename, std::ios::app);
    if (!m_fp_I) {
        std::cerr << "Failed to open the file for writing." << std::endl;
        return false;
    }

    for (int i = 0; i < matrix.size(0); ++i) {
        for (int j = 0; j < matrix.size(1); ++j) {
            m_fp_I << matrix[i][j].item<float>() << " ";
        }
        m_fp_I << std::endl;
    }
    
    return true;
}


// -----------------------------------------------------------------------------
// Function to load data from text file and return as torch::Tensor
torch::Tensor loadFromTxt(const std::string& filePath) {
    std::ifstream inFile(filePath);
    if (inFile.fail()) {
        throw std::runtime_error("Failed to open file: " + filePath);
    }

    std::string line;
    std::vector<std::vector<float>> matrixData;

    while (std::getline(inFile, line)) {
        std::stringstream ss(line);
        std::vector<float> rowData;
        float value;
        while (ss >> value) {
            rowData.push_back(value);
        }
        matrixData.push_back(rowData);
    }

    // Deduce tensor dimensions
    int64_t rows = static_cast<int64_t>(matrixData.size());
    int64_t cols = (rows > 0) ? static_cast<int64_t>(matrixData[0].size()) : 0;

    // Ensure that all rows have the same number of columns
    for (const auto& row : matrixData) {
        if (static_cast<int64_t>(row.size()) != cols) {
            throw std::runtime_error("Inconsistent number of columns in the file.");
        }
    }

    // Create and populate the tensor
    torch::Tensor tensor = torch::empty({rows, cols}, torch::kFloat32);

    for (int64_t i = 0; i < rows; ++i) {
        for (int64_t j = 0; j < cols; ++j) {
            tensor[i][j] = matrixData[i][j];
        }
    }

    return tensor;
}

// Shu-----------------------------------------------------------------------------
// void applyWheelSinkage(torch::Tensor& heightMap, torch::Tensor wheelCenter, 
//     int cx_idx, int cy_idx, float radius, float width, float gridSize) {
//     auto nx = heightMap.size(2);
//     auto ny = heightMap.size(3);
//     auto cx = cx_idx * gridSize;
//     auto cy = cy_idx * gridSize;
//     auto cz = wheelCenter[2].item<float>();

//     // Create meshgrid for x and y coordinates
//     auto x = torch::arange(0, nx, 1, torch::kFloat32) * gridSize;
//     auto y = torch::arange(0, ny, 1, torch::kFloat32) * gridSize;
//     auto xv = x.unsqueeze(1).expand({nx, ny});
//     auto yv = y.unsqueeze(0).expand({nx, ny});

//     // Calculate distances from the wheel center
//     auto distance_x = torch::abs(xv - cx);
//     auto distance_y = torch::abs(yv - cy);

//     // Mask for points within the wheel's width
//     auto within_width = distance_y < (width / 2);

//     // Mask for points within the wheel's radius
//     auto within_radius = distance_x < radius;

//     // Calculate wheel surface height
//     auto wheel_surface_height = cz - torch::sqrt(radius * radius - distance_x.pow(2));

//     // Mask for points where the wheel surface is below the current height
//     auto below_current_height = wheel_surface_height < heightMap.index({0, 0});

//     // Combine masks
//     auto mask = within_width & within_radius & below_current_height;

//     // Update heights
//     auto updated_heights = torch::where(mask, wheel_surface_height, heightMap.index({0, 0}));

//     // Apply updated heights to heightMap
//     heightMap.index_put_({0, 0}, updated_heights);
// }
// Shu-----------------------------------------------------------------------------
torch::Tensor applyWheelSinkage(torch::Tensor& heightMap, torch::Tensor wheelCenter, 
    int cx_idx, int cy_idx, float radius, float width, float gridSize, double threshold = 1e-3) {
    auto nx = heightMap.size(2);
    auto ny = heightMap.size(3);
    auto cx = cx_idx * gridSize;
    auto cy = cy_idx * gridSize;
    auto cz = wheelCenter[2].item<float>();

    // Create meshgrid for x and y coordinates
    auto x = torch::arange(0, nx, 1, torch::kFloat32) * gridSize;
    auto y = torch::arange(0, ny, 1, torch::kFloat32) * gridSize;
    auto xv = x.unsqueeze(1).expand({nx, ny});
    auto yv = y.unsqueeze(0).expand({nx, ny});

    // Calculate distances from the wheel center
    auto distance_x = torch::abs(xv - cx);
    auto distance_y = torch::abs(yv - cy);

    // Mask for points within the wheel's width
    auto within_width = distance_y < (width / 2);

    // Mask for points within the wheel's radius
    auto within_radius = distance_x < radius;
    // std::cout << "within_width.sum()"<< within_width.sum().item()<< "within_radius.sum()=" << within_radius.sum().item<float>() << std::endl;
    
    // Calculate wheel surface height
    auto wheel_surface_height = cz - torch::sqrt(radius * radius - distance_x.pow(2));

    // Mask for points where the wheel surface is below the current height
    auto below_current_height = (wheel_surface_height - heightMap.index({0, 0})) < threshold;
    // std::cout << "below_current_height.sum()=" << below_current_height.sum().item<float>() << "below_current_height.sizes()"<< below_current_height.sizes()<< std::endl;
    // Combine masks
    auto mask = within_width & within_radius & below_current_height;

    if(mask.sum().item<float>() == 0){
        std::cout << "mask.sum()=0" << std::endl;    
    }
    
    // Update heights
    auto updated_heights = torch::where(mask, wheel_surface_height, heightMap.index({0, 0}));
    
    // Apply updated heights to heightMap
    heightMap.index_put_({0, 0}, updated_heights);

    return mask;
}
// -----------------------------------------------------------------------------
// Load container to apply wheel forces and torques from data file
class TerrainForceLoaderML : public ChLoadContainer {
  public:
    TerrainForceLoaderML(const std::string& input_folderpath, std::string& output_folderpath, 
        std::vector<std::shared_ptr<ChBodyAuxRef>> wheels, 
        TorchModelRunner model_runner_wheel1, TorchModelRunner model_runner_wheel2,
        Heightmap GHM_0, Heightmap GHM_1, Heightmap GHM_2, 
        ChVector<> terrain_initLoc, float wheel_radius, float wheel_width, float heightmap_grid,
        double SCM_ML_switch, double dt_dump)
        : m_output_folderpath(output_folderpath), m_wheels(wheels), m_num_frames(10000), m_crt_frame(0), 
        m_model_runner_wheel1(model_runner_wheel1),m_model_runner_wheel2(model_runner_wheel2), 
        m_GHM_0(GHM_0), m_GHM_1(GHM_1), m_GHM_2(GHM_2),
        m_terrain_initLoc(terrain_initLoc), 
        m_wheel_radius(wheel_radius), m_wheel_width(wheel_width), m_heightmap_grid(heightmap_grid),
        m_duration_Img(0), m_duration_F(0), m_duration_all(0), m_SCM_ML_switch(SCM_ML_switch), m_dt_dump(dt_dump){
        
        std::string fn_dataPT = input_folderpath;
        std::cout << "fn_dataPT=" << fn_dataPT << std::endl;
        
        m_I_min_max_wheel1 = loadFromTxt(fn_dataPT + "I_min_max_wheel1.txt");
        m_F_min_max_wheel1 = loadFromTxt(fn_dataPT + "F_min_max_wheel1.txt");
        m_Vec_min_max_wheel1 = loadFromTxt(fn_dataPT + "Vec_min_max_wheel1.txt");

        m_I_min_max_wheel2 = loadFromTxt(fn_dataPT + "I_min_max_wheel2.txt");
        m_F_min_max_wheel2 = loadFromTxt(fn_dataPT + "F_min_max_wheel2.txt");
        m_Vec_min_max_wheel2 = loadFromTxt(fn_dataPT + "Vec_min_max_wheel2.txt");
        
        std::cout << "Load normalized files, done." << std::endl;
        std::cout << "m_I_min_max_wheel1=" << m_I_min_max_wheel1 << ",m_Vec_min_max_wheel1="<<m_Vec_min_max_wheel1 
            <<", m_F_min_max_wheel1="<< m_F_min_max_wheel1<< std::endl;
        std::cout << "m_I_min_max_wheel2=" << m_I_min_max_wheel2 << ",m_Vec_min_max_wheel2="<<m_Vec_min_max_wheel2
            <<", m_F_min_max_wheel2="<< m_F_min_max_wheel2<< std::endl;

        
        if (!filesystem::path(m_output_folderpath).exists()) {
            std::filesystem::create_directory(m_output_folderpath);
        }
    }

    ~TerrainForceLoaderML() { 
        // m_fstream.close(); 
    }

    // Apply forces to wheel bodies, at the beginning of each timestep.
    virtual void Setup() override {
        // std::cout << "line 202" << std::endl;
        auto start_0 = std::chrono::high_resolution_clock::now();
        // Reset load list
        GetLoadList().clear();
        // std::cout << "line 206" << std::endl;
        // std::cout << "line57: Setup m_crt_frame=" << m_crt_frame << std::endl;
        // Generate loads for this time step
        // int wheel_idx;
        // torch::Tensor inI_1chan, inV_ts, F_ts, dI; 
        // torch::Tensor inI_1chan_nm, inV_ts_nm, inI_2chan_nm, outI_nm; 
        torch::Tensor HM_wheel_cur, HM_wheel_org, Vec_ts, F_ts, HM_sinkage, mask_contact, HM_diff;
        torch::Tensor Vec_ts_nm, F_ts_nm, HM_sinkage_nm;
        std::vector<float> positions;
        // std::string filename_hmap_I00;
        
        // std::cout << "line 213" << std::endl;
        for (int iwheel = 0; iwheel < 4; iwheel++) {
            // std::cout << "**********time =" << ChTime <<", i=" << i << std::endl;
            positions = {static_cast<float>(m_wheels[iwheel]->GetPos().x() + m_terrain_initLoc[0]), 
                         static_cast<float>(m_wheels[iwheel]->GetPos().y() + m_terrain_initLoc[1]), 
                         static_cast<float>(m_wheels[iwheel]->GetPos().z() - m_terrain_initLoc[2])};
            torch::Tensor position_tensor = torch::tensor(positions);

            if (iwheel <= 1){
                HM_wheel_cur = m_GHM_1.get_local_heightmap(position_tensor, 96, 72);
                mask_contact = applyWheelSinkage(HM_wheel_cur, position_tensor, m_cx_idx, m_cy_idx, 
                        m_wheel_radius, m_wheel_width, m_heightmap_grid);
                HM_wheel_org = m_GHM_0.get_local_heightmap(position_tensor, 96, 72);
                HM_sinkage = torch::zeros({1, 1, 96, 72});
                HM_diff = HM_wheel_org.index({0, 0}) - HM_wheel_cur.index({0, 0});
                HM_sinkage.index_put_({0, 0, mask_contact}, HM_diff.index({mask_contact}));
                
                // updata heightmap
                m_GHM_1.update_heightmap(position_tensor, HM_wheel_cur);
                m_GHM_2.update_heightmap(position_tensor, HM_wheel_cur);

                // m_model_runner_F = m_model_runner_wheel1;
                m_I_min_max = m_I_min_max_wheel1;
                m_F_min_max = m_F_min_max_wheel1;
                m_Vec_min_max = m_Vec_min_max_wheel1;

            }else{
                HM_wheel_cur = m_GHM_2.get_local_heightmap(position_tensor, 96, 72);
                mask_contact = applyWheelSinkage(HM_wheel_cur, position_tensor, m_cx_idx, m_cy_idx, 
                        m_wheel_radius, m_wheel_width, m_heightmap_grid);
                HM_wheel_org = m_GHM_1.get_local_heightmap(position_tensor, 96, 72);
                HM_sinkage = torch::zeros({1, 1, 96, 72});
                HM_diff = HM_wheel_org.index({0, 0}) - HM_wheel_cur.index({0, 0});
                HM_sinkage.index_put_({0, 0, mask_contact}, HM_diff.index({mask_contact}));
                
                // updata heightmap
                m_GHM_2.update_heightmap(position_tensor, HM_wheel_cur);

                // m_model_runner_F = m_model_runner_wheel2;
                m_I_min_max = m_I_min_max_wheel2;
                m_F_min_max = m_F_min_max_wheel2;
                m_Vec_min_max = m_Vec_min_max_wheel2;
            }
            Vec_ts = torch::tensor({m_wheels[iwheel]->GetPos_dt().x(), m_wheels[iwheel]->GetPos_dt().z()}).to(torch::kFloat32);
            Vec_ts = Vec_ts.unsqueeze(0);
            // calculate the force
            if (torch::allclose(HM_sinkage, torch::zeros_like(HM_sinkage))){
                std::cout << "ChTime="<< ChTime<<", Wheel"<< iwheel << ", sinkage=0, z=" 
                    << position_tensor[2].item()
                    <<",wheel_x=" << m_wheels[iwheel]->GetPos().x()
                    <<",HM_center=" << HM_wheel_cur[0][0][m_cx_idx][m_cy_idx].item()
                    <<",HM_center_org=" << HM_wheel_org[0][0][m_cx_idx][m_cy_idx].item()
                    << ",mask_contact" << mask_contact.sum().item<double>() 
                    << std::endl;
                F_ts = torch::zeros({1, 3});
                F_ts[0][1] = -1000;
            }else{
                // std::cout << "line 342" << std::endl;    
                HM_sinkage_nm = (HM_sinkage - m_I_min_max[0]) / (m_I_min_max[1] - m_I_min_max[0]);
                // std::cout << "line 344" << std::endl;    
                Vec_ts_nm = (Vec_ts - m_Vec_min_max[0]) / (m_Vec_min_max[1] - m_Vec_min_max[0]);
                // std::cout << "line 347" << std::endl;    
                if(iwheel <= 1){
                    F_ts_nm = m_model_runner_wheel1.runModel(HM_sinkage_nm, Vec_ts_nm);
                }else{
                    F_ts_nm = m_model_runner_wheel2.runModel(HM_sinkage_nm, Vec_ts_nm);
                }
                // std::cout << "line 353" << std::endl;    
                F_ts_nm = F_ts_nm.clamp(0, 1); // make F_ts_nm in the range of [0,1]
                F_ts = F_ts_nm * (m_F_min_max[1] - m_F_min_max[0]) + m_F_min_max[0];
            }

            // std::cout << "line 358" << std::endl;    
            double remainder = std::fmod(ChTime, m_dt_dump);
            if (remainder < 1e-6 || (m_dt_dump - remainder) < 1e-6) {
                // std::cout << "ChTime=" << ChTime << std::endl;
                write_output(m_output_folderpath + "I_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", HM_wheel_cur[0][0]);
                // std::cout << "line 361" << std::endl;    
                write_output(m_output_folderpath + "Iskg_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", HM_sinkage[0][0]);
                // std::cout << "line 363, Vec_ts.size()" << Vec_ts.sizes() << std::endl;
                write_output(m_output_folderpath + "Vec_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", Vec_ts);
                // std::cout << "line 365" << std::endl;
                write_output(m_output_folderpath + "F_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", F_ts);
            }
            // std::cout << "line 368" << std::endl;
            if(ChTime >= m_SCM_ML_switch){
                double damping_coeff = 1e5;
                F_ts[0][1] = F_ts[0][1].item<double>() - damping_coeff * m_wheels[iwheel]->GetPos_dt().z();
                auto force_load = chrono_types::make_shared<ChLoadBodyForce>(m_wheels[iwheel], ChVector<>(F_ts[0][0].item<double>(), 0.0, F_ts[0][1].item<double>()), false,
                                                                            m_wheels[iwheel]->GetPos(), false);
                auto torque_load =
                    chrono_types::make_shared<ChLoadBodyTorque>(m_wheels[iwheel], ChVector<>(0.0,F_ts[0][2].item<double>(),0.0), false);
                Add(force_load);
                Add(torque_load);
            }
            
        }
        if (m_crt_frame < m_num_frames - 1)
            m_crt_frame++;
        // std::cout << "line 380" << std::endl;
        // Invoke base class method
        ChLoadContainer::Update(ChTime, true);
        // std::cout << "line 349" << std::endl;
    }

    // UpdateHM(double pos_x, double pos_y, inI_1chan){
    //      m_HM.update_heightmap(position_tensor, inI_1chan);
    // }

  private:
    // std::ifstream m_fstream;
    int m_num_frames;
    int m_crt_frame;
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_wheels;
    TorchModelRunner m_model_runner_wheel1, m_model_runner_wheel2; //, m_model_runner_F;
    Heightmap m_GHM_0, m_GHM_1, m_GHM_2;
    torch::Tensor m_I_min_max_wheel1, m_F_min_max_wheel1, m_Vec_min_max_wheel1;
    torch::Tensor m_I_min_max_wheel2, m_F_min_max_wheel2, m_Vec_min_max_wheel2;
    torch::Tensor m_I_min_max, m_F_min_max, m_Vec_min_max;
    // torch::Tensor m_I_min_max, m_F_min_max, m_dF_min_max, m_Vec_min_max;
    std::string m_output_folderpath;
    ChVector<> m_terrain_initLoc;
    float m_wheel_width, m_wheel_radius, m_heightmap_grid;
    std::chrono::duration<double, std::milli> m_duration_Img, m_duration_F, m_duration_all;
    double m_SCM_ML_switch, m_dt_dump;
    int m_cx_idx = 42, m_cy_idx = 36;
};
// -----------------------------------------------------------------------------
Eigen::MatrixXd GetSCMHM(ChTerrain* terrain, double pos_x, double pos_y, double heightmap_grid, 
    double heightmap_cutoff_x_backward, double heightmap_cutoff_x_forward, double heightmap_cutoff_y_left, double heightmap_cutoff_y_right,
    int nx = 96, int ny = 72){
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
        return hmap_matrix;

    }
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
    
    if (argc != 9) {
        std::cout << "Wrong argv.\nUsage: " << argv[0] 
        << " <dt> <terrain_grid> <time_tot> <dt_dump> <terrain_initX> <terrain_initH> <chassis_density>" << std::endl;
        return 1;
    }
    double dt = std::stod(argv[1]); // 1e-4;
    double terrain_grid = std::stod(argv[2]); // 0.1;
    double time_tot = std::stod(argv[3]); // 3;
    double dt_dump = std::stod(argv[4]); // 1e-4;
    double terrain_initX = std::stod(argv[5]); 
    double terrain_initH = std::stod(argv[6]);
    double chassis_density = std::stod(argv[7]); // 1000;
    double SCM_ML_switch = std::stod(argv[8]);
    double terrain_initY = 0.0; //0.3; // Default value by ChTireTestRig.cpp
    // generate initial terrain from bmp file
    double terrain_sizeX = 50, terrain_sizeY = 1;
    double terrain_hMin = 0, terrain_hMax = 0.1; // 0.05; //
    std::string heightmap_file = "/home/swang597/Documents/Research/chrono_fork_radu/build/data/vehicle/terrain/height_maps/terrain_heightmap_smooth_horizontal_terrGrid0.005_wx50_wy1.bmp";
    double heightmap_grid = terrain_grid;
    double heightmap_cutoff_y_left = 0.18; //0.15; // cylinder wheel width 0.128*2
    double heightmap_cutoff_y_right = 0.18; //0.15; 
    double heightmap_cutoff_x_backward = 0.21; //0.24; // cylinder wheel radius 0.208
    double heightmap_cutoff_x_forward = 0.27; //0.18;
    int cx_idx = 42, cy_idx = 36;

    float wheel_radius = 0.2145; //0.208, 
    float wheel_width = 0.256;


    // bool flag_heightmap_save = false; //true;
    // bool flag_vis = true; //false;
    bool flag_save_vedio = true; //false;
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
    auto viper = CreateViper(sys);
    
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
    Heightmap GHM_0 = Heightmap::init_flat(wx, wy, delta);
    Heightmap GHM_1 = Heightmap::init_flat(wx, wy, delta);
    Heightmap GHM_2 = Heightmap::init_flat(wx, wy, delta);
    std::string output_folderpath = out_dir + "_HMflat_SCM2MLSwitch" + 
            std::to_string(SCM_ML_switch) + "_dt" + std::to_string(dt) + 
            "_chassis_density" + std::to_string(chassis_density) + 
            "MLRig2Wheels/";
    
    
    // Create the bumpy terrain ----------------------------------------------
    // auto terrain_SCM = CreateTerrain(delta, enable_bulldozing, sys, 
    //         heightmap_file, wx, wy, terrain_hMin, terrain_hMax, terrain_initX, terrain_initH);    
    // Heightmap HM = Heightmap::init_bmp(heightmap_file, wx, wy, terrain_hMin, terrain_hMax, delta);
    // std::string output_folderpath = out_dir + "_HMbmp_terrain_hMax"+std::to_string(terrain_hMax)+"_SCM2MLSwitch" + std::to_string(SCM_ML_switch) + "_dt" + std::to_string(dt) + "_240509/";
    
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
    
    if(!std::filesystem::exists(output_folderpath))
        std::filesystem::create_directory(output_folderpath);

    std::string folderpath_normlized = "/home/swang597/Documents/Research/chrono_fork_rserban/Project_TireTestRig2Wheels/build_SCM_argVx/DEMO_OUTPUT/Data_raw_var_Vx_Load_initX_Dataset_collectDataWheel2x0.5_train_240608/";
    std::string model_path = "/home/swang597/Documents/Research/Project_heightmap/Code/Pytorch_cpp_model/Model/Model_MyDatasetRig2Wheels_IskgVxVz_F_240610/";
    std::string model_path_wheel1;
    std::string model_path_wheel2;
    model_path_wheel1 = model_path + "modelF_wheel1_cpu.pt";
    model_path_wheel2 = model_path + "modelF_wheel2_cpu.pt";
    TorchModelRunner model_runner_wheel1(model_path_wheel1);
    TorchModelRunner model_runner_wheel2(model_path_wheel2);
    std::cout << "Load NN model done." << std::endl;

    ChVector<> terrain_initLoc(-terrain_initX + 0.5*wx, -terrain_initY + 0.5*wy, terrain_initH);
    auto terrain_ML = chrono_types::make_shared<TerrainForceLoaderML>(folderpath_normlized,
                        output_folderpath, wheels, model_runner_wheel1, model_runner_wheel2, 
                        GHM_0, GHM_1, GHM_2, terrain_initLoc,
                        wheel_radius, wheel_width, heightmap_grid, SCM_ML_switch, dt_dump);
    std::cout << "TerrainForceLoaderML done." << std::endl;
    sys.Add(terrain_ML);
    std::cout << "Add terrain done." << std::endl;

    // Create the run-time visualization interface
    // auto vis = CreateVisualization(vis_type, true, sys); //Add grid
    auto vis = CreateVisualization(vis_type, false, sys);

    // Open I/O files
    std::ofstream SCM_forces(output_folderpath + "SCM_force_saved.txt", std::ios::trunc);
    std::ofstream ROVER_states(output_folderpath + "ROVER_states_applied.txt", std::ios::trunc);
    std::string HMfilename_SCM;
    Eigen::MatrixXd HM_SCM;
    torch::Tensor inI_1chan;
    // Simulation loop
    ChVector<> wheelContFList_F;
    ChVector<> wheelContFList_M;

    for (int istep = 0; istep < num_steps; istep++) {
        if (istep % 100 == 0)
            cout << "Time: " << sys.GetChTime() << endl;
        double time = sys.GetChTime();
        double remainder = std::fmod(time, dt_dump);
        if(sys.GetChTime() < SCM_ML_switch && (remainder < 1e-6 || (0.01 - remainder) < 1e-6)){
            // save heightmap 
            for(int iwheel = 0; iwheel < 4; iwheel++) {
                ChVector<> wheel_pos = wheels[iwheel]->GetPos();
                HMfilename_SCM = output_folderpath + "hmap_SCM_wheel" + std::to_string(iwheel) + "_Tat" + std::to_string(istep * dt) + ".txt";
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

#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->SetCameraTarget(viper->GetChassis()->GetPos());
        vis->Render();
        vis->EndScene();
#endif
        // std::cout << "line498" << std::endl;
        // Advance system dynamics
        sys.DoStepDynamics(dt);
        // std::cout << "line501" << std::endl;
        viper->Update();
        // std::cout << "line502" << std::endl;
        
        
        
        // std::cout << "line504" << std::endl;
        // Save SCM terrain forces that were applied during the *previous* step
        // if ((istep+1) % 20 == 0){
        
        if (remainder < 1e-6 || (0.01 - remainder) < 1e-6) {
            SCM_forces << time << "    ";
            for (int iwheel = 0; iwheel < 4; iwheel++) {
                terrain_SCM->GetContactForceBody(wheels[iwheel], wheelContFList_F, wheelContFList_M);
                SCM_forces << std::setprecision(20) << wheelContFList_F << "  "
                        << wheelContFList_M << "    ";
            }
            SCM_forces << endl;

        // Save vehicle states
        
            ROVER_states << time << "   ";
            for (int iwheel = 0; iwheel < 4; iwheel++) {
                ROVER_states << wheels[iwheel]->GetPos() << "   " << wheels[iwheel]->GetPos_dt() << "   " << wheels[iwheel]->GetWvel_par()
                            << "   ";
            }
            ROVER_states << endl;
        }

        if(flag_save_vedio && istep % save_vedio_fps == 0){
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
