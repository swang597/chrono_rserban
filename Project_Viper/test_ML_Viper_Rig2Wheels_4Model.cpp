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
torch::Tensor applyWheelSinkageOnGHM(torch::Tensor& heightMap, torch::Tensor wheelCenter, 
                                torch::Tensor gridOrigin, float radius, float width, float gridSize, 
                                double threshold = 1e-3) {
    // Extract wheel center coordinates
    float cx = wheelCenter[0].item<float>();
    float cy = wheelCenter[1].item<float>();
    float cz = wheelCenter[2].item<float>();

    // Extract grid origin coordinates
    float ox = gridOrigin[0].item<float>();
    float oy = gridOrigin[1].item<float>();
    
    int nx = heightMap.size(2);
    int ny = heightMap.size(3);

    // Create meshgrid for x and y coordinates in real-world space
    auto x = torch::arange(0, nx, 1, torch::kFloat32) * gridSize + ox;
    auto y = torch::arange(0, ny, 1, torch::kFloat32) * gridSize + oy;
    auto xv = x.unsqueeze(1).expand({nx, ny});
    auto yv = y.unsqueeze(0).expand({nx, ny});

    // Calculate distances from the wheel center
    auto distance_x = torch::abs(xv - cx);
    auto distance_y = torch::abs(yv - cy);

    // Mask for points within the wheel's width
    auto within_width = distance_y < (width / 2);

    // Mask for points within the wheel's radius
    auto within_radius = distance_x < radius;

    // Calculate wheel surface height
    auto wheel_surface_height = cz - torch::sqrt(radius * radius - distance_x.pow(2));
    // Compute the element-wise minimum between wheel_surface_height and the heightMap at index {0, 0}
    auto min_heights = torch::min(wheel_surface_height, heightMap.index({0, 0}));

    // Mask for points where the wheel surface is below the current height
    auto below_current_height = (wheel_surface_height - heightMap.index({0, 0})) < threshold;

    // Combine masks
    auto mask = within_width & within_radius & below_current_height;

    if (mask.sum().item<float>() == 0) {
        std::cout << "mask.sum()=0" << std::endl;    
    }

    // Update heights
    auto updated_heights = torch::where(mask, min_heights, heightMap.index({0, 0}));

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
        TorchModelRunner model_runner_wheel1_FxTy, TorchModelRunner model_runner_wheel2_FxTy, 
        TorchModelRunner model_runner_wheel1_Fz, TorchModelRunner model_runner_wheel2_Fz, 
        Heightmap GHM_0, Heightmap GHM_1, Heightmap GHM_2, 
        ChVector<> terrain_initLoc, float wheel_radius, float wheel_width, float heightmap_grid,
        double SCM_ML_switch, double dt_dump, double HM_cutoff_y_left, double HM_cutoff_y_right,
        double HM_cutoff_x_backward, double HM_cutoff_x_forward, int cx_idx, int cy_idx)
        : m_output_folderpath(output_folderpath), m_wheels(wheels), m_num_frames(10000), m_crt_frame(0), 
        m_model_runner_wheel1_FxTy(model_runner_wheel1_FxTy), m_model_runner_wheel2_FxTy(model_runner_wheel2_FxTy), 
        m_model_runner_wheel1_Fz(model_runner_wheel1_Fz), m_model_runner_wheel2_Fz(model_runner_wheel2_Fz), 
        m_GHM_0(GHM_0), m_GHM_1(GHM_1), m_GHM_2(GHM_2),
        m_terrain_initLoc(terrain_initLoc), 
        m_wheel_radius(wheel_radius), m_wheel_width(wheel_width), m_heightmap_grid(heightmap_grid),
        m_duration_Img(0), m_duration_F(0), m_duration_all(0), m_SCM_ML_switch(SCM_ML_switch), m_dt_dump(dt_dump),
        m_cx_idx(cx_idx), m_cy_idx(cy_idx),
        m_HM_cutoff_y_left(HM_cutoff_y_left), m_HM_cutoff_y_right(HM_cutoff_y_right), m_HM_cutoff_x_backward(HM_cutoff_x_backward), m_HM_cutoff_x_forward(HM_cutoff_x_forward){
        
        std::string fn_dataPT = input_folderpath;
        std::cout << "fn_dataPT=" << fn_dataPT << std::endl;
        
        //check if all files are exist
        if (!filesystem::path(fn_dataPT + "Iskg_min_max_wheel1.txt").exists()) {
            std::cerr << "Error: file not found: " << fn_dataPT + "Iskg_min_max_wheel1.txt" << std::endl;
        }
        if (!filesystem::path(fn_dataPT + "F_min_max_wheel1.txt").exists()) {
            std::cerr << "Error: file not found: " << fn_dataPT + "F_min_max_wheel1.txt" << std::endl;
        }
        if (!filesystem::path(fn_dataPT + "Vec_min_max_wheel1.txt").exists()) {
            std::cerr << "Error: file not found: " << fn_dataPT + "Vec_min_max_wheel1.txt" << std::endl;
        }
        if (!filesystem::path(fn_dataPT + "Iskg_min_max_wheel2.txt").exists()) {
            std::cerr << "Error: file not found: " << fn_dataPT + "Iskg_min_max_wheel2.txt" << std::endl;
        }
        if (!filesystem::path(fn_dataPT + "F_min_max_wheel2.txt").exists()) {
            std::cerr << "Error: file not found: " << fn_dataPT + "F_min_max_wheel2.txt" << std::endl;
        }
        if (!filesystem::path(fn_dataPT + "Vec_min_max_wheel2.txt").exists()) {
            std::cerr << "Error: file not found: " << fn_dataPT + "Vec_min_max_wheel2.txt" << std::endl;
        }

        m_I_min_max_wheel1 = loadFromTxt(fn_dataPT + "Iskg_min_max_wheel1.txt");
        m_F_min_max_wheel1 = loadFromTxt(fn_dataPT + "F_min_max_wheel1.txt");
        m_Vec_min_max_wheel1 = loadFromTxt(fn_dataPT + "Vec_min_max_wheel1.txt");

        m_I_min_max_wheel2 = loadFromTxt(fn_dataPT + "Iskg_min_max_wheel2.txt");
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

        m_HM_nx = static_cast<int>(std::round((m_HM_cutoff_x_backward + m_HM_cutoff_x_forward) / m_heightmap_grid));
        m_HM_ny = static_cast<int>(std::round((m_HM_cutoff_y_left + m_HM_cutoff_y_right) / m_heightmap_grid));

    }

    ~TerrainForceLoaderML() { 
        // m_fstream.close(); 
    }

    // Apply forces to wheel bodies, at the beginning of each timestep.
    virtual void Setup() override {
        // // std::cout << "line 202" << std::endl;
        auto start_0 = std::chrono::high_resolution_clock::now();
        // Reset load list
        GetLoadList().clear();
        // // std::cout << "line 206" << std::endl;
        torch::Tensor HM_wheel_cur, HM_wheel_org, Vec_ts, F_ts, HM_sinkage, I_sinkage, mask_contact, HM_diff;
        torch::Tensor Vec_ts_nm, F_ts_nm, I_sinkage_nm;
        torch::Tensor F_ts_nm_wheel1_FxTy, F_ts_nm_wheel1_Fz, F_ts_nm_wheel2_FxTy, F_ts_nm_wheel2_Fz;
        torch::Tensor F_ts_nm_wheel1, F_ts_nm_wheel2;
        torch::Tensor F_ts_front, F_ts_rear;
        
        // std::string filename_hmap_I00;
        
        // // std::cout << "line 213  "**********time =" << ChTime << std::endl;
        for (int iwheel = 0; iwheel < 4; iwheel++) {
            
            double cx = m_wheels[iwheel]->GetPos().x()  + m_terrain_initLoc[0]; 
            double cy = m_wheels[iwheel]->GetPos().y()  + m_terrain_initLoc[1];
            double cz = m_wheels[iwheel]->GetPos().z()  - m_terrain_initLoc[2];

            torch::Tensor position_tensor = torch::tensor({static_cast<float>(cx), static_cast<float>(cy), static_cast<float>(cz)});

            // Calculate the fractional part of the wheel position
            double fx = cx / m_heightmap_grid - static_cast<int>(std::floor(cx / m_heightmap_grid));
            double fy = cy / m_heightmap_grid - static_cast<int>(std::floor(cy / m_heightmap_grid));

            double w00 = (1 - fx) * (1 - fy);
            double w01 = fx * (1 - fy);
            double w10 = (1 - fx) * fy;
            double w11 = fx * fy;

            torch::Tensor gridOrigin = torch::tensor({static_cast<float>(cx- m_HM_cutoff_x_forward), static_cast<float>(cy- m_HM_cutoff_y_left)});
            double threshold = 1e-3; //m_heightmap_grid;
            torch::Tensor HM_sinkage = torch::zeros({m_HM_nx+1, m_HM_ny+1});
            torch::Tensor max_I_sinkage ;
            // std::cout << "line 271, wheel=" <<iwheel  << std::endl;
            if (iwheel <= 1){
                // std::cout << "line 273 gridOrigin=" << gridOrigin << std::endl;
                HM_wheel_cur = m_GHM_1.get_local_heightmap_by_origin(gridOrigin, m_HM_nx+1, m_HM_ny+1);
                // std::cout << "line 275" << std::endl;
                mask_contact = applyWheelSinkageOnGHM(HM_wheel_cur, position_tensor, gridOrigin, 
                                        m_wheel_radius, m_wheel_width, m_heightmap_grid, threshold);
                // std::cout << "line 278" << std::endl;
                HM_wheel_org = m_GHM_0.get_local_heightmap_by_origin(gridOrigin, m_HM_nx+1, m_HM_ny+1);
                auto HM_wheel_diff = HM_wheel_org.index({0, 0}) - HM_wheel_cur.index({0, 0});
                HM_sinkage.index_put_({mask_contact}, HM_wheel_diff.index({mask_contact}));
                // std::cout << "line 282" << std::endl;
                I_sinkage = w00 * HM_sinkage.index({torch::indexing::Slice(0, m_HM_nx), torch::indexing::Slice(0, m_HM_ny)}) +
                                w01 * HM_sinkage.index({torch::indexing::Slice(0, m_HM_nx), torch::indexing::Slice(1, m_HM_ny+1)}) +
                                w10 * HM_sinkage.index({torch::indexing::Slice(1, m_HM_nx+1), torch::indexing::Slice(0, m_HM_ny)}) +
                                w11 * HM_sinkage.index({torch::indexing::Slice(1, m_HM_nx+1), torch::indexing::Slice(1, m_HM_ny+1)});
                I_sinkage = I_sinkage.unsqueeze(0).unsqueeze(0);
                // std::cout << "line 288" << std::endl;
                // updata heightmap
                m_GHM_1.update_heightmap_by_origin(gridOrigin, HM_wheel_cur);
                m_GHM_2.update_heightmap_by_origin(gridOrigin, HM_wheel_cur);
                // std::cout << "line 292" << std::endl;
                m_I_min_max = m_I_min_max_wheel1;
                m_F_min_max = m_F_min_max_wheel1;
                m_Vec_min_max = m_Vec_min_max_wheel1;
            }else{
                // std::cout << "line 298 gridOrigin=" << gridOrigin << std::endl;
                HM_wheel_cur = m_GHM_2.get_local_heightmap_by_origin(gridOrigin, m_HM_nx+1, m_HM_ny+1);
                
                mask_contact = applyWheelSinkageOnGHM(HM_wheel_cur, position_tensor, gridOrigin, 
                                        m_wheel_radius, m_wheel_width, m_heightmap_grid, threshold);
                HM_wheel_org = m_GHM_1.get_local_heightmap_by_origin(gridOrigin, m_HM_nx+1, m_HM_ny+1);
                auto HM_wheel_diff = HM_wheel_org.index({0, 0}) - HM_wheel_cur.index({0, 0});
                HM_sinkage.index_put_({mask_contact}, HM_wheel_diff.index({mask_contact}));
                // std::cout << "line 305" << std::endl;
                I_sinkage = w00 * HM_sinkage.index({torch::indexing::Slice(0, m_HM_nx), torch::indexing::Slice(0, m_HM_ny)}) +
                                w01 * HM_sinkage.index({torch::indexing::Slice(0, m_HM_nx), torch::indexing::Slice(1, m_HM_ny+1)}) +
                                w10 * HM_sinkage.index({torch::indexing::Slice(1, m_HM_nx+1), torch::indexing::Slice(0, m_HM_ny)}) +
                                w11 * HM_sinkage.index({torch::indexing::Slice(1, m_HM_nx+1), torch::indexing::Slice(1, m_HM_ny+1)});
                I_sinkage = I_sinkage.unsqueeze(0).unsqueeze(0);
                // std::cout << "line 311" << std::endl;
                // updata heightmap
                m_GHM_2.update_heightmap_by_origin(gridOrigin, HM_wheel_cur);
                // std::cout << "line 314" << std::endl;
                m_I_min_max = m_I_min_max_wheel2;
                m_F_min_max = m_F_min_max_wheel2;
                m_Vec_min_max = m_Vec_min_max_wheel2;
            }
            Vec_ts = torch::tensor({m_wheels[iwheel]->GetPos_dt().x()}).to(torch::kFloat32);
            Vec_ts = Vec_ts.unsqueeze(0);
            // std::cout << "line 319" << std::endl;
            // calculate the force
            if (torch::allclose(HM_sinkage, torch::zeros_like(HM_sinkage))){
                // std::cout << "line 322" << std::endl;
                // std::cout << "ChTime="<< ChTime<<", Wheel"<< iwheel << ", sinkage=0, z=" 
                //     << position_tensor[2].item()
                //     <<",wheel_x=" << m_wheels[iwheel]->GetPos().x()
                //     <<",HM_center=" << HM_wheel_cur[0][0][m_cx_idx][m_cy_idx].item()
                //     <<",HM_center_org=" << HM_wheel_org[0][0][m_cx_idx][m_cy_idx].item()
                //     << ",mask_contact" << mask_contact.sum().item<double>() 
                //     << std::endl;
                F_ts = torch::zeros({1, 3});
                // F_ts[0][1] = -1000;
            }else{
                // std::cout << "line 333" << std::endl;    
                I_sinkage_nm = (I_sinkage - m_I_min_max[0]) / (m_I_min_max[1] - m_I_min_max[0]);
                // std::cout << "line 344" << std::endl;    
                Vec_ts_nm = (Vec_ts - m_Vec_min_max[0]) / (m_Vec_min_max[1] - m_Vec_min_max[0]);
                // std::cout << "line 347" << std::endl;    
                if(iwheel <= 1){
                    F_ts_nm_wheel1_FxTy = m_model_runner_wheel1_FxTy.runModel(I_sinkage_nm, Vec_ts_nm);
                    // F_ts_nm_wheel1_FxTy = F_ts_nm_wheel1_FxTy.clamp(0, 1); // make F_ts_nm_wheel1 in the range of [0,1]
                    
                    F_ts_nm_wheel1_Fz = m_model_runner_wheel1_Fz.runModel(I_sinkage_nm, Vec_ts_nm);
                    // F_ts_nm_wheel1_Fz = F_ts_nm_wheel1_Fz.clamp(0, 1); // make F_ts_nm_wheel1 in the range of [0,1]
                    F_ts_nm_wheel1 = torch::zeros({1, 3});
                    F_ts_nm_wheel1.index({0, 0}) = F_ts_nm_wheel1_FxTy.index({0, 0}); //F_ts_nm_wheel1[0][0] = F_ts_nm_wheel1_FxTy[0][0];
                    F_ts_nm_wheel1.index({0, 1}) = F_ts_nm_wheel1_Fz.index({0, 0});
                    // max_I_sinkage = I_sinkage.max();
                    // F_ts_nm_wheel1.index({0, 1}) = 18667.05 * max_I_sinkage.item<float>() + 32.65; // F_ts_nm_wheel1_Fz[0][0];
                    // F_ts_nm_wheel1[0][1] = 18667.05*max_I_sinkage.item<float>() + 32.65; //F_ts_nm_wheel1_Fz[0][0];
                    F_ts_nm_wheel1.index({0, 2}) = F_ts_nm_wheel1_FxTy.index({0, 1}); //F_ts_nm_wheel1[0][2] = F_ts_nm_wheel1_FxTy[0][1];
                    // F_ts_nm = F_ts_nm_wheel1.clamp(0, 1); // make F_ts_nm in the range of [0,1]
                    F_ts_nm = F_ts_nm_wheel1;
                    // std::cout << "line 348" << std::endl;
                }else{
                    F_ts_nm_wheel2_FxTy = m_model_runner_wheel2_FxTy.runModel(I_sinkage_nm, Vec_ts_nm);
                    // F_ts_nm_wheel2_FxTy = F_ts_nm_wheel2_FxTy.clamp(0, 1); // make F_ts_nm_wheel1 in the range of [0,1]
                    
                    F_ts_nm_wheel2_Fz = m_model_runner_wheel2_Fz.runModel(I_sinkage_nm, Vec_ts_nm);
                    // F_ts_nm_wheel2_Fz = F_ts_nm_wheel2_Fz.clamp(0, 1); // make F_ts_nm_wheel1 in the range of [0,1]
                    F_ts_nm_wheel2 = torch::zeros({1, 3});
                    // F_ts_nm_wheel2[0][0] = F_ts_nm_wheel2_FxTy[0][0];
                    F_ts_nm_wheel2.index({0, 0}) = F_ts_nm_wheel2_FxTy.index({0, 0});
                    F_ts_nm_wheel2.index({0, 1}) = F_ts_nm_wheel2_Fz.index({0, 0});
                    // max_I_sinkage = I_sinkage.max();
                    // F_ts_nm_wheel2[0][1] = 27356.5*max_I_sinkage.item<float>() - 317.64; //F_ts_nm_wheel2_Fz[0][0];
                    // F_ts_nm_wheel2.index({0, 1}) = 27356.5*max_I_sinkage.item<float>() - 317.64; //F_ts_nm_wheel2_Fz[0][0];
                    // F_ts_nm_wheel2[0][2] = F_ts_nm_wheel2_FxTy[0][1];
                    F_ts_nm_wheel2.index({0, 2}) = F_ts_nm_wheel2_FxTy.index({0, 1});
                    // F_ts_nm = F_ts_nm_wheel2.clamp(0, 1); // make F_ts_nm in the range of [0,1]
                    F_ts_nm = F_ts_nm_wheel2;
                    // std::cout << "line 359" << std::endl;
                }
                F_ts = F_ts_nm * (m_F_min_max[1] - m_F_min_max[0]) + m_F_min_max[0];
                // std::cout << "line 360 F_ts=" << F_ts <<", F_ts_nm = " << F_ts_nm << std::endl;
                
             
                // if(Vec_ts_nm[0][0].item<float>() < 0.0){
                //     std::cout << "wheel="<< iwheel << ", Vec_ts_nm: " << Vec_ts_nm << std::endl;
                //     std::cout << "Bf F_ts=" << F_ts << std::endl;
                //     // std::cout << "Bf F_ts_wheel1 change" << std::endl;
                //     F_ts.index({0, 0}) = torch::abs(F_ts.index({0, 0}));
                //     F_ts.index({0, 2}) = -torch::abs(F_ts.index({0, 2}));
                //     std::cout << "Af F_ts=" << F_ts << std::endl;
                // }else if (Vec_ts_nm[0][0].item<float>() > 1.0){
                //     std::cout << "wheel="<< iwheel << ", Vec_ts_nm: " << Vec_ts_nm << std::endl;
                //     std::cout << "Bf F_ts=" << F_ts << std::endl;
                //     F_ts.index({0, 0}) = -torch::abs(F_ts.index({0, 0}));
                //     F_ts.index({0, 2}) = torch::abs(F_ts.index({0, 2}));
                //     std::cout << "Af F_ts=" << F_ts << std::endl;
                // }
                // // std::cout << "line 353" << std::endl;    
                // F_ts_nm = F_ts_nm.clamp(0, 1); // make F_ts_nm in the range of [0,1]
                // F_ts = torch::zeros({1, 3});
                
                // F_ts[1] = F_ts_nm * (m_F_min_max[1] - m_F_min_max[0]) + m_F_min_max[0];
                // F_ts[0] = 100;
                // F_ts[2] = 0;
            }
            
            if (iwheel == 0){
                F_ts_front = F_ts;
            }else if (iwheel == 1){
                F_ts_front += F_ts;
            }else if (iwheel == 2){
                F_ts_rear = F_ts;
            }else{
                F_ts_rear += F_ts;
            }

            // std::cout << "line 388" << std::endl;    
            double remainder = std::fmod(ChTime, m_dt_dump);
            if (remainder < 1e-6 || (m_dt_dump - remainder) < 1e-6) {
                // std::cout << "ChTime=" << ChTime << std::endl;
                write_output(m_output_folderpath + "I_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", HM_wheel_cur[0][0]);
                // std::cout << "line 361" << std::endl;    
                write_output(m_output_folderpath + "Iskg_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", I_sinkage[0][0]);
                // std::cout << "line 363, Vec_ts.size()" << Vec_ts.sizes() << std::endl;
                write_output(m_output_folderpath + "Vec_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", Vec_ts);
                // std::cout << "line 365" << std::endl;
                write_output(m_output_folderpath + "F_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", F_ts);
                // std::cout << "line 367" << std::endl;
            }
            
        }

        // std::cout << "line 368" << std::endl;
        F_ts_front = F_ts_front * 0.5;
        F_ts_rear = F_ts_rear * 0.5;
        // std::cout << "Frone wheel, F_ts=" << ChVector<>(F_ts_front[0][0].item<double>(), 0.0, F_ts_front[0][1].item<double>()) 
        //             <<", M=" << ChVector<>(0.0,F_ts_front[0][2].item<double>(),0.0)<< std::endl;
        // std::cout << "Rear wheel, F_ts=" << ChVector<>(F_ts_rear[0][0].item<double>(), 0.0, F_ts_rear[0][1].item<double>())
        //             <<", M=" << ChVector<>(0.0,F_ts_rear[0][2].item<double>(),0.0)<< std::endl;
        if(ChTime >= m_SCM_ML_switch){
            
            for(int iwheel=0; iwheel<2; iwheel++){
                auto force_load = chrono_types::make_shared<ChLoadBodyForce>(m_wheels[iwheel], ChVector<>(F_ts_front[0][0].item<double>(), 0.0, F_ts_front[0][1].item<double>()), false,
                                                                            m_wheels[iwheel]->GetPos(), false);
                auto torque_load =
                    chrono_types::make_shared<ChLoadBodyTorque>(m_wheels[iwheel], ChVector<>(0.0,F_ts_front[0][2].item<double>(),0.0), false);
                Add(force_load);
                Add(torque_load);
            }
            for(int iwheel=2; iwheel<4; iwheel++){
                auto force_load = chrono_types::make_shared<ChLoadBodyForce>(m_wheels[iwheel], ChVector<>(F_ts_rear[0][0].item<double>(), 0.0, F_ts_rear[0][1].item<double>()), false,
                                                                            m_wheels[iwheel]->GetPos(), false);
                auto torque_load =
                    chrono_types::make_shared<ChLoadBodyTorque>(m_wheels[iwheel], ChVector<>(0.0,F_ts_rear[0][2].item<double>(),0.0), false);
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
    // TorchModelRunner m_model_runner_wheel1, m_model_runner_wheel2; //, m_model_runner_F;
    TorchModelRunner m_model_runner_wheel1_FxTy, m_model_runner_wheel2_FxTy;
    TorchModelRunner m_model_runner_wheel1_Fz, m_model_runner_wheel2_Fz;
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
    int m_cx_idx, m_cy_idx;
    double m_HM_cutoff_y_left, m_HM_cutoff_y_right, m_HM_cutoff_x_backward, m_HM_cutoff_x_forward;
    int m_HM_nx, m_HM_ny;
    
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
    
    if (argc != 11) {
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
    int flag_bump = std::stoi(argv[9]);
    double terrain_hMax = std::stod(argv[10]);
    double terrain_initY = 0.0; //0.3; // Default value by ChTireTestRig.cpp
    // generate initial terrain from bmp file
    double terrain_sizeX = 50, terrain_sizeY = 1;
    double terrain_hMin = 0;
    // std::string heightmap_file = "/home/swang597/Documents/Research/chrono_fork_radu/build/data/vehicle/terrain/height_maps/terrain_heightmap_smooth_horizontal_terrGrid0.005_wx50_wy1.bmp";
    std::string heightmap_file = "/home/swang597/Documents/Research/Project_heightmap/Fig/Terrain_bmp/terrain_profile.bmp";
    double heightmap_grid = terrain_grid;

    double HM_cutoff_y_left = 0.18; //0.15; // cylinder wheel width 0.128*2
    double HM_cutoff_y_right = 0.18; //0.15; 
    double HM_cutoff_x_backward = 0.24; // 0.21; //0.24; // cylinder wheel radius 0.208
    double HM_cutoff_x_forward = 0.24; //0.27; //0.18;
    int cx_idx = 48, cy_idx = 36;
    
    float wheel_radius = 0.2145; //0.208, 
    float wheel_width = 0.256;
    
    bool flag_save_vedio = true; //false;
    int num_steps = int(time_tot/dt);
    int save_vedio_fps = int(num_steps/100.0);
    int idx_vedio = 0;

    // Create the Chrono system (Z up)
    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Create the rover 
    auto viper = CreateViper(sys);
    
    // Cache wheel bodies
    // std::vector<std::shared_ptr<ChBodyAuxRef>> wheels{
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
    std::string output_folderpath;
    Heightmap GHM_0, GHM_1, GHM_2;
    std::shared_ptr<vehicle::SCMTerrain> terrain_SCM;

    if(flag_bump == 0){
        // Create the flat terrain ----------------------------------------------
        terrain_SCM = CreateTerrain(delta, enable_bulldozing, sys);
        GHM_0 = Heightmap::init_flat(wx, wy, delta);
        GHM_1 = Heightmap::init_flat(wx, wy, delta);
        GHM_2 = Heightmap::init_flat(wx, wy, delta);
        output_folderpath = out_dir + "_HMflat_SCM2MLSwitch" + 
                std::to_string(SCM_ML_switch) + "_dt" + std::to_string(dt) + 
                "_chassis_density" + std::to_string(chassis_density) + 
                "MLRig2Wheels/";
    }else{
        // Create the bumpy terrain ----------------------------------------------
        terrain_SCM = CreateTerrain(delta, enable_bulldozing, sys, 
                heightmap_file, wx, wy, terrain_hMin, terrain_hMax, terrain_initX, terrain_initH);    
        GHM_0 = Heightmap::init_bmp(heightmap_file, wx, wy, terrain_hMin, terrain_hMax, delta);
        GHM_1 = Heightmap::init_bmp(heightmap_file, wx, wy, terrain_hMin, terrain_hMax, delta);
        GHM_2 = Heightmap::init_bmp(heightmap_file, wx, wy, terrain_hMin, terrain_hMax, delta);
        output_folderpath = out_dir + "_HMbmp_hMax"+std::to_string(terrain_hMax)+"_SCM2MLSwitch" + 
                std::to_string(SCM_ML_switch) + "_dt" + std::to_string(dt) + 
                "_chassis_density" + std::to_string(chassis_density) + 
                "MLRig2Wheels/";
    }

    // auto terrain_SCM = CreateTerrain(delta, enable_bulldozing, sys);
    // Heightmap GHM_0 = Heightmap::init_flat(wx, wy, delta);
    // Heightmap GHM_1 = Heightmap::init_flat(wx, wy, delta);
    // Heightmap GHM_2 = Heightmap::init_flat(wx, wy, delta);
    // std::string output_folderpath = out_dir + "_HMflat_SCM2MLSwitch" + 
    //         std::to_string(SCM_ML_switch) + "_dt" + std::to_string(dt) + 
    //         "_chassis_density" + std::to_string(chassis_density) + 
    //         "MLRig2Wheels/";
    
    
    // Create the bumpy terrain ----------------------------------------------
    // auto terrain_SCM = CreateTerrain(delta, enable_bulldozing, sys, 
    //         heightmap_file, wx, wy, terrain_hMin, terrain_hMax, terrain_initX, terrain_initH);    
    // Heightmap GHM_0 = Heightmap::init_bmp(heightmap_file, wx, wy, terrain_hMin, terrain_hMax, delta);
    // Heightmap GHM_1 = Heightmap::init_bmp(heightmap_file, wx, wy, terrain_hMin, terrain_hMax, delta);
    // Heightmap GHM_2 = Heightmap::init_bmp(heightmap_file, wx, wy, terrain_hMin, terrain_hMax, delta);
    // std::string output_folderpath = out_dir + "_HMbmp_hMax"+std::to_string(terrain_hMax)+"_SCM2MLSwitch" + 
    //         std::to_string(SCM_ML_switch) + "_dt" + std::to_string(dt) + 
    //         "_chassis_density" + std::to_string(chassis_density) + 
    //         "MLRig2Wheels/";
    
    std::cout << "output_folderpath=" << output_folderpath << std::endl;
    

    terrain_SCM->SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.20); // set sinkage plot range
    // terrain_SCM->SetVectorizedSinkage();
    terrain_SCM->SetMLSwitchTime(SCM_ML_switch);
    std::vector<std::shared_ptr<chrono::ChBody>> base_wheels;
    for (auto& wheel : wheels) {
        base_wheels.push_back(std::static_pointer_cast<chrono::ChBody>(wheel));
    }
    // terrain_SCM->SetWheels(base_wheels);
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

    std::string folderpath_normlized = "/home/swang597/Documents/Research/chrono_fork_rserban/Project_TireTestRig2Wheels/build_SCM_ML_genData/DEMO_OUTPUT_threshold5e-3/Data_raw_updataGHM_HMbump_varLoad_Dataset_normFiles/";
    
    std::string model_path0_wheel1_FxTy = "/home/swang597/Documents/Research/Project_heightmap/Code/Pytorch_cpp_model/Model/Model_MyDatasetRig2Wheels_IskgVxVz_FxTy_240903/";
    std::string model_path0_wheel2_FxTy = "/home/swang597/Documents/Research/Project_heightmap/Code/Pytorch_cpp_model/Model/Model_MyDatasetRig2Wheels_IskgVxVz_FxTy_240903/";
    std::string model_path0_wheel1_Fz = "/home/swang597/Documents/Research/Project_heightmap/Code/Pytorch_cpp_model/Model/Model_MyDatasetRig2Wheels_Iskg_Fz_240903/";
    std::string model_path0_wheel2_Fz = "/home/swang597/Documents/Research/Project_heightmap/Code/Pytorch_cpp_model/Model/Model_MyDatasetRig2Wheels_Iskg_Fz_240903/";
    
    std::string model_path_wheel1_FxTy, model_path_wheel1_Fz;
    std::string model_path_wheel2_FxTy, model_path_wheel2_Fz;
    model_path_wheel1_FxTy = model_path0_wheel1_FxTy + "modelF_wheel1_cpu.pt";
    model_path_wheel2_FxTy = model_path0_wheel2_FxTy + "modelF_wheel2_cpu.pt";
    model_path_wheel1_Fz = model_path0_wheel1_Fz + "modelF_wheel1_cpu.pt";
    model_path_wheel2_Fz = model_path0_wheel2_Fz + "modelF_wheel2_cpu.pt";
    TorchModelRunner model_runner_wheel1_FxTy(model_path_wheel1_FxTy);
    TorchModelRunner model_runner_wheel2_FxTy(model_path_wheel2_FxTy);
    TorchModelRunner model_runner_wheel1_Fz(model_path_wheel1_Fz);
    TorchModelRunner model_runner_wheel2_Fz(model_path_wheel2_Fz);
    
    std::cout << "Load NN model done." << std::endl;

    ChVector<> terrain_initLoc(-terrain_initX + 0.5*wx, -terrain_initY + 0.5*wy, terrain_initH);
    auto terrain_ML = chrono_types::make_shared<TerrainForceLoaderML>(folderpath_normlized,
                        output_folderpath, wheels,
                        model_runner_wheel1_FxTy, model_runner_wheel2_FxTy,
                        model_runner_wheel1_Fz, model_runner_wheel2_Fz,
                        GHM_0, GHM_1, GHM_2, terrain_initLoc,
                        wheel_radius, wheel_width, heightmap_grid, SCM_ML_switch, dt_dump,
                        HM_cutoff_y_left, HM_cutoff_y_right, HM_cutoff_x_backward, HM_cutoff_x_forward, 
                        cx_idx, cy_idx);

    // auto terrain_ML = chrono_types::make_shared<TerrainForceLoaderML>(folderpath_normlized,
    //                     output_folderpath, wheels, model_runner_wheel1, model_runner_wheel2, 
    //                     GHM_0, GHM_1, GHM_2, terrain_initLoc,
    //                     wheel_radius, wheel_width, heightmap_grid, SCM_ML_switch, dt_dump);
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
        // if(sys.GetChTime() < SCM_ML_switch && (remainder < 1e-6 || (0.01 - remainder) < 1e-6)){
        //     // save heightmap 
        //     for(int iwheel = 0; iwheel < 4; iwheel++) {
        //         ChVector<> wheel_pos = wheels[iwheel]->GetPos();
        //         HMfilename_SCM = output_folderpath + "hmap_SCM_wheel" + std::to_string(iwheel) + "_Tat" + std::to_string(istep * dt) + ".txt";
        //         SaveHeightmap(terrain_SCM.get(), wheel_pos[0], wheel_pos[1], heightmap_grid, 
        //             HM_cutoff_x_backward, HM_cutoff_x_forward, HM_cutoff_y_left,
        //             HM_cutoff_y_right, HMfilename_SCM, istep, istep, dt);
        //         // std::cout << "Save heightmap:" << HMfilename_SCM << std::endl;
        //         // HM_SCM = GetSCMHM(terrain_SCM.get(), wheel_pos[0], wheel_pos[1], heightmap_grid, 
        //         //          heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left,heightmap_cutoff_y_right);
        //         // inI_1chan = torch::from_blob(HM_SCM.data(), {1, 1, HM_SCM.rows(), HM_SCM.cols()}, torch::kFloat32);
        //         // terrain_ML->UpdateHM(wheel_pos, inI_1chan);   
        //     }
        // }

#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->SetCameraTarget(viper->GetChassis()->GetPos());
        vis->Render();
        vis->EndScene();
#endif
        // // std::cout << "line498" << std::endl;
        // Advance system dynamics
        sys.DoStepDynamics(dt);
        // // std::cout << "line501" << std::endl;
        viper->Update();
        // // std::cout << "line502" << std::endl;
        
        
        
        // // std::cout << "line504" << std::endl;
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
    // // std::cout << "line513" << std::endl;

    SCM_forces.close();
    ROVER_states.close();
    // // std::cout << "line517" << std::endl;
    return 0;
}
