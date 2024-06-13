// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demonstration of the single-wheel tire test rig.
//
// =============================================================================

#include <algorithm>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_models/vehicle/hmmwv/tire/HMMWV_ANCFTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_FialaTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_ReissnerTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_RigidTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_TMeasyTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_Pac89Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFToroidalTire.h"
// #include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig2Wheels.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_vehicle/terrain/SCMTerrain.h" //Shu added

#include "TorchModelRunner.hpp"
#include "Heightmap.hpp"
#include <filesystem>
#include <c10/core/Device.h>
#include <c10/core/DeviceType.h>
#include <torch/torch.h>
#include <iostream>
#include <map>
#include "cnpy.h"
//Shu added
#include <torch/torch.h>
#include <chrono> // for time profiling

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
    #include "chrono_postprocess/ChBlender.h"
#endif

#include "demos/SetChronoSolver.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;

// Shu-----------------------------------------------------------------------------
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


// Shu-----------------------------------------------------------------------------
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
    // else{
    //     std::cout << "--> mask.sum()=" << mask.sum().item<float>() << std::endl;
    // }
    
    
    // std::cout <<"applyWheelSinkage: mask.sum()=" << mask.sum().item<float>() << std::endl;

    // std::cout << "***cz=" << cz << std::endl;
    // // if(ChTime == 1.5){
    //     int slice_num_x = 12;
    //     int slice_num_y = 12;
    //     torch::Tensor submatrix;
    //     submatrix = mask.index({torch::indexing::Slice(0, torch::indexing::None, slice_num_x), torch::indexing::Slice(0, torch::indexing::None, slice_num_y)});
    //     std::cout << "mask=\n" << submatrix << std::endl;

    //     submatrix = wheel_surface_height.index({torch::indexing::Slice(0, torch::indexing::None, slice_num_x), torch::indexing::Slice(0, torch::indexing::None, slice_num_y)});
    //     std::cout << "wheel_surface_height=\n" << submatrix << std::endl;

    //     submatrix = heightMap.index({0, 0, torch::indexing::Slice(0, torch::indexing::None, slice_num_x), torch::indexing::Slice(0, torch::indexing::None, slice_num_y)});
    //     std::cout << "heightMap=\n" << submatrix << std::endl;

        // submatrix = below_current_height.index({torch::indexing::Slice(0, torch::indexing::None, slice_num_x), torch::indexing::Slice(0, torch::indexing::None, slice_num_y)});
        // std::cout << "below_current_height=\n" << submatrix << std::endl;
        // submatrix = within_width.index({torch::indexing::Slice(0, torch::indexing::None, slice_num_x), torch::indexing::Slice(0, torch::indexing::None, slice_num_y)});
        // std::cout << "within_width=\n" << submatrix << std::endl;
        // submatrix = within_radius.index({torch::indexing::Slice(0, torch::indexing::None, slice_num_x), torch::indexing::Slice(0, torch::indexing::None, slice_num_y)});
        // std::cout << "within_radius=\n" << submatrix << std::endl;
    // }

    // Update heights
    auto updated_heights = torch::where(mask, wheel_surface_height, heightMap.index({0, 0}));
    // if(updated_heights.sum().item<float>() == 0){
    //     std::cout << "updated_heights.sum()=0" << std::endl;
    // }
        // submatrix = updated_heights.index({torch::indexing::Slice(0, torch::indexing::None, slice_num_x), torch::indexing::Slice(0, torch::indexing::None, slice_num_y)});
        // std::cout << "updated_heights=\n" << submatrix << std::endl;

    // Apply updated heights to heightMap
    heightMap.index_put_({0, 0}, updated_heights);

    return mask;
}

// auto terrain_ML = chrono_types::make_shared<TerrainForceLoader>(folderpath_normlized,
//                         output_folderpath, wheel, wheel2, model_runner_wheel1, model_runner_wheel2, GHM_0, terrain_initLoc,
//                         wheel_radius, wheel_width, heightmap_grid, SCM_ML_switch, dt_HM);

// Shu -----------------------------------------------------------------------------
class TerrainForceLoader : public ChLoadContainer {
  public:
    TerrainForceLoader(const std::string& input_folderpath, std::string& output_folderpath, 
    std::shared_ptr<hmmwv::HMMWV_Wheel> wheel1, std::shared_ptr<hmmwv::HMMWV_Wheel> wheel2, 
        TorchModelRunner model_runner_wheel1, TorchModelRunner model_runner_wheel2, 
        Heightmap GHM_0, Heightmap GHM_1, Heightmap GHM_2, 
        ChVector<> terrain_initLoc, float wheel_radius, float wheel_width, float heightmap_grid,
        double SCM_ML_switch, double dt_dump)
        : m_output_folderpath(output_folderpath), m_wheel1(wheel1), m_wheel2(wheel2), m_num_frames(10000), m_crt_frame(0), 
        m_model_runner_wheel1(model_runner_wheel1), m_model_runner_wheel2(model_runner_wheel2), 
        m_GHM_0(GHM_0), m_GHM_1(GHM_1), m_GHM_2(GHM_2), m_terrain_initLoc(terrain_initLoc), 
        m_wheel_radius(wheel_radius), m_wheel_width(wheel_width), m_heightmap_grid(heightmap_grid),
        m_duration_Img(0), m_duration_F(0), m_duration_all(0), m_SCM_ML_switch(SCM_ML_switch), m_dt_dump(dt_dump),
        m_cx_idx(42), m_cy_idx(36){
        // ChVector<> location(terrain_initX, m_terrain_offset, terrain_initH);
        // Read normalized parameters
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

        // m_GHM_1 = m_GHM_0.clone();
        // m_GHM_2 = m_GHM_0.clone();

        if (!filesystem::path(m_output_folderpath).exists()) {
            std::filesystem::create_directory(m_output_folderpath);
        }
        std::cout << "m_output_folderpath=" << m_output_folderpath << std::endl;
        
    }

    ~TerrainForceLoader() { 
        // m_fstream.close(); 
    }


    // Apply forces to wheel bodies, at the beginning of each timestep.
    // Read HM from SCM dump hmap
    virtual void Setup() override {
        auto start_0 = std::chrono::high_resolution_clock::now();
        GetLoadList().clear();
        torch::Tensor HM_wheel1_cur, HM_wheel2_cur, Vec_ts_wheel1, Vec_ts_wheel2, F_ts_wheel1, F_ts_wheel2; 
        torch::Tensor HM_wheel1_org, HM_wheel2_org;
        torch::Tensor HM_sinkage_wheel1, HM_sinkage_wheel2;
        torch::Tensor HM_sinkage_nm_wheel1, HM_sinkage_nm_wheel2, Vec_ts_nm_wheel1, Vec_ts_nm_wheel2, F_ts_nm_wheel1, F_ts_nm_wheel2; 
        torch::Tensor mask_contact_wheel1, mask_contact_wheel2;
        std::vector<float> positions_wheel1, positions_wheel2;
        m_wheel1_state = m_wheel1->GetState();
        m_wheel2_state = m_wheel2->GetState();
    
        // *********** NN_I save HM to check HM:**********
        positions_wheel1 = {static_cast<float>(m_wheel1_state.pos[0] + m_terrain_initLoc[0]), 
                    static_cast<float>(m_wheel1_state.pos[1] + m_terrain_initLoc[1]), 
                    static_cast<float>(m_wheel1_state.pos[2] - m_terrain_initLoc[2])};
        positions_wheel2 = {static_cast<float>(m_wheel2_state.pos[0] + m_terrain_initLoc[0]),
                    static_cast<float>(m_wheel2_state.pos[1] + m_terrain_initLoc[1]), 
                    static_cast<float>(m_wheel2_state.pos[2] - m_terrain_initLoc[2])};
        torch::Tensor position_ts_wheel1 = torch::tensor(positions_wheel1);
        torch::Tensor position_ts_wheel2 = torch::tensor(positions_wheel2);
        // std::cout << "line257" << std::endl;
        // Load HM from Heightmap
        HM_wheel1_cur = m_GHM_1.get_local_heightmap(position_ts_wheel1, 96, 72);
        HM_wheel2_cur = m_GHM_2.get_local_heightmap(position_ts_wheel2, 96, 72);
        // std::cout << "line261" << std::endl;
        // Update HM by Geometry compute applyWheelSinkage
        auto start_I = std::chrono::high_resolution_clock::now();
        mask_contact_wheel1 = applyWheelSinkage(HM_wheel1_cur, position_ts_wheel1, m_cx_idx, m_cy_idx, m_wheel_radius, m_wheel_width, m_heightmap_grid);
        mask_contact_wheel2 = applyWheelSinkage(HM_wheel2_cur, position_ts_wheel2, m_cx_idx, m_cy_idx, m_wheel_radius, m_wheel_width, m_heightmap_grid);
        auto end_I = std::chrono::high_resolution_clock::now();
        // std::cout << "mask_contact_wheel1.sum()=" << mask_contact_wheel1.sum().item<float>() << std::endl;
        // std::cout << "mask_contact_wheel2.sum()=" << mask_contact_wheel2.sum().item<float>() << std::endl;

        // std::cout << "line266" << std::endl;
        HM_wheel1_org = m_GHM_0.get_local_heightmap(position_ts_wheel1, 96, 72);
        HM_wheel2_org = m_GHM_1.get_local_heightmap(position_ts_wheel2, 96, 72);
        // std::cout << "HM_wheel1_org.size" << HM_wheel1_org.sizes() << std::endl;
        // std::cout << "HM_wheel2_org.size" << HM_wheel2_org.sizes() << std::endl;
        
        HM_sinkage_wheel1 = torch::zeros({1, 1, 96, 72});
        HM_sinkage_wheel2 = torch::zeros({1, 1, 96, 72});
        
        // Update sinkage maps
        auto HM_wheel1_diff = HM_wheel1_org.index({0, 0}) - HM_wheel1_cur.index({0, 0});
        auto HM_wheel2_diff = HM_wheel2_org.index({0, 0}) - HM_wheel2_cur.index({0, 0});

        HM_sinkage_wheel1.index_put_({0, 0, mask_contact_wheel1}, HM_wheel1_diff.index({mask_contact_wheel1}));
        HM_sinkage_wheel2.index_put_({0, 0, mask_contact_wheel2}, HM_wheel2_diff.index({mask_contact_wheel2}));

        // HM_sinkage_wheel1 = HM_wheel1_org - HM_wheel1;
        // HM_sinkage_wheel2 = HM_wheel2_org - HM_wheel2;

        m_GHM_1.update_heightmap(position_ts_wheel1, HM_wheel1_cur);
        m_GHM_2.update_heightmap(position_ts_wheel1, HM_wheel1_cur);

        m_GHM_2.update_heightmap(position_ts_wheel2, HM_wheel2_cur);
            
        auto start_F = std::chrono::high_resolution_clock::now();
        // If HM_sinkage_wheel1 is all zeros, then F = {0, 0, 0}
        if (torch::allclose(HM_sinkage_wheel1, torch::zeros_like(HM_sinkage_wheel1))){
            std::cout << "ChTime="<< ChTime<<", Wheel_1, sinkage=0, z=" << position_ts_wheel1[2].item()
                <<"wheel_x=" << m_wheel1_state.pos[0] 
                <<"HM_center=" << HM_wheel1_cur[0][0][m_cx_idx][m_cy_idx].item()
                <<"HM_center_org=" << HM_wheel1_org[0][0][m_cx_idx][m_cy_idx].item()
                << "mask_contact_wheel1" << mask_contact_wheel1.sizes()<<"," << mask_contact_wheel1.sum().item<double>() 
                << std::endl;
            F_ts_wheel1 = torch::zeros({1, 3});
            F_ts_wheel1[0][1] = -1000;
        }else{
            HM_sinkage_nm_wheel1 = (HM_sinkage_wheel1 - m_I_min_max_wheel1[0]) / (m_I_min_max_wheel1[1] - m_I_min_max_wheel1[0]);
            Vec_ts_wheel1 = torch::tensor({m_wheel1_state.lin_vel[0], m_wheel1_state.lin_vel[2]}).to(torch::kFloat32);
            Vec_ts_wheel1 = Vec_ts_wheel1.unsqueeze(0);
            Vec_ts_nm_wheel1 = (Vec_ts_wheel1 - m_Vec_min_max_wheel1[0]) / (m_Vec_min_max_wheel1[1] - m_Vec_min_max_wheel1[0]);
            
            F_ts_nm_wheel1 = m_model_runner_wheel1.runModel(HM_sinkage_nm_wheel1, Vec_ts_nm_wheel1);
            F_ts_nm_wheel1 = F_ts_nm_wheel1.clamp(0, 1); // make F_ts_nm_wheel1 in the range of [0,1]
            F_ts_wheel1 = F_ts_nm_wheel1 * (m_F_min_max_wheel1[1] - m_F_min_max_wheel1[0]) + m_F_min_max_wheel1[0];
        }
        if (torch::allclose(HM_sinkage_wheel2, torch::zeros_like(HM_sinkage_wheel2))){
            std::cout << "ChTime="<< ChTime<<", Wheel_2, sinkage=0, z=" << position_ts_wheel2[2].item() 
                <<"wheel_x=" << m_wheel2_state.pos[0] <<"HM_center=" << HM_wheel2_cur[0][0][m_cx_idx][m_cy_idx].item() << std::endl;
            F_ts_wheel2 = torch::zeros({1, 3});
            F_ts_wheel2[0][1] = -1000;
        }else{
            HM_sinkage_nm_wheel2 = (HM_sinkage_wheel2 - m_I_min_max_wheel2[0]) / (m_I_min_max_wheel2[1] - m_I_min_max_wheel2[0]);
            Vec_ts_wheel2 = torch::tensor({m_wheel2_state.lin_vel[0], m_wheel2_state.lin_vel[2]}).to(torch::kFloat32);
            Vec_ts_wheel2 = Vec_ts_wheel2.unsqueeze(0);
            Vec_ts_nm_wheel2 = (Vec_ts_wheel2 - m_Vec_min_max_wheel2[0]) / (m_Vec_min_max_wheel2[1] - m_Vec_min_max_wheel2[0]);
            
            F_ts_nm_wheel2 = m_model_runner_wheel2.runModel(HM_sinkage_nm_wheel2, Vec_ts_nm_wheel2);
            F_ts_nm_wheel2 = F_ts_nm_wheel2.clamp(0, 1); // make F_ts_nm_wheel2 in the range of [0,1]
            F_ts_wheel2 = F_ts_nm_wheel2 * (m_F_min_max_wheel2[1] - m_F_min_max_wheel2[0]) + m_F_min_max_wheel2[0];
        }
        auto end_F = std::chrono::high_resolution_clock::now();

        double remainder = std::fmod(ChTime, m_dt_dump);
        if (remainder < 1e-6 || (m_dt_dump - remainder) < 1e-6) {
            write_output(m_output_folderpath + "I_wheel1_t" + std::to_string(ChTime) + ".txt", HM_wheel1_cur[0][0]);
            write_output(m_output_folderpath + "Iskg_wheel1_t" + std::to_string(ChTime) + ".txt", HM_sinkage_wheel1[0][0]);
            write_output(m_output_folderpath + "Vec_wheel1_t" + std::to_string(ChTime) + ".txt", Vec_ts_wheel1);
            write_output(m_output_folderpath + "F_wheel1_t" + std::to_string(ChTime) + ".txt", F_ts_wheel1);
        
            write_output(m_output_folderpath + "I_wheel2_t" + std::to_string(ChTime) + ".txt", HM_wheel2_cur[0][0]);
            write_output(m_output_folderpath + "Iskg_wheel2_t" + std::to_string(ChTime) + ".txt", HM_sinkage_wheel2[0][0]);
            write_output(m_output_folderpath + "Vec_wheel2_t" + std::to_string(ChTime) + ".txt", Vec_ts_wheel2);
            write_output(m_output_folderpath + "F_wheel2_t" + std::to_string(ChTime) + ".txt", F_ts_wheel2);
        
        }
        // if (torch::allclose(HM_sinkage_wheel1, torch::zeros_like(HM_sinkage_wheel1)) && 
        //     torch::allclose(HM_sinkage_wheel2, torch::zeros_like(HM_sinkage_wheel2))){
        //     return;
        // }
        // std::cout << "HM_sinkage_wheel1.size" << HM_sinkage_wheel1.sizes() << std::endl;
        // std::cout << "HM_sinkage_wheel2.size" << HM_sinkage_wheel2.sizes() << std::endl;

        // std::cout << "line269" << std::endl;
        // HM_sinkage_nm_wheel2 = (HM_sinkage_wheel2 - m_I_min_max_wheel2[0]) / (m_I_min_max_wheel2[1] - m_I_min_max_wheel2[0]);
        // std::cout << "line272" << std::endl;
        // Use wheel1's sinkage to update m_GHM_1 and m_GHM_2
        
        // Use wheel2's sinkage to update m_GHM_2 and m_GHM_0
        // m_GHM_2.update_heightmap(position_ts_wheel2, HM_wheel2);
        // m_GHM_0.update_heightmap(position_ts_wheel2, HM_nm_wheel2);
        // std::cout << "line277" << std::endl;
        // write_output(m_output_folderpath + "Iskg_t" + std::to_string(ChTime) + ".txt", inI_1chan[0][0]);
        // write_output(m_output_folderpath + "NN_Iin_Tat" + std::to_string(ChTime) + "_pos.txt", position_ts.unsqueeze(0));
        
        // Vec_ts_wheel2 = torch::tensor({m_wheel2_state.lin_vel[0], m_wheel2_state.lin_vel[2]}).to(torch::kFloat32);
        // std::cout << "line296" << std::endl;
        // Vec_ts_nm_wheel2 = (Vec_ts_wheel2 - m_Vec_min_max_wheel2[0]) / (m_Vec_min_max_wheel2[1] - m_Vec_min_max_wheel2[0]);
        // std::cout << "line299" << std::endl;
        
        // Vec_ts_nm_wheel2 = Vec_ts_nm_wheel2.unsqueeze(0);
        
        
        // F_ts_nm_wheel2 = m_model_runner_wheel2.runModel(HM_sinkage_nm_wheel2, Vec_ts_nm_wheel2);
        
        // F_ts_wheel2 = F_ts_nm_wheel2 * (m_F_min_max_wheel2[1] - m_F_min_max_wheel2[0]) + m_F_min_max_wheel2[0];
        // std::cout << "line304" << std::endl;
        // Hybrid 
        if(ChTime >= m_SCM_ML_switch){
            // std::cout << "ML ChTime=" << ChTime << std::endl;
            // if(inV_ts_nm[0][0].item<double>() > 1.0){
            //     F_ts[0][0] = 0;
            //     F_ts[0][1] = 0;
            //     F_ts[0][2] = 0;
            // }
            // Add a damping force to reduce the bumping effect
            double damping_coeff = 1e5;
            F_ts_wheel1[0][1] = F_ts_wheel1[0][1].item<double>() - damping_coeff * m_wheel1_state.lin_vel[2];
            F_ts_wheel2[0][1] = F_ts_wheel2[0][1].item<double>() - damping_coeff * m_wheel2_state.lin_vel[2];

            auto force_load_wheel1 = chrono_types::make_shared<ChLoadBodyForce>(m_wheel1->GetSpindle(), ChVector<>(F_ts_wheel1[0][0].item<double>(), 0.0, F_ts_wheel1[0][1].item<double>()), false,
                                                                        m_wheel1_state.pos, false);
            auto torque_load_wheel1 =
                chrono_types::make_shared<ChLoadBodyTorque>(m_wheel1->GetSpindle(), ChVector<>(0.0,F_ts_wheel1[0][2].item<double>(),0.0), false);
            
            auto force_load_wheel2 = chrono_types::make_shared<ChLoadBodyForce>(m_wheel2->GetSpindle(), ChVector<>(F_ts_wheel2[0][0].item<double>(), 0.0, F_ts_wheel2[0][1].item<double>()), false,
                                                                        m_wheel2_state.pos, false);
            auto torque_load_wheel2 =
                chrono_types::make_shared<ChLoadBodyTorque>(m_wheel2->GetSpindle(), ChVector<>(0.0,F_ts_wheel2[0][2].item<double>(),0.0), false);

            // Add the load to the load container
            Add(force_load_wheel1);
            Add(torque_load_wheel1);
            Add(force_load_wheel2);
            Add(torque_load_wheel2);
        }
        auto end_0 = std::chrono::high_resolution_clock::now();
        m_duration_all += end_0 - start_0;
        m_duration_F += end_F - start_F;
        m_duration_Img += end_I - start_I;
        if (m_crt_frame % 20 == 0){
            // std::cout << "ChTime" << ChTime <<",m_duration_all=" << m_duration_all.count() 
            // <<",m_duration_F=" << m_duration_F.count()  
            // <<",m_duration_Img="<<m_duration_Img.count()  <<"ms"<< std::endl;
            std::cout << ChTime <<"," << m_duration_all.count() 
            <<"," << m_duration_F.count()  
            <<","<<m_duration_Img.count() <<",0.0,0.0" << std::endl;
        }

        if (m_crt_frame < m_num_frames - 1)
            m_crt_frame++;

        // Invoke base class method
        ChLoadContainer::Update(ChTime, true);
    }

  private:
    // std::ifstream m_fstream;
    int m_num_frames;
    int m_crt_frame;
    std::shared_ptr<hmmwv::HMMWV_Wheel> m_wheel1, m_wheel2;
    WheelState m_wheel1_state, m_wheel2_state;
    // std::vector<std::array<ChVector<>, 4>> m_forces;
    // std::vector<std::array<ChVector<>, 4>> m_torques;
    TorchModelRunner m_model_runner_wheel1, m_model_runner_wheel2;
    Heightmap m_GHM_0, m_GHM_1, m_GHM_2;
    torch::Tensor m_I_min_max_wheel1, m_F_min_max_wheel1, m_Vec_min_max_wheel1;
    torch::Tensor m_I_min_max_wheel2, m_F_min_max_wheel2, m_Vec_min_max_wheel2;
    std::string m_output_folderpath;
    ChVector<> m_terrain_initLoc;
    float m_wheel_width, m_wheel_radius, m_heightmap_grid;
    std::chrono::duration<double, std::milli> m_duration_Img, m_duration_F, m_duration_all;
    double m_SCM_ML_switch, m_dt_dump;
    int m_cx_idx, m_cy_idx;
    // double m_terrain_initX, m_terrain_initY;
    // std::ofstream m_fp_I, m_fp_dI, m_fp_Vec, m_fp_F;
};

// -----------------------------------------------------------------------------            
void SaveHeightmap(ChTerrain* terrain, double pos_x, double pos_y, double heightmap_grid, 
    double heightmap_cutoff_x_backward, double heightmap_cutoff_x_forward, double heightmap_cutoff_y_left, double heightmap_cutoff_y_right,
    std::string out_dir, int istep_pos, int istep_cur, double dt, int nx = 96, int ny = 72){
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
        std::string HMfilename = out_dir; // + "/hmap_Pat" + std::to_string(istep_pos * dt) + "_Tat" + std::to_string(istep_cur * dt) + ".txt";
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
void SaveHeightmapBinary(ChTerrain* terrain, double pos_x, double pos_y, double heightmap_grid, 
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

        // Save the matrix to a binary file
        std::string binaryFilename = out_dir + "/hmap_Pat" + std::to_string(istep_pos * dt) + "_Tat" + std::to_string(istep_cur * dt) + ".bin";
        std::ofstream file_binary(binaryFilename, std::ios::out | std::ios::binary | std::ios::trunc);
        if (file_binary.is_open()) {
            // Write the dimensions first
            int rows = hmap_matrix.rows(), cols = hmap_matrix.cols();
            file_binary.write(reinterpret_cast<char*>(&rows), sizeof(rows));
            file_binary.write(reinterpret_cast<char*>(&cols), sizeof(cols));

            // Write the matrix data
            file_binary.write(reinterpret_cast<char*>(hmap_matrix.data()), rows * cols * sizeof(double));
            file_binary.close();
        } else {
            std::cerr << "Unable to open file: " << binaryFilename << std::endl;
        }
}
// -----------------------------------------------------------------------------

// Contact formulation type (SMC or NSC)
// ChContactMethod contact_method = ChContactMethod::NSC;
ChContactMethod contact_method = ChContactMethod::SMC;

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Tire model
enum class TireType { RIGID, TMEASY, FIALA, PAC89, PAC02, ANCF4, ANCF8, ANCF_TOROIDAL, REISSNER };
// TireType tire_type = TireType::TMEASY;
TireType tire_type = TireType::RIGID;

// Read from JSON specification file?
bool use_JSON = true;

bool gnuplot_output = true;
bool blender_output = true;

// -----------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    
    SetChronoDataPath("/home/swang597/Documents/Research/chrono_fork_radu/build/data/");
    SetDataPath("/home/swang597/Documents/Research/chrono_fork_radu/build/data/vehicle/");
    
    if (argc != 10) {
        std::cout << "Wrong argv.\nUsage: " << argv[0] 
        << " <dt> <terrain_grid> <time_tot> <dt_HM> <terrain_initX> <terrain_initH> <normal_load>" << std::endl;
        return 1;
        
    }
    double dt = std::stod(argv[1]); // 1e-4;
    double terrain_grid = std::stod(argv[2]); // 0.1;
    double time_tot = std::stod(argv[3]); // 3;
    double dt_HM = std::stod(argv[4]); // 1e-4;
    double terrain_initX = std::stod(argv[5]); 
    double terrain_initH = std::stod(argv[6]);
    double normal_load = std::stod(argv[7]); // 1000;
    double fixed_Vx = std::stod(argv[8]); // 0.2;
    double SCM_ML_switch = std::stod(argv[9]); // 3.0;
    double terrain_initY = 0.3; // Default value by ChTireTestRig.cpp
    // generate initial terrain from bmp file
    double terrain_sizeX = 50, terrain_sizeY = 1;
    double terrain_hMin = 0, terrain_hMax = 0.1; 
    std::string heightmap_file = "/home/swang597/Documents/Research/chrono_fork_radu/build/data/vehicle/terrain/height_maps/terrain_heightmap_smooth_horizontal_terrGrid0.005_wx50_wy1.bmp";
    
    double dx_2wheels = -1.0, dy_2wheels = 0.0, dz_2wheels = 0.0;

    double time_delay = sqrt(2 * std::abs(terrain_initH) / 9.81) + 0.5; //sqrt(2*abs(H)/9.81) + 0.5
    time_tot += time_delay;

    double heightmap_grid = terrain_grid;
    double heightmap_cutoff_y_left = 0.18; //0.15; // cylinder wheel width 0.128*2
    double heightmap_cutoff_y_right = 0.18; //0.15; 
    double heightmap_cutoff_x_backward = 0.21; //0.24; // cylinder wheel radius 0.208
    double heightmap_cutoff_x_forward = 0.27; //0.18;
    int cx_idx = 42, cy_idx = 36;
    
    float wheel_radius = 0.2145; //0.208, 
    float wheel_width = 0.256;

    bool flag_heightmap_save = true;
    bool flag_vis = true; //false;
    bool flag_save_vedio = true; //false;

    int num_steps = int(time_tot/dt);
    int ndt_HM = int(dt_HM/dt);
    int save_vedio_fps = int(num_steps/100.0);
    int idx_vedio = 0;
    
    
    std::cout << "num_steps: " << num_steps << ", ndt_HM:" << ndt_HM << ", time_delay="<< time_delay<< std::endl;

    // Output directory
    // Hybrid_fixW_onlyNNF1Chan_dt
    // Hybrid_fixW_onlyNNF1Chan_expScale_dt
    // SCM_fixW_dt
    bool flag_varVx = true;
    const std::string out_dir = GetChronoOutputPath() + "Hybrid_fixW_dt" + std::to_string(dt) + "_terrGrid" +
                            std::to_string(terrain_grid) + "terrX" + std::to_string(terrain_initX) + "terrH" + 
                            std::to_string(terrain_initH)+ "normLoad" + std::to_string(normal_load)+ "_HMFlat_varVx_240610";

    // Create wheel and tire subsystems
    auto wheel = chrono_types::make_shared<hmmwv::HMMWV_Wheel>("Wheel");
    auto wheel2 = chrono_types::make_shared<hmmwv::HMMWV_Wheel>("Wheel2");

    std::shared_ptr<ChTire> tire;
    std::shared_ptr<ChTire> tire2;
    
    if (tire_type == TireType::ANCF_TOROIDAL) {
        auto ancf_tire = chrono_types::make_shared<ANCFToroidalTire>("ANCFtoroidal tire");
        ancf_tire->SetRimRadius(0.27);
        ancf_tire->SetHeight(0.18);
        ancf_tire->SetThickness(0.015);
        ancf_tire->SetDivCircumference(40);
        ancf_tire->SetDivWidth(8);
        ancf_tire->SetPressure(320e3);
        ancf_tire->SetAlpha(0.15);
        tire = ancf_tire;
    } else if (use_JSON) {
        std::string tire_file;
        switch (tire_type) {
            case TireType::RIGID:
                // tire_file = "hmmwv/tire/HMMWV_RigidTire.json";
                tire_file = "viper/Viper_RigidMeshTire_cylind.json";
                break;
            case TireType::TMEASY:
                tire_file = "hmmwv/tire/HMMWV_TMeasyTire.json";
                break;
            case TireType::FIALA:
                tire_file = "hmmwv/tire/HMMWV_FialaTire.json";
                break;
            case TireType::PAC89:
                tire_file = "hmmwv/tire/HMMWV_Pac89Tire.json";
                break;
            case TireType::PAC02:
                tire_file = "hmmwv/tire/HMMWV_Pac02Tire.json";
                break;
            case TireType::ANCF4:
                tire_file = "hmmwv/tire/HMMWV_ANCF4Tire_Lumped.json";
                break;
            case TireType::ANCF8:
                tire_file = "hmmwv/tire/HMMWV_ANCF8Tire_Lumped.json";
                break;
            case TireType::REISSNER:
                tire_file = "hmmwv/tire/HMMWV_ReissnerTire.json";
                break;
        }
        tire = ReadTireJSON(vehicle::GetDataFile(tire_file));
        tire2 = ReadTireJSON(vehicle::GetDataFile(tire_file));
    } else {
        switch (tire_type) {
            case TireType::RIGID:
                tire = chrono_types::make_shared<hmmwv::HMMWV_RigidTire>("Rigid tire");
                break;
            case TireType::TMEASY:
                tire = chrono_types::make_shared<hmmwv::HMMWV_TMeasyTire>("TMeasy tire");
                break;
            case TireType::FIALA:
                tire = chrono_types::make_shared<hmmwv::HMMWV_FialaTire>("Fiala tire");
                break;
            case TireType::PAC89:
                tire = chrono_types::make_shared<hmmwv::HMMWV_Pac89Tire>("Pac89 tire");
                break;
            case TireType::ANCF4:
                tire = chrono_types::make_shared<hmmwv::HMMWV_ANCFTire>("ANCF tire",
                                                                        hmmwv::HMMWV_ANCFTire::ElementType::ANCF_4);
                break;
            case TireType::ANCF8:
                tire = chrono_types::make_shared<hmmwv::HMMWV_ANCFTire>("ANCF tire",
                                                                        hmmwv::HMMWV_ANCFTire::ElementType::ANCF_8);
                break;
            case TireType::REISSNER:
                tire = chrono_types::make_shared<hmmwv::HMMWV_ReissnerTire>("Reissner tire");
                break;
        }
    }

    // Create system and set solver
    ChSystem* sys = nullptr;
    ChSolver::Type solver_type;
    ChTimestepper::Type integrator_type;
    double step_size = dt;

    std::cout << "Tire Type: " << tire->GetTemplateName() <<",tire->GetRadius()="<< tire->GetRadius()<< std::endl;

    if (tire_type == TireType::ANCF4 || tire_type == TireType::ANCF8 || tire_type == TireType::ANCF_TOROIDAL ||
        tire_type == TireType::REISSNER) {
        if (contact_method != ChContactMethod::SMC)
            std::cout << "\nWarning! Contact formulation changed to SMC.\n" << std::endl;
        contact_method = ChContactMethod::SMC;
    }

    switch (contact_method) {
        case ChContactMethod::SMC:
            std::cout << "Contact Method:SMC" << std::endl;
            sys = new ChSystemSMC;
            step_size = dt;
            solver_type = ChSolver::Type::PARDISO_MKL;
            integrator_type = ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
            std::static_pointer_cast<ChDeformableTire>(tire)->SetContactFaceThickness(0.02);
            break;
            

        case ChContactMethod::NSC:
            std::cout << "Contact Method:NSC" << std::endl;
            sys = new ChSystemNSC;
            step_size = 1e-3;
            solver_type = ChSolver::Type::BARZILAIBORWEIN;
            integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
            break;
    }

    SetChronoSolver(*sys, solver_type, integrator_type);

    // Create and configure test rig
    // ChTireTestRig rig(wheel, tire, sys);
    ChTireTestRig2Wheels rig(wheel, wheel2, tire, tire2, dx_2wheels, dy_2wheels, dz_2wheels, sys);

    ////rig.SetGravitationalAcceleration(0);
    // rig.SetNormalLoad(normal_load);
    rig.SetNormalLoad(normal_load, 2500 - normal_load);

    ////rig.SetCamberAngle(+15 * CH_C_DEG_TO_RAD);

    rig.SetTireStepsize(step_size);
    rig.SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
    rig.SetTireVisualizationType(VisualizationType::MESH);

    // Set terrain
    // rig.SetTerrainRigid(0.8, 0, 2e7);
    rig.SetTerrainSCM(0.82e6, 0.14e4, 1.0, 0.017e4, 35.0, 1.78e-2);
    
    // Create the terrain loader ----------------------------------------------
    float wx=50, wy=4.0, delta=0.005;
    std::string output_folderpath =   out_dir + "/";
    
    // Flat terrain
    Heightmap GHM_0 = Heightmap::init_flat(wx, wy, delta);
    Heightmap GHM_1 = Heightmap::init_flat(wx, wy, delta);
    Heightmap GHM_2 = Heightmap::init_flat(wx, wy, delta);

    // BMP terrain
    // static Heightmap init_bmp(const std::string& heightmap_file, float wx, float wy, float hMin, float hMax, float delta, std::vector<float> crop_times = {});
    // Heightmap GHM_0 = Heightmap::init_bmp(heightmap_file, wx, wy, terrain_hMin, terrain_hMax, delta);
    // Heightmap GHM_1 = Heightmap::init_bmp(heightmap_file, wx, wy, terrain_hMin, terrain_hMax, delta);
    // Heightmap GHM_2 = Heightmap::init_bmp(heightmap_file, wx, wy, terrain_hMin, terrain_hMax, delta);
    // std::string output_folderpath =   out_dir + "_HMbmp/";

    // std::cout << "HM.get_global_heightmap().sizes()=" << HM.get_global_heightmap().sizes() << std::endl;
    // write_output(out_dir + "_HMbmp/NN_HM_global.txt", HM.get_global_heightmap()[0][0]);
            
    // torch::Tensor Hloc = HM.get_local_heightmap(torch::tensor({0,0,0}), 120, 96);
    // std::cout << "Heightmap size: " << HM.get_global_heightmap().sizes() << std::endl;
    // std::cout << "Hloc size: " << Hloc.sizes() << std::endl;

    // Heightmap GHM_1 = GHM_0.clone();
    // Heightmap GHM_2 = GHM_0.clone();
    
    //make output folder
    if(!std::filesystem::exists(output_folderpath))
        std::filesystem::create_directory(output_folderpath);

    // Load NN model
    // std::string folderpath_normlized = "/home/swang597/Documents/Research/chrono_fork_radu/project_TireTestRig/build/DEMO_OUTPUT/Dataset_4_ML_train_largerHM_231208/";
    // std::string model_path = "/home/swang597/Documents/Research/chrono_fork_radu/project_TireTestRig/Model/Model_py2cpp_2stepNN_iDT1_img96by72_varKsize3333_largerHM_231208/";
    
    std::string folderpath_normlized = "/home/swang597/Documents/Research/chrono_fork_rserban/Project_TireTestRig2Wheels/build_SCM_argVx/DEMO_OUTPUT/Data_raw_var_Vx_Load_initX_Dataset_collectDataWheel2x0.5_train_240608/";
    
    std::string model_path0_wheel1 = "/home/swang597/Documents/Research/Project_heightmap/Code/Pytorch_cpp_model/Model/Model_MyDatasetRig2Wheels_IskgVxVz_F_240610/";
    std::string model_path0_wheel2 = "/home/swang597/Documents/Research/Project_heightmap/Code/Pytorch_cpp_model/Model/Model_MyDatasetRig2Wheels_IskgVxVz_F_240610/";
    std::string model_path_wheel1;
    std::string model_path_wheel2;
    model_path_wheel1 = model_path0_wheel1 + "modelF_wheel1_cpu.pt";
    model_path_wheel2 = model_path0_wheel2 + "modelF_wheel2_cpu.pt";
    TorchModelRunner model_runner_wheel1(model_path_wheel1);
    TorchModelRunner model_runner_wheel2(model_path_wheel2);
    
    std::cout << "Load NN model done." << std::endl;
    // auto terrain = chrono_types::make_shared<TerrainForceLoader>(SCM_filename, wheels);
    // std::shared_ptr<ChBodyAuxRef> wheel_body = wheel->GetSpindle()->GetBody();
    // // std::shared_ptr<chrono::ChBodyAuxRef> wheel_body = std::dynamic_pointer_cast<chrono::ChBodyAuxRef>(wheel->GetSpindle());
    // std::cout << "wheel_body->GetPos()=" << wheel_body->GetPos() << std::endl;
    ChVector<> terrain_initLoc(-terrain_initX + 0.5*wx, -terrain_initY + 0.5*wy, terrain_initH);
    auto terrain_ML = chrono_types::make_shared<TerrainForceLoader>(folderpath_normlized,
                        output_folderpath, wheel, wheel2, model_runner_wheel1, model_runner_wheel2,
                        GHM_0, GHM_1, GHM_2, terrain_initLoc,
                        wheel_radius, wheel_width, heightmap_grid, SCM_ML_switch, dt_HM);

    std::cout << "TerrainForceLoader done." << std::endl;
    sys->Add(terrain_ML);
    std::cout << "Add terrain done." << std::endl;

    if(!flag_varVx){
        rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunction_Const>(fixed_Vx));
    }
    rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunction_Const>(10 * CH_C_RPM_TO_RPS));
    // rig.SetSlipAngleFunction(chrono_types::make_shared<ChFunction_Sine>(0, 0.2, 5 * CH_C_DEG_TO_RAD));
    

    // Scenario: specified longitudinal slip (overrrides other definitons of motion functions)
    ////rig.SetConstantLongitudinalSlip(0.2, 0.1);

    // Initialize the tire test rig
    rig.SetTimeDelay(time_delay);
    // std::cout << "Before Initialize tire test rig. line656" << std::endl;
    
    // Flat terrain
    // rig.Initialize(ChTireTestRig2Wheels::Mode::TEST);
    // rig.Initialize(ChTireTestRig2Wheels::Mode::TEST, terrain_grid); //Shu added
    rig.Initialize(ChTireTestRig2Wheels::Mode::TEST, terrain_grid, terrain_initX, terrain_initH); //Shu added
    
    // BMP terrain
    // rig.Initialize(ChTireTestRig2Wheels::Mode::TEST, terrain_sizeX, terrain_grid); //Shu added
    // rig.Initialize(ChTireTestRig2Wheels::Mode::TEST, heightmap_file, terrain_sizeX, terrain_sizeY,
    //      terrain_hMin, terrain_hMax, terrain_grid, terrain_initX, terrain_initH); //Shu added
    // rig.Initialize(ChTireTestRig2Wheels::Mode::TEST, //heightmap_file, 
    //     terrain_sizeX, terrain_sizeY,
    //      terrain_hMin, terrain_hMax, terrain_grid); //Shu added
    std::cout << "After Initialize tire test rig. line665" << std::endl;
    // Optionally, modify tire visualization (can be done only after initialization)
    if (auto tire_def = std::dynamic_pointer_cast<ChDeformableTire>(tire)) {
        if (tire_def->GetMeshVisualization())
            tire_def->GetMeshVisualization()->SetColorscaleMinMax(0.0, 5.0);  // range for nodal speed norm
    }
    // std::cout << "After tire_def->GetMeshVisualization()->SetColorscaleMinMax(0.0, 5.0); line672" << std::endl;
    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    // std::cout << "line677" << std::endl;
    std::shared_ptr<ChVisualSystem> vis;
    if(flag_vis){
// Create the vehicle run-time visualization interface and the interactive driver
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(sys);
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(1200, 600);
            vis_irr->SetWindowTitle("Tire Test Rig");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector<>(1.0, 2.5, 1.0));
            vis_irr->AddLightDirectional();

            vis_irr->GetActiveCamera()->setFOV(irr::core::PI / 4.5f);
            // vis_irr->GetActiveCamera()->setTarget(
            //     irr::core::vector3df( (wheel->GetState().pos[0],wheel->GetState().pos[1],wheel->GetState().pos[2]) ) );
            

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(sys);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
            vis_vsg->SetWindowSize(1200, 600);
            vis_vsg->SetWindowTitle("Tire Test Rig");
            vis_vsg->AddCamera(ChVector<>(1.0, 2.5, 1.0));
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    }
    

// Create the Blender exporter
#ifdef CHRONO_POSTPROCESS
    postprocess::ChBlender blender_exporter(sys);

    if (blender_output) {
        std::string blender_dir = out_dir + "/blender";
        if (!filesystem::create_directory(filesystem::path(blender_dir))) {
            std::cout << "Error creating directory " << blender_dir << std::endl;
            return 1;
        }

        blender_exporter.SetBlenderUp_is_ChronoZ();
        blender_exporter.SetBasePath(blender_dir);
        blender_exporter.AddAll();
        blender_exporter.SetCamera(ChVector<>(3, 3, 1), ChVector<>(0, 0, 0), 50);
        blender_exporter.ExportScript();
    }
#endif

    // Perform the simulation
    ChFunction_Recorder long_slip;
    ChFunction_Recorder slip_angle;
    ChFunction_Recorder camber_angle;

    double time_offset = 2;

    // auto terrain_SCM = rig.GetTerrain();
    auto terrain_SCM = std::dynamic_pointer_cast<chrono::vehicle::SCMTerrain>(rig.GetTerrain());
    // TerrainForce terrain_force;
    // TerrainForce terrain_force_loc;

    terrain_SCM->SetMLSwitchTime(SCM_ML_switch);

    WheelState wheel_state, wheel_state_wheel2;
    // Open I/O files
    std::ofstream SCM_forces(out_dir + "/SCM_force_wheel1.txt", std::ios::trunc);
    std::ofstream ROVER_states(out_dir + "/ROVER_states_wheel1.txt", std::ios::trunc);
    std::ofstream SCM_forces_wheel2(out_dir + "/SCM_force_wheel2.txt", std::ios::trunc);
    std::ofstream ROVER_states_wheel2(out_dir + "/ROVER_states_wheel2.txt", std::ios::trunc);
    std::string HMfilename_SCM_wheel1, HMfilename_SCM_wheel2;


    // WheelState wheel1_state = wheel->GetState();
    // WheelState wheel2_state = wheel2->GetState();
    // // Open I/O files
    // std::ofstream SCM_forces(out_dir + "/SCM_force_saved.txt", std::ios::trunc);
    // std::ofstream ROVER_states(out_dir + "/ROVER_states_saved.txt", std::ios::trunc);
    
    double time;
    for (int istep = 0; istep < num_steps; istep++) {
        time = sys->GetChTime();

        if (istep % 100 == 0)
            std::cout << "Time: " << time << std::endl;
        
        // if (time > time_offset) {
        // if (time > time_delay) {
        //     long_slip.AddPoint(time, tire->GetLongitudinalSlip());
        //     slip_angle.AddPoint(time, tire->GetSlipAngle() * CH_C_RAD_TO_DEG);
        //     camber_angle.AddPoint(time, tire->GetCamberAngle() * CH_C_RAD_TO_DEG);
        // }
        // std::cout << "line888" << std::endl;
        auto& loc = rig.GetPos();
        // std::cout << "loc: " << loc << std::endl;
        // std::cout << "line883" << std::endl;
        if(flag_vis){
            vis->UpdateCamera(loc + ChVector<>(-3.0, 3.5, 1), loc + ChVector<>(-1, 0, 0));
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }
        // std::cout << "line890" << std::endl;

        if(flag_vis && flag_save_vedio  && istep % save_vedio_fps == 0){ //
            std::string imgName = out_dir + "/img_" + std::to_string(idx_vedio) + ".jpg";
            vis->WriteImageToFile(imgName);
            idx_vedio++;
        }
        // std::cout << "line896" << std::endl;
        ////tools::drawAllContactPoints(vis.get(), 1.0, ContactsDrawMode::CONTACT_NORMALS);
        rig.Advance(step_size);
        // std::cout << "line899" << std::endl;
        auto terrain_force = tire->ReportTireForce(terrain_SCM.get());
        auto terrain_force_wheel2 = tire2->ReportTireForce(terrain_SCM.get());
        
        if(istep % ndt_HM == 0){
            SCM_forces << time << 
            "   " << terrain_force.point <<     ///< global location of the force application point
            "   " << terrain_force.force <<     ///< force vector, epxressed in the global frame
            "   " << terrain_force.moment <<    ///< moment vector, expressed in the global frame
            // "   " << force <<     ///< force vector, epxressed in the global frame
            // "   " << torque <<    ///< moment vector, expressed in the global frame
            std::endl;
            // std::cout << "line911" << std::endl;
            // Save vehicle states
            wheel_state = wheel->GetState();
            ROVER_states << time << 
            "   " << wheel_state.pos << 
            "   "<< wheel_state.rot <<  /// 4 elements
            "   " << wheel_state.lin_vel << 
            "   " << wheel_state.ang_vel << 
            "   " <<  wheel_state.omega << 
            std::endl;

            SCM_forces_wheel2 << time << 
            "   " << terrain_force_wheel2.point <<     ///< global location of the force application point
            "   " << terrain_force_wheel2.force <<     ///< force vector, epxressed in the global frame
            "   " << terrain_force_wheel2.moment <<    ///< moment vector, expressed in the global frame
            // "   " << force <<     ///< force vector, epxressed in the global frame
            // "   " << torque <<    ///< moment vector, expressed in the global frame
            std::endl;

            // Save vehicle states
            wheel_state_wheel2 = wheel2->GetState();
            ROVER_states_wheel2 << time << 
            "   " << wheel_state_wheel2.pos << 
            "   "<< wheel_state_wheel2.rot <<  /// 4 elements
            "   " << wheel_state_wheel2.lin_vel << 
            "   " << wheel_state_wheel2.ang_vel << 
            "   " <<  wheel_state_wheel2.omega << 
            std::endl;

        }

        // Save heightmap
        if(flag_heightmap_save && istep % ndt_HM == 0){
            if(time >= time_delay && wheel_state_wheel2.pos[0] > -0.22 ){
            // SaveHeightmap(terrain.get(), pos_x_pre, pos_y_pre, heightmap_grid, 
            //     heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left, 
            //     heightmap_cutoff_y_right, out_dir, istep - ndt_HM, istep, dt);
            HMfilename_SCM_wheel1 = out_dir + "/hmap_SCM_wheel1_t" + std::to_string(time) + ".txt";
            HMfilename_SCM_wheel2 = out_dir + "/hmap_SCM_wheel2_t" + std::to_string(time) + ".txt";

            SaveHeightmap(terrain_SCM.get(), wheel_state.pos[0], wheel_state.pos[1], heightmap_grid, 
                heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left,
                heightmap_cutoff_y_right, HMfilename_SCM_wheel1, istep, istep, dt);

            SaveHeightmap(terrain_SCM.get(), wheel_state_wheel2.pos[0], wheel_state_wheel2.pos[1], heightmap_grid, 
                heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left,
                heightmap_cutoff_y_right, HMfilename_SCM_wheel2, istep, istep, dt);
                
            // pos_x_pre = wheel_state.pos[0];
            // pos_y_pre = wheel_state.pos[1];
            }
        }


        // ////tools::drawAllContactPoints(vis.get(), 1.0, ContactsDrawMode::CONTACT_NORMALS);
        // // std::cout << "vis done." << std::endl;
        // std::cout << "line957" << std::endl;
        // // Save heightmap
        // if(flag_heightmap_save){
        //     // SaveHeightmap(terrain_SCM.get(), pos_x_pre, pos_y_pre, heightmap_grid, 
        //     //     heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left, 
        //     //     heightmap_cutoff_y_right, out_dir, istep - ndt_HM, istep, dt);
            
        //     SaveHeightmap(terrain_SCM.get(), wheel1_state.pos[0], wheel1_state.pos[1], heightmap_grid, 
        //         heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left,
        //         heightmap_cutoff_y_right, out_dir, istep, istep, dt);
                
            
        // }

        // std::cout << "line900" << std::endl;
        // rig.Advance(step_size);
        // // terrain_SCM->PrintStepStatistics(std::cout); //print SCM step statistics
        // if(istep % 20 == 0){
        //     std::cout << time <<",";
        //     terrain_SCM->PrintAccumulateTimeProfiling(std::cout);
        // }
        

        // // std::cout << "line932" <<    std::endl;
        // auto terrain_force = tire->ReportTireForce(terrain_SCM.get());
        // // std::cout << "line934" << std::endl;
        // // std::cout <<"To txt:" << time << "   " << terrain_force.point << "   " << terrain_force.force << "   " << terrain_force.moment << std::endl;
        // if(istep % ndt_HM == 0){
        //     SCM_forces << time << 
        //     "   " << terrain_force.point <<     ///< global location of the force application point
        //     "   " << terrain_force.force <<     ///< force vector, epxressed in the global frame
        //     "   " << terrain_force.moment <<    ///< moment vector, expressed in the global frame
        //     // "   " << force <<     ///< force vector, epxressed in the global frame
        //     // "   " << torque <<    ///< moment vector, expressed in the global frame
        //     std::endl;
        //     // std::cout << "line944" << std::endl;
        //     // Save vehicle states
        //     wheel1_state = wheel->GetState();
        //     wheel2_state = wheel2->GetState();
        //     ROVER_states << time << 
        //     "   " << wheel1_state.pos << 
        //     "   "<< wheel1_state.rot <<  /// 4 elements
        //     "   " << wheel1_state.lin_vel << 
        //     "   " << wheel1_state.ang_vel << 
        //     "   " <<  wheel1_state.omega << 
        //     std::endl;

        // }
        
        // if(flag_save_vedio && istep % save_vedio_fps == 0){
        //     std::string imgName = output_folderpath + "img_" + std::to_string(idx_vedio) + ".jpg";
        //     vis->WriteImageToFile(imgName);
        //     idx_vedio++;
        // }
        
        
        // std::cout << "Time: " << time << "  "
        //           << "  Long. slip: " << tire->GetLongitudinalSlip() << "  "
        //           << "  Slip angle: " << tire->GetSlipAngle() * CH_C_RAD_TO_DEG << "  "
        //           << "  Camber angle: " << tire->GetCamberAngle() * CH_C_RAD_TO_DEG << std::endl;

#ifdef CHRONO_POSTPROCESS
        if (blender_output)
            blender_exporter.ExportData();
#endif

        ////std::cout << sys.GetChTime() << std::endl;
        ////auto long_slip = tire->GetLongitudinalSlip();
        ////auto slip_angle = tire->GetSlipAngle();
        ////auto camber_angle = tire->GetCamberAngle();
        ////std::cout << "   " << long_slip << " " << slip_angle << " " << camber_angle << std::endl;
        ////auto tforce = rig.ReportTireForce();
        ////auto frc = tforce.force;
        ////auto pnt = tforce.point;
        ////auto trq = tforce.moment;
        ////std::cout << "   " << frc.x() << " " << frc.y() << " " << frc.z() << std::endl;
        ////std::cout << "   " << pnt.x() << " " << pnt.y() << " " << pnt.z() << std::endl;
        ////std::cout << "   " << trq.x() << " " << trq.y() << " " << trq.z() << std::endl;
    }

#ifdef CHRONO_POSTPROCESS
    if (gnuplot_output && sys->GetChTime() > time_offset) {
        postprocess::ChGnuPlot gplot_long_slip(out_dir + "/tmp1.gpl");
        gplot_long_slip.SetGrid();
        gplot_long_slip.SetLabelX("time (s)");
        gplot_long_slip.SetLabelY("Long. slip");
        gplot_long_slip.Plot(long_slip, "", " with lines lt -1 lc rgb'#00AAEE' ");

        postprocess::ChGnuPlot gplot_slip_angle(out_dir + "/tmp2.gpl");
        gplot_slip_angle.SetGrid();
        gplot_slip_angle.SetLabelX("time (s)");
        gplot_slip_angle.SetLabelY("Slip angle");
        gplot_slip_angle.Plot(slip_angle, "", " with lines lt -1 lc rgb'#00AAEE' ");

        postprocess::ChGnuPlot gplot_camber_angle(out_dir + "/tmp3.gpl");
        gplot_camber_angle.SetGrid();
        gplot_camber_angle.SetLabelX("time (s)");
        gplot_camber_angle.SetLabelY("Camber angle");
        gplot_camber_angle.Plot(camber_angle, "", " with lines lt -1 lc rgb'#00AAEE' ");
    }
#endif

    return 0;
}
