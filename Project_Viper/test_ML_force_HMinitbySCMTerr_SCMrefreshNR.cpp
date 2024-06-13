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
#include <chrono> // for time profiling
#include <vector>

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;
// -----------------------------------------------------------------------------
torch::Tensor fill_between_edges_vectorized(const torch::Tensor& mask) {
    if (!mask.any().item<bool>()) {
        return mask;
    }

    auto cummax_top_bottom = std::get<0>(mask.cummax(0));
    auto cummax_bottom_top = std::get<0>(mask.flip(0).cummax(0)).flip(0);
    auto filled_mask = cummax_top_bottom & cummax_bottom_top;

    return filled_mask;
}

std::pair<torch::Tensor, torch::Tensor> find_wheel_contact(const torch::Tensor& height_map, torch::Tensor wheel_center, 
                                                           float radius, float width, float grid_size, 
                                                           float threshold, int cx_idx, int cy_idx) {
    int64_t nx = height_map.size(-2);
    int64_t ny = height_map.size(-1);
    float cx = cx_idx * grid_size;
    float cy = cy_idx * grid_size;
    float cz = wheel_center[2].item<float>();
    // std::cout << "line 62" << std::endl;
    torch::Tensor x = torch::arange(0, nx, torch::kFloat32) * grid_size;
    torch::Tensor y = torch::arange(0, ny, torch::kFloat32) * grid_size;
    torch::Tensor xv = x.unsqueeze(1).expand({nx, ny});
    torch::Tensor yv = y.unsqueeze(0).expand({nx, ny});

    torch::Tensor distance_x = torch::abs(xv - cx);
    torch::Tensor distance_y = torch::abs(yv - cy);

    torch::Tensor within_width = distance_y < (width / 2);
    torch::Tensor within_radius = distance_x < radius;

    torch::Tensor wheel_surface_height = cz - torch::sqrt(radius * radius - distance_x.pow(2)).clamp_min(0);
    torch::Tensor nearby_current_height = torch::abs(wheel_surface_height - height_map[0][0]) < threshold;

    torch::Tensor mask = within_width & within_radius & nearby_current_height;
    // std::cout << "line 78" << std::endl;
    torch::Tensor edge_kernel = torch::tensor({{{1, 1, 1},
                                                {1, -8, 1},
                                                {1, 1, 1}}}, torch::kFloat32).unsqueeze(0); //.unsqueeze(0);
    // std::cout << "line 82" << std::endl;
    torch::Tensor mask_float = mask.to(torch::kFloat32).unsqueeze(0).unsqueeze(0);
    // std::cout << "line 84" << std::endl;
    if (!mask.any().item<bool>()) {
        // std::cout << "mask is empty." << std::endl;
        return std::make_pair(mask, mask);
    }
    // std::cout << "edge_kernel size: " << edge_kernel.sizes() << std::endl;
    // std::cout << "mask_float size: " << mask_float.sizes() << std::endl;
    // // std::cout << "mask is not empty." << std::endl;
    torch::Tensor edges = torch::conv2d(mask_float, edge_kernel, {}, 1, 1);
    // std::cout << "line 86" << std::endl;
    torch::Tensor edge_mask = edges.squeeze().to(torch::kBool);
    // std::cout << "line 88" << std::endl;
    return std::make_pair(mask, edge_mask);
}   

torch::Tensor compute_dHM_eff(torch::Tensor height_map, torch::Tensor wheel_center, float wheel_radius, 
                              float wheel_width, float terrain_grid, float threshold_contact, 
                              int cx_idx, int cy_idx) {
    // std::cout << "line 93" << std::endl;
    if (height_map.dim() != 4) {
        height_map = height_map.unsqueeze(0).unsqueeze(0).to(torch::kFloat32);
    }
    // std::cout << "line 106: height_map center height: " << height_map[0][0][cx_idx][cy_idx].item<float>() << std::endl;
    // std::cout << "line 107: wheel_center: " << wheel_center << std::endl;
    // std::cout << "line 97" << std::endl;
    auto mask_result= find_wheel_contact(height_map, wheel_center, wheel_radius, wheel_width, 
                                                terrain_grid, threshold_contact, cx_idx, cy_idx);
    // std::cout << "line 100" << std::endl;
    torch::Tensor mask = mask_result.first;
    torch::Tensor edge_mask = mask_result.second;
    if (!mask.any().item<bool>()) {
        // std::cout << "mask is empty." << std::endl;
        return torch::Tensor();
    }
    // std::cout << "line 118: mask size: " << mask.sizes() << std::endl;
    // std::cout << "line 102" << std::endl;
    auto filled_mask = fill_between_edges_vectorized(mask);
    if (!filled_mask.any().item<bool>()) {
        // std::cout << "filled_mask is empty." << std::endl;
        return torch::Tensor();
    }
    // std::cout << "line 125: filled_mask size: " << filled_mask.sizes() << std::endl;
    int64_t nx = height_map.size(-2);
    int64_t ny = height_map.size(-1);
    // Initialize a 2D tensor with the first column of height_map
    torch::Tensor first_column = height_map[0][0].index({torch::indexing::Slice(), 0});
    // std::cout << "line 130: first_column size: " << first_column.sizes() << std::endl;
    torch::Tensor data_HM_init = first_column.unsqueeze(1).expand({nx, ny});
    // std::cout << "line 131: data_HM_init size: " << data_HM_init.sizes() << std::endl;
    // // Initialize a 2D array (matrix) with zeros
    // Eigen::MatrixXd data_HM_init = Eigen::MatrixXd::Zero(nx, ny);
    // // Fill data_HM_init with the first column of height_map
    // for (int i = 0; i < nx; ++i) {
    //     data_HM_init(i, Eigen::all) = height_map(i, 0);
    // }
    // // Convert data_HM_init to a tensor
    // auto options = torch::TensorOptions().dtype(torch::kFloat32);
    // torch::Tensor data_HM_geo_ts = torch::from_blob(data_HM_init.data(), {1, 1, nx, ny}, options).clone();

    // torch::Tensor data_HM_init = height_map[0][0].clone();
    // torch::Tensor data_HM_geo_ts = height_map.unsqueeze(0).unsqueeze(0).to(torch::kFloat32);
    // std::cout << "line 144: data_HM_geo_ts size: " << data_HM_geo_ts.sizes() << std::endl;
    // Placeholder for apply_wheel_sinkage function
    // apply_wheel_sinkage(data_HM_geo_ts, wheel_center_ts, wheel_radius, wheel_width, terrain_grid, cx_idx, cy_idx);

    torch::Tensor dHM_eff = torch::zeros({nx, ny}, torch::kFloat32);
    torch::Tensor data_HM_geo_np = height_map[0][0];
    // std::cout << "line 136 data_HM_geo_np.sizes="<< data_HM_geo_np.sizes() << std::endl;
    // std::cout << "line 137 data_HM_init.sizes="<< data_HM_init.sizes() << std::endl;
    //count the number of filled pixels
    // int num_filled = filled_mask.sum().item<int>();
    // std::cout << "line 140 num_filled=" << num_filled << std::endl;
    // auto HM_eff = data_HM_init.index({filled_mask}) - data_HM_geo_np.index({filled_mask});
    // // count the number of non-zero elements in HM_eff
    // int num_nonzero = HM_eff.nonzero().size(0);
    // std::cout << "line 144 num_nonzero=" << num_nonzero << std::endl;
    dHM_eff.index_put_({filled_mask}, data_HM_init.index({filled_mask}) - data_HM_geo_np.index({filled_mask}));
    // std::cout << "line 116" << std::endl;
    return dHM_eff;
}
//----------------------------------------------
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
void applyWheelSinkage(torch::Tensor& heightMap, torch::Tensor wheelCenter, 
    int cx_idx, int cy_idx, float radius, float width, float gridSize) {
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

    // Calculate wheel surface height
    auto wheel_surface_height = cz - torch::sqrt(radius * radius - distance_x.pow(2));

    // Mask for points where the wheel surface is below the current height
    auto below_current_height = wheel_surface_height < heightMap.index({0, 0});

    // Combine masks
    auto mask = within_width & within_radius & below_current_height;

    // Update heights
    auto updated_heights = torch::where(mask, wheel_surface_height, heightMap.index({0, 0}));

    // Apply updated heights to heightMap
    heightMap.index_put_({0, 0}, updated_heights);
}
// -----------------------------------------------------------------------------

// Load container to apply wheel forces and torques from data file
class TerrainForceLoaderML : public ChLoadContainer {
  public:
    TerrainForceLoaderML(const std::string& input_folderpath, std::string& output_folderpath, 
        std::vector<std::shared_ptr<ChBodyAuxRef>> wheels, TorchModelRunner model_runner_F,
        Heightmap HM, ChVector<> terrain_initLoc, float wheel_radius, float wheel_width, float heightmap_grid,
        double SCM_ML_switch, double dt_dump)
        : m_output_folderpath(output_folderpath), m_wheels(wheels), m_num_frames(10000), m_crt_frame(0), 
        m_model_runner_F(model_runner_F), m_HM(HM), m_terrain_initLoc(terrain_initLoc), 
        m_wheel_radius(wheel_radius), m_wheel_width(wheel_width), m_heightmap_grid(heightmap_grid),
        m_duration_Img(0), m_duration_F(0), m_duration_all(0), m_SCM_ML_switch(SCM_ML_switch), m_dt_dump(dt_dump){
        // std::cout << "***** line 238" << std::endl;
        std::string fn_dataPT = input_folderpath;
        // std::cout << "fn_dataPT=" << fn_dataPT << std::endl;
        
        m_I_min_max = loadFromTxt(fn_dataPT + "I_min_max_p1.txt");
        m_F_min_max = loadFromTxt(fn_dataPT + "F_min_max_p1.txt");
        m_Vec_min_max = loadFromTxt(fn_dataPT + "Vec_min_max_p1.txt");
        
        // std::cout << "Load normalized files, done." << std::endl;
        // std::cout << "m_I_min_max=" << m_I_min_max << std::endl;
        // std::cout << "m_F_min_max=" << m_F_min_max << std::endl;
        // std::cout << "m_Vec_min_max=" << m_Vec_min_max << std::endl;
        
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
        torch::Tensor inI_1chan, inV_ts, F_ts, dI; 
        torch::Tensor inI_1chan_nm, inV_ts_nm, inI_2chan_nm, outI_nm; 
        torch::Tensor inI_eff;
        std::vector<float> positions;
        std::string filename_hmap_I00;
        int cx_idx = 42, cy_idx = 36;
        float threshold_contact = 0.02;
        

        // std::cout << "line 213" << std::endl;
        for (int iwheel = 0; iwheel < 4; iwheel++) {
            positions = {static_cast<float>(m_wheels[iwheel]->GetPos().x() + m_terrain_initLoc[0]), 
                         static_cast<float>(m_wheels[iwheel]->GetPos().y() + m_terrain_initLoc[1]), 
                         static_cast<float>(m_wheels[iwheel]->GetPos().z() - m_terrain_initLoc[2])};
            // std::cout << "positions=" << positions << std::endl;
            torch::Tensor position_tensor = torch::tensor(positions);
            // std::cout << "line 224, position_tensor: " << position_tensor << std::endl;
            
            // torch::Tensor Hloc = m_HM.get_local_heightmap(torch::tensor({5.0,1.3,1.25}), 96, 72);
            // // std::cout << "Hloc size: " << Hloc.sizes() << std::endl;
            // Hloc = m_HM.get_local_heightmap(position_tensor, 96, 72);
            // std::cout << "Heightmap size: " << m_HM.get_global_heightmap().sizes() << std::endl;
            // // std::cout << "Hloc size: " << Hloc.sizes() << std::endl;

            // if(ChTime < m_SCM_ML_switch){
            if(ChTime < 0){
                filename_hmap_I00 = m_output_folderpath +  "hmap_SCM_wheel" + std::to_string(iwheel) + "_Tat"+ std::to_string(ChTime) +".txt";
                if(!std::filesystem::exists(filename_hmap_I00)){
                    // std::cout << "File " << filename_hmap_I00 << " does not exist." << std::endl;
                    exit(1);
                }
                // std::cout << "line 301" << std::endl;
                inI_1chan = loadFromTxt(filename_hmap_I00);
                inI_1chan = inI_1chan - m_terrain_initLoc[2];
                inI_1chan = inI_1chan.unsqueeze(0).unsqueeze(0);
            }else{
                inI_1chan = m_HM.get_local_heightmap(position_tensor, 96, 72);
                auto start_I = std::chrono::high_resolution_clock::now();
                applyWheelSinkage(inI_1chan, position_tensor, cx_idx, cy_idx, m_wheel_radius, m_wheel_width, m_heightmap_grid);
                auto end_I = std::chrono::high_resolution_clock::now();
            }
            m_HM.update_heightmap(position_tensor, inI_1chan);
                
            
            // std::cout << "inI_1chan.size" << inI_1chan.sizes() << std::endl;
            // std::cout << "I ** inI_1chan min max: (" << inI_1chan.min().item() << ", " << inI_1chan.max().item() <<")"<< std::endl;
            // std::cout << "line 316" << std::endl;
            inI_eff = compute_dHM_eff(inI_1chan, position_tensor, m_wheel_radius, m_wheel_width, 
                                           m_heightmap_grid, threshold_contact, cx_idx, cy_idx);
            if(inI_eff.sizes()[0] == 0){
                // std::cout << "inI_eff is empty." << std::endl;
                F_ts = torch::zeros({1,3}, torch::kFloat32);
                inV_ts = torch::zeros({1,3}, torch::kFloat32);
            }else{
                // std::cout << "inI_eff.size" << inI_eff.sizes() << std::endl;
                inI_eff = inI_eff.unsqueeze(0).unsqueeze(0);
                inI_1chan_nm = (inI_eff - m_I_min_max[0]) / (m_I_min_max[1] - m_I_min_max[0]);
                // std::cout << "line 321" << std::endl;
                // write_output(m_output_folderpath + "NN_Iin_Tat" + std::to_string(ChTime) + ".txt", inI_1chan[0][0]);
                // std::cout << "I ** inI_1chan_nm min max: (" << inI_1chan_nm.min().item() << ", " << inI_1chan_nm.max().item() <<")"<< std::endl;
                // std::cout << "inI_1chan.sizes()=" << inI_1chan.sizes() << std::endl;
                inV_ts = torch::tensor({
                    m_wheels[iwheel]->GetPos_dt().x(), m_wheels[iwheel]->GetPos_dt().z(),
                    m_wheels[iwheel]->GetWvel_par().y(), //wy
                    }).to(torch::kFloat32);
                // std::cout << "line 329" << std::endl;
                // std::cout << "inV_ts=" << inV_ts[0].item()<<","<< inV_ts[1].item()<<","<< inV_ts[2].item() << std::endl;
                inV_ts_nm = inV_ts;
                // std::cout << "line 332" << std::endl;
                for (int v_idx = 0; v_idx < m_Vec_min_max.size(1); v_idx++){
                    auto diff = m_Vec_min_max[1][v_idx] - m_Vec_min_max[0][v_idx];
                    // std::cout << "diff=" << diff.item<double>() << std::endl;
                    if(diff.item<double>() < 1e-6){
                        inV_ts_nm[v_idx] = 0.5;
                    } else {
                        inV_ts_nm[v_idx] = (inV_ts[v_idx] - m_Vec_min_max[0][v_idx]) / (m_Vec_min_max[1][v_idx] - m_Vec_min_max[0][v_idx]);
                        // // std::cout << "inV_ts_nm["<<i<<"]=" << inV_ts_nm[v_idx].item<double>() << << std::endl;
                    } 
                }     
                
                inV_ts_nm = inV_ts_nm.unsqueeze(0);
                // std::cout << "line 345" << std::endl;
                // std::cout << "inV_ts_nm.size="<<inV_ts_nm.sizes() <<":" << inV_ts_nm[0][0].item<double>() << "," << inV_ts_nm[0][1].item<double>() << "," << inV_ts_nm[0][2].item<double>() << std::endl;
                // std::cout << "inI_1chan_nm.sizes()=" << inI_1chan_nm.sizes() << std::endl;
                
                auto start_F = std::chrono::high_resolution_clock::now();
                at::Tensor F_ts_nm = m_model_runner_F.runModel(inI_1chan_nm, inV_ts_nm);
                auto end_F = std::chrono::high_resolution_clock::now();
                // std::cout << "line 349" << std::endl;                

                for(int iforce = 0; iforce < F_ts_nm.size(1); iforce++){
                    F_ts_nm[0][iforce] = F_ts_nm[0][iforce].clamp(0, 1);
                }


                F_ts = F_ts_nm * (m_F_min_max[1] - m_F_min_max[0]) + m_F_min_max[0];
                

                // std::cout << "F_ts=" << F_ts[0][0].item<double>() << "," << F_ts[0][1].item<double>() << "," << F_ts[0][2].item<double>() << std::endl;
                
            
            
                // std::cout << "line 352" << std::endl;
                // std::cout << "line 260" << std::endl;
                // std::cout << "inI_1chan.size" << inI_1chan.sizes() << std::endl;
                // if(m_crt_frame % 20 == 0){
                double remainder = std::fmod(ChTime, m_dt_dump);
                if (remainder < 1e-6 || (m_dt_dump - remainder) < 1e-6) {
                    // std::cout << "ChTime=" << ChTime << std::endl;
                    write_output(m_output_folderpath + "I_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", position_tensor.unsqueeze(0));
                    write_output(m_output_folderpath + "I_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", inI_1chan[0][0], true);
                    write_output(m_output_folderpath + "Ieff_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", inI_eff[0][0]);
                    
                    // write_output(m_output_folderpath + "Inm_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", position_tensor.unsqueeze(0));
                    // write_output(m_output_folderpath + "Inm_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", inI_1chan_nm[0][0], true);
                    
                    write_output(m_output_folderpath + "Vec_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", inV_ts.unsqueeze(0));
                    // write_output(m_output_folderpath + "Vecnm_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", inV_ts_nm);
                    
                    write_output(m_output_folderpath + "F_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", F_ts);
                    // write_output(m_output_folderpath + "Fnm_wheel" + std::to_string(iwheel) + "_t" + std::to_string(ChTime) + ".txt", F_ts_nm);    
                }
                // std::cout << "line 386" << std::endl;
            }

            // double SCM_ML_switch = 4.0;
            if(ChTime >= m_SCM_ML_switch){
                // std::cout << "ML ChTime=" << ChTime << std::endl;
                // if(inV_ts_nm[0][0].item<double>() > 1.0){
                //     F_ts[0][0] = 0;
                //     F_ts[0][1] = 0;
                //     F_ts[0][2] = 0;
                // }
                // std::cout << "line 325" << std::endl;
                // if(iwheel >=2){
                //     F_ts[0][0] = 0; //Fx
                //     F_ts[0][1] = 0; //Fy
                //     F_ts[0][2] = 0; //Fz
                // }
                
                auto force_load = chrono_types::make_shared<ChLoadBodyForce>(m_wheels[iwheel], ChVector<>(F_ts[0][0].item<double>(), 0.0, F_ts[0][1].item<double>()), false,
                                                                            m_wheels[iwheel]->GetPos(), false);
                auto torque_load =
                    chrono_types::make_shared<ChLoadBodyTorque>(m_wheels[iwheel], ChVector<>(0.0,F_ts[0][2].item<double>(),0.0), false);
                // std::cout << "line 330" << std::endl;
                // Add the load to the load container
                Add(force_load);
                Add(torque_load);
                // std::cout << "line 333" << std::endl;
            }
            
        }
        if (m_crt_frame < m_num_frames - 1)
            m_crt_frame++;
        // std::cout << "line 346" << std::endl;
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
    // std::vector<std::array<ChVector<>, 4>> m_forces;
    // std::vector<std::array<ChVector<>, 4>> m_torques;
    // std::map<int, TorchModelRunner> m_model_runner_Img, m_model_runner_F;
    TorchModelRunner m_model_runner_F;
    Heightmap m_HM;
    torch::Tensor m_I_min_max, m_F_min_max, m_dF_min_max, m_Vec_min_max;
    std::string m_output_folderpath;
    ChVector<> m_terrain_initLoc;
    float m_wheel_width, m_wheel_radius, m_heightmap_grid;
    std::chrono::duration<double, std::milli> m_duration_Img, m_duration_F, m_duration_all;
    double m_SCM_ML_switch, m_dt_dump;
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
            // std::cout << "yVec.size()=" << yVec.size() << std::endl;
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
            // std::cout << "yVec.size()=" << yVec.size() << std::endl;
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

    // double time_delay = sqrt(2 * std::abs(terrain_initH) / 9.81) + 0.5; //sqrt(2*abs(H)/9.81) + 0.5
    // time_tot += time_delay;
    int num_steps = int(time_tot/dt);
    // int ndt_dump = int(dt_dump/dt);
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
    double time_ramp_Wy = 2, Wy = 10 * CH_C_RPM_TO_RPS;
    auto driver = chrono_types::make_shared<ViperSpeedDriver>(time_ramp_Wy, Wy);
    ChVector<> viper_init_loc=ChVector<>(-5, 0, -0.5); //(10, 2, 0.25)
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
    Heightmap HM = Heightmap::init_flat(wx, wy, delta);
    std::string output_folderpath = out_dir + "_HMflat_SCM2MLSwitch" + 
            std::to_string(SCM_ML_switch) + "_dt" + std::to_string(dt) + 
            "_density" + std::to_string(chassis_density) + 
            "_SCMrefreshNR_240530/";
    
    
    // Create the bumpy terrain ----------------------------------------------
    // auto terrain_SCM = CreateTerrain(delta, enable_bulldozing, sys, 
    //         heightmap_file, wx, wy, terrain_hMin, terrain_hMax, terrain_initX, terrain_initH);    
    // Heightmap HM = Heightmap::init_bmp(heightmap_file, wx, wy, terrain_hMin, terrain_hMax, delta);
    // std::string output_folderpath = out_dir + "_HMbmp_terrain_hMax"+std::to_string(terrain_hMax)+"_SCM2MLSwitch" + std::to_string(SCM_ML_switch) + "_dt" + std::to_string(dt) + "_240509/";
    
    // std::cout << "output_folderpath=" << output_folderpath << std::endl;
    

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
    
    // torch::Tensor Hloc = HM.get_local_heightmap(torch::tensor({0,0,0}), 96, 72);
    // std::cout << "Heightmap size: " << HM.get_global_heightmap().sizes() << std::endl;
    

    // Define NN parameters
    // int img_channel_NN1=1, img_channel_NN2=1;
    // int input_vec_colsize=8, output_vec_colsize=3;
    // int img_rowsize=96, img_colsize=72;
    // int Ksize=5, padding=2, poolSize1=2, poolSize2=3;
    // std::string folderpath_normlized = "/home/swang597/Documents/Research/Project_heightmap/Data/DataPT_dt0.005_dumpfre1_hmapGrid0.005_Wy10_20231023_txt/";
    // std::string output_folderpath = folderpath_normlized + "Output_MLChrono_testFx100_231102_dt" +std::to_string(time_step) +"/";
    //make output folder
    if(!std::filesystem::exists(output_folderpath))
        std::filesystem::create_directory(output_folderpath);

    std::string folderpath_normlized = "/home/swang597/Documents/Research/chrono_fork_rserban/Project_Viper/build_SCM_refreshNR/DEMO_OUTPUT/Dataset_train_96by72_Viper_fixWy_varDensity_IeffVxVz_F_240516/";
    
    // std::string model_path_Img0 = "/home/swang597/Documents/Research/Project_heightmap/Code/Pytorch_cpp_model/Model/Model_2stepNN_iDT1_img96by72_varKsize3333_varVx_I00V0_I01F0_240229/";
    // Wrong small Norm Load training data model
    // std::string model_path = "/home/swang597/Documents/Research/Project_heightmap/Code/Pytorch_cpp_model/Model/Model_2stepNN_iDT1_img96by72_varKsize3333_varVx_I00V0_I01F0_240227/";
    // Use correct Norm Load training data model [500, 1350, 2000]
    std::string model_path = "/home/swang597/Documents/Research/Project_heightmap/Code/Pytorch_cpp_model/Model/Model_dataFromViperCylindWheel_SCMrefresh2s_IeffVxVzWy_F_240528/";
    // std::string model_path = "/home/swang597/Documents/Research/Project_heightmap/Code/Pytorch_cpp_model/Model/Model_dataFromViperCylindWheel_bump_SCMrefresh2s_IeffVxVzWy_F_240529/";
    // std::string model_path_Img;
    std::string model_path_F;
    // model_path_Img = model_path_Img0 + "modelImg_cpu.pt";
    model_path_F =   model_path + "modelF_cpu.pt";
    // TorchModelRunner model_runner_Img(model_path_Img);
    TorchModelRunner model_runner_F(model_path_F);
    // std::cout << "Load NN model done." << std::endl;

    ChVector<> terrain_initLoc(-terrain_initX + 0.5*wx, -terrain_initY + 0.5*wy, terrain_initH);
    auto terrain_ML = chrono_types::make_shared<TerrainForceLoaderML>(folderpath_normlized,
                        output_folderpath, wheels, model_runner_F, HM, 
                        terrain_initLoc,
                        wheel_radius, wheel_width, heightmap_grid, SCM_ML_switch, dt_dump);
    // std::cout << "TerrainForceLoaderML done." << std::endl;
    sys.Add(terrain_ML);
    // std::cout << "Add terrain done." << std::endl;

    // Load NN model
    // std::string model_path = "/home/swang597/Documents/Research/Project_heightmap/Model/Model_py2cpp_2stepNN_iDT5_img120by96_230930/";
    // std::map<int, TorchModelRunner> model_runner_Img, model_runner_F;
    // model_runner_Img.emplace(10, TorchModelRunner(model_path + "modelImg_wheel10_cpu.pt"));
    // model_runner_Img.emplace(23, TorchModelRunner(model_path + "modelImg_wheel23_cpu.pt"));
    // model_runner_F.emplace(10, TorchModelRunner(model_path + "modelF_wheel10_cpu.pt"));
    // model_runner_F.emplace(23, TorchModelRunner(model_path + "modelF_wheel10_cpu.pt"));
    
    // auto terrain = chrono_types::make_shared<TerrainForceLoaderML>(SCM_filename, wheels);
    // auto terrain = chrono_types::make_shared<TerrainForceLoaderML>(folderpath_normlized,
    //                     output_folderpath, wheels, model_runner_Img, model_runner_F, HM);
    // sys.Add(terrain);
    // Create the terrain loader, done! ---------------------------------------

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

        if(sys.GetChTime() < SCM_ML_switch){
            // save heightmap 
            for(int iwheel = 0; iwheel < 4; iwheel++) {
                ChVector<> wheel_pos = wheels[iwheel]->GetPos();
                HMfilename_SCM = output_folderpath + "hmap_SCM_wheel" + std::to_string(iwheel) + "_Tat" + std::to_string(istep * dt) + ".txt";
                SaveHeightmap(terrain_SCM.get(), wheel_pos[0], wheel_pos[1], heightmap_grid, 
                    heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left,
                    heightmap_cutoff_y_right, HMfilename_SCM, istep, istep, dt);
                // std::cout << "Save heightmap:" << HMfilename_SCM << std::endl;
                // std::cout << "wheel pos_xyz: " << wheel_pos[0] << ", " << wheel_pos[1] << std::endl;
                // double dz = wheel_pos[2] - terrain_SCM.get()->GetHeight(ChVector<>(wheel_pos[0], wheel_pos[1], 10));
                // std::cout << "dz: " << dz << std::endl;
                // std::cout << "wheel pos_z: " << wheel_pos[2] << std::endl;
                // std::cout << "sinkage: " << terrain_SCM.get()->GetHeight(ChVector<>(wheel_pos[0], wheel_pos[1], 10)) << std::endl;
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
        double time = sys.GetChTime();
        
        
        // std::cout << "line504" << std::endl;
        // Save SCM terrain forces that were applied during the *previous* step
        // if ((istep+1) % 20 == 0){
        double remainder = std::fmod(time, dt_dump);
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
