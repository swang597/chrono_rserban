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
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

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

// Shu -----------------------------------------------------------------------------
// auto terrain = chrono_types::make_shared<TerrainForceLoader>(folderpath_normlized,output_folderpath, wheel, model_runner_Img, model_runner_F, HM);
// Load container to apply wheel forces and torques from data file
class TerrainForceLoader : public ChLoadContainer {
  public:
    TerrainForceLoader(const std::string& input_folderpath, std::string& output_folderpath, 
    std::shared_ptr<hmmwv::HMMWV_Wheel> wheel, 
        TorchModelRunner model_runner_Img, TorchModelRunner model_runner_F, 
        Heightmap HM, ChVector<> terrain_initLoc)
        : m_output_folderpath(output_folderpath), m_wheel(wheel), m_num_frames(10000), m_crt_frame(0), 
        m_model_runner_Img(model_runner_Img), m_model_runner_F(model_runner_F), 
        m_HM(HM), m_terrain_initLoc(terrain_initLoc) {
        // ChVector<> location(terrain_initX, m_terrain_offset, terrain_initH);
        // Read normalized parameters
        std::string fn_dataPT = input_folderpath;
        std::cout << "fn_dataPT=" << fn_dataPT << std::endl;
        
        m_I_min_max = loadFromTxt(fn_dataPT + "I_min_max_p1.txt");
        m_F_min_max = loadFromTxt(fn_dataPT + "F_min_max_p1.txt");
        m_Vec_min_max = loadFromTxt(fn_dataPT + "Vec_min_max_p1.txt");
        
        std::cout << "Load normalized files, done." << std::endl;
        std::cout << "m_I_min_max=" << m_I_min_max << std::endl;
        std::cout << "m_F_min_max=" << m_F_min_max << std::endl;
        std::cout << "m_Vec_min_max=" << m_Vec_min_max << std::endl;

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
        GetLoadList().clear();
        torch::Tensor inI_1chan, inV_ts, F_ts, dI; 
        torch::Tensor inI_1chan_nm, inV_ts_nm, inI_2chan_nm, dI_nm; 
        std::vector<float> positions;
        m_wheel_state = m_wheel->GetState();
        // std::cout << "line 187" << std::endl;
        F_ts = torch::zeros({1,3}).to(torch::kFloat32);
        // std::cout << "line 189" << std::endl;
        // std::string folder_HM_SCM = "/home/swang597/Documents/Research/chrono_fork_rserban/Project_TireTestRig/build_SCM_ML/DEMO_OUTPUT/SCM_fixW_dt.000500_terrGrid0.005000terrX-23.000000terrH-1.000000normLoad1000.000000/";
        std::string folder_HM_SCM = m_output_folderpath.substr(0, m_output_folderpath.length() - 7) + '/';
        std::string filename_hmap_I00;
        // std::cout << "line 192" << std::endl;
        // if(m_crt_frame % 10 == 0){
            filename_hmap_I00 = folder_HM_SCM + "hmap_Pat" + std::to_string(ChTime) + "_Tat"+ std::to_string(ChTime) +".txt";
            // std::cout << "line 191" << std::endl;
            if(!std::filesystem::exists(filename_hmap_I00)){
                std::cout << "File " << filename_hmap_I00 << " does not exist." << std::endl;
                exit(1);
            }
            // std::cout << "line 196" << std::endl;
            inI_1chan = loadFromTxt(filename_hmap_I00);
            inI_1chan = inI_1chan - m_terrain_initLoc[2];
            // std::cout << "max(inI_1chan)=" << inI_1chan.max().item() << ", min(inI_1chan)=" << inI_1chan.min().item() << std::endl;
            inI_1chan_nm = (inI_1chan - m_I_min_max[0]) / (m_I_min_max[1] - m_I_min_max[0]);
            // inI_1chan_nm = torch::clamp(inI_1chan_nm, 0.0, 1.0);
            inI_1chan_nm = inI_1chan_nm.unsqueeze(0).unsqueeze(0);
            // std::cout << "line 203" << std::endl;
            inV_ts = torch::tensor({m_wheel_state.pos[2] - m_terrain_initLoc[2],
                    m_wheel_state.lin_vel[0], m_wheel_state.lin_vel[2]
                    }).to(torch::kFloat32);
            // std::cout << "line 207" << std::endl;
            inV_ts_nm = inV_ts;
            // std::cout << "line 209" << std::endl;
            for (int i = 0; i < m_Vec_min_max.size(1); i++){
                auto diff = m_Vec_min_max[1][i] - m_Vec_min_max[0][i];
                if(diff.item<double>() < 1e-6){
                    inV_ts_nm[i] = 0.5;
                } else {
                    inV_ts_nm[i] = (inV_ts[i] - m_Vec_min_max[0][i]) / (m_Vec_min_max[1][i] - m_Vec_min_max[0][i]);
                } 
                // std::cout << "m_Vec_min_max["<< i<<"] = " << m_Vec_min_max[i][0].item() << ","<< m_Vec_min_max[i][1].item()  << std::endl;
            }            
            // std::cout << "inV_ts_nm=" << inV_ts_nm << std::endl;
            
            inV_ts_nm = inV_ts_nm.unsqueeze(0);
            // std::cout << "line 220" << std::endl;

            write_output(m_output_folderpath + "xyz_t" + std::to_string(ChTime) + ".txt", torch::tensor({m_wheel_state.pos[0],
                m_wheel_state.pos[1], m_wheel_state.pos[2]}).to(torch::kFloat32).unsqueeze(0));
            write_output(m_output_folderpath + "Vec_t" + std::to_string(ChTime) + ".txt", inV_ts.unsqueeze(0));
            write_output(m_output_folderpath + "Vecnm_t" + std::to_string(ChTime) + ".txt", inV_ts_nm);
            // std::cout << "line 226" << std::endl;
            at::Tensor F_ts_nm = m_model_runner_F.runModel(inI_1chan_nm, inV_ts_nm);
            // std::cout << "line 228" << std::endl;
            F_ts = F_ts_nm * (m_F_min_max[1] - m_F_min_max[0]) + m_F_min_max[0];
            // std::cout << "F_ts = "<< F_ts <<",F_ts_nm="<< F_ts_nm << std::endl;
            
            write_output(m_output_folderpath + "F_t" + std::to_string(ChTime) + ".txt", F_ts);
            write_output(m_output_folderpath + "Fnm_t" + std::to_string(ChTime) + ".txt", F_ts_nm);
        // }
        
        // Hybrid 
        if(m_wheel_state.pos[0] > 0.109){
            // fix Wy, time =  1.5 x= 0.109764; time =  1.495 x= 0.10869
            // if Vz is out of range, enlarge the Fz
            
            // if(inV_ts_nm[0][2].item<double>() < 0.0 || inV_ts_nm[0][2].item<double>() > 1.0){
            //     // Calculate the deviation from the range.
            //     double deviation = (inV_ts_nm[0][2].item<double>() > 1) ? (inV_ts_nm[0][2].item<double>() - 1) : (-inV_ts_nm[0][2].item<double>());
            //     // Apply logarithmic scaling based on the deviation.
            //     // S = std::log10(1 + deviation * 10) + 1;
            //     double S = exp(deviation*10);
            //     std::cout << "Vz="<< inV_ts_nm[0][2].item<double>() <<" is out of range, need to enlarge by S="<< S << std::endl;
            //     if(inV_ts_nm[0][2].item<double>() > 1.0){
            //         F_ts[0][1] = 0;
            //     }else{
            //         F_ts[0][1] = 1500 * S;
            //     }
            // }
            
            if(inV_ts_nm[0][0].item<double>() > 1.0){
                F_ts[0][0] = 0;
                F_ts[0][1] = 0;
                F_ts[0][2] = 0;
            }

            auto force_load = chrono_types::make_shared<ChLoadBodyForce>(m_wheel->GetSpindle(), ChVector<>(F_ts[0][0].item<double>(), 0.0, F_ts[0][1].item<double>()), false,
                                                                        m_wheel_state.pos, false);
            auto torque_load =
                chrono_types::make_shared<ChLoadBodyTorque>(m_wheel->GetSpindle(), ChVector<>(0.0,F_ts[0][2].item<double>(),0.0), false);
            // Add the load to the load container
            Add(force_load);
            Add(torque_load);
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
    std::shared_ptr<hmmwv::HMMWV_Wheel> m_wheel;
    WheelState m_wheel_state;
    // std::vector<std::array<ChVector<>, 4>> m_forces;
    // std::vector<std::array<ChVector<>, 4>> m_torques;
    TorchModelRunner m_model_runner_Img, m_model_runner_F;
    Heightmap m_HM;
    torch::Tensor m_I_min_max, m_dI_min_max, m_F_min_max, m_dF_min_max, m_Vec_min_max;
    std::string m_output_folderpath;
    ChVector<> m_terrain_initLoc;
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
        std::string HMfilename = out_dir + "/hmap_Pat" + std::to_string(istep_pos * dt) + "_Tat" + std::to_string(istep_cur * dt) + ".txt";
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
    
    if (argc != 8) {
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
    double terrain_initY = 0.3; // Default value by ChTireTestRig.cpp
    // generate initial terrain from bmp file
    double terrain_sizeX = 50, terrain_sizeY = 1;
    double terrain_hMin = 0, terrain_hMax = 0.1; 
    std::string heightmap_file = "/home/swang597/Documents/Research/chrono_fork_radu/build/data/vehicle/terrain/height_maps/terrain_heightmap_smooth_horizontal_terrGrid0.005_wx50_wy1.bmp";
    // std::string heightmap_file = "/home/swang597/Documents/Research/build_chrono/data/vehicle/terrain/height_maps/wave_heightmap_5000-100.bmp";

    double time_delay = sqrt(2 * std::abs(terrain_initH) / 9.81) + 0.5; //sqrt(2*abs(H)/9.81) + 0.5
    time_tot += time_delay;

    double heightmap_grid = terrain_grid;
    double heightmap_cutoff_y_left = 0.18; //0.15; // cylinder wheel width 0.128*2
    double heightmap_cutoff_y_right = 0.18; //0.15; 
    double heightmap_cutoff_x_backward = 0.21; //0.24; // cylinder wheel radius 0.208
    double heightmap_cutoff_x_forward = 0.27; //0.18;

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
    const std::string out_dir = GetChronoOutputPath() + "Hybrid_fixW_onlyNNF1Chan_dt" + std::to_string(dt) + "_terrGrid" +
                            std::to_string(terrain_grid) + "terrX" + std::to_string(terrain_initX) + "terrH" + 
                            std::to_string(terrain_initH)+ "normLoad" + std::to_string(normal_load);

    // Create wheel and tire subsystems
    auto wheel = chrono_types::make_shared<hmmwv::HMMWV_Wheel>("Wheel");

    std::shared_ptr<ChTire> tire;
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
    ChTireTestRig rig(wheel, tire, sys);

    ////rig.SetGravitationalAcceleration(0);
    rig.SetNormalLoad(normal_load);

    ////rig.SetCamberAngle(+15 * CH_C_DEG_TO_RAD);

    rig.SetTireStepsize(step_size);
    rig.SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
    rig.SetTireVisualizationType(VisualizationType::MESH);

    // Set terrain
    // rig.SetTerrainRigid(0.8, 0, 2e7);
    rig.SetTerrainSCM(0.82e6, 0.14e4, 1.0, 0.017e4, 35.0, 1.78e-2);
    
    // Create the terrain loader ----------------------------------------------
    float wx=50, wy=4.0, delta=0.005;
    // Heightmap HM = Heightmap::init_flat(wx, wy, delta);
    // static Heightmap init_bmp(const std::string& heightmap_file, float wx, float wy, float hMin, float hMax, float delta, std::vector<float> crop_times = {});
    Heightmap HM = Heightmap::init_bmp(heightmap_file, wx, wy, terrain_hMin, terrain_hMax, delta);

    // torch::Tensor Hloc = HM.get_local_heightmap(torch::tensor({0,0,0}), 120, 96);
    // std::cout << "Heightmap size: " << HM.get_global_heightmap().sizes() << std::endl;
    // std::cout << "Hloc size: " << Hloc.sizes() << std::endl;

    // Define NN parameters
    int img_channel_NN1=1, img_channel_NN2=1;
    int input_vec_colsize=8, output_vec_colsize=3;
    int img_rowsize=96, img_colsize=72;
    int Ksize=5, padding=2, poolSize1=2, poolSize2=3;
    std::string output_folderpath =    out_dir + "_HMbmp/";
    
    //make output folder
    if(!std::filesystem::exists(output_folderpath))
        std::filesystem::create_directory(output_folderpath);


    // Load NN model
    // std::string folderpath_normlized = "/home/swang597/Documents/Research/chrono_fork_radu/project_TireTestRig/build/DEMO_OUTPUT/Dataset_4_ML_train_largerHM_231208/";
    // std::string model_path = "/home/swang597/Documents/Research/chrono_fork_radu/project_TireTestRig/Model/Model_py2cpp_2stepNN_iDT1_img96by72_varKsize3333_largerHM_231208/";
    
    std::string folderpath_normlized = "/home/swang597/Documents/Research/chrono_fork_radu/project_TireTestRig/build_SCM_argVx/DEMO_OUTPUT/Dataset_train_96by72_2phase_varVx_I00V0_I01F0_240227/";
    std::string model_path_Img0 = "/home/swang597/Documents/Research/Project_heightmap/Code/Pytorch_cpp_model/Model/Model_py2cpp_2stepNN_iDT1_img96by72_varKsize3333_largerHM_2phase_240216/";
    std::string model_path = "/home/swang597/Documents/Research/Project_heightmap/Code/Pytorch_cpp_model/Model/Model_2stepNN_iDT1_img96by72_varKsize3333_varVx_I00V0_I01F0_240227/";
    std::string model_path_Img;
    std::string model_path_F;
    model_path_Img = model_path_Img0 + "modelImg_cpu.pt";
    model_path_F =   model_path + "modelF_cpu.pt";
    TorchModelRunner model_runner_Img(model_path_Img);
    TorchModelRunner model_runner_F(model_path_F);
    
    std::cout << "Load NN model done." << std::endl;
    // auto terrain = chrono_types::make_shared<TerrainForceLoader>(SCM_filename, wheels);
    // std::shared_ptr<ChBodyAuxRef> wheel_body = wheel->GetSpindle()->GetBody();
    // // std::shared_ptr<chrono::ChBodyAuxRef> wheel_body = std::dynamic_pointer_cast<chrono::ChBodyAuxRef>(wheel->GetSpindle());
    // std::cout << "wheel_body->GetPos()=" << wheel_body->GetPos() << std::endl;
    ChVector<> terrain_initLoc(terrain_initX + 0.5*wx, terrain_initY + 0.5*wy, terrain_initH);
    auto terrain_ML = chrono_types::make_shared<TerrainForceLoader>(folderpath_normlized,
                        output_folderpath, wheel, model_runner_Img, model_runner_F, HM, terrain_initLoc);
    std::cout << "TerrainForceLoader done." << std::endl;
    sys->Add(terrain_ML);
    std::cout << "Add terrain done." << std::endl;

    // rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunction_Const>(2)); //Vx
    rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunction_Const>(10 * CH_C_RPM_TO_RPS));
    // rig.SetSlipAngleFunction(chrono_types::make_shared<ChFunction_Sine>(0, 0.2, 5 * CH_C_DEG_TO_RAD));
    

    // Scenario: specified longitudinal slip (overrrides other definitons of motion functions)
    ////rig.SetConstantLongitudinalSlip(0.2, 0.1);

    // Initialize the tire test rig
    rig.SetTimeDelay(time_delay);
    // std::cout << "Before Initialize tire test rig. line656" << std::endl;
    // rig.Initialize(ChTireTestRig::Mode::TEST);
    // rig.Initialize(ChTireTestRig::Mode::TEST, terrain_grid); //Shu added
    // rig.Initialize(ChTireTestRig::Mode::TEST, terrain_sizeX, terrain_grid); //Shu added
    rig.Initialize(ChTireTestRig::Mode::TEST, heightmap_file, terrain_sizeX, terrain_sizeY,
         terrain_hMin, terrain_hMax, terrain_grid, terrain_initX, terrain_initH); //Shu added
    // rig.Initialize(ChTireTestRig::Mode::TEST, //heightmap_file, 
    //     terrain_sizeX, terrain_sizeY,
    //      terrain_hMin, terrain_hMax, terrain_grid); //Shu added
    // std::cout << "After Initialize tire test rig. line665" << std::endl;
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

    auto terrain_SCM = rig.GetTerrain();
    // TerrainForce terrain_force;
    // TerrainForce terrain_force_loc;

    WheelState wheel_state;
    // Open I/O files
    std::ofstream SCM_forces(out_dir + "/SCM_force_saved.txt", std::ios::trunc);
    std::ofstream ROVER_states(out_dir + "/ROVER_states_saved.txt", std::ios::trunc);
    
    double time;
    double nstep_contactable = 0;
    double pos_x_pre = wheel_state.pos[0];
    double pos_y_pre = wheel_state.pos[1];
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
        // std::cout << "line891" << std::endl;
        if(flag_vis){
            vis->UpdateCamera(loc + ChVector<>(1.0, 2.5, 0.5), loc + ChVector<>(0, 0.25, -0.25));
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }
        ////tools::drawAllContactPoints(vis.get(), 1.0, ContactsDrawMode::CONTACT_NORMALS);
        // std::cout << "vis done." << std::endl;
        // std::cout << "line957" << std::endl;
        // Save heightmap
        if(flag_heightmap_save){
            // SaveHeightmap(terrain_SCM.get(), pos_x_pre, pos_y_pre, heightmap_grid, 
            //     heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left, 
            //     heightmap_cutoff_y_right, out_dir, istep - ndt_HM, istep, dt);
            
            SaveHeightmap(terrain_SCM.get(), wheel_state.pos[0], wheel_state.pos[1], heightmap_grid, 
                heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left,
                heightmap_cutoff_y_right, out_dir, istep, istep, dt);
                
            pos_x_pre = wheel_state.pos[0];
            pos_y_pre = wheel_state.pos[1];
        }

        // std::cout << "line900" << std::endl;
        rig.Advance(step_size);
        //Shu
        // if(istep < 1000){
        //     rig.Advance(step_size);
        // }else{
        //     // std::cout << "rig.Advance_ML." << std::endl;
        //     rig.Advance_ML(step_size);
        // }
        // rig.Advance(step_size);
        // rig.Advance_ML(step_size);
        // Synchronize subsystems
        // m_terrain->Synchronize(time);
        // m_tire->Synchronize(time, *m_terrain.get());
        // m_spindle_body->Empty_forces_accumulators();
        // m_wheel->Synchronize();
        // Advance state
        // m_terrain->Advance(step);
        // m_tire->Advance(step);
        // m_system->DoStepDynamics(step);
        // std::cout << "rig.Advance done." << std::endl;

        // Save tire forces
        // auto scm = dynamic_cast<SCMTerrain>(terrain.get());
        // auto scm = dynamic_cast<chrono::vehicle::SCMTerrain*>(terrain.get());

        // ChVector<> force;
        // ChVector<> torque;
        // auto tire_body = tire->GetChBody(); // Hypothetical function to get the associated ChBody

        // scm->GetContactForceBody(tire_body, force, torque);
        // // terrain.get()->GetContactForceBody(tire, force, torque);

        // std::cout << "line932" << std::endl;
        auto terrain_force = tire->ReportTireForce(terrain_SCM.get());
        // std::cout << "line934" << std::endl;
        // std::cout <<"To txt:" << time << "   " << terrain_force.point << "   " << terrain_force.force << "   " << terrain_force.moment << std::endl;
        if(istep % ndt_HM == 0){
            SCM_forces << time << 
            "   " << terrain_force.point <<     ///< global location of the force application point
            "   " << terrain_force.force <<     ///< force vector, epxressed in the global frame
            "   " << terrain_force.moment <<    ///< moment vector, expressed in the global frame
            // "   " << force <<     ///< force vector, epxressed in the global frame
            // "   " << torque <<    ///< moment vector, expressed in the global frame
            std::endl;
            // std::cout << "line944" << std::endl;
            // Save vehicle states
            wheel_state = wheel->GetState();
            ROVER_states << time << 
            "   " << wheel_state.pos << 
            "   "<< wheel_state.rot <<  /// 4 elements
            "   " << wheel_state.lin_vel << 
            "   " << wheel_state.ang_vel << 
            "   " <<  wheel_state.omega << 
            std::endl;

        }
        
        if(flag_save_vedio && istep % save_vedio_fps == 0){
            std::string imgName = output_folderpath + "img_" + std::to_string(idx_vedio) + ".jpg";
            vis->WriteImageToFile(imgName);
            idx_vedio++;
        }
        
        // // std::cout << "wheel_state.pos[2]: " << wheel_state.pos[2] << ",wheel_state.lin_vel[2]:"<< wheel_state.lin_vel[2]<< ",nstep_contactable="<< nstep_contactable << std::endl;
        // if (wheel_state.pos[2] < (terrain_initH + 0.2) && std::fabs(wheel_state.lin_vel[2]) < 0.01){
        //     nstep_contactable += 1;
        //     // std::cout << "nstep_contactable: " << nstep_contactable << std::endl;
        // }
        // if(nstep_contactable > 500){
        //     std::cout << "Time: " << time << "Reached nstep_contactable."<< std::endl;
        //     // break;
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
