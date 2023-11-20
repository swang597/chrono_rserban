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

// -----------------------------------------------------------------------------

// Load container to apply wheel forces and torques from data file
class TerrainForceLoader : public ChLoadContainer {
  public:
    TerrainForceLoader(const std::string& input_folderpath, std::string& output_folderpath, std::vector<std::shared_ptr<ChBodyAuxRef>> wheels, 
        std::map<int, TorchModelRunner> model_runner_Img, std::map<int, TorchModelRunner> model_runner_F, 
        Heightmap HM)
        : m_output_folderpath(output_folderpath), m_wheels(wheels), m_num_frames(10000), m_crt_frame(0), 
        m_model_runner_Img(model_runner_Img), m_model_runner_F(model_runner_F), m_HM(HM) {
        
        // Read normalized parameters
        for (int iwheel = 0; iwheel < 4; ++iwheel) {
            std::string fn_dataPT = input_folderpath + "wheel" + std::to_string(iwheel) + "/";
            
            m_I_min_max[iwheel] = loadFromTxt(fn_dataPT + "I_min_max.txt");
            m_dI_min_max[iwheel] = loadFromTxt(fn_dataPT + "dI_min_max.txt");
            m_F_min_max[iwheel] = loadFromTxt(fn_dataPT + "F_min_max.txt");
            m_dF_min_max[iwheel] = loadFromTxt(fn_dataPT + "dF_min_max.txt");
            m_Vec_min_max[iwheel] = loadFromTxt(fn_dataPT + "Vec_min_max.txt");

            // std::cout << "m_I_min_max["<< iwheel <<"]: \n" << m_I_min_max[iwheel] << std::endl;
            // std::cout << "m_dI_min_max[iwheel]: \n" << m_dI_min_max[iwheel] << std::endl;
            // std::cout << "m_F_min_max[iwheel]: \n" << m_F_min_max[iwheel] << std::endl;
            // std::cout << "m_dF_min_max[iwheel]: \n" << m_dF_min_max[iwheel] << std::endl;
            // std::cout << "m_Vec_min_max[iwheel]: \n" << m_Vec_min_max[iwheel] << std::endl;
            
        }
        std::cout << "Load normalized files, done." << std::endl;
        
        if (!filesystem::path(m_output_folderpath).exists()) {
            std::filesystem::create_directory(m_output_folderpath);
        }
    }

    ~TerrainForceLoader() { 
        // m_fstream.close(); 
    }

    // Apply forces to wheel bodies, at the beginning of each timestep.
    virtual void Setup() override {
        // Reset load list
        GetLoadList().clear();
        // std::cout << "line57: Setup m_crt_frame=" << m_crt_frame << std::endl;
        // Generate loads for this time step
        int wheel_idx;
        torch::Tensor inI_1chan, inV_ts, F_ts, dI; 
        torch::Tensor inI_1chan_nm, inV_ts_nm, inI_2chan_nm; 
        std::vector<float> positions;
        for (int i = 0; i < 4; i++) {
            // std::cout << "**********time =" << ChTime <<", i=" << i << std::endl;
            if(i<2) wheel_idx = 10;
            else    wheel_idx = 23;
            positions = {static_cast<float>(m_wheels[i]->GetPos().x()), 
                        static_cast<float>(m_wheels[i]->GetPos().y()), 
                        static_cast<float>(m_wheels[i]->GetPos().z())};
            // std::cout << "positions=" << positions << std::endl;
            torch::Tensor position_tensor = torch::tensor(positions);
            
            inI_1chan = m_HM.get_local_heightmap(position_tensor, 120, 96);
            // std::cout << "I ** inI_1chan min max: (" << inI_1chan.min().item() << ", " << inI_1chan.max().item() <<")"<< std::endl;
            // std::cout << "I ** m_I_min_max["<< i<<"] = " << m_I_min_max[i][0].item() << ","<< m_I_min_max[i][1].item()  << std::endl;

            inI_1chan_nm = (inI_1chan - m_I_min_max[i][0]) / (m_I_min_max[i][1] - m_I_min_max[i][0]);
            // std::cout << "I ** inI_1chan_nm min max: (" << inI_1chan_nm.min().item() << ", " << inI_1chan_nm.max().item() <<")"<< std::endl;
            // std::cout << "inI_1chan.sizes()=" << inI_1chan.sizes() << std::endl;

            inV_ts = torch::tensor({m_wheels[i]->GetPos().z(),
                m_wheels[i]->GetPos_dt().x(), m_wheels[i]->GetPos_dt().y(), m_wheels[i]->GetPos_dt().z(),
                m_wheels[i]->GetWvel_par().x(), m_wheels[i]->GetWvel_par().y(), m_wheels[i]->GetWvel_par().z(),
                time_step}).to(torch::kFloat32);

            // std::cout << "m_Vec_min_max[i][0]=" << m_Vec_min_max[i][0] << ",m_Vec_min_max[i][1]"<< m_Vec_min_max[i][1]<< std::endl;
            // std::cout << "m_Vec_min_max[i][0] size=" << m_Vec_min_max[i][0].sizes() << ",m_Vec_min_max[i][1] size"<< m_Vec_min_max[i][1].sizes()<< std::endl;
            // std::cout << "inV_ts size=" << inV_ts.sizes() << std::endl;
            inV_ts_nm = (inV_ts - m_Vec_min_max[i][0]) / (m_Vec_min_max[i][1] - m_Vec_min_max[i][0]);
            
            inV_ts_nm = inV_ts_nm.unsqueeze(0);
            // std::cout << "V ** inV_ts:    " << inV_ts << std::endl;
            // std::cout << "V ** inV_ts_nm: " << inV_ts_nm << std::endl;
            // std::cout << "V ** [] m_Vec_min_max["<< i<<"] = " << m_Vec_min_max[i] << std::endl;
            // run models
            at::Tensor dI_nm = m_model_runner_Img.at(wheel_idx).runModel(inI_1chan_nm, inV_ts_nm);
            // std::cout << "****** dI_nm.sizes()=" << dI_nm.sizes() << std::endl;
            // std::cout << "dI ** dI_nm min max: " << dI_nm.min().item() << " " << dI_nm.max().item() << std::endl;
            inI_2chan_nm = torch::cat({inI_1chan_nm, dI_nm}, 1);
            // std::cout << "****** inI_2chan_nm.sizes()=" << inI_2chan_nm.sizes() << std::endl;
            // std::cout << "I ** inI_2chan_nm min max: " << inI_2chan_nm.min().item() << " " << inI_2chan_nm.max().item() << std::endl;

            dI = dI_nm * (m_dI_min_max[i][1] - m_dI_min_max[i][0]) + m_dI_min_max[i][0];
            // std::cout << "dI done "<< std::endl;
            write_output(m_output_folderpath + "I_wheel" + std::to_string(i) + "_t" + std::to_string(ChTime) + ".txt", position_tensor.unsqueeze(0));
            write_output(m_output_folderpath + "I_wheel" + std::to_string(i) + "_t" + std::to_string(ChTime) + ".txt", inI_1chan[0][0], true);
            
            write_output(m_output_folderpath + "Inm_wheel" + std::to_string(i) + "_t" + std::to_string(ChTime) + ".txt", position_tensor.unsqueeze(0));
            write_output(m_output_folderpath + "Inm_wheel" + std::to_string(i) + "_t" + std::to_string(ChTime) + ".txt", inI_1chan_nm[0][0], true);
            
            // std::cout << "write I done "<< std::endl;
            write_output(m_output_folderpath + "dI_wheel" + std::to_string(i) + "_t" + std::to_string(ChTime) + ".txt", dI[0][0]);
            write_output(m_output_folderpath + "dInm_wheel" + std::to_string(i) + "_t" + std::to_string(ChTime) + ".txt", dI_nm[0][0]);
            // std::cout << "write dI done "<< std::endl;
            
            write_output(m_output_folderpath + "xyz_wheel" + std::to_string(i) + "_t" + std::to_string(ChTime) + ".txt", torch::tensor({m_wheels[i]->GetPos().x(),
            m_wheels[i]->GetPos().y(), m_wheels[i]->GetPos().z()}).to(torch::kFloat32).unsqueeze(0));
            write_output(m_output_folderpath + "Vec_wheel" + std::to_string(i) + "_t" + std::to_string(ChTime) + ".txt", inV_ts.unsqueeze(0));
            write_output(m_output_folderpath + "Vecnm_wheel" + std::to_string(i) + "_t" + std::to_string(ChTime) + ".txt", inV_ts_nm);
            // std::cout << "write Vec done "<< std::endl;
            // if(m_fp_I)
            // m_fp_I << inI_1chan << std::endl;
            // m_fp_dI << dI << std::endl;
            // m_fp_Vec << inV_ts << std::endl;
            

            inI_1chan = inI_1chan + dI;
            m_HM.update_heightmap(position_tensor, inI_1chan);

            at::Tensor F_ts_nm = m_model_runner_F.at(wheel_idx).runModel(inI_2chan_nm, inV_ts_nm);
            // std::cout << "****** F_ts_nm.sizes()=" << F_ts_nm.sizes() << "F_ts_nm = "<< F_ts_nm << std::endl;

            F_ts = F_ts_nm * (m_F_min_max[i][1] - m_F_min_max[i][0]) + m_F_min_max[i][0];
            // std::cout << "F_ts = "<< F_ts << std::endl;
            
            write_output(m_output_folderpath + "F_wheel" + std::to_string(i) + "_t" + std::to_string(ChTime) + ".txt", F_ts);
            write_output(m_output_folderpath + "Fnm_wheel" + std::to_string(i) + "_t" + std::to_string(ChTime) + ".txt", F_ts_nm);

            // std::cout << "****** F_ts.sizes()=" << F_ts.sizes()  << "F_ts = "<< F_ts << std::endl;
            // std::cout << F_ts[0] << std::endl;
            // std::cout <<"F_ts ChVector"<< ChVector<>(F_ts[0][0].item<double>(), 0.0, F_ts[0][1].item<double>()) << std::endl;
            // std::cout <<"F_ts ChVector"<< ChVector<>(0.0,F_ts[0][2].item<double>(),0.0) << std::endl;

            auto force_load = chrono_types::make_shared<ChLoadBodyForce>(m_wheels[i], ChVector<>(F_ts[0][0].item<double>(), 0.0, F_ts[0][1].item<double>()), false,
                                                                         m_wheels[i]->GetPos(), false);
            auto torque_load =
                chrono_types::make_shared<ChLoadBodyTorque>(m_wheels[i], ChVector<>(0.0,F_ts[0][2].item<double>(),0.0), false);

            // auto force_load = chrono_types::make_shared<ChLoadBodyForce>(m_wheels[i], ChVector<>(100.0, 0.0, 0.0), false,
            //                                                              m_wheels[i]->GetPos(), false);
            // auto torque_load =
            //     chrono_types::make_shared<ChLoadBodyTorque>(m_wheels[i], ChVector<>(0.0,0.0,0.0), false);

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
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_wheels;
    std::vector<std::array<ChVector<>, 4>> m_forces;
    std::vector<std::array<ChVector<>, 4>> m_torques;
    std::map<int, TorchModelRunner> m_model_runner_Img, m_model_runner_F;
    Heightmap m_HM;
    std::unordered_map<int, torch::Tensor> m_I_min_max, m_dI_min_max, m_F_min_max, m_dF_min_max, m_Vec_min_max;
    std::string m_output_folderpath;
    // std::ofstream m_fp_I, m_fp_dI, m_fp_Vec, m_fp_F;
};

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    
    SetChronoDataPath("/home/swang597/Documents/Research/chrono_fork_radu/build/data/");
    
    // Output directory MUST exist
    if (!filesystem::path(out_dir).exists() || !filesystem::path(out_dir).is_directory()) {
        cout << "Data directory does NOT exist!" << endl;
        return 1;
    }
    utils::CSV_writer csv(" ");

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

    // Create the terrain loader ----------------------------------------------
    float wx=50, wy=4.0, delta=0.005;
    Heightmap HM = Heightmap::init_flat(wx, wy, delta);
    // torch::Tensor Hloc = HM.get_local_heightmap(torch::tensor({0,0,0}), 120, 96);
    // std::cout << "Heightmap size: " << HM.get_global_heightmap().sizes() << std::endl;
    // std::cout << "Hloc size: " << Hloc.sizes() << std::endl;

    // Define NN parameters
    int img_channel_NN1=2, img_channel_NN2=1;
    int input_vec_colsize=8, output_vec_colsize=3;
    int img_rowsize=120, img_colsize=96;
    int Ksize=5, padding=2, poolSize1=2, poolSize2=3;
    std::string folderpath_normlized = "/home/swang597/Documents/Research/Project_heightmap/Data/DataPT_dt0.005_dumpfre1_hmapGrid0.005_Wy10_20231023_txt/";
    std::string output_folderpath = folderpath_normlized + "Output_MLChrono_testFx100_231102_dt" +std::to_string(time_step) +"/";
    //make output folder
    if(!std::filesystem::exists(output_folderpath))
        std::filesystem::create_directory(output_folderpath);


    // Load NN model
    std::string model_path = "/home/swang597/Documents/Research/Project_heightmap/Model/Model_py2cpp_2stepNN_iDT5_img120by96_230930/";
    std::map<int, TorchModelRunner> model_runner_Img, model_runner_F;
    model_runner_Img.emplace(10, TorchModelRunner(model_path + "modelImg_wheel10_cpu.pt"));
    model_runner_Img.emplace(23, TorchModelRunner(model_path + "modelImg_wheel23_cpu.pt"));
    model_runner_F.emplace(10, TorchModelRunner(model_path + "modelF_wheel10_cpu.pt"));
    model_runner_F.emplace(23, TorchModelRunner(model_path + "modelF_wheel10_cpu.pt"));
    
    // auto terrain = chrono_types::make_shared<TerrainForceLoader>(SCM_filename, wheels);
    auto terrain = chrono_types::make_shared<TerrainForceLoader>(folderpath_normlized,output_folderpath, wheels, model_runner_Img, model_runner_F, HM);
    sys.Add(terrain);
    // Create the terrain loader, done! ---------------------------------------

    // Create the run-time visualization interface
    auto vis = CreateVisualization(vis_type, true, sys);

    // Open I/O files
    std::ofstream ROVER_states(out_dir + "/ROVER_states_applied.txt", std::ios::trunc);

    // Simulation loop
    for (int istep = 0; istep < num_steps; istep++) {
        if (istep % 100 == 0)
            cout << "Time: " << sys.GetChTime() << endl;

#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->SetCameraTarget(viper->GetChassis()->GetPos());
        vis->Render();
        vis->EndScene();
#endif

        // Advance system dynamics
        sys.DoStepDynamics(time_step);
        viper->Update();

        double time = sys.GetChTime();

        // Save vehicle states
        ROVER_states << time << "   ";
        for (int i = 0; i < 4; i++) {
            ROVER_states << wheels[i]->GetPos() << "   " << wheels[i]->GetPos_dt() << "   " << wheels[i]->GetWvel_par()
                         << "   ";
        }
        ROVER_states << endl;
    }

    ////SCM_forces.close();
    ROVER_states.close();

    return 0;
}
