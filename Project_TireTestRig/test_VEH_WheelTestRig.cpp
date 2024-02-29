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
void SaveHeightmap(ChTerrain* terrain, double pos_x, double pos_y, double heightmap_grid, 
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
        // Save the matrix to a file (assuming Eigen's IO)
        // std::string HMfilename = out_dir + "/hmap_Pat" + std::to_string(istep_pos * dt) + "_Tat" + std::to_string(istep_cur * dt) + ".txt";
        // std::ofstream file_HM(HMfilename, std::ios::trunc);

        // // std::ofstream file(heightmapName.str());
        // if (file_HM.is_open()) {
        //     file_HM << hmap_matrix << std::endl;
        //     file_HM.close();
        // } else {
        //     std::cerr << "Unable to open file: " << HMfilename << std::endl;
        // }

}
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
    // generate initial terrain from bmp file
    double terrain_sizeX = 50, terrain_sizeY = 1;
    double terrain_hMin = 0, terrain_hMax = 0.1; 
    std::string heightmap_file = "/home/swang597/Documents/Research/chrono_fork_radu/build/data/vehicle/terrain/height_maps/terrain_heightmap_smooth_horizontal_terrGrid0.005_wx50_wy1.bmp";
    // std::string heightmap_file = "/home/swang597/Documents/Research/build_chrono/data/vehicle/terrain/height_maps/wave_heightmap_5000-100.bmp";

    double time_tot = std::stod(argv[3]); // 3;
    double dt_HM = std::stod(argv[4]); // 1e-4;
    double terrain_initX = std::stod(argv[5]); 
    double terrain_initH = std::stod(argv[6]);
    double normal_load = std::stod(argv[7]); // 1000;

    double time_delay = sqrt(2 * std::abs(terrain_initH) / 9.81) + 0.5; //sqrt(2*abs(H)/9.81) + 0.5
    time_tot += time_delay;

    double heightmap_grid = terrain_grid;
    double heightmap_cutoff_y_left = 0.18; //0.15; // cylinder wheel width 0.128*2
    double heightmap_cutoff_y_right = 0.18; //0.15; 
    double heightmap_cutoff_x_backward = 0.21; //0.24; // cylinder wheel radius 0.208
    double heightmap_cutoff_x_forward = 0.27; //0.18;

    bool flag_heightmap_save = false; //true;
    bool flag_vis = false;

    int num_steps = int(time_tot/dt);
    int ndt_HM = int(dt_HM/dt);
    std::cout << "num_steps: " << num_steps << ", ndt_HM:" << ndt_HM << std::endl;

    // Output directory
    // const std::string out_dir = GetChronoOutputPath() + "TIRE_TEST_RIG_dt" + std::to_string(dt) + "_terrGrid" +
    //                         std::to_string(terrain_grid) + "terrX" + std::to_string(terrain_initX) + "terrH" + 
    //                         std::to_string(terrain_initH)+ "normLoad" + std::to_string(normal_load) + "noHM";
    const std::string out_dir = "/home/swang597/Documents/Research/chrono_fork_radu/project_TireTestRig/build/DEMO_OUTPUT_2phase_240211/" +
                            "TIRE_TEST_RIG_dt" + std::to_string(dt) + "_terrGrid" +
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

    // rig.SetTerrainRigid(0.8, 0, 2e7);
    rig.SetTerrainSCM(0.82e6, 0.14e4, 1.0, 0.017e4, 35.0, 1.78e-2);

    // Set test scenario
    // -----------------

    // Scenario: driven wheel
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunction_Const>(10.0));
    ////rig.Initialize();

    // Scenario: pulled wheel
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunction_Const>(1.0));
    ////rig.Initialize();

    // Scenario: imobilized wheel
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0.0));
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0.0));
    ////rig.Initialize();

    // Scenario: prescribe all motion functions
    //   longitudinal speed: 0.2 m/s
    //   angular speed: 10 RPM
    //   slip angle: sinusoidal +- 5 deg with 5 s period
    rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunction_Const>(2));
    rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunction_Const>(10 * CH_C_RPM_TO_RPS));
    // rig.SetSlipAngleFunction(chrono_types::make_shared<ChFunction_Sine>(0, 0.2, 5 * CH_C_DEG_TO_RAD));
    

    // Scenario: specified longitudinal slip (overrrides other definitons of motion functions)
    ////rig.SetConstantLongitudinalSlip(0.2, 0.1);

    // Initialize the tire test rig
    rig.SetTimeDelay(time_delay);
    // rig.Initialize(ChTireTestRig::Mode::TEST);
    // rig.Initialize(ChTireTestRig::Mode::TEST, terrain_grid); //Shu added
    // rig.Initialize(ChTireTestRig::Mode::TEST, terrain_sizeX, terrain_grid); //Shu added
    rig.Initialize(ChTireTestRig::Mode::TEST, heightmap_file, terrain_sizeX, terrain_sizeY,
         terrain_hMin, terrain_hMax, terrain_grid, terrain_initX, terrain_initH); //Shu added
    // rig.Initialize(ChTireTestRig::Mode::TEST, //heightmap_file, 
    //     terrain_sizeX, terrain_sizeY,
    //      terrain_hMin, terrain_hMax, terrain_grid); //Shu added

    // Optionally, modify tire visualization (can be done only after initialization)
    if (auto tire_def = std::dynamic_pointer_cast<ChDeformableTire>(tire)) {
        if (tire_def->GetMeshVisualization())
            tire_def->GetMeshVisualization()->SetColorscaleMinMax(0.0, 5.0);  // range for nodal speed norm
    }

    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

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

    auto terrain = rig.GetTerrain();
    // TerrainForce terrain_force;
    // TerrainForce terrain_force_loc;

    WheelState wheel_state;
    // Open I/O files
    std::ofstream SCM_forces(out_dir + "/SCM_force_saved.txt", std::ios::trunc);
    std::ofstream ROVER_states(out_dir + "/ROVER_states_saved.txt", std::ios::trunc);
    
    double time;
    // double nstep_contactable = 0;
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

        auto& loc = rig.GetPos();
        if(flag_vis){
            vis->UpdateCamera(loc + ChVector<>(1.0, 2.5, 0.5), loc + ChVector<>(0, 0.25, -0.25));
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }
        ////tools::drawAllContactPoints(vis.get(), 1.0, ContactsDrawMode::CONTACT_NORMALS);
        rig.Advance(step_size);
        

        // Save tire forces
        // auto scm = dynamic_cast<SCMTerrain>(terrain.get());
        // auto scm = dynamic_cast<chrono::vehicle::SCMTerrain*>(terrain.get());

        // ChVector<> force;
        // ChVector<> torque;
        // auto tire_body = tire->GetChBody(); // Hypothetical function to get the associated ChBody

        // scm->GetContactForceBody(tire_body, force, torque);
        // // terrain.get()->GetContactForceBody(tire, force, torque);
        auto terrain_force = tire->ReportTireForce(terrain.get());
        
        // std::cout <<"To txt:" << time << "   " << terrain_force.point << "   " << terrain_force.force << "   " << terrain_force.moment << std::endl;
        if(istep % ndt_HM == 0){
            SCM_forces << time << 
            "   " << terrain_force.point <<     ///< global location of the force application point
            "   " << terrain_force.force <<     ///< force vector, epxressed in the global frame
            "   " << terrain_force.moment <<    ///< moment vector, expressed in the global frame
            // "   " << force <<     ///< force vector, epxressed in the global frame
            // "   " << torque <<    ///< moment vector, expressed in the global frame
            std::endl;

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
        

        // Save heightmap
        if(flag_heightmap_save && istep % ndt_HM == 0){
            SaveHeightmap(terrain.get(), pos_x_pre, pos_y_pre, heightmap_grid, 
                heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left, 
                heightmap_cutoff_y_right, out_dir, istep - ndt_HM, istep, dt);
            
            SaveHeightmap(terrain.get(), wheel_state.pos[0], wheel_state.pos[1], heightmap_grid, 
                heightmap_cutoff_x_backward, heightmap_cutoff_x_forward, heightmap_cutoff_y_left,
                heightmap_cutoff_y_right, out_dir, istep, istep, dt);
                
            pos_x_pre = wheel_state.pos[0];
            pos_y_pre = wheel_state.pos[1];
        }
        // std::cout << "wheel_state.pos[2]: " << wheel_state.pos[2] << ",wheel_state.lin_vel[2]:"<< wheel_state.lin_vel[2]<< ",nstep_contactable="<< nstep_contactable << std::endl;
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
