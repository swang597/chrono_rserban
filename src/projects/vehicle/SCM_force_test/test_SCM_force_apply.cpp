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

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

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

    // Create the run-time visualization interface
    auto vis = CreateVisualization(vis_type, true, sys);

    // Open I/O files
    std::ifstream SCM_forces(SCM_filename);
    std::ofstream ROVER_states(out_dir + "/ROVER_states_applied.txt", std::ios::trunc);

    if (!SCM_forces.is_open()) {
        cout << "Unable to open the " << SCM_filename << " file!" << endl;
        return 1;
    }

    // Simulation loop
    std::string line;
    double force_time;
    ChVector<> abs_force;
    ChVector<> abs_torque;

    for (int istep = 0; istep < num_steps; istep++) {
        if (istep % 100 == 0)
            cout << "Time: " << sys.GetChTime() << endl;

#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->SetCameraTarget(viper->GetChassis()->GetPos());
        vis->Render();
        vis->EndScene();
#endif

        // Read SCM force from file and apply to vehicle wheels
        std::getline(SCM_forces, line);
        std::istringstream iss(line);
        iss >> force_time;
        for (int i = 0; i < 4; i++) {
            iss >> abs_force.x() >> abs_force.y() >> abs_force.z();
            iss >> abs_torque.x() >> abs_torque.y() >> abs_torque.z();

            wheels[i]->Empty_forces_accumulators();
            wheels[i]->Accumulate_force(abs_force, wheels[i]->GetPos(), false);
            wheels[i]->Accumulate_torque(abs_torque, false);
        }

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

    SCM_forces.close();
    ROVER_states.close();

    return 0;
}
