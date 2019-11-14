#include <assert.h>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_mkl/ChSolverMKL.h"

#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

using namespace chrono;
using namespace chrono::irrlicht;

using std::cout;
using std::endl;

int main(int argc, char* argv[]) {
    // Settings
    bool use_MKL = true;
    bool use_HHT = true;

    // -------------
    // Create system
    // -------------
    ChSystemSMC mphysicalSystem;
    mphysicalSystem.Set_G_acc(ChVector<>(0.01, 0.0, 0.0));

    // Ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    mphysicalSystem.Add(ground);

    // FEA mesh
    auto msection_cable = chrono_types::make_shared<fea::ChBeamSectionCable>();
    msection_cable->SetDiameter(0.002);
    msection_cable->SetYoungModulus(5e3);
    msection_cable->SetDensity(2000);
    msection_cable->SetBeamRaleyghDamping(0.2);

    int N_x = 2;
    int N_y = 2;
    int num_elements = 4;
    double delta = 0.1;

    auto my_mesh = chrono_types::make_shared<fea::ChMesh>();
    fea::ChBuilderBeamANCF builder;
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> base_nodes;
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> tip_nodes;

    for (int j = 0; j < N_y; j++) {
        for (int i = 0; i < N_x; i++) {
            double loc_x = i * delta;
            double loc_y = j * delta;
            builder.BuildBeam(
                my_mesh,                        // the mesh where to put the created nodes and elements
                msection_cable,                 // the ChBeamSectionCable to use for the ChElementBeamANCF elements
                num_elements,                   // the number of ChElementBeamANCF to create
                ChVector<>(loc_x, loc_y, 0.0),  // the 'A' point in space (beginning of beam)
                ChVector<>(loc_x, loc_y, 0.1)   // the 'B' point in space (end of beam) _1D_elementsNodes_mesh,
            );

            base_nodes.push_back(builder.GetLastBeamNodes().front());
            tip_nodes.push_back(builder.GetLastBeamNodes().back());
        }
    }

    for (auto node : base_nodes) {
        node->SetFixed(true);

        ////auto pos_const = chrono_types::make_shared<fea::ChLinkPointFrame>();
        ////pos_const->Initialize(node, ground);
        ////mphysicalSystem.Add(pos_const);

        ////auto dir_const = chrono_types::make_shared<fea::ChLinkDirFrame>();
        ////dir_const->Initialize(node, ground);
        ////dir_const->SetDirectionInAbsoluteCoords(node->D);
        ////mphysicalSystem.Add(dir_const);
    }

    mphysicalSystem.Add(my_mesh);

    // Mesh visualization
    auto mvisualizebeamA = chrono_types::make_shared<fea::ChVisualizationFEAmesh>(*my_mesh);
    mvisualizebeamA->SetFEMdataType(fea::ChVisualizationFEAmesh::E_PLOT_ANCF_BEAM_BD);
    mvisualizebeamA->SetColorscaleMinMax(-20, 20);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddAsset(mvisualizebeamA);

    // ----------------------
    // Run-time visualization
    // ----------------------
    ChIrrApp application(&mphysicalSystem, L"Test ANCF Cables", irr::core::dimension2d<irr::u32>(1600, 1200), false,
                         true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights(irr::core::vector3df(30.f, -100.f, 30.f), irr::core::vector3df(30.f, -80.f, -30.f));
    application.AddTypicalCamera(irr::core::vector3df(0.05f, -0.3f, 0.05f), irr::core::vector3df(0.05f, 0.0f, 0.0f));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // ---------------
    // Solver settings
    // ---------------
    if (use_MKL) {
        auto mkl_solver = chrono_types::make_shared<ChSolverMKL>();
        mkl_solver->LockSparsityPattern(true);
        mphysicalSystem.SetSolver(mkl_solver);
    } else {
        mphysicalSystem.SetSolverType(ChSolver::Type::MINRES);
        mphysicalSystem.SetSolverWarmStarting(true);
        mphysicalSystem.SetTolForce(1e-10);
        mphysicalSystem.SetMaxItersSolverSpeed(10000);
        mphysicalSystem.SetMaxItersSolverStab(10000);
    }

    // -------------------
    // Integrator settings
    // -------------------
    if (use_HHT) {
        mphysicalSystem.SetTimestepperType(ChTimestepper::Type::HHT);
        auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(mphysicalSystem.GetTimestepper());
        mystepper->SetAlpha(-0.1);
        mystepper->SetMaxiters(100);
        mystepper->SetAbsTolerances(1e-10, 1e-10);
        mystepper->SetMode(ChTimestepperHHT::ACCELERATION);
        mystepper->SetScaling(true);
        mystepper->SetVerbose(false);
    } else {
        mphysicalSystem.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
        ////mphysicalSystem.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    }

    // ------------
    // Setup output
    // ------------
    std::string out_dir = "../TEST_ancf_cable";
    if (filesystem::path(out_dir).exists()) {
        cout << "Output directory already exists" << endl;
    } else if (filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Create directory = " << filesystem::path(out_dir).make_absolute() << endl;
    } else {
        cout << "Error creating output directory" << endl;
        return 1;
    }

    // ---------------
    // Simulation loop
    // ---------------
    utils::CSV_writer csv(" ");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(18);

    int step_number = 0;
    double dt = 0.01;
    application.SetTimestep(dt);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        ChIrrTools::drawAllCOGs(mphysicalSystem, application.GetVideoDriver(), 0.05);
        application.DoStep();
        application.EndScene();
        step_number++;

        if (step_number == 1) {
            for (auto node : my_mesh->GetNodes()) {
                cout << node->GetIndex() << " " << node->NodeGetOffset_x() << " " << node->NodeGetOffset_w() << endl;
            }
        }

        csv << mphysicalSystem.GetChTime();
        for (auto node : tip_nodes) {
            csv << node->GetPos();
        }
        csv << endl;
        csv.write_to_file(out_dir + "/results.dat");
    }

    return 0;
}
