// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Test for disabling mesh constraints
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_mkl/ChSolverMKL.h"

#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

using namespace chrono;

// Define FIX_NODES to fix mesh nodes.
// Otherwise, use constraints.
////#define FIX_NODES

int main(int argc, char* argv[]) {
    // Settings
    bool use_MKL = false;
    bool use_HHT = false;

    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(-1.0, 0.0, 0.0));
    ////sys.Set_G_acc(ChVector<>(0.0, -1.0, 0.0));
    ////sys.Set_G_acc(ChVector<>(0.0, 0.0, -1.0));
    ////sys.Set_G_acc(ChVector<>(0.0, 0.0, 0.0));

    // Ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    sys.Add(ground);

    auto mesh = chrono_types::make_shared<fea::ChMesh>();

    auto section = chrono_types::make_shared<fea::ChBeamSectionCable>();
    section->SetDiameter(0.002);
    section->SetYoungModulus(1e6);
    section->SetDensity(2000);
    section->SetBeamRaleyghDamping(0.8);

    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> base_nodes;
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> tip_nodes;

    for (int j = 0; j < 1; ++j) {
        fea::ChBuilderCableANCF builder;
        builder.BuildBeam(mesh,                            //
                          section,                         //
                          4,                               //
                          ChVector<>(0.0, 0.0, -0.1 * j),  // base location
                          ChVector<>(0.0, 0.1, -0.1 * j)   // tip location
        );

        ////builder.GetLastBeamNodes().back()->SetForce(ChVector<>(-0.1, 0, 0));

        base_nodes.push_back(builder.GetLastBeamNodes().front());
        tip_nodes.push_back(builder.GetLastBeamNodes().back());
    }

#ifdef FIX_NODES

    for (auto node : base_nodes)
        node->SetFixed(true);

    for (auto node : tip_nodes)
        node->SetFixed(true);

#else

    std::vector<std::shared_ptr<ChLinkBase>> base_cnstr;
    std::vector<std::shared_ptr<ChLinkBase>> tip_cnstr;

    for (auto node : base_nodes) {
        auto pos_cnstr = chrono_types::make_shared<fea::ChLinkPointFrame>();
        pos_cnstr->Initialize(node, ground);
        sys.Add(pos_cnstr);

        auto dir_cnstr = chrono_types::make_shared<fea::ChLinkDirFrame>();
        dir_cnstr->Initialize(node, ground);
        dir_cnstr->SetDirectionInAbsoluteCoords(node->D);
        sys.Add(dir_cnstr);

        base_cnstr.push_back(pos_cnstr);
        base_cnstr.push_back(dir_cnstr);
    }

    for (auto node : tip_nodes) {
        auto pos_cnstr = chrono_types::make_shared<fea::ChLinkPointFrame>();
        pos_cnstr->Initialize(node, ground);
        sys.Add(pos_cnstr);

        auto dir_cnstr = chrono_types::make_shared<fea::ChLinkDirFrame>();
        dir_cnstr->Initialize(node, ground);
        dir_cnstr->SetDirectionInAbsoluteCoords(node->D);
        sys.Add(dir_cnstr);

        tip_cnstr.push_back(pos_cnstr);
        tip_cnstr.push_back(dir_cnstr);
    }

#endif

    sys.Add(mesh);

    auto mvisA = chrono_types::make_shared<fea::ChVisualizationFEAmesh>(*mesh);
    mvisA->SetFEMdataType(fea::ChVisualizationFEAmesh::E_PLOT_ANCF_BEAM_BD);
    mvisA->SetColorscaleMinMax(-02, 20);
    mvisA->SetSmoothFaces(true);
    mvisA->SetWireframe(false);
    mesh->AddAsset(mvisA);

    auto mvisB = chrono_types::make_shared<fea::ChVisualizationFEAmesh>(*mesh);
    mvisB->SetFEMglyphType(fea::ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisB->SetFEMdataType(fea::ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisB->SetSymbolsThickness(0.004);
    mvisB->SetSymbolsScale(0.002);
    mvisB->SetZbufferHide(false);
    mesh->AddAsset(mvisB);

    // Run-time visualization
    irrlicht::ChIrrApp application(&sys, L"Cables FEM", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    application.AddLogo();
    application.AddSkyBox();
    application.AddTypicalLights();
    application.AddCamera(irr::core::vector3df(0.0f, 0.05f, 0.2f));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Solver settings
    if (use_MKL) {
        auto mkl_solver = chrono_types::make_shared<ChSolverMKL>();
        mkl_solver->LockSparsityPattern(true);
        sys.SetSolver(mkl_solver);
    } else {
        auto minres_solver = chrono_types::make_shared<ChSolverMINRES>();
        minres_solver->SetMaxIterations(400);
        minres_solver->SetTolerance(1e-12);
        minres_solver->EnableWarmStart(true);
        sys.SetSolver(minres_solver);
    }

    // Integrator settings
    // -------------------
    if (use_HHT) {
        sys.SetTimestepperType(ChTimestepper::Type::HHT);
        auto stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
        stepper->SetAlpha(-0.1);
        stepper->SetMaxiters(100);
        stepper->SetAbsTolerances(1e-10, 1e-10);
        stepper->SetMode(ChTimestepperHHT::ACCELERATION);
        stepper->SetScaling(true);
        stepper->SetVerbose(false);
    } else {
        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
        ////sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    }

    // Simulation loop
    application.SetTimestep(0.01);

    bool disabled = false;
    while (application.GetDevice()->run()) {
        if (!disabled && sys.GetChTime() > 2) {
#ifdef FIX_NODES
            for (auto node : tip_nodes)
                node->SetFixed(false);
#else
            for (auto cnstr : tip_cnstr)
                cnstr->SetDisabled(true);
#endif
            disabled = true;
        }

        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
