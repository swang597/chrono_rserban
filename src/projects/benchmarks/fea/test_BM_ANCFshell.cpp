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
// Benchmark test for ANCF shell elements
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChBenchmark.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChIrrApp.h"
#endif

#ifdef CHRONO_PARDISO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

using namespace chrono;
using namespace chrono::fea;

template <int N>
class ANCFshell : public utils::ChBenchmarkTest {
  public:
    virtual ~ANCFshell() { delete m_system; }

    ChSystem* GetSystem() override { return m_system; }
    void ExecuteStep() override { m_system->DoStepDynamics(1e-4); }

    void SimulateVis();

  protected:
    ANCFshell(ChSolver::Type solver_type);

    ChSystemSMC* m_system;
};

template <int N>
class ANCFshell_MINRES : public ANCFshell<N> {
  public:
    ANCFshell_MINRES() : ANCFshell<N>(ChSolver::Type::MINRES) {}
};

template <int N>
class ANCFshell_MKL : public ANCFshell<N> {
  public:
    ANCFshell_MKL() : ANCFshell<N>(ChSolver::Type::CUSTOM) {}
};

template <int N>
ANCFshell<N>::ANCFshell(ChSolver::Type solver_type) {
    m_system = new ChSystemSMC();
    m_system->Set_G_acc(ChVector<>(0, -9.8, 0));

    // Set solver parameters
#ifndef CHRONO_PARDISO_MKL
    if (solver_type == ChSolver::Type::CUSTOM)
        solver_type = ChSolver::Type::MINRES;
#endif

    switch (solver_type) {
        case ChSolver::Type::MINRES: {
            auto minres_solver = chrono_types::make_shared<ChSolverMINRES>();
            minres_solver->SetMaxIterations(100);
            minres_solver->SetTolerance(1e-14);
            minres_solver->EnableDiagonalPreconditioner(true);
            m_system->SetSolver(minres_solver);
            break;
        }
        case ChSolver::Type::CUSTOM: {
#ifdef CHRONO_PARDISO_MKL
            auto mkl_solver = chrono_types::make_shared<ChSolverMKL>();
            mkl_solver->LockSparsityPattern(true);
            mkl_solver->SetVerbose(false);
            m_system->SetSolver(mkl_solver);
#endif
            break;
        }
    }

    // Set up integrator
    m_system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(100);
    integrator->SetAbsTolerances(1e-5);
    integrator->SetMode(ChTimestepperHHT::ACCELERATION);
    integrator->SetScaling(true);
    integrator->SetVerbose(false);

    // Mesh properties
    double length = 1;
    double width = 0.1;
    double thickness = 0.01;

    double rho = 500;
    ChVector<> E(2.1e7, 2.1e7, 2.1e7);
    ChVector<> nu(0.3, 0.3, 0.3);
    ChVector<> G(8.0769231e6, 8.0769231e6, 8.0769231e6);
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu, G);

    // Create mesh nodes and elements
    auto mesh = chrono_types::make_shared<ChMesh>();
    m_system->Add(mesh);

    auto vis_surf = chrono_types::make_shared<ChVisualizationFEAmesh>(*mesh);
    vis_surf->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    vis_surf->SetWireframe(true);
    vis_surf->SetDrawInUndeformedReference(true);
    mesh->AddAsset(vis_surf);

    auto vis_node = chrono_types::make_shared<ChVisualizationFEAmesh>(*mesh);
    vis_node->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    vis_node->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    vis_node->SetSymbolsThickness(0.004);
    mesh->AddAsset(vis_node);

    int n_nodes = 2 * (1 + N);
    double dx = length / N;
    ChVector<> dir(0, 1, 0);

    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, 0, -width / 2), dir);
    auto nodeB = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, 0, +width / 2), dir);
    nodeA->SetFixed(true);
    nodeB->SetFixed(true);
    mesh->AddNode(nodeA);
    mesh->AddNode(nodeB);

    for (int i = 1; i < N; i++) {
        auto nodeC = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(i * dx, 0, -width / 2), dir);
        auto nodeD = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(i * dx, 0, +width / 2), dir);
        mesh->AddNode(nodeC);
        mesh->AddNode(nodeD);

        auto element = chrono_types::make_shared<ChElementShellANCF>();
        element->SetNodes(nodeA, nodeB, nodeD, nodeC);
        element->SetDimensions(dx, width);
        element->AddLayer(thickness, 0 * CH_C_DEG_TO_RAD, mat);
        element->SetAlphaDamp(0.0);
        element->SetGravityOn(false);
        mesh->AddElement(element);

        nodeA = nodeC;
        nodeB = nodeD;
    }
}

template <int N>
void ANCFshell<N>::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    irrlicht::ChIrrApp application(m_system, L"ANCF shells", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    application.AddLogo();
    application.AddSkyBox();
    application.AddTypicalLights();
    application.AddCamera(irr::core::vector3df(-0.2f, 0.2f, 0.2f), irr::core::vector3df(0, 0, 0));

    application.AssetBindAll();
    application.AssetUpdateAll();

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        irrlicht::ChIrrTools::drawSegment(application.GetVideoDriver(), ChVector<>(0), ChVector<>(1, 0, 0),
                                          irr::video::SColor(255, 255, 0, 0));
        irrlicht::ChIrrTools::drawSegment(application.GetVideoDriver(), ChVector<>(0), ChVector<>(0, 1, 0),
                                          irr::video::SColor(255, 0, 255, 0));
        irrlicht::ChIrrTools::drawSegment(application.GetVideoDriver(), ChVector<>(0), ChVector<>(0, 0, 1),
                                          irr::video::SColor(255, 0, 0, 255));
        ExecuteStep();
        application.EndScene();
    }
#endif
}

// =============================================================================

#define NUM_SKIP_STEPS 10  // number of steps for hot start
#define NUM_SIM_STEPS 100  // number of simulation steps for each benchmark

CH_BM_SIMULATION_LOOP(ANCFshell08_MINRES, ANCFshell_MINRES<8>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell16_MINRES, ANCFshell_MINRES<16>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell32_MINRES, ANCFshell_MINRES<32>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell64_MINRES, ANCFshell_MINRES<64>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);

#ifdef CHRONO_PARDISO_MKL
CH_BM_SIMULATION_LOOP(ANCFshell08_MKL, ANCFshell_MKL<8>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell16_MKL, ANCFshell_MKL<16>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell32_MKL, ANCFshell_MKL<32>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell64_MKL, ANCFshell_MKL<64>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
#endif

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_IRRLICHT
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        ANCFshell_MINRES<16> test;
        test.SimulateVis();
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
