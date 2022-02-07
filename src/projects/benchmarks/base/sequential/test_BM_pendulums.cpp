// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// Benchmark test for pendulum chain.
//
// =============================================================================

#include "chrono/utils/ChBenchmark.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/solver/ChSolverBB.h"

#include "chrono_mkl/ChSolverMKL.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

// =============================================================================

template <int N>
class ChainTest : public utils::ChBenchmarkTest {
  public:
    ChainTest();
    ~ChainTest() { delete m_system; }

    ChSystem* GetSystem() override { return m_system; }
    void ExecuteStep() override { m_system->DoStepDynamics(m_step); }

    void SimulateVis();

  private:
    ChSystem* m_system;
    double m_length;
    double m_step;
};

template <int N>
ChainTest<N>::ChainTest() : m_length(0.25), m_step(1e-3) {
    ChTimestepper::Type integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    ChSolver::Type solver_type = ChSolver::Type::PSOR;
    ChContactMethod contact_method = ChContactMethod::NSC;

    // Create system
    m_system = (contact_method == ChContactMethod::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                        : static_cast<ChSystem*>(new ChSystemSMC);
    m_system->Set_G_acc(ChVector<>(0, -1, 0));

    // Set solver parameters
    switch (solver_type) {
        case ChSolver::Type::CUSTOM: {
            auto mkl_solver = chrono_types::make_shared<ChSolverMKL>();
            mkl_solver->LockSparsityPattern(true);
            mkl_solver->SetVerbose(false);
            m_system->SetSolver(mkl_solver);
            break;
        }
        case ChSolver::Type::PSOR: {
            auto psor_solver = chrono_types::make_shared<ChSolverPSOR>();
            psor_solver->SetMaxIterations(50);
            psor_solver->SetTolerance(1e-12);
            psor_solver->SetOmega(0.8);
            psor_solver->SetSharpnessLambda(1.0);
            m_system->SetSolver(psor_solver);
            m_system->SetMaxPenetrationRecoverySpeed(1.5);
            m_system->SetMinBounceSpeed(2.0);
            break;
        }
        case ChSolver::Type::BARZILAIBORWEIN: {
            auto bb_solver = chrono_types::make_shared<ChSolverPSOR>();
            bb_solver->SetMaxIterations(50);
            bb_solver->SetTolerance(1e-12);
            bb_solver->SetOmega(0.8);
            bb_solver->SetSharpnessLambda(1.0);
            m_system->SetSolver(bb_solver);
            m_system->SetMaxPenetrationRecoverySpeed(1.5);
            m_system->SetMinBounceSpeed(2.0);
            break;
        }
    }

    // Set integrator parameters
    switch (integrator_type) {
        case ChTimestepper::Type::HHT: {
            m_system->SetTimestepperType(ChTimestepper::Type::HHT);
            auto integrator = std::static_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper());
            integrator->SetAlpha(-0.2);
            integrator->SetMaxiters(50);
            integrator->SetAbsTolerances(1e-4, 1e2);
            integrator->SetMode(ChTimestepperHHT::ACCELERATION);
            integrator->SetStepControl(false);
            integrator->SetModifiedNewton(false);
            integrator->SetScaling(true);
            integrator->SetVerbose(false);
            break;
        }
    }

    // Create ground
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetBodyFixed(true);
    m_system->AddBody(ground);

    // Create pendulums
    double width = 0.025;
    double density = 500;
    for (int ib = 0; ib < N; ib++) {
        auto prev = m_system->Get_bodylist().back();

        auto pend = chrono_types::make_shared<ChBodyEasyBox>(m_length, width, width, density, true, false);
        pend->SetPos(ChVector<>((ib + 0.5) * m_length, 0, 0));
        pend->AddAsset(chrono_types::make_shared<ChColorAsset>(0.5f * (ib % 2), 0.0f, 0.5f * (ib % 2 - 1)));
        m_system->AddBody(pend);

        auto rev = chrono_types::make_shared<ChLinkLockRevolute>();
        rev->Initialize(pend, prev, ChCoordsys<>(ChVector<>(ib * m_length, 0, 0)));
        m_system->AddLink(rev);
    }
}

template <int N>
void ChainTest<N>::SimulateVis() {
    float offset = static_cast<float>(N * m_length);

    ChIrrApp application(m_system, L"Pendulum chain", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    application.AddLogo();
    application.AddSkyBox();
    application.AddTypicalLights();
    application.AddCamera(irr::core::vector3df(0, -offset / 2, offset), irr::core::vector3df(0, -offset / 2, 0));

    application.AssetBindAll();
    application.AssetUpdateAll();

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        m_system->DoStepDynamics(m_step);
        application.EndScene();
    }
}

// =============================================================================

#define NUM_SKIP_STEPS 2000  // number of steps for hot start
#define NUM_SIM_STEPS 1000   // number of simulation steps for each benchmark

CH_BM_SIMULATION_LOOP(Chain04, ChainTest<4>,  NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(Chain08, ChainTest<8>,  NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(Chain16, ChainTest<16>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(Chain32, ChainTest<32>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(Chain64, ChainTest<64>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        ChainTest<4> test;
        test.SimulateVis();
        return 0;
    }

    ::benchmark::RunSpecifiedBenchmarks();
}
