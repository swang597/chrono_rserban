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

#include "ChBenchmark.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_mkl/ChSolverMKL.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

// =============================================================================

template <typename int N>
class Chain : public utils::ChBenchmarkTest {
  public:
    Chain();
    ~Chain() { delete m_system; }

    ChSystem* GetSystem() override { return m_system; }
    void ExecuteStep() override { m_system->DoStepDynamics(m_step); }

    void SimulateVis();

  private:
    ChSystem* m_system;
    static const double m_length;
    static const double m_step;
};

template <typename int N>
const double Chain<N>::m_length = 0.25;

template <typename int N>
const double Chain<N>::m_step = 1e-3;

template <typename int N>
Chain<N>::Chain() {
    ChTimestepper::Type integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    ChSolver::Type solver_type = ChSolver::Type::SOR;
    ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC;

    // Create system
    m_system = (contact_method == ChMaterialSurface::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                          : static_cast<ChSystem*>(new ChSystemSMC);
    m_system->Set_G_acc(ChVector<>(0, -1, 0));

    // Set solver parameters
    switch (solver_type) {
        case ChSolver::Type::CUSTOM: {
            auto mkl_solver = std::make_shared<ChSolverMKL<>>();
            mkl_solver->SetSparsityPatternLock(true);
            mkl_solver->SetVerbose(false);
            m_system->SetSolver(mkl_solver);
            break;
        }
        case ChSolver::Type::SOR: {
            m_system->SetSolverType(ChSolver::Type::SOR);
            m_system->SetMaxItersSolverSpeed(50);
            m_system->SetMaxItersSolverStab(50);
            m_system->SetTol(0);
            m_system->SetMaxPenetrationRecoverySpeed(1.5);
            m_system->SetMinBounceSpeed(2.0);
            m_system->SetSolverOverrelaxationParam(0.8);
            m_system->SetSolverSharpnessParam(1.0);
            break;
        }
        case ChSolver::Type::BARZILAIBORWEIN: {
            m_system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
            m_system->SetMaxItersSolverSpeed(50);
            m_system->SetMaxItersSolverStab(50);
            m_system->SetTol(0);
            m_system->SetMaxPenetrationRecoverySpeed(1.5);
            m_system->SetMinBounceSpeed(2.0);
            m_system->SetSolverOverrelaxationParam(0.8);
            m_system->SetSolverSharpnessParam(1.0);
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
    auto ground = std::make_shared<ChBody>(contact_method);
    ground->SetBodyFixed(true);
    m_system->AddBody(ground);

    // Create pendulums
    double width = 0.025;
    double density = 500;
    for (int ib = 0; ib < N; ib++) {
        auto prev = m_system->Get_bodylist().back();

        auto pend = std::make_shared<ChBodyEasyBox>(m_length, width, width, density, false, true, contact_method);
        pend->SetPos(ChVector<>((ib + 0.5) * m_length, 0, 0));
        pend->AddAsset(std::make_shared<ChColorAsset>(0.5f * (ib % 2), 0.0f, 0.5f * (ib % 2 - 1)));
        m_system->AddBody(pend);

        auto rev = std::make_shared<ChLinkLockRevolute>();
        rev->Initialize(pend, prev, ChCoordsys<>(ChVector<>(ib * m_length, 0, 0)));
        m_system->AddLink(rev);
    }
}

template <typename int N>
void Chain<N>::SimulateVis() {
    float offset = static_cast<float>(N * m_length);

    ChIrrApp application(m_system, L"Pendulum chain", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(0, -offset / 2, offset), irr::core::vector3df(0, -offset / 2, 0));

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

CH_BM_SIMULATION(Chain04, Chain<4>,  NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION(Chain08, Chain<8>,  NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION(Chain16, Chain<16>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION(Chain32, Chain<32>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION(Chain64, Chain<64>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        Chain<4> chain;
        chain.SimulateVis();
        return 0;
    }

    ::benchmark::RunSpecifiedBenchmarks();
}
