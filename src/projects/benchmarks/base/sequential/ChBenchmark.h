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

#include "benchmark/benchmark.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_mkl/ChSolverMKL.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

// =============================================================================

/// Base class for a Chrono benchmark test.
class ChBenchmarkTest {
  public:
    virtual ~ChBenchmarkTest() {}

    virtual void ExecuteStep() = 0;
    virtual ChSystem* GetSystem() = 0;

    void Simulate(int num_steps);
    void ResetTimers();

    double m_timer_step;
    double m_timer_setup;
    double m_timer_collision_broad;
    double m_timer_collision_narrow;
    double m_timer_solver;
    double m_timer_update;

  protected:
      ChBenchmarkTest();
};

ChBenchmarkTest::ChBenchmarkTest()
    : m_timer_step(0),
      m_timer_setup(0),
      m_timer_collision_broad(0),
      m_timer_collision_narrow(0),
      m_timer_solver(0),
      m_timer_update(0) {}

void ChBenchmarkTest::Simulate(int num_steps) {
    ResetTimers();
    for (int i = 0; i < num_steps; i++) {
        ExecuteStep();
        m_timer_step += GetSystem()->GetTimerStep();
        m_timer_setup += GetSystem()->GetTimerSetup();
        m_timer_collision_broad += GetSystem()->GetTimerCollisionBroad();
        m_timer_collision_narrow += GetSystem()->GetTimerCollisionNarrow();
        m_timer_solver += GetSystem()->GetTimerSolver();
        m_timer_update += GetSystem()->GetTimerUpdate();
    }
}

void ChBenchmarkTest::ResetTimers() {
    m_timer_step = 0;
    m_timer_setup = 0;
    m_timer_collision_broad = 0;
    m_timer_collision_narrow = 0;
    m_timer_solver = 0;
    m_timer_update = 0;
}

/// Generic benchmark fixture for Chrono tests.
/// The first template parameter is a ChBenchmarkTest.
/// The second template parameter is the initial number of simulation steps (hot start).
template <typename TEST, typename int SKIP>
class ChBenchmarkFixture : public ::benchmark::Fixture {
public:
    ChBenchmarkFixture() {
        m_test = new TEST();
        m_test->Simulate(SKIP);
    }

    ~ChBenchmarkFixture() { delete m_test; }

    void Report(benchmark::State& st) {
        st.counters["Step"] = m_test->m_timer_step * 1e3;
        st.counters["Setup"] = m_test->m_timer_setup * 1e3;
        st.counters["Solve"] = m_test->m_timer_solver * 1e3;
        st.counters["Update"] = m_test->m_timer_update * 1e3;
    }

    TEST* m_test;
};

/// Define and register a test named TEST_NAME using the specified ChBenchmark TEST.
/// An initial SKIP_STEPS integration steps are performed for hot start, after which
/// measurements are conducted for batches of SIM_STEPS integration steps.
/// The test is repeated REPETITIONS number of times, to collect statistics.
#define CH_BM_SIMULATION(TEST_NAME, TEST, SKIP_STEPS, SIM_STEPS, REPETITIONS) \
    using TEST_NAME = ChBenchmarkFixture<TEST, SKIP_STEPS>;             \
    BENCHMARK_DEFINE_F(TEST_NAME, Simulate)(benchmark::State & st) {    \
        while (st.KeepRunning()) {                                      \
            m_test->Simulate(SIM_STEPS);                                \
        }                                                               \
        Report(st);                                                     \
    }                                                                   \
    BENCHMARK_REGISTER_F(TEST_NAME, Simulate)->Unit(benchmark::kMillisecond)->Repetitions(REPETITIONS);

// =============================================================================

template <typename int N>
class Chain : public ChBenchmarkTest {
  public:
    Chain();
    ~Chain();

    ChSystem* GetSystem() override { return m_system; }
    void ExecuteStep() override { m_system->DoStepDynamics(m_step); }

    void SimulateVis(double tend, double step);

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
void Chain<N>::SimulateVis(double tend, double step) {
    float offset = static_cast<float>(N * m_length);

    ChIrrApp application(m_system, L"Pendulum chain", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(0, -offset / 2, offset), irr::core::vector3df(0, -offset / 2, 0));

    application.AssetBindAll();
    application.AssetUpdateAll();

    while (m_system->GetChTime() <= tend && application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        m_system->DoStepDynamics(step);
        application.EndScene();
    }
}

template <typename int N>
Chain<N>::~Chain() {
    delete m_system;
}

// =============================================================================

#define NUM_SKIP_STEPS 2000  // number of steps for hot start
#define NUM_SIM_STEPS 1000   // number of simulation steps for each benchmark

CH_BM_SIMULATION(Chain04, Chain<4>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION(Chain08, Chain<8>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION(Chain16, Chain<16>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION(Chain32, Chain<32>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION(Chain64, Chain<64>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        Chain<4> chain;
        chain.SimulateVis(20, 1e-3);
        return 0;
    }

    ::benchmark::RunSpecifiedBenchmarks();
}
