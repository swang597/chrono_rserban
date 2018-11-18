// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_BENCHMARK_H
#define CH_BENCHMARK_H

#include "benchmark/benchmark.h"
#include "chrono/physics/Chsystem.h"

namespace chrono {
namespace utils {

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
    double m_timer_collision;
    double m_timer_collision_broad;
    double m_timer_collision_narrow;
    double m_timer_solver;
    double m_timer_update;

  protected:
    ChBenchmarkTest();
};

inline ChBenchmarkTest::ChBenchmarkTest()
    : m_timer_step(0),
      m_timer_setup(0),
      m_timer_collision(0),
      m_timer_collision_broad(0),
      m_timer_collision_narrow(0),
      m_timer_solver(0),
      m_timer_update(0) {}

inline void ChBenchmarkTest::Simulate(int num_steps) {
    ResetTimers();
    for (int i = 0; i < num_steps; i++) {
        ExecuteStep();
        m_timer_step += GetSystem()->GetTimerStep();
        m_timer_setup += GetSystem()->GetTimerSetup();
        m_timer_collision += GetSystem()->GetTimerCollision();
        m_timer_collision_broad += GetSystem()->GetTimerCollisionBroad();
        m_timer_collision_narrow += GetSystem()->GetTimerCollisionNarrow();
        m_timer_solver += GetSystem()->GetTimerSolver();
        m_timer_update += GetSystem()->GetTimerUpdate();
    }
}

inline void ChBenchmarkTest::ResetTimers() {
    m_timer_step = 0;
    m_timer_setup = 0;
    m_timer_collision = 0;
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
        st.counters["Collision"] = m_test->m_timer_collision * 1e3;
        st.counters["Broad"] = m_test->m_timer_collision_broad * 1e3;
        st.counters["Narrow"] = m_test->m_timer_collision_narrow * 1e3;
    }

    TEST* m_test;
};

/// Define and register a test named TEST_NAME using the specified ChBenchmark TEST.
/// An initial SKIP_STEPS integration steps are performed for hot start, after which
/// measurements are conducted for batches of SIM_STEPS integration steps.
/// The test is repeated REPETITIONS number of times, to collect statistics.
#define CH_BM_SIMULATION(TEST_NAME, TEST, SKIP_STEPS, SIM_STEPS, REPETITIONS) \
    using TEST_NAME = chrono::utils::ChBenchmarkFixture<TEST, SKIP_STEPS>;    \
    BENCHMARK_DEFINE_F(TEST_NAME, Simulate)(benchmark::State & st) {          \
        while (st.KeepRunning()) {                                            \
            m_test->Simulate(SIM_STEPS);                                      \
        }                                                                     \
        Report(st);                                                           \
    }                                                                         \
    BENCHMARK_REGISTER_F(TEST_NAME, Simulate)->Unit(benchmark::kMillisecond)->Repetitions(REPETITIONS);

}  // end namespace utils

}  // end namespace chrono

#endif