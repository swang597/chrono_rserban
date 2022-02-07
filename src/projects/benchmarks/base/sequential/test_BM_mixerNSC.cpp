// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
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
// Benchmark test for contact simulation using NSC contact.
//
// =============================================================================

#include "chrono/utils/ChBenchmark.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

// =============================================================================

template <int N>
class MixerTestNSC : public utils::ChBenchmarkTest {
  public:
    MixerTestNSC();
    ~MixerTestNSC() { delete m_system; }

    ChSystem* GetSystem() override { return m_system; }
    void ExecuteStep() override { m_system->DoStepDynamics(m_step); }

    void SimulateVis();

  private:
    ChSystemNSC* m_system;
    double m_step;
};

template <int N>
MixerTestNSC<N>::MixerTestNSC() : m_system(new ChSystemNSC()), m_step(0.02) {
    auto sphere_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    sphere_mat->SetFriction(0.2f);
    auto other_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    for (int bi = 0; bi < N; bi++) {
        auto sphereBody = chrono_types::make_shared<ChBodyEasySphere>(1.1, 1000, true, true, sphere_mat);
        sphereBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        m_system->Add(sphereBody);

        auto boxBody = chrono_types::make_shared<ChBodyEasyBox>(1.5, 1.5, 1.5, 100, true, true, other_mat);
        boxBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        m_system->Add(boxBody);

        auto mcylBody = chrono_types::make_shared<ChBodyEasyCylinder>(0.75, 0.5, 100, true, true, other_mat);
        mcylBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        m_system->Add(mcylBody);
    }

    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto mixer_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mixer_mat->SetFriction(0.4f);

    auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(20, 1, 20, 1000, true, true, floor_mat);
    floorBody->SetPos(ChVector<>(0, -5, 0));
    floorBody->SetBodyFixed(true);
    m_system->Add(floorBody);

    auto wallBody1 = chrono_types::make_shared<ChBodyEasyBox>(1, 10, 20.99, 1000, true, true, floor_mat);
    wallBody1->SetPos(ChVector<>(-10, 0, 0));
    wallBody1->SetBodyFixed(true);
    m_system->Add(wallBody1);

    auto wallBody2 = chrono_types::make_shared<ChBodyEasyBox>(1, 10, 20.99, 1000, true, true, floor_mat);
    wallBody2->SetPos(ChVector<>(10, 0, 0));
    wallBody2->SetBodyFixed(true);
    m_system->Add(wallBody2);

    auto wallBody3 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 10, 1, 1000, true, true, floor_mat);
    wallBody3->SetPos(ChVector<>(0, 0, -10));
    wallBody3->SetBodyFixed(true);
    m_system->Add(wallBody3);

    auto wallBody4 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 10, 1, 1000, true, true, floor_mat);
    wallBody4->SetPos(ChVector<>(0, 0, 10));
    wallBody4->SetBodyFixed(true);
    m_system->Add(wallBody4);

    auto rotatingBody = chrono_types::make_shared<ChBodyEasyBox>(10, 5, 1, 4000, true, true, mixer_mat);
    rotatingBody->SetPos(ChVector<>(0, -1.6, 0));
    m_system->Add(rotatingBody);

    auto my_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_motor->Initialize(rotatingBody, floorBody, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
    auto mfun = chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 2.0);
    my_motor->SetSpeedFunction(mfun);
    m_system->AddLink(my_motor);
}

template <int N>
void MixerTestNSC<N>::SimulateVis() {
    ChIrrApp application(m_system, L"Rigid contacts", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    application.AddLogo();
    application.AddSkyBox();
    application.AddTypicalLights();
    application.AddCamera(irr::core::vector3df(0, 14, -20));

    application.AssetBindAll();
    application.AssetUpdateAll();

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        ExecuteStep();
        application.EndScene();
    }
}

// =============================================================================

#define NUM_SKIP_STEPS 2000  // number of steps for hot start
#define NUM_SIM_STEPS 1000   // number of simulation steps for each benchmark

CH_BM_SIMULATION_LOOP(MixerNSC032, MixerTestNSC<32>,  NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(MixerNSC064, MixerTestNSC<64>,  NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        MixerTestNSC<64> test;
        test.SimulateVis();
        return 0;
    }

    ::benchmark::RunSpecifiedBenchmarks();
}
