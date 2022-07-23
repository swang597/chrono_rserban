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
// Chrono test program for:
// - collision shapes with different material properties within same model
// - rolling friction
//
// The global reference frame has Y up.
//
// =============================================================================

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

// --------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // ----------------
    // Parameters
    // ----------------
    double radius = 0.5;
    double density = 1000;
    double mass = density * (4.0 / 3.0) * CH_C_PI * pow(radius, 3);
    double inertia = (2.0 / 5.0) * mass * pow(radius, 2);
    double initial_angspeed = 10;
    double initial_linspeed = initial_angspeed * radius;

    float sliding_friction = 0.1f;
    float rolling_friction = 0.1f;

    double time_step = 1e-3;

    double tolerance = 0;
    double contact_recovery_speed = 1e8;
    double collision_envelope = 0.02 * radius;

    // -----------------
    // Create the system
    // -----------------

    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, -10, 0));

    // Set solver settings
    system.SetSolverType(ChSolver::Type::APGD);
    system.SetSolverMaxIterations(100);
    system.SetSolverTolerance(tolerance);
    system.SetSolverForceTolerance(tolerance);
    system.SetMaxPenetrationRecoverySpeed(contact_recovery_speed);

    // ----------
    // Add bodies
    // ----------

    auto container = std::shared_ptr<ChBody>(system.NewBody());
    system.Add(container);
    container->SetPos(ChVector<>(0, 0, 0));
    container->SetBodyFixed(true);
    container->SetIdentifier(-1);

    container->SetCollide(true);
    container->GetCollisionModel()->SetEnvelope(collision_envelope);
    container->GetCollisionModel()->ClearModel();
    for (int i = 0; i < 5; i++) {
        auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        material->SetFriction(0.15f * (i + 1));
        material->SetRollingFriction(0.05f * (i + 1) / 5.0f);
        utils::AddBoxGeometry(container.get(), material, ChVector<>(20, 0.5, 1.8 * radius),
                              ChVector<>(0, -0.5, i * 4.0 * radius));
    }
    container->GetCollisionModel()->BuildModel();

    container->GetVisualShape(0)->SetColor(ChColor(0.4f, 0.4f, 0.2f));

    auto ball_material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ball_material->SetFriction(1.0);
    ball_material->SetRollingFriction(1.0);

    std::vector<std::shared_ptr<ChBody>> balls;
    for (int i = 0; i < 5; i++) {
        auto ball = std::shared_ptr<ChBody>(system.NewBody());
        ball->SetMass(mass);
        ball->SetPos(ChVector<>(-9, radius, 4.0 * i * radius));
        ball->SetPos_dt(ChVector<>(initial_linspeed, 0, 0));
        ball->SetWvel_par(ChVector<>(0, 0, -initial_angspeed));
        ball->SetInertiaXX(ChVector<>(inertia));

        ball->SetCollide(true);
        ball->GetCollisionModel()->SetEnvelope(collision_envelope);
        ball->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(ball.get(), ball_material, radius);
        ball->GetCollisionModel()->BuildModel();

        ball->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));

        system.AddBody(ball);
        balls.push_back(ball);
    }

    // -------------------------------
    // Create the visualization window
    // -------------------------------

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Rolling test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(10, 10, -20));
    vis->AttachSystem(&system);

    // ---------------
    // Simulate system
    // ---------------

    while (vis->Run()) {
        std::cout << "time: " << system.GetChTime();
        for (auto ball : balls)
            std::cout << "   " << ball->GetPos().y();
        std::cout << std::endl;

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        system.DoStepDynamics(time_step);
    }

    return 0;
}
