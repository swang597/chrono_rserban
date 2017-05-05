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
// Chrono test program for rolling friction
//
// The global reference frame has Y up.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono/physics/ChSystemNSC.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChIrrApp.h"
#endif

using namespace chrono;

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
    double collision_envelope = .05 * radius;

    // -----------------
    // Create the system
    // -----------------

    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, -10, 0));

    // Set solver settings
    system.SetMaxItersSolverSpeed(100);
    system.SetMaxPenetrationRecoverySpeed(contact_recovery_speed);
    system.SetTol(tolerance);
    system.SetTolForce(tolerance);
    system.SetSolverType(ChSolver::Type::APGD);

    // ----------
    // Add bodies
    // ----------

    auto container = std::shared_ptr<ChBody>(system.NewBody());
    system.Add(container);
    container->SetPos(ChVector<>(0, 0, 0));
    container->SetBodyFixed(true);
    container->SetIdentifier(-1);

    container->GetMaterialSurfaceNSC()->SetFriction(sliding_friction);
    container->GetMaterialSurfaceNSC()->SetRollingFriction(rolling_friction);

    container->SetCollide(true);
    container->GetCollisionModel()->SetEnvelope(collision_envelope);
    container->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(container.get(), ChVector<>(20, .5, 20), ChVector<>(0, -.5, 0));
    container->GetCollisionModel()->BuildModel();

    container->AddAsset(std::make_shared<ChColorAsset>(ChColor(0.4f, 0.4f, 0.2f)));

    auto ball = std::shared_ptr<ChBody>(system.NewBody());
    ChVector<> pos = ChVector<>(0, radius, 0);
    ChVector<> vel = ChVector<>(initial_linspeed, 0, 0);
    ChVector<> wvel = ChVector<>(0, 0, -initial_angspeed);
    ball->SetMass(mass);
    ball->SetPos(pos);
    ball->SetPos_dt(vel);
    ball->SetWvel_par(wvel);
    ball->SetInertiaXX(ChVector<>(inertia));

    ball->GetMaterialSurfaceNSC()->SetFriction(sliding_friction);
    ball->GetMaterialSurfaceNSC()->SetRollingFriction(rolling_friction);

    ball->SetCollide(true);
    ball->GetCollisionModel()->SetEnvelope(collision_envelope);
    ball->GetCollisionModel()->ClearModel();
    utils::AddSphereGeometry(ball.get(), radius);
    ball->GetCollisionModel()->BuildModel();

    ball->AddAsset(std::make_shared<ChColorAsset>(ChColor(0.2f, 0.3f, 0.4f)));

    system.AddBody(ball);

#ifdef CHRONO_IRRLICHT
    // -------------------------------
    // Create the visualization window
    // -------------------------------

    irrlicht::ChIrrApp application(&system, L"Rolling test", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    irrlicht::ChIrrWizard::add_typical_Logo(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Lights(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(10, 10, -20));

    application.AssetBindAll();
    application.AssetUpdateAll();
#endif

    // ---------------
    // Simulate system
    // ---------------

    double time_end = 20.0;
    double time_out = 2.5;
    bool output = false;

    while (system.GetChTime() < time_end) {
        system.DoStepDynamics(time_step);

        auto pos = ball->GetPos();
        printf("T: %f  Pos: %f %f %f\n", system.GetChTime(), pos.x(), pos.y(), pos.z());

        // if (!output && system.GetChTime() >= time_out) {
        //    for (int i = 1; i <= 10; i++) {
        //        auto pos = system.Get_bodylist()->at(i)->GetPos();
        //        std::cout << pos.x() << std::endl;
        //    }
        //    output = true;
        //}

#ifdef CHRONO_IRRLICHT
        if (application.GetDevice()->run()) {
            application.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            application.DrawAll();
            application.EndScene();
        } else {
            return 1;
        }
#endif
    }

    return 0;
}
