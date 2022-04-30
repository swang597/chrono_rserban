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
// Chrono test for using collision families
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::collision;

bool ball_collision = false;

int main(int argc, char* argv[]) {
    // Create system
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, -1, 0));

    // Shared contact material (default properties)
    auto contact_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Create the falling balls
    double mass = 1;
    double radius = 0.15;
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

    // Lower ball
    auto ball_lower = std::shared_ptr<ChBody>(sys.NewBody());
    ball_lower->SetIdentifier(1);
    ball_lower->SetMass(mass);
    ball_lower->SetInertiaXX(inertia);
    ball_lower->SetPos(ChVector<>(0, 2, 0));
    ball_lower->SetBodyFixed(false);
    ball_lower->SetCollide(true);

    ball_lower->GetVisualShape(0)->SetColor(ChColor(1.0f, 0.0f, 0.0f));

    ball_lower->GetCollisionModel()->ClearModel();
    ball_lower->GetCollisionModel()->SetFamily(3);
    if (!ball_collision) {
        ball_lower->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);
    }
    utils::AddSphereGeometry(ball_lower.get(), contact_mat, radius);
    ball_lower->GetCollisionModel()->BuildModel();

    sys.AddBody(ball_lower);

    // Upper ball
    auto ball_upper = std::shared_ptr<ChBody>(sys.NewBody());
    ball_upper->SetIdentifier(2);
    ball_upper->SetMass(mass);
    ball_upper->SetInertiaXX(inertia);
    ball_upper->SetPos(ChVector<>(radius, 3, 0));
    ball_upper->SetPos_dt(ChVector<>(0, -1, 0));
    ball_upper->SetBodyFixed(false);
    ball_upper->SetCollide(true);

    ball_upper->GetVisualShape(0)->SetColor(ChColor(0.0f, 1.0f, 0.0f));

    ball_upper->GetCollisionModel()->ClearModel();
    ball_upper->GetCollisionModel()->SetFamily(3);
    if (!ball_collision) {
        ball_upper->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);
    }
    utils::AddSphereGeometry(ball_upper.get(), contact_mat, radius);
    ball_upper->GetCollisionModel()->BuildModel();

    sys.AddBody(ball_upper);

    // Plate
    auto plate = std::shared_ptr<ChBody>(sys.NewBody());
    plate->SetIdentifier(0);
    plate->SetPos(ChVector<>(0, 0, 0));
    plate->SetBodyFixed(true);
    plate->SetCollide(true);

    plate->GetVisualShape(0)->SetColor(ChColor(0.3f, 0.3f, 0.5f));

    plate->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(plate.get(), contact_mat, ChVector<>(10 * radius, radius, 10 * radius));
    plate->GetCollisionModel()->BuildModel();

    sys.AddBody(plate);

    // Create the visualization window
    auto vis = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Collision family");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0, 2, -4));
    sys.SetVisualSystem(vis);

    for (const auto& body : sys.Get_bodylist()) {
        std::cout << "Body " << body->GetIdentifier() << "  family: " << body->GetCollisionModel()->GetFamily()
                  << "  family mask: " << body->GetCollisionModel()->GetFamilyMask() << std::endl;
    }

    // Run simulation for specified time
    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        vis->EndScene();
        sys.DoStepDynamics(1e-3);
    }

    return 0;
}
