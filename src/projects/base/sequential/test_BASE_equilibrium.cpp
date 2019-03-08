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
// Simple example demonstrating equilibrium analyis.
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include <cstdio>

#include "chrono/assets/ChPointPointDrawing.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, 0, 0));

    // Create bodies.
    auto ground = std::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);

    auto body1 = std::make_shared<ChBodyEasyCylinder>(0.1, 1.0, 1000, false, true);
    body1->SetPos(ChVector<>(0.5, 0, 0));
    body1->SetRot(Q_ROTATE_Y_TO_X);
    body1->AddAsset(std::make_shared<ChColorAsset>(1.0f, 0.0f, 0.0f, 0.0f));
    system.AddBody(body1);

    auto body2 = std::make_shared<ChBodyEasyCylinder>(0.1, 1.0, 1000, false, true);
    body2->SetPos(ChVector<>(2.5, 0, 0));
    body2->SetRot(Q_ROTATE_Y_TO_X);
    body2->AddAsset(std::make_shared<ChColorAsset>(0.0f, 1.0f, 0.0f, 0.0f));
    system.AddBody(body2);

    auto body3 = std::make_shared<ChBodyEasyCylinder>(0.1, 2.0, 1000, false, true);
    body3->SetPos(ChVector<>(2.0, 0, 0));
    body3->SetRot(Q_ROTATE_Y_TO_X);
    body3->AddAsset(std::make_shared<ChColorAsset>(0.0f, 0.0f, 1.0f, 0.0f));
    system.AddBody(body3);

    // Add joints.
    auto rev1 = std::make_shared<ChLinkLockRevolute>();
    rev1->Initialize(ground, body1, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    system.AddLink(rev1);

    auto rev2 = std::make_shared<ChLinkLockRevolute>();
    rev2->Initialize(ground, body2, ChCoordsys<>(ChVector<>(2, 0, 0), QUNIT));
    system.AddLink(rev2);

    auto rev13 = std::make_shared<ChLinkLockRevolute>();
    rev13->Initialize(body1, body3, ChCoordsys<>(ChVector<>(1, 0, 0), QUNIT));
    system.AddLink(rev13);

    auto rev23 = std::make_shared<ChLinkLockRevolute>();
    rev23->Initialize(body2, body3, ChCoordsys<>(ChVector<>(3, 0, 0), QUNIT));
    system.AddLink(rev23);

    // Create the spring between body1 and ground.
    // The spring end points are specified in the body relative frames.
    auto spring = std::make_shared<ChLinkSpring>();
    spring->Initialize(ground, body1, true, ChVector<>(1.5, 1, 0), ChVector<>(0, 0.5, 0), false, 0.5);
    spring->Set_SpringK(3e2);
    spring->Set_SpringR(2e2);
    system.AddLink(spring);

    spring->AddAsset(std::make_shared<ChColorAsset>(0.0f, 0.0f, 0.0f, 0.0f));
    spring->AddAsset(std::make_shared<ChPointPointSpring>(0.05, 80, 15));

    // Perform equilibrium analysis.
    auto p1 = body1->GetPos();
    std::cout << "Body1 pos: " << p1.x() << " " << p1.y() << " " << p1.z() << std::endl;
    system.DoStaticNonlinear(20);
    p1 = body1->GetPos();
    std::cout << "Body1 pos: " << p1.x() << " " << p1.y() << " " << p1.z() << std::endl;

    // Create Irrlicht window.
    ChIrrApp application(&system, L"Equilibrium demo", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 0, 6));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    application.SetTimestep(0.001);
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
