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
// Demo for universal joint with a rotational damper.
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

// -----------------------------------------------------------------------------

// Callback class implementing the torque for a ChLinkRotSpringCB link.
class RotationalDamper : public ChLinkRSDA::TorqueFunctor {
    virtual double evaluate(double time,      // current time
                            double angle,     // relative angle of rotation
                            double vel,       // relative angular speed
                            ChLinkRSDA* link  // back-pointer to associated link
                            ) override {
        double torque = -300 * vel;
        return torque;
    }
};

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create system
    ChSystemSMC system;

    // Create the ground body
    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);
    ground->SetPos(ChVector<>(0, 0, 0));
    ground->SetRot(ChQuaternion<>(1, 0, 0, 0));

    // Add visualization assets to represent the arm of the universal joint's
    // cross associated with the ground (a cylinder)
    {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = ChVector<>(-0.2, 0, 0);
        cyl->GetCylinderGeometry().p2 = ChVector<>(+0.2, 0, 0);
        cyl->GetCylinderGeometry().rad = 0.05;
        ground->AddAsset(cyl);

        auto col = chrono_types::make_shared<ChColorAsset>();
        col->SetColor(ChColor(0.6f, 0, 0));
        ground->AddAsset(col);
    }

    // Create the articulated body
    auto body = chrono_types::make_shared<ChBody>();
    system.AddBody(body);
    body->SetIdentifier(1);
    body->SetBodyFixed(false);
    body->SetCollide(false);
    body->SetMass(100);
    body->SetInertiaXX(ChVector<>(0.1, 0.01, 0.1));
    body->SetPos(ChVector<>(0, -2, 0));
    body->SetRot(ChQuaternion<>(1, 0, 0, 0));
    body->SetPos_dt(ChVector<>(5, 0, 5));

    // Add visualization assets to represent the body (a box) and the arm of the
    // universal joint's cross associated with this shaft (a cylinder)
    {
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = ChVector<>(0.15, 0.9 * 2, 0.15);
        body->AddAsset(box);

        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = ChVector<>(0, 2, -0.2);
        cyl->GetCylinderGeometry().p2 = ChVector<>(0, 2, +0.2);
        cyl->GetCylinderGeometry().rad = 0.05;
        body->AddAsset(cyl);

        auto col = chrono_types::make_shared<ChColorAsset>();
        col->SetColor(ChColor(0, 0, 0.6f));
        body->AddAsset(col);
    }

    // Connect the body to the ground through a universal joint (loacted at the origin).
    // Its kinematic constraints will enforce orthogonality of the associated cross.
    auto ujoint = chrono_types::make_shared<ChLinkUniversal>();
    system.AddLink(ujoint);
    ujoint->Initialize(ground, body, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)));

    // Add a rotational spring-damper
    auto torque = chrono_types::make_shared<RotationalDamper>();
    auto rsda = chrono_types::make_shared<ChLinkRSDA>();
    rsda->Initialize(ground, body, ChCoordsys<>());
    rsda->RegisterTorqueFunctor(torque);
    system.AddLink(rsda);

    // Create the Irrlicht application
    ChIrrApp application(&system, L"U-joint with rotational damper", irr::core::dimension2d<irr::u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(0, 2, 0));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    application.SetTimestep(5e-4);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
