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
// Test for actuating a joint using motion functions specified either on the
// joint or on an underlying joint marker.
//
// To replicate a driven revolute joint, use a ChLinkLockLock joint and specify
// motion axis and angle function (see "Test 1" below).
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrTools.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    // Create system
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<double>(0, 0, 0));

    // Create the bodies
    auto body1 = std::shared_ptr<ChBody>(sys.NewBody());
    body1->SetPos(ChVector<>(-0.5, 0, 0));
    body1->SetBodyFixed(true);
    auto cyl1 = chrono_types::make_shared<ChCylinderShape>(0.1, 1);
    cyl1->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    body1->AddVisualShape(cyl1, ChFrame<>(VNULL, Q_from_AngY(CH_C_PI_2)));
    sys.AddBody(body1);

    auto body2 = std::shared_ptr<ChBody>(sys.NewBody());
    body2->SetPos(ChVector<>(+0.5, 0, 0));
    auto box2 = chrono_types::make_shared<ChBoxShape>(2, 0.4, 0.4);
    box2->SetColor(ChColor(0.0f, 1.0f, 0.0f));
    body2->AddVisualShape(box2);
    sys.AddBody(body2);

    // Create a sine function
    std::shared_ptr<ChFunction_Sine> fun = chrono_types::make_shared<ChFunction_Sine>(0.0, 0.1, CH_C_PI_4);

    // ---------------------------------------------

    // Test 0
    //   LockRevolute moving under gravity.
    //   No actuation.
    // Test 1
    //   LockLock with one actuated rotation axis.
    //   Equivalent to a driver constraint for revolute joint.
    // Test 2
    //   LockRevolute with rotation motion applied to the underlying marker along the joint's DOF.
    //   No effect on the relative body motion.
    // Test 3
    //   LockRevolute with rotation motion applied to underlying marker along an axis other than the joint's DOF.
    //   This has the effect of rotating the joint's DOF axis.
    //
    // NOTES:
    //   Body1 (red cylinder) fixed to ground
    //   Default gravity set to [0, 0, 0]
    //   Default motion axis is along Z, i.e. [0, 0, 1]

    enum TestSelector { TEST0, TEST1, TEST2, TEST3 };
    TestSelector test = TEST1;

    switch (test) {
        case TEST0: {
            auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
            sys.AddLink(joint);
            joint->Initialize(body1, body2, ChCoordsys<>(ChVector<>(0, 0, 0)));
            sys.Set_G_acc(ChVector<>(0, -10, 0));
            break;
        }
        case TEST1: {
            auto joint = chrono_types::make_shared<ChLinkLockLock>();
            sys.AddLink(joint);
            joint->Initialize(body1, body2, ChCoordsys<>(ChVector<>(0, 0, 0)));
            joint->SetMotion_axis(ChVector<>(0, 0, 1));
            joint->SetMotion_ang(fun);
            break;
        }
        case TEST2: {
            auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
            sys.AddLink(joint);
            joint->Initialize(body1, body2, ChCoordsys<>(ChVector<>(0, 0, 0)));
            joint->GetMarker1()->SetMotion_ang(fun);
            break;
        }
        case TEST3: {
            auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
            sys.AddLink(joint);
            joint->Initialize(body1, body2, ChCoordsys<>(ChVector<>(0, 0, 0)));
            joint->GetMarker1()->SetMotion_axis(ChVector<>(0, 1, 0));
            joint->GetMarker1()->SetMotion_ang(fun);
            break;
        }
    }

    // ----------------------------------------------

    // Create the visualization window
    auto vis = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Rev motion");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0, 1, 2));
    vis->AttachSystem(&sys);

    // Run simulation for specified time
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        ////irrlicht::tools::drawAllCOGs(vis.get(), 0.5);
        irrlicht::tools::drawAllLinkframes(vis.get(), 1);
        vis->EndScene();
        sys.DoStepDynamics(1e-3);
    }

    return 0;
}
