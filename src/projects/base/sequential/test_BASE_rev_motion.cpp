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
// Test for actuating a "revolute" joint using motion functions:
// Use a ChLinkLockLock joint and specify motion axis and angle function.
//
// =============================================================================

#include <omp.h>
#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrTools.h"

using namespace chrono;
using namespace chrono::collision;

int main(int argc, char* argv[]) {
    // Create system
    ChSystemNSC my_sys;
    my_sys.Set_G_acc(ChVector<double>(0, 0, 0));

    // Create the bodies
    auto body1 = std::shared_ptr<ChBody>(my_sys.NewBody());
    body1->SetPos(ChVector<>(-0.5, 0, 0));
    auto cyl1 = std::make_shared<ChCylinderShape>();
    cyl1->GetCylinderGeometry().p1 = ChVector<>(-0.5, 0, 0);
    cyl1->GetCylinderGeometry().p2 = ChVector<>(+0.5, 0, 0);
    cyl1->GetCylinderGeometry().rad = 0.1;
    body1->AddAsset(cyl1);
    body1->AddAsset(std::make_shared<ChColorAsset>(1.0f, 0.0f, 0.0f));
    my_sys.AddBody(body1);

    auto body2 = std::shared_ptr<ChBody>(my_sys.NewBody());
    body2->SetPos(ChVector<>(+0.5, 0, 0));
    auto box2 = std::make_shared<ChBoxShape>();
    box2->GetBoxGeometry().SetLengths(ChVector<>(1, 0.2, 0.2));
    body2->AddAsset(box2);
    body2->AddAsset(std::make_shared<ChColorAsset>(0.0f, 1.0f, 0.0f));
    my_sys.AddBody(body2);

    // Create the joint and actuate rotation axis (driver constraint for revolute joint)
    auto joint = std::make_shared<ChLinkLockLock>();
    my_sys.AddLink(joint);
    joint->Initialize(body1, body2, ChCoordsys<>(ChVector<>(0, 0, 0)));

    std::shared_ptr<ChFunction_Sine> fun = std::make_shared<ChFunction_Sine>(0.0, 0.05, 1);
    joint->SetMotion_axis(ChVector<>(0, 0, 1));
    joint->SetMotion_ang(fun);

    ////auto joint = std::make_shared<ChLinkLockRevolute>();
    ////my_sys.AddLink(joint);
    ////joint->Initialize(body1, body2, ChCoordsys<>(ChVector<>(0, 0, 0)));
    ////joint->GetMarker1()->SetMotion_ang(fun);

    // Create the visualization window
    irrlicht::ChIrrApp application(&my_sys, L"Rev motion", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    irrlicht::ChIrrWizard::add_typical_Logo(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Lights(application.GetDevice(), irr::core::vector3df(30.f, 100.f, 30.f),
                                              irr::core::vector3df(30.f, 80.f, -30.f));
    irrlicht::ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(0, 1, 2));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Run simulation for specified time
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        irrlicht::ChIrrTools::drawAllCOGs(my_sys, application.GetVideoDriver(), 0.5);
        irrlicht::ChIrrTools::drawAllLinkframes(my_sys, application.GetVideoDriver(), 1);
        application.EndScene();
        my_sys.DoStepDynamics(1e-3);
    }

    return 0;
}
