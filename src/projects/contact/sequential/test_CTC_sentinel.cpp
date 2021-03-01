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
// Test using a collision shape as a "sentinel" body
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

// ====================================================================================

class Monitor : public collision::ChCollisionSystem::NarrowphaseCallback {
public:
    Monitor(std::shared_ptr<ChBody> sentinel) : m_sentinel(sentinel) {}
    virtual bool OnNarrowphase(collision::ChCollisionInfo& contactinfo) {
        auto c1 = contactinfo.modelA->GetContactable();
        auto c2 = contactinfo.modelB->GetContactable();
        if (c1 == m_sentinel.get() || c2 == m_sentinel.get()) {
            std::cout << "Collision" << std::endl;
            return false;
        }
        return true;
    }
private:
    std::shared_ptr<ChBody> m_sentinel;
};

// ====================================================================================

int main(int argc, char* argv[]) {
    // Create the system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<>(0, -10, 0));

    // Create the Irrlicht visualization
    ChIrrApp application(&system, L"DEM demo", core::dimension2d<u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 3, -6));

    // Shared contact material
    auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    // Create the sentinel sphere
    auto sentinel = chrono_types::make_shared<ChBody>();

    sentinel->SetIdentifier(0);
    sentinel->SetMass(1);
    sentinel->SetPos(ChVector<>(0, 4, 0));
    sentinel->SetCollide(true);
    sentinel->SetBodyFixed(true);

    sentinel->GetCollisionModel()->ClearModel();
    sentinel->GetCollisionModel()->AddSphere(material, 0.2);
    sentinel->GetCollisionModel()->BuildModel();

    auto sphereS = chrono_types::make_shared<ChSphereShape>();
    sphereS->GetSphereGeometry().rad = 0.2;
    sentinel->AddAsset(sphereS);

    system.AddBody(sentinel);

    // Create a falling ball
    double mass = 100;
    double radius = 1;

    auto ball = chrono_types::make_shared<ChBody>();

    ball->SetIdentifier(1);
    ball->SetMass(mass);
    ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector<>(1, 1, 1));
    ball->SetPos(ChVector<>(0.9, 1.1, 0));
    ball->SetCollide(true);
    ball->SetBodyFixed(false);

    ball->GetCollisionModel()->ClearModel();
    ball->GetCollisionModel()->AddSphere(material, radius);
    ball->GetCollisionModel()->BuildModel();

    auto sphereB = chrono_types::make_shared<ChSphereShape>();
    sphereB->GetSphereGeometry().rad = radius;
    ball->AddAsset(sphereB);

    auto mtexture = chrono_types::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("textures/bluewhite.png"));
    ball->AddAsset(mtexture);

    system.AddBody(ball);

    // Create ground
    double width = 2;
    double length = 2;
    double thickness = 0.1;

    auto ground = chrono_types::make_shared<ChBody>();

    ground->SetIdentifier(-1);
    ground->SetMass(1);
    ground->SetPos(ChVector<>(0, 0, 0));
    ground->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ground->SetCollide(true);
    ground->SetBodyFixed(true);

    ground->GetCollisionModel()->ClearModel();
    ground->GetCollisionModel()->AddBox(material, width, thickness, length, ChVector<>(0, -thickness, 0));
    ground->GetCollisionModel()->BuildModel();

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = ChVector<>(width, thickness, length);
    box->GetBoxGeometry().Pos = ChVector<>(0, -thickness, 0);
    ground->AddAsset(box);

    system.AddBody(ground);

    // Complete asset construction
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Custom callback
    system.GetCollisionSystem()->RegisterNarrowphaseCallback(chrono_types::make_shared<Monitor>(sentinel));

    // Simulation loop
    double time_step = 1e-3;
    double speed = -2;
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();

        auto crt_pos = sentinel->GetPos();
        crt_pos.y() += speed * time_step;
        sentinel->SetPos(crt_pos);

        system.DoStepDynamics(time_step);

        application.EndScene();
    }

    return 0;
}
