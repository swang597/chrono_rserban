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
// Mechanism sent by Dmitry (Isymtec).
// Badly sclaed (very large "articulated inertia" due to extreme joint poisition)
// Default solver (with default settings) for SMC systems has convergence issues.
// Solutions:
//    - switch to NSC (default solver a bit more robust)
//    - change solver type (e.g. Barzilai-Borwein)
//    - increase maximum number of iterations
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrTools.h"

using namespace chrono;

std::shared_ptr<ChLinkLockLock> model1(std::shared_ptr<ChSystem> m_System) {
    // Create the ground body
    auto m_Ground = std::make_shared<ChBodyAuxRef>();
    m_Ground->SetBodyFixed(true);
    auto box = std::make_shared<ChBoxShape>();
    box->GetBoxGeometry().SetLengths(ChVector<>(1, 0.1, 1));
    m_Ground->AddAsset(box);
    m_Ground->AddAsset(std::make_shared<ChColorAsset>(0.0f, 0.0f, 1.0f));
    m_System->AddBody(m_Ground);

    auto m_Body = std::make_shared<chrono::ChBodyAuxRef>();
    auto sphere = std::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = 0.2;
    m_Body->AddAsset(sphere);
    m_System->AddBody(m_Body);

    auto m_Link = std::make_shared<chrono::ChLinkLockLock>();

    auto m_MarkerLinkBody = std::make_shared<chrono::ChMarker>();
    m_Body->AddMarker(m_MarkerLinkBody);

    auto m_MarkerLinkGround = std::make_shared<chrono::ChMarker>();
    m_Ground->AddMarker(m_MarkerLinkGround);

    ChVector<> markerCoorAbs{0., 0., 100.};
    Coordsys markerAbsCoor{markerCoorAbs};
    m_MarkerLinkBody->Impose_Abs_Coord(markerAbsCoor);
    m_MarkerLinkGround->Impose_Abs_Coord(markerAbsCoor);

    m_Link->Initialize(m_MarkerLinkGround, m_MarkerLinkBody);
    m_System->AddLink(m_Link);

    ////m_System->Update();
    ////m_System->DoFullAssembly();

    return m_Link;
}

int main(int argc, char* argv[]) {
    // Create system
    auto m_System = std::make_shared<ChSystemSMC>();
    ////auto m_System = std::make_shared<ChSystemNSC>();
    m_System->Set_G_acc(ChVector<>(0.0, -10., 0.));

    ////m_System->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    ////m_System->SetMaxItersSolverSpeed(200);

    auto m_Link = model1(m_System);

    // Create the visualization window
    irrlicht::ChIrrApp application(m_System.get(), L"Lock-lock", irr::core::dimension2d<irr::u32>(800, 600), false,
                                   true);
    irrlicht::ChIrrWizard::add_typical_Logo(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Lights(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(0, 1, 2));
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Run simulation for specified time
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        irrlicht::ChIrrTools::drawAllCOGs(*m_System, application.GetVideoDriver(), 0.5);
        application.EndScene();

        GetLog() << "Time: " << m_System->GetChTime();

        auto cnstr = m_Link->GetC();
        for (int i = 0; i < 6; i++) {
            GetLog() << "  " << cnstr->GetElement(i, 0);
        }

        auto mpos = m_Link->GetMarker2()->GetAbsCoord().pos;
        GetLog() << "  X: " << mpos.x() << "  Y: " << mpos.y() << "  Z: " << mpos.z() << "\n";

        auto bpos = m_Link->GetBody2()->GetPos();
        GetLog() << "  X: " << bpos.x() << "  Y: " << bpos.y() << "  Z: " << bpos.z() << "\n";

        m_System->DoStepDynamics(1e-3);
    }

    return 0;
}
