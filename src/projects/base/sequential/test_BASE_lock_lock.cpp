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

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrTools.h"

using namespace chrono;

std::shared_ptr<ChLinkLockLock> model1(std::shared_ptr<ChSystem> sys) {
    // Create the ground body
    auto m_Ground = chrono_types::make_shared<ChBodyAuxRef>();
    m_Ground->SetBodyFixed(true);
    auto box = chrono_types::make_shared<ChBoxShape>();
    box->SetColor(ChColor(0.0f, 0.0f, 1.0f));
    box->GetBoxGeometry().SetLengths(ChVector<>(1, 0.1, 1));
    m_Ground->AddVisualShape(box);
    sys->AddBody(m_Ground);

    auto m_Body = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    auto sphere = chrono_types::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = 0.2;
    m_Body->AddVisualShape(sphere);
    sys->AddBody(m_Body);

    auto m_Link = chrono_types::make_shared<chrono::ChLinkLockLock>();

    auto m_MarkerLinkBody = chrono_types::make_shared<chrono::ChMarker>();
    m_Body->AddMarker(m_MarkerLinkBody);

    auto m_MarkerLinkGround = chrono_types::make_shared<chrono::ChMarker>();
    m_Ground->AddMarker(m_MarkerLinkGround);

    ChVector<> markerCoorAbs{0., 0., 100.};
    Coordsys markerAbsCoor{markerCoorAbs};
    m_MarkerLinkBody->Impose_Abs_Coord(markerAbsCoor);
    m_MarkerLinkGround->Impose_Abs_Coord(markerAbsCoor);

    m_Link->Initialize(m_MarkerLinkGround, m_MarkerLinkBody);
    sys->AddLink(m_Link);

    ////sys->Update();
    ////sys->DoFullAssembly();

    return m_Link;
}

int main(int argc, char* argv[]) {
    // Create system
    auto sys = chrono_types::make_shared<ChSystemSMC>();
    ////auto sys = chrono_types::make_shared<ChSystemNSC>();
    sys->Set_G_acc(ChVector<>(0.0, -10., 0.));

    ////sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    ////sys->SetSolverMaxIterations(200);

    auto m_Link = model1(sys);

    // Create the visualization window
    auto vis = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Lock-lock");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0, 1, 2));
    vis->AttachSystem(sys.get());

    // Run simulation for specified time
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        irrlicht::tools::drawAllCOGs(vis.get(), 0.5);
        vis->EndScene();

        GetLog() << "Time: " << sys->GetChTime();

        auto cnstr = m_Link->GetConstraintViolation();
        for (int i = 0; i < 6; i++) {
            GetLog() << "  " << cnstr(i);
        }

        auto mpos = m_Link->GetMarker2()->GetAbsCoord().pos;
        GetLog() << "  X: " << mpos.x() << "  Y: " << mpos.y() << "  Z: " << mpos.z() << "\n";

        auto bpos = m_Link->GetBody2()->GetPos();
        GetLog() << "  X: " << bpos.x() << "  Y: " << bpos.y() << "  Z: " << bpos.z() << "\n";

        sys->DoStepDynamics(1e-3);
    }

    return 0;
}
