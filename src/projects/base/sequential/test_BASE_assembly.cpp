/// shaft, connected to the bround by revolute joint at the point <100, 0, 0>
/// initial linear speed is (0, 0, 300.);
/// initial angular speed angularVelAbs = (0, 3., 0);

#include <cmath>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;


ChVector<> getLinearAccelAbs(std::shared_ptr<ChBody> body) {
    auto& bodyFrame = body->GetFrame_REF_to_abs();
    ChVector<> linearAccAbs = body->GetFrame_REF_to_abs().GetPos_dtdt();
    return linearAccAbs;
}


int main(int argc, char* argv[]) {

    ChSystemNSC m_System;

    m_System.Set_G_acc(ChVector<>(0, 0, 0));

    // Create the ground (fixed) body
    // ------------------------------
    auto ground = chrono_types::make_shared<ChBody>();
    m_System.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);

    auto m_Body = chrono_types::make_shared<ChBody>();
    m_System.AddBody(m_Body);
    m_Body->SetIdentifier(1);
    m_Body->SetBodyFixed(false);
    m_Body->SetCollide(false);
    m_Body->SetMass(1);
    m_Body->SetInertiaXX(ChVector<>(1, 1, 1));
    m_Body->SetPos(ChVector<>(0, 0, 0));
    m_Body->SetRot(ChQuaternion<>(1, 0, 0, 0));

    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(-100, 0, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(100, 0, 0);
    cyl->GetCylinderGeometry().rad = 2;
    m_Body->AddAsset(cyl);
    auto col = chrono_types::make_shared<ChColorAsset>(0.6f, 0.0f, 0.0f);
    m_Body->AddAsset(col);

    ChVector<> linearVelAbs(0, 0, 300.);
    m_Body->SetPos_dt(linearVelAbs);
    ChVector<> angularVelAbs(0, 3, 0);  ////  RADU:  changed this!!!
    m_Body->SetWvel_par(angularVelAbs);
    m_Body->SetPos_dt(linearVelAbs);

    ChVector<> translation(100, 0, 0);
    ChVector<> orientationRad = CH_C_DEG_TO_RAD * 90.;
    ChQuaternion<> quat;
    quat.Q_from_AngX(orientationRad[0]);

    Coordsys absCoor{translation, quat};

    auto markerBodyLink =
        std::make_shared<ChMarker>("MarkerBodyLink", m_Body.get(), ChCoordsys<>(), ChCoordsys<>(), ChCoordsys<>());

    m_Body->AddMarker(markerBodyLink);
    markerBodyLink->Impose_Abs_Coord(absCoor);

    auto markerGroundLink =
        std::make_shared<ChMarker>("MarkerGroundLink", ground.get(), ChCoordsys<>(), ChCoordsys<>(), ChCoordsys<>());

    ground->AddMarker(markerGroundLink);
    markerGroundLink->Impose_Abs_Coord(absCoor);

    auto m_Link = std::shared_ptr<chrono::ChLinkMarkers>(new ChLinkLockRevolute());
    m_Link->Initialize(markerGroundLink, markerBodyLink);
    m_System.AddLink(m_Link);

    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    solver->SetVerbose(false);
    m_System.SetSolver(solver);

    m_System.SetSolverForceTolerance(1e-12);

    m_System.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
    m_System.SetStep(1e-10);

    ChIrrApp application(&m_System, L"ChBodyAuxRef demo", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 20, 150));

    application.AssetBindAll();
    application.AssetUpdateAll();

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        ////m_System.DoStepDynamics(1e-4);
        application.EndScene();
    }


    m_System.DoFullAssembly();
    auto linearAccelInit = getLinearAccelAbs(m_Body);




}
