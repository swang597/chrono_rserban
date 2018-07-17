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
// Test compliant steering column for a Pitman arm mechanism.
//
// Use a 4-bar mechanism modeled after the Chrono::Vehicle Pitman Arm steering.
// Use numerical values from the HMMWV steering mechanism.
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChRealtimeStep.h"

#include "chrono/physics/ChShaftsGear.h"
#include "chrono/physics/ChShaftsMotorAngle.h"
#include "chrono/physics/ChShaftsTorsionSpring.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChPointPointDrawing.h"

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace filesystem;

using std::cout;
using std::endl;

// =============================================================================

double GetSteering(double time) {
    double freq = 0.5;
    return std::sin(2 * CH_C_PI * freq * time);
}

// Steering input applied to shaft
void ApplySteering(double time, double steering, std::shared_ptr<ChShaftsMotorAngle> element) {
    double max_angle = 50.0 * (CH_C_PI / 180);
    auto fun = std::static_pointer_cast<ChFunction_Setpoint>(element->GetAngleFunction());
    fun->SetSetpoint(max_angle * steering, time);
}

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    // -------------
    // Create bodies
    // -------------

    // Chassis (fixed to ground)
    auto chassis = std::shared_ptr<ChBody>(sys.NewBody());
    chassis->SetBodyFixed(true);
    sys.AddBody(chassis);

    // Steering link
    auto link = std::shared_ptr<ChBody>(sys.NewBody());
    link->SetPos(ChVector<>(0.129, 0, 0));
    link->SetRot(QUNIT);
    link->SetMass(3.68);
    link->SetInertiaXX(ChVector<>(0.252, 0.00233, 0.254));
    link->SetInertiaXY(ChVector<>(0.0, 0.0, 0.0));
    sys.AddBody(link);

    // Visualization of the steering link
    double link_radius = 0.03;
    auto pP = link->TransformPointParentToLocal(ChVector<>(0.129, 0.249, 0));        // univ joint
    auto pI = link->TransformPointParentToLocal(ChVector<>(0.129, -0.325, 0));       // S location of revsph
    auto pTP = link->TransformPointParentToLocal(ChVector<>(0.195, 0.448, 0.035));   // tierod loc (Pitman arm side)
    auto pTI = link->TransformPointParentToLocal(ChVector<>(0.195, -0.448, 0.035));  // tierod loc (idler side)
    {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = pP;
        cyl->GetCylinderGeometry().p2 = pI;
        cyl->GetCylinderGeometry().rad = link_radius;
        link->AddAsset(cyl);
        auto cyl_P = std::make_shared<ChCylinderShape>();
        cyl_P->GetCylinderGeometry().p1 = pP;
        cyl_P->GetCylinderGeometry().p2 = pTP;
        cyl_P->GetCylinderGeometry().rad = link_radius;
        link->AddAsset(cyl_P);
        auto cyl_I = std::make_shared<ChCylinderShape>();
        cyl_I->GetCylinderGeometry().p1 = pI;
        cyl_I->GetCylinderGeometry().p2 = pTI;
        cyl_I->GetCylinderGeometry().rad = link_radius;
        link->AddAsset(cyl_I);
        auto col = std::make_shared<ChColorAsset>();
        col->SetColor(ChColor(0.2f, 0.7f, 0.7f));
        link->AddAsset(col);
    }

    // Markers on steering link (at tie-rod connections)
    auto markerP = std::make_shared<ChMarker>();
    markerP->Impose_Rel_Coord(ChCoordsys<>(pTP));
    link->AddMarker(markerP);

    auto markerI = std::make_shared<ChMarker>();
    markerI->Impose_Rel_Coord(ChCoordsys<>(pTI));
    link->AddMarker(markerI);

    // Pitman arm body
    auto arm = std::shared_ptr<ChBody>(sys.NewBody());
    arm->SetPos(ChVector<>(0.064, 0.249, 0));
    arm->SetRot(QUNIT);
    arm->SetMass(1.605);
    arm->SetInertiaXX(ChVector<>(0.00638, 0.00756, 0.00150));
    arm->SetInertiaXY(ChVector<>(0.0, 0.0, 0.0));
    sys.AddBody(arm);

    double arm_radius = 0.02;

    auto pC = arm->TransformPointParentToLocal(ChVector<>(0, 0.249, 0));      // rev joint loc
    auto pL = arm->TransformPointParentToLocal(ChVector<>(0.129, 0.249, 0));  // univ joint loc
    {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = pC;
        cyl->GetCylinderGeometry().p2 = pL;
        cyl->GetCylinderGeometry().rad = arm_radius;
        arm->AddAsset(cyl);
        auto col = std::make_shared<ChColorAsset>();
        col->SetColor(ChColor(0.7f, 0.7f, 0.2f));
        arm->AddAsset(col);
    }

    // -------------
    // Create joints
    // -------------

    auto revolute = std::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(chassis, arm, ChCoordsys<>(ChVector<>(0, 0.249, 0), QUNIT));
    sys.AddLink(revolute);

    ChVector<> u(0, 0, 1);  // univ axis on arm
    ChVector<> v(1, 0, 0);  // univ axis on link
    ChVector<> w(0, 1, 0);  // w = u x v
    ChMatrix33<> rot;
    rot.Set_A_axis(u, v, w);
    auto universal = std::make_shared<ChLinkUniversal>();
    universal->Initialize(arm, link, ChFrame<>(ChVector<>(0.129, 0.249, 0), rot.Get_A_quaternion()));
    sys.AddLink(universal);

    double distance = (ChVector<>(0.129, -0.325, 0) - ChVector<>(0, -0.325, 0)).Length();
    auto revsph = std::make_shared<ChLinkRevoluteSpherical>();
    revsph->Initialize(chassis, link, ChCoordsys<>(ChVector<>(0, -0.325, 0), QUNIT), distance);
    revsph->AddAsset(std::make_shared<ChPointPointSegment>());
    sys.AddLink(revsph);

    // ---------------------
    // Model steering column
    // ---------------------
    bool rigid = false;

    // Maximum angle at steering wheel
    double max_angle = 200.0 * (CH_C_PI / 180);
    // Steering colum transmission ratio
    double gear_ratio = 4.0;

    // Chassis -- shaftC -- shaftC1 -- shaftA1 -- shaftA -- Arm
    auto shaftC = std::make_shared<ChShaft>();
    shaftC->SetInertia(0.01);
    sys.Add(shaftC);

    auto shaftC1 = std::make_shared<ChShaft>();
    shaftC1->SetInertia(0.01);
    sys.Add(shaftC1);

    auto shaftA1 = std::make_shared<ChShaft>();
    shaftA1->SetInertia(0.01);
    sys.Add(shaftA1);

    auto shaftA = std::make_shared<ChShaft>();
    shaftA->SetInertia(0.01);
    sys.Add(shaftA);

    // Rigidly attach shaftA to the arm body
    auto shaft_arm = std::make_shared<ChShaftsBody>();
    shaft_arm->Initialize(shaftA, arm, ChVector<>(0, 0, 1));
    sys.Add(shaft_arm);

    // Rigidly attach shaftC to the chassis body
    auto shaft_chassis = std::make_shared<ChShaftsBody>();
    shaft_chassis->Initialize(shaftC, chassis, ChVector<>(0, 0, 1));
    sys.Add(shaft_chassis);

    // A motor (for steering input) between shaftC and shaftC1
    auto shaft_motor = std::make_shared<ChShaftsMotorAngle>();
    shaft_motor->Initialize(shaftC, shaftC1);
    auto motor_fun = std::make_shared<ChFunction_Setpoint>();
    shaft_motor->SetAngleFunction(motor_fun);
    sys.Add(shaft_motor);

    // A compliant or a rigid connection between shaftA1 and shaftC1
    if (rigid) {
        // Use a geear couple with ratio=1
        auto rigid_connection = std::make_shared<ChShaftsGear>();
        rigid_connection->SetTransmissionRatio(1.0);
        rigid_connection->Initialize(shaftA1, shaftC1);
        sys.Add(rigid_connection);
        cout << "RIGID" << endl;
    } else {
        // Use a torsional spring between shaftA1 and shaftC1
        auto spring_connection = std::make_shared<ChShaftsTorsionSpring>();
        spring_connection->SetTorsionalStiffness(2 * CH_C_RAD_TO_DEG);
        ////spring_connection->SetTorsionalDamping(10);
        spring_connection->Initialize(shaftC1, shaftA1);
        sys.Add(spring_connection);
        cout << "COMPLIANT" << endl;
    }

    // A reduction gear between shaftA and shaftA1
    // (note order of connected shafts for gear_ratio > 1)
    auto shaft_gear = std::make_shared<ChShaftsGear>();
    shaft_gear->SetTransmissionRatio(gear_ratio);
    shaft_gear->Initialize(shaftA, shaftA1);
    sys.Add(shaft_gear);

    // ---------------------------
    // Create Irrlicht application
    // ---------------------------

    ChIrrApp application(&sys, L"Compliant steering", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(0, 0, -2), irr::core::vector3df(0, 0, 0));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // -------------
    // Set up output
    // -------------

    std::string out_dir = "../TEST_steering_compliance";
    std::string out_file = out_dir + "/out.txt";

    bool out_dir_exists = path(out_dir).exists();
    if (out_dir_exists) {
        cout << "Output directory already exists" << endl;
    } else if (create_directory(path(out_dir))) {
        cout << "Create directory = " << path(out_dir).make_absolute() << endl;
    } else {
        cout << "Error creating output directory" << endl;
        return 1;
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    // ---------------
    // Simulation loop
    // ---------------

    ChRealtimeStepTimer realtime_timer;

    while (application.GetDevice()->run()) {
        // Render scene
        application.BeginScene();
        application.DrawAll();
        application.EndScene();

        // Update steering input
        double time = sys.GetChTime();
        double steering = GetSteering(time);
        motor_fun->SetSetpoint(max_angle * steering, time);
        ////ApplySteering(time, steering, shaft_motor);

        // Output
        auto aPos = arm->GetPos();
        auto aRot = arm->GetRot();
        auto aAngles = aRot.Q_to_Euler123();
        auto lPos = link->GetPos();
        auto mP = markerP->GetAbsCoord().pos;
        auto mI = markerI->GetAbsCoord().pos;
        csv << time << steering << aPos << aAngles << lPos << mP << mI << endl;

        // Advance dynamics
        double step = realtime_timer.SuggestSimulationStep(1e-3);
        sys.DoStepDynamics(step);

        if (time > 5.0)
            break;
    }

    csv.write_to_file(out_file);

    return 0;
}
