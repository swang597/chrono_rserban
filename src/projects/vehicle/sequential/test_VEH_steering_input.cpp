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
// Test different ways of driving a steering mechanism.
//
// Use a 4-bar mechanism modeled after the Chrono::Vehicle Pitman Arm steering.
// Use numerical values from the HMMWV steering mechanism.
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChShaftsBody.h"
#include "chrono/physics/ChShaftsMotorAngle.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChPointPointShape.h"

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace filesystem;

using std::cout;
using std::endl;

// =============================================================================

enum Model {
    LINK_MOTOR,  // revolute between chassis and arm
    SHAFT_MOTOR  // shaft attached to arm and shaft attached to chassis
};

Model model = SHAFT_MOTOR;

// =============================================================================

class SteeringFunction : public ChFunction {
  public:
    SteeringFunction() : m_max_angle(0), m_val(0), m_der(0), m_val_prev(0), m_time_prev(0) {}
    SteeringFunction(double max_angle) : m_max_angle(max_angle), m_val(0), m_der(0), m_val_prev(0), m_time_prev(0) {}

    void Update(double time, double steering) {
        if (time <= m_time_prev)
            return;

        m_val = m_max_angle * steering;
        m_der = (m_val - m_val_prev) / (time - m_time_prev);

        m_val_prev = m_val;
        m_time_prev = time;
    }

    virtual SteeringFunction* Clone() const override { return new SteeringFunction(); }

    virtual double Get_y(double x) const override { return m_val; }
    virtual double Get_y_dx(double x) const override { return m_der; }

  private:
    double m_max_angle;
    double m_val;
    double m_der;
    double m_time_prev;
    double m_val_prev;
};

// =============================================================================

double GetSteering(double time) {
    double freq = 0.5;
    return std::sin(2 * CH_C_PI * freq * time);
}

// Steering input applied to the LinkMotor
void ApplySteering(double time, double steering, std::shared_ptr<ChLinkMotorRotationAngle> element) {
    double max_angle = 50.0 * (CH_C_PI / 180);
    auto fun = std::static_pointer_cast<ChFunction_Setpoint>(element->GetAngleFunction());
    fun->SetSetpoint(max_angle * steering, time);
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
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = pP;
        cyl->GetCylinderGeometry().p2 = pI;
        cyl->GetCylinderGeometry().rad = link_radius;
        link->AddVisualShape(cyl);
        auto cyl_P = chrono_types::make_shared<ChCylinderShape>();
        cyl_P->GetCylinderGeometry().p1 = pP;
        cyl_P->GetCylinderGeometry().p2 = pTP;
        cyl_P->GetCylinderGeometry().rad = link_radius;
        link->AddVisualShape(cyl_P);
        auto cyl_I = chrono_types::make_shared<ChCylinderShape>();
        cyl_I->GetCylinderGeometry().p1 = pI;
        cyl_I->GetCylinderGeometry().p2 = pTI;
        cyl_I->GetCylinderGeometry().rad = link_radius;
        cyl_I->SetColor(ChColor(0.2f, 0.7f, 0.7f));
        link->AddVisualShape(cyl_I);
    }

    // Markers on steering link (at tie-rod connections)
    auto markerP = chrono_types::make_shared<ChMarker>();
    markerP->Impose_Rel_Coord(ChCoordsys<>(pTP));
    link->AddMarker(markerP);

    auto markerI = chrono_types::make_shared<ChMarker>();
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
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = pC;
        cyl->GetCylinderGeometry().p2 = pL;
        cyl->GetCylinderGeometry().rad = arm_radius;
        cyl->SetColor(ChColor(0.7f, 0.7f, 0.2f));
        arm->AddVisualShape(cyl);
    }

    // -------------
    // Create joints
    // -------------

    double max_angle = 50.0 * (CH_C_PI / 180);

    auto revolute_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    revolute_motor->Initialize(chassis, arm, ChFrame<>(ChVector<>(0, 0.249, 0), QUNIT));
    auto motor_fun = chrono_types::make_shared<ChFunction_Setpoint>();
    revolute_motor->SetAngleFunction(motor_fun);

    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(chassis, arm, ChCoordsys<>(ChVector<>(0, 0.249, 0), QUNIT));
    auto shaftA = chrono_types::make_shared<ChShaft>();
    shaftA->SetInertia(0.01);
    auto shaft_arm = chrono_types::make_shared<ChShaftsBody>();
    shaft_arm->Initialize(shaftA, arm, ChVector<>(0, 0, 1));
    auto shaftC = chrono_types::make_shared<ChShaft>();
    shaftC->SetInertia(0.01);
    auto shaft_chassis = chrono_types::make_shared<ChShaftsBody>();
    shaft_chassis->Initialize(shaftC, chassis, ChVector<>(0, 0, 1));
    auto shaft_motor = chrono_types::make_shared<ChShaftsMotorAngle>();
    shaft_motor->Initialize(shaftA, shaftC);
    shaft_motor->SetAngleFunction(motor_fun);

    switch (model) {
        case LINK_MOTOR:
            sys.AddLink(revolute_motor);
            break;
        case SHAFT_MOTOR: {
            sys.AddLink(revolute);
            sys.Add(shaftA);
            sys.Add(shaftC);
            sys.Add(shaft_arm);
            sys.Add(shaft_chassis);
            sys.Add(shaft_motor);
            break;
        }
    }

    ChVector<> u(0, 0, 1);  // univ axis on arm
    ChVector<> v(1, 0, 0);  // univ axis on link
    ChVector<> w(0, 1, 0);  // w = u x v
    ChMatrix33<> rot;
    rot.Set_A_axis(u, v, w);
    auto universal = chrono_types::make_shared<ChLinkUniversal>();
    universal->Initialize(arm, link, ChFrame<>(ChVector<>(0.129, 0.249, 0), rot.Get_A_quaternion()));
    sys.AddLink(universal);

    double distance = (ChVector<>(0.129, -0.325, 0) - ChVector<>(0, -0.325, 0)).Length();
    auto revsph = chrono_types::make_shared<ChLinkRevoluteSpherical>();
    revsph->Initialize(chassis, link, ChCoordsys<>(ChVector<>(0, -0.325, 0), QUNIT), distance);
    revsph->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());
    sys.AddLink(revsph);

    // ---------------------------
    // Create Irrlicht application
    // ---------------------------

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Compliant steering");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 0, -2));
    vis->AddTypicalLights();
    vis->AttachSystem(&sys);

    // -------------
    // Set up output
    // -------------

    std::string out_dir = "../TEST_steering_input";
    std::string out_file;
    switch (model) {
        case LINK_MOTOR:
            out_file = out_dir + "/out_LinkMotor.txt";
            break;
        case SHAFT_MOTOR:
            out_file = out_dir + "/out_ShaftMotor.txt";
            break;
    }

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

    while (vis->Run()) {
        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Update steering input
        double time = sys.GetChTime();
        double steering = GetSteering(time);
        switch (model) {
            case LINK_MOTOR:
                ApplySteering(time, steering, revolute_motor);
                break;
            case SHAFT_MOTOR:
                ApplySteering(time, steering, shaft_motor);
                break;
        }

        // Output
        auto aPos = arm->GetPos();
        auto aRot = arm->GetRot();
        auto aAngles = aRot.Q_to_Euler123();
        auto lPos = link->GetPos();
        auto mP = markerP->GetAbsCoord().pos;
        auto mI = markerI->GetAbsCoord().pos;
        csv << time << steering << aPos << aAngles << lPos << mP << mI << endl;

        // Advance dynamics
        sys.DoStepDynamics(1e-3);
    }

    csv.write_to_file(out_file);

    return 0;
}
