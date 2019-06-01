// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Test tire models outside Chrono::Vehicle AND with a different frame convention.
// Compare a right frame with X forward and Z up (the ISO frame in Chrono::Vehicle)
// and a setup with X forward and Y up.
//
// Create two mechanisms (in disjunct Chrono systems) and simulate them concurrently.
// Each mechanism has a HMMWV TMeasy tire attached to a wheel body which is 
// constrained to move in a vertical plane.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_TMeasyTire.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/ChVehicleModelData.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

double grav = 9.8;

double step_size = 1e-3;
double tire_step_size = 1e-3;

double wheel_mass = 500;  // wheel mass (large to account for chassis load)
double wheel_I1 = 0.05;   // moment of inertia about rotation axis
double wheel_I2 = 0.1;    // moments of inertia about other 2 axes
double wheel_init_height = 0.6;
double wheel_init_omega = 10;

// =============================================================================

class MechanismISO {
  public:
    MechanismISO(ChSystem* sys);
    void Advance(double step);
    const WheelState& GetWheelState() const { return m_wheelstate; }
    const TerrainForce& GetTireForce() const { return m_tireforce; }
  private:
    class Terrain : public ChTerrain {
      public:
        Terrain() {}
        virtual double GetHeight(double x, double y) const override { return 0; }
        virtual ChVector<> GetNormal(double x, double y) const override { return ChVector<>(0, 0, 1); }
        virtual float GetCoefficientFriction(double x, double y) const override { return 0.8f; }
    };

    ChSystem* m_sys;
    std::shared_ptr<ChBody> m_wheel;
    std::shared_ptr<ChTire> m_tire;
    WheelState m_wheelstate;
    TerrainForce m_tireforce;
    Terrain m_terrain;
};

MechanismISO::MechanismISO(ChSystem* sys) : m_sys(sys) {
    m_tire = std::make_shared<hmmwv::HMMWV_TMeasyTire>("TMeasy tire");

    auto ground = std::shared_ptr<ChBody>(m_sys->NewBody());
    m_sys->AddBody(ground);
    ground->SetBodyFixed(true);
    {
        auto box = std::make_shared<ChBoxShape>();
        box->GetBoxGeometry().SetLengths(ChVector<>(10, 2, 0.2));
        box->GetBoxGeometry().Pos = ChVector<>(0, 0, -0.1);
        ground->AddAsset(box);
    }

    m_wheel = std::shared_ptr<ChBody>(m_sys->NewBody());
    m_sys->AddBody(m_wheel);
    m_wheel->SetMass(wheel_mass);
    m_wheel->SetInertiaXX(ChVector<>(wheel_I2, wheel_I1, wheel_I2));
    m_wheel->SetPos(ChVector<>(0, 0, wheel_init_height));
    m_wheel->SetWvel_loc(ChVector<>(0, +wheel_init_omega, 0));  // for the wheel to rotate in positive X
    {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().rad = 0.1;
        cyl->GetCylinderGeometry().p1 = ChVector<>(0, -0.1, 0);
        cyl->GetCylinderGeometry().p2 = ChVector<>(0, +0.1, 0);
        m_wheel->AddAsset(cyl);
    }

    auto link = std::make_shared<ChLinkLockPlanePlane>();
    link->Initialize(ground, m_wheel, ChCoordsys<>(VNULL, Q_from_AngX(CH_C_PI_2)));
    m_sys->AddLink(link);

    m_tire->SetStepsize(tire_step_size);
    m_tire->Initialize(m_wheel, LEFT);
    m_tire->SetVisualizationType(VisualizationType::MESH);
}

void MechanismISO::Advance(double step) {
    double time = m_sys->GetChTime();
    
    m_wheelstate.pos = m_wheel->GetPos();
    m_wheelstate.rot = m_wheel->GetRot();
    m_wheelstate.lin_vel = m_wheel->GetPos_dt();
    m_wheelstate.ang_vel = m_wheel->GetWvel_par();
    ChVector<> ang_vel_loc = m_wheelstate.rot.RotateBack(m_wheelstate.ang_vel);
    m_wheelstate.omega = ang_vel_loc.y();

    m_tireforce = m_tire->ReportTireForce(&m_terrain);
    auto tireforce = m_tire->GetTireForce();

    // Synchronize and advance subsystems
    m_tire->Synchronize(time, m_wheelstate, m_terrain);
    m_wheel->Empty_forces_accumulators();
    m_wheel->Accumulate_force(tireforce.force, tireforce.point, false);
    m_wheel->Accumulate_torque(tireforce.moment, false);

    m_tire->Advance(step);
    m_sys->DoStepDynamics(step);
}

// =============================================================================

WheelState ConvertState_YUP_to_ISO(const WheelState& ws_YUP) {
    // Coordinate transforms
    ChFrame<> iso_X_yup(VNULL, Q_from_AngX(CH_C_PI_2));           // YUP -> ISO
    ChFrame<> wiso_X_wyup = iso_X_yup;                            // Wheel_YUP -> Wheel_ISO
    ChFrame<> wyup_X_wiso = wiso_X_wyup.GetInverse();             // Wheel_ISO -> Wheel_Wheel_YUP
    ChFrame<> yup_X_wyup(ws_YUP.pos, ws_YUP.rot);                 // Wheel_YUP -> YUP
    ChFrame<> iso_X_wiso = iso_X_yup * yup_X_wyup * wyup_X_wiso;  // Wheel_ISO -> ISO

    // Wheel body state in ISO frame
    WheelState ws_ISO;
    ws_ISO.pos = iso_X_wiso.GetPos();
    ws_ISO.rot = iso_X_wiso.GetRot();
    ws_ISO.lin_vel = iso_X_yup.TransformDirectionLocalToParent(ws_YUP.lin_vel);
    ws_ISO.ang_vel = iso_X_yup.TransformDirectionLocalToParent(ws_YUP.ang_vel);
    auto ang_vel_loc = ws_ISO.rot.RotateBack(ws_ISO.ang_vel);
    ws_ISO.omega = ang_vel_loc.y();

    return ws_ISO;
}

TerrainForce ConvertForce_ISO_to_YUP(const TerrainForce& tf_ISO) {
    // Coordinate transforms
    ChFrame<> yup_X_iso(VNULL, Q_from_AngX(-CH_C_PI_2));  // ISO -> YUP

    // Terrain force in YUP frame
    TerrainForce tf_YUP;
    tf_YUP.point = yup_X_iso.TransformPointLocalToParent(tf_ISO.point);
    tf_YUP.force = yup_X_iso.TransformDirectionLocalToParent(tf_ISO.force);
    tf_YUP.moment = yup_X_iso.TransformDirectionLocalToParent(tf_ISO.moment);

    return tf_YUP;

    /*
    // Alternatively, using the iso_X_yup transform...
    ChFrame<> iso_X_yup(VNULL, Q_from_AngX(CH_C_PI_2));
    TerrainForce tf_YUP;
    tf_YUP.point = iso_X_yup.TransformPointParentToLocal(tf_ISO.point);
    tf_YUP.force = iso_X_yup.TransformDirectionParentToLocal(tf_ISO.force);
    tf_YUP.moment = iso_X_yup.TransformDirectionParentToLocal(tf_ISO.moment);

    return tf_YUP;
    */
}

ChVector<> ConvertInertia_ISO_to_YUP(const ChVector<>& inertia_ISO) {
    // Coordinate transforms
    ChMatrix33<> iso_R_yup(Q_from_AngX(+CH_C_PI_2));
    ChMatrix33<> yup_R_iso(Q_from_AngX(-CH_C_PI_2));

    // Moments of inertia in YUP frame
    ChMatrix33<> J_ISO(inertia_ISO);
    ChMatrix33<> J_YUP = yup_R_iso * J_ISO * iso_R_yup;
    ChVector<> inertia_YUP = J_YUP.Get_Diag();

    /*
    // Alternatively, using the iso_X_yup transform...
    ChFrame<> iso_X_yup(VNULL, Q_from_AngX(CH_C_PI_2));
    ChFrame<> yup_X_iso = iso_X_yup.GetInverse();
    ChMatrix33<> J_ISO(inertia_ISO);
    ChMatrix33<> J_YUP = (yup_X_iso.GetA() * J_ISO) * iso_X_yup.GetA();
    ChVector<> inertia_YUP = J_YUP.Get_Diag();
    */

    /*
    // Alternatively, just flip elements (for the particular case of YUP frame)
    ChVector<> inertia_YUP(inertia_ISO.x(), inertia_ISO.z(), inertia_ISO.y());
    */

    return inertia_YUP;
}

// =============================================================================

class MechanismYUP {
  public:
    MechanismYUP(ChSystem* sys);
    void Advance(double step);
    void Advance(double step, const WheelState& ws_ISO);
    const TerrainForce& GetTireForce() const { return m_tireforce; }

  private:
    class Terrain : public ChTerrain {
      public:
        Terrain() {}
        virtual double GetHeight(double x, double y) const override { return 0; }
        virtual ChVector<> GetNormal(double x, double y) const override { return ChVector<>(0, 0, 1); }
        virtual float GetCoefficientFriction(double x, double y) const override { return 0.8f; }
    };

    ChSystem* m_sys;
    std::shared_ptr<ChBody> m_wheel;
    std::shared_ptr<ChTire> m_tire;
    TerrainForce m_tireforce;
    Terrain m_terrain;
};

MechanismYUP::MechanismYUP(ChSystem* sys) : m_sys(sys) {
    m_tire = std::make_shared<hmmwv::HMMWV_TMeasyTire>("TMeasy tire");

    auto ground = std::shared_ptr<ChBody>(m_sys->NewBody());
    m_sys->AddBody(ground);
    ground->SetBodyFixed(true);
    {
        auto box = std::make_shared<ChBoxShape>();
        box->GetBoxGeometry().SetLengths(ChVector<>(10, 0.2, 2));
        box->GetBoxGeometry().Pos = ChVector<>(0, -0.1, 0);
        ground->AddAsset(box);
    }

    m_wheel = std::shared_ptr<ChBody>(m_sys->NewBody());
    m_sys->AddBody(m_wheel);
    m_wheel->SetMass(wheel_mass);
    m_wheel->SetInertiaXX(ChVector<>(wheel_I2, wheel_I2, wheel_I1));
    m_wheel->SetPos(ChVector<>(0, wheel_init_height, 0));
    m_wheel->SetWvel_loc(ChVector<>(0, 0, -wheel_init_omega));  // for the wheel to rotate in positive X
    {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().rad = 0.1;
        cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, -0.1);
        cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, +0.1);
        m_wheel->AddAsset(cyl);
    }

    auto link = std::make_shared<ChLinkLockPlanePlane>();
    link->Initialize(ground, m_wheel, ChCoordsys<>(VNULL, QUNIT));
    m_sys->AddLink(link);

    m_tire->SetStepsize(tire_step_size);
    m_tire->Initialize(m_wheel, LEFT);

    // IMPORTANT: override wheel body inertia!
    m_wheel->SetInertiaXX(ChVector<>(wheel_I2, wheel_I2, wheel_I1) + ConvertInertia_ISO_to_YUP(m_tire->GetInertia()));

    // Cannot directly use tire visualization (incorrect rotation)
    m_tire->SetVisualizationType(VisualizationType::NONE);
    {
        auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile("hmmwv/hmmwv_tire.obj"), false, false);
        trimesh->Transform(VNULL, ChMatrix33<>(Q_from_AngX(-CH_C_PI_2)));
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName("hmmwv_tire_POV_geom");
        trimesh_shape->SetStatic(true);
        m_wheel->AddAsset(trimesh_shape);
    }
}

void MechanismYUP::Advance(double step) {
    double time = m_sys->GetChTime();

    // Wheel body state in YUP frame
    WheelState ws_YUP;
    ws_YUP.pos = m_wheel->GetPos();
    ws_YUP.rot = m_wheel->GetRot();
    ws_YUP.lin_vel = m_wheel->GetPos_dt();
    ws_YUP.ang_vel = m_wheel->GetWvel_par();
    auto ang_vel_loc = ws_YUP.rot.RotateBack(ws_YUP.ang_vel);
    ws_YUP.omega = ang_vel_loc.z();

    // Wheel body state in ISO frame
    WheelState ws_ISO = ConvertState_YUP_to_ISO(ws_YUP);

    // Tire forces in ISO frame
    auto tf_ISO = m_tire->GetTireForce();                // forces to be applied to wheel body
    auto tfR_ISO = m_tire->ReportTireForce(&m_terrain);  // reporting

    // Tire forces in YUP frame
    auto tf_YUP = ConvertForce_ISO_to_YUP(tf_ISO);
    m_tireforce = ConvertForce_ISO_to_YUP(tfR_ISO);

    // Synchronize and advance subsystems
    m_tire->Synchronize(time, ws_ISO, m_terrain);
    m_wheel->Empty_forces_accumulators();
    m_wheel->Accumulate_force(tf_YUP.force, tf_YUP.point, false);
    m_wheel->Accumulate_torque(tf_YUP.moment, false);

    m_tire->Advance(step);
    m_sys->DoStepDynamics(step);
}

// Debug: inject wheel state from outside (e.g. from the ISO model)
void MechanismYUP::Advance(double step, const WheelState& ws_ISO) {
    double time = m_sys->GetChTime();

    // Tire forces in ISO frame
    auto tf_ISO = m_tire->GetTireForce();                // forces to be applied to wheel body
    auto tfR_ISO = m_tire->ReportTireForce(&m_terrain);  // reporting

    // Tire forces in YUP frame
    auto tf_YUP = ConvertForce_ISO_to_YUP(tf_ISO);
    m_tireforce = ConvertForce_ISO_to_YUP(tfR_ISO);

    // Synchronize and advance subsystems
    m_tire->Synchronize(time, ws_ISO, m_terrain);
    m_wheel->Empty_forces_accumulators();
    m_wheel->Accumulate_force(tf_YUP.force, tf_YUP.point, false);
    m_wheel->Accumulate_torque(tf_YUP.moment, false);

    m_tire->Advance(step);
    m_sys->DoStepDynamics(step);
}

// =============================================================================

void TestConversions() {
    WheelState ws_YUP;
    ws_YUP.pos = ChVector<>(1, 2, 3);
    ws_YUP.rot = Q_from_AngZ(CH_C_PI / 6);
    ws_YUP.lin_vel = ChVector<>(0.1, 0.2, 0.3);
    ws_YUP.ang_vel = ChVector<>(0, 0, 10);
    auto ang_vel_loc = ws_YUP.rot.RotateBack(ws_YUP.ang_vel);
    ws_YUP.omega = ang_vel_loc.z();
    ChMatrix33<> R_YUP(ws_YUP.rot);

    WheelState ws_ISO = ConvertState_YUP_to_ISO(ws_YUP);
    ChMatrix33<> R_ISO(ws_ISO.rot);

    TerrainForce tf_ISO;
    tf_ISO.point = ChVector<>(1, 2, 3);
    tf_ISO.force = ChVector<>(0.1, 0.2, 0.3);
    tf_ISO.moment = ChVector<>(10, 20, 30);

    TerrainForce tf_YUP = ConvertForce_ISO_to_YUP(tf_ISO);
}

// =============================================================================

int main(int argc, char* argv[]) {
    ////TestConversions();

    ChSystemNSC sysISO;
    sysISO.Set_G_acc(ChVector<>(0, 0, -grav));
    sysISO.SetMaxItersSolverSpeed(150);
    sysISO.SetMaxPenetrationRecoverySpeed(4.0);
    sysISO.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    MechanismISO mISO(&sysISO);

    ChSystemNSC sysYUP;
    sysYUP.Set_G_acc(ChVector<>(0, -grav, 0));
    sysYUP.SetMaxItersSolverSpeed(150);
    sysYUP.SetMaxPenetrationRecoverySpeed(4.0);
    sysYUP.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    MechanismYUP mYUP(&sysYUP);

    irrlicht::ChIrrApp appISO(&sysISO, L"Tire ISO csys", irr::core::dimension2d<irr::u32>(800, 600));
    appISO.AddTypicalLogo();
    appISO.AddTypicalSky();
    appISO.AddTypicalLights();
    appISO.AddTypicalCamera(irr::core::vector3df(1, 1, 3), irr::core::vector3df(0, 0, 0));
    appISO.AssetBindAll();
    appISO.AssetUpdateAll();

    irrlicht::ChIrrApp appYUP(&sysYUP, L"Tire YUP csys", irr::core::dimension2d<irr::u32>(800, 600));
    appYUP.AddTypicalLogo();
    appYUP.AddTypicalSky();
    appYUP.AddTypicalLights();
    appYUP.AddTypicalCamera(irr::core::vector3df(1, 1, 3), irr::core::vector3df(0, 0, 0));
    appYUP.AssetBindAll();
    appYUP.AssetUpdateAll();

    while (appISO.GetDevice()->run()) {
        appISO.BeginScene();
        appISO.DrawAll();
        irrlicht::ChIrrTools::drawAllCOGs(sysISO, appISO.GetVideoDriver(), 1);
        appISO.EndScene();

        appYUP.BeginScene();
        appYUP.DrawAll();
        irrlicht::ChIrrTools::drawAllCOGs(sysYUP, appYUP.GetVideoDriver(), 1);
        appYUP.EndScene();

        mISO.Advance(step_size);
        mYUP.Advance(step_size);

        auto tfISO = mISO.GetTireForce();
        auto tfYUP = mYUP.GetTireForce();

        ////std::cout << sysISO.GetChTime() << std::endl;
        ////std::cout << "   ISO  " << tfISO.force.x() << "  " << tfISO.force.y() << "  " << tfISO.force.z() << std::endl;
        ////std::cout << "        " << tfISO.point.x() << "  " << tfISO.point.y() << "  " << tfISO.point.z() << std::endl;
        ////std::cout << "        " << tfISO.moment.x() << "  " << tfISO.moment.y() << "  " << tfISO.moment.z() << std::endl;
        ////std::cout << "   YUP  " << tfYUP.force.x() << "  " << tfYUP.force.y() << "  " << tfYUP.force.z() << std::endl;
        ////std::cout << "        " << tfYUP.point.x() << "  " << tfYUP.point.y() << "  " << tfYUP.point.z() << std::endl;
        ////std::cout << "        " << tfYUP.moment.x() << "  " << tfYUP.moment.y() << "  " << tfYUP.moment.z() << std::endl;
    }

    return 0;
}
