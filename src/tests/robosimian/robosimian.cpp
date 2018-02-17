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
// =============================================================================

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "robosimian.h"

using namespace chrono;

namespace robosimian {

// =============================================================================

const double Chassis::m_mass = 40.0;
const ChVector<> Chassis::m_COM(-0.000820, -0.002060, -0.021800);
const ChVector<> Chassis::m_inertia_xx(0.248970, 0.703230, 0.772560);
const ChVector<> Chassis::m_inertia_xy(0, 0, 0.001820);
const std::string Chassis::m_vis_mesh_name("robosim_chassis");

const std::string Limb::m_vis_mesh_names[] = {
    "robosim_fts",         "robosim_pitch_link", "robosim_roll_link",
    "robosim_pitch_link",  "robosim_roll_link",  "robosim_pitch_link",
    "robosim_roll_link",   "robosim_ft_adapter", "robosim_force_torque_sensor",
    "robosim_wheel_mount", "robosim_wheel"};

// =============================================================================

RoboSimian::RoboSimian(ChMaterialSurface::ContactMethod contact_method, bool fixed) : m_owns_system(true) {
    m_system = (contact_method == ChMaterialSurface::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                          : static_cast<ChSystem*>(new ChSystemSMC);
    m_system->Set_G_acc(ChVector<>(0, 0, -9.81));

    // Integration and Solver settings
    m_system->SetMaxItersSolverSpeed(150);
    m_system->SetMaxItersSolverStab(150);
    m_system->SetMaxPenetrationRecoverySpeed(4.0);
    m_system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // Create components
    Create(fixed);
}

RoboSimian::RoboSimian(ChSystem* system, bool fixed) : m_owns_system(false), m_system(system) {
    // Create components
    Create(fixed);
}

void RoboSimian::Create(bool fixed) {
    m_chassis = std::make_shared<Chassis>("chassis", m_system, fixed);
}

void RoboSimian::Initialize(const ChCoordsys<>& pos) {
    m_chassis->Initialize(pos);
}

void RoboSimian::SetVisualizationTypeChassis(VisualizationType vis) {
    m_chassis->SetVisualizationType(vis);
}

void RoboSimian::SetVisualizationTypeLimb(LimbID which, VisualizationType vis) {
    m_limbs[which]->SetVisualizationType(vis);
}

RoboSimian::~RoboSimian() {}

// =============================================================================

Part::Part(const std::string& name) : m_name(name) {}

void Part::SetVisualizationType(VisualizationType vis) {
    RemoveVisualizationAssets();
    AddVisualizationAssets(vis);
}

// =============================================================================

Chassis::Chassis(const std::string& name, ChSystem* system, bool fixed) : Part(name) {
    m_body = std::shared_ptr<ChBodyAuxRef>(system->NewBodyAuxRef());
    m_body->SetIdentifier(0);
    m_body->SetNameString("chassis_body");
    m_body->SetMass(m_mass);
    m_body->SetFrame_COG_to_REF(ChFrame<>(m_COM, ChQuaternion<>(1, 0, 0, 0)));
    m_body->SetInertiaXX(m_inertia_xx);
    m_body->SetInertiaXY(m_inertia_xy);
    m_body->SetBodyFixed(fixed);
    system->Add(m_body);

    // Create the set of primitive shapes
    m_boxes.push_back(BoxShape(VNULL, QUNIT, ChVector<>(0.257, 0.50, 0.238)));
    m_boxes.push_back(BoxShape(VNULL, QUNIT, ChVector<>(0.93, 0.230, 0.238)));
    m_boxes.push_back(
        BoxShape(ChVector<>(+0.25393, +0.075769, 0), Q_from_AngZ(-0.38153), ChVector<>(0.36257, 0.23, 0.238)));
    m_boxes.push_back(
        BoxShape(ChVector<>(-0.25393, +0.075769, 0), Q_from_AngZ(+0.38153), ChVector<>(0.36257, 0.23, 0.238)));
    m_boxes.push_back(
        BoxShape(ChVector<>(+0.25393, -0.075769, 0), Q_from_AngZ(+0.38153), ChVector<>(0.36257, 0.23, 0.238)));
    m_boxes.push_back(
        BoxShape(ChVector<>(-0.25393, -0.075769, 0), Q_from_AngZ(-0.38153), ChVector<>(0.36257, 0.23, 0.238)));

    m_cylinders.push_back(CylinderShape(ChVector<>(0.417050, 0, -0.158640),
                                        Q_from_AngZ(CH_C_PI_2) * Q_from_AngX(CH_C_PI_2 - 0.383972), 0.05, 0.144));
}

void Chassis::SetCollide(bool val) {
    m_body->SetCollide(val);
}

void Chassis::Initialize(const ChCoordsys<>& pos) {
    m_body->SetFrame_REF_to_abs(ChFrame<>(pos));
    ////m_body->SetPos_dt(fwd_vel * pos.TransformDirectionLocalToParent(ChVector<>(1, 0, 0)));

    //// TODO:
    //// - set contact material properties

    m_body->GetCollisionModel()->ClearModel();

    ////m_body->GetCollisionModel()->SetFamily(collision_family);

    for (auto sphere : m_spheres) {
        m_body->GetCollisionModel()->AddSphere(sphere.m_radius, sphere.m_pos);
    }
    for (auto box : m_boxes) {
        ChVector<> hdims = box.m_dims / 2;
        m_body->GetCollisionModel()->AddBox(hdims.x(), hdims.y(), hdims.z(), box.m_pos, box.m_rot);
    }
    for (auto cyl : m_cylinders) {
        m_body->GetCollisionModel()->AddCylinder(cyl.m_radius, cyl.m_radius, cyl.m_length / 2, cyl.m_pos, cyl.m_rot);
    }

    m_body->GetCollisionModel()->BuildModel();
}

void Chassis::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.4f, 0.4f, 0.7f));
    m_body->AddAsset(col);

    if (vis == VisualizationType::MESH) {
        std::string vis_mesh_file = "robosimian/obj/" + m_vis_mesh_name + ".obj";
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(GetChronoDataFile(vis_mesh_file), false, false);
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_vis_mesh_name);
        m_body->AddAsset(trimesh_shape);
        return;
    }

    for (auto box : m_boxes) {
        auto box_shape = std::make_shared<ChBoxShape>();
        box_shape->GetBoxGeometry().SetLengths(box.m_dims);
        box_shape->Pos = box.m_pos;
        box_shape->Rot = box.m_rot;
        m_body->AddAsset(box_shape);
    }

    for (auto cyl : m_cylinders) {
        auto cyl_shape = std::make_shared<ChCylinderShape>();
        cyl_shape->GetCylinderGeometry().rad = cyl.m_radius;
        cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, cyl.m_length / 2, 0);
        cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, -cyl.m_length / 2, 0);
        cyl_shape->Pos = cyl.m_pos;
        cyl_shape->Rot = cyl.m_rot;
        m_body->AddAsset(cyl_shape);
    }

    for (auto sphere : m_spheres) {
        auto sphere_shape = std::make_shared<ChSphereShape>();
        sphere_shape->GetSphereGeometry().rad = sphere.m_radius;
        sphere_shape->Pos = sphere.m_pos;
        m_body->AddAsset(sphere_shape);
    }
}

void Chassis::RemoveVisualizationAssets() {
    m_body->GetAssets().clear();
}

// =============================================================================

Limb::Limb(const std::string& name, ChSystem* system) : Part(name) {}

void Limb::Initialize() {}

ChColor Limb::GetColor(size_t index) {
    if (index % 2 == 0)
        return ChColor(0.7f, 0.4f, 0.4f);
    else
        return ChColor(0.4f, 0.7f, 0.4f);
}

void Limb::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;
}

void Limb::RemoveVisualizationAssets() {}

}  // end namespace robosimian
