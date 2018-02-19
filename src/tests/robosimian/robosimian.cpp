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

const Link FtsLink("robosim_fts", 0.0, ChVector<>(0, 0, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 0), {});

const Link PitchLink("robosim_pitch_link",
                     0.429,
                     ChVector<>(-0.050402, 0.012816, 0.000000),
                     ChVector<>(0.000741, 0.002044, 0.001885),
                     ChVector<>(-0.000215, 0.000000, 0.000000),
                     {CylinderShape(ChVector<>(-0.07, 0, 0), Q_from_AngZ(CH_C_PI_2), 0.055, 0.07)});

const Link RollLink("robosim_roll_link",
                    3.975000,
                    ChVector<>(0.067354, -0.096472, 0.000262),
                    ChVector<>(0.044712, 0.033445, 0.074293),
                    ChVector<>(-0.030354, 0.000012, -0.000141),
                    {CylinderShape(ChVector<>(0.065, -0.12, 0), Q_from_AngZ(CH_C_PI_2), 0.055, 0.24),
                     CylinderShape(ChVector<>(0.0, -0.035, 0), QUNIT, 0.055, 0.075)});

const Link RollLinkLast("robosim_roll_link",
                        3.975000,
                        ChVector<>(0.067354, -0.096472, 0.000262),
                        ChVector<>(0.044712, 0.033445, 0.074293),
                        ChVector<>(-0.030354, 0.000012, -0.000141),
                        {CylinderShape(ChVector<>(0.105, -0.12, 0), Q_from_AngZ(CH_C_PI_2), 0.055, 0.32),
                         CylinderShape(ChVector<>(0.0, -0.035, 0), QUNIT, 0.055, 0.075)});

const Link RollLinkLastWheel("robosim_roll_link_w_wheel",
                             3.975000,
                             ChVector<>(0.067354, -0.096472, 0.000262),
                             ChVector<>(0.044712, 0.033445, 0.074293),
                             ChVector<>(-0.030354, 0.000012, -0.000141),
                             {CylinderShape(ChVector<>(0.105, -0.12, 0), Q_from_AngZ(CH_C_PI_2), 0.055, 0.32),
                              CylinderShape(ChVector<>(0.0, -0.035, 0), QUNIT, 0.055, 0.075),
                              CylinderShape(ChVector<>(0.0, -0.19, 0), QUNIT, 0.080, 0.0375)});

const int num_links = 7;

const LinkData front_links[] = {
    {"link0", FtsLink, false},     //
    {"link1", PitchLink, true},    //
    {"link2", RollLink, true},     //
    {"link3", PitchLink, true},    //
    {"link4", RollLink, true},     //
    {"link5", PitchLink, true},    //
    {"link6", RollLinkLast, true}  //
};

const LinkData rear_links[] = {
    {"link0", FtsLink, false},          //
    {"link1", PitchLink, true},         //
    {"link2", RollLink, true},          //
    {"link3", PitchLink, true},         //
    {"link4", RollLink, true},          //
    {"link5", PitchLink, true},         //
    {"link6", RollLinkLastWheel, true}  //
};

const int num_joints = 6;

// {name, parent_link, child_link, fixed?, actuated?, xyz, rpy, axis}
const JointData joints[] = {

    {"joint1", "link0", "link1", false, true, ChVector<>(0.17203, 0.00000, 0.00000),
     ChVector<>(3.14159, 0.00000, 0.00000), ChVector<>(1, 0, 0)},

    {"joint2", "link1", "link2", false, true, ChVector<>(0.00000, 0.00000, 0.00000),
     ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(0, -1, 0)},

    {"joint3", "link2", "link3", false, true, ChVector<>(0.28650, -0.11700, 0.00000),
     ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(1, 0, 0)},

    {"joint4", "link3", "link4", false, true, ChVector<>(0.00000, 0.00000, 0.00000),
     ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(0, -1, 0)},

    {"joint5", "link4", "link5", false, true, ChVector<>(0.28650, -0.11700, 0.00000),
     ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(1, 0, 0)},

    {"joint6", "link5", "link6", false, true, ChVector<>(0.00000, 0.00000, 0.00000),
     ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(0, -1, 0)}

};

// =============================================================================

ChQuaternion<> rpy2quat(const ChVector<>& rpy) {
    return Q_from_AngZ(rpy.z()) * Q_from_AngY(rpy.y()) * Q_from_AngX(rpy.x());
}

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
    m_limbs.push_back(std::make_shared<Limb>("limb1", FR, front_links, m_system));
    m_limbs.push_back(std::make_shared<Limb>("limb2", RR, rear_links, m_system));
    m_limbs.push_back(std::make_shared<Limb>("limb3", RL, rear_links, m_system));
    m_limbs.push_back(std::make_shared<Limb>("limb4", FL, front_links, m_system));
}

void RoboSimian::Initialize(const ChCoordsys<>& pos) {
    m_chassis->Initialize(pos);

    m_limbs[FR]->Initialize(m_chassis->m_body, ChVector<>(+0.29326, +0.20940, 0.03650),
                            ChVector<>(0.00000, -1.57080, -0.26180));
    m_limbs[RR]->Initialize(m_chassis->m_body, ChVector<>(-0.29326, +0.20940, 0.03650),
                            ChVector<>(0.00000, -1.57080, +0.26180));
    m_limbs[RL]->Initialize(m_chassis->m_body, ChVector<>(-0.29326, -0.20940, 0.03650),
                            ChVector<>(0.00000, -1.57080, 2.87979));
    m_limbs[FL]->Initialize(m_chassis->m_body, ChVector<>(+0.29326, -0.20940, 0.03650),
                            ChVector<>(0.00000, -1.57080, 3.40339));
}

void RoboSimian::SetVisualizationTypeChassis(VisualizationType vis) {
    m_chassis->SetVisualizationType(vis);
}

void RoboSimian::SetVisualizationTypeLimbs(VisualizationType vis) {
    for (auto limb : m_limbs)
        limb->SetVisualizationType(vis);
}

RoboSimian::~RoboSimian() {}

// =============================================================================

Part::Part(const std::string& name, ChSystem* system) : m_name(name) {
    m_body = std::shared_ptr<ChBodyAuxRef>(system->NewBodyAuxRef());
    m_body->SetNameString(name + "_body");
}

void Part::SetVisualizationType(VisualizationType vis) {
    m_body->GetAssets().clear();
    AddVisualizationAssets(vis);
}

void Part::SetCollide(bool val) {
    m_body->SetCollide(val);
}

void Part::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(m_color);
    m_body->AddAsset(col);

    if (vis == VisualizationType::MESH) {
        std::string vis_mesh_file = "robosimian/obj/" + m_mesh_name + ".obj";
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(GetChronoDataFile(vis_mesh_file), false, false);
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_mesh_name);
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

void Part::AddCollisionShapes() {
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

// =============================================================================

Chassis::Chassis(const std::string& name, ChSystem* system, bool fixed) : Part(name, system) {
    double mass = 40.0;
    ChVector<> com(-0.000820, -0.002060, -0.021800);
    ChVector<> inertia_xx(0.248970, 0.703230, 0.772560);
    ChVector<> inertia_xy(0, 0, 0.001820);

    m_body->SetIdentifier(0);
    m_body->SetMass(mass);
    m_body->SetFrame_COG_to_REF(ChFrame<>(com, ChQuaternion<>(1, 0, 0, 0)));
    m_body->SetInertiaXX(inertia_xx);
    m_body->SetInertiaXY(inertia_xy);
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

    // Geometry for link0 (all limbs); these links are fixed to the chassis
    m_cylinders.push_back(
        CylinderShape(ChVector<>(+0.29326, +0.20940, 0.03650 - 0.025), Q_from_AngX(CH_C_PI_2), 0.05, 0.145));
    m_cylinders.push_back(
        CylinderShape(ChVector<>(-0.29326, +0.20940, 0.03650 - 0.025), Q_from_AngX(CH_C_PI_2), 0.05, 0.145));
    m_cylinders.push_back(
        CylinderShape(ChVector<>(-0.29326, -0.20940, 0.03650 - 0.025), Q_from_AngX(CH_C_PI_2), 0.05, 0.145));
    m_cylinders.push_back(
        CylinderShape(ChVector<>(+0.29326, -0.20940, 0.03650 - 0.025), Q_from_AngX(CH_C_PI_2), 0.05, 0.145));

    // Set the name of the visualization mesh
    m_mesh_name = "robosim_chassis";
    m_color = ChColor(0.4f, 0.4f, 0.7f);
}

void Chassis::Initialize(const ChCoordsys<>& pos) {
    m_body->SetFrame_REF_to_abs(ChFrame<>(pos));
    ////m_body->SetPos_dt(fwd_vel * pos.TransformDirectionLocalToParent(ChVector<>(1, 0, 0)));

    //// TODO:
    //// - set contact material properties

    AddCollisionShapes();
}

// =============================================================================

Limb::Limb(const std::string& name, LimbID id, const LinkData data[], ChSystem* system) : m_id(id) {
    for (int i = 0; i < num_links; i++) {
        auto link = std::make_shared<Part>(name + "_" + data[i].name, system);
        link->m_body->SetIdentifier(4 * id + i);
        link->m_body->SetMass(data[i].link.m_mass);
        link->m_body->SetFrame_COG_to_REF(ChFrame<>(data[i].link.m_com, ChQuaternion<>(1, 0, 0, 0)));
        link->m_body->SetInertiaXX(data[i].link.m_inertia_xx);
        link->m_body->SetInertiaXY(data[i].link.m_inertia_xy);
        link->m_mesh_name = data[i].link.m_mesh_name;
        link->m_color = (i % 2 == 0 ? ChColor(0.7f, 0.4f, 0.4f) : ChColor(0.4f, 0.7f, 0.4f));

        //// TODO: create primitives
        for (auto cyl : data[i].link.m_shapes) {
            link->m_cylinders.push_back(cyl);
        }

        if (data[i].include)
            system->Add(link->m_body);

        m_links.insert(std::make_pair(data[i].name, link));
    }
}

void Limb::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector<>& xyz, const ChVector<>& rpy) {
    // Accumulate frame transform while moving down the chain.
    ChFrame<> frame = chassis->GetFrame_REF_to_abs();

    // Set absolute position of link0.
    auto parent_body = chassis;
    auto child_body = m_links.find("link0")->second->m_body;
    frame = frame * ChFrame<>(xyz, rpy2quat(rpy));
    child_body->SetFrame_REF_to_abs(frame);

    // Traverse chain
    for (int i = 0; i < num_joints; i++) {
        auto parent = m_links.find(joints[i].linkA)->second;
        auto child = m_links.find(joints[i].linkB)->second;
        auto parent_body = parent->m_body;
        auto child_body = child->m_body;
        ChFrame<> crt(joints[i].xyz, rpy2quat(joints[i].rpy));
        frame = frame * crt;
        child_body->SetFrame_REF_to_abs(frame);
        child->AddCollisionShapes();
    }
}

void Limb::SetVisualizationType(VisualizationType vis) {
    for (auto link : m_links)
        link.second->SetVisualizationType(vis);
}

}  // end namespace robosimian
