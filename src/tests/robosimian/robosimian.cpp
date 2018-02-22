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
#include "chrono/motion_functions/ChFunction_Setpoint.h"

#include "robosimian.h"

using namespace chrono;

namespace robosimian {

// =============================================================================

// Concrete Link types
//     mesh_name, mass, com, inertia_xx, inertia_xy, shapes

const Link FtsLink("robosim_fts",
                   ChVector<>(0, 0, 0),
                   ChColor(1.0f, 0.0f, 0.0f),
                   0.0,
                   ChVector<>(0, 0, 0),
                   ChVector<>(0, 0, 0),
                   ChVector<>(0, 0, 0),
                   {});

const Link PitchLink("robosim_pitch_link",
                     ChVector<>(0, 0, 0),
                     ChColor(0.4f, 0.7f, 0.4f),
                     0.429,
                     ChVector<>(-0.050402, 0.012816, 0.000000),
                     ChVector<>(0.000741, 0.002044, 0.001885),
                     ChVector<>(-0.000215, 0.000000, 0.000000),
                     {CylinderShape(ChVector<>(-0.07, 0, 0), Q_from_AngZ(CH_C_PI_2), 0.055, 0.07)});

const Link RollLink("robosim_roll_link",
                    ChVector<>(0, 0, 0),
                    ChColor(0.7f, 0.4f, 0.4f),
                    3.975000,
                    ChVector<>(0.067354, -0.096472, 0.000262),
                    ChVector<>(0.044712, 0.033445, 0.074293),
                    ChVector<>(-0.030354, 0.000012, -0.000141),
                    {CylinderShape(ChVector<>(0.065, -0.12, 0), Q_from_AngZ(CH_C_PI_2), 0.055, 0.24),
                     CylinderShape(ChVector<>(0.0, -0.035, 0), QUNIT, 0.055, 0.075)});

const Link RollLinkLast("robosim_roll_link",
                        ChVector<>(0, 0, 0),
                        ChColor(0.7f, 0.4f, 0.4f),
                        3.975000,
                        ChVector<>(0.067354, -0.096472, 0.000262),
                        ChVector<>(0.044712, 0.033445, 0.074293),
                        ChVector<>(-0.030354, 0.000012, -0.000141),
                        {CylinderShape(ChVector<>(0.105, -0.12, 0), Q_from_AngZ(CH_C_PI_2), 0.055, 0.32),
                         CylinderShape(ChVector<>(0.0, -0.035, 0), QUNIT, 0.055, 0.075)});

const Link RollLinkLastWheel("robosim_roll_link_w_wheel",
                             ChVector<>(0, 0, 0),
                             ChColor(0.7f, 0.4f, 0.4f),
                             3.975000,
                             ChVector<>(0.067354, -0.096472, 0.000262),
                             ChVector<>(0.044712, 0.033445, 0.074293),
                             ChVector<>(-0.030354, 0.000012, -0.000141),
                             {CylinderShape(ChVector<>(0.105, -0.12, 0), Q_from_AngZ(CH_C_PI_2), 0.055, 0.32),
                              CylinderShape(ChVector<>(0.0, -0.035, 0), QUNIT, 0.055, 0.075),
                              CylinderShape(ChVector<>(0.0, -0.19, 0), QUNIT, 0.080, 0.0375)});

const Link FtAdapterLink("robosim_ft_adapter",
                         ChVector<>(0, 0, 0),
                         ChColor(0.4f, 0.4f, 0.4f),
                         3.975000,
                         ChVector<>(0.067354, -0.096472, 0.000262),
                         ChVector<>(0.044712, 0.033445, 0.074293),
                         ChVector<>(-0.030354, 0.000012, -0.000141),
                         {});

const Link FtLink("robosim_force_torque_sensor",
                  ChVector<>(0, 0, 0),
                  ChColor(0.4f, 0.4f, 0.4f),
                  3.975000,
                  ChVector<>(0.067354, -0.096472, 0.000262),
                  ChVector<>(0.044712, 0.033445, 0.074293),
                  ChVector<>(-0.030354, 0.000012, -0.000141),
                  {});

const Link WheelMountLink("robosim_wheel_mount",
                          ChVector<>(0.12024, 0, 0),
                          ChColor(0.4f, 0.7f, 0.4f),
                          1.760000,
                          ChVector<>(0.112290, -0.000164, 0.000000),
                          ChVector<>(0.001913, 0.006680, 0.006125),
                          ChVector<>(-0.000007, 0.000000, 0.000000),
                          {CylinderShape(ChVector<>(0.12024, 0.02, 0), QUNIT, 0.0545, 0.175)});

const Link WheelLink("robosim_wheel",
                     ChVector<>(0, 0, 0),
                     ChColor(0.3f, 0.3f, 0.3f),
                     1.760000,
                     ChVector<>(0.112290, -0.000164, 0.000000),
                     ChVector<>(0.001913, 0.006680, 0.006125),
                     ChVector<>(-0.000007, 0.000000, 0.000000),
                     {CylinderShape(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2), 0.12, 0.123)});

// List of links for front and rear legs
//     name, link, body included?

const int num_links = 11;

const LinkData front_links[] = {
    {"link0", FtsLink, false},                //
    {"link1", PitchLink, true},               //
    {"link2", RollLink, true},                //
    {"link3", PitchLink, true},               //
    {"link4", RollLink, true},                //
    {"link5", PitchLink, true},               //
    {"link6", RollLinkLast, true},            //
    {"ftadapter_link", FtAdapterLink, true},  //
    {"ft_link", FtLink, true},                //
    {"link7", WheelMountLink, true},          //
    {"link8", WheelLink, true}                //
};

const LinkData rear_links[] = {
    {"link0", FtsLink, false},                //
    {"link1", PitchLink, true},               //
    {"link2", RollLink, true},                //
    {"link3", PitchLink, true},               //
    {"link4", RollLink, true},                //
    {"link5", PitchLink, true},               //
    {"link6", RollLinkLastWheel, true},       //
    {"ftadapter_link", FtAdapterLink, true},  //
    {"ft_link", FtLink, true},                //
    {"link7", WheelMountLink, true},          //
    {"link8", WheelLink, true}                //
};

// List of joints in a limb chain
//     name, parent_link, child_link, fixed?, xyz, rpy, axis

const int num_joints = 10;

const JointData joints[] = {

    { "joint1", "link0", "link1", false, ChVector<>(0.17203, 0.00000, 0.00000),
    ChVector<>(3.14159, 0.00000, 0.00000), ChVector<>(1, 0, 0) },

    { "joint2", "link1", "link2", false, ChVector<>(0.00000, 0.00000, 0.00000),
    ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(0, -1, 0) },

    { "joint3", "link2", "link3", false, ChVector<>(0.28650, -0.11700, 0.00000),
    ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(1, 0, 0) },

    { "joint4", "link3", "link4", false, ChVector<>(0.00000, 0.00000, 0.00000),
    ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(0, -1, 0) },

    { "joint5", "link4", "link5", false, ChVector<>(0.28650, -0.11700, 0.00000),
    ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(1, 0, 0) },

    { "joint6", "link5", "link6", false, ChVector<>(0.00000, 0.00000, 0.00000),
    ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(0, -1, 0) },

    { "ftadapter_joint", "link6", "ftadapter_link", true, ChVector<>(0.20739, -0.12100, 0.00000),
    ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(1, 0, 0) },

    { "ft_joint", "ftadapter_link", "ft_link", true, ChVector<>(0.0263755, 0.00000, 0.00000),
    ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(1, 0, 0) },

    { "joint7", "link6", "link7", false, ChVector<>(0.19250, -0.11700, 0.00000),
    ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(1, 0, 0) },

    { "joint8", "link7", "link8", false, ChVector<>(0.12024, 0.17200, 0.00000),
    ChVector<>(-1.57000, 0.00000, 0.00000), ChVector<>(0, 0, 1) }

};

// =============================================================================

// Convert a triplet (roll-pitch-yaw) to a quaternion
ChQuaternion<> rpy2quat(const ChVector<>& rpy) {
    return Q_from_AngZ(rpy.z()) * Q_from_AngY(rpy.y()) * Q_from_AngX(rpy.x());
}

// Calculate a coordinate system with the Z direction along 'axis' (given in 'base').
// Implicit assumption: 'axis' is always along X, Y, or Z.
ChCoordsys<> calcJointFrame(const ChFrame<>& base, const ChVector<>& axis) {
    ChVector<> u;
    ChVector<> v;
    ChVector<> w = axis;
    if (std::abs(axis.x()) > 0.5) {
        v = ChVector<>(0, 1, 0);
        u = Vcross(v, w);
    } else {
        u = ChVector<>(1, 0, 0);
        v = Vcross(w, u);
    }
    ChMatrix33<> A;
    A.Set_A_axis(u, v, w);
    ChMatrix33<> B = base.GetA() * A;
    return ChCoordsys<>(base.GetPos(), B.Get_A_quaternion());
}

// =============================================================================

class ContactManager : public ChContactContainer::ReportContactCallback {
  public:
    ContactManager();
    void Process(RoboSimian* robot);

  private:
    /// Callback, used to report contact points already added to the container.
    /// If it returns false, the contact scanning will be stopped.
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const ChVector<>& react_forces,
                                 const ChVector<>& react_torques,
                                 ChContactable* modA,
                                 ChContactable* modB) override;

    int m_num_contacts;
};

ContactManager::ContactManager() {}

void ContactManager::Process(RoboSimian* robot) {
    std::cout << "Report contacts" << std::endl;
    m_num_contacts = 0;
    robot->GetSystem()->GetContactContainer()->ReportAllContacts(this);
    std::cout << "  total actual contacts: " << m_num_contacts << std::endl << std::endl;
}

bool ContactManager::OnReportContact(const ChVector<>& pA,
                                     const ChVector<>& pB,
                                     const ChMatrix33<>& plane_coord,
                                     const double& distance,
                                     const ChVector<>& react_forces,
                                     const ChVector<>& react_torques,
                                     ChContactable* modA,
                                     ChContactable* modB) {
    // Only report contacts with negative penetration (i.e. actual contacts).
    if (distance >= 0)
        return true;

    auto bodyA = dynamic_cast<ChBodyAuxRef*>(modA);
    auto bodyB = dynamic_cast<ChBodyAuxRef*>(modB);

    // Filter robot bodies based on their IDs.
    bool a = (bodyA && bodyA->GetId() < 100);
    bool b = (bodyB && bodyB->GetId() < 100);

    if (!a && !b)
        return true;

    std::cout << "   " << (a ? bodyA->GetNameString() : "other") << " - " << (b ? bodyB->GetNameString() : "other")
              << std::endl;

    m_num_contacts++;

    return true;
}

// =============================================================================

RoboSimian::RoboSimian(ChMaterialSurface::ContactMethod contact_method, bool fixed)
    : m_owns_system(true), m_mode(ActuationMode::ANGLE), m_contacts(new ContactManager) {
    m_system = (contact_method == ChMaterialSurface::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                          : static_cast<ChSystem*>(new ChSystemSMC);
    m_system->Set_G_acc(ChVector<>(0, 0, -9.81));

    // Integration and Solver settings
    m_system->SetMaxItersSolverSpeed(150);
    m_system->SetMaxItersSolverStab(150);
    m_system->SetMaxPenetrationRecoverySpeed(4.0);
    m_system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    Create(fixed);
}

RoboSimian::RoboSimian(ChSystem* system, bool fixed)
    : m_owns_system(false), m_system(system), m_mode(ActuationMode::ANGLE), m_contacts(new ContactManager) {
    Create(fixed);
}

RoboSimian::~RoboSimian() {
    delete m_contacts;
}

void RoboSimian::Create(bool fixed) {
    // Set default collision model envelope commensurate with model dimensions.
    // Note that an SMC system automatically sets envelope to 0.
    if (m_system->GetContactMethod() == ChMaterialSurface::NSC) {
        collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
        collision::ChCollisionModel::SetDefaultSuggestedMargin(0.005);
    }

    m_chassis = std::make_shared<Chassis>("chassis", m_system, fixed);

    m_limbs.push_back(std::make_shared<Limb>("limb1", FR, front_links, m_system));
    m_limbs.push_back(std::make_shared<Limb>("limb2", RR, rear_links, m_system));
    m_limbs.push_back(std::make_shared<Limb>("limb3", RL, rear_links, m_system));
    m_limbs.push_back(std::make_shared<Limb>("limb4", FL, front_links, m_system));

    // The differential-drive wheels will be removed from robosimian
    ////m_wheel_left = std::make_shared<WheelDD>("dd_wheel_left", 1, m_system);
    ////m_wheel_right = std::make_shared<WheelDD>("dd_wheel_right", 2, m_system);

    // Default visualization: PRIMITIVES
    SetVisualizationTypeChassis(VisualizationType::PRIMITIVES);
    SetVisualizationTypeLimbs(VisualizationType::PRIMITIVES);
    SetVisualizationTypeWheels(VisualizationType::PRIMITIVES);
}

void RoboSimian::Initialize(const ChCoordsys<>& pos) {
    m_chassis->Initialize(pos);

    m_limbs[FR]->Initialize(m_chassis->m_body, ChVector<>(+0.29326, +0.20940, 0.03650),
                            ChVector<>(0.00000, -1.57080, -0.26180), m_mode);
    m_limbs[RR]->Initialize(m_chassis->m_body, ChVector<>(-0.29326, +0.20940, 0.03650),
                            ChVector<>(0.00000, -1.57080, +0.26180), m_mode);
    m_limbs[RL]->Initialize(m_chassis->m_body, ChVector<>(-0.29326, -0.20940, 0.03650),
                            ChVector<>(0.00000, -1.57080, 2.87979), m_mode);
    m_limbs[FL]->Initialize(m_chassis->m_body, ChVector<>(+0.29326, -0.20940, 0.03650),
                            ChVector<>(0.00000, -1.57080, 3.40339), m_mode);

    ////m_wheel_left->Initialize(m_chassis->m_body, ChVector<>(-0.42943, -0.19252, 0.06380),
    ////                         ChVector<>(0.00000, +1.57080, -1.57080));
    ////m_wheel_right->Initialize(m_chassis->m_body, ChVector<>(-0.42943, +0.19252, 0.06380),
    ////                          ChVector<>(0.00000, -1.57080, -1.57080));
}

void RoboSimian::SetVisualizationTypeChassis(VisualizationType vis) {
    m_chassis->SetVisualizationType(vis);
}

void RoboSimian::SetVisualizationTypeLimb(LimbID id, VisualizationType vis) {
    m_limbs[id]->SetVisualizationType(vis);
}

void RoboSimian::SetVisualizationTypeLimbs(VisualizationType vis) {
    for (auto limb : m_limbs)
        limb->SetVisualizationType(vis);
}

void RoboSimian::SetVisualizationTypeWheels(VisualizationType vis) {
    ////m_wheel_left->SetVisualizationType(vis);
    ////m_wheel_right->SetVisualizationType(vis);
}

void RoboSimian::Activate(LimbID id, const std::string& motor_name, double time, double val) {
    m_limbs[id]->Activate(motor_name, time, val);
}

void RoboSimian::ReportContacts() {
    m_contacts->Process(this);
}

// =============================================================================

Part::Part(const std::string& name, ChSystem* system) : m_name(name) {
    m_body = std::shared_ptr<ChBodyAuxRef>(system->NewBodyAuxRef());
    m_body->SetNameString(name + "_body");
}

void Part::SetVisualizationType(VisualizationType vis) {
    m_body->GetAssets().clear();
    AddVisualizationAssets(vis);
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
        //// HACK: a trimesh visual asset ignores transforms! Explicitly offset vertices.
        trimesh.Transform(m_offset, ChMatrix33<>(1));
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_mesh_name);
        ////trimesh_shape->Pos = m_offset;
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
        //// HACK: Chrono::OpenGL does not properly account for Pos & Rot
        ChCoordsys<> csys(cyl.m_pos, cyl.m_rot);
        ChVector<> p1 = csys * ChVector<>(0, cyl.m_length / 2, 0);
        ChVector<> p2 = csys * ChVector<>(0, -cyl.m_length / 2, 0);
        auto cyl_shape = std::make_shared<ChCylinderShape>();
        cyl_shape->GetCylinderGeometry().rad = cyl.m_radius;
        cyl_shape->GetCylinderGeometry().p1 = p1;
        cyl_shape->GetCylinderGeometry().p2 = p2;
        m_body->AddAsset(cyl_shape);
    }

    for (auto sphere : m_spheres) {
        auto sphere_shape = std::make_shared<ChSphereShape>();
        sphere_shape->GetSphereGeometry().rad = sphere.m_radius;
        sphere_shape->Pos = sphere.m_pos;
        m_body->AddAsset(sphere_shape);
    }
}

void Part::AddCollisionShapes(int collision_family) {
    m_body->GetCollisionModel()->ClearModel();

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

    // Note: collision_family is either 0 or 1
    m_body->GetCollisionModel()->SetFamily(collision_family);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1 - collision_family);

    // Note: call this AFTER setting the collision family (required for Chrono::Parallel)
    m_body->SetCollide(true);
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
    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);

    //// TODO: set contact material properties
}

void Chassis::Initialize(const ChCoordsys<>& pos) {
    m_body->SetFrame_REF_to_abs(ChFrame<>(pos));

    AddCollisionShapes(0);
}

// =============================================================================

WheelDD::WheelDD(const std::string& name, int id, chrono::ChSystem* system) : Part(name, system) {
    double mass = 3.492500;
    ChVector<> com(0, 0, 0);
    ChVector<> inertia_xx(0.01, 0.01, 0.02);
    ChVector<> inertia_xy(0, 0, 0);

    m_body->SetIdentifier(id);
    m_body->SetMass(mass);
    m_body->SetFrame_COG_to_REF(ChFrame<>(com, ChQuaternion<>(1, 0, 0, 0)));
    m_body->SetInertiaXX(inertia_xx);
    m_body->SetInertiaXY(inertia_xy);
    system->Add(m_body);

    m_cylinders.push_back(CylinderShape(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2), 0.074, 0.038));

    m_mesh_name = "robosim_dd_wheel";
    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.3f, 0.3f, 0.3f);

    //// TODO: set contact material properties
}

void WheelDD::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector<>& xyz, const ChVector<>& rpy) {
    const ChFrame<>& X_GP = chassis->GetFrame_REF_to_abs();  // global -> parent
    ChFrame<> X_PC(xyz, rpy2quat(rpy));                      // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                            // global -> child
    m_body->SetFrame_REF_to_abs(X_GC);

    AddCollisionShapes(1);

    // Add joint
    auto joint = std::make_shared<ChLinkLockRevolute>();
    joint->Initialize(chassis, m_body, calcJointFrame(X_GC, ChVector<>(0, 0, 1)));
    chassis->GetSystem()->AddLink(joint);
}

// =============================================================================

Limb::Limb(const std::string& name, LimbID id, const LinkData data[], ChSystem* system) : m_name(name), m_id(id) {
    for (int i = 0; i < num_links; i++) {
        auto link = std::make_shared<Part>(m_name + "_" + data[i].name, system);

        link->m_body->SetIdentifier(3 + 4 * id + i);
        link->m_body->SetMass(data[i].link.m_mass);
        link->m_body->SetFrame_COG_to_REF(ChFrame<>(data[i].link.m_com, ChQuaternion<>(1, 0, 0, 0)));
        link->m_body->SetInertiaXX(data[i].link.m_inertia_xx);
        link->m_body->SetInertiaXY(data[i].link.m_inertia_xy);

        link->m_mesh_name = data[i].link.m_mesh_name;
        link->m_offset = data[i].link.m_offset;
        link->m_color = data[i].link.m_color;

        for (auto cyl : data[i].link.m_shapes) {
            link->m_cylinders.push_back(cyl);
        }

        if (data[i].include)
            system->Add(link->m_body);

        //// TODO: set contact material properties

        m_links.insert(std::make_pair(data[i].name, link));
    }
}

void Limb::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector<>& xyz, const ChVector<>& rpy, ActuationMode mode) {
    // Set absolute position of link0
    auto parent_body = chassis;                                  // parent body
    auto child_body = m_links.find("link0")->second->m_body;     // child body
    const ChFrame<>& X_GP = parent_body->GetFrame_REF_to_abs();  // global -> parent
    ChFrame<> X_PC(xyz, rpy2quat(rpy));                          // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                                // global -> child
    child_body->SetFrame_REF_to_abs(X_GC);

    // Traverse chain (base-to-tip)
    //   set absolute position of the child body
    //   add collision shapes on child body
    //   create joint between parent and child
    for (int i = 0; i < num_joints; i++) {
        auto parent = m_links.find(joints[i].linkA)->second;         // parent Part
        auto child = m_links.find(joints[i].linkB)->second;          // child Part
        auto parent_body = parent->m_body;                           // parent body
        auto child_body = child->m_body;                             // child body
        const ChFrame<>& X_GP = parent_body->GetFrame_REF_to_abs();  // global -> parent
        ChFrame<> X_PC(joints[i].xyz, rpy2quat(joints[i].rpy));      // parent -> child
        ChFrame<> X_GC = X_GP * X_PC;                                // global -> child
        child_body->SetFrame_REF_to_abs(X_GC);

        // First joint connects directly to chassis
        if (i == 0)
            parent_body = chassis;

        // Place parent and child bodies in different collision families
        int family = (i + 1) % 2;
        child->AddCollisionShapes(family);

        // If the current joint is fixed, create a lock-lock joint
        if (joints[i].fixed) {
            auto joint = std::make_shared<ChLinkLockLock>();
            joint->SetNameString(m_name + "_" + joints[i].name);
            joint->Initialize(parent_body, child_body, calcJointFrame(X_GC, joints[i].axis));
            chassis->GetSystem()->AddLink(joint);
            m_joints.insert(std::make_pair(joints[i].name, joint));        
            continue;
        }
        
        ////auto joint = std::make_shared<ChLinkLockRevolute>();
        ////joint->SetNameString(m_name + "_" + joints[i].name);
        ////joint->Initialize(parent_body, child_body, calcJointFrame(X_GC, joints[i].axis));
        ////chassis->GetSystem()->AddLink(joint);
        ////m_joints.insert(std::make_pair(joints[i].name, joint));
        ////continue;

        // Create a motor (for now, ignore 'mode')
        auto motor_fun = std::make_shared<ChFunction_Setpoint>();

        auto joint = std::make_shared<ChLinkMotorRotationAngle>();
        joint->SetNameString(m_name + "_" + joints[i].name);
        joint->Initialize(parent_body, child_body, ChFrame<>(calcJointFrame(X_GC, joints[i].axis)));
        joint->SetAngleFunction(motor_fun);
        chassis->GetSystem()->AddLink(joint);
        m_joints.insert(std::make_pair(joints[i].name, joint));
        m_motors.insert(std::make_pair(joints[i].name, joint));
    }
}

void Limb::SetVisualizationType(VisualizationType vis) {
    for (auto link : m_links)
        link.second->SetVisualizationType(vis);
}

void Limb::Activate(const std::string& motor_name, double time, double val) {
    auto itr = m_motors.find(motor_name);
    if (itr == m_motors.end())
        return;

    // Note: currently hard-coded for angle motor
    auto motor = std::static_pointer_cast<ChLinkMotorRotationAngle>(itr->second);
    auto fun = std::static_pointer_cast<ChFunction_Setpoint>(motor->GetAngleFunction());
    fun->SetSetpoint(val, time);
}

}  // end namespace robosimian
