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
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono/motion_functions/ChFunction_Setpoint.h"

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "robosimian.h"

using namespace chrono;

namespace robosimian {

// =============================================================================

// Concrete Link types
//     mesh_name, offset, color, mass, com, inertia_xx, inertia_xy, shapes

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
                     0.428770,
                     ChVector<>(-0.050402, 0.012816, 0.000000),
                     ChVector<>(0.000670, 0.000955, 0.000726),
                     ChVector<>(0.000062, 0.000000, 0.000000),
                     {CylinderShape(ChVector<>(-0.07, 0, 0), Q_from_AngZ(CH_C_PI_2), 0.055, 0.07)});

const Link RollLink("robosim_roll_link",
                    ChVector<>(0, 0, 0),
                    ChColor(0.7f, 0.4f, 0.4f),
                    4.078540,
                    ChVector<>(0.066970, -0.090099, -0.000084),
                    ChVector<>(0.010580, 0.025014, 0.031182),
                    ChVector<>(-0.008765, -0.000002, 0.000007),
                    {CylinderShape(ChVector<>(0.065, -0.12, 0), Q_from_AngZ(CH_C_PI_2), 0.055, 0.24),
                     CylinderShape(ChVector<>(0.0, -0.035, 0), QUNIT, 0.055, 0.075)});

const Link RollLinkLast("robosim_roll_link",
                        ChVector<>(0, 0, 0),
                        ChColor(0.7f, 0.4f, 0.4f),
                        4.078540,
                        ChVector<>(0.066970, -0.090099, -0.000084),
                        ChVector<>(0.010580, 0.025014, 0.031182),
                        ChVector<>(-0.008765, -0.000002, 0.000007),
                        {CylinderShape(ChVector<>(0.105, -0.12, 0), Q_from_AngZ(CH_C_PI_2), 0.055, 0.32),
                         CylinderShape(ChVector<>(0.0, -0.035, 0), QUNIT, 0.055, 0.075)});

const Link RollLinkLastWheel("robosim_roll_link_w_wheel",
                             ChVector<>(0, 0, 0),
                             ChColor(0.7f, 0.4f, 0.4f),
                             4.078540,
                             ChVector<>(0.066970, -0.090099, -0.000084),
                             ChVector<>(0.010580, 0.025014, 0.031182),
                             ChVector<>(-0.008765, -0.000002, 0.000007),
                             {CylinderShape(ChVector<>(0.105, -0.12, 0), Q_from_AngZ(CH_C_PI_2), 0.055, 0.32),
                              CylinderShape(ChVector<>(0.0, -0.035, 0), QUNIT, 0.055, 0.075),
                              CylinderShape(ChVector<>(0.0, -0.19, 0), QUNIT, 0.080, 0.0375)});

const Link FtAdapterLink("robosim_ft_adapter",
                         ChVector<>(0, 0, 0),
                         ChColor(0.4f, 0.4f, 0.4f),
                         0.253735,
                         ChVector<>(-0.00531, -0.00060, -0.001873),
                         ChVector<>(0.00042, 0.00024, 0.00023),
                         ChVector<>(0, 0, 0),
                         {});

const Link FtLink("robosim_force_torque_sensor",
                  ChVector<>(0, 0, 0),
                  ChColor(0.4f, 0.4f, 0.4f),
                  0.195418,
                  ChVector<>(-0.000135, 0.000118, 0.000084),
                  ChVector<>(0.000086, 0.000056, 0.000057),
                  ChVector<>(0, 0, 0),
                  {});

const Link WheelMountLink("robosim_wheel_mount",
                          ChVector<>(0.12024, 0, 0),
                          ChColor(0.4f, 0.7f, 0.4f),
                          3.1775,
                          ChVector<>(-0.005260, 0.042308, 0.000088),
                          ChVector<>(0.010977, 0.005330, 0.011405),
                          ChVector<>(0.000702, 0.000026, -0.000028),
                          {CylinderShape(ChVector<>(0.12024, 0.02, 0), QUNIT, 0.0545, 0.175)});

const Link WheelLink("robosim_wheel",
                     ChVector<>(0, 0, 0),
                     ChColor(0.6f, 0.6f, 0.6f),
                     1.499326,
                     ChVector<>(0.0, 0.0, -0.000229),
                     ChVector<>(0.006378, 0.006377, 0.009155),
                     ChVector<>(0, 0, 0),
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

    {"joint1", "link0", "link1", ActuationMode::ANGLE, ChVector<>(0.17203, 0.00000, 0.00000),
     ChVector<>(3.14159, 0.00000, 0.00000), ChVector<>(1, 0, 0)},

    {"joint2", "link1", "link2", ActuationMode::ANGLE, ChVector<>(0.00000, 0.00000, 0.00000),
     ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(0, -1, 0)},

    {"joint3", "link2", "link3", ActuationMode::ANGLE, ChVector<>(0.28650, -0.11700, 0.00000),
     ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(1, 0, 0)},

    {"joint4", "link3", "link4", ActuationMode::ANGLE, ChVector<>(0.00000, 0.00000, 0.00000),
     ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(0, -1, 0)},

    {"joint5", "link4", "link5", ActuationMode::ANGLE, ChVector<>(0.28650, -0.11700, 0.00000),
     ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(1, 0, 0)},

    {"joint6", "link5", "link6", ActuationMode::ANGLE, ChVector<>(0.00000, 0.00000, 0.00000),
     ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(0, -1, 0)},

    {"ftadapter_joint", "link6", "ftadapter_link", ActuationMode::FIXED, ChVector<>(0.20739, -0.12100, 0.00000),
     ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(1, 0, 0)},

    {"ft_joint", "ftadapter_link", "ft_link", ActuationMode::FIXED, ChVector<>(0.0263755, 0.00000, 0.00000),
     ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(1, 0, 0)},

    {"joint7", "link6", "link7", ActuationMode::ANGLE, ChVector<>(0.19250, -0.11700, 0.00000),
     ChVector<>(0.00000, 0.00000, 0.00000), ChVector<>(1, 0, 0)},

    {"joint8", "link7", "link8", ActuationMode::SPEED, ChVector<>(0.12024, 0.17200, 0.00000),
     ChVector<>(-1.57000, 0.00000, 0.00000), ChVector<>(0, 0, 1)}

};

// =============================================================================

// Convert a triplet (roll-pitch-yaw) to a quaternion
ChQuaternion<> rpy2quat(const ChVector<>& rpy) {
    return Q_from_AngZ(rpy.z()) * Q_from_AngY(rpy.y()) * Q_from_AngX(rpy.x());
}

// =============================================================================

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

// Convert the specified inertia properties into the centroidal reference frame.
// It is assumed that the centroidal frame is parallel with the reference frame.
class InertiaConverter {
  public:
    InertiaConverter(double mass, const ChVector<>& com, const ChVector<>& inertia_xx, const ChVector<>& inertia_xy) {
        // Inertia matrix (wrt reference frame)
        ChMatrix33<> J(inertia_xx, inertia_xy);

        // Convert inertia to centroidal frame (parallel-axis theorem, no rotation)
        ChVector<> diag(com.y() * com.y() + com.z() * com.z(),  //
                        com.x() * com.x() + com.z() * com.z(),  //
                        com.x() * com.x() + com.y() * com.y());
        ChVector<> off_diag(-com.x() * com.y(),  //
                            -com.x() * com.z(),  //
                            -com.y() * com.z());
        ChMatrix33<> offset(diag, off_diag);

        ChMatrix33<> Jc = J - offset * mass;

        // Extract centroidal moments and products of inertia
        m_inertia_xx.x() = Jc.Get33Element(0, 0);
        m_inertia_xx.y() = Jc.Get33Element(1, 1);
        m_inertia_xx.z() = Jc.Get33Element(2, 2);

        m_inertia_xy.x() = Jc.Get33Element(0, 1);
        m_inertia_xy.y() = Jc.Get33Element(0, 2);
        m_inertia_xy.z() = Jc.Get33Element(1, 2);

        /*
        std::cout << mass <<                                                            // mass
            " " << com.x() << "  " << com.y() << " " << com.z() <<                      // COM offset
            " " << inertia_xx.x() << " " << inertia_xx.y() << " " << inertia_xx.z() <<  // moments (reference frame)
            " " << inertia_xy.x() << " " << inertia_xy.y() << " " << inertia_xy.z() <<  // products (reference frame)
            " " << m_inertia_xx.x() << " " << m_inertia_xx.y() << " " << m_inertia_xx.z() <<  // moments (centroidal)
            " " << m_inertia_xy.x() << " " << m_inertia_xy.y() << " " << m_inertia_xy.z() <<  // products (centroidal)
            std::endl;
        */
    }

    ChVector<> m_inertia_xx;  ///< moments of inertia (centroidal)
    ChVector<> m_inertia_xy;  ///< products of inertia (centroidal)
};

// =============================================================================

class ContactManager : public ChContactContainer::ReportContactCallback {
  public:
    ContactManager();
    void Process(RoboSimian* robot);

  private:
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
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
                                     const double& eff_radius,
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

RoboSimian::RoboSimian(ChMaterialSurface::ContactMethod contact_method, bool has_sled, bool fixed)
    : m_owns_system(true), m_contacts(new ContactManager) {
    m_system = (contact_method == ChMaterialSurface::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                          : static_cast<ChSystem*>(new ChSystemSMC);
    m_system->Set_G_acc(ChVector<>(0, 0, -9.81));

    // Integration and Solver settings
    m_system->SetMaxItersSolverSpeed(150);
    m_system->SetMaxItersSolverStab(150);
    m_system->SetMaxPenetrationRecoverySpeed(4.0);
    m_system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    Create(has_sled, fixed);
}

RoboSimian::RoboSimian(ChSystem* system, bool has_sled, bool fixed)
    : m_owns_system(false), m_system(system), m_contacts(new ContactManager) {
    Create(has_sled, fixed);
}

RoboSimian::~RoboSimian() {
    if (m_owns_system)
        delete m_system;
    delete m_contacts;
}

void RoboSimian::Create(bool has_sled, bool fixed) {
    // Set default collision model envelope commensurate with model dimensions.
    // Note that an SMC system automatically sets envelope to 0.
    if (m_system->GetContactMethod() == ChMaterialSurface::NSC) {
        collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
        collision::ChCollisionModel::SetDefaultSuggestedMargin(0.005);
    }

    m_chassis = std::make_shared<Chassis>("chassis", m_system, fixed);

    if (has_sled)
        m_sled = std::make_shared<Sled>("sled", m_system);

    m_limbs.push_back(std::make_shared<Limb>("limb1", FR, front_links, m_system));
    m_limbs.push_back(std::make_shared<Limb>("limb2", RR, rear_links, m_system));
    m_limbs.push_back(std::make_shared<Limb>("limb3", RL, rear_links, m_system));
    m_limbs.push_back(std::make_shared<Limb>("limb4", FL, front_links, m_system));

    // The differential-drive wheels will be removed from robosimian
    ////m_wheel_left = std::make_shared<WheelDD>("dd_wheel_left", 2, m_system);
    ////m_wheel_right = std::make_shared<WheelDD>("dd_wheel_right", 3, m_system);

    // Default visualization: COLLISION shapes
    SetVisualizationTypeChassis(VisualizationType::COLLISION);
    SetVisualizationTypeSled(VisualizationType::COLLISION);
    SetVisualizationTypeLimbs(VisualizationType::COLLISION);
    SetVisualizationTypeWheels(VisualizationType::COLLISION);
}

void RoboSimian::Initialize(const ChCoordsys<>& pos) {
    m_chassis->Initialize(pos);

    if (m_sled)
        m_sled->Initialize(m_chassis->m_body, ChVector<>(0.0, 0.0, 0.21), ChVector<>(1.570796, 0, 0));

    m_limbs[FR]->Initialize(m_chassis->m_body, ChVector<>(+0.29326, +0.20940, 0.03650),
                            ChVector<>(0.00000, -1.57080, -0.26180), CollisionFamily::LIMB_FR);
    m_limbs[RR]->Initialize(m_chassis->m_body, ChVector<>(-0.29326, +0.20940, 0.03650),
                            ChVector<>(0.00000, -1.57080, +0.26180), CollisionFamily::LIMB_RR);
    m_limbs[RL]->Initialize(m_chassis->m_body, ChVector<>(-0.29326, -0.20940, 0.03650),
                            ChVector<>(0.00000, -1.57080, 2.87979), CollisionFamily::LIMB_RL);
    m_limbs[FL]->Initialize(m_chassis->m_body, ChVector<>(+0.29326, -0.20940, 0.03650),
                            ChVector<>(0.00000, -1.57080, 3.40339), CollisionFamily::LIMB_FL);

    ////m_wheel_left->Initialize(m_chassis->m_body, ChVector<>(-0.42943, -0.19252, 0.06380),
    ////                         ChVector<>(0.00000, +1.57080, -1.57080));
    ////m_wheel_right->Initialize(m_chassis->m_body, ChVector<>(-0.42943, +0.19252, 0.06380),
    ////                          ChVector<>(0.00000, -1.57080, -1.57080));
}

void RoboSimian::SetCollide(int flags) {
    m_chassis->SetCollide((flags & static_cast<int>(CollisionFlags::CHASSIS)) != 0);

    if (m_sled)
        m_sled->SetCollide((flags & static_cast<int>(CollisionFlags::SLED)) != 0);

    for (auto limb : m_limbs) {
        limb->SetCollideLinks((flags & static_cast<int>(CollisionFlags::LIMBS)) != 0);
        limb->SetCollideWheel((flags & static_cast<int>(CollisionFlags::WHEELS)) != 0);
    }
}

void RoboSimian::SetVisualizationTypeChassis(VisualizationType vis) {
    m_chassis->SetVisualizationType(vis);
}

void RoboSimian::SetVisualizationTypeSled(VisualizationType vis) {
    if (m_sled)
        m_sled->SetVisualizationType(vis);
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

void RoboSimian::SetDriver(std::shared_ptr<Driver> driver) {
    m_driver = driver;
}

void RoboSimian::Activate(LimbID id, const std::string& motor_name, double time, double val) {
    m_limbs[id]->Activate(motor_name, time, val);
}

void RoboSimian::DoStepDynamics(double step) {
    if (m_driver) {
        // Update driver
        double time = m_system->GetChTime();
        m_driver->Update(time);

        // Get driver activations and apply to limbs
        Actuation actuation = m_driver->GetActuation();
        for (int i = 0; i < 4; i++)
            m_limbs[i]->Activate(time, actuation[i]);
    }

    // Advance system state
    m_system->DoStepDynamics(step);
}

void RoboSimian::ReportContacts() {
    m_contacts->Process(this);
}

// =============================================================================

class axpby {
  public:
    double operator()(const double& v1, const double& v2) { return a1 * v1 + a2 * v2; }
    double a1;
    double a2;
};

DriverFile::DriverFile(const std::string& filename, bool repeat) : m_repeat(repeat), m_offset(0) {
    m_ifstream.open(filename.c_str());
    LoadDataLine(m_time_1, m_actuations_1);
    LoadDataLine(m_time_2, m_actuations_2);
}

DriverFile::~DriverFile() {
}

void DriverFile::LoadDataLine(double& time, Actuation& activations) {
    m_ifstream >> time;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 8; j++) {
            m_ifstream >> activations[i][j];
        }
    }
}

void DriverFile::Update(double time) {
    // In the ease-in interval, return first data entry
    if (time < m_offset) {
        m_actuations = m_actuations_1;
        return;
    }

    // Offset time
    double t = time - m_offset;

    // Check if moving to new interval while checking for eof.
    // If reaching the end of file and we are in cycle mode, rewind input stream and update offset.
    // Otherwise, return to stop updates.
    while (t > m_time_2) {
        m_time_1 = m_time_2;
        m_actuations_1 = m_actuations_2;
        if (m_ifstream.eof()) {
            if (m_repeat) {
                m_ifstream.clear();
                m_ifstream.seekg(0);
                LoadDataLine(m_time_1, m_actuations_1);
                LoadDataLine(m_time_2, m_actuations_2);
                m_offset = time;
            }
            return;
        }
        LoadDataLine(m_time_2, m_actuations_2);
    }

    // Interpolate  v = alpha_1 * v_1 + alpha_2 * v_2
    axpby op;
    op.a1 = (t - m_time_2) / (m_time_1 - m_time_2);
    op.a2 = (t - m_time_1) / (m_time_2 - m_time_1);
    for (int i = 0; i < 4; i++) {
        std::transform(m_actuations_1[i].begin(), m_actuations_1[i].end(), m_actuations_2[i].begin(),
                       m_actuations[i].begin(), op);
    }
}

// ---------------

const std::string DriverFiles::m_phase_names[] = { "Pose", "Start", "Cycle", "Stop" };

DriverFiles::DriverFiles(const std::string& filename_start,
                         const std::string& filename_cycle,
                         const std::string& filename_stop,
                         bool repeat)
    : m_repeat(repeat), m_offset(0), m_phase(POSE) {
    assert(!filename_cycle.empty());
    m_ifs_cycle.open(filename_cycle.c_str());

    if (!filename_start.empty()) {
        m_ifs_start.open(filename_start.c_str());
        m_ifs = &m_ifs_start;
    } else {
        m_ifs = &m_ifs_cycle;
    }

    if (!filename_stop.empty()) {
        m_ifs_stop.open(filename_stop.c_str());
    }

    LoadDataLine(m_time_1, m_actuations_1);
    LoadDataLine(m_time_2, m_actuations_2);
}

DriverFiles::~DriverFiles() {
}

void DriverFiles::LoadDataLine(double& time, Actuation& activations) {
    *m_ifs >> time;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 8; j++) {
            *m_ifs >> activations[i][j];
        }
    }
}

void DriverFiles::Update(double time) {
    // In the POSE phase, always return the first data entry
    if (m_phase == POSE) {
        m_actuations = m_actuations_1;
        if (time >= m_offset) {
            if (m_ifs_start.is_open())
                m_phase = START;
            else
                m_phase = CYCLE;
            std::cout << "time = " << time << "  Switch to phase: " << GetCurrentPhase() << std::endl;
        }
        return;
    }

    // Offset time
    double t = time - m_offset;

    switch (m_phase) {
        case START:
            while (t > m_time_2) {
                m_time_1 = m_time_2;
                m_actuations_1 = m_actuations_2;
                if (!m_ifs->eof()) {
                    LoadDataLine(m_time_2, m_actuations_2);
                } else {
                    m_phase = CYCLE;
                    m_ifs = &m_ifs_cycle;
                    LoadDataLine(m_time_1, m_actuations_1);
                    LoadDataLine(m_time_2, m_actuations_2);
                    m_offset = time;
                    std::cout << "time = " << time << "  Switch to phase: " << GetCurrentPhase() << std::endl;
                    return;
                }
            }

            break;

        case CYCLE:
            while (t > m_time_2) {
                m_time_1 = m_time_2;
                m_actuations_1 = m_actuations_2;
                if (m_ifs->eof()) {
                    if (m_repeat) {
                        m_ifs->clear();
                        m_ifs->seekg(0);
                        LoadDataLine(m_time_1, m_actuations_1);
                        LoadDataLine(m_time_2, m_actuations_2);
                        m_offset = time;
                        std::cout << "time = " << time << " New cycle" << std::endl;
                    }
                    return;
                }
                LoadDataLine(m_time_2, m_actuations_2);
            }

            break;

        case STOP:
            //// TODO
            break;
    }

    // Interpolate  v = alpha_1 * v_1 + alpha_2 * v_2
    axpby op;
    op.a1 = (t - m_time_2) / (m_time_1 - m_time_2);
    op.a2 = (t - m_time_1) / (m_time_2 - m_time_1);
    for (int i = 0; i < 4; i++) {
        std::transform(m_actuations_1[i].begin(), m_actuations_1[i].end(), m_actuations_2[i].begin(),
            m_actuations[i].begin(), op);
    }
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
        trimesh.LoadWavefrontMesh(GetChronoDataFile(vis_mesh_file), true, false);
        //// HACK: a trimesh visual asset ignores transforms! Explicitly offset vertices.
        trimesh.Transform(m_offset, ChMatrix33<>(1));
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_mesh_name);
        ////trimesh_shape->Pos = m_offset;
        trimesh_shape->SetStatic(true);
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
        //// HACK: Chrono::OpenGL does not properly account for Pos & Rot.
        ////       So transform the end points explicitly.
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

    for (auto mesh : m_meshes) {
        std::string vis_mesh_file = "robosimian/obj/" + mesh.m_name + ".obj";
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(GetChronoDataFile(vis_mesh_file), true, false);
        //// HACK: a trimesh visual asset ignores transforms! Explicitly offset vertices.
        trimesh.Transform(mesh.m_pos, ChMatrix33<>(mesh.m_rot));
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(mesh.m_name);
        ////trimesh_shape->Pos = m_offset;
        trimesh_shape->SetStatic(true);
        m_body->AddAsset(trimesh_shape);
    }
}

void Part::AddCollisionShapes() {
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
    for (auto mesh : m_meshes) {
        std::string vis_mesh_file = "robosimian/obj/" + mesh.m_name + ".obj";
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(GetChronoDataFile(vis_mesh_file), false, false);
        if (mesh.m_convex) {
            m_body->GetCollisionModel()->AddConvexHull(trimesh.getCoordsVertices(), mesh.m_pos, mesh.m_rot);
        } else {
            m_body->GetCollisionModel()->AddTriangleMesh(trimesh, false, false, mesh.m_pos, mesh.m_rot, 0.01);
        }
    }

    m_body->GetCollisionModel()->BuildModel();
}

// =============================================================================

Chassis::Chassis(const std::string& name, ChSystem* system, bool fixed) : Part(name, system), m_collide(false) {
    double mass = 46.658335;
    ChVector<> com(0.040288, -0.001937, -0.073574);
    ChVector<> inertia_xx(1.272134, 2.568776, 3.086984);
    ChVector<> inertia_xy(0.008890, -0.13942, 0.000325);

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

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::CHASSIS);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::LIMB_FR);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::LIMB_RR);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::LIMB_RL);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::LIMB_FL);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::SLED);

    // Note: call this AFTER setting the collision family (required for Chrono::Parallel)
    m_body->SetCollide(m_collide);
}

void Chassis::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
}

// =============================================================================

Sled::Sled(const std::string& name, chrono::ChSystem* system) : Part(name, system), m_collide(true) {
    double mass = 2.768775;
    ChVector<> com(0.000000, 0.000000, 0.146762);
    ChVector<> inertia_xx(0.034856, 0.082427, 0.105853);
    ChVector<> inertia_xy(0.000007, -0.000002, 0);

    m_body->SetIdentifier(1);
    m_body->SetMass(mass);
    m_body->SetFrame_COG_to_REF(ChFrame<>(com, ChQuaternion<>(1, 0, 0, 0)));
    m_body->SetInertiaXX(inertia_xx);
    m_body->SetInertiaXY(inertia_xy);
    system->Add(m_body);

    m_meshes.push_back(MeshShape(ChVector<>(0, 0, 0), QUNIT, "robosim_sled_coll", true));

    m_mesh_name = "robosim_sled";
    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.7f, 0.7f, 0.7f);

    //// TODO: set contact material properties
}

void Sled::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector<>& xyz, const ChVector<>& rpy) {
    const ChFrame<>& X_GP = chassis->GetFrame_REF_to_abs();  // global -> parent
    ChFrame<> X_PC(xyz, rpy2quat(rpy));                      // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                            // global -> child
    m_body->SetFrame_REF_to_abs(X_GC);

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::SLED);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::LIMB_FR);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::LIMB_RR);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::LIMB_RL);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::LIMB_FL);

    // Note: call this AFTER setting the collision family (required for Chrono::Parallel)
    m_body->SetCollide(m_collide);

    // Add joint (weld)
    auto joint = std::make_shared<ChLinkLockLock>();
    joint->Initialize(chassis, m_body, calcJointFrame(X_GC, ChVector<>(1, 0, 0)));
    chassis->GetSystem()->AddLink(joint);
}

void Sled::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
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

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::WHEEL_DD);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::CHASSIS);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::SLED);

    // Note: call this AFTER setting the collision family (required for Chrono::Parallel)
    m_body->SetCollide(true);

    // Add joint
    auto joint = std::make_shared<ChLinkLockRevolute>();
    joint->Initialize(chassis, m_body, calcJointFrame(X_GC, ChVector<>(0, 0, 1)));
    chassis->GetSystem()->AddLink(joint);
}

// =============================================================================

Limb::Limb(const std::string& name, LimbID id, const LinkData data[], ChSystem* system)
    : m_name(name), m_id(id), m_collide_links(false), m_collide_wheel(true) {
    for (int i = 0; i < num_links; i++) {
        auto link = std::make_shared<Part>(m_name + "_" + data[i].name, system);

        double mass = data[i].link.m_mass;
        ChVector<> com = data[i].link.m_com;
        ChVector<> inertia_xx = data[i].link.m_inertia_xx;
        ChVector<> inertia_xy = data[i].link.m_inertia_xy;

        link->m_body->SetIdentifier(4 + 4 * id + i);
        link->m_body->SetMass(mass);
        link->m_body->SetFrame_COG_to_REF(ChFrame<>(com, ChQuaternion<>(1, 0, 0, 0)));
        link->m_body->SetInertiaXX(inertia_xx);
        link->m_body->SetInertiaXY(inertia_xy);

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
        if (data[i].name.compare("link8") == 0)
            m_wheel = link;
    }
}

void Limb::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                      const ChVector<>& xyz,
                      const ChVector<>& rpy,
                      CollisionFamily::Enum collision_family) {
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

        // Add contact geometry to child body
        child->AddCollisionShapes();

        // Place all links from this limb in the same collision family
        child_body->GetCollisionModel()->SetFamily(collision_family);
        child_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(collision_family);

        // Note: call this AFTER setting the collision family (required for Chrono::Parallel)
        if (child == m_wheel)
            child_body->SetCollide(m_collide_wheel);
        else
            child_body->SetCollide(m_collide_links);

        // Create joint
        switch (joints[i].mode) {
            case ActuationMode::FIXED: {
                auto joint = std::make_shared<ChLinkLockLock>();
                joint->SetNameString(m_name + "_" + joints[i].name);
                joint->Initialize(parent_body, child_body, calcJointFrame(X_GC, joints[i].axis));
                chassis->GetSystem()->AddLink(joint);
                m_joints.insert(std::make_pair(joints[i].name, joint));
                break;
            }
            case ActuationMode::ANGLE: {
                auto motor_fun = std::make_shared<ChFunction_Setpoint>();
                auto joint = std::make_shared<ChLinkMotorRotationAngle>();
                joint->SetNameString(m_name + "_" + joints[i].name);
                joint->Initialize(parent_body, child_body, ChFrame<>(calcJointFrame(X_GC, joints[i].axis)));
                joint->SetAngleFunction(motor_fun);
                chassis->GetSystem()->AddLink(joint);
                m_joints.insert(std::make_pair(joints[i].name, joint));
                m_motors.insert(std::make_pair(joints[i].name, joint));
                break;
            }
            case ActuationMode::SPEED: {
                auto motor_fun = std::make_shared<ChFunction_Setpoint>();
                auto joint = std::make_shared<ChLinkMotorRotationSpeed>();
                joint->SetNameString(m_name + "_" + joints[i].name);
                joint->Initialize(parent_body, child_body, ChFrame<>(calcJointFrame(X_GC, joints[i].axis)));
                joint->SetSpeedFunction(motor_fun);
                chassis->GetSystem()->AddLink(joint);
                m_joints.insert(std::make_pair(joints[i].name, joint));
                m_motors.insert(std::make_pair(joints[i].name, joint));
                break;
            }
        }
    }
}

void Limb::SetVisualizationType(VisualizationType vis) {
    for (auto link : m_links)
        link.second->SetVisualizationType(vis);

    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("greenwhite.png"));
    m_wheel->m_body->AddAsset(texture);
}

void Limb::Activate(const std::string& motor_name, double time, double val) {
    auto itr = m_motors.find(motor_name);
    if (itr == m_motors.end())
        return;

    // Note: currently hard-coded for angle motor
    auto fun = std::static_pointer_cast<ChFunction_Setpoint>(itr->second->GetMotorFunction());
    fun->SetSetpoint(-val, time);
}

static std::string motor_names[] = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"};

void Limb::Activate(double time, const std::array<double, 8>& vals) {
    //// TODO: Not a terribly satisfying solution...
    for (int i = 0; i < 8; i++) {
        auto fun = std::static_pointer_cast<ChFunction_Setpoint>(m_motors[motor_names[i]]->GetMotorFunction());
        fun->SetSetpoint(-vals[i], time);
    }
}

void Limb::SetCollideLinks(bool state) {
    m_collide_links = state;
    for (auto link : m_links) {
        if (link.second != m_wheel)
            link.second->m_body->SetCollide(state);
    }
}

void Limb::SetCollideWheel(bool state) {
    m_collide_wheel = state;
    m_wheel->m_body->SetCollide(state);
}

}  // end namespace robosimian
