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
// Authors: Radu Serban, Asher ELmquist
// =============================================================================
//
// Simple driveline model. This is a hardcoded driveline that represents
// the WVP 4WD driveline. It is for temporary use as the ChaftsDriveline
// seems to generate excess noise and the simpledriveline template does not
// connect properly to the simeplePowertrain model.
//
// =============================================================================

#include <cmath>

#include "chrono_models/vehicle/wvp/WVP_SimpleDriveline.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Construct a default 4WD simple driveline.
// -----------------------------------------------------------------------------
WVP_SimpleDriveline::WVP_SimpleDriveline(const std::string& name) : ChDrivelineWV(name) {}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline subsystem to the axles of the specified
// suspension subsystems.
// -----------------------------------------------------------------------------
void WVP_SimpleDriveline::Initialize(std::shared_ptr<ChChassis> chassis,
                                     const ChAxleList& axles,
                                     const std::vector<int>& driven_axles) {
    assert(axles.size() >= 2);

    m_driven_axles = driven_axles;

    // Grab handles to the suspension wheel shafts.
    m_front_left = axles[m_driven_axles[0]]->m_suspension->GetAxle(LEFT);
    m_front_right = axles[m_driven_axles[0]]->m_suspension->GetAxle(RIGHT);

    m_rear_left = axles[m_driven_axles[1]]->m_suspension->GetAxle(LEFT);
    m_rear_right = axles[m_driven_axles[1]]->m_suspension->GetAxle(RIGHT);
}

// -----------------------------------------------------------------------------
// This utility function implements a simple model of Torsen limited-slip
// differential with a max_bias:1 torque bias ratio.
// We hardcode the speed difference range over which the torque bias grows from
// a value of 1 to a value of max_bias to the interval [0.25, 0.5].
// -----------------------------------------------------------------------------
void differentialSplit(double torque,
                       double max_bias,
                       double speed_left,
                       double speed_right,
                       double& torque_left,
                       double& torque_right) {
    double diff = std::abs(speed_left - speed_right);

    // The bias grows from 1 at diff=0.25 to max_bias at diff=0.5
    double bias = 1;
    if (diff > 0.5)
        bias = max_bias;
    else if (diff > 0.25)
        bias = 4 * (max_bias - 1) * diff + (2 - max_bias);

    // Split torque to the slow and fast wheels.
    double alpha = bias / (1 + bias);
    double slow = alpha * torque;
    double fast = torque - slow;

    if (std::abs(speed_left) < std::abs(speed_right)) {
        torque_left = slow;
        torque_right = fast;
    } else {
        torque_left = fast;
        torque_right = slow;
    }
}

// -----------------------------------------------------------------------------
void WVP_SimpleDriveline::Synchronize(double time, const DriverInputs& driver_inputs, double torque) {
    // Enforce driveshaft speed
    m_driveshaft_speed = 0;
    if (!m_diffLockCenter) {
        double speed_front =
            0.5 * m_gearHubReduction * m_diffGearReduction * (m_front_left->GetPos_dt() + m_front_right->GetPos_dt());
        double speed_rear =
            0.5 * m_gearHubReduction * m_diffGearReduction * (m_rear_left->GetPos_dt() + m_rear_right->GetPos_dt());
        double alpha = m_frontTorqueFraction;

        m_driveshaft_speed = -(alpha * speed_front + (1 - alpha) * speed_rear);
    }

    // Split the input torque front/back.
    double torque_front = torque * m_frontTorqueFraction;
    double torque_rear = torque - torque_front;

    // Split the axle torques for the corresponding left/right wheels and apply
    // them to the suspension wheel shafts.
    double torque_left;
    double torque_right;

    torque_front = torque_front * (m_gearHubReduction * m_diffGearReduction);
    torque_rear = torque_rear * (m_gearHubReduction * m_diffGearReduction);

    differentialSplit(torque_front, m_frontDifferentialMaxBias, m_front_left->GetPos_dt(), m_front_right->GetPos_dt(),
                      torque_left, torque_right);
    m_front_left->SetAppliedTorque(-torque_left);
    m_front_right->SetAppliedTorque(-torque_right);

    differentialSplit(torque_rear, m_rearDifferentialMaxBias, m_rear_left->GetPos_dt(), m_rear_right->GetPos_dt(),
                      torque_left, torque_right);
    m_rear_left->SetAppliedTorque(-torque_left);
    m_rear_right->SetAppliedTorque(-torque_right);
}

// -----------------------------------------------------------------------------
double WVP_SimpleDriveline::GetSpindleTorque(int axle, VehicleSide side) const {
    if (axle == m_driven_axles[0]) {
        switch (side) {
            case LEFT:
                return -m_front_left->GetAppliedTorque();
            case RIGHT:
                return -m_front_right->GetAppliedTorque();
        }
    } else if (axle == m_driven_axles[1]) {
        switch (side) {
            case LEFT:
                return -m_rear_left->GetAppliedTorque();
            case RIGHT:
                return -m_rear_right->GetAppliedTorque();
        }
    }

    return 0;
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
