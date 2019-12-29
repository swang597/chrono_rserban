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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simple driveline model. This template can be used to model a 6WD driveline.
// It uses a constant front1/front2/rear1/rear2 torque split (a fixed value of 1/4)
// and a simple model for Torsen limited-slip differentials.
//
// =============================================================================

#include <cmath>

#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline8WD.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a default 6WD simple driveline.
// -----------------------------------------------------------------------------
ChSimpleDriveline8WD::ChSimpleDriveline8WD(const std::string& name) : ChDrivelineWV(name) {}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline to the specified axles.
// -----------------------------------------------------------------------------
void ChSimpleDriveline8WD::Initialize(std::shared_ptr<ChBody> chassis,
                                      const ChAxleList& axles,
                                      const std::vector<int>& driven_axles) {
    assert(axles.size() >= 4);

    m_driven_axles = driven_axles;

    // Grab handles to the suspension wheel shafts.
    m_front1_left = axles[m_driven_axles[0]]->m_suspension->GetAxle(LEFT);
    m_front1_right = axles[m_driven_axles[0]]->m_suspension->GetAxle(RIGHT);

    m_front2_left = axles[m_driven_axles[1]]->m_suspension->GetAxle(LEFT);
    m_front2_right = axles[m_driven_axles[1]]->m_suspension->GetAxle(RIGHT);

    m_rear1_left = axles[m_driven_axles[2]]->m_suspension->GetAxle(LEFT);
    m_rear1_right = axles[m_driven_axles[2]]->m_suspension->GetAxle(RIGHT);

    m_rear2_left = axles[m_driven_axles[3]]->m_suspension->GetAxle(LEFT);
    m_rear2_right = axles[m_driven_axles[3]]->m_suspension->GetAxle(RIGHT);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChSimpleDriveline8WD::GetDriveshaftSpeed() const {
    double speed_front1 = 0.5 * (m_front1_left->GetPos_dt() + m_front1_right->GetPos_dt());
    double speed_front2 = 0.5 * (m_front2_left->GetPos_dt() + m_front2_right->GetPos_dt());
    double speed_rear1 = 0.5 * (m_rear1_left->GetPos_dt() + m_rear1_right->GetPos_dt());
    double speed_rear2 = 0.5 * (m_rear2_left->GetPos_dt() + m_rear2_right->GetPos_dt());
    double alpha = 1.0 / 4.0;

    return alpha * speed_front1 + alpha * speed_front2 + alpha * speed_rear1 + alpha * speed_rear2;
}

// -----------------------------------------------------------------------------
// This utility function implements a simple model of Torsen limited-slip
// differential with a max_bias:1 torque bias ratio.
// We hardcode the speed difference range over which the torque bias grows from
// a value of 1 to a value of max_bias to the interval [0.25, 0.5].
// -----------------------------------------------------------------------------
void differentialSplit8WD(double torque,
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
// -----------------------------------------------------------------------------
void ChSimpleDriveline8WD::Synchronize(double torque) {
    // Split the input torque front/back.
    const double alpha = 1.0 / 4.0;
    double torque_front1 = alpha * torque;
    double torque_front2 = alpha * torque;
    double torque_rear1 = alpha * torque;
    double torque_rear2 = alpha * torque;

    // Split the axle torques for the corresponding left/right wheels and apply
    // them to the suspension wheel shafts.
    double torque_left;
    double torque_right;

    differentialSplit8WD(torque_front1, GetFront1DifferentialMaxBias(), m_front1_left->GetPos_dt(),
                         m_front1_right->GetPos_dt(), torque_left, torque_right);
    m_front1_left->SetAppliedTorque(-torque_left);
    m_front1_right->SetAppliedTorque(-torque_right);

    differentialSplit8WD(torque_front2, GetFront2DifferentialMaxBias(), m_front2_left->GetPos_dt(),
                         m_front2_right->GetPos_dt(), torque_left, torque_right);
    m_front2_left->SetAppliedTorque(-torque_left);
    m_front2_right->SetAppliedTorque(-torque_right);

    differentialSplit8WD(torque_rear1, GetRear1DifferentialMaxBias(), m_rear1_left->GetPos_dt(),
                         m_rear1_right->GetPos_dt(), torque_left, torque_right);
    m_rear1_left->SetAppliedTorque(-torque_left);
    m_rear1_right->SetAppliedTorque(-torque_right);

    differentialSplit8WD(torque_rear2, GetRear2DifferentialMaxBias(), m_rear2_left->GetPos_dt(),
                         m_rear2_right->GetPos_dt(), torque_left, torque_right);
    m_rear2_left->SetAppliedTorque(-torque_left);
    m_rear2_right->SetAppliedTorque(-torque_right);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChSimpleDriveline8WD::GetSpindleTorque(int axle, VehicleSide side) const {
    if (axle == m_driven_axles[0]) {
        switch (side) {
            case LEFT:
                return -m_front1_left->GetAppliedTorque();
            case RIGHT:
                return -m_front1_right->GetAppliedTorque();
        }
    } else if (axle == m_driven_axles[1]) {
        switch (side) {
            case LEFT:
                return -m_front2_left->GetAppliedTorque();
            case RIGHT:
                return -m_front2_right->GetAppliedTorque();
        }
    } else if (axle == m_driven_axles[2]) {
        switch (side) {
            case LEFT:
                return -m_rear1_left->GetAppliedTorque();
            case RIGHT:
                return -m_rear1_right->GetAppliedTorque();
        }
    } else if (axle == m_driven_axles[3]) {
        switch (side) {
            case LEFT:
                return -m_rear2_left->GetAppliedTorque();
            case RIGHT:
                return -m_rear2_right->GetAppliedTorque();
        }
    }

    return 0;
}

}  // end namespace vehicle
}  // end namespace chrono
