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
// GD250 rotary arm steering model.
//
// =============================================================================

#include "chrono_models/vehicle/g-wagon/gd250_RotaryArm.h"

namespace chrono {
namespace vehicle {
namespace gwagon {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double GD250_RotaryArm::m_pitmanArmMass = 1.605;

const double GD250_RotaryArm::m_pitmanArmRadius = 0.02;

const double GD250_RotaryArm::m_maxAngle = 16.5 * (CH_C_PI / 180);

const ChVector<> GD250_RotaryArm::m_pitmanArmInertiaMoments(0.00638, 0.00756, 0.00150);
const ChVector<> GD250_RotaryArm::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
GD250_RotaryArm::GD250_RotaryArm(const std::string& name) : ChRotaryArm(name) {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------
const ChVector<> GD250_RotaryArm::getLocation(PointId which) {
    switch (which) {
        case ARM_L:
            return ChVector<>(0.6, 0.7325 - 0.2, 0.2);
        case ARM_C:
            return ChVector<>(0.6, 0.7325 - 0.2, 0.4);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> GD250_RotaryArm::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector<>(0, 1, 0);
        default:
            return ChVector<>(0, 1, 0);
    }
}

}  // end namespace gwagon
}  // end namespace vehicle
}  // end namespace chrono
