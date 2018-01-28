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
// WVP Pitman arm steering model with compliant steering column.
//
// =============================================================================

#include "chrono_models/vehicle/wvp/WVP_PitmanArmShafts.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double WVP_PitmanArmShafts::m_steeringLinkMass = 3.0;
const double WVP_PitmanArmShafts::m_pitmanArmMass = 2.0;

const double WVP_PitmanArmShafts::m_steeringLinkRadius = 0.03;
const double WVP_PitmanArmShafts::m_pitmanArmRadius = 0.02;

//// NOTE: maxAngle (at steering wheel) and gear ratio set such that max angle at revolute = 41.25
const double WVP_PitmanArmShafts::m_maxAngle = 780.0 * (CH_C_PI / 180);
const double WVP_PitmanArmShafts::m_gearRatio = 780.0 / 41.25;
//// NOTE: steering compliance 2000 N.mm/deg (too low?)
const double WVP_PitmanArmShafts::m_steeringCompliance = 2 * CH_C_RAD_TO_DEG;
//// NOTE: shaft inertia approximated (likely too high)
const double WVP_PitmanArmShafts::m_columnInertia = 5e-2;

const ChVector<> WVP_PitmanArmShafts::m_steeringLinkInertiaMoments(6.34e-2, 4.47e-4, 6.37e-2);
const ChVector<> WVP_PitmanArmShafts::m_steeringLinkInertiaProducts(0.0, 0.0, 0.0);

const ChVector<> WVP_PitmanArmShafts::m_pitmanArmInertiaMoments(1.0e-3, 1.0e-3, 1.0e-3);
const ChVector<> WVP_PitmanArmShafts::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_PitmanArmShafts::WVP_PitmanArmShafts(const std::string& name, bool rigid_connection)
    : ChPitmanArmShafts(name, true, rigid_connection) {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------
const ChVector<> WVP_PitmanArmShafts::getLocation(PointId which) {
    switch (which) {
        case STEERINGLINK:
            return ChVector<>(-483e-3, 7e-3, 24e-3);
        case PITMANARM:
            return ChVector<>(-629e-3, 474e-3, 40e-3);
        case REV:
            return ChVector<>(-773e-3, 475e-3, 57e-3);
        case UNIV:
            return ChVector<>(-483e-3, 475e-3, 24e-3);
        case REVSPH_R:
            return ChVector<>(-773e-3, -375e-3, 57e-3);
        case REVSPH_S:
            return ChVector<>(-483e-3, -375e-3, 24e-3);
        case TIEROD_PA:
            return ChVector<>(-383e-3, 125e-3, 29e-3);
        case TIEROD_IA:
            return ChVector<>(-383e-3, -125e-3, 29e-3);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> WVP_PitmanArmShafts::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector<>(0, 0, 1);
        case UNIV_AXIS_ARM:
            return ChVector<>(0, 0, 1);
        case UNIV_AXIS_LINK:
            return ChVector<>(1, 0, 0);
        case REVSPH_AXIS:
            return ChVector<>(0, 0, 1);
        default:
            return ChVector<>(0, 0, 1);
    }
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
