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
// MAN 10t simple driveline model.
//
// =============================================================================

#include "chrono_models/vehicle/man/MAN_10t_SimpleDriveline.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double MAN_10t_SimpleDriveline::m_front1_diff_bias = 2.0;
const double MAN_10t_SimpleDriveline::m_front2_diff_bias = 2.0;
const double MAN_10t_SimpleDriveline::m_rear1_diff_bias = 2.0;
const double MAN_10t_SimpleDriveline::m_rear2_diff_bias = 2.0;

// -----------------------------------------------------------------------------
// Constructor of MAN_10t_SimpleDriveline.
// -----------------------------------------------------------------------------
MAN_10t_SimpleDriveline::MAN_10t_SimpleDriveline(const std::string& name) : ChSimpleDriveline8WD(name) {}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
