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
// WVP shafts-based brake model.
//
// =============================================================================

#include "chrono_models/vehicle/wvp/WVP_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double WVP_BrakeShafts::m_maxtorque = 4000;
const double WVP_BrakeShafts::m_shaft_inertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_BrakeShafts::WVP_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
