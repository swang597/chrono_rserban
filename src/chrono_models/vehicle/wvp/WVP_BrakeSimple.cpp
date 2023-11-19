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
// Authors: Asher Elmquist
// =============================================================================
//
// WVP simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/wvp/WVP_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double WVP_BrakeSimple::m_maxtorque = 4000;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_BrakeSimple::WVP_BrakeSimple(const std::string& name) : ChBrakeSimple(name) {}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
