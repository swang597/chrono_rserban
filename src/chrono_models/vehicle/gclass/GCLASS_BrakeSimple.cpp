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
// Authors: Alessandro Tasora
// =============================================================================
//
// UAZBUS simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/gclass/GCLASS_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace gclass {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double GCLASS_BrakeSimpleFront::m_maxtorque = 1600;

const double GCLASS_BrakeSimpleRear::m_maxtorque = 1500;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
GCLASS_BrakeSimpleFront::GCLASS_BrakeSimpleFront(const std::string& name) : ChBrakeSimple(name) {}

GCLASS_BrakeSimpleRear::GCLASS_BrakeSimpleRear(const std::string& name) : ChBrakeSimple(name) {}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
