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
// GD250 simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/g-wagon/GD250_BrakeSimple.h"

namespace chrono {
    namespace vehicle {
        namespace gwagon {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

            const double GD250_BrakeSimpleFront::m_maxtorque = 1600;

            const double GD250_BrakeSimpleRear::m_maxtorque = 1500;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
            GD250_BrakeSimpleFront::GD250_BrakeSimpleFront(const std::string& name) : ChBrakeSimple(name) {}

            GD250_BrakeSimpleRear::GD250_BrakeSimpleRear(const std::string& name) : ChBrakeSimple(name) {}

        }  // end namespace gwagon
    }  // end namespace vehicle
}  // end namespace chrono
