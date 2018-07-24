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

#include "vehicleKD.h"
#include "framework.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace av {

VehicleKD::VehicleKD(Framework* framework, const chrono::ChCoordsys<>& pos) : Vehicle(framework) {}

VehicleKD::~VehicleKD() {
    //
}

void VehicleKD::Synchronize(double time) {
    //
}

void VehicleKD::Advance(double step) {
    //
}

}  // end namespace av
