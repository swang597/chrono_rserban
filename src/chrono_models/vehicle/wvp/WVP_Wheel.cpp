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
// WVP wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/wvp/WVP_Wheel.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double WVP_Wheel::m_mass = 134.9;//206;// 134.9;
const ChVector<> WVP_Wheel::m_inertia(4.38, 9.16, 4.38);

const double WVP_Wheel::m_radius = 1.096/2.0;
const double WVP_Wheel::m_width = 0.372;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_Wheel::WVP_Wheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "wvp/wvp_rim.obj";
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
