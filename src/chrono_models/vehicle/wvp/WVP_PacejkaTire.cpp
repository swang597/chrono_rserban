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
// WVP Pacejka 2002 tire subsystem
//
// TODO: Needs Pacejka *.tir parameter file specific to this tire (365/80R20)
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/wvp/WVP_PacejkaTire.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double WVP_PacejkaTire::m_mass = 71.1;
const ChVector<> WVP_PacejkaTire::m_inertia(9.62, 16.84, 9.62);

const std::string WVP_PacejkaTire::m_pacTireFile = "wvp/tire/WVP_pacejka.tir";

const std::string WVP_PacejkaTire::m_meshFile_left = "wvp/wvp_tire_left.obj";
const std::string WVP_PacejkaTire::m_meshFile_right = "wvp/wvp_tire_right.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_PacejkaTire::WVP_PacejkaTire(const std::string& name) : ChPacejkaTire(name, vehicle::GetDataFile(m_pacTireFile)) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_PacejkaTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChPacejkaTire::AddVisualizationAssets(vis);
    }
}

void WVP_PacejkaTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChPacejkaTire::RemoveVisualizationAssets();
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
