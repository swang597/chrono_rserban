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
#include "chrono_models/vehicle/wvp/WVP_Pac02Tire.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double WVP_Pac02Tire::m_mass = 71.1;
const ChVector<> WVP_Pac02Tire::m_inertia(9.62, 16.84, 9.62);

const std::string WVP_Pac02Tire::m_pacTireFile = "hmmwv/tire/HMMWV_pacejka.tir";

const std::string WVP_Pac02Tire::m_meshName = "hmmwv_tire_POV_geom";
const std::string WVP_Pac02Tire::m_meshFile = "hmmwv/hmmwv_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_Pac02Tire::WVP_Pac02Tire(const std::string& name) : ChPacejkaTire(name, vehicle::GetDataFile(m_pacTireFile)) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_Pac02Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_wheel->AddAsset(m_trimesh_shape);
    } else {
        ChPacejkaTire::AddVisualizationAssets(vis);
    }
}

void WVP_Pac02Tire::RemoveVisualizationAssets() {
    ChPacejkaTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by HMMWV_Pac02Tire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
    if (it != m_wheel->GetAssets().end())
        m_wheel->GetAssets().erase(it);
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
