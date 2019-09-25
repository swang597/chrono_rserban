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

const std::string WVP_WheelLeft::m_meshName = "wheel_L_POV_geom";
const std::string WVP_WheelLeft::m_meshFile = "hmmwv/wheel_L.obj";

const std::string WVP_WheelRight::m_meshName = "wheel_R_POV_geom";
const std::string WVP_WheelRight::m_meshFile = "hmmwv/wheel_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_Wheel::WVP_Wheel(const std::string& name) : ChWheel(name) {}

WVP_WheelLeft::WVP_WheelLeft(const std::string& name) : WVP_Wheel(name) {}

WVP_WheelRight::WVP_WheelRight(const std::string& name) : WVP_Wheel(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_Wheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetMeshFile(), false, false);
        trimesh->Transform(ChVector<>(0, m_offset, 0), ChMatrix33<>(1));
        m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(GetMeshName());
        GetSpindle()->AddAsset(m_trimesh_shape);
    } else {
        ChWheel::AddVisualizationAssets(vis);
    }
}

void WVP_Wheel::RemoveVisualizationAssets() {
    ChWheel::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by WVP_Wheel::AddVisualizationAssets.
    // This is important for the ChWheel object because a tire may add its own assets
    // to the same body (the spindle).
    auto it = std::find(GetSpindle()->GetAssets().begin(), GetSpindle()->GetAssets().end(), m_trimesh_shape);
    if (it != GetSpindle()->GetAssets().end())
        GetSpindle()->GetAssets().erase(it);
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
