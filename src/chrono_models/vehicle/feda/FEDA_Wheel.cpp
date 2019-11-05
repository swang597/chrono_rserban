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
// Authors: Radu Serban, Justin Madsen, Rainer Gericke
// =============================================================================
//
// FEDA wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/feda/FEDA_Wheel.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double FEDA_Wheel::m_mass = 18.8;
const ChVector<> FEDA_Wheel::m_inertia(0.113, 0.113, 0.113);

const double FEDA_Wheel::m_radius = 0.28575;
const double FEDA_Wheel::m_width = 0.254;

const std::string FEDA_WheelLeft::m_meshName = "wheel_L_POV_geom";
const std::string FEDA_WheelLeft::m_meshFile = "FEDA/wheel_L.obj";

const std::string FEDA_WheelRight::m_meshName = "wheel_R_POV_geom";
const std::string FEDA_WheelRight::m_meshFile = "FEDA/wheel_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FEDA_Wheel::FEDA_Wheel(const std::string& name) : ChWheel(name) {}

FEDA_WheelLeft::FEDA_WheelLeft(const std::string& name) : FEDA_Wheel(name) {}

FEDA_WheelRight::FEDA_WheelRight(const std::string& name) : FEDA_Wheel(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void FEDA_Wheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetMeshFile(), false, false);
        trimesh->Transform(ChVector<>(0, m_offset, 0), ChMatrix33<>(1));
        m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(GetMeshName());
        m_trimesh_shape->SetStatic(true);
        GetSpindle()->AddAsset(m_trimesh_shape);
    } else {
        ChWheel::AddVisualizationAssets(vis);
    }
}

void FEDA_Wheel::RemoveVisualizationAssets() {
    ChWheel::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by FEDA_Wheel::AddVisualizationAssets.
    // This is important for the ChWheel object because a tire may add its own assets
    // to the same body (the spindle).
    auto it = std::find(GetSpindle()->GetAssets().begin(), GetSpindle()->GetAssets().end(), m_trimesh_shape);
    if (it != GetSpindle()->GetAssets().end())
        GetSpindle()->GetAssets().erase(it);
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
