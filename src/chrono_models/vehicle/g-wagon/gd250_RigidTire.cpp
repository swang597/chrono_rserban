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
// GD250 rigid tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/g-wagon/gd250_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace gwagon {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double GD250_RigidTire::m_radius = 0.372;
const double GD250_RigidTire::m_width = 0.228;

const double GD250_RigidTire::m_mass = 19.8;
const ChVector<> GD250_RigidTire::m_inertia(1.2369, 2.22357, 1.2369);

const std::string GD250_RigidTire::m_meshFile = "g-wagon/gd250_tire_fine.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
GD250_RigidTire::GD250_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        SetMeshFilename(GetDataFile("uaz/uaz_tire_fine.obj"), 0.005);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void GD250_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void GD250_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void GD250_RigidTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}

}  // end namespace gwagon
}  // end namespace vehicle
}  // end namespace chrono
