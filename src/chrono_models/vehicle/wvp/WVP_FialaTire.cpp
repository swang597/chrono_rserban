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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// WVP Fiala subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/wvp/WVP_FialaTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double WVP_FialaTire::m_normalDamping = 4077;

const double WVP_FialaTire::m_mass = 71.1;
const ChVector<> WVP_FialaTire::m_inertia(9.62, 16.84, 9.62);

const std::string WVP_FialaTire::m_meshFile_left = "wvp/wvp_tire_left.obj";
const std::string WVP_FialaTire::m_meshFile_right = "wvp/wvp_tire_right.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_FialaTire::WVP_FialaTire(const std::string& name) : ChFialaTire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_FialaTire::SetFialaParams() {
    // Parametes were fit at roughly 1700lbs and 24psi
    // Note, the width is not based on the width of the physical tire.
    // It was soley based on providing a good fit to the aligning torque curve.
    /*m_unloaded_radius = 0.548;
    m_width = 0.372;
    m_rolling_resistance = 0.001;  // Assume very small since no other data exists
    m_c_slip = 246517.6;
    m_c_alpha = 203760.0;
    m_u_min = 0.75;
    m_u_max = 0.8;
    m_relax_length_x = .12388;
    m_relax_length_y = .012388;*/

    m_unloaded_radius = 0.548;
    m_width = 0.2;
    m_rolling_resistance = 0.001;
    m_c_slip = 246517.6;
    m_c_alpha = 203760;
    m_u_min = .75;
    m_u_max = 0.8;
    m_relax_length_x = 1.2388;
    m_relax_length_y = 1.2388;
}

double WVP_FialaTire::GetNormalStiffnessForce(double depth) const {
    // corresponding depths = 0 : 0.00254 : 0.08128
    /*double normalforcetabel[33] = {};*/

    depth = depth * (depth > 0);  // Ensure that depth is positive;
    /*std::cout<<"FIALA"<<std::endl;*/
    /*double position = (33. * (depth / 0.08128));

    // Linear extrapolation if the depth is at or greater than the maximum depth in the table (0.08128)
    if (position >= 32) {
        return (22129.9025595000 + (depth - 0.08128) * 3.283628164370066e+05);
    }
    // Linearly interpolate between the table entries
    else {
        double scale = std::ceil(position) - position;
        return (normalforcetabel[int(std::floor(position))] * (1 - scale) +
                normalforcetabel[int(std::floor(position) + 1)] * scale);
    }*/
    return 407715.0 * depth;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_FialaTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChFialaTire::AddVisualizationAssets(vis);
    }
}

void WVP_FialaTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChFialaTire::RemoveVisualizationAssets();
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
