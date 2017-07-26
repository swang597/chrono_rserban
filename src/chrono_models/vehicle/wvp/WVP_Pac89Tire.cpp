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
// Authors: Radu Serban, Michael Taylor, Asher Elmquist
// =============================================================================
//
// WVP PAC89 tire subsystem
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/wvp/WVP_Pac89Tire.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double WVP_Pac89Tire::m_normalDamping = 8154;

const std::string WVP_Pac89Tire::m_meshName = "hmmwv_tire_POV_geom";
const std::string WVP_Pac89Tire::m_meshFile = "hmmwv/hmmwv_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_Pac89Tire::WVP_Pac89Tire(const std::string& name) : ChPac89Tire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_Pac89Tire::SetPac89Params() {

    m_unloaded_radius = 1.0960/2; //Diameter to Radius
    m_width = 0.372;
    m_rolling_resistance = 0.0;
    m_lateral_stiffness = 185*1000.; // N/mm -> N/m
    m_measured_side = LEFT;

    m_PacCoeff.A0 = 1.0012;
    m_PacCoeff.A1 = -0.6774;
    m_PacCoeff.A2 = 1017.452;
    m_PacCoeff.A3 = 5707.292;
    m_PacCoeff.A4 = 62.6561;
    m_PacCoeff.A5 = 0.0;
    m_PacCoeff.A6 = 0.008;
    m_PacCoeff.A7 = 0.9419;
    m_PacCoeff.A8 = 0.1124;
    m_PacCoeff.A9 = -0.0026;
    m_PacCoeff.A10 = -0.1197;
    m_PacCoeff.A11 = -17.2366;
    m_PacCoeff.A12 = 9.9733;
    m_PacCoeff.A13 = -10.7445;

    m_PacCoeff.B0 = 1.0000;
    m_PacCoeff.B1 = -5.0940;
    m_PacCoeff.B2 = 883.1000;
    m_PacCoeff.B3 = 0.2720;
    m_PacCoeff.B4 = 117.3000;
    m_PacCoeff.B5 = 0.000;
    m_PacCoeff.B6 = 0.0002;
    m_PacCoeff.B7 = -0.0593;
    m_PacCoeff.B8 = 0.2285;
    m_PacCoeff.B9 = 0.0026;
    m_PacCoeff.B10 = 0.9228;

    m_PacCoeff.C0 = 1.8392;
    m_PacCoeff.C1 = -0.2289;
    m_PacCoeff.C2 = -11.239;
    m_PacCoeff.C3 = -0.0298;
    m_PacCoeff.C4 = -6.6725;
    m_PacCoeff.C5 = 0.0;
    m_PacCoeff.C6 = 0.0;
    m_PacCoeff.C7 = .0035;
    m_PacCoeff.C8 = -0.282;
    m_PacCoeff.C9 = 2.0;
    m_PacCoeff.C10 = 0.3595;
    m_PacCoeff.C11 = 0.0157;
    m_PacCoeff.C12 = -0.0011;
    m_PacCoeff.C13 = -0.2298;
    m_PacCoeff.C14 = 0.0194;
    m_PacCoeff.C15 = -0.764;
    m_PacCoeff.C16 = -2.275;
    m_PacCoeff.C17 = 40.78;

    //parameters from the HMMWV
    // m_unloaded_radius = 326.0/1000;
    // m_width = 245.0/1000;
    // m_rolling_resistance = 0.0;
    // m_lateral_stiffness = 190*1000.; // N/mm -> N/m
    // m_measured_side = LEFT;

    // m_PacCoeff.A0 = 1.650;
    // m_PacCoeff.A1 = -34.0;
    // m_PacCoeff.A2 = 1250.0;
    // m_PacCoeff.A3 = 3036.0;
    // m_PacCoeff.A4 = 12.80;
    // m_PacCoeff.A5 = 0.00501;
    // m_PacCoeff.A6 = -0.02103;
    // m_PacCoeff.A7 = 0.77394;
    // m_PacCoeff.A8 = 0.0022890;
    // m_PacCoeff.A9 = 0.013442;
    // m_PacCoeff.A10 = 0.003709;
    // m_PacCoeff.A11 = 19.1656;
    // m_PacCoeff.A12 = 1.21356;
    // m_PacCoeff.A13 = 6.26206;

    // m_PacCoeff.B0 = 1.67272;
    // m_PacCoeff.B1 = -9.46;
    // m_PacCoeff.B2 = 1490.0;
    // m_PacCoeff.B3 = 30.0;
    // m_PacCoeff.B4 = 176.0;
    // m_PacCoeff.B5 = 0.08860;
    // m_PacCoeff.B6 = 0.00402;
    // m_PacCoeff.B7 = -0.06150;
    // m_PacCoeff.B8 = 0.20;
    // m_PacCoeff.B9 = 0.02990;
    // m_PacCoeff.B10 = -0.176;

    // m_PacCoeff.C0 = 2.34;
    // m_PacCoeff.C1 = 1.4950;
    // m_PacCoeff.C2 = 6.416654;
    // m_PacCoeff.C3 = -3.57403;
    // m_PacCoeff.C4 = -0.087737;
    // m_PacCoeff.C5 = 0.098410;
    // m_PacCoeff.C6 = 0.0027699;
    // m_PacCoeff.C7 = -0.0001151;
    // m_PacCoeff.C8 = 0.10;
    // m_PacCoeff.C9 = -1.3329;
    // m_PacCoeff.C10 = 0.025501;
    // m_PacCoeff.C11 = -0.02357;
    // m_PacCoeff.C12 = 0.03027;
    // m_PacCoeff.C13 = -0.0647;
    // m_PacCoeff.C14 = 0.0211329;
    // m_PacCoeff.C15 = 0.89469;
    // m_PacCoeff.C16 = -0.099443;
    // m_PacCoeff.C17 = -3.336941;

}

double WVP_Pac89Tire::GetNormalStiffnessForce(double depth) const {
    // corresponding depths = 0 : 0.01 : 0.10
    double normalforcetabel[11] = {0.0,
                                   3276.0,
                                   6729.0,
                                   10361.0,
                                   14171.0,
                                   18159.0,
                                   22325.0,
                                   26670.0,
                                   31192.0,
                                   35893.0,
                                   40772.0};

    depth = depth * (depth > 0);  // Ensure that depth is positive;

    double position = (10. * (depth / 0.1));

    // Linear extrapolation if the depth is at or greater than the maximum depth in the table (0.08128)
    if (position >= 10) {
        return (40772.0 + (depth - 0.1) * 407720.0);
    }
    // Linearly interpolate between the table entries
    else {
        double scale = std::ceil(position) - position;
        return (normalforcetabel[int(std::floor(position))] * (1 - scale) +
                normalforcetabel[int(std::floor(position) + 1)] * scale);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_Pac89Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_wheel->AddAsset(m_trimesh_shape);
    }
    else {
        ChPac89Tire::AddVisualizationAssets(vis);
    }
}

void WVP_Pac89Tire::RemoveVisualizationAssets() {
    ChPac89Tire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by WVP_FialaTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
    if (it != m_wheel->GetAssets().end())
        m_wheel->GetAssets().erase(it);
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
