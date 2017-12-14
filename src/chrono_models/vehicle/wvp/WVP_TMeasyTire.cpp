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
// Authors: Rainer Gericke
// =============================================================================
//
// WVP TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/wvp/WVP_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string WVP_TMeasyTire::m_meshName = "hmmwv_tire_POV_geom";
const std::string WVP_TMeasyTire::m_meshFile = "hmmwv/hmmwv_tire.obj";

const double WVP_TMeasyTire::m_mass = 71.1;
const ChVector<> WVP_TMeasyTire::m_inertia(9.62, 16.84, 9.62);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_TMeasyTire::WVP_TMeasyTire(const std::string& name) : ChTMeasyTire(name) {
	// Set Handling Charecteristics
    SetTMeasyParams();
    
    // Set nonlinear vertical stiffness
    std::vector<double> disp, force;
    disp.push_back(0.01); force.push_back(3276.0);
    disp.push_back(0.02); force.push_back(6729.0);
    disp.push_back(0.03); force.push_back(10361.0);
    disp.push_back(0.04); force.push_back(14171.0);
    disp.push_back(0.05); force.push_back(18159.0);
    disp.push_back(0.06); force.push_back(22325.0);
    disp.push_back(0.07); force.push_back(26670.0);
    disp.push_back(0.08); force.push_back(31192.0);
    disp.push_back(0.09); force.push_back(35893.0);
    disp.push_back(0.10); force.push_back(40772.0);
    
    VerticalStiffnessByTable(disp,force);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_TMeasyTire::SetTMeasyParams() {
    // Tire Size = 365/80R20 152K

    unsigned int li     = 152;  
    const double in2m   = 0.0254;
    double 		 w      = 0.365;
    double 		 r      = 0.8;
    double 		 rimdia = 20.0 * in2m;

    GuessTruck80Par(li,     // load index
                    w,      // tire width
                    r,      // aspect ratio
                    rimdia  // rim diameter
    );
}

void WVP_TMeasyTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/365_80R20_" + GetName() + ".gpl";
    WritePlots(filename, "365/80R20 152K");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_wheel->AddAsset(m_trimesh_shape);
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void WVP_TMeasyTire::RemoveVisualizationAssets() {
    ChTMeasyTire::RemoveVisualizationAssets();

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
