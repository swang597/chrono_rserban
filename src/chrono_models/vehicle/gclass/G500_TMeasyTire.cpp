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
// G500 TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/gclass/G500_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace gclass {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double G500_TMeasyTireFront::m_mass = 37.6;
const ChVector<> G500_TMeasyTireFront::m_inertia(3.84, 6.69, 3.84);
const std::string G500_TMeasyTireFront::m_meshFile = "gclass/gd250_tire.obj";

const double G500_TMeasyTireRear::m_mass = 37.6;
const ChVector<> G500_TMeasyTireRear::m_inertia(3.84, 6.69, 3.84);
const std::string G500_TMeasyTireRear::m_meshFile = "gclass/gd250_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
G500_TMeasyTireFront::G500_TMeasyTireFront(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

void G500_TMeasyTireFront::SetTMeasyParams() {
    // Tire Size = 37 x 12.5 x 16.5 Load Range D
    // Tire Load 3850 lbs at 50 psi (Goodyear Military Tire Brochure 6th Edition)
    
    unsigned int li = 121;  // UAZ data
    const double in2m = 0.0254;
    double w = 0.225;
    double r = 0.75;
    double rimdia = 16.0 * in2m;
    double pres_li = 590000;
    double pres_use = 220000;
    
    GuessTruck80Par(li,       // tire load index []
                    w,        // tire width [m]
                    r,        // aspect ratio []
                    rimdia,   // rim diameter [m],
                    pres_li,  // infl. pressure for load index
                    pres_use  // infl. pressure for usage
    );
}

void G500_TMeasyTireFront::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/225_75R16_" + GetName() + ".gpl";
    WritePlots(filename, "225/75R20");
}

void G500_TMeasyTireFront::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void G500_TMeasyTireFront::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
G500_TMeasyTireRear::G500_TMeasyTireRear(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

void G500_TMeasyTireRear::SetTMeasyParams() {
    // Tire Size = 37 x 12.5 x 16.5 Load Range D
    // Tire Load 3850 lbs at 50 psi (Goodyear Military Tire Brochure 6th Edition)
    
    unsigned int li = 121;  // UAZ data
    const double in2m = 0.0254;
    double w = 0.225;
    double r = 0.75;
    double rimdia = 16.0 * in2m;
    double pres_li = 590000;
    double pres_use = 240000;

    GuessTruck80Par(li,       // tire load index []
                    w,        // tire width [m]
                    r,        // aspect ratio []
                    rimdia,   // rim diameter [m],
                    pres_li,  // infl. pressure for load index
                    pres_use  // infl. pressure for usage
    );
}

void G500_TMeasyTireRear::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/225_75R16_" + GetName() + ".gpl";
    WritePlots(filename, "225/75R20");
}

void G500_TMeasyTireRear::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void G500_TMeasyTireRear::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
