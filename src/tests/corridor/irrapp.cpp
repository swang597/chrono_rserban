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
// =============================================================================

#include "irrapp.h"

namespace av {

IrrApp::IrrApp(std::shared_ptr<Vehicle> vehicle, irr::core::dimension2d<irr::u32> dims)
    : ChVehicleIrrApp(&vehicle->GetVehicle(), nullptr, L"Corridor Demo", dims), m_vehicle(vehicle) {
    //
}

void IrrApp::renderOtherStats(int left, int top) {
    char msg[100];
    
    if (m_vehicle->m_SPAT_phase >= 0) {
        sprintf(msg, "MAP id: %d", m_vehicle->m_MAP_id);
        renderTextBox(std::string(msg), left, top - 130, 120, 15, irr::video::SColor(255, 255, 0, 0));
        sprintf(msg, "SPaT phase: %d", m_vehicle->m_SPAT_phase);
        renderTextBox(std::string(msg), left, top - 110, 120, 15, irr::video::SColor(255, 255, 0, 0));
        sprintf(msg, "SPaT time: %.1f", m_vehicle->m_SPAT_time);
        renderTextBox(std::string(msg), left, top - 90, 120, 15, irr::video::SColor(255, 255, 0, 0));
    }
}

}  // end namespace av
