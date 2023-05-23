// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Mike Taylor, Asher Elmquist
// =============================================================================
//
// Automatic transmssion model for the WVP vehicle.
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/wvp/WVP_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace wvp {

const double rpm2rads = CH_C_PI / 30;
const double drive_eff = 0.5137; //based on auxillary power consumption and gearbox efficiencies

WVP_AutomaticTransmissionSimpleMap::WVP_AutomaticTransmissionSimpleMap()
    : ChAutomaticTransmissionSimpleMap("Transmission") {}

void WVP_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd_gear_ratios,
                                                       double& reverse_gear_ratio) {
    reverse_gear_ratio = -0.2;

    fwd_gear_ratios.push_back(0.1708);  // 1st
    fwd_gear_ratios.push_back(0.2791);  // 2nd
    fwd_gear_ratios.push_back(0.4218);  // 3rd
    fwd_gear_ratios.push_back(0.6223);  // 4th
    fwd_gear_ratios.push_back(1.0173);  // 5th
    fwd_gear_ratios.push_back(1.5361);  // 6th
}

void WVP_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2210 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2700 * rpm2rads));
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
