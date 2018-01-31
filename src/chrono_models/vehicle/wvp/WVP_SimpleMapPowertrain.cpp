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
// Authors: Radu Serban, Mike Taylor, Asher Elmquist
// =============================================================================
//
// Simple powertrain model for the HMMWV vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/wvp/WVP_SimpleMapPowertrain.h"

namespace chrono {
namespace vehicle {
namespace wvp {

const double rpm2rads = CH_C_PI / 30;
const double drive_eff = 0.5137; //based on auxillary power consumption and gearbox efficiencies

WVP_SimpleMapPowertrain::WVP_SimpleMapPowertrain() : ChSimpleMapPowertrain("Powertrain") {}

double WVP_SimpleMapPowertrain::GetMaxEngineSpeed() {
    return 2700 * rpm2rads;
}

void WVP_SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    map0.AddPoint(-100 * rpm2rads, 0.0);
    map0.AddPoint(800 * rpm2rads, -20.0);
    map0.AddPoint(1000 * rpm2rads, -20.0);
    map0.AddPoint(1200 * rpm2rads, -30.0);
    map0.AddPoint(1400 * rpm2rads, -30.0);
    map0.AddPoint(1600 * rpm2rads, -30.0);
    map0.AddPoint(1800 * rpm2rads, -40.0);
    map0.AddPoint(2000 * rpm2rads, -50.0);
    map0.AddPoint(2200 * rpm2rads, -70.0);
    map0.AddPoint(2400 * rpm2rads, -100.0);
    map0.AddPoint(2700 * rpm2rads, -800.0);

    mapF.AddPoint(-100 * rpm2rads, 806.7*drive_eff); //406.7
    mapF.AddPoint(800 * rpm2rads, 817.9*drive_eff); //517.9
    mapF.AddPoint(1000 * rpm2rads, 926.0*drive_eff);
    mapF.AddPoint(1200 * rpm2rads, 1216.2*drive_eff);
    mapF.AddPoint(1400 * rpm2rads, 1300.2*drive_eff);
    mapF.AddPoint(1600 * rpm2rads, 1300.2*drive_eff);
    mapF.AddPoint(1800 * rpm2rads, 1227.0*drive_eff);
    mapF.AddPoint(2000 * rpm2rads, 1136.2*drive_eff);
    mapF.AddPoint(2200 * rpm2rads, 1041.3*drive_eff);
    mapF.AddPoint(2400 * rpm2rads, 0);
    mapF.AddPoint(2700 * rpm2rads, -200.0);
}

void WVP_SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd_gear_ratios, double& reverse_gear_ratio) {
    reverse_gear_ratio = -0.2;

    fwd_gear_ratios.push_back(0.1708);  // 1st
    fwd_gear_ratios.push_back(0.2791);  // 2nd
    fwd_gear_ratios.push_back(0.4218);  // 3rd
    fwd_gear_ratios.push_back(0.6223);  // 4th
    fwd_gear_ratios.push_back(1.0173);  // 5th
    fwd_gear_ratios.push_back(1.5361);  // 6th
}

void WVP_SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
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
