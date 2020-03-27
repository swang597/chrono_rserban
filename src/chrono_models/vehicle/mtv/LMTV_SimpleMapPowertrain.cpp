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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simple powertrain model for the LMTV 2.5t vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// Original Eingine: Caterpillar 3116 6.6 litre 4-stroke Diesel 6 cylinder inline 171 kW
//          Gearbox: Allison 4700SP
//
//
// =============================================================================

#include "chrono_models/vehicle/mtv/LMTV_SimpleMapPowertrain.h"

namespace chrono {
namespace vehicle {
namespace mtv {

const double rpm2rads = CH_C_PI / 30;
const double lbft2nm = 1.3558;

LMTV_SimpleMapPowertrain::LMTV_SimpleMapPowertrain(const std::string& name) : ChSimpleMapPowertrain(name) {}

double LMTV_SimpleMapPowertrain::GetMaxEngineSpeed() {
    return 2400 * rpm2rads;
}

void LMTV_SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    map0.AddPoint(-100 * rpm2rads, 0.000 * lbft2nm);
    map0.AddPoint(0 * rpm2rads, 0.0 * lbft2nm);
    map0.AddPoint(100 * rpm2rads, 0.0 * lbft2nm);
    map0.AddPoint(400 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(600 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(800 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1000 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1200 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1400 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1600 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1800 * rpm2rads, -30.0 * lbft2nm);
    map0.AddPoint(2000 * rpm2rads, -30.0 * lbft2nm);
    map0.AddPoint(2100 * rpm2rads, -40.0 * lbft2nm);
    map0.AddPoint(2300 * rpm2rads, -100.0 * lbft2nm);
    map0.AddPoint(2500 * rpm2rads, -150.0 * lbft2nm);

    mapF.AddPoint(-100.0 * rpm2rads, 0.00);
    mapF.AddPoint(0 * rpm2rads, 200.00);
    mapF.AddPoint(500 * rpm2rads, 300);
    mapF.AddPoint(1000 * rpm2rads, 500);
    mapF.AddPoint(1200 * rpm2rads, 572);
    mapF.AddPoint(1400 * rpm2rads, 664);
    mapF.AddPoint(1600 * rpm2rads, 713);
    mapF.AddPoint(1800 * rpm2rads, 733);
    mapF.AddPoint(2000 * rpm2rads, 725);
    mapF.AddPoint(2100 * rpm2rads, 717);
    mapF.AddPoint(2200 * rpm2rads, 707);
    mapF.AddPoint(2400 * rpm2rads, 682);
    mapF.AddPoint(2500 * rpm2rads, 0);
}

void LMTV_SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd_gear_ratios, double& reverse_gear_ratio) {
    reverse_gear_ratio = -1.0 / 11.8;

    fwd_gear_ratios.push_back(1.0 / 11.921875);
    fwd_gear_ratios.push_back(1.0 / 5.484375);
    fwd_gear_ratios.push_back(1.0 / 2.984375);
    fwd_gear_ratios.push_back(1.0 / 2.234375);
    fwd_gear_ratios.push_back(1.0 / 1.5625);
    fwd_gear_ratios.push_back(1.0 / 1.15625);
    fwd_gear_ratios.push_back(1.0);
}

void LMTV_SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1100 * rpm2rads, 2300 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1100 * rpm2rads, 2300 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1100 * rpm2rads, 2300 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1100 * rpm2rads, 2300 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1100 * rpm2rads, 2300 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1100 * rpm2rads, 2300 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1100 * rpm2rads, 2300 * rpm2rads));
}

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono
