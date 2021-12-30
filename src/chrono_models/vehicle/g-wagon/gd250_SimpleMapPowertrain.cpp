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
// Simple powertrain model for the UAZBUS vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/g-wagon/gd250_SimpleMapPowertrain.h"

namespace chrono {
    namespace vehicle {
        namespace gwagon {

            const double rpm2rads = CH_C_PI / 30;

            GD250_SimpleMapPowertrain::GD250_SimpleMapPowertrain(const std::string& name) : ChSimpleMapPowertrain(name) {}

            double GD250_SimpleMapPowertrain::GetMaxEngineSpeed() {
                return 4600 * rpm2rads;
            }

            void GD250_SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
                // Mercedes 5 Cyl. Diesel Engine OM602D25 66kW
                map0.AddPoint(-10.0,	                          0.0);
                map0.AddPoint(10.0,	                          0.0);
                map0.AddPoint(rpm2rads*1000.00,	-10.00);
                map0.AddPoint(rpm2rads*1530.00,	-12.00);
                map0.AddPoint(rpm2rads*2050.00,	-14.00);
                map0.AddPoint(rpm2rads*2550.00,	-16.00);
                map0.AddPoint(rpm2rads*2880.00,	-18.00);
                map0.AddPoint(rpm2rads*3070.00,	-20.00);
                map0.AddPoint(rpm2rads*3560.00,	-22.00);
                map0.AddPoint(rpm2rads*4030.00,	-24.00);
                map0.AddPoint(rpm2rads*4550.00,	-26.00);
                map0.AddPoint(rpm2rads*4700.00,	-28.00);

                mapF.AddPoint(-10.0,	        25);
                mapF.AddPoint(rpm2rads*300.00,	50.00);
                mapF.AddPoint(rpm2rads*700.00,	100.00);
                mapF.AddPoint(rpm2rads*1000.00,	132.00);
                mapF.AddPoint(rpm2rads*1530.00,	144.00);
                mapF.AddPoint(rpm2rads*2050.00,	152.00);
                mapF.AddPoint(rpm2rads*2550.00,	155.00);
                mapF.AddPoint(rpm2rads*2880.00,	156.00);
                mapF.AddPoint(rpm2rads*3070.00,	154.00);
                mapF.AddPoint(rpm2rads*3560.00,	153.00);
                mapF.AddPoint(rpm2rads*4030.00,	150.00);
                mapF.AddPoint(rpm2rads*4550.00,	140.00);
                mapF.AddPoint(rpm2rads*4600.00,	-100);
                mapF.AddPoint(rpm2rads*4700.00,	-500);
            }

            void GD250_SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
                rev = -1.0/5.224;

                fwd.push_back(1.0/4.124/1.94);
                fwd.push_back(1.0/4.124);
                fwd.push_back(1.0/2.641);
                fwd.push_back(1.0/1.58);
                fwd.push_back(1.0);
            }

            void GD250_SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
                shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3000 * rpm2rads));
                shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
                shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
                shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
                shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
                // shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 4000 * rpm2rads));
            }

        }  // end namespace uaz
    }  // end namespace vehicle
}  // end namespace chrono
