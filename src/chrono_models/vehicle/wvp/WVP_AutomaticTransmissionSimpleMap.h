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

#ifndef WVP_AUTOMATIC_TRANSMISSION_SIMPLEMAP_H
#define WVP_AUTOMATIC_TRANSMISSION_SIMPLEMAP_H

#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace wvp {

/// @addtogroup vehicle_models_wvp
/// @{

/// WVP automatic transmission model template based on a simple gear-shifting model.
class CH_MODELS_API WVP_AutomaticTransmissionSimpleMap : public ChAutomaticTransmissionSimpleMap {
  public:
    WVP_AutomaticTransmissionSimpleMap();

    /// Set the gears, i.e. the transmission ratios of the various gears.
    /// A concrete class must populate the vector of forward gear ratios, ordered as 1st, 2nd, etc.
    /// and provide a value for the single reverse gear ratio.
    virtual void SetGearRatios(std::vector<double>& fwd_gear_ratios,  ///< [out] list of forward gear ratios
                               double& reverse_gear_ratio             ///< [out] single reverse gear ratio
                               ) override;

    /// Set the ideal shift points for automatic gear shifting.
    /// For each forward gear, specify a pair (min, max) with the minimum and
    /// maximum engine speed for shifting (down and up, respectively).
    virtual void SetShiftPoints(
        std::vector<std::pair<double, double>>& shift_bands  ///< [out] down-shift/up-shift points
        ) override;
};

/// @} vehicle_models_wvp

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono

#endif
