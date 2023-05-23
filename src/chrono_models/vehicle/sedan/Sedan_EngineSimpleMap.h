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
// Authors: Radu Serban, Asher Elmquist, Marcel Offermans
// =============================================================================
//
// Simple engine model for the Sedan vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
//
// =============================================================================

#ifndef SEDAN_ENGINESIMPLEMAP_H
#define SEDAN_ENGINESIMPLEMAP_H

#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace sedan {

/// @addtogroup vehicle_models_sedan
/// @{

/// Simple Sedan powertrain subsystem (based on engine speed-torque maps).
class CH_MODELS_API Sedan_EngineSimpleMap : public ChEngineSimpleMap {
  public:
    Sedan_EngineSimpleMap(const std::string& name);

    /// Specify maximum engine speed.
    double GetMaxEngineSpeed() override;

    /// Set the engine speed-torque maps.
    /// A concrete class must add the speed-torque points to the provided maps,
    /// using the ChFunction_Recorder::AddPoint() function.
    void SetEngineTorqueMaps(ChFunction_Recorder& map0,  ///< [out] engine map at zero throttle
                             ChFunction_Recorder& mapF   ///< [out] engine map at full throttle
                             ) override;
};

/// @} vehicle_models_sedan

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono

#endif
