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
// MAN 7t simple driveline model.
//
// =============================================================================

#ifndef MAN7T_SIMPLEDRIVELINE_H
#define MAN7T_SIMPLEDRIVELINE_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline6WD.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace man {

/// @addtogroup vehicle_models_man
/// @{

/// Simple MAN 5t driveline subsystem (purely kinematic).
class CH_MODELS_API MAN_7t_SimpleDriveline : public ChSimpleDriveline6WD {
  public:
    MAN_7t_SimpleDriveline(const std::string& name);

    ~MAN_7t_SimpleDriveline() {}

    virtual double GetFrontDifferentialMaxBias() const override { return m_front_diff_bias; }
    virtual double GetMidDifferentialMaxBias() const override { return m_mid_diff_bias; }
    virtual double GetRearDifferentialMaxBias() const override { return m_rear_diff_bias; }

  private:
    static const double m_front_diff_bias;
    static const double m_mid_diff_bias;
    static const double m_rear_diff_bias;
};

/// @} vehicle_models_man

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono

#endif
