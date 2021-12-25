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
// Authors: Alessandro Tasora
// =============================================================================
//
// UAZBUS simple brake models (front and rear).
//
// =============================================================================

#ifndef GD250_BRAKESIMPLE_H
#define GD250_BRAKESIMPLE_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
    namespace vehicle {
        namespace gwagon {

/// @addtogroup vehicle_models_uaz
/// @{

/// Simple UAZBUS front brake subsystem (torque applied directly to the spindle joint).
            class CH_MODELS_API GD250_BrakeSimpleFront : public ChBrakeSimple {
            public:
            GD250_BrakeSimpleFront(const std::string& name);
            virtual ~GD250_BrakeSimpleFront() {}

            virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

        private:
        static const double m_maxtorque;
    };

/// Simple UAZBUS rear brake subsystem (torque applied directly to the spindle joint).
    class CH_MODELS_API GD250_BrakeSimpleRear : public ChBrakeSimple {
    public:
    GD250_BrakeSimpleRear(const std::string& name);
    virtual ~GD250_BrakeSimpleRear() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

private:
static const double m_maxtorque;
};

/// @} vehicle_models_uaz

}  // end namespace gwagon
}  // end namespace vehicle
}  // end namespace chrono

#endif
