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
// Author: Radu Serban
// =============================================================================
//
// =============================================================================

#ifndef AV_IRRAPP_H
#define AV_IRRAPP_H

#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "vehicle.h"

namespace av {

class IrrApp : public chrono::vehicle::ChVehicleIrrApp {
  public:
    IrrApp(std::shared_ptr<Vehicle> vehicle,
           irr::core::dimension2d<irr::u32> dims = irr::core::dimension2d<irr::u32>(1000, 800));

  private:
    virtual void renderOtherStats(int left, int top) override;

    std::shared_ptr<Vehicle> m_vehicle;
};

}  // end namespace av

#endif