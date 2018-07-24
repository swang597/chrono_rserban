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

#ifndef AV_VEHICLE_KD_H
#define AV_VEHICLE_KD_H


#include "agent.h"

namespace av {

class VehicleKD : public Vehicle {
  public:
    ~VehicleKD();

  private:
    VehicleKD(Framework* framework, const chrono::ChCoordsys<>& pos);
    virtual void Synchronize(double time);
    virtual void Advance(double step);

    friend class Framework;
};

}  // end namespace av

#endif
