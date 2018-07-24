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

#ifndef AV_VAN_H
#define AV_VAN_H

#include "chrono_models/vehicle/uaz/UAZBUS.h"

#include "vehicle.h"

namespace av {

class VanAV : public Vehicle {
  public:
    ~VanAV();

  private:
    VanAV(Framework* framework, const chrono::ChCoordsys<>& init_pos);

    virtual chrono::ChVector<> GetPosition() override { return m_uaz->GetVehicle().GetVehiclePos(); }
    virtual chrono::vehicle::ChVehicle& GetVehicle() override { return m_uaz->GetVehicle(); }
    virtual chrono::vehicle::ChPowertrain& GetPowertrain() override { return m_uaz->GetPowertrain(); }

    virtual double GetLookAheadDistance() override { return 5; }
    virtual chrono::ChVector<> GetSteeringGainsPID() override { return chrono::ChVector<>(0.8, 0, 0); }
    virtual chrono::ChVector<> GetSpeedGansPID() { return chrono::ChVector<>(0.4, 0, 0); }

    virtual void Synchronize(double time) override;
    virtual void Advance(double step) override;

    std::shared_ptr<chrono::vehicle::uaz::UAZBUS> m_uaz;

    friend class Framework;
};

}  // end namespace av

#endif
