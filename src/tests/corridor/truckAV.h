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

#ifndef AV_TRUCK_H
#define AV_TRUCK_H

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "vehicle.h"

namespace av {

class TruckAV : public Vehicle {
  public:
    ~TruckAV();

    void recieveMessage(Message newMessage);
    void sendMessages(double time);
    void processMessages();

  private:
    TruckAV(Framework* framework, const chrono::ChCoordsys<>& init_pos);

    virtual chrono::ChCoordsys<> GetPosition() const override;
    virtual chrono::vehicle::ChVehicle& GetVehicle() const override;
    virtual chrono::vehicle::ChPowertrain& GetPowertrain() const override;

    virtual double GetLookAheadDistance() const override { return 5; }
    virtual chrono::ChVector<> GetSteeringGainsPID() const override { return chrono::ChVector<>(0.8, 0, 0); }
    virtual chrono::ChVector<> GetSpeedGainsPID() const override { return chrono::ChVector<>(0.4, 0, 0); }

    virtual void Synchronize(double time) override;
    virtual void Advance(double step) override;

    std::shared_ptr<chrono::vehicle::hmmwv::HMMWV_Reduced> m_hmmwv;

    friend class Framework;
};

}  // end namespace av

#endif
