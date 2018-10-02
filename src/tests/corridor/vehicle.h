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

#ifndef AV_VEHICLE_H
#define AV_VEHICLE_H

#include "chrono/core/ChBezierCurve.h"

#include "chrono_vehicle/ChPowertrain.h"
#include "chrono_vehicle/ChVehicle.h"

#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChSteeringController.h"

#include "agent.h"

namespace av {

class Vehicle;

typedef std::unordered_map<unsigned int, std::shared_ptr<Vehicle>> VehicleList;

class Vehicle : public Agent {
  public:
    enum Type { TRUCK, SEDAN, VAN, BUS };

    virtual ~Vehicle();

    Type GetType() const { return m_vehicle_type; }
    const std::string& GetTypeName() const { return m_types[m_vehicle_type]; }

    virtual chrono::ChCoordsys<> GetPosition() const override;
    virtual void Broadcast(double time) override;
    virtual void Unicast(double time) override;
    virtual void ProcessMessages() override;

    virtual chrono::vehicle::ChVehicle& GetVehicle() const = 0;
    virtual chrono::vehicle::ChPowertrain& GetPowertrain() const = 0;

    virtual double GetLookAheadDistance() const = 0;
    virtual chrono::ChVector<> GetSteeringGainsPID() const = 0;
    virtual chrono::ChVector<> GetSpeedGainsPID() const = 0;

    static std::shared_ptr<Vehicle> Find(unsigned int id);
    static VehicleList GetList() { return m_vehicles; }

  protected:
    Vehicle(Framework* framework, Type vehicle_type);

    void SetupDriver(std::shared_ptr<chrono::ChBezierCurve> curve, bool closed, double target_speed);
    void AdvanceDriver(double step);

    double m_steering;
    double m_throttle;
    double m_braking;

  private:
    Type m_vehicle_type;

    chrono::vehicle::ChPathSteeringController* m_steeringPID;
    chrono::vehicle::ChSpeedController* m_speedPID;
    double m_target_speed;

    static const double m_throttle_threshold;
    static const std::string m_types[4];
    static VehicleList m_vehicles;

    friend class Framework;
};

}  // end namespace av

#endif
