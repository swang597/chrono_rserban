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
// Authors: Radu Serban
// =============================================================================
//
// =============================================================================

#include "vehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace av {

const double Vehicle::m_throttle_threshold = 0.2;
VehicleList Vehicle::m_vehicles;

// -----------------------------------------------------------------------------

Vehicle::Vehicle(Framework* framework) : Agent(framework), m_steeringPID(nullptr), m_speedPID(nullptr), m_steering(0), m_throttle(0), m_braking(0) {
    //
}

Vehicle::~Vehicle() {
    delete m_steeringPID;
    delete m_speedPID;
}

std::shared_ptr<Vehicle> Vehicle::Find(unsigned int id) {
    auto it = m_vehicles.find(id);
    if (it != m_vehicles.end())
        return it->second;
    return nullptr;
}

void Vehicle::SetupDriver(std::shared_ptr<chrono::ChBezierCurve> curve, bool closed, double target_speed) {
    m_steeringPID = new ChPathSteeringController(curve, closed);
    m_speedPID = new ChSpeedController();
    m_target_speed = target_speed;

    double dist = GetLookAheadDistance();
    ChVector<> steering_gains = GetSteeringGainsPID();
    ChVector<> speed_gains = GetSpeedGainsPID();
    m_steeringPID->SetLookAheadDistance(dist);
    m_steeringPID->SetGains(steering_gains.x(), steering_gains.y(), steering_gains.z());
    m_speedPID->SetGains(speed_gains.x(), speed_gains.y(), speed_gains.z());

    m_steeringPID->Reset(GetVehicle());
    m_speedPID->Reset(GetVehicle());
}

void Vehicle::AdvanceDriver(double step) {
    // Set the throttle and braking values based on the output from the speed controller.
    double out_speed = m_speedPID->Advance(GetVehicle(), m_target_speed, step);
    ChClampValue(out_speed, -1.0, 1.0);

    if (out_speed > 0) {
        // Vehicle moving too slow
        m_braking = 0;
        m_throttle = out_speed;
    } else if (m_throttle > m_throttle_threshold) {
        // Vehicle moving too fast: reduce throttle
        m_braking = 0;
        m_throttle = 1 + out_speed;
    } else {
        // Vehicle moving too fast: apply brakes
        m_braking = -out_speed;
        m_throttle = 0;
    }

    // Set the steering value based on the output from the steering controller.
    double out_steering = m_steeringPID->Advance(GetVehicle(), step);
    ChClampValue(out_steering, -1.0, 1.0);
    m_steering = out_steering;
}

}  // end namespace av
