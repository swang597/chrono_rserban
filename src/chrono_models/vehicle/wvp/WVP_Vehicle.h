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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// Base class for the WVP vehicle models
//
// =============================================================================

#ifndef WVP_VEHICLE_H
#define WVP_VEHICLE_H

#include <vector>

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

#include "chrono_models/vehicle/wvp/WVP_Chassis.h"
#include "chrono_models/vehicle/wvp/WVP_BrakeSimple.h"
#include "chrono_models/vehicle/wvp/WVP_DoubleWishbone.h"
#include "chrono_models/vehicle/wvp/WVP_Driveline4WD.h"
#include "chrono_models/vehicle/wvp/WVP_SimpleDriveline.h"
#include "chrono_models/vehicle/wvp/WVP_PitmanArm.h"
#include "chrono_models/vehicle/wvp/WVP_Wheel.h"

namespace chrono {
namespace vehicle {
namespace wvp {

class CH_MODELS_API WVP_Vehicle : public ChWheeledVehicle {
  public:
    WVP_Vehicle(const bool fixed = false,
                ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC,
                ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE);

    WVP_Vehicle(ChSystem* system,
                const bool fixed = false,
                ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE);

    ~WVP_Vehicle();

    virtual int GetNumberAxles() const override { return 2; }

    void SetInitWheelAngVel(const std::vector<double>& omega) {
        assert(omega.size() == 4);
        m_omega = omega;
    }

    double GetSpringForce(const WheelID& wheel_id) const;
    double GetSpringLength(const WheelID& wheel_id) const;
    double GetSpringDeformation(const WheelID& wheel_id) const;

    double GetShockForce(const WheelID& wheel_id) const;
    double GetShockLength(const WheelID& wheel_id) const;
    double GetShockVelocity(const WheelID& wheel_id) const;

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

    // Log debugging information
    void LogHardpointLocations();  /// suspension hardpoints at design
    void DebugLog(int what);       /// shock forces and lengths, constraints, etc.

  private:
    void Create(bool fixed, ChassisCollisionType chassis_collision_type);

    std::vector<double> m_omega;
};

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono

#endif
