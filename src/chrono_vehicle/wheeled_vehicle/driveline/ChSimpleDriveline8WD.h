// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simple driveline model. This template can be used to model a 6WD driveline.
// It uses a constant front1/front2/rear1/rear2 torque split (a fixed value of
// 1/4) and a simple model for Torsen limited-slip differentials.
//
// =============================================================================

#ifndef CH_SIMPLE_DRIVELINE_8WD_H
#define CH_SIMPLE_DRIVELINE_8WD_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChDrivelineWV.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_driveline
/// @{

/// Simple driveline model. This template can be used to model a 6WD driveline.
/// It uses a constant front1/front2/rear1/rear2 torque split (a value of and 1/4)
/// and a simple model for Torsen limited-slip differentials.
class CH_VEHICLE_API ChSimpleDriveline8WD : public ChDrivelineWV {
  public:
    ChSimpleDriveline8WD(const std::string& name);

    virtual ~ChSimpleDriveline8WD() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "SimpleDriveline6WD"; }

    /// Return the number of driven axles.
    virtual int GetNumDrivenAxles() const final override { return 4; }

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the specified axle subsystems.
    virtual void Initialize(std::shared_ptr<ChBody> chassis,      ///< handle to the chassis body
                            const ChAxleList& axles,              ///< list of all vehicle axle subsystems
                            const std::vector<int>& driven_axles  ///< indexes of the driven vehicle axles
                            ) override;

    /// Get the angular speed of the driveshaft.
    /// This represents the output from the driveline subsystem that is passed to
    /// the powertrain system.
    virtual double GetDriveshaftSpeed() const override;

    /// Update the driveline subsystem: apply the specified motor torque.
    /// This represents the input to the driveline subsystem from the powertrain
    /// system.
    virtual void Synchronize(double torque) override;

    /// Get the motor torque to be applied to the specified spindle.
    virtual double GetSpindleTorque(int axle, VehicleSide side) const override;

  protected:
    /// Return the torque bias ratio for the front differential.
    /// This is a simple model of a Torsen limited-slip differential.
    virtual double GetFront1DifferentialMaxBias() const = 0;

    /// Return the torque bias ratio for the mid differential.
    /// This is a simple model of a Torsen limited-slip differential.
    virtual double GetFront2DifferentialMaxBias() const = 0;

    /// Return the torque bias ratio for the rear differential.
    /// This is a simple model of a Torsen limited-slip differential.
    virtual double GetRear1DifferentialMaxBias() const = 0;

    /// Return the torque bias ratio for the rear differential.
    /// This is a simple model of a Torsen limited-slip differential.
    virtual double GetRear2DifferentialMaxBias() const = 0;

  private:
    std::shared_ptr<ChShaft> m_front1_left;
    std::shared_ptr<ChShaft> m_front1_right;
    std::shared_ptr<ChShaft> m_front2_left;
    std::shared_ptr<ChShaft> m_front2_right;
    std::shared_ptr<ChShaft> m_rear1_left;
    std::shared_ptr<ChShaft> m_rear1_right;
    std::shared_ptr<ChShaft> m_rear2_left;
    std::shared_ptr<ChShaft> m_rear2_right;
};

/// @} vehicle_wheeled_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
