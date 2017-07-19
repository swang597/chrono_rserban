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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// A driver model that uses a path to bring the vehicle up to speed. It then
// switches to an open loop controller based on steering data provided in a
// text file. The vehicle will still try to maintain the original
// target velocity
//
// =============================================================================

#ifndef WVP_FOLLOWER_DATA_DRIVER_H
#define WVP_FOLLOWER_DATA_DRIVER_H

#include <string>

#include "chrono/motion_functions/ChFunction_Recorder.h"

#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChSteeringController.h"

namespace chrono {
namespace vehicle {
namespace wvp{

/// @addtogroup vehicle_driver
/// @{

/// Closed-loop path-follower driver model.
/// A driver model that uses a path steering controller and a speed controller.
/// The steering controller adjusts the steering input to follow the prescribed
/// path.  The output from the speed controller is used to adjust throttle and
/// braking inputs in order to maintain the prescribed constant vehicle speed.
///
/// @sa ChPathSteeringController
/// @sa ChSpeedController
class CH_VEHICLE_API WVP_FollowerDataDriver : public ChDriver {
  public:
    /// Construct using JSON specification files.
    /// The two files must contain specification for the path-follower steering controller
    /// and the constant-speed controller, respectively.
    WVP_FollowerDataDriver(ChVehicle& vehicle,                    ///< associated vehicle
                         const std::string& steering_filename,  ///< JSON file with steering controller specification
                         const std::string& speed_filename,     ///< JSON file with speed controller specification
                         std::shared_ptr<ChBezierCurve> path,   ///< Bezier curve with target path
                         const std::string& path_name,          ///< name of the path curve
                         double target_speed,                   ///< constant target speed
                         const std::string data_input_file, //location of data input file
                         int time_column,   //which column in csv is time data -> 0 indexed
                         int steering_column, //which column in csv is steering data -> 0 indexed
                         double time_til_steady //when should simulation switch to input data
                         );

    ~WVP_FollowerDataDriver() {}

    /// Set the desired vehicle speed.
    void SetDesiredSpeed(double val) { m_target_speed = val; }

    /// Specify the throttle value below which braking is enabled.
    /// If the vehicle is moving faster than the set speed, the controller attempts to
    /// reduce speed either by reducing the throttle input (if the current throttle input
    /// is above the threshold value) or by applying brakes (otherwise).
    void SetThreshholdThrottle(double val) { m_throttle_threshold = val; }

    /// Get the underlying steering controller object.
    ChPathSteeringController& GetSteeringController() { return m_steeringPID; }

    /// Get the underlying speed controller object.
    ChSpeedController& GetSpeedController() { return m_speedPID; }

    /// Reset the underlying controllers.
    void Reset();

    /// Advance the state of this driver system by the specified duration.
    virtual void Advance(double step) override;

    /// Export the Bezier curve for POV-Ray postprocessing.
    void ExportPathPovray(const std::string& out_dir);

  private:
    void Create();

    ChPathSteeringController m_steeringPID;  ///< steering controller
    ChSpeedController m_speedPID;            ///< speed controller
    double m_target_speed;                   ///< desired vehicle speed
    std::string m_pathName;                  ///< for path visualization
    double m_throttle_threshold;             ///< throttle value below which brakes are applied

    ChFunction_Recorder m_steering_map;
    double m_time_til_steady;
    double m_dataStartTime = 0;

    void ParseCSV(std::string m_data_input_file, int timeCol, int steeringCol);

};

/// @} vehicle_driver
} //end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono

#endif
