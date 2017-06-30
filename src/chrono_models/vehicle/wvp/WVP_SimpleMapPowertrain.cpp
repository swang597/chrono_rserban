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
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Simple powertrain model for the M113 vehicle.
// - both power and torque limited
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_models/vehicle/wvp/WVP_SimpleMapPowertrain.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_SimpleMapPowertrain::WVP_SimpleMapPowertrain()
    : ChPowertrain(),
      m_motorSpeed(0),
      m_motorTorque(0),
      m_shaftTorque(0),
      m_gear_ratios({-0.2, 0.1708, 0.2791, 0.4218, 0.6223, 1.0173, 1.5361})//,
      /*m_zeroThrottleMap({-10.472,83.776,104.720,125.664,146.608,167.552,188.496,209.440,230.383,251.327},
                        {0.000,-20.0, -20.0, -30.0, -30.0, -30.000, -40.000, -50.000, -70.000, -800.000}),
      m_fullThrottleMap({-10.472,83.776,104.720,125.664,146.608,167.552,188.496,209.440,230.383,251.327},
                        {406.7,517.9,926.0,1216.2,1300.2,1227.0,1136.2,1041.3,-271.2})*/

                         {
                           m_zeroThrottleFunction.AddPoint(-10.472,0.000);
                           m_zeroThrottleFunction.AddPoint(83.776,-20.0);
                           m_zeroThrottleFunction.AddPoint(104.720,-20.0);
                           m_zeroThrottleFunction.AddPoint(125.664,-30.0);
                           m_zeroThrottleFunction.AddPoint(146.608,-30.0);
                           m_zeroThrottleFunction.AddPoint(167.552,-30.0);
                           m_zeroThrottleFunction.AddPoint(188.496,-40.0);
                           m_zeroThrottleFunction.AddPoint(209.440,-50.0);
                           m_zeroThrottleFunction.AddPoint(230.383,-70.0);
                           m_zeroThrottleFunction.AddPoint(251.327,-100.0);
                           m_zeroThrottleFunction.AddPoint(282.743,-800.0);

                           m_fullThrottleFunction.AddPoint(-10.472,406.7);
                           m_fullThrottleFunction.AddPoint(83.776,517.9);
                           m_fullThrottleFunction.AddPoint(104.720,926.0);
                           m_fullThrottleFunction.AddPoint(125.664,1216.2);
                           m_fullThrottleFunction.AddPoint(146.608,1300.2);
                           m_fullThrottleFunction.AddPoint(167.552,1300.2);
                           m_fullThrottleFunction.AddPoint(188.496,1227.0);
                           m_fullThrottleFunction.AddPoint(209.440,1136.2);
                           m_fullThrottleFunction.AddPoint(230.383,1041.3);
                           m_fullThrottleFunction.AddPoint(251.327,-271.2);
                           m_fullThrottleFunction.AddPoint(282.743,-800.0);

                         }

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_SimpleMapPowertrain::Initialize(std::shared_ptr<ChBody> chassis, std::shared_ptr<ChShaft> driveshaft) {
    SetSelectedGear(1);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_SimpleMapPowertrain::SetSelectedGear(int igear) {
    assert(igear >= 0);
    assert(igear < m_gear_ratios.size());
    /*std::cout<<"Gear change: "<<m_current_gear<<"->"<<igear<<std::endl;*/
    m_current_gear = igear;
    m_current_gear_ratio = m_gear_ratios[igear];

}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_SimpleMapPowertrain::SetDriveMode(ChPowertrain::DriveMode mode) {
    m_drive_mode = mode;
    switch (mode) {
        case FORWARD:
            SetSelectedGear(1);
            break;
        case REVERSE:
            SetSelectedGear(0);
            break;
        case NEUTRAL:
            m_current_gear_ratio = 1e20;
            break;
    }
}

//check if transmission should shift based on "ideal" shift points
//TODO: make this take a small amount of time - might cause issues to be instantaneous
void WVP_SimpleMapPowertrain::CheckShift(){
  switch (m_current_gear){
    case 1:
      if(m_motorSpeed>(2226.0*CH_C_PI / 30.)) SetSelectedGear(2);
      break;
    case 2:
      if(m_motorSpeed>(2225.0*CH_C_PI / 30.)) SetSelectedGear(3);
      else if(m_motorSpeed<(1000.0*CH_C_PI / 30.)) SetSelectedGear(1);
      break;
    case 3:
      if(m_motorSpeed>(2210.0*CH_C_PI / 30.)) SetSelectedGear(4);
      else if(m_motorSpeed<(1000.0*CH_C_PI / 30.)) SetSelectedGear(2);
      break;
    case 4:
      if(m_motorSpeed>(2226.0*CH_C_PI / 30.)) SetSelectedGear(5);
      else if(m_motorSpeed<(1000.0*CH_C_PI / 30.)) SetSelectedGear(3);
      break;
    case 5:
      if(m_motorSpeed>(2225.0*CH_C_PI / 30.)) SetSelectedGear(6);
      else if(m_motorSpeed<(1000.0*CH_C_PI / 30.)) SetSelectedGear(4);
      break;
    case 6:
      if(m_motorSpeed<(1000.0*CH_C_PI / 30.)) SetSelectedGear(5);
      break;
    default: break;
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_SimpleMapPowertrain::Synchronize(double time, double throttle, double shaft_speed) {
    // The motor speed is the shaft speed multiplied by gear ratio inversed: (limited to 8000rpm)
    CheckShift();

    m_motorSpeed = shaft_speed / m_current_gear_ratio;
    m_motorSpeed = m_motorSpeed > (2700. * CH_C_PI / 30.) ? (2700. * CH_C_PI / 30.) : m_motorSpeed;
    m_motorSpeed = m_motorSpeed < 0.0 ? 0.0 : m_motorSpeed;

    // Motor torque is linearly interpolated by throttle gas value:
    double zeroThrottleTorque;
    double fullThrottleTorque;
    double curve_dot;   // not used
    double curve_ddot;  // not used
    /*m_zeroThrottleMap.Evaluate(m_motorSpeed, zeroThrottleTorque, curve_dot, curve_ddot);*/
    /*m_fullThrottleMap.Evaluate(m_motorSpeed, fullThrottleTorque, curve_dot, curve_ddot);*/
    fullThrottleTorque = m_fullThrottleFunction.Get_y(m_motorSpeed);
    zeroThrottleTorque = m_zeroThrottleFunction.Get_y(m_motorSpeed);

    m_motorTorque = zeroThrottleTorque * (1 - throttle) + fullThrottleTorque * (throttle);

    // The torque at motor shaft:
    m_shaftTorque = m_motorTorque / m_current_gear_ratio;

    std::cout<<"Engine speed|"<<m_motorSpeed<<"|Gear|"<<m_current_gear<<"|Engine torque|"<<m_motorTorque<<std::endl;
}

}  // end namespace WVP
}  // end namespace vehicle
}  // end namespace chrono
