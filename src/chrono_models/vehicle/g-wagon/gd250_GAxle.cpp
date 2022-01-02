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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Rear GD250 suspension subsystem.
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/g-wagon/gd250_GAxle.h"

namespace chrono {
namespace vehicle {
namespace gwagon {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double GD250_GAxle::m_axleTubeMass = 124.0;
const double GD250_GAxle::m_panhardRodMass = 10.0;
const double GD250_GAxle::m_longLinkMass = 10.0;
const double GD250_GAxle::m_arbMass = 5.0;
const double GD250_GAxle::m_spindleMass = 14.705;

const double GD250_GAxle::m_axleTubeRadius = 0.0476;
const double GD250_GAxle::m_panhardRodRadius = 0.03;
const double GD250_GAxle::m_longLinkRadius = 0.03;
const double GD250_GAxle::m_arbRadius = 0.025;
const double GD250_GAxle::m_spindleRadius = 0.10;
const double GD250_GAxle::m_spindleWidth = 0.06;

const ChVector<> GD250_GAxle::m_axleTubeInertia(22.21, 0.0775, 22.21);
const ChVector<> GD250_GAxle::m_panhardRodInertia(1.0, 0.04, 1.0);
const ChVector<> GD250_GAxle::m_longLinkInertia(0.04, 1.0, 1.0);
const ChVector<> GD250_GAxle::m_arbInertia(0.02, 0.5, 0.5);
const ChVector<> GD250_GAxle::m_spindleInertia(0.04117, 0.07352, 0.04117);

const double GD250_GAxle::m_arb_stiffness = 1000.0;
const double GD250_GAxle::m_arb_damping = 10.0;

const double GD250_GAxle::m_springDesignLength = 0.3;
const double GD250_GAxle::m_springCoefficient = 102328.0584;
const double GD250_GAxle::m_springRestLength = m_springDesignLength + 0.0621225507207084;
const double GD250_GAxle::m_springMinLength = m_springDesignLength - 0.08;
const double GD250_GAxle::m_springMaxLength = m_springDesignLength + 0.08;
const double GD250_GAxle::m_damperCoefficient = 25591.00572;
const double GD250_GAxle::m_damperDegressivityCompression = 3.0;
const double GD250_GAxle::m_damperDegressivityExpansion = 1.0;
const double GD250_GAxle::m_axleShaftInertia = 0.4;

// ---------------------------------------------------------------------------------------
// GD250 spring functor class - implements a linear spring + bump stop + rebound stop
// ---------------------------------------------------------------------------------------
class GD250_SpringForceRearCmpl : public ChLinkTSDA::ForceFunctor {
  public:
    GD250_SpringForceRearCmpl(double spring_constant, double min_length, double max_length);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;

  private:
    double m_spring_constant;
    double m_min_length;
    double m_max_length;

    ChFunction_Recorder m_bump;
};

GD250_SpringForceRearCmpl::GD250_SpringForceRearCmpl(double spring_constant, double min_length, double max_length)
    : m_spring_constant(spring_constant), m_min_length(min_length), m_max_length(max_length) {
    // From ADAMS/Car
    m_bump.AddPoint(0.0, 0.0);
    m_bump.AddPoint(2.0e-3, 200.0);
    m_bump.AddPoint(4.0e-3, 400.0);
    m_bump.AddPoint(6.0e-3, 600.0);
    m_bump.AddPoint(8.0e-3, 800.0);
    m_bump.AddPoint(10.0e-3, 1000.0);
    m_bump.AddPoint(20.0e-3, 2500.0);
    m_bump.AddPoint(30.0e-3, 4500.0);
    m_bump.AddPoint(40.0e-3, 7500.0);
    m_bump.AddPoint(50.0e-3, 12500.0);
}

double GD250_SpringForceRearCmpl::evaluate(double time,
                                           double rest_length,
                                           double length,
                                           double vel,
                                           const ChLinkTSDA& link) {
    double force = 0;

    double defl_spring = rest_length - length;
    double defl_bump = 0.0;
    double defl_rebound = 0.0;

    if (length < m_min_length) {
        defl_bump = m_min_length - length;
    }

    if (length > m_max_length) {
        defl_rebound = length - m_max_length;
    }

    force = defl_spring * m_spring_constant + m_bump.Get_y(defl_bump) - m_bump.Get_y(defl_rebound);

    return force;
}

// -----------------------------------------------------------------------------
// GD250 shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class GD250_ShockForceRearCmpl : public ChLinkTSDA::ForceFunctor {
  public:
    GD250_ShockForceRearCmpl(double compression_slope,
                             double compression_degressivity,
                             double expansion_slope,
                             double expansion_degressivity);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;

  private:
    double m_slope_compr;
    double m_slope_expand;
    double m_degres_compr;
    double m_degres_expand;
};

GD250_ShockForceRearCmpl::GD250_ShockForceRearCmpl(double compression_slope,
                                                   double compression_degressivity,
                                                   double expansion_slope,
                                                   double expansion_degressivity)
    : m_slope_compr(compression_slope),
      m_degres_compr(compression_degressivity),
      m_slope_expand(expansion_slope),
      m_degres_expand(expansion_degressivity) {}

double GD250_ShockForceRearCmpl::evaluate(double time,
                                          double rest_length,
                                          double length,
                                          double vel,
                                          const ChLinkTSDA& link) {
    // Simple model of a degressive damping characteristic
    double force = 0;

    // Calculate Damping Force
    if (vel >= 0) {
        force = -m_slope_expand / (1.0 + m_degres_expand * std::abs(vel)) * vel;
    } else {
        force = -m_slope_compr / (1.0 + m_degres_compr * std::abs(vel)) * vel;
    }

    return force;
}

GD250_GAxle::GD250_GAxle(const std::string& name) : ChGAxle(name) {
    m_springForceCB =
        chrono_types::make_shared<GD250_SpringForceRearCmpl>(m_springCoefficient, m_springMinLength, m_springMaxLength);

    m_shockForceCB = chrono_types::make_shared<GD250_ShockForceRearCmpl>(
        m_damperCoefficient, m_damperDegressivityCompression, m_damperCoefficient, m_damperDegressivityExpansion);

    m_bushingData = std::make_shared<ChVehicleBushingData>();
    // Hutchinson Radiaflex #511157
    double load = 1200;
    double defl = 0.007;
    m_bushingData->K_lin = load / defl;
    // rubber wisdom from
    // ContiTech
    // https://www.continental-industry.com/antivibration/de/Theorie?site2open=SchwingmetallProduktbeschreibungWirkungsweiseDaempfung
    // D = 0.5*tan(phi)
    // 40 Shore -> phi = 2.54 deg
    // 55 Shore -> phi = 4.5 deg
    // 65 Shore -> phi = 7.0 deg
    double phi = 2.54;                            // mechanical loss angle
    double D = 0.5 * tan(CH_C_DEG_TO_RAD * phi);  // Degree of damping
    double m = 120;                               // axle mass
    double f0 = 10.0;                             // est. axle eigenfrequency
    double w0 = CH_C_2PI * f0;                    // nat. axle eigenfrequency
    m_bushingData->D_lin = 2.0 * m * w0 * D;
    std::cout << "Test Damping = " << (100.0 * m_bushingData->D_lin / m_bushingData->K_lin) << " %" << std::endl;
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
GD250_GAxle::~GD250_GAxle() {}

const ChVector<> GD250_GAxle::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector<>(0.0, 0.5142, m_axleTubeRadius);
        case SPRING_C:
            return ChVector<>(0.0, 0.5142, m_axleTubeRadius + m_springDesignLength);
        case SHOCK_A:
            return ChVector<>(0.125, 0.5842, -0.0507);
        case SHOCK_C:
            return ChVector<>(0.20, 0.5142, m_axleTubeRadius + m_springDesignLength);
        case SPINDLE:
            return ChVector<>(0.0, 0.7325, 0.0);
        case PANHARD_A:
            return ChVector<>(-0.1, -0.5142, 0.0);
        case PANHARD_C:
            return ChVector<>(-0.1, 0.5142, 0.0);
        case LONGLINK_O:
            return ChVector<>(-0.1, 0.5142, -0.05);
        case LONGLINK_I:
            return ChVector<>(0.1, 0.5142, -0.05);
        case LONGLINK_C:
            return ChVector<>(0.8, 0.5142, -0.05);
        case ANTIROLL_A:
            return ChVector<>(0.0, 0.4, -0.05);
        case ANTIROLL_C:
            return ChVector<>(0.4, 0.4, -0.05);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // namespace gwagon
}  // end namespace vehicle
}  // end namespace chrono
