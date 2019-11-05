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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Front and Rear FEDA suspension subsystems (double A-arm).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/feda/FEDA_DoubleWishbone.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;
static const double lbfpin2Npm = 175.12677;
static const double psi2pascal = 6894.7572932;

const double FEDA_DoubleWishboneFront::m_UCAMass = 8.45;
const double FEDA_DoubleWishboneFront::m_LCAMass = 31.55;
const double FEDA_DoubleWishboneFront::m_uprightMass = 36.27;
const double FEDA_DoubleWishboneFront::m_spindleMass = 13.08;

const double FEDA_DoubleWishboneFront::m_spindleRadius = 0.10;
const double FEDA_DoubleWishboneFront::m_spindleWidth = 0.06;
const double FEDA_DoubleWishboneFront::m_LCARadius = 0.03;
const double FEDA_DoubleWishboneFront::m_UCARadius = 0.02;
const double FEDA_DoubleWishboneFront::m_uprightRadius = 0.04;

// TODO: Fix these values
const ChVector<> FEDA_DoubleWishboneFront::m_spindleInertia(5.32e-4, 5.52E-04, 5.32e-4);
const ChVector<> FEDA_DoubleWishboneFront::m_UCAInertiaMoments(0.03, 0.03, 0.06276);
const ChVector<> FEDA_DoubleWishboneFront::m_UCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> FEDA_DoubleWishboneFront::m_LCAInertiaMoments(0.4, 0.4, 0.8938);
const ChVector<> FEDA_DoubleWishboneFront::m_LCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> FEDA_DoubleWishboneFront::m_uprightInertiaMoments(0.1656, 0.1934, 0.04367);
const ChVector<> FEDA_DoubleWishboneFront::m_uprightInertiaProducts(0.0, 0.0, 0.0);

const double FEDA_DoubleWishboneFront::m_axleInertia = 0.4;

const double FEDA_DoubleWishboneFront::m_springCoefficient = 76000;
const double FEDA_DoubleWishboneFront::m_springRestLength = 0.60208;
const double FEDA_DoubleWishboneFront::m_bumpstop_clearance = 0.11;
const double FEDA_DoubleWishboneFront::m_reboundstop_clearance = 0.11;
const double FEDA_DoubleWishboneFront::m_air_pressure = 18.0 * psi2pascal;

// -----------------------------------------------------------------------------

const double FEDA_DoubleWishboneRear::m_UCAMass = 8.45;
const double FEDA_DoubleWishboneRear::m_LCAMass = 31.55;
const double FEDA_DoubleWishboneRear::m_uprightMass = 36.27;
const double FEDA_DoubleWishboneRear::m_spindleMass = 13.08;

const double FEDA_DoubleWishboneRear::m_spindleRadius = 0.10;
const double FEDA_DoubleWishboneRear::m_spindleWidth = 0.06;
const double FEDA_DoubleWishboneRear::m_LCARadius = 0.03;
const double FEDA_DoubleWishboneRear::m_UCARadius = 0.02;
const double FEDA_DoubleWishboneRear::m_uprightRadius = 0.04;

// TODO: Fix these values
const ChVector<> FEDA_DoubleWishboneRear::m_spindleInertia(5.32e-4, 5.52E-04, 5.32e-4);
const ChVector<> FEDA_DoubleWishboneRear::m_UCAInertiaMoments(0.03, 0.03, 0.06276);
const ChVector<> FEDA_DoubleWishboneRear::m_UCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> FEDA_DoubleWishboneRear::m_LCAInertiaMoments(0.4, 0.4, 0.8938);
const ChVector<> FEDA_DoubleWishboneRear::m_LCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> FEDA_DoubleWishboneRear::m_uprightInertiaMoments(0.1656, 0.1934, 0.04367);
const ChVector<> FEDA_DoubleWishboneRear::m_uprightInertiaProducts(0.0, 0.0, 0.0);

const double FEDA_DoubleWishboneRear::m_axleInertia = 0.4;

const double FEDA_DoubleWishboneRear::m_springCoefficient = 76000;
const double FEDA_DoubleWishboneRear::m_springRestLength = 0.60208;
const double FEDA_DoubleWishboneRear::m_bumpstop_clearance = 0.11;
const double FEDA_DoubleWishboneRear::m_reboundstop_clearance = 0.11;
const double FEDA_DoubleWishboneRear::m_air_pressure = 18.0 * psi2pascal;

// --------------------------------------------------------------------------------------------------------------------------------
// FEDA spring functor class - implements a linear spring in combination with an airspring + bistops
// --------------------------------------------------------------------------------------------------------------------------------
class AirCoilSpringBistopForce : public ChLinkSpringCB::ForceFunctor {
  public:
    /// Use default bump stop and rebound stop maps
    AirCoilSpringBistopForce(double k, double min_length, double max_length, double p0)
        : m_k(k),
          m_min_length(min_length),
          m_max_length(max_length),
          m_kappa(1.3),
          m_piston_radius(0.08),
          m_cylinder_compression_length(0.16),
          m_P0(p0) {
        // percalculations
        double A0 = pow(m_piston_radius, 2.0) * CH_C_PI;
        double V0 = m_cylinder_compression_length * A0;
        m_F0 = m_P0 * A0;
        m_hf0 = m_P0 * V0 / m_F0;

        // From ADAMS/Car example
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
        m_bump.AddPoint(60.0e-3, 125000.0);

        m_rebound.AddPoint(0.0, 0.0);
        m_rebound.AddPoint(2.0e-3, 200.0);
        m_rebound.AddPoint(4.0e-3, 400.0);
        m_rebound.AddPoint(6.0e-3, 600.0);
        m_rebound.AddPoint(8.0e-3, 800.0);
        m_rebound.AddPoint(10.0e-3, 1000.0);
        m_rebound.AddPoint(20.0e-3, 2500.0);
        m_rebound.AddPoint(30.0e-3, 4500.0);
        m_rebound.AddPoint(40.0e-3, 7500.0);
        m_rebound.AddPoint(50.0e-3, 12500.0);
        m_rebound.AddPoint(60.0e-3, 125000.0);
    }

    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override {
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

        force = m_F0 * pow(m_hf0, m_kappa) / pow(m_hf0 - defl_spring, m_kappa) + defl_spring * m_k +
                m_bump.Get_y(defl_bump) - m_rebound.Get_y(defl_rebound);

        return force;
    }

  private:
    double m_k;
    double m_min_length;
    double m_max_length;

    // airspring model (cylinder/piston)
    double m_kappa;  // polytropial exponent
    double m_piston_radius;
    double m_cylinder_compression_length;
    double m_P0;   // gas pressure at design position [pas]
    double m_hf0;  // value to ease the calculation [m]
    double m_F0;   // gas force at design position [N]

    ChFunction_Recorder m_bump;
    ChFunction_Recorder m_rebound;
};

// -----------------------------------------------------------------------------
// FEDA shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
FEDA_DoubleWishboneFront::FEDA_DoubleWishboneFront(const std::string& name) : ChDoubleWishbone(name) {
    m_springForceCB = new AirCoilSpringBistopForce(m_springCoefficient, m_springRestLength - m_bumpstop_clearance,
                                                   m_springRestLength + m_reboundstop_clearance, m_air_pressure);

    m_shockForceCB = new MapDamperForce();
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(-5, -11936.925);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(-3, -11368.5);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(-0.33, -10335);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(-0.22, -8376);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(-0.13, -5789);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(-0.05, -2672);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(-0.02, -654);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(0, 0);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(0.02, 652);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(0.05, 1649);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(0.13, 2975);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(0.22, 4718);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(0.33, 7496);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(3, 8245.6);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(5, 8657.88);
}

FEDA_DoubleWishboneRear::FEDA_DoubleWishboneRear(const std::string& name) : ChDoubleWishbone(name) {
    m_springForceCB = new AirCoilSpringBistopForce(m_springCoefficient, m_springRestLength - m_bumpstop_clearance,
                                                   m_springRestLength + m_reboundstop_clearance, m_air_pressure);

    m_shockForceCB = new MapDamperForce();
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(-5, -11936.925);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(-3, -11368.5);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(-0.33, -10335);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(-0.22, -8376);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(-0.13, -5789);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(-0.05, -2672);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(-0.02, -654);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(0, 0);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(0.02, 652);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(0.05, 1649);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(0.13, 2975);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(0.22, 4718);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(0.33, 7496);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(3, 8245.6);
    dynamic_cast<MapDamperForce*>(m_shockForceCB)->add_point(5, 8657.88);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
FEDA_DoubleWishboneFront::~FEDA_DoubleWishboneFront() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

FEDA_DoubleWishboneRear::~FEDA_DoubleWishboneRear() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> FEDA_DoubleWishboneFront::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector<>(0.0, 0.97663, 0);
        case UPRIGHT:
            return ChVector<>(0, 0.87, 0);
        case UCA_F:
            return ChVector<>(0.0478, 0.2324, 0.3469);
        case UCA_B:
            return ChVector<>(-0.3215, 0.2324, 0.3469);
        case UCA_U:
            return ChVector<>(-0.01759, 0.6744, 0.30589);
        case UCA_CM:
            return ChVector<>(-0.0971018, 0.379784667, 0.333246667);
        case LCA_F:
            return ChVector<>(0.16781, 0.2245, -0.08);
        case LCA_B:
            return ChVector<>(-0.45219, 0.22245, -0.119);
        case LCA_U:
            return ChVector<>(0.00789, 0.80719, -0.13904);
        case LCA_CM:
            return ChVector<>(-0.092163333, 0.418393333, -0.1128);
        case SHOCK_C:
            return ChVector<>(0.09397, 0.493925, 0.46209);
        case SHOCK_A:
            return ChVector<>(0.09397, 0.65153, -0.119);
        case SPRING_C:
            return ChVector<>(0.09397, 0.493925, 0.46209);
        case SPRING_A:
            return ChVector<>(0.09397, 0.65153, -0.119);
        case TIEROD_C:
            return ChVector<>(-0.24078, 0.379095, 0.04);
        case TIEROD_U:
            return ChVector<>(-0.207, 0.82618, 0);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> FEDA_DoubleWishboneRear::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector<>(0.0, 0.97663, 0);
        case UPRIGHT:
            return ChVector<>(0, 0.87, 0);
        case UCA_F:
            return ChVector<>(0.0478, 0.2324, 0.3469);
        case UCA_B:
            return ChVector<>(-0.3215, 0.2324, 0.3469);
        case UCA_U:
            return ChVector<>(-0.01759, 0.6744, 0.30589);
        case UCA_CM:
            return ChVector<>(-0.0971018, 0.379784667, 0.333246667);
        case LCA_F:
            return ChVector<>(0.16781, 0.2245, -0.08);
        case LCA_B:
            return ChVector<>(-0.45219, 0.22245, -0.119);
        case LCA_U:
            return ChVector<>(0.00789, 0.80719, -0.13904);
        case LCA_CM:
            return ChVector<>(-0.092163333, 0.418393333, -0.1128);
        case SHOCK_C:
            return ChVector<>(-0.09397, 0.493925, 0.46209);
        case SHOCK_A:
            return ChVector<>(-0.09397, 0.65153, -0.119);
        case SPRING_C:
            return ChVector<>(-0.09397, 0.493925, 0.46209);
        case SPRING_A:
            return ChVector<>(-0.09397, 0.65153, -0.119);
        case TIEROD_C:
            return ChVector<>(0.24078, 0.379095, 0.04);
        case TIEROD_U:
            return ChVector<>(0.207, 0.82618, 0);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
