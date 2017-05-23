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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Front and Rear WVP suspension subsystems (double A-arm).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the supspension.
//
// =============================================================================

#include "chrono_models/vehicle/wvp/WVP_DoubleWishbone.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;
static const double lbfpin2Npm = 175.12677;

const double WVP_DoubleWishboneFront::m_UCAMass = 14;
const double WVP_DoubleWishboneFront::m_LCAMass = 42;
const double WVP_DoubleWishboneFront::m_uprightMass = 150;
const double WVP_DoubleWishboneFront::m_spindleMass = 52;

const double WVP_DoubleWishboneFront::m_spindleRadius = 0.10; //TODO: not given
const double WVP_DoubleWishboneFront::m_spindleWidth = 0.06; //TODO: not given
const double WVP_DoubleWishboneFront::m_LCARadius = 0.03; //TODO: not given
const double WVP_DoubleWishboneFront::m_UCARadius = 0.02; //TODO: not given
const double WVP_DoubleWishboneFront::m_uprightRadius = 0.04; //TODO: not given

const ChVector<> WVP_DoubleWishboneFront::m_spindleInertia(0.04117, 0.07352, 0.04117);

const ChVector<> WVP_DoubleWishboneFront::m_UCAInertiaMoments(0.03, 0.03, 0.06276);
const ChVector<> WVP_DoubleWishboneFront::m_UCAInertiaProducts(0.0, 0.0, 0.0);

const ChVector<> WVP_DoubleWishboneFront::m_LCAInertiaMoments(0.4, 0.4, 0.8938);
const ChVector<> WVP_DoubleWishboneFront::m_LCAInertiaProducts(0.0, 0.0, 0.0);

const ChVector<> WVP_DoubleWishboneFront::m_uprightInertiaMoments(0.1656, 0.1934, 0.04367);
const ChVector<> WVP_DoubleWishboneFront::m_uprightInertiaProducts(0.0, 0.0, 0.0);

const double WVP_DoubleWishboneFront::m_axleInertia = 0.4;//TODO: not correct frame

const double WVP_DoubleWishboneFront::m_springCoefficient = 167062.000;//TODO: change to lookup table
const double WVP_DoubleWishboneFront::m_springRestLength = 0.339;//TODO: change to lookup table

// -----------------------------------------------------------------------------

const double WVP_DoubleWishboneRear::m_UCAMass = 14;
const double WVP_DoubleWishboneRear::m_LCAMass = 42;
const double WVP_DoubleWishboneRear::m_uprightMass = 150;
const double WVP_DoubleWishboneRear::m_spindleMass = 52;

const double WVP_DoubleWishboneRear::m_spindleRadius = 0.10; //TODO: not given
const double WVP_DoubleWishboneRear::m_spindleWidth = 0.06;//TODO: not given
const double WVP_DoubleWishboneRear::m_LCARadius = 0.03;//TODO: not given
const double WVP_DoubleWishboneRear::m_UCARadius = 0.02;//TODO: not given
const double WVP_DoubleWishboneRear::m_uprightRadius = 0.04;//TODO: not given

const ChVector<> WVP_DoubleWishboneRear::m_spindleInertia(0.04117, 0.07352, 0.04117);

const ChVector<> WVP_DoubleWishboneRear::m_UCAInertiaMoments(0.03, 0.03, 0.06276);
const ChVector<> WVP_DoubleWishboneRear::m_UCAInertiaProducts(0.0, 0.0, 0.0);

const ChVector<> WVP_DoubleWishboneRear::m_LCAInertiaMoments(0.4, 0.4, 0.8938);
const ChVector<> WVP_DoubleWishboneRear::m_LCAInertiaProducts(0.0, 0.0, 0.0);

const ChVector<> WVP_DoubleWishboneRear::m_uprightInertiaMoments(0.1656, 0.1934, 0.04367);
const ChVector<> WVP_DoubleWishboneRear::m_uprightInertiaProducts(0.0, 0.0, 0.0);

const double WVP_DoubleWishboneRear::m_axleInertia = 0.4;//TODO: not correct frame

const double WVP_DoubleWishboneRear::m_springCoefficient = 1.000;//TODO: change to lookup table
const double WVP_DoubleWishboneRear::m_springRestLength = .822;//TODO: change to lookup table

// -----------------------------------------------------------------------------
// WVP shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class WVP_ShockForce : public ChLinkSpringCB::ForceFunctor {
  public:
    WVP_ShockForce(double midstroke_compression_slope,
                     double midstroke_rebound_slope,
                     double bumpstop_compression_slope,
                     double bumpstop_rebound_slope,
                     double metalmetal_slope,
                     double min_bumpstop_compression_force,
                     double midstroke_lower_bound,
                     double midstroke_upper_bound,
                     double metalmetal_lower_bound,
                     double metalmetal_upper_bound);

    virtual double operator()(double time, double rest_length, double length, double vel);

  private:
    double m_ms_compr;
    double m_ms_rebound;
    double m_bs_compr;
    double m_bs_rebound;
    double m_metal_K;
    double m_F0;
    double m_ms_min_length;
    double m_ms_max_length;
    double m_min_length;
    double m_max_length;
};

WVP_ShockForce::WVP_ShockForce(double midstroke_compression_slope,
                                   double midstroke_rebound_slope,
                                   double bumpstop_compression_slope,
                                   double bumpstop_rebound_slope,
                                   double metalmetal_slope,
                                   double min_bumpstop_compression_force,
                                   double midstroke_lower_bound,
                                   double midstroke_upper_bound,
                                   double metalmetal_lower_bound,
                                   double metalmetal_upper_bound)
    : m_ms_compr(midstroke_compression_slope),
      m_ms_rebound(midstroke_rebound_slope),
      m_bs_compr(bumpstop_compression_slope),
      m_bs_rebound(bumpstop_rebound_slope),
      m_metal_K(metalmetal_slope),
      m_F0(min_bumpstop_compression_force),
      m_ms_min_length(midstroke_lower_bound),
      m_ms_max_length(midstroke_upper_bound),
      m_min_length(metalmetal_lower_bound),
      m_max_length(metalmetal_upper_bound) {}

double WVP_ShockForce::operator()(double time, double rest_length, double length, double vel) {
    /*
    // On midstroke curve
    if (length >= m_min_length && length <= m_max_length)
      return (vel >= 0) ? -m_ms_rebound * vel : -m_ms_compr * vel;

    // Hydraulic bump engaged
    return (vel >= 0) ? -m_bs_rebound * vel : -m_bs_compr * vel + m_F0;
    */

    double force = 0;

    // Calculate Damping Force
    if (vel >= 0) {
        force = (length >= m_ms_max_length) ? -m_bs_rebound * vel : -m_ms_rebound * vel;
    } else {
        force = (length <= m_ms_min_length) ? -m_bs_compr * vel : -m_ms_compr * vel;
    }

    // Add in Shock metal to metal contact force
    if (length <= m_min_length) {
        force = m_metal_K * (m_min_length - length);
    } else if (length >= m_max_length) {
        force = -m_metal_K * (length - m_max_length);
    }

    return force;
}

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
WVP_DoubleWishboneFront::WVP_DoubleWishboneFront(const std::string& name) : ChDoubleWishbone(name, true) {
    m_springForceCB = new LinearSpringForce(m_springCoefficient  // coefficient for linear spring
                                            );

    m_shockForceCB = new WVP_ShockForce(lbfpin2Npm * 71.50,   // midstroke_compression_slope
                                          lbfpin2Npm * 128.25,  // midstroke_rebound_slope
                                          lbfpin2Npm * 33.67,   // bumpstop_compression_slope
                                          lbfpin2Npm * 343.00,  // bumpstop_rebound_slope
                                          lbfpin2Npm * 150000,  // metalmetal_slope
                                          lbf2N * 3350,         // min_bumpstop_compression_force
                                          in2m * 13.76,         // midstroke_lower_bound
                                          in2m * 15.85,         // midstroke_upper_bound
                                          in2m * 12.76,         // metalmetal_lower_bound
                                          in2m * 16.48          // metalmetal_upper_boun
                                          );
}

WVP_DoubleWishboneRear::WVP_DoubleWishboneRear(const std::string& name) : ChDoubleWishbone(name, true) {
    m_springForceCB = new LinearSpringForce(m_springCoefficient  // coefficient for linear spring
                                            );

    m_shockForceCB = new WVP_ShockForce(lbfpin2Npm * 83.00,   // midstroke_compression_slope
                                          lbfpin2Npm * 200.00,  // midstroke_rebound_slope
                                          lbfpin2Npm * 48.75,   // bumpstop_compression_slope
                                          lbfpin2Npm * 365.00,  // bumpstop_rebound_slope
                                          lbfpin2Npm * 150000,  // metalmetal_slope
                                          lbf2N * 3350,         // min_bumpstop_compression_force
                                          in2m * 13.76,         // midstroke_lower_bound
                                          in2m * 15.85,         // midstroke_upper_bound
                                          in2m * 12.76,         // metalmetal_lower_bound
                                          in2m * 16.48          // metalmetal_upper_bound
                                          );
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
WVP_DoubleWishboneFront::~WVP_DoubleWishboneFront() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

WVP_DoubleWishboneRear::~WVP_DoubleWishboneRear() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> WVP_DoubleWishboneFront::getLocation(PointId which) {
    switch (which) {
      case SPINDLE:
          return ChVector<>(0,1.033,0);
      case UPRIGHT:
          return ChVector<>(0, 1.033, 0);
      case UCA_F:
          return ChVector<>(.309,.192,.365);
      case UCA_B:
          return ChVector<>(-.149,.192,.365);
      case UCA_U:
          return ChVector<>(-.008,.772,.234);
      case UCA_CM:
          return ChVector<>(.041,.428,.331);
      case LCA_F:
          return ChVector<>(.360,.192,.012);
      case LCA_B:
          return ChVector<>(-.250,.192,.012);
      case LCA_U:
          return ChVector<>(.007,.832,-.152);
      case LCA_CM:
          return ChVector<>(.039,.406,-.043);
      case SHOCK_C:
          return ChVector<>(-.107,.547,-.111);
      case SHOCK_A:
          return ChVector<>(-.107,.540,.724);
      case SPRING_C:
          return ChVector<>(-.107,.547,-.111);
      case SPRING_A:
          return ChVector<>(-.107,.540,.724);
      case TIEROD_C:
          return ChVector<>(-.383,.125,.029);
      case TIEROD_U:
          return ChVector<>(-.398,.750,-.128);
      default:
          return ChVector<>(0, 0, 0);
    }
}

const ChVector<> WVP_DoubleWishboneRear::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector<>(-4.039,1.033,0);
        case UPRIGHT:
            return ChVector<>(-4.039, 1.033,0);//TODO: not given
        case UCA_F:
            return ChVector<>(-3.729,.192,.365);
        case UCA_B:
            return ChVector<>(-4.187,.192,.365);
        case UCA_U:
            return ChVector<>(-4.047,.772,.234);
        case UCA_CM:
            return ChVector<>(-3.998,.428,.331);
        case LCA_F:
            return ChVector<>(-3.679,.192,.012);
        case LCA_B:
            return ChVector<>(-4.288,.192,.012);
        case LCA_U:
            return ChVector<>(-4.032,.832,-.152);
        case LCA_CM:
            return ChVector<>(-4.000,.406,-.043);
        case SHOCK_C:
            return ChVector<>(-4.146,.547,-.111);
        case SHOCK_A:
            return ChVector<>(-4.146,.540,.724);
        case SPRING_C:
            return ChVector<>(-4.146,.547,-.111);
        case SPRING_A:
            return ChVector<>(-4.146,.540,.724);
        case TIEROD_C:
            return ChVector<>(-4.422,.125,.029);
        case TIEROD_U:
            return ChVector<>(-4.437,.750,-.128);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
