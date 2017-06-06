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

const double WVP_DoubleWishboneFront::m_spindleRadius = 0.10;
const double WVP_DoubleWishboneFront::m_spindleWidth = 0.06;
const double WVP_DoubleWishboneFront::m_LCARadius = 0.03;
const double WVP_DoubleWishboneFront::m_UCARadius = 0.02;
const double WVP_DoubleWishboneFront::m_uprightRadius = 0.04;

const ChVector<> WVP_DoubleWishboneFront::m_spindleInertia(0.04117, 0.07352, 0.04117);

const ChVector<> WVP_DoubleWishboneFront::m_UCAInertiaMoments(0.03, 0.03, 0.06276);
const ChVector<> WVP_DoubleWishboneFront::m_UCAInertiaProducts(0.0, 0.0, 0.0);

const ChVector<> WVP_DoubleWishboneFront::m_LCAInertiaMoments(0.4, 0.4, 0.8938);
const ChVector<> WVP_DoubleWishboneFront::m_LCAInertiaProducts(0.0, 0.0, 0.0);

const ChVector<> WVP_DoubleWishboneFront::m_uprightInertiaMoments(0.1656, 0.1934, 0.04367);
const ChVector<> WVP_DoubleWishboneFront::m_uprightInertiaProducts(0.0, 0.0, 0.0);

const double WVP_DoubleWishboneFront::m_axleInertia = 9.6e-3;

const double WVP_DoubleWishboneFront::m_springRestLength = .779;

// -----------------------------------------------------------------------------

const double WVP_DoubleWishboneRear::m_UCAMass = 14;
const double WVP_DoubleWishboneRear::m_LCAMass = 42;
const double WVP_DoubleWishboneRear::m_uprightMass = 150;
const double WVP_DoubleWishboneRear::m_spindleMass = 52;

const double WVP_DoubleWishboneRear::m_spindleRadius = 0.10;
const double WVP_DoubleWishboneRear::m_spindleWidth = 0.06;
const double WVP_DoubleWishboneRear::m_LCARadius = 0.03;
const double WVP_DoubleWishboneRear::m_UCARadius = 0.02;
const double WVP_DoubleWishboneRear::m_uprightRadius = 0.04;

const ChVector<> WVP_DoubleWishboneRear::m_spindleInertia(0.04117, 0.07352, 0.04117);

const ChVector<> WVP_DoubleWishboneRear::m_UCAInertiaMoments(0.03, 0.03, 0.06276);
const ChVector<> WVP_DoubleWishboneRear::m_UCAInertiaProducts(0.0, 0.0, 0.0);

const ChVector<> WVP_DoubleWishboneRear::m_LCAInertiaMoments(0.4, 0.4, 0.8938);
const ChVector<> WVP_DoubleWishboneRear::m_LCAInertiaProducts(0.0, 0.0, 0.0);

const ChVector<> WVP_DoubleWishboneRear::m_uprightInertiaMoments(0.1656, 0.1934, 0.04367);
const ChVector<> WVP_DoubleWishboneRear::m_uprightInertiaProducts(0.0, 0.0, 0.0);

const double WVP_DoubleWishboneRear::m_axleInertia = 9.6e-3;

const double WVP_DoubleWishboneRear::m_springRestLength = .831;

// -----------------------------------------------------------------------------
// WVP suspension spring functor class
// -----------------------------------------------------------------------------
class WVP_SpringForce : public ChLinkSpringCB::ForceFunctor {
  public:
    WVP_SpringForce(double ride_height_length);

  private:
    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override;

    double m_ride_height_length;
    ChFunction_Recorder m_map;
};

WVP_SpringForce::WVP_SpringForce(double ride_height_length) : m_ride_height_length(ride_height_length) {
    m_map.AddPoint(-127.8e-3, 20.72e3);
    m_map.AddPoint(-120e-3, 21.16e3);
    m_map.AddPoint(-110e-3, 21.73e3);
    m_map.AddPoint(-100e-3, 22.3e3);
    m_map.AddPoint(-90e-3, 22.87e3);
    m_map.AddPoint(-80e-3, 23.44e3);
    m_map.AddPoint(-70e-3, 24.01e3);
    m_map.AddPoint(-60e-3, 24.58e3);
    m_map.AddPoint(-50e-3, 25.15e3);
    m_map.AddPoint(-40e-3, 25.72e3);
    m_map.AddPoint(-30e-3, 26.29e3);
    m_map.AddPoint(-20e-3, 26.86e3);
    m_map.AddPoint(-10e-3, 27.43e3);
    m_map.AddPoint(0, 28e3);
    m_map.AddPoint(10e-3, 28.57e3);
    m_map.AddPoint(20e-3, 29.14e3);
    m_map.AddPoint(30e-3, 29.71e3);
    m_map.AddPoint(40e-3, 30.28e3);
    m_map.AddPoint(50e-3, 30.85e3);
    m_map.AddPoint(60e-3, 31.42e3);
    m_map.AddPoint(70e-3, 31.99e3);
    m_map.AddPoint(80e-3, 32.56e3);
    m_map.AddPoint(90e-3, 33.13e3);
    m_map.AddPoint(100e-3, 33.70e3);
    m_map.AddPoint(110e-3, 34.27e3);
    m_map.AddPoint(120e-3, 34.84e3);
    m_map.AddPoint(130e-3, 35.41e3);
    m_map.AddPoint(140e-3, 35.98e3);
    m_map.AddPoint(150e-3, 36.55e3);
    m_map.AddPoint(160e-3, 37.12e3);
    m_map.AddPoint(170e-3, 37.69e3);
    m_map.AddPoint(180e-3, 38.26e3);
    m_map.AddPoint(190e-3, 38.83e3);
    m_map.AddPoint(200e-3, 39.40e3);
    m_map.AddPoint(210e-3, 39.97e3);
    m_map.AddPoint(220e-3, 40.54e3);
    m_map.AddPoint(227.2e-3, 40.95e3);

    //TODO add points for bumpstops
    //

}

double WVP_SpringForce::operator()(double time, double rest_length, double length, double vel, ChLinkSpringCB* link) {
    return m_map.Get_y(length - rest_length);
}

// -----------------------------------------------------------------------------
// WVP shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class WVP_ShockForce : public ChLinkSpringCB::ForceFunctor {
  public:
    WVP_ShockForce(const std::string axle_name) : m_axle_name(axle_name) {}

    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override;

  private:
    std::string m_axle_name;

    std::shared_ptr<ChLinkSpringCB> m_shock_left;
    std::shared_ptr<ChLinkSpringCB> m_shock_right;

    friend class WVP_DoubleWishboneFront;
    friend class WVP_DoubleWishboneRear;
};

double WVP_ShockForce::operator()(double time, double rest_length, double length, double vel, ChLinkSpringCB* link) {
    double force = 0;

    VehicleSide side;
    double displ_mine, displ_other;
    double vel_mine, vel_other;

    if (link == m_shock_left.get()) {
        side = VehicleSide::LEFT;
        displ_mine = m_shock_left->GetSpringDeform();
        vel_mine = m_shock_left->GetSpringVelocity();
        displ_other = m_shock_right->GetSpringDeform();
        vel_other = m_shock_right->GetSpringVelocity();
    } else {
        side = VehicleSide::RIGHT;
        displ_mine = m_shock_right->GetSpringDeform();
        vel_mine = m_shock_right->GetSpringVelocity();
        displ_other = m_shock_left->GetSpringDeform();
        vel_other = m_shock_left->GetSpringVelocity();
    }

    bool parallel_travel = (displ_mine * displ_other > 0);

    //std::cout << m_axle_name << " " << side << " | " << displ_mine << "  " << displ_other << std::endl;

    ////  TODO
    if(parallel_travel){ //lookup table for parallel travel

    }else{ //lookup table for single wheel travel

    }

    return force;
}

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
WVP_DoubleWishboneFront::WVP_DoubleWishboneFront(const std::string& name) : ChDoubleWishbone(name, true) {
    m_springForceCB = new WVP_SpringForce(779e-3);
    m_shockForceCB = new WVP_ShockForce("front axle");
}

WVP_DoubleWishboneRear::WVP_DoubleWishboneRear(const std::string& name) : ChDoubleWishbone(name, true) {
    m_springForceCB = new WVP_SpringForce(831e-3);
    m_shockForceCB = new WVP_ShockForce("rear axle");
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
// Initialization of the suspension subsystems.
// -----------------------------------------------------------------------------
void WVP_DoubleWishboneFront::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                         const ChVector<>& location,
                                         std::shared_ptr<ChBody> tierod_body,
                                         int steering_index,
                                         double left_ang_vel,
                                         double right_ang_vel) {
    // Invoke base class method.
    ChDoubleWishbone::Initialize(chassis, location, tierod_body, steering_index, left_ang_vel, right_ang_vel);

    // Set the left and right shock elements in the shock force functor class.
    // Note that this can only be done here, after these elements were created.
    static_cast<WVP_ShockForce*>(m_shockForceCB)->m_shock_left = GetShock(VehicleSide::LEFT);
    static_cast<WVP_ShockForce*>(m_shockForceCB)->m_shock_right = GetShock(VehicleSide::RIGHT);
}

void WVP_DoubleWishboneRear::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                        const ChVector<>& location,
                                        std::shared_ptr<ChBody> tierod_body,
                                        int steering_index,
                                        double left_ang_vel,
                                        double right_ang_vel) {
    // Invoke base class method.
    ChDoubleWishbone::Initialize(chassis, location, tierod_body, steering_index, left_ang_vel, right_ang_vel);

    // Set the left and right shock elements in the shock force functor class.
    // Note that this can only be done here, after these elements were created.
    static_cast<WVP_ShockForce*>(m_shockForceCB)->m_shock_left = GetShock(VehicleSide::LEFT);
    static_cast<WVP_ShockForce*>(m_shockForceCB)->m_shock_right = GetShock(VehicleSide::RIGHT);
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
          return ChVector<>(-.107,.540,.724);
      case SHOCK_A:
          return ChVector<>(-.107,.547,-.111);
      case SPRING_C:
          return ChVector<>(-.107,.540,.724);
      case SPRING_A:
          return ChVector<>(-.107,.547,-.111);
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
          return ChVector<>(-.107,.540,.724);
      case SHOCK_A:
          return ChVector<>(-.107,.547,-.111);
      case SPRING_C:
          return ChVector<>(-.107,.540,.724);
      case SPRING_A:
          return ChVector<>(-.107,.547,-.111);
      case TIEROD_C:
          return ChVector<>(-.383,.125,.029);
      case TIEROD_U:
          return ChVector<>(-.398,.750,-.128);
      default:
          return ChVector<>(0, 0, 0);
    }
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
