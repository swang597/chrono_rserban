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
const double FEDA_DoubleWishboneFront::m_springRestLength =
    0.60208;  // distance between top and arm mount points of the strut in design position
const double FEDA_DoubleWishboneFront::m_springF0 = 0.125550934 * FEDA_DoubleWishboneFront::m_springCoefficient;
const double FEDA_DoubleWishboneFront::m_bumpstop_clearance = 0.11;
const double FEDA_DoubleWishboneFront::m_reboundstop_clearance = 0.11;
const double FEDA_DoubleWishboneFront::m_air_pressure[] = {18.0 * psi2pascal, 36.0 * psi2pascal, 50.0 * psi2pascal};

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
const double FEDA_DoubleWishboneRear::m_springRestLength =
    0.60208;  // distance between top and arm mount points of the strut in design position
const double FEDA_DoubleWishboneRear::m_springF0 = 0.125550934 * FEDA_DoubleWishboneRear::m_springCoefficient;
const double FEDA_DoubleWishboneRear::m_bumpstop_clearance = 0.11;
const double FEDA_DoubleWishboneRear::m_reboundstop_clearance = 0.11;
const double FEDA_DoubleWishboneRear::m_air_pressure[] = {18.0 * psi2pascal, 36.0 * psi2pascal, 50.0 * psi2pascal};

// --------------------------------------------------------------------------------------------------------------------------------
// FEDA spring functor class - implements a linear spring in combination with an airspring + bistops
// --------------------------------------------------------------------------------------------------------------------------------
class AirCoilSpringBistopForce : public ChLinkSpringCB::ForceFunctor {
  public:
    /// Use default bump stop and rebound stop maps
    AirCoilSpringBistopForce(double k, double min_length, double max_length, double coilSpringF0, double p0)
        : m_k(k),
          m_min_length(min_length),
          m_max_length(max_length),
          m_kappa(1.3),
          m_piston_radius(0.207 / 2.0),
          m_cylinder_compression_length(0.16),
          m_coilSpringF0(coilSpringF0),
          m_P0(p0) {
        // percalculations
        double A0 = pow(m_piston_radius, 2.0) * CH_C_PI;
        double V0 = m_cylinder_compression_length * A0;
        m_airSpringF0 = m_P0 * A0;
        GetLog() << "Fzero = " << m_airSpringF0 << "N\n";
        m_hf0 = m_P0 * V0 / m_airSpringF0;

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

        force = m_airSpringF0 * pow(m_hf0, m_kappa) / pow(m_hf0 - defl_spring, m_kappa) + defl_spring * m_k +
                m_coilSpringF0 + m_bump.Get_y(defl_bump) - m_rebound.Get_y(defl_rebound);
        // GetLog() << "d =" << defl_spring << " m\n";
        return force;
    }

  private:
    double m_k;
    double m_min_length;
    double m_max_length;
    double m_coilSpringF0;  // preload of the coil spring

    // airspring model (cylinder/piston)
    double m_kappa;  // polytropial exponent
    double m_piston_radius;
    double m_cylinder_compression_length;
    double m_P0;           // gas pressure at design position [pas]
    double m_hf0;          // value to ease the calculation [m]
    double m_airSpringF0;  // gas force at design position [N]

    ChFunction_Recorder m_bump;
    ChFunction_Recorder m_rebound;
};

// -----------------------------------------------------------------------------
// FEDA shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------

class MyShockForce : public ChLinkSpringCB::ForceFunctor {
  public:
    virtual double operator()(double time,          // current time
                              double rest_length,   // undeformed length
                              double length,        // current length
                              double vel,           // current velocity (positive when extending)
                              ChLinkSpringCB* link  // back-pointer to associated link
                              ) override {
        // Access states
        auto states = link->GetStates();

        // ....

        return 0;
    }
};

class MyShockODE : public ChLinkSpringCB::ODE {
  public:
    virtual int GetNumStates() const override { return 2; }
    virtual void SetInitialConditions(ChVectorDynamic<>& states,  ///< output vector containig initial conditions
                                      ChLinkSpringCB* link        ///< back-pointer to associated link
                                      ) override {
        states(0) = 1;
        states(1) = 0;
    }
    virtual void CalculateRHS(double time,
                              const ChVectorDynamic<>& states,  ///< current states
                              ChVectorDynamic<>& rhs,           ///< output vector containing the ODE right-hand side
                              ChLinkSpringCB* link              ///< back-pointer to associated link
                              ) override {
        rhs(0) = states(0);
        rhs(1) = std::cos(time);
    }
};

class FEDA_ShockForce : public ChLinkSpringCB::ForceFunctor {
  public:
    virtual double operator()(double time,          // current time
                              double rest_length,   // undeformed length
                              double length,        // current length
                              double vel,           // current velocity (positive when extending)
                              ChLinkSpringCB* link  // back-pointer to associated link
                              ) override {
        // Access states
        auto& states = link->GetStates();

        // ....


        return -states(1);  // flip the sign to make the force acting against velocity
    }
};

class FEDA_ShockODE : public ChLinkSpringCB::ODE {
  public:
    FEDA_ShockODE() {
        // Setup damper tables for high freqnency signals
        m_hf_damper_table.AddPoint(-5, -11936.925);
        m_hf_damper_table.AddPoint(-3, -11368.5);
        m_hf_damper_table.AddPoint(-0.33, -10335);
        m_hf_damper_table.AddPoint(-0.22, -8376);
        m_hf_damper_table.AddPoint(-0.13, -5789);
        m_hf_damper_table.AddPoint(-0.05, -2672);
        m_hf_damper_table.AddPoint(-0.02, -654);
        m_hf_damper_table.AddPoint(0, 0);
        m_hf_damper_table.AddPoint(0.02, 652);
        m_hf_damper_table.AddPoint(0.05, 1649);
        m_hf_damper_table.AddPoint(0.13, 2975);
        m_hf_damper_table.AddPoint(0.22, 4718);
        m_hf_damper_table.AddPoint(0.33, 7496);
        m_hf_damper_table.AddPoint(3, 8245.6);
        m_hf_damper_table.AddPoint(5, 8657.88);

        // Setup damper tables for low freqnency signals
        m_lf_damper_table.AddPoint(-5, -12183.06833);
        m_lf_damper_table.AddPoint(-3, -11602.92222);
        m_lf_damper_table.AddPoint(-0.33, -10548);
        m_lf_damper_table.AddPoint(-0.22, -8620);
        m_lf_damper_table.AddPoint(-0.13, -6669);
        m_lf_damper_table.AddPoint(-0.05, -2935);
        m_lf_damper_table.AddPoint(-0.02, -986);
        m_lf_damper_table.AddPoint(0, 0);
        m_lf_damper_table.AddPoint(0.02, 766);
        m_lf_damper_table.AddPoint(0.05, 3621);
        m_lf_damper_table.AddPoint(0.13, 12628);
        m_lf_damper_table.AddPoint(0.22, 14045);
        m_lf_damper_table.AddPoint(0.33, 15444);
        m_lf_damper_table.AddPoint(3, 16988.27778);
        m_lf_damper_table.AddPoint(5, 17837.69167);
    }

    virtual int GetNumStates() const override { return 2; }
    virtual void SetInitialConditions(ChVectorDynamic<>& states,  ///< output vector containig initial conditions
                                      ChLinkSpringCB* link        ///< back-pointer to associated link
                                      ) override {
        // we start with zero velocity and zero force
        states(0) = 0;
        states(1) = 0;
    }

    virtual void CalculateRHS(double time,
                              const ChVectorDynamic<>& states,  ///< current states
                              ChVectorDynamic<>& rhs,           ///< output vector containing the ODE right-hand side
                              ChLinkSpringCB* link              ///< back-pointer to associated link
                              ) override {

        // ODE1
        // y_dot0 = (u0 - y0)/T0;
        // u0 is damper velocity vel = input
        // y0 = delayed damper velocity v_del = stage(0) = output
        const double T0 = 0.04;
        const double T1 = 1.02e-3;
        double vel = link->GetDist_dt();
        rhs(0) = (vel - states(0)) / T0;
        double vel_delayed = states(0);
        double vel_min = std::min(vel, vel_delayed);

        double force_hf = m_hf_damper_table.Get_y(vel);
        double force_lf = m_lf_damper_table.Get_y(vel_min);
        double force1 = 0.0;
        if (vel > 0.0) {
            force1 = force_hf + force_lf;
        } else {
            force1 = force_hf;
        }

        // ODE2
        // y_dot1 = (u1 - y1)/T1;
        // u1 = damper force1 = input
        // y1 = delayed damper force = stage(1) = output
        rhs(1) = (force1 - states(1)) / T1;
    }

  private:
    ChFunction_Recorder m_hf_damper_table;
    ChFunction_Recorder m_lf_damper_table;
};

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
FEDA_DoubleWishboneFront::FEDA_DoubleWishboneFront(const std::string& name, int ride_height_mode)
    : ChDoubleWishbone(name) {
    m_ride_height_mode = ChClamp(ride_height_mode, 0, 2);
    GetLog() << "Ride Height Front = " << m_ride_height_mode << " (Pressure = " << m_air_pressure[m_ride_height_mode]
             << " Pas)\n";
    m_springForceCB = new AirCoilSpringBistopForce(m_springCoefficient, m_springRestLength - m_bumpstop_clearance,
                                                   m_springRestLength + m_reboundstop_clearance, m_springF0,
                                                   m_air_pressure[m_ride_height_mode]);

    m_shockForceCB = new FEDA_ShockForce();
}

FEDA_DoubleWishboneRear::FEDA_DoubleWishboneRear(const std::string& name, int ride_height_mode)
    : ChDoubleWishbone(name) {
    m_ride_height_mode = ChClamp(ride_height_mode, 0, 2);
    GetLog() << "Ride Height Rear = " << m_ride_height_mode << " (Pressure = " << m_air_pressure[m_ride_height_mode]
             << " Pas)\n";
    m_springForceCB = new AirCoilSpringBistopForce(m_springCoefficient, m_springRestLength - m_bumpstop_clearance,
                                                   m_springRestLength + m_reboundstop_clearance, m_springF0,
                                                   m_air_pressure[m_ride_height_mode]);
    m_shockForceCB = new FEDA_ShockForce();
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

void FEDA_DoubleWishboneFront::Initialize(
    std::shared_ptr<ChBodyAuxRef> chassis,  // [in] handle to the chassis body
    const ChVector<>& location,             // [in] location relative to the chassis frame
    std::shared_ptr<ChBody> tierod_body,    // [in] body to which tireods are connected
    int steering_index,                     // [in] index of the associated steering mechanism
    double left_ang_vel,                    // [in] initial angular velocity of left wheel
    double right_ang_vel                    // [in] initial angular velocity of right wheel
) {
    ChDoubleWishbone::Initialize(chassis, location, tierod_body, steering_index, left_ang_vel, right_ang_vel);

    m_shockODE = new FEDA_ShockODE;
    GetShock(LEFT)->RegisterODE(m_shockODE);
    GetShock(RIGHT)->RegisterODE(m_shockODE);
}

void FEDA_DoubleWishboneRear::Initialize(
    std::shared_ptr<ChBodyAuxRef> chassis,  // [in] handle to the chassis body
    const ChVector<>& location,             // [in] location relative to the chassis frame
    std::shared_ptr<ChBody> tierod_body,    // [in] body to which tireods are connected
    int steering_index,                     // [in] index of the associated steering mechanism
    double left_ang_vel,                    // [in] initial angular velocity of left wheel
    double right_ang_vel                    // [in] initial angular velocity of right wheel
) {
    ChDoubleWishbone::Initialize(chassis, location, tierod_body, steering_index, left_ang_vel, right_ang_vel);

    m_shockODE = new FEDA_ShockODE;
    GetShock(LEFT)->RegisterODE(m_shockODE);
    GetShock(RIGHT)->RegisterODE(m_shockODE);
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
