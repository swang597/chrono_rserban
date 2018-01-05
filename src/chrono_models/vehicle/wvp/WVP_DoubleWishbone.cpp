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

#include <cmath>

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
const double WVP_DoubleWishboneFront::m_LCAMass = 42 + 40;
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

const double WVP_DoubleWishboneFront::m_springRestLength = .779;  //+.088;
/*const double WVP_DoubleWishboneFront::m_springCoefficient = 769149.000;*/

const double WVP_DoubleWishboneFront::m_springPreload = 5900;//5200.0;

// -----------------------------------------------------------------------------

//*************************************************
// const double frontOffset = .03;
// const double rearOffset = .03;
//*************************************************

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

const double WVP_DoubleWishboneRear::m_springRestLength = .831;  //+.051;
/*const double WVP_DoubleWishboneRear::m_springCoefficient = 769149.000;*/

const double WVP_DoubleWishboneRear::m_springPreload = 2300;// 2500.0;

// -----------------------------------------------------------------------------
// Implementatin of the WVP suspension spring functor class
// -----------------------------------------------------------------------------

WVP_SpringForce::WVP_SpringForce(int axle, double rest_length, double preload)
    : m_axle_index(axle), m_rest_length(rest_length), m_preload(preload) {
    //****Curve from data with bump stops added. Data simplified for linear sections given that this is a linear
    // interpolation rebound stop region
    m_map.AddPoint(-154.0e-3, -40.78e3);
    m_map.AddPoint(-124.0e-3, 20.93e3);
    // pure spring region
    m_map.AddPoint(0, 28.0e3);
    m_map.AddPoint(160.0e-3, 37.12e3);
    // jounce stop region
    m_map.AddPoint(169.0e-3, 37.63e3);
    m_map.AddPoint(176.5e-3, 38.19e3);
    m_map.AddPoint(181.5e-3, 38.66e3);
    m_map.AddPoint(187.0e-3, 39.96e3);
    m_map.AddPoint(189.5e-3, 40.60e3);
    m_map.AddPoint(191.5e-3, 41.62e3);
    m_map.AddPoint(191.8e-3, 41.91e3);
    m_map.AddPoint(192.2e-3, 42.73e3);
    m_map.AddPoint(193.0e-3, 47.28e3);
    m_map.AddPoint(193.5e-3, 50.10e3);
    m_map.AddPoint(194.0e-3, 53.01e3);
    m_map.AddPoint(194.5e-3, 56.19e3);
    m_map.AddPoint(195.0e-3, 60.26e3);
    m_map.AddPoint(195.5e-3, 65.24e3);
    m_map.AddPoint(196.0e-3, 70.22e3);
}

double WVP_SpringForce::operator()(double time, double rest_length, double length, double vel, ChLinkSpringCB* link) {
    // grab the value from the spring map
    return m_map.Get_y(m_rest_length - length) + m_preload;
}

double WVP_SpringForce::evaluate(double displ) {
    return m_map.Get_y(-displ) + m_preload;
}

// -----------------------------------------------------------------------------
// Implementation of the WVP shock functor class
// -----------------------------------------------------------------------------

WVP_ShockForce::WVP_ShockForce(int axle_index, double rest_length)
    : m_axle_index(axle_index), m_rest_length(rest_length) {
    // add single damping table

    ChFunction_Recorder m_single_damp_map1;
    m_single_damp_map1.AddPoint(-127.8e-3, -1.27e5);
    m_single_damp_map1.AddPoint(-90e-3, -1.27e5);
    m_single_damp_map1.AddPoint(-80e-3, -1.26e5);
    m_single_damp_map1.AddPoint(-50e-3, -1.26e5);
    m_single_damp_map1.AddPoint(-40e-3, -1.24e5);
    m_single_damp_map1.AddPoint(-30e-3, -1.13e5);
    m_single_damp_map1.AddPoint(-20e-3, -8.99e4);
    m_single_damp_map1.AddPoint(120e-3, -8.99e4);
    m_single_damp_map1.AddPoint(130e-3, -8.99e4);
    m_single_damp_map1.AddPoint(140e-3, -8.99e4);
    m_single_damp_map1.AddPoint(150e-3, -8.99e4);
    m_single_damp_map1.AddPoint(160e-3, -8.99e4);
    m_single_damp_map1.AddPoint(170e-3, -8.99e4);
    m_single_damp_map1.AddPoint(227.2e-3, -8.99e4);

    ChFunction_Recorder m_single_damp_map75;
    m_single_damp_map75.AddPoint(-127.8e-3, -8.71e4);
    m_single_damp_map75.AddPoint(-90e-3, -8.71e4);
    m_single_damp_map75.AddPoint(-80e-3, -8.60e4);
    m_single_damp_map75.AddPoint(-50e-3, -8.60e4);
    m_single_damp_map75.AddPoint(-40e-3, -8.30e4);
    m_single_damp_map75.AddPoint(-30e-3, -7.2e4);
    m_single_damp_map75.AddPoint(-20e-3, -4.91e4);
    m_single_damp_map75.AddPoint(120e-3, -4.91e4);
    m_single_damp_map75.AddPoint(130e-3, -4.91e4);
    m_single_damp_map75.AddPoint(140e-3, -4.91e4);
    m_single_damp_map75.AddPoint(150e-3, -4.91e4);
    m_single_damp_map75.AddPoint(160e-3, -4.91e4);
    m_single_damp_map75.AddPoint(170e-3, -4.91e4);
    m_single_damp_map75.AddPoint(227.2e-3, -4.91e4);

    ChFunction_Recorder m_single_damp_map5;
    m_single_damp_map5.AddPoint(-127.8e-3, -6.00e4);
    m_single_damp_map5.AddPoint(-90e-3, -6.00e4);
    m_single_damp_map5.AddPoint(-80e-3, -5.89e4);
    m_single_damp_map5.AddPoint(-50e-3, -5.89e4);
    m_single_damp_map5.AddPoint(-40e-3, -5.62e4);
    m_single_damp_map5.AddPoint(-30e-3, -4.47e4);
    m_single_damp_map5.AddPoint(-20e-3, -2.18e4);
    m_single_damp_map5.AddPoint(120e-3, -2.18e4);
    m_single_damp_map5.AddPoint(130e-3, -2.18e4);
    m_single_damp_map5.AddPoint(140e-3, -2.18e4);
    m_single_damp_map5.AddPoint(150e-3, -2.18e4);
    m_single_damp_map5.AddPoint(160e-3, -2.18e4);
    m_single_damp_map5.AddPoint(170e-3, -2.18e4);
    m_single_damp_map5.AddPoint(227.2e-3, -2.18e4);

    ChFunction_Recorder m_single_damp_map25;
    m_single_damp_map25.AddPoint(-127.8e-3, -4.41e4);
    m_single_damp_map25.AddPoint(-90e-3, -4.41e4);
    m_single_damp_map25.AddPoint(-80e-3, -4.10e4);
    m_single_damp_map25.AddPoint(-50e-3, -4.10e4);
    m_single_damp_map25.AddPoint(-40e-3, -3.83e4);
    m_single_damp_map25.AddPoint(-30e-3, -2.71e4);
    m_single_damp_map25.AddPoint(-20e-3, -3.90e3);
    m_single_damp_map25.AddPoint(120e-3, -3.90e3);
    m_single_damp_map25.AddPoint(130e-3, -3.90e3);
    m_single_damp_map25.AddPoint(140e-3, -3.90e3);
    m_single_damp_map25.AddPoint(150e-3, -3.90e3);
    m_single_damp_map25.AddPoint(160e-3, -3.90e3);
    m_single_damp_map25.AddPoint(170e-3, -3.90e3);
    m_single_damp_map25.AddPoint(227.2e-3, -3.90e3);

    ChFunction_Recorder m_single_damp_map_0;
    m_single_damp_map_0.AddPoint(-127.8e-3, 0);
    m_single_damp_map_0.AddPoint(-90e-3, 0);
    m_single_damp_map_0.AddPoint(-80e-3, 0);
    m_single_damp_map_0.AddPoint(-50e-3, 0);
    m_single_damp_map_0.AddPoint(-40e-3, 0);
    m_single_damp_map_0.AddPoint(-30e-3, 0);
    m_single_damp_map_0.AddPoint(-20e-3, 0);
    m_single_damp_map_0.AddPoint(120e-3, 0);
    m_single_damp_map_0.AddPoint(130e-3, 0);
    m_single_damp_map_0.AddPoint(140e-3, 0);
    m_single_damp_map_0.AddPoint(150e-3, 0);
    m_single_damp_map_0.AddPoint(160e-3, 0);
    m_single_damp_map_0.AddPoint(170e-3, 0);
    m_single_damp_map_0.AddPoint(227.2e-3, 0);

    ChFunction_Recorder m_single_damp_map_5;
    m_single_damp_map_5.AddPoint(-127.8e-3, 1.79e4);
    m_single_damp_map_5.AddPoint(-90e-3, 1.79e4);
    m_single_damp_map_5.AddPoint(-80e-3, 1.79e4);
    m_single_damp_map_5.AddPoint(-50e-3, 1.79e4);
    m_single_damp_map_5.AddPoint(-40e-3, 1.79e4);
    m_single_damp_map_5.AddPoint(-30e-3, 1.79e4);
    m_single_damp_map_5.AddPoint(-20e-3, 1.79e4);
    m_single_damp_map_5.AddPoint(120e-3, 1.79e4);
    m_single_damp_map_5.AddPoint(130e-3, 3.57e4);
    m_single_damp_map_5.AddPoint(140e-3, 5.62e4);
    m_single_damp_map_5.AddPoint(150e-3, 6.51e4);
    m_single_damp_map_5.AddPoint(160e-3, 6.71e4);
    m_single_damp_map_5.AddPoint(170e-3, 7.16e4);
    m_single_damp_map_5.AddPoint(227.2e-3, 7.16e4);

    ChFunction_Recorder m_single_damp_map_1;
    m_single_damp_map_1.AddPoint(-127.8e-3, 7.62e4);
    m_single_damp_map_1.AddPoint(-90e-3, 7.62e4);
    m_single_damp_map_1.AddPoint(-80e-3, 7.62e4);
    m_single_damp_map_1.AddPoint(-50e-3, 7.62e4);
    m_single_damp_map_1.AddPoint(-40e-3, 7.62e4);
    m_single_damp_map_1.AddPoint(-30e-3, 7.62e4);
    m_single_damp_map_1.AddPoint(-20e-3, 7.62e4);
    m_single_damp_map_1.AddPoint(120e-3, 7.62e4);
    m_single_damp_map_1.AddPoint(130e-3, 1.30e5);
    m_single_damp_map_1.AddPoint(140e-3, 2.26e5);
    m_single_damp_map_1.AddPoint(150e-3, 2.66e5);
    m_single_damp_map_1.AddPoint(160e-3, 2.83e5);
    m_single_damp_map_1.AddPoint(170e-3, 2.92e5);
    m_single_damp_map_1.AddPoint(227.2e-3, 2.92e5);

    ChFunction_Recorder m_single_damp_map_15;
    m_single_damp_map_15.AddPoint(-127.8e-3, 1.65e5);
    m_single_damp_map_15.AddPoint(-90e-3, 1.65e5);
    m_single_damp_map_15.AddPoint(-80e-3, 1.65e5);
    m_single_damp_map_15.AddPoint(-50e-3, 1.65e5);
    m_single_damp_map_15.AddPoint(-40e-3, 1.65e5);
    m_single_damp_map_15.AddPoint(-30e-3, 1.65e5);
    m_single_damp_map_15.AddPoint(-20e-3, 1.65e5);
    m_single_damp_map_15.AddPoint(120e-3, 1.65e5);
    m_single_damp_map_15.AddPoint(130e-3, 2.79e5);
    m_single_damp_map_15.AddPoint(140e-3, 4.93e5);
    m_single_damp_map_15.AddPoint(150e-3, 5.87e5);
    m_single_damp_map_15.AddPoint(160e-3, 6.25e5);
    m_single_damp_map_15.AddPoint(170e-3, 6.44e5);
    m_single_damp_map_15.AddPoint(227.2e-3, 6.44e5);

    ChFunction_Recorder m_single_damp_map_2;
    m_single_damp_map_2.AddPoint(-127.8e-3, 2.86e5);
    m_single_damp_map_2.AddPoint(-90e-3, 2.86e5);
    m_single_damp_map_2.AddPoint(-80e-3, 2.86e5);
    m_single_damp_map_2.AddPoint(-50e-3, 2.86e5);
    m_single_damp_map_2.AddPoint(-40e-3, 2.86e5);
    m_single_damp_map_2.AddPoint(-30e-3, 2.86e5);
    m_single_damp_map_2.AddPoint(-20e-3, 2.86e5);
    m_single_damp_map_2.AddPoint(120e-3, 2.86e5);
    m_single_damp_map_2.AddPoint(130e-3, 4.66e5);
    m_single_damp_map_2.AddPoint(140e-3, 8.76e5);
    m_single_damp_map_2.AddPoint(150e-3, 1.02e6);
    m_single_damp_map_2.AddPoint(160e-3, 1.10e6);
    m_single_damp_map_2.AddPoint(170e-3, 1.15e6);
    m_single_damp_map_2.AddPoint(227.2e-3, 1.15e6);

    // add these ChFunction_Recorders to map
    m_single_damp_map.insert({2.0, m_single_damp_map_2});
    m_single_damp_map.insert({1.5, m_single_damp_map_15});
    m_single_damp_map.insert({1.0, m_single_damp_map_1});
    m_single_damp_map.insert({0.5, m_single_damp_map_5});
    m_single_damp_map.insert({0, m_single_damp_map_0});
    m_single_damp_map.insert({-0.25, m_single_damp_map25});
    m_single_damp_map.insert({-0.5, m_single_damp_map5});
    m_single_damp_map.insert({-0.75, m_single_damp_map75});
    m_single_damp_map.insert({-1.0, m_single_damp_map1});

    // add parallel daping table
    ChFunction_Recorder m_para_damp_map1;
    m_para_damp_map1.AddPoint(-127.8e-3, -8.99e4);
    m_para_damp_map1.AddPoint(110e-3, -8.99e4);
    m_para_damp_map1.AddPoint(120e-3, -8.99e4);
    m_para_damp_map1.AddPoint(130e-3, -8.99e4);
    m_para_damp_map1.AddPoint(140e-3, -8.99e4);
    m_para_damp_map1.AddPoint(150e-3, -8.99e4);
    m_para_damp_map1.AddPoint(160e-3, -8.99e4);
    m_para_damp_map1.AddPoint(170e-3, -8.99e4);
    m_para_damp_map1.AddPoint(227.2e-3, -8.99e4);

    ChFunction_Recorder m_para_damp_map75;
    m_para_damp_map75.AddPoint(-127.8e-3, -4.91e4);
    m_para_damp_map75.AddPoint(110e-3, -4.91e4);
    m_para_damp_map75.AddPoint(120e-3, -4.91e4);
    m_para_damp_map75.AddPoint(130e-3, -4.91e4);
    m_para_damp_map75.AddPoint(140e-3, -4.91e4);
    m_para_damp_map75.AddPoint(150e-3, -4.91e4);
    m_para_damp_map75.AddPoint(160e-3, -4.91e4);
    m_para_damp_map75.AddPoint(170e-3, -4.91e4);
    m_para_damp_map75.AddPoint(227.2e-3, -4.91e4);

    ChFunction_Recorder m_para_damp_map5;
    m_para_damp_map5.AddPoint(-127.8e-3, -2.18e4);
    m_para_damp_map5.AddPoint(110e-3, -2.18e4);
    m_para_damp_map5.AddPoint(120e-3, -2.18e4);
    m_para_damp_map5.AddPoint(130e-3, -2.18e4);
    m_para_damp_map5.AddPoint(140e-3, -2.18e4);
    m_para_damp_map5.AddPoint(150e-3, -2.18e4);
    m_para_damp_map5.AddPoint(160e-3, -2.18e4);
    m_para_damp_map5.AddPoint(170e-3, -2.18e4);
    m_para_damp_map5.AddPoint(227.2e-3, -2.18e4);

    ChFunction_Recorder m_para_damp_map25;
    m_para_damp_map25.AddPoint(-127.8e-3, -3.90e3);
    m_para_damp_map25.AddPoint(110e-3, -3.90e3);
    m_para_damp_map25.AddPoint(120e-3, -3.90e3);
    m_para_damp_map25.AddPoint(130e-3, -3.90e3);
    m_para_damp_map25.AddPoint(140e-3, -3.90e3);
    m_para_damp_map25.AddPoint(150e-3, -3.90e3);
    m_para_damp_map25.AddPoint(160e-3, -3.90e3);
    m_para_damp_map25.AddPoint(170e-3, -3.90e3);
    m_para_damp_map25.AddPoint(227.2e-3, -3.90e3);

    ChFunction_Recorder m_para_damp_map_0;
    m_para_damp_map_0.AddPoint(-127.8e-3, 0);
    m_para_damp_map_0.AddPoint(110e-3, 0);
    m_para_damp_map_0.AddPoint(120e-3, 0);
    m_para_damp_map_0.AddPoint(130e-3, 0);
    m_para_damp_map_0.AddPoint(140e-3, 0);
    m_para_damp_map_0.AddPoint(150e-3, 0);
    m_para_damp_map_0.AddPoint(160e-3, 0);
    m_para_damp_map_0.AddPoint(170e-3, 0);
    m_para_damp_map_0.AddPoint(227.2e-3, 0);

    ChFunction_Recorder m_para_damp_map_5;
    m_para_damp_map_5.AddPoint(-127.8e-3, 1.79e4);
    m_para_damp_map_5.AddPoint(110e-3, 1.79e4);
    m_para_damp_map_5.AddPoint(120e-3, 1.79e4);
    m_para_damp_map_5.AddPoint(130e-3, 3.57e4);
    m_para_damp_map_5.AddPoint(140e-3, 5.62e4);
    m_para_damp_map_5.AddPoint(150e-3, 6.51e4);
    m_para_damp_map_5.AddPoint(160e-3, 6.71e4);
    m_para_damp_map_5.AddPoint(170e-3, 7.16e4);
    m_para_damp_map_5.AddPoint(227.2e-3, 7.16e4);

    ChFunction_Recorder m_para_damp_map_1;
    m_para_damp_map_1.AddPoint(-127.8e-3, 7.26e4);
    m_para_damp_map_1.AddPoint(110e-3, 7.26e4);
    m_para_damp_map_1.AddPoint(120e-3, 7.26e4);
    m_para_damp_map_1.AddPoint(130e-3, 1.30e4);
    m_para_damp_map_1.AddPoint(140e-3, 2.26e5);
    m_para_damp_map_1.AddPoint(150e-3, 2.66e5);
    m_para_damp_map_1.AddPoint(160e-3, 2.83e5);
    m_para_damp_map_1.AddPoint(170e-3, 2.92e5);
    m_para_damp_map_1.AddPoint(227.2e-3, 2.92e5);

    ChFunction_Recorder m_para_damp_map_15;
    m_para_damp_map_15.AddPoint(-127.8e-3, 1.65e5);
    m_para_damp_map_15.AddPoint(110e-3, 1.65e5);
    m_para_damp_map_15.AddPoint(120e-3, 1.65e5);
    m_para_damp_map_15.AddPoint(130e-3, 2.79e5);
    m_para_damp_map_15.AddPoint(140e-3, 4.93e5);
    m_para_damp_map_15.AddPoint(150e-3, 5.87e5);
    m_para_damp_map_15.AddPoint(160e-3, 6.25e5);
    m_para_damp_map_15.AddPoint(170e-3, 6.44e5);
    m_para_damp_map_15.AddPoint(227.2e-3, 6.44e5);

    ChFunction_Recorder m_para_damp_map_2;
    m_para_damp_map_2.AddPoint(-127.8e-3, 2.86e5);
    m_para_damp_map_2.AddPoint(110e-3, 2.86e5);
    m_para_damp_map_2.AddPoint(120e-3, 2.86e5);
    m_para_damp_map_2.AddPoint(130e-3, 4.66e5);
    m_para_damp_map_2.AddPoint(140e-3, 8.76e5);
    m_para_damp_map_2.AddPoint(150e-3, 1.02e6);
    m_para_damp_map_2.AddPoint(160e-3, 1.10e6);
    m_para_damp_map_2.AddPoint(170e-3, 1.15e6);
    m_para_damp_map_2.AddPoint(227.2e-3, 1.15e6);

    // add these ChFunction_Recorders to map
    m_para_damp_map.insert({2.0, m_para_damp_map_2});
    m_para_damp_map.insert({1.5, m_para_damp_map_15});
    m_para_damp_map.insert({1.0, m_para_damp_map_1});
    m_para_damp_map.insert({0.5, m_para_damp_map_5});
    m_para_damp_map.insert({0, m_para_damp_map_0});
    m_para_damp_map.insert({-0.25, m_para_damp_map25});
    m_para_damp_map.insert({-0.5, m_para_damp_map5});
    m_para_damp_map.insert({-0.75, m_para_damp_map75});
    m_para_damp_map.insert({-1.0, m_para_damp_map1});

    // roll stabilization table
    /*m_roll_map.AddPoint(-127.8e-3 + 15e-3, -145.2e3 + 30e-3);
    m_roll_map.AddPoint(-120e-3 + 15e-3, -140.4e3 + 30e-3);
    m_roll_map.AddPoint(-110e-3 + 15e-3, -129.6e3 + 30e-3);
    m_roll_map.AddPoint(-100e-3 + 15e-3, -122.1e3 + 30e-3);
    m_roll_map.AddPoint(-90e-3 + 15e-3, -111.6e3 + 30e-3);
    m_roll_map.AddPoint(-80e-3 + 15e-3, -104.4e3 + 30e-3);
    m_roll_map.AddPoint(-70e-3 + 15e-3, -94.8e3 + 30e-3);
    m_roll_map.AddPoint(-60e-3 + 15e-3, -88.2e3 + 30e-3);
    m_roll_map.AddPoint(-50e-3 + 15e-3, -82.2e3 + 30e-3);
    m_roll_map.AddPoint(-40e-3 + 15e-3, -76.2e3 + 30e-3);
    m_roll_map.AddPoint(-30e-3 + 15e-3, -66.9e3 + 30e-3);
    m_roll_map.AddPoint(-20e-3 + 15e-3, -50.4e3 + 30e-3);
    m_roll_map.AddPoint(0, 0);
    m_roll_map.AddPoint(-10e-3 + 15e-3, -13.8e3 + 30e-3);
    m_roll_map.AddPoint(0 + 15e-3, -11.4e3 + 30e-3);
    m_roll_map.AddPoint(10e-3 + 15e-3, -9.3e3 + 30e-3);
    m_roll_map.AddPoint(20e-3 + 15e-3, -6.6e3 + 30e-3);
    m_roll_map.AddPoint(30e-3 + 15e-3, -4.8e3 + 30e-3);
    m_roll_map.AddPoint(40e-3 + 15e-3, -2.7e3 + 30e-3);
    m_roll_map.AddPoint(50e-3 + 15e-3, -0.6e3 + 30e-3);
    m_roll_map.AddPoint(60e-3 + 15e-3, 1.2e3 + 30e-3);
    m_roll_map.AddPoint(70e-3 + 15e-3, 3e3 + 30e-3);
    m_roll_map.AddPoint(80e-3 + 15e-3, 5.1e3 + 30e-3);
    m_roll_map.AddPoint(90e-3 + 15e-3, 6.9e3 + 30e-3);
    m_roll_map.AddPoint(100e-3 + 15e-3, 8.4e3 + 30e-3);
    m_roll_map.AddPoint(110e-3 + 15e-3, 10.2e3 + 30e-3);
    m_roll_map.AddPoint(120e-3 + 15e-3, 12e3 + 30e-3);
    m_roll_map.AddPoint(130e-3 + 15e-3, 13.2e3 + 30e-3);
    m_roll_map.AddPoint(140e-3 + 15e-3, 14.7e3 + 30e-3);
    m_roll_map.AddPoint(150e-3 + 15e-3, 15.9e3 + 30e-3);
    m_roll_map.AddPoint(160e-3 + 15e-3, 16.8e3 + 30e-3);
    m_roll_map.AddPoint(170e-3 + 15e-3, 18e3 + 30e-3);
    m_roll_map.AddPoint(180e-3 + 15e-3, 18.6e3 + 30e-3);
    m_roll_map.AddPoint(190e-3 + 15e-3, 18.9e3 + 30e-3);
    m_roll_map.AddPoint(200e-3 + 15e-3, 18.9e3 + 30e-3);
    m_roll_map.AddPoint(210e-3 + 15e-3, 19.2e3 + 30e-3);
    m_roll_map.AddPoint(220e-3 + 15e-3, 19.2e3 + 30e-3);
    m_roll_map.AddPoint(227.2e-3 + 15e-3, 19.5e3 + 30e-3);*/

	//roll stabilization version with only vertical offset
	m_roll_map.AddPoint(-127.8e-3, -145.2e3 + 11.4e3);
	m_roll_map.AddPoint(-120e-3, -140.4e3 + 11.4e3);
	m_roll_map.AddPoint(-110e-3, -129.6e3 + 11.4e3);
	m_roll_map.AddPoint(-100e-3, -122.1e3 + 11.4e3);
	m_roll_map.AddPoint(-90e-3, -111.6e3 + 11.4e3);
	m_roll_map.AddPoint(-80e-3, -104.4e3 + 11.4e3);
	m_roll_map.AddPoint(-70e-3, -94.8e3 + 11.4e3);
	m_roll_map.AddPoint(-60e-3, -88.2e3 + 11.4e3);
	m_roll_map.AddPoint(-50e-3, -82.2e3 + 11.4e3);
	m_roll_map.AddPoint(-40e-3, -76.2e3 + 11.4e3);
	m_roll_map.AddPoint(-30e-3, -66.9e3 + 11.4e3);
	m_roll_map.AddPoint(-20e-3, -50.4e3 + 11.4e3);
	m_roll_map.AddPoint(-10e-3, -13.8e3 + 11.4e3);
	m_roll_map.AddPoint(0, -11.4e3 + 11.4e3);
	m_roll_map.AddPoint(10e-3, -9.3e3 + 11.4e3);
	m_roll_map.AddPoint(20e-3, -6.6e3 + 11.4e3);
	m_roll_map.AddPoint(30e-3, -4.8e3 + 11.4e3);
	m_roll_map.AddPoint(40e-3, -2.7e3 + 11.4e3);
	m_roll_map.AddPoint(50e-3, -0.6e3 + 11.4e3);
	m_roll_map.AddPoint(60e-3, 1.2e3 + 11.4e3);
	m_roll_map.AddPoint(70e-3, 3e3 + 11.4e3);
	m_roll_map.AddPoint(80e-3, 5.1e3 + 11.4e3);
	m_roll_map.AddPoint(90e-3, 6.9e3 + 11.4e3);
	m_roll_map.AddPoint(100e-3, 8.4e3 + 11.4e3);
	m_roll_map.AddPoint(110e-3, 10.2e3 + 11.4e3);
	m_roll_map.AddPoint(120e-3, 12e3 + 11.4e3);
	m_roll_map.AddPoint(130e-3, 13.2e3 + 11.4e3);
	m_roll_map.AddPoint(140e-3, 14.7e3 + 11.4e3);
	m_roll_map.AddPoint(150e-3, 15.9e3 + 11.4e3);
	m_roll_map.AddPoint(160e-3, 16.8e3 + 11.4e3);
	m_roll_map.AddPoint(170e-3, 18e3 + 11.4e3);
	m_roll_map.AddPoint(180e-3, 18.6e3 + 11.4e3);
	m_roll_map.AddPoint(190e-3, 18.9e3 + 11.4e3);
	m_roll_map.AddPoint(200e-3, 18.9e3 + 11.4e3);
	m_roll_map.AddPoint(210e-3, 19.2e3 + 11.4e3);
	m_roll_map.AddPoint(220e-3, 19.2e3 + 11.4e3);
	m_roll_map.AddPoint(227.2e-3, 19.5e3 + 11.4e3);
}

double WVP_ShockForce::operator()(double time, double rest_length, double length, double vel, ChLinkSpringCB* link) {
    double length_mine;
    double length_other;
    double vel_mine;
    double vel_other;

    if (link == m_shock_left.get()) {
        length_mine = m_shock_left->GetSpringLength();
        vel_mine = m_shock_left->GetSpringVelocity();
        length_other = m_shock_right->GetSpringLength();
        vel_other = m_shock_right->GetSpringVelocity();
    } else {
        length_mine = m_shock_right->GetSpringLength();
        vel_mine = m_shock_right->GetSpringVelocity();
        length_other = m_shock_left->GetSpringLength();
        vel_other = m_shock_left->GetSpringVelocity();
    }

    double displ_mine = length_mine - m_rest_length;
    double displ_other = length_other - m_rest_length;

    assert(std::abs(length - length_mine) < 1e-8);
    assert(std::abs(vel - vel_mine) < 1e-8);

    bool parallel_travel = (displ_mine * displ_other >= 0);

	//flip sign of displacement and velocity since we assume compression negative, and WVP data has compression positive

    double force = 0;
    if (parallel_travel) {
        // lookup table for parallel travel
        force = interpolate2D(-displ_mine, -vel_mine, m_para_damp_map);  // damping
    } else {
        // lookup table for single wheel travel
        force = interpolate2D(-displ_mine, -vel_mine, m_single_damp_map);  // damping
        force += m_roll_map.Get_y(-displ_mine);                             // roll stabilization
    }

    return force;
}

double WVP_ShockForce::evaluate(double displ, double vel, double displ_other, double vel_other) {
    bool parallel_travel = (displ * displ_other >= 0);

	//flip sign of displacement and velocity since we assume compression negative, and WVP data has compression positive

    double force = 0;
    if (parallel_travel) {
        force = interpolate2D(-displ, -vel, m_para_damp_map);  // damping
    } else {
        force = interpolate2D(-displ, -vel, m_single_damp_map);  // damping
        force += m_roll_map.Get_y(-displ);                        // roll stabilization
    }

    return force;
}

double WVP_ShockForce::interpolate2D(double x, double y, std::map<double, ChFunction_Recorder> map2D) {
    // do a 2D linear interpolation from the map data that is given to us
    if (map2D.empty())
        return 0;

    // check if value is out of bounds
    if (y <= map2D.begin()->first) {
        return map2D.begin()->second.Get_y(x);
    } else if (y >= (--map2D.end())->first) {
        return (--map2D.end())->second.Get_y(x);
    }

    // std::cout<<"checked outside of range"<<std::endl;

    // find the maps that correspond to either side of the value
    std::map<double, ChFunction_Recorder>::iterator it = map2D.begin();
    double prevY = it->first;
    ChFunction_Recorder prevX = it->second;
    double currY = it->first;
    ChFunction_Recorder currX = it->second;
    // std::cout<<"before for loop"<<std::endl;
    while (it != map2D.end()) {
        // std::cout<<"looped"<<std::endl;
        if (it->first > y) {
            currY = it->first;
            currX = it->second;
            it = map2D.end();
        } else {
            prevY = it->first;
            prevX = it->second;
            ++it;
        }
    }
    // get the interpolated value from those two maps
    double z0 = prevX.Get_y(x);
    double z1 = currX.Get_y(x);
    double y0 = prevY;
    double y1 = currY;

    // interpolate between those two points
    return (z0 * (y1 - y) + z1 * (y - y0)) / (y1 - y0);
}

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
WVP_DoubleWishboneFront::WVP_DoubleWishboneFront(const std::string& name) : ChDoubleWishbone(name, true) {
    m_springForceCB = new WVP_SpringForce(0, m_springRestLength, m_springPreload);
    ////m_springForceCB = new LinearSpringForce(m_springCoefficient);

    m_shockForceCB = new WVP_ShockForce(0, m_springRestLength);
}

WVP_DoubleWishboneRear::WVP_DoubleWishboneRear(const std::string& name) : ChDoubleWishbone(name, true) {
    m_springForceCB = new WVP_SpringForce(1, m_springRestLength, m_springPreload);
    ////m_springForceCB = new LinearSpringForce(m_springCoefficient);

    m_shockForceCB = new WVP_ShockForce(1, m_springRestLength);
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
    GetShock(VehicleSide::LEFT)->SetSpringRestLength(m_springRestLength);
    GetShock(VehicleSide::RIGHT)->SetSpringRestLength(m_springRestLength);
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
    GetShock(VehicleSide::LEFT)->SetSpringRestLength(m_springRestLength);
    GetShock(VehicleSide::RIGHT)->SetSpringRestLength(m_springRestLength);
    static_cast<WVP_ShockForce*>(m_shockForceCB)->m_shock_left = GetShock(VehicleSide::LEFT);
    static_cast<WVP_ShockForce*>(m_shockForceCB)->m_shock_right = GetShock(VehicleSide::RIGHT);
}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> WVP_DoubleWishboneFront::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector<>(0, 1.033, 0);
        case UPRIGHT:
            return ChVector<>(0, 1.033, 0);
        case UCA_F:
            return ChVector<>(.309, .192, .365);
        case UCA_B:
            return ChVector<>(-.149, .192, .365);
        case UCA_U:
            return ChVector<>(-.008, .772, .234);
        case UCA_CM:
            return ChVector<>(.041, .428, .331);
        case LCA_F:
            return ChVector<>(.360, .192, .012);
        case LCA_B:
            return ChVector<>(-.250, .192, .012);
        case LCA_U:
            return ChVector<>(.007, .832, -.152);
        case LCA_CM:
            return ChVector<>(.039, .406, -.043);
        case SHOCK_C:
            return ChVector<>(-.107, .540, .724);
        case SHOCK_A:
            return ChVector<>(-.107, .547, -.111);
        case SPRING_C:
            return ChVector<>(-.107, .540, .724);
        case SPRING_A:
            return ChVector<>(-.107, .547, -.111);
        case TIEROD_C:
            return ChVector<>(-.383, .125, .029);
        case TIEROD_U:
            return ChVector<>(-.398, .750, -.128);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> WVP_DoubleWishboneRear::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector<>(0, 1.033, 0);
        case UPRIGHT:
            return ChVector<>(0, 1.033, 0);
        case UCA_F:
            return ChVector<>(.310, .192, .365);
        case UCA_B:
            return ChVector<>(-.148, .192, .365);
        case UCA_U:
            return ChVector<>(-.008, .772, .234);
        case UCA_CM:
            return ChVector<>(.041, .428, .331);
        case LCA_F:
            return ChVector<>(.360, .192, .012);
        case LCA_B:
            return ChVector<>(-.249, .192, .012);
        case LCA_U:
            return ChVector<>(.007, .832, -.152);
        case LCA_CM:
            return ChVector<>(.040, .406, -.043);
        case SHOCK_C:
            return ChVector<>(-.107, .540, .724);
        case SHOCK_A:
            return ChVector<>(-.107, .547, -.111);
        case SPRING_C:
            return ChVector<>(-.107, .540, .724);
        case SPRING_A:
            return ChVector<>(-.107, .547, -.111);
        case TIEROD_C:
            return ChVector<>(-.383, .125, .029);
        case TIEROD_U:
            return ChVector<>(-.398, .750, -.128);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
