// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// WVP engine model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/wvp/WVP_EngineShafts.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// Static variables
const double WVP_EngineShafts::m_motorblock_inertia = 10.5;
const double WVP_EngineShafts::m_motorshaft_inertia = 1.1;

WVP_EngineShafts::WVP_EngineShafts() : ChEngineShafts("Engine", ChVector<>(1, 0, 0)) {}

void WVP_EngineShafts::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;
    double ftlb_to_Nm = 1.355818;

    map->AddPoint(-100 * rpm_to_radsec, 300*ftlb_to_Nm);  // to start engine
    /*map->AddPoint(800 * rpm_to_radsec, 382*ftlb_to_Nm); //*/
    map->AddPoint(1000 * rpm_to_radsec, 683*ftlb_to_Nm);
    map->AddPoint(1200 * rpm_to_radsec, 897*ftlb_to_Nm);
    map->AddPoint(1400 * rpm_to_radsec, 959*ftlb_to_Nm);
    map->AddPoint(1600 * rpm_to_radsec, 959*ftlb_to_Nm);
    map->AddPoint(1800 * rpm_to_radsec, 905*ftlb_to_Nm);
    map->AddPoint(2000 * rpm_to_radsec, 838*ftlb_to_Nm);
    map->AddPoint(2200 * rpm_to_radsec, 768*ftlb_to_Nm);
    
    map->AddPoint(2400 * rpm_to_radsec, -200*ftlb_to_Nm); //
    /*map->AddPoint(2700 * rpm_to_radsec, -200*ftlb_to_Nm);  // fading out of engine torque*/
}

void WVP_EngineShafts::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-50 * rpm_to_radsec, 30);  // it should never work in negative direction, anyway..
    map->AddPoint(0 * rpm_to_radsec, 0);
    map->AddPoint(50 * rpm_to_radsec, -30);
    map->AddPoint(1000 * rpm_to_radsec, -50);
    map->AddPoint(2000 * rpm_to_radsec, -70);
    map->AddPoint(3000 * rpm_to_radsec, -90);
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
