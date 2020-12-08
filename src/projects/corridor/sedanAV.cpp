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
// Authors: Radu Serban
// =============================================================================
//
// =============================================================================

#include "sedanAV.h"
#include "framework.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;

namespace av {

SedanAV::SedanAV(Framework* framework, unsigned int id, const chrono::ChCoordsys<>& init_pos)
    : Vehicle(framework, id, Vehicle::SEDAN) {
    m_sedan = chrono_types::make_shared<Sedan>(framework->m_system);

    m_sedan->SetChassisFixed(false);
    m_sedan->SetChassisCollisionType(CollisionType::MESH);
    m_sedan->SetTireType(TireModelType::RIGID);
    m_sedan->SetTireStepSize(framework->m_step);
    m_sedan->SetInitPosition(init_pos);
    m_sedan->Initialize();

    m_sedan->SetChassisVisualizationType(VisualizationType::NONE);
    m_sedan->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_sedan->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    m_sedan->SetWheelVisualizationType(VisualizationType::NONE);
    m_sedan->SetTireVisualizationType(VisualizationType::PRIMITIVES);
}

SedanAV::~SedanAV() {
    //
}

ChWheeledVehicle& SedanAV::GetVehicle() const {
    return m_sedan->GetVehicle();
}

void SedanAV::Synchronize(double time) {
    m_sedan->Synchronize(time, m_driver_inputs, *m_framework->m_terrain);
}

void SedanAV::Advance(double step) {
    m_sedan->Advance(step);
    AdvanceDriver(step);
}

}  // end namespace av
