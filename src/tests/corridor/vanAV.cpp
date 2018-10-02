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

#include "vanAV.h"
#include "framework.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::uaz;

namespace av {

VanAV::VanAV(Framework* framework, const chrono::ChCoordsys<>& init_pos) : Vehicle(framework) {
    m_uaz = std::make_shared<UAZBUS>(framework->m_system);

    m_uaz->SetChassisFixed(false);
    m_uaz->SetChassisCollisionType(ChassisCollisionType::NONE);
    m_uaz->SetTireType(TireModelType::RIGID);
    m_uaz->SetTireStepSize(framework->m_step);
    m_uaz->SetVehicleStepSize(framework->m_step);
    m_uaz->SetInitPosition(init_pos);
    m_uaz->Initialize();

    m_uaz->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    m_uaz->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_uaz->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    m_uaz->SetWheelVisualizationType(VisualizationType::NONE);
    m_uaz->SetTireVisualizationType(VisualizationType::PRIMITIVES);

    m_messageFrequency = 1;
}

VanAV::~VanAV() {
    //
}

void VanAV::recieveMessage(Message newMessage) {
    m_messagesIncoming.push(newMessage);
}

void VanAV::sendMessages(double time) {
    if (m_messageFrequency == -1) {
        return;
    }

    if (time - m_lastMessageTime < 1 / m_messageFrequency) {
        return;
    } else {
        m_lastMessageTime = time;
    }

    auto vehicleList = Vehicle::GetList();
    auto trafficLightList = TrafficLight::GetList();
    Message currentMessage(m_id, "VanAV " + std::to_string(m_id));

    for (auto v : vehicleList) {
        if (v.second->GetId() != m_id && (GetPosition().pos - v.second->GetPosition().pos).Length() <= 1000) {
            v.second->recieveMessage(currentMessage);
        }
    }

    for (auto l : trafficLightList) {
        if ((GetPosition().pos - l.second->GetPosition().pos).Length() <= 1000) {
            l.second->recieveMessage(currentMessage);
        }
    }
}

void VanAV::processMessages() {
    while (!m_messagesIncoming.empty()) {
        std::cout << "VanAV " << m_id << " recieved message from " << m_messagesIncoming.front().getText() << std::endl;
        m_messagesIncoming.pop();
    }
}

ChCoordsys<> VanAV::GetPosition() const {
    return ChCoordsys<>(m_uaz->GetVehicle().GetVehiclePos(), m_uaz->GetVehicle().GetVehicleRot());
}

ChVehicle& VanAV::GetVehicle() const {
    return m_uaz->GetVehicle();
}

ChPowertrain& VanAV::GetPowertrain() const {
    return m_uaz->GetPowertrain();
}

void VanAV::Synchronize(double time) {
    m_uaz->Synchronize(time, m_steering, m_braking, m_throttle, *m_framework->m_terrain);
}

void VanAV::Advance(double step) {
    m_uaz->Advance(step);
    AdvanceDriver(step);
}

}  // end namespace av
