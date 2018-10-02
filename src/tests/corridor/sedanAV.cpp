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

SedanAV::SedanAV(Framework* framework, const chrono::ChCoordsys<>& init_pos) : Vehicle(framework) {
    m_sedan = std::make_shared<Sedan>(framework->m_system);

    m_sedan->SetChassisFixed(false);
    m_sedan->SetChassisCollisionType(ChassisCollisionType::NONE);
    m_sedan->SetTireType(TireModelType::RIGID);
    m_sedan->SetTireStepSize(framework->m_step);
    m_sedan->SetVehicleStepSize(framework->m_step);
    m_sedan->SetInitPosition(init_pos);
    m_sedan->Initialize();

    m_sedan->SetChassisVisualizationType(VisualizationType::NONE);
    m_sedan->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_sedan->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    m_sedan->SetWheelVisualizationType(VisualizationType::NONE);
    m_sedan->SetTireVisualizationType(VisualizationType::PRIMITIVES);

    m_messageFrequency = 1;
}

SedanAV::~SedanAV() {
    //
}

void SedanAV::recieveMessage(Message newMessage) {
    m_messagesIncoming.push(newMessage);
}

void SedanAV::sendMessages(double time) {
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
    Message currentMessage(m_id, "SedanAV " + std::to_string(m_id));

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

void SedanAV::processMessages() {
    while (!m_messagesIncoming.empty()) {
        std::cout << "SedanAV " << m_id << " recieved message from " << m_messagesIncoming.front().getText()
                  << std::endl;
        m_messagesIncoming.pop();
    }
}

ChCoordsys<> SedanAV::GetPosition() const {
    return ChCoordsys<>(m_sedan->GetVehicle().GetVehiclePos(), m_sedan->GetVehicle().GetVehicleRot());
}

ChVehicle& SedanAV::GetVehicle() const {
    return m_sedan->GetVehicle();
}

ChPowertrain& SedanAV::GetPowertrain() const {
    return m_sedan->GetPowertrain();
}

void SedanAV::Synchronize(double time) {
    m_sedan->Synchronize(time, m_steering, m_braking, m_throttle, *m_framework->m_terrain);
}

void SedanAV::Advance(double step) {
    m_sedan->Advance(step);
    AdvanceDriver(step);
}

}  // end namespace av
