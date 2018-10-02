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

#include "traffic_light.h"
#include "framework.h"

using namespace chrono;

namespace av {

TrafficLightList TrafficLight::m_traffic_lights;

// -----------------------------------------------------------------------------

TrafficLight::TrafficLight(Framework* framework,
                           const chrono::ChVector<>& center,
                           double radius,
                           const chrono::ChCoordsys<>& pos)
    : Agent(framework), m_center(center), m_radius(radius), m_pos(pos) {
    m_body = std::shared_ptr<ChBody>(framework->m_system->NewBody());
    m_body->SetPos(pos.pos);
    m_body->SetRot(pos.rot);
    m_body->SetBodyFixed(true);
    m_body->SetCollide(false);

    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, 3);
    cyl->GetCylinderGeometry().rad = 0.2;
    m_body->AddAsset(cyl);

    auto sphere = std::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = 0.4;
    sphere->Pos = ChVector<>(0, 0, 3);
    m_body->AddAsset(sphere);

    m_body->AddAsset(std::make_shared<ChColorAsset>(ChColor(0.6f, 0, 0)));

    framework->m_system->AddBody(m_body);

    m_bcast_freq = 0.5;
}

TrafficLight::~TrafficLight() {
    //
}

std::shared_ptr<TrafficLight> TrafficLight::Find(unsigned int id) {
    auto it = m_traffic_lights.find(id);
    if (it != m_traffic_lights.end())
        return it->second;
    return nullptr;
}

void TrafficLight::Broadcast(double time) {
    Message currentMessage(m_id, time, "Traffic Light " + std::to_string(m_id));

    for (auto v : Vehicle::GetList()) {
        if ((GetPosition().pos - v.second->GetPosition().pos).Length() <= 1000) {
            Send(v.second, currentMessage);
        }
    }
}

void TrafficLight::Unicast(double time) {
    //
}

void TrafficLight::ProcessMessages() {
    while (!m_messages.empty()) {
        std::cout << "Traffic Light " << m_id << " received message from " << m_messages.front().getText()
                  << " time stamp " << m_messages.front().getTimeStamp() << std::endl;
        m_messages.pop();
    }
}

void TrafficLight::Synchronize(double time) {
    //
}

void TrafficLight::Advance(double step) {
    //
}

}  // end namespace av
