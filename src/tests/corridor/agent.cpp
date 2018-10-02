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

#include "agent.h"

namespace av {

// Initialize static members
unsigned int Agent::m_nextID = 0;
AgentList Agent::m_agents;

// -----------------------------------------------------------------------------

Agent::Agent(Framework* framework) : m_framework(framework), m_bcast_time(0), m_bcast_freq(-1){
    //
}

Agent::~Agent() {
    //
}

std::shared_ptr<Agent> Agent::Find(unsigned int id) {
    auto it = m_agents.find(id);
    if (it != m_agents.end())
        return it->second;
    return nullptr;
}

unsigned int Agent::GenerateID() {
    return m_nextID++;
}

void Agent::SendMessages(double time) {
    if (m_bcast_freq > 0 && std::abs(time - m_bcast_time) < 1e-6) {
        m_bcast_time += 1 / m_bcast_freq;
        Broadcast(time);
    }

    Unicast(time);
}

void Agent::Send(std::shared_ptr<Agent> destination, Message message) {
    destination->m_messages.push(message);
}

}  // end namespace av
