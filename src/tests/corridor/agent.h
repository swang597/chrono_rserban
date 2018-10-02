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
// Author: Radu Serban
// =============================================================================
//
// =============================================================================

#ifndef AV_AGENT_H
#define AV_AGENT_H

#include <queue>
#include <string>
#include <unordered_map>

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"
#include "message.h"

namespace av {
class Framework;

class Agent;

typedef std::unordered_map<unsigned int, std::shared_ptr<Agent>> AgentList;

class Agent {
  public:
    virtual ~Agent();

    unsigned int GetId() const { return m_id; }
    virtual chrono::ChCoordsys<> GetPosition() const = 0;

    static std::shared_ptr<Agent> Find(unsigned int id);
    static AgentList GetList() { return m_agents; }

  protected:
    Agent(Framework* framework, unsigned int id);

    void Send(std::shared_ptr<Agent> destination, std::shared_ptr<Message> message);

    unsigned int m_id;
    Framework* m_framework;

    std::queue<std::shared_ptr<Message>> m_messages;
    double m_bcast_freq;

    static unsigned int GenerateID();
    static unsigned int m_nextID;

  private:
    void SendMessages(double time);

    virtual void Broadcast(double time) = 0;
    virtual void Unicast(double time) = 0;
    virtual void ProcessMessages() = 0;

    virtual void Synchronize(double time) = 0;
    virtual void Advance(double step) = 0;

    double m_bcast_time;

    static AgentList m_agents;

    friend class Framework;
};

}  // end namespace av

#endif
