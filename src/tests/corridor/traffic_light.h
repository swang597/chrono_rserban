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

#ifndef AV_TRAFFIC_LIGHT_H
#define AV_TRAFFIC_LIGHT_H

#include "agent.h"
#include "fsm/fsm.hpp"
#include "scene.h"

namespace av {

class TrafficLight;

typedef std::unordered_map<unsigned int, std::shared_ptr<TrafficLight>> TrafficLightList;

class TrafficLight : public Agent {
  public:
    ~TrafficLight();

    chrono::ChVector<> GetCenter() const { return m_center; }
    double GetRadius() const { return m_radius; }

    virtual chrono::ChCoordsys<> GetPosition() const override { return m_pos; }

    static std::shared_ptr<TrafficLight> Find(unsigned int id);
    static TrafficLightList GetList() { return m_traffic_lights; }

    virtual void sendMessages(double time) override;
    virtual void processMessages() override;

  protected:
    TrafficLight(Framework* framework,
                 const chrono::ChVector<>& center,
                 double radius,
                 const chrono::ChCoordsys<>& pos);

    chrono::ChVector<> m_center;
    double m_radius;

    chrono::ChCoordsys<> m_pos;
    std::shared_ptr<chrono::ChBody> m_body;

    fsm::stack m_fsm;

    static TrafficLightList m_traffic_lights;

    friend class Framework;
};

}  // end namespace av

#endif
