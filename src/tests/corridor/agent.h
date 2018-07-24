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

#include <string>
#include <unordered_map>

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"

namespace av {

class Framework;

class Agent {
  public:
    enum Type { VEHICLE, TRAFFIC };

    virtual ~Agent();

    unsigned int GetId() const { return m_id; }

  protected:
    Agent(Framework* framework);

    unsigned int m_id;
    Framework* m_framework;

  protected:
    static unsigned int GenerateID();
    static unsigned int m_nextID;
};

}  // end namespace av

#endif
