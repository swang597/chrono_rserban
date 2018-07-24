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

#ifndef AV_SCENE_H
#define AV_SCENE_H

#include "chrono/core/ChVector.h"
#include "chrono/core/ChVector2.h"

namespace av {

typedef chrono::ChVector2<> GPScoord;

struct Area {
    Area(const chrono::ChVector<>& loc_min, const chrono::ChVector<>& loc_max) : m_min(loc_min), m_max(loc_max) {}
    chrono::ChVector<> m_min;
    chrono::ChVector<> m_max;
};

class Scene {
  public:
    Scene(const GPScoord& origin,
          const std::string& vis_model_file,
          const std::string& coll_model_file);

    chrono::ChVector2<> FromGPS(const GPScoord& gps) const;

  private:
    GPScoord m_origin;
    std::string m_vis_file;
    std::string m_coll_file;

    double m_lat0;
    double m_long0;
    double m_cos0;

    static const double m_radius;

    friend class Framework;
};

}  // end namespace av

#endif
