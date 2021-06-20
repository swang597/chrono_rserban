// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Dispatcher for the ray intersection test
//
// =============================================================================

#pragma once

#include "chrono/collision/chrono/ChCollisionData.h"
#include "chrono/collision/chrono/ChConvexShape.h"

namespace chrono {
namespace collision {

/// @addtogroup collision_mc
/// @{

/// Class for performing ray intersection tests.
class ChApi ChRayTest {
  public:
    /// Information on ray intersection test result.
    struct RayHitInfo {
        int shapeID;   ///< identifier of closest hit shape
        real3 point;   ///< hit point in absolute frame
        real3 normal;  ///< normal to shape at hit point
        real t;        ///< ray parameter at hit point (in [0,1])
        real dist;     ///< distance to hit point from ray origin
    };

    ChRayTest(std::shared_ptr<ChCollisionData> data);

    /// Check for intersection of the given ray with all collision shapes in the system.
    /// Performs both a broadphase and narrowphase.
    bool Check(const real3& start,  ///< ray start point
               const real3& end,    ///< ray end point
               RayHitInfo& info     ///< [output] test result info
    );

  private:
    /// Set identifiers of candidate shapes for ray test.
    void FindCandidates(const real3& start,  ///< ray start point
                        const real3& end     ///< ray end point
    );

    /// Dispatcher for analytic functions for ray intersection with primitive shapes.
    bool CheckShape(const ConvexBase& shape,  ///< candidate shape
                    const real3& start,       ///< ray start point
                    const real3& end,         ///< ray end point
                    real3& normal,            ///< [output] normal to shape at intersectin point
                    real& mindist2            ///< [output] smallest squared distance to ray origin
    );

    std::vector<int> candidate_shapes;         ///< identifiers of candidate shapes for current ray test
    std::shared_ptr<ChCollisionData> cd_data;  ///< shared collision detection data
};

/// @} collision_mc

}  // end namespace collision
}  // end namespace chrono
