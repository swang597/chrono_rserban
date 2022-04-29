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
// Rigid terrain
//
// =============================================================================

#include <cmath>
#include <cstdlib>

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"

#include "RigidTerrainSlope.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Constructor.
// -----------------------------------------------------------------------------
RigidTerrainSlope::RigidTerrainSlope(ChSystem* system) : m_friction(0.8f) {
    // Create the ground body and add it to the system.
    m_ground = std::shared_ptr<ChBody>(system->NewBody());
    m_ground->SetIdentifier(-1);
    m_ground->SetName("ground");
    m_ground->SetPos(ChVector<>(0, 0, 0));
    m_ground->SetBodyFixed(true);
    m_ground->SetCollide(true);
    system->AddBody(m_ground);
}

// -----------------------------------------------------------------------------
// Set the texture and texture scaling
// -----------------------------------------------------------------------------
void RigidTerrainSlope::SetTexture(const std::string tex_file, float tex_scale_x, float tex_scale_y) {
    m_ground->GetVisualShape(0)->SetTexture(tex_file, tex_scale_x, tex_scale_y);
}

// -----------------------------------------------------------------------------
// Initialize the terrain as a rigid box
// -----------------------------------------------------------------------------
void RigidTerrainSlope::Initialize(std::shared_ptr<ChMaterialSurface> mat,
                                   double height1,
                                   double height2,
                                   double grade,
                                   double sizeX,
                                   double sizeY) {
    m_friction = mat->GetSfriction();

    m_height1 = height1;
    m_height2 = height2;
    m_grade = grade;
    m_rise = std::abs(height2 - height1);
    m_run = (grade < 0.1) ? sizeX : 100 * m_rise / m_grade;
    m_angle = std::atan2(m_rise, m_run);

    double depth = 10;  // thickness of each contact box

    m_ground->GetCollisionModel()->ClearModel();

    {
        m_ground->GetCollisionModel()->AddBox(mat, sizeX / 2, sizeY / 2, depth / 2,
                                              ChVector<>(-sizeX / 2, 0, height1 - depth / 2));
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = ChVector<>(sizeX / 2, sizeY / 2, depth / 2);
        m_ground->AddVisualShape(box, ChFrame<>(ChVector<>(-sizeX / 2, 0, height1 - depth / 2)));
    }

    {
        double hx = (m_run / std::cos(m_angle)) / 2;
        double hy = sizeY / 2;
        double hz = depth / 2;

        double x = (m_run + depth * std::sin(m_angle)) / 2;
        double y = 0;
        double z = (m_height1 + m_height2 - depth * std::cos(m_angle)) / 2;

        ChMatrix33<> rot(-m_angle, ChVector<>(0, 1, 0));

        m_ground->GetCollisionModel()->AddBox(mat, hx, hy, hz, ChVector<>(x, y, z), rot);

        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = ChVector<>(hx, hy, hz);
        m_ground->AddVisualShape(box, ChFrame<>(ChVector<>(x, y, z), rot));
    }

    {
        m_ground->GetCollisionModel()->AddBox(mat, sizeX / 2, sizeY / 2, depth / 2,
                                              ChVector<>(m_run + sizeX / 2, 0, height2 - depth / 2));
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = ChVector<>(sizeX / 2, sizeY / 2, depth / 2);
        m_ground->AddVisualShape(box, ChFrame<>(ChVector<>(m_run + sizeX / 2, 0, height2 - depth / 2)));
    }

    m_ground->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// Return the terrain height at the specified location
// -----------------------------------------------------------------------------
double RigidTerrainSlope::GetHeight(const ChVector<>& loc) const {
    if (loc.x() <= 0)
        return m_height1;

    if (loc.x() > m_run)
        return m_height2;

    return m_height1 + m_rise * loc.x() / m_run;
}

// -----------------------------------------------------------------------------
// Return the terrain normal at the specified location
// -----------------------------------------------------------------------------
ChVector<> RigidTerrainSlope::GetNormal(const ChVector<>& loc) const {
    if (loc.x() < 0 || loc.x() > m_run)
        return ChVector<>(0, 0, 1);

    return ChVector<>(-std::sin(m_angle), 0, std::cos(m_angle));
}

}  // end namespace vehicle
}  // end namespace chrono
