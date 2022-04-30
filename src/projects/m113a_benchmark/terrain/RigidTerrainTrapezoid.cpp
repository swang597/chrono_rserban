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
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Rigid terrain with a trapezoid shape
//
// =============================================================================

#include <cmath>
#include <cstdlib>

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"

#include "RigidTerrainTrapezoid.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Constructor.
// -----------------------------------------------------------------------------
RigidTerrainTrapezoid::RigidTerrainTrapezoid(ChSystem* system) : m_widebase(false), m_friction(0.8f) {
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
void RigidTerrainTrapezoid::SetTexture(const std::string tex_file, float tex_scale_x, float tex_scale_y) {
    m_ground->GetVisualShape(0)->SetTexture(tex_file, tex_scale_x, tex_scale_y);
}

// -----------------------------------------------------------------------------
// Initialize the terrain
// -----------------------------------------------------------------------------
void RigidTerrainTrapezoid::Initialize(std::shared_ptr<ChMaterialSurface> mat,  // [in] contact material
                                       double height1,                          // [in] terrain height (before slope)
                                       double height2,                          // [in] terrain height (after 1st slope)
                                       double height3,                          // [in] terrain height (after 2nd slope)
                                       double angle1,  // [in] 1st slope angle with respect to ground
                                       double angle2,  // [in] 2nd slope angle with respect to ground
                                       double sizeX1,  // [in] terrain dimension in the X direction (1st flat area)
                                       double sizeX2,  // [in] terrain dimension in the X direction (2nd flat area)
                                       double sizeX3,  // [in] terrain dimension in the X direction (3rd flat area)
                                       double sizeY,   // [in] terrain dimension in the Y direction
                                       double offsetX  // [in] offset in the +X direction for the start of 1st slope
) {
    m_friction = mat->GetSfriction();

    m_widebase = false;

    m_height1 = height1;
    m_height2 = height2;
    m_height3 = height3;
    m_sizeX2 = sizeX2;
    m_angle1 = angle1;
    m_angle2 = angle2;
    m_offsetX = offsetX;
    m_sizeY = sizeY;
    double m_rise1 = 0;
    double m_rise2 = 0;
    double m_run1 = 0;
    double m_run2 = 0;

    // Limit the angles to positive angles less than or equal to 90 degrees
    m_angle1 = std::abs(m_angle1);
    m_angle2 = std::abs(m_angle2);
    m_angle1 = (m_angle1 > CH_C_PI_2) ? CH_C_PI_2 : m_angle1;
    m_angle2 = (m_angle2 > CH_C_PI_2) ? CH_C_PI_2 : m_angle2;

    // Force the heights to be continuous if the provided angle between them is zero
    // Otherwise ensure the angle is > 0.01 to prevent rounding errors
    if (m_angle1 == 0)
        m_height2 = m_height1;
    else if (m_angle1 == CH_C_PI_2) {
        m_rise1 = std::abs(m_height2 - m_height1);
    } else {
        m_rise1 = std::abs(m_height2 - m_height1);
        m_run1 = (m_angle1 < 0.01) ? m_rise1 / std::tan(0.01) : m_rise1 / std::tan(m_angle1);
    }

    if (m_angle2 == 0)
        m_height3 = m_height2;
    else if (m_angle2 == CH_C_PI_2) {
        m_rise2 = std::abs(m_height3 - m_height2);
    } else {
        m_rise2 = std::abs(m_height3 - m_height2);
        m_run2 = (m_angle2 < 0.01) ? m_rise2 / std::tan(0.01) : m_rise2 / std::tan(m_angle2);
    }

    // Set the sign of the rise term and angle to indicate if the terrain is assending or desending along +X
    if (m_height2 < m_height1) {
        m_rise1 = -m_rise1;
        m_angle1 = -m_angle1;
    }
    if (m_height3 < m_height2) {
        m_rise2 = -m_rise2;
        m_angle2 = -m_angle2;
    }

    double depth = (std::abs(height2 - height1) > std::abs(height3 - height2))
                       ? 10 + std::abs(height2 - height1)
                       : 10 + std::abs(height3 - height2);  // thickness of each contact box

    m_ground->GetCollisionModel()->ClearModel();

    {
        m_ground->GetCollisionModel()->AddBox(mat, sizeX1 / 2, sizeY / 2, depth / 2,
                                              ChVector<>(m_offsetX - sizeX1 / 2, 0, height1 - depth / 2));
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = ChVector<>(sizeX1 / 2, sizeY / 2, depth / 2);
        m_ground->AddVisualShape(box, ChFrame<>(ChVector<>(m_offsetX - sizeX1 / 2, 0, height1 - depth / 2)));
    }

    if (m_run1 > 0)  // There is not a step change in height, so an angled section is needed
    {
        double hx = (std::sqrt(m_rise1 * m_rise1 + m_run1 * m_run1)) / 2;
        double hy = sizeY / 2;
        double hz = sizeX2 / 2;  // ensure that if angle1 = 90deg, the box dos not extend too far

        double x = m_offsetX + (m_run1 + sizeX2 * std::sin(m_angle1)) / 2;
        double y = 0;
        double z = (m_height1 + m_height2 - sizeX2 * std::cos(m_angle1)) / 2;

        ChMatrix33<> rot(-m_angle1, ChVector<>(0, 1, 0));

        m_ground->GetCollisionModel()->AddBox(mat, hx, hy, hz, ChVector<>(x, y, z), rot);

        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = ChVector<>(hx, hy, hz);
        m_ground->AddVisualShape(box, ChFrame<>(ChVector<>(x, y, z), rot));
    }

    {
        m_ground->GetCollisionModel()->AddBox(mat, sizeX2 / 2, sizeY / 2, depth / 2,
                                              ChVector<>(m_offsetX + m_run1 + sizeX2 / 2, 0, height2 - depth / 2));
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = ChVector<>(sizeX2 / 2, sizeY / 2, depth / 2);
        m_ground->AddVisualShape(box, ChFrame<>(ChVector<>(m_offsetX + m_run1 + sizeX2 / 2, 0, height2 - depth / 2)));
    }

    if (m_run2 > 0)  // There is not a step change in height, so an angled section is needed
    {
        double hx = (std::sqrt(m_rise2 * m_rise2 + m_run2 * m_run2)) / 2;
        double hy = sizeY / 2;
        double hz = sizeX2 / 2;  // ensure that if angle1 = 90deg, the box dos not extend too far

        double x = m_offsetX + m_run1 + m_sizeX2 + (m_run2 + sizeX2 * std::sin(m_angle2)) / 2;
        double y = 0;
        double z = (m_height2 + m_height3 - sizeX2 * std::cos(m_angle2)) / 2;

        ChMatrix33<> rot(-m_angle2, ChVector<>(0, 1, 0));

        m_ground->GetCollisionModel()->AddBox(mat, hx, hy, hz, ChVector<>(x, y, z), rot);

        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = ChVector<>(hx, hy, hz);
        m_ground->AddVisualShape(box, ChFrame<>(ChVector<>(x, y, z), rot));
    }

    {
        m_ground->GetCollisionModel()->AddBox(
            mat, sizeX3 / 2, sizeY / 2, depth / 2,
            ChVector<>(m_offsetX + m_run1 + sizeX2 + m_run2 + sizeX3 / 2, 0, height3 - depth / 2));
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = ChVector<>(sizeX3 / 2, sizeY / 2, depth / 2);
        m_ground->AddVisualShape(
            box, ChFrame<>(ChVector<>(m_offsetX + m_run1 + sizeX2 + m_run2 + sizeX3 / 2, 0, height3 - depth / 2)));
    }

    m_ground->GetCollisionModel()->BuildModel();
}

void RigidTerrainTrapezoid::Initialize(std::shared_ptr<ChMaterialSurface> mat,  // [in] contact material
                                       double heightT,                          // [in] terrain height
                                       ChVector2<> sizeT,                       // [in] total terrain length x width
                                       double heightB,                          // [in] berm height
                                       ChVector2<> sizeB,                       // [in] berm length x width
                                       double angle,                            // [in] slope angle
                                       double offsetX  // [in] offset in the +X direction for the start of up-slope
) {
    assert(heightB > heightT);
    assert(angle > 0);

    m_friction = mat->GetSfriction();

    m_widebase = true;

    m_height1 = heightT;
    m_height2 = heightB;
    m_offsetX = offsetX;
    m_angle1 = angle;
    m_angle2 = -angle;

    m_rise1 = heightB - heightT;
    m_run1 = m_rise1 / std::tan(angle);

    m_rise2 = -m_rise1;
    m_run2 = m_run1;

    m_ground->GetCollisionModel()->ClearModel();

    // Up slope
    {
        double hx = (std::sqrt(m_rise1 * m_rise1 + m_run1 * m_run1)) / 2;
        double hy = sizeB.y() / 2;
        double hz = sizeB.x() / 2;

        double x = m_offsetX + (m_run1 + sizeB.x() * std::sin(angle)) / 2;
        double y = 0;
        double z = (heightT + heightB - sizeB.x() * std::cos(angle)) / 2;

        ChMatrix33<> rot(-angle, ChVector<>(0, 1, 0));

        m_ground->GetCollisionModel()->AddBox(mat, hx, hy, hz, ChVector<>(x, y, z), rot);

        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = ChVector<>(hx, hy, hz);
        m_ground->AddVisualShape(box, ChFrame<>(ChVector<>(x, y, z), rot));
    }

    // Terrain & berm
    {
        double x_center = m_offsetX + m_run1 + sizeB.x() / 2;

        double depthT = 10 + heightT;
        m_ground->GetCollisionModel()->AddBox(mat, sizeT.x() / 2, sizeT.y() / 2, depthT / 2,
                                              ChVector<>(x_center, 0, heightT - depthT / 2));
        auto boxT = chrono_types::make_shared<ChBoxShape>();
        boxT->GetBoxGeometry().Size = ChVector<>(sizeT.x() / 2, sizeT.y() / 2, depthT / 2);
        m_ground->AddVisualShape(boxT, ChFrame<>(ChVector<>(x_center, 0, heightT - depthT / 2)));

        double depthB = 10 + heightB;
        m_ground->GetCollisionModel()->AddBox(mat, sizeB.x() / 2, sizeB.y() / 2, depthB / 2,
                                              ChVector<>(x_center, 0, heightB - depthB / 2));
        auto boxB = chrono_types::make_shared<ChBoxShape>();
        boxB->GetBoxGeometry().Size = ChVector<>(sizeB.x() / 2, sizeB.y() / 2, depthB / 2);
        m_ground->AddVisualShape(boxB, ChFrame<>(ChVector<>(x_center, 0, heightB - depthB / 2)));
    }

    // Down slope
    {
        double hx = (std::sqrt(m_rise2 * m_rise2 + m_run2 * m_run2)) / 2;
        double hy = sizeB.y() / 2;
        double hz = sizeB.x() / 2;

        double x = m_offsetX + m_run1 + sizeB.x() + (m_run2 - sizeB.x() * std::sin(angle)) / 2;
        double y = 0;
        double z = (heightB + heightT - sizeB.x() * std::cos(angle)) / 2;

        ChMatrix33<> rot(angle, ChVector<>(0, 1, 0));

        m_ground->GetCollisionModel()->AddBox(mat, hx, hy, hz, ChVector<>(x, y, z), rot);

        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = ChVector<>(hx, hy, hz);
        m_ground->AddVisualShape(box, ChFrame<>(ChVector<>(x, y, z), rot));
    }

    m_ground->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// Return the terrain height at the specified location
// -----------------------------------------------------------------------------
double RigidTerrainTrapezoid::GetHeight(const ChVector<>& loc) const {
    if (m_widebase) {
        //// TODO
        return m_height1;
    }

    double x = loc.x();
    double y = loc.y();

    if ((y > m_sizeY / 2) || (y < -m_sizeY / 2))
        return m_height1;

    if (x <= m_offsetX)
        return m_height1;

    if (x <= m_offsetX + m_run1)
        return m_height1 + m_rise1 * (x - m_offsetX) / m_run1;

    if (x <= m_offsetX + m_run1 + m_sizeX2)
        return m_height2;

    if (x <= m_offsetX + m_run1 + m_sizeX2 + m_run2)
        return m_height2 + m_rise2 * (x - (m_offsetX + m_run1 + m_sizeX2)) / m_run2;

    return m_height3;
}

// -----------------------------------------------------------------------------
// Return the terrain normal at the specified location
// -----------------------------------------------------------------------------
ChVector<> RigidTerrainTrapezoid::GetNormal(const ChVector<>& loc) const {
    if (m_widebase) {
        //// TODO
        return ChVector<>(0, 0, 1);
    }

    double x = loc.x();
    double y = loc.y();

    if ((y > m_sizeY / 2) || (y < -m_sizeY / 2))
        return ChVector<>(0, 0, 1);

    if (x <= m_offsetX)
        return ChVector<>(0, 0, 1);

    if (x <= m_offsetX + m_run1)
        return ChVector<>(-std::sin(m_angle1), 0, std::cos(m_angle1));

    if (x <= m_offsetX + m_run1 + m_sizeX2)
        return ChVector<>(0, 0, 1);

    if (x <= m_offsetX + m_run1 + m_sizeX2 + m_run2)
        return ChVector<>(-std::sin(m_angle2), 0, std::cos(m_angle2));

    return ChVector<>(0, 0, 1);
}

}  // end namespace vehicle
}  // end namespace chrono
