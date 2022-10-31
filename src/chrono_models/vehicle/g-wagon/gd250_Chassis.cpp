// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// GD250 chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/g-wagon/gd250_Chassis.h"

namespace chrono {
namespace vehicle {
namespace gwagon {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double GD250_Chassis::m_body_mass = 2254.0;
const ChVector<> GD250_Chassis::m_body_inertiaXX(785.0, 2612.0, 2761.0);
const ChVector<> GD250_Chassis::m_body_inertiaXY(0, 0, 0);
const ChVector<> GD250_Chassis::m_body_COM_loc(-1.422, 0.0, 0.3);
const ChVector<> GD250_Chassis::m_connector_rear_loc(-3.5, 0, -0.05);
const ChCoordsys<> GD250_Chassis::m_driverCsys(ChVector<>(-1.1, 0.7, 0.5), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
GD250_Chassis::GD250_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
    : ChRigidChassis(name, fixed) {
    // In this model, we use a single material with default properties.
    ChContactMaterialData minfo;
    m_geometry.m_materials.push_back(minfo);

    m_body_inertia(0, 0) = m_body_inertiaXX.x();
    m_body_inertia(1, 1) = m_body_inertiaXX.y();
    m_body_inertia(2, 2) = m_body_inertiaXX.z();

    m_body_inertia(0, 1) = m_body_inertiaXX.x();
    m_body_inertia(0, 2) = m_body_inertiaXX.y();
    m_body_inertia(1, 2) = m_body_inertiaXX.z();
    m_body_inertia(1, 0) = m_body_inertiaXX.x();
    m_body_inertia(2, 0) = m_body_inertiaXX.y();
    m_body_inertia(2, 1) = m_body_inertiaXX.z();

    //// TODO:
    //// A more appropriate contact shape from primitives
    ChVehicleGeometry::BoxShape box1(ChVector<>(-1.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0), ChVector<>(1.6, 1.0, 0.2));

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box1);

    m_geometry.m_has_mesh = true;
    m_geometry.m_vis_mesh_file = "g-wagon/gd250_Chassis.obj";

    m_geometry.m_has_collision = (chassis_collision_type != CollisionType::NONE);
    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box1.m_matID = 0;
            m_geometry.m_coll_boxes.push_back(box1);
            break;
        case CollisionType::HULLS: {
            ChVehicleGeometry::ConvexHullsShape hull("uaz/GD250_Chassis_simple.obj", 0);
            m_geometry.m_coll_hulls.push_back(hull);
            break;
        }
        default:
            break;
    }
}

}  // end namespace gwagon
}  // end namespace vehicle
}  // end namespace chrono
