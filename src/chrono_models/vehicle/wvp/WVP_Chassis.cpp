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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// WVP chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/wvp/WVP_Chassis.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double WVP_Chassis::m_body_mass = 6751.0+160.7+160-76.938;
//7071.71; //6074.77; //6074.77 to get close to given spring lengths //this is entire vehicle with payload
const ChVector<> WVP_Chassis::m_body_inertiaXX(6700, 25000, 28000);
const ChVector<> WVP_Chassis::m_body_inertiaXY(0, 0, 0);
const ChVector<> WVP_Chassis::m_body_COM_loc(-1.950, 0.016, .560);  //-2.138,.0129,1.355);
const ChCoordsys<> WVP_Chassis::m_driverCsys(ChVector<>(-1.8,.7,.560), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_Chassis::WVP_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
    : ChRigidChassis(name, fixed) {
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
    ChVehicleGeometry::BoxShape box1(ChVector<>(0.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0),
                                          ChVector<>(2.0, 1.0, 0.2));

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box1);

    m_geometry.m_has_mesh = true;
    m_geometry.m_vis_mesh_file = "wvp/wvp_chassis.obj";

    m_geometry.m_has_collision = (chassis_collision_type != CollisionType::NONE);
    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box1.m_matID = 0;
            m_geometry.m_coll_boxes.push_back(box1);
            break;
        case CollisionType::MESH: {
            ////ChRigidChassisGeometry::ConvexHullsShape hull("wvp/wvp_chassis_simple.obj", 0);
            ////m_geometry.m_coll_hulls.push_back(hull);
            break;
        }
        default:
            break;
    }
}

void WVP_Chassis::CreateContactMaterials(ChContactMethod contact_method) {
    // Create the contact materials.
    // In this model, we use a single material with default properties.
    MaterialInfo minfo;
    m_geometry.m_materials.push_back(minfo.CreateMaterial(contact_method));
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
