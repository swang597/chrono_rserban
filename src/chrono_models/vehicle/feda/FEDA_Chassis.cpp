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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// FEDA chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/feda/FEDA_Chassis.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
/* VIPER config
const double FEDA_Chassis::m_mass = 4450;
const ChVector<> FEDA_Chassis::m_inertiaXX(2420.0, 8200.0, 7100.0);
const ChVector<> FEDA_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> FEDA_Chassis::m_COM_loc(-1.591564, 0.0889, 0.57);
*/
// configuration as tested on proving ground
const double FEDA_Chassis::m_mass = 5672.87;
const ChVector<> FEDA_Chassis::m_inertiaXX(5.74E+03, 7.66E+03, 9.87E+03);
const ChVector<> FEDA_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> FEDA_Chassis::m_COM_loc(-(1.0 - 0.4162) * 3.302, 0.0889, 0.61);

const ChCoordsys<> FEDA_Chassis::m_driverCsys(ChVector<>(-1.35, 0.52, 1.01), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FEDA_Chassis::FEDA_Chassis(const std::string& name, bool fixed, ChassisCollisionType chassis_collision_type)
    : ChRigidChassis(name, fixed) {
    m_inertia(0, 0) = m_inertiaXX.x();
    m_inertia(1, 1) = m_inertiaXX.y();
    m_inertia(2, 2) = m_inertiaXX.z();

    m_inertia(0, 1) = m_inertiaXY.x();
    m_inertia(0, 2) = m_inertiaXY.y();
    m_inertia(1, 2) = m_inertiaXY.z();
    m_inertia(1, 0) = m_inertiaXY.x();
    m_inertia(2, 0) = m_inertiaXY.y();
    m_inertia(2, 1) = m_inertiaXY.z();

    //// TODO:
    //// A more appropriate contact shape from primitives
    BoxShape box1(ChVector<>(0.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0), ChVector<>(1.0, 0.5, 0.2));

    m_has_primitives = true;
    m_vis_boxes.push_back(box1);

    m_has_mesh = true;
    m_vis_mesh_name = "FEDA_chassis_POV_geom";
    m_vis_mesh_file = "feda/meshes/feda_body.obj";

    m_has_collision = (chassis_collision_type != ChassisCollisionType::NONE);
    switch (chassis_collision_type) {
        case ChassisCollisionType::PRIMITIVES:
            m_coll_boxes.push_back(box1);
            break;
        case ChassisCollisionType::MESH:
            m_coll_mesh_names.push_back("feda/meshes/FEDA_chassis_col.obj");
            break;
        default:
            break;
    }
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
