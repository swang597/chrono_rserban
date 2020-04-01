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
// Template for a rigid-body chassis vehicle subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/chassis/ChTorsionChassis.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTorsionChassis::ChTorsionChassis(const std::string& name, bool fixed)
    : ChChassis(name, fixed),
      m_has_primitives(false),
      m_has_rear_primitives(false),
      m_has_mesh(false),
      m_has_rear_mesh(false),
      m_has_collision(false) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTorsionChassis::Initialize(ChSystem* system,
                                  const ChCoordsys<>& chassisPos,
                                  double chassisFwdVel,
                                  int collision_family) {
    // Invoke the base class method to construct the frame body.
    ChChassis::Initialize(system, chassisPos, chassisFwdVel);

    // Add the additional mass
    m_rear_body = std::shared_ptr<ChBodyAuxRef>(system->NewBodyAuxRef());
    m_rear_body->SetIdentifier(1);
    m_rear_body->SetNameString(m_name + " bodyRear");
    m_rear_body->SetMass(GetRearMass());
    m_rear_body->SetFrame_COG_to_REF(ChFrame<>(GetRearLocalPosCOM(), ChQuaternion<>(1, 0, 0, 0)));
    m_rear_body->SetInertia(GetRearInertia());
    m_rear_body->SetBodyFixed(false);  // fixing m_body is enough

    m_rear_body->SetFrame_REF_to_abs(ChFrame<>(chassisPos));
    m_rear_body->SetPos_dt(chassisFwdVel * chassisPos.TransformDirectionLocalToParent(ChVector<>(1, 0, 0)));

    system->Add(m_rear_body);

    // generate a rotational joint between m_body and m_rear_body
    ChCoordsys<> rev_csys(GetTorsionJointLocalPos(), Q_from_AngAxis(CH_C_PI / 2.0, VECT_Y));
    m_torsion_joint = chrono_types::make_shared<ChLinkLockRevolute>();
    m_torsion_joint->SetNameString(m_name + " torsionJoint");
    m_torsion_joint->Initialize(m_body, m_rear_body, rev_csys);

    system->AddLink(m_torsion_joint);
    // build a rotational spring that simulates the torsional stiffness of the chassis
    m_torsion_spring = chrono_types::make_shared<ChLinkRotSpringCB>();
    m_torsion_spring->SetNameString(m_name + " torsionSpring");
    m_torsion_spring->Initialize(m_body, m_rear_body, rev_csys);
    double K = GetTorsionStiffness();
    double C = K / 100;  // damping should not be zero
    m_torsion_spring->RegisterTorqueFunctor(new LinearSpringDamperTorque(K, C, 0));

    system->AddLink(m_torsion_spring);

    // Set chassis body contact material properties.
    switch (m_body->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            m_body->GetMaterialSurfaceNSC()->SetFriction(m_friction);
            m_body->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurface::SMC:
            m_body->GetMaterialSurfaceSMC()->SetFriction(m_friction);
            m_body->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
            m_body->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
            m_body->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
            m_body->GetMaterialSurfaceSMC()->SetKn(m_kn);
            m_body->GetMaterialSurfaceSMC()->SetGn(m_gn);
            m_body->GetMaterialSurfaceSMC()->SetKt(m_kt);
            m_body->GetMaterialSurfaceSMC()->SetGt(m_gt);
            break;
    }

    // If collision shapes were defined, create the contact geometry and enable contact
    // for the chassis's rigid body.
    // NOTE: setting the collision family is deferred to the containing vehicle system
    // (which can also disable contact between the chassis and certain vehicle subsystems).
    if (m_has_collision) {
        m_body->SetCollide(true);

        m_body->GetCollisionModel()->ClearModel();

        m_body->GetCollisionModel()->SetFamily(collision_family);

        for (auto sphere : m_coll_spheres) {
            m_body->GetCollisionModel()->AddSphere(sphere.m_radius, sphere.m_pos);
        }
        for (auto box : m_coll_boxes) {
            ChVector<> hdims = box.m_dims / 2;
            m_body->GetCollisionModel()->AddBox(hdims.x(), hdims.y(), hdims.z(), box.m_pos, box.m_rot);
        }
        for (auto cyl : m_coll_cylinders) {
            m_body->GetCollisionModel()->AddCylinder(cyl.m_radius, cyl.m_radius, cyl.m_length / 2, cyl.m_pos,
                                                     cyl.m_rot);
        }
        for (auto mesh_name : m_coll_mesh_names) {
            geometry::ChTriangleMeshConnected mesh;
            std::vector<std::vector<ChVector<>>> hulls;
            utils::LoadConvexHulls(vehicle::GetDataFile(mesh_name), mesh, hulls);
            for (int c = 0; c < hulls.size(); c++) {
                m_body->GetCollisionModel()->AddConvexHull(hulls[c]);
            }
        }
        m_body->GetCollisionModel()->BuildModel();
    }
}
// -----------------------------------------------------------------------------
// Get the current COM location of the suspension subsystem.
// -----------------------------------------------------------------------------
ChVector<> ChTorsionChassis::GetTotalCOMPos() {
    ChVector<> com(0, 0, 0);
    com += GetMass() * m_body->GetPos();
    com += GetRearMass() * m_rear_body->GetPos();
    return com / GetTotalMass();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTorsionChassis::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_vis_mesh_file), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(m_vis_mesh_file).stem());
        trimesh_shape->SetStatic(true);
        m_body->AddAsset(trimesh_shape);
    }

    if (vis == VisualizationType::MESH && m_has_rear_mesh) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_vis_rear_mesh_file), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(m_vis_rear_mesh_file).stem());
        trimesh_shape->SetStatic(true);
        m_body->AddAsset(trimesh_shape);
    }

    if (vis == VisualizationType::PRIMITIVES && m_has_primitives) {
        for (auto sphere : m_vis_spheres) {
            auto sphere_shape = chrono_types::make_shared<ChSphereShape>();
            sphere_shape->GetSphereGeometry().rad = sphere.m_radius;
            sphere_shape->Pos = sphere.m_pos;
            m_body->AddAsset(sphere_shape);
        }

        for (auto box : m_vis_boxes) {
            auto box_shape = chrono_types::make_shared<ChBoxShape>();
            box_shape->GetBoxGeometry().SetLengths(box.m_dims);
            box_shape->Pos = box.m_pos;
            box_shape->Rot = box.m_rot;
            m_body->AddAsset(box_shape);
        }

        for (auto cyl : m_vis_cylinders) {
            auto cyl_shape = chrono_types::make_shared<ChCylinderShape>();
            cyl_shape->GetCylinderGeometry().rad = cyl.m_radius;
            cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, cyl.m_length / 2, 0);
            cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, -cyl.m_length / 2, 0);
            cyl_shape->Pos = cyl.m_pos;
            cyl_shape->Rot = cyl.m_rot;
            m_body->AddAsset(cyl_shape);
        }
    }

    if (vis == VisualizationType::PRIMITIVES && m_has_rear_primitives) {
        for (auto sphere : m_rear_vis_spheres) {
            auto sphere_shape = chrono_types::make_shared<ChSphereShape>();
            sphere_shape->GetSphereGeometry().rad = sphere.m_radius;
            sphere_shape->Pos = sphere.m_pos;
            m_rear_body->AddAsset(sphere_shape);
        }

        for (auto box : m_rear_vis_boxes) {
            auto box_shape = chrono_types::make_shared<ChBoxShape>();
            box_shape->GetBoxGeometry().SetLengths(box.m_dims);
            box_shape->Pos = box.m_pos;
            box_shape->Rot = box.m_rot;
            m_rear_body->AddAsset(box_shape);
        }

        for (auto cyl : m_rear_vis_cylinders) {
            auto cyl_shape = chrono_types::make_shared<ChCylinderShape>();
            cyl_shape->GetCylinderGeometry().rad = cyl.m_radius;
            cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, cyl.m_length / 2, 0);
            cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, -cyl.m_length / 2, 0);
            cyl_shape->Pos = cyl.m_pos;
            cyl_shape->Rot = cyl.m_rot;
            m_rear_body->AddAsset(cyl_shape);
        }
    }

    auto sphere_shape_front = chrono_types::make_shared<ChSphereShape>();
    sphere_shape_front->GetSphereGeometry().rad = 0.1;
    sphere_shape_front->Pos = GetLocalPosCOM();
    m_body->AddAsset(sphere_shape_front);

    auto sphere_shape_rear = chrono_types::make_shared<ChSphereShape>();
    sphere_shape_rear->GetSphereGeometry().rad = 0.1;
    sphere_shape_rear->Pos = GetRearLocalPosCOM();
    m_rear_body->AddAsset(sphere_shape_rear);
}

void ChTorsionChassis::RemoveVisualizationAssets() {
    m_body->GetAssets().clear();
    m_rear_body->GetAssets().clear();
}

void ChTorsionChassis::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_body);
    bodies.push_back(m_rear_body);
    ChPart::ExportBodyList(jsonDocument, bodies);

    ChPart::ExportMarkerList(jsonDocument, m_markers);
}

void ChTorsionChassis::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBodyAuxRef>> bodies;
    bodies.push_back(m_body);
    bodies.push_back(m_rear_body);
    database.WriteAuxRefBodies(bodies);

    database.WriteMarkers(m_markers);
}

}  // end namespace vehicle
}  // end namespace chrono
