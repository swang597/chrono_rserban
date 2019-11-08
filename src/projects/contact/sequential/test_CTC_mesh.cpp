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
// Test mesh collision
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

// ====================================================================================

int main(int argc, char* argv[]) {
    // ---------------------
    // Simulation parameters
    // ---------------------

    double gravity = 9.81;    // gravitational acceleration
    double time_step = 1e-4;  // integration step size

    enum class ContactType {
        MESH_BOX,  // use box for ground collision model
        MESH_MESH  // use mesh for ground collision model
    };
    ContactType contact_type = ContactType::MESH_MESH;

    double mesh_swept_sphere_radius = 0.005; 

    // ---------------------------
    // Contact material properties
    // ---------------------------
    ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::SMC;
    bool use_mat_properties = true;

    float object_friction = 0.9f;
    float object_restitution = 0.1f;
    float object_young_modulus = 2e7f;
    float object_poisson_ratio = 0.3f;
    float object_adhesion = 0.0f;
    float object_kn = 2e5;
    float object_gn = 40;
    float object_kt = 2e5;
    float object_gt = 20;

    float ground_friction = 0.9f;
    float ground_restitution = 0.01f;
    float ground_young_modulus = 1e6f;
    float ground_poisson_ratio = 0.3f;
    float ground_adhesion = 0.0f;
    float ground_kn = 2e5;
    float ground_gn = 40;
    float ground_kt = 2e5;
    float ground_gt = 20;

    // ---------------------------------
    // Parameters for the falling object
    // ---------------------------------

    ChVector<> pos(0, 0.75, 0);
    ChVector<> init_vel(0, 0, 0);
    ChVector<> init_omg(0, 0, 0);

    // ---------------------------------
    // Parameters for the containing bin
    // ---------------------------------

    double width = 2;
    double length = 1;
    double thickness = 0.1;

    // -----------------
    // Create the system
    // -----------------

    ChSystem* system;

    switch (contact_method) {
        case ChMaterialSurface::NSC:
            system = new ChSystemNSC();
            break;
        case ChMaterialSurface::SMC:
            system = new ChSystemSMC(use_mat_properties);
            break;
    }

    system->Set_G_acc(ChVector<>(0, -gravity, 0));

    // Create the Irrlicht visualization
    ChIrrApp application(system, L"mesh collision", irr::core::dimension2d<irr::u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(0.5, 1, -2), irr::core::vector3df(0, 0, 0));

    // This means that contact forces will be shown in Irrlicht application
    application.SetSymbolscale(5e-3);
    application.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_FORCES);

    // Rotation Z->Y (becauset meshes used here assume Z up)
    ChQuaternion<> z2y = Q_from_AngX(-CH_C_PI_2);

    // Create the falling object
    auto object = std::shared_ptr<ChBody>(system->NewBody());
    system->AddBody(object);

    object->SetMass(200);
    object->SetInertiaXX(40.0 * ChVector<>(1, 1, 0.2));
    object->SetPos(pos);
    object->SetRot(z2y);
    object->SetPos_dt(init_vel);
    object->SetWvel_par(init_omg);
    object->SetCollide(true);
    object->SetBodyFixed(false);

    switch (object->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            object->GetMaterialSurfaceNSC()->SetFriction(object_friction);
            object->GetMaterialSurfaceNSC()->SetRestitution(object_restitution);
            break;
        case ChMaterialSurface::SMC:
            object->GetMaterialSurfaceSMC()->SetFriction(object_friction);
            object->GetMaterialSurfaceSMC()->SetRestitution(object_restitution);
            object->GetMaterialSurfaceSMC()->SetYoungModulus(object_young_modulus);
            object->GetMaterialSurfaceSMC()->SetPoissonRatio(object_poisson_ratio);
            object->GetMaterialSurfaceSMC()->SetKn(object_kn);
            object->GetMaterialSurfaceSMC()->SetGn(object_gn);
            object->GetMaterialSurfaceSMC()->SetKt(object_kt);
            object->GetMaterialSurfaceSMC()->SetGt(object_gt);
            break;
    }

    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(GetChronoDataFile("vehicle/hmmwv/hmmwv_tire.obj"), true, false);

    object->GetCollisionModel()->ClearModel();
    object->GetCollisionModel()->AddTriangleMesh(trimesh, false, false, ChVector<>(0), ChMatrix33<>(1),
                                                 mesh_swept_sphere_radius);
    object->GetCollisionModel()->BuildModel();

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    object->AddAsset(trimesh_shape);

    std::shared_ptr<ChColorAsset> mcol(new ChColorAsset);
    mcol->SetColor(ChColor(0.3f, 0.3f, 0.3f));
    object->AddAsset(mcol);

    // Create ground body
    auto ground = std::shared_ptr<ChBody>(system->NewBody());
    system->AddBody(ground);

    ground->SetMass(1);
    ground->SetPos(ChVector<>(0, 0, 0));
    ground->SetRot(z2y);
    ground->SetCollide(true);
    ground->SetBodyFixed(true);

    switch (object->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            ground->GetMaterialSurfaceNSC()->SetFriction(ground_friction);
            ground->GetMaterialSurfaceNSC()->SetRestitution(ground_restitution);
            break;
        case ChMaterialSurface::SMC:
            ground->GetMaterialSurfaceSMC()->SetFriction(ground_friction);
            ground->GetMaterialSurfaceSMC()->SetRestitution(ground_restitution);
            ground->GetMaterialSurfaceSMC()->SetYoungModulus(ground_young_modulus);
            ground->GetMaterialSurfaceSMC()->SetPoissonRatio(ground_poisson_ratio);
            ground->GetMaterialSurfaceSMC()->SetKn(ground_kn);
            ground->GetMaterialSurfaceSMC()->SetGn(ground_gn);
            ground->GetMaterialSurfaceSMC()->SetKt(ground_kt);
            ground->GetMaterialSurfaceSMC()->SetGt(ground_gt);
            break;
    }

    switch (contact_type) {
        case ContactType::MESH_BOX: {
            ground->GetCollisionModel()->ClearModel();
            ground->GetCollisionModel()->AddBox(width, length, thickness, ChVector<>(0, 0, -thickness));
            ground->GetCollisionModel()->BuildModel();

            auto box = chrono_types::make_shared<ChBoxShape>();
            box->GetBoxGeometry().Size = ChVector<>(width, length, thickness);
            box->GetBoxGeometry().Pos = ChVector<>(0, 0, -thickness);
            ground->AddAsset(box);

            break;
        }
        case ContactType::MESH_MESH: {
            auto trimesh_ground = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
            trimesh_ground->LoadWavefrontMesh(GetChronoDataFile("vehicle/terrain/meshes/ramp_10x1.obj"), true, false);

            ground->GetCollisionModel()->ClearModel();
            ground->GetCollisionModel()->AddTriangleMesh(trimesh_ground, false, false, ChVector<>(0), ChMatrix33<>(1),
                                                         mesh_swept_sphere_radius);
            ground->GetCollisionModel()->BuildModel();

            auto trimesh_ground_shape = chrono_types::make_shared<ChTriangleMeshShape>();
            trimesh_ground_shape->SetMesh(trimesh_ground);
            ground->AddAsset(trimesh_ground_shape);

            break;
        }
    }

    // Complete asset construction
    application.AssetBindAll();
    application.AssetUpdateAll();

    // ---------------
    // Simulation loop
    // ---------------
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();

        system->DoStepDynamics(time_step);

        application.EndScene();
    }

    return 0;
}
