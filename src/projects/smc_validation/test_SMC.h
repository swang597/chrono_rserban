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
// Authors: Cecily Sunday, Radu Serban
// =============================================================================
//
// Common utility functions for SMC validation tests
//
// =============================================================================

#include <iostream>
#include <fstream>

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::collision;

int CreateOutputPath(const std::string& projname) {
    const std::string outdir = "../SMC_VALIDATION/" + projname;

    if (!filesystem::create_directory(filesystem::path("../SMC_VALIDATION")))
        return 1;
    if (!filesystem::create_directory(filesystem::path(outdir)))
        return 1;

    // Redirect stdout
    const std::string outdir_log = outdir + "/userlog.txt";
    fflush(stdout);
    freopen(outdir_log.c_str(), "w", stdout);

    chrono::SetChronoOutputPath(outdir);

    return 0;
}

ChSystemSMC::ContactForceModel force_to_enum(const std::string& str) {
    if (str == "hooke")
        return ChSystemSMC::ContactForceModel::Hooke;
    else if (str == "hertz")
        return ChSystemSMC::ContactForceModel::Hertz;
    else if (str == "plaincoulomb")
        return ChSystemSMC::ContactForceModel::PlainCoulomb;
    else if (str == "flores")
        return ChSystemSMC::ContactForceModel::Flores;
    else
        fprintf(stderr, "WARNING: Could not map desired force model. Reset to Hertz model.\n");
    return ChSystemSMC::ContactForceModel::Hertz;
}

std::shared_ptr<ChBody> AddSphere(int id,
                                  ChSystemParallelSMC* msystem,
                                  std::shared_ptr<ChMaterialSurfaceSMC> mat,
                                  double radius,
                                  double mass,
                                  ChVector<> pos,
                                  ChVector<> init_v) {
    // Shared parameters for the falling ball
    ChVector<> inertia(0.4 * mass * radius * radius * ChVector<>(1, 1, 1));
    ChQuaternion<> rot(1, 0, 0, 0);
    ChVector<> init_w(0, 0, 0);

    // Create a spherical body. Set body parameters and sphere collision model
    auto body = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
    body->SetIdentifier(id);
    body->SetMass(mass);
    body->SetPos(pos);
    body->SetRot(rot);
    body->SetPos_dt(init_v);
    body->SetWvel_par(init_w);
    body->SetInertiaXX(inertia);
    body->SetBodyFixed(false);
    body->SetCollide(true);

    body->GetCollisionModel()->ClearModel();
    utils::AddSphereGeometry(body.get(), mat, radius);
    body->GetCollisionModel()->BuildModel();

    // Attach a texture to the sphere
    auto mtexture = chrono_types::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("redwhite.png"));
    body->AddAsset(mtexture);

    // Return a pointer to the sphere object
    msystem->AddBody(body);
    return body;
}

std::shared_ptr<ChBody> AddWall(int id,
                                ChSystemParallelSMC* msystem,
                                std::shared_ptr<ChMaterialSurfaceSMC> mat,
                                ChVector<> size,
                                double mass,
                                ChVector<> pos,
                                ChVector<> init_v,
                                bool wall) {
    // Set parameters for the containing bin
    ChVector<> inertia((1.0 / 12.0) * mass * (pow(size.y(), 2) + pow(size.z(), 2)),
                       (1.0 / 12.0) * mass * (pow(size.x(), 2) + pow(size.z(), 2)),
                       (1.0 / 12.0) * mass * (pow(size.x(), 2) + pow(size.y(), 2)));
    ChQuaternion<> rot(1, 0, 0, 0);

    // Create container. Set body parameters and container collision model
    auto body = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
    body->SetIdentifier(id);
    body->SetMass(mass);
    body->SetPos(pos);
    body->SetRot(rot);
    body->SetPos_dt(init_v);
    body->SetInertiaXX(inertia);
    body->SetBodyFixed(wall);
    body->SetCollide(true);

    body->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(body.get(), mat, size / 2);
    body->GetCollisionModel()->BuildModel();

    // Attach a color to the visible container
    auto mvisual = chrono_types::make_shared<ChColorAsset>();
    mvisual->SetColor(ChColor(0.55f, 0.57f, 0.67f));
    body->AddAsset(mvisual);

    // Return a pointer to the wall object
    msystem->AddBody(body);
    return body;
}

void SetSimParameters(ChSystemParallelSMC* msystem, ChVector<> gravity, ChSystemSMC::ContactForceModel fmodel) {
    // Set solver settings and collision detection parameters
    msystem->Set_G_acc(gravity);

    msystem->GetSettings()->solver.max_iteration_bilateral = 100;
    msystem->GetSettings()->solver.tolerance = 1e-3;

    msystem->GetSettings()->solver.contact_force_model = fmodel;  /// Types: Hooke, Hertz, PlainCoulomb, Flores
    msystem->GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;
    msystem->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::MultiStep;

    msystem->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);
    msystem->GetSettings()->collision.narrowphase_algorithm =
        NarrowPhaseType::NARROWPHASE_HYBRID_MPR;  /// Types: NARROWPHASE_HYBRID_MPR, NARROWPHASE_R, NARROWPHASE_MPR

    msystem->ChangeCollisionSystem(
        CollisionSystemType::COLLSYS_PARALLEL);                  /// Types:: COLLSYS_PARALLEL, COLLSYS_BULLET_PARALLEL
    msystem->SetTimestepperType(ChTimestepper::Type::LEAPFROG);  /// Types: LEAPFROG....
}

bool CalcKE(ChSystemParallelSMC* msystem, const double& threshold) {
    const std::shared_ptr<ChBody> body = msystem->Get_bodylist().at(1);

    ChVector<> eng_trn = 0.5 * body->GetMass() * body->GetPos_dt() * body->GetPos_dt();
    ChVector<> eng_rot = 0.5 * body->GetInertiaXX() * body->GetWvel_par() * body->GetWvel_par();

    double KE_trn = eng_trn.x() + eng_trn.y() + eng_trn.z();
    double KE_rot = eng_rot.x() + eng_rot.y() + eng_rot.z();
    double KE_tot = KE_trn + KE_rot;

    if (KE_tot < threshold)
        return true;
    return false;
}

bool CalcAverageKE(ChSystemParallelSMC* msystem, const double& threshold) {
    // Calculate average KE
    double KE_trn = 0;
    double KE_rot = 0;

    for (int i = 0; i < msystem->Get_bodylist().size(); ++i) {
        const std::shared_ptr<ChBody> body = msystem->Get_bodylist().at(i);

        ChVector<> eng_trn = 0.5 * body->GetMass() * body->GetPos_dt() * body->GetPos_dt();
        ChVector<> eng_rot = 0.5 * body->GetInertiaXX() * body->GetWvel_par() * body->GetWvel_par();

        KE_trn += eng_trn.x() + eng_trn.y() + eng_trn.z();
        KE_rot += eng_rot.x() + eng_rot.y() + eng_rot.z();
    }

    double KE_trn_avg = KE_trn / msystem->Get_bodylist().size();
    double KE_rot_avg = KE_rot / msystem->Get_bodylist().size();
    double KE_tot_avg = KE_trn_avg + KE_rot_avg;

    // Return true if the calc falls below the given threshold
    if (KE_tot_avg < threshold)
        return true;
    return false;
}

#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono::irrlicht;
using namespace irr;
using namespace irr::video;

ChIrrApp* SetSimVis(ChSystemParallelSMC* msystem, double time_step, bool vis) {
    if (vis) {
        ChIrrApp* application = new ChIrrApp(msystem, L"Two sphere SMC test", core::dimension2d<u32>(800, 600));

        // Add camera, lights, logo and sky in Irrlicht scene
        application->AddTypicalLogo();
        application->AddTypicalSky();
        application->AddTypicalLights();
        application->AddTypicalCamera(core::vector3df(0, 0, -7.5));

        // Complete asset construction: convert all assets to Irrlicht
        application->SetStepManage(true);
        application->SetTimestep(time_step);
        application->AssetBindAll();
        application->AssetUpdateAll();

        return application;
    } else {
        return NULL;
    }
}
#endif
