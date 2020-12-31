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
// Test program for Chrono::Multicore with Irrlicht visualization
//
// =============================================================================

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"

using namespace chrono;
using namespace chrono::irrlicht;

// -----------------------------------------------------------------------------
// Create the container (fixed to ground).
// -----------------------------------------------------------------------------
void AddContainer(ChSystemMulticoreNSC* sys) {
    // Create a common material
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);

    // Create the containing bin (4 x 4 x 1), tilted about Y
    auto bin = chrono_types::make_shared<ChBody>(chrono_types::make_shared<collision::ChCollisionModelMulticore>());
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(Q_from_AngY(CH_C_PI / 20));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    ChVector<> hdim(2, 2, 0.5);
    double hthick = 0.1;

    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hdim.y(), hthick), ChVector<>(0, 0, -hthick));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hthick, hdim.y(), hdim.z()),
                          ChVector<>(-hdim.x() - hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hthick, hdim.y(), hdim.z()),
                          ChVector<>(hdim.x() + hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hthick, hdim.z()),
                          ChVector<>(0, -hdim.y() - hthick, hdim.z()), QUNIT, false);
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hthick, hdim.z()),
                          ChVector<>(0, +hdim.y() + hthick, hdim.z()), QUNIT, false);
    bin->GetCollisionModel()->BuildModel();

    sys->AddBody(bin);
}

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a rectangular grid.
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemMulticore* sys) {
    // Common material
    auto ballMat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ballMat->SetFriction(0.4f);

    // Create the falling balls
    double mass = 1;
    double radius = 0.15;
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

    for (int ix = -2; ix <= 2; ix++) {
        for (int iy = -2; iy <= 2; iy++) {
            ChVector<> pos(0.4 * ix, 0.4 * iy, 1);

            auto ball = chrono_types::make_shared<ChBody>(chrono_types::make_shared<collision::ChCollisionModelMulticore>());

            ball->SetMass(mass);
            ball->SetInertiaXX(inertia);
            ball->SetPos(pos);
            ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
            ball->SetBodyFixed(false);
            ball->SetCollide(true);

            ball->GetCollisionModel()->ClearModel();
            utils::AddSphereGeometry(ball.get(), ballMat, radius);
            ball->GetCollisionModel()->BuildModel();

            sys->AddBody(ball);
        }
    }
}

// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create system
    // -------------

    ChSystemMulticoreNSC msystem;

    // Set number of threads
    int threads = 8;
    int max_threads = omp_get_num_procs();
    if (threads > max_threads)
        threads = max_threads;
    msystem.SetNumThreads(threads);

    // Set gravitational acceleration
    msystem.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Set solver parameters
    msystem.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    msystem.GetSettings()->solver.max_iteration_normal = 0;
    msystem.GetSettings()->solver.max_iteration_sliding = 100;
    msystem.GetSettings()->solver.max_iteration_spinning = 0;
    msystem.GetSettings()->solver.max_iteration_bilateral = 0;
    msystem.GetSettings()->solver.tolerance = 1e-3;
    msystem.GetSettings()->solver.alpha = 0;
    msystem.GetSettings()->solver.contact_recovery_speed = 10000;
    msystem.ChangeSolverType(SolverType::APGD);
    msystem.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    msystem.GetSettings()->collision.collision_envelope = 0.01;
    msystem.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // Create the fixed and moving bodies
    // ----------------------------------

    AddContainer(&msystem);
    AddFallingBalls(&msystem);

    // Create Irrlicht application
    // ---------------------------

    ChIrrApp application(&msystem, L"Multicore + Irrlicht", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights(irr::core::vector3df(30.f, -100.f, 30.f), irr::core::vector3df(30.f, -80.f, -30.f));
    application.AddTypicalCamera(irr::core::vector3df(0, -5, 1), irr::core::vector3df(0, 0, 0));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Perform the simulation
    // ----------------------

    application.SetTimestep(5e-3);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
