
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
// ChronoParallel test for box-box collision
//
// The global reference frame has Y up.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;

// -----------------------------------------------------------------------------
// Callback functor for contact reporting
// -----------------------------------------------------------------------------
class ContactManager : public ChContactContainer::ReportContactCallback {
  public:
    ContactManager(std::shared_ptr<ChBody> box) : m_box(box) {}

  private:
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        if (modA == m_box.get()) {
            printf("  %6.3f  %6.3f  %6.3f\n", pA.x(), pA.y(), pA.z());
        } else if (modB == m_box.get()) {
            printf("  %6.3f  %6.3f  %6.3f\n", pB.x(), pB.y(), pB.z());
        }
        return true;
    }

    std::shared_ptr<ChBody> m_box;
};

void ReportContacts(ChSystemParallel* system, unsigned int id) {
    auto bb = system->data_manager->host_data.bids_rigid_rigid;
    auto p1 = system->data_manager->host_data.cpta_rigid_rigid;
    auto p2 = system->data_manager->host_data.cptb_rigid_rigid;

    for (uint i = 0; i < system->data_manager->num_rigid_contacts; i++) {
        // IDs of bodies in contact
        int b1 = bb[i].x;
        int b2 = bb[i].y;

        if (id == b1) {
            printf("  %6.3f  %6.3f  %6.3f\n", p1[i].x, p1[i].y, p1[i].z);
        } else if (id == b2) {
            printf("  %6.3f  %6.3f  %6.3f\n", p2[i].x, p2[i].y, p2[i].z);
        }
    }
}

// --------------------------------------------------------------------------

int main(int argc, char** argv) {
    // ----------------
    // Parameters
    // ----------------

    double time_end = 4;
    double time_step = 1e-3;

    double tolerance = 0;
    double contact_recovery_speed = 1e8;
    double collision_envelope = .001;

    uint max_iteration_normal = 0;
    uint max_iteration_sliding = 0;
    uint max_iteration_spinning = 100;
    uint max_iteration_bilateral = 0;

    // ------------------------
    // Create the parallel system
    // --------------------------

    ChSystemParallelNSC system;
    system.Set_G_acc(ChVector<>(0, -10, 0));

    // Set number of threads
    system.SetParallelThreadNumber(1);
    CHOMPfunctions::SetNumThreads(1);

    // Set solver settings
    system.ChangeSolverType(SolverType::APGD);

    system.GetSettings()->perform_thread_tuning = false;

    system.GetSettings()->solver.solver_mode = SolverMode::SPINNING;
    system.GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    system.GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    system.GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    system.GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    system.GetSettings()->solver.alpha = 0;
    system.GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    system.GetSettings()->solver.use_full_inertia_tensor = false;
    system.GetSettings()->solver.tolerance = tolerance;

    system.GetSettings()->collision.collision_envelope = collision_envelope;
    system.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    system.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // ----------
    // Add bodies
    // ----------

    auto container = std::shared_ptr<ChBody>(system.NewBody());
    system.Add(container);
    container->SetPos(ChVector<>(0, 0, 0));
    container->SetBodyFixed(true);
    container->SetIdentifier(-1);

    container->GetMaterialSurfaceNSC()->SetFriction(0.4f);

    container->SetCollide(true);
    container->GetCollisionModel()->SetEnvelope(collision_envelope);
    container->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(container.get(), ChVector<>(4, .5, 4), ChVector<>(0, -.5, 0));
    container->GetCollisionModel()->BuildModel();

    container->AddAsset(std::make_shared<ChColorAsset>(ChColor(0.4f, 0.4f, 0.2f)));

    auto box = std::shared_ptr<ChBody>(system.NewBody());
    box->SetMass(10);
    box->SetPos(ChVector<>(1, 2, 1));
    box->SetInertiaXX(ChVector<>(1, 1, 1));

    box->GetMaterialSurfaceNSC()->SetFriction(0.4f);

    box->SetCollide(true);
    box->GetCollisionModel()->SetEnvelope(collision_envelope);
    box->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(box.get(), ChVector<>(0.4, 0.2, 0.1));
    box->GetCollisionModel()->BuildModel();

    box->AddAsset(std::make_shared<ChColorAsset>(ChColor(0.2f, 0.3f, 0.4f)));

    system.AddBody(box);

#ifdef CHRONO_OPENGL
    // -------------------------------
    // Create the visualization window
    // -------------------------------

    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Rolling test", &system);
    gl_window.SetCamera(ChVector<>(6, 6, 10), ChVector<>(0, 0, 0), ChVector<>(0, 1, 0), 0.05f);
    gl_window.SetRenderMode(opengl::WIREFRAME);
#endif

    // ---------------
    // Simulate system
    // ---------------

    ContactManager cmanager(box);

    while (system.GetChTime() < time_end) {
        // Process contacts
        std::cout << system.GetChTime() << "  " << system.GetNcontacts() << std::endl;
        ReportContacts(&system, box->GetId());

        // Advance dynamics
        system.DoStepDynamics(time_step);

#ifdef CHRONO_OPENGL
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        if (gl_window.Active()) {
            gl_window.Render();
        } else {
            return 1;
        }
#endif
    }

    return 0;
}