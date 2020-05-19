
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
#include "chrono/assets/ChColorAsset.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;

bool report_contacts = true;
bool report_aabb = true;

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
                                 const double& eff_radius,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        printf("  ------------\n");
        if (modA == m_box.get()) {
            printf("  box:    %6.3f  %6.3f  %6.3f\n", pA.x(), pA.y(), pA.z());
            printf("  other:  %6.3f  %6.3f  %6.3f\n", pB.x(), pB.y(), pB.z());
        } else if (modB == m_box.get()) {
            printf("  other:  %6.3f  %6.3f  %6.3f\n", pA.x(), pA.y(), pA.z());
            printf("  box:    %6.3f  %6.3f  %6.3f\n", pB.x(), pB.y(), pB.z());
        }
        ChVector<> nrm = plane_coord.Get_A_Xaxis();
        printf("  dist:   %6.4f\n", distance);
        printf("  normal: %6.3f  %6.3f  %6.3f\n", nrm.x(), nrm.y(), nrm.z());

        return true;
    }

    std::shared_ptr<ChBody> m_box;
};

// -------------------------------------------------------
// Access contact information directly in the data manager
// -------------------------------------------------------
void ReportContacts(ChSystemParallel& system, unsigned int id) {
    printf("  ------------\n");
    printf("  Total number contacts: %d\n", system.GetNcontacts());

    auto& bb = system.data_manager->host_data.bids_rigid_rigid;
    auto& p1 = system.data_manager->host_data.cpta_rigid_rigid;
    auto& p2 = system.data_manager->host_data.cptb_rigid_rigid;

    for (uint i = 0; i < system.data_manager->num_rigid_contacts; i++) {
        // IDs of bodies in contact
        int b1 = bb[i].x;
        int b2 = bb[i].y;

        if (id == b1) {
            printf("  %6.3f  %6.3f  %6.3f\n", p1[i].x, p1[i].y, p1[i].z);
        } else if (id == b2) {
            printf("   %6.3f  %6.3f  %6.3f\n", p2[i].x, p2[i].y, p2[i].z);
        }
    }
}

// -------------------------------------------------------
// Access AABB information directly in the data manager
// -------------------------------------------------------
void ReportShapeAABB(ChSystemParallel& system) {
    printf("  ------------\n");

    auto& aabb_min = system.data_manager->host_data.aabb_min;
    auto& aabb_max = system.data_manager->host_data.aabb_max;
    auto& id_rigid = system.data_manager->shape_data.id_rigid;
    auto& offset = system.data_manager->measures.collision.global_origin;

    printf("  AABB offset: %6.3f %6.3f %6.3f\n", offset.x, offset.y, offset.z);
    printf("  Number rigid shapes: %d\n", system.data_manager->num_rigid_shapes);

    for (uint i = 0; i < system.data_manager->num_rigid_shapes; i++) {
        auto min = aabb_min[i] + offset;
        auto max = aabb_max[i] + offset;
        printf("  shape %d (on body %d)   AABB (%6.3f %6.3f %6.3f) - (%6.3f %6.3f %6.3f)\n",  //
               i, id_rigid[i], min.x, min.y, min.z, max.x, max.y, max.z);
    }
}

// -------------------------------------------------------
// Report body AABB
// -------------------------------------------------------
void ReportBodyAABB(ChSystemParallel& system) {
    printf("  ------------\n");
    printf("  Number rigid bodies: %d\n", system.GetNbodies());

    ChVector<> min;
    ChVector<> max;
    for (auto b : system.Get_bodylist()) {
        b->GetCollisionModel()->GetAABB(min, max);
        printf("  body %d   AABB (%6.3f %6.3f %6.3f) - (%6.3f %6.3f %6.3f)\n",  //
               b->GetId(), min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
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

    // Shared contact material
    auto contact_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    contact_mat->SetFriction(0.4f);

    // ----------
    // Add bodies
    // ----------

    auto container = std::shared_ptr<ChBody>(system.NewBody());
    system.Add(container);
    container->SetPos(ChVector<>(0, 0, 0));
    container->SetBodyFixed(true);
    container->SetIdentifier(-1);

    container->SetCollide(true);
    container->GetCollisionModel()->SetEnvelope(collision_envelope);
    container->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(container.get(), contact_mat, ChVector<>(4, .5, 4), ChVector<>(0, -.5, 0));
    container->GetCollisionModel()->BuildModel();

    container->AddAsset(chrono_types::make_shared<ChColorAsset>(ChColor(0.4f, 0.4f, 0.2f)));

    auto box = std::shared_ptr<ChBody>(system.NewBody());
    box->SetMass(10);
    box->SetPos(ChVector<>(1, 2, 1));
    box->SetInertiaXX(ChVector<>(1, 1, 1));

    box->SetCollide(true);
    box->GetCollisionModel()->SetEnvelope(collision_envelope);
    box->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(box.get(), contact_mat, ChVector<>(0.4, 0.2, 0.1));
    box->GetCollisionModel()->BuildModel();

    box->AddAsset(chrono_types::make_shared<ChColorAsset>(ChColor(0.2f, 0.3f, 0.4f)));

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

    auto cmanager = chrono_types::make_shared<ContactManager>(box);

    while (system.GetChTime() < time_end) {
        // Advance dynamics
        system.DoStepDynamics(time_step);

        std::cout << "\nTime: " << system.GetChTime() << std::endl;

        // Process contacts
        if (report_contacts && system.GetNcontacts() > 0) {
            ReportContacts(system, box->GetId());
            system.GetContactContainer()->ReportAllContacts(cmanager);
        }

        // Display AABB information
        if (report_aabb) {
            ReportShapeAABB(system);
            system.CalculateBodyAABB();
            ReportBodyAABB(system);
        }

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