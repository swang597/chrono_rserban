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
// Chrono test for box-box collision
//
// The global reference frame has Y up.
//
// =============================================================================

#include <cstdio>
#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono/physics/ChSystemNSC.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChIrrApp.h"
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

// --------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // ----------------
    // Parameters
    // ----------------

    double time_end = 4;
    double time_step = 1e-3;

    double tolerance = 0;
    double contact_recovery_speed = 1e8;
    double collision_envelope = .001;

    // -----------------
    // Create the system
    // -----------------

    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, -10, 0));

    // Set solver settings
    system.SetMaxItersSolverSpeed(100);
    system.SetMaxPenetrationRecoverySpeed(contact_recovery_speed);
    system.SetTol(tolerance);
    system.SetTolForce(tolerance);
    system.SetSolverType(ChSolver::Type::APGD);

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

#ifdef CHRONO_IRRLICHT
    // -------------------------------
    // Create the visualization window
    // -------------------------------

    irrlicht::ChIrrApp application(&system, L"Rolling test", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    irrlicht::ChIrrWizard::add_typical_Logo(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Lights(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(6, 6, -10));

    application.AssetBindAll();
    application.AssetUpdateAll();
#endif

    // ---------------
    // Simulate system
    // ---------------

    ContactManager cmanager(box);

    while (system.GetChTime() < time_end) {
        // Process contacts
        std::cout << system.GetChTime() << "  " << system.GetNcontacts() << std::endl;
        system.GetContactContainer()->ReportAllContacts(&cmanager);

        // Advance dynamics
        system.DoStepDynamics(time_step);

#ifdef CHRONO_IRRLICHT
        // Render scene
        if (application.GetDevice()->run()) {
            application.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            application.DrawAll();
            application.EndScene();
        } else {
            return 1;
        }
#endif
    }

    return 0;
}
