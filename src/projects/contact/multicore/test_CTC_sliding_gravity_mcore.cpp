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
//  This project simulates a box with a non-zero initial horizontal velocity
//  sliding across a horizontal plate. See Xiang et al. (2009) Test 0.
//  The purpose of this test is to validate the implementation of sliding
//  friction in ChIterativeSolverMulticoreSMC.
//
// =============================================================================

#include <iostream>
#include <fstream>

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::irrlicht;
using namespace irr;

class ContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    ContactReporter(std::shared_ptr<ChBody> body) : m_body(body) {}
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        const ChVector<>& p = m_body->GetPos();
        const ChVector<>& nrm = plane_coord.Get_A_Xaxis();
        std::cout << "  ---" << std::endl;
        std::cout << "  B:   " << p.x() << "  " << p.y() << "  " << p.z() << std::endl;
        std::cout << "  pA:  " << pA.x() << "  " << pA.y() << "  " << pA.z() << std::endl;
        std::cout << "  pB:  " << pB.x() << "  " << pB.y() << "  " << pB.z() << std::endl;
        std::cout << "  nrm: " << nrm.x() << "  " << nrm.y() << "  " << nrm.z() << std::endl;
        std::cout << "  penetration: " << distance << "  eff. radius: " << eff_radius << std::endl;
        std::cout << "  frc: " << cforce.x() << "  " << cforce.y() << "  " << cforce.z() << std::endl;
        std::cout << "  trq: " << ctorque.x() << "  " << ctorque.y() << "  " << ctorque.z() << std::endl;

        return true;
    }
    std::shared_ptr<ChBody> m_body;
};

std::shared_ptr<ChBody> AddBoxBody(int id,
                                   ChSystemMulticoreSMC* sys,
                                   std::shared_ptr<ChMaterialSurfaceSMC> mat,
                                   ChVector<> size,
                                   double mass,
                                   ChVector<> pos,
                                   bool fixed) {
    ChVector<> inertia((1.0 / 12.0) * mass * (pow(size.y(), 2) + pow(size.z(), 2)),
                       (1.0 / 12.0) * mass * (pow(size.x(), 2) + pow(size.z(), 2)),
                       (1.0 / 12.0) * mass * (pow(size.x(), 2) + pow(size.y(), 2)));

    // Create container. Set body parameters and container collision model
    auto body = chrono_types::make_shared<ChBody>(ChCollisionSystemType::CHRONO);
    body->SetIdentifier(id);
    body->SetMass(mass);
    body->SetInertiaXX(inertia);
    body->SetPos(pos);
    body->SetBodyFixed(fixed);
    body->SetCollide(true);

    body->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(body.get(), mat, size / 2);
    body->GetCollisionModel()->BuildModel();
    body->GetVisualShape(0)->SetColor(ChColor(0.55f, 0.57f, 0.67f));

    // Return a pointer to the wall object
    sys->AddBody(body);
    return body;
}

std::shared_ptr<ChBody> AddSphereBody(int id,
                                      ChSystemMulticoreSMC* sys,
                                      std::shared_ptr<ChMaterialSurfaceSMC> mat,
                                      ChVector<> size,
                                      double mass,
                                      ChVector<> pos,
                                      bool fixed) {
    ChVector<> inertia((1.0 / 12.0) * mass * (pow(size.y(), 2) + pow(size.z(), 2)),
                       (1.0 / 12.0) * mass * (pow(size.x(), 2) + pow(size.z(), 2)),
                       (1.0 / 12.0) * mass * (pow(size.x(), 2) + pow(size.y(), 2)));

    // Create container. Set body parameters and container collision model
    auto body = chrono_types::make_shared<ChBody>(ChCollisionSystemType::CHRONO);
    body->SetIdentifier(id);
    body->SetMass(mass);
    body->SetInertiaXX(inertia);
    body->SetPos(pos);
    body->SetBodyFixed(fixed);
    body->SetCollide(true);

    auto hsize = size / 2;

    // collision
    body->GetCollisionModel()->ClearModel();
    body->GetCollisionModel()->AddSphere(mat, 0.1, ChVector<>(+hsize.x(), -hsize.y() + 0.1, +hsize.z()));
    body->GetCollisionModel()->AddSphere(mat, 0.1, ChVector<>(+hsize.x(), -hsize.y() + 0.1, -hsize.z()));
    body->GetCollisionModel()->AddSphere(mat, 0.1, ChVector<>(-hsize.x(), -hsize.y() + 0.1, +hsize.z()));
    body->GetCollisionModel()->AddSphere(mat, 0.1, ChVector<>(-hsize.x(), -hsize.y() + 0.1, -hsize.z()));
    body->GetCollisionModel()->BuildModel();

    // visualization
    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = hsize;
    box->SetColor(ChColor(0.55f, 0.57f, 0.67f));
    body->AddVisualShape(box);

    // Return a pointer to the wall object
    sys->AddBody(body);
    return body;
}

double CalcKE(ChSystemMulticoreSMC* sys) {
    const std::shared_ptr<ChBody> body = sys->Get_bodylist().at(1);

    ChVector<> eng_trn = 0.5 * body->GetMass() * body->GetPos_dt() * body->GetPos_dt();
    ChVector<> eng_rot = 0.5 * body->GetInertiaXX() * body->GetWvel_par() * body->GetWvel_par();

    double KE_trn = eng_trn.x() + eng_trn.y() + eng_trn.z();
    double KE_rot = eng_rot.x() + eng_rot.y() + eng_rot.z();
    double KE_tot = KE_trn + KE_rot;

    return KE_tot;
}

bool CalcAverageKE(ChSystemMulticoreSMC* sys, const double& threshold) {
    // Calculate average KE
    double KE_trn = 0;
    double KE_rot = 0;

    for (int i = 0; i < sys->Get_bodylist().size(); ++i) {
        const std::shared_ptr<ChBody> body = sys->Get_bodylist().at(i);

        ChVector<> eng_trn = 0.5 * body->GetMass() * body->GetPos_dt() * body->GetPos_dt();
        ChVector<> eng_rot = 0.5 * body->GetInertiaXX() * body->GetWvel_par() * body->GetWvel_par();

        KE_trn += eng_trn.x() + eng_trn.y() + eng_trn.z();
        KE_rot += eng_rot.x() + eng_rot.y() + eng_rot.z();
    }

    double KE_trn_avg = KE_trn / sys->Get_bodylist().size();
    double KE_rot_avg = KE_rot / sys->Get_bodylist().size();
    double KE_tot_avg = KE_trn_avg + KE_rot_avg;

    // Return true if the calc falls below the given threshold
    if (KE_tot_avg < threshold)
        return true;
    return false;
}

// =============================================================================

int main(int argc, char* argv[]) {
    // Print the sim set - up parameters to userlog
    GetLog() << "\nCopyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << ".VCMS\n";

    // Execute test for each force model
    std::vector<ChSystemSMC::ContactForceModel> fmodels = {
        ChSystemSMC::ContactForceModel::Hooke, ChSystemSMC::ContactForceModel::Hertz,
        ChSystemSMC::ContactForceModel::PlainCoulomb, ChSystemSMC::ContactForceModel::Flores};

    for (int f = 0; f < fmodels.size(); ++f) {
        GetLog() << "\nModel #" << f << "\n";

        // Create a shared material to be used by the all bodies
        float y_modulus = 2.0e5f;  /// Default 2e5
        float p_ratio = 0.3f;      /// Default 0.3f
        float s_frict = 0.5f;      /// Usually in 0.1 range, rarely above. Default 0.6f
        float k_frict = 0.5f;      /// Default 0.6f
        float roll_frict = 0.0f;   /// Usually around 1E-3
        float spin_frict = 0.0f;   /// Usually around 1E-3
        float cor_in = 0.0f;       /// Default 0.4f
        float ad = 0.0f;           /// Magnitude of the adhesion in the Constant adhesion model
        float adDMT = 0.0f;        /// Magnitude of the adhesion in the DMT adhesion model
        float adSPerko = 0.0f;     /// Magnitude of the adhesion in the SPerko adhesion model

        auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
        mat->SetYoungModulus(y_modulus);
        mat->SetPoissonRatio(p_ratio);
        mat->SetSfriction(s_frict);
        mat->SetKfriction(k_frict);
        mat->SetRollingFriction(roll_frict);
        mat->SetSpinningFriction(spin_frict);
        mat->SetRestitution(cor_in);
        mat->SetAdhesion(ad);
        mat->SetAdhesionMultDMT(adDMT);
        mat->SetAdhesionSPerko(adSPerko);

        // Create a multicore SMC system and set the system parameters
        double time_step = 1.0E-5;
        ChVector<> gravity(0, -9.81, 0);

        ChSystemMulticoreSMC sys;
        sys.Set_G_acc(gravity);
        sys.GetSettings()->solver.max_iteration_bilateral = 100;
        sys.GetSettings()->solver.tolerance = 1e-3;
        sys.GetSettings()->solver.contact_force_model = fmodels[f];
        sys.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;
        sys.GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::OneStep;
        sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);
        sys.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
        sys.SetCollisionSystemType(ChCollisionSystemType::CHRONO);
        sys.SetNumThreads(2);

        // Add the wall to the system
        double wmass = 10.0;
        ChVector<> wsize(8, 1, 3);
        ChVector<> wpos(0, -wsize.y() / 2 - 0.5, 0);
        auto wall = AddBoxBody(-1, &sys, mat, wsize, wmass, wpos, true);

        // Add the block to the system
        double bmass = 1.0;
        ChVector<> bsize(0.5, 0.5, 0.5);
        ChVector<> bpos(0, bsize.y() / 2 - 0.49, 0);
        auto body = AddBoxBody(0, &sys, mat, bsize, bmass, bpos, false);
        ////auto body = AddSphereBody(0, &sys, mat, bsize, bmass, bpos, false);

        // Create the Irrlicht visualization.
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("Sliding box SMC test");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddTypicalLights();
        vis->AddCamera(ChVector<>(0, 0, -7.5));
        sys.SetVisualSystem(vis);

        // Callback for contact reporting
        auto creporter = chrono_types::make_shared<ContactReporter>(body);

        // Let the block settle of the plate before giving it a push
        double time_end = 2.0;
        while (sys.GetChTime() < time_end) {
            vis->BeginScene();
            vis->DrawAll();
            vis->EndScene();
            sys.DoStepDynamics(time_step);

            ////sys.GetContactContainer()->ReportAllContacts(creporter);

            if (CalcKE(&sys) < 1e-9) {
                GetLog() << "[settling] KE falls below threshold at t = " << sys.GetChTime() << "\n";
                break;
            }
        }

        // Give th block a horizontal push
        ChVector<> init_bv(5, 0, 0);
        body->SetPos_dt(init_bv);

        // Iterate through simulation. Calculate resultant forces and motion for each timestep
        time_end = sys.GetChTime() + 2.0;
        while (vis->Run()) {
            vis->BeginScene();
            vis->DrawAll();
            vis->EndScene();

            ////std::cout << "============= " << sys.GetChTime() << std::endl;
            sys.DoStepDynamics(time_step);
            ////sys.GetContactContainer()->ReportAllContacts(creporter);
            ////ChVector<> frc = body->GetContactForce();
            ////std::cout << "  ----------- " << std::endl;
            ////std::cout << frc.x() << "  " << frc.y() << "  " << frc.z() << std::endl;

            if (CalcKE(&sys) < 1e-9) {
                GetLog() << "[simulation] KE falls below threshold at t = " << sys.GetChTime() << "\n";
                break;
            }

            if (sys.GetChTime() > time_end) {
                GetLog() << "[simulation] KE still above threshold!\n";
                GetLog() << "Kinetic energy: " << CalcKE(&sys) << "\n";
                break;
            }
        }

        // Check results. The error should be >= 0.5% and the block's velocity should be < 1e-4
        double d_ref = abs((init_bv.Length() * init_bv.Length()) / (2 * s_frict * gravity.y()));
        double d_act = body->GetPos().x() - bpos.x();
        double d_err = abs((d_ref - d_act) / d_ref) * 100;

        GetLog() << "Error " << d_err << " %\n";
    }

    return 0;
}
