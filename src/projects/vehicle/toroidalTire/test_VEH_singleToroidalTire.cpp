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
// Quarter-vehicle toroidal ANCF or Reissner tire test rig.
// The rig mechanism consists of a chassis constrained to only move in a vertical
// plane and a wheel body connected to the chassis through a revolute joint.
//
// The tire can be modeled either with ANCF 4-node shells or Reissner shells.
//
// The integrator is HHT. The solver can be MINRES or PARDISO.
//
// The coordinate frame respects the ISO standard adopted in Chrono::Vehicle:
// right-handed frame with X pointing towards the front, Y to the left, and Z up
//
// =============================================================================
////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono_mkl/ChSolverMKL.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFToroidalTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ReissnerToroidalTire.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::irrlicht;
using namespace irr;

// =============================================================================

// FEA shell type
enum class FEAType { ANCF, REISSNER };
FEAType fea_type = FEAType::ANCF;

// Solver settings (MINRES or PARDISO)
ChSolver::Type solver_type = ChSolver::Type::PARDISO;

double step_size = 1e-3;

// =============================================================================
// Contact reporter class

class MyContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    MyContactReporter(std::shared_ptr<ChBody> ground) : m_ground(ground) {}

  private:
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& react_forces,
                                 const ChVector<>& react_torques,
                                 ChContactable* objA,
                                 ChContactable* objB) override {
        ChVector<> force = plane_coord * react_forces;
        ChVector<> point = (objA == m_ground.get()) ? pA : pB;
        std::cout << "---  " << distance << std::endl;
        std::cout << "     " << point.x() << "  " << point.y() << "  " << point.z() << std::endl;
        std::cout << "     " << force.x() << "  " << force.y() << "  " << force.z() << std::endl;

        return true;
    }

    std::shared_ptr<ChBody> m_ground;
};

// =============================================================================
// Custom wheel object (zero mass and inertia)

class MyWheel : public ChWheel {
  public:
    MyWheel() : ChWheel("my_wheel") {}
    virtual double GetMass() const override { return 0; }
    virtual ChVector<> GetInertia() const override { return ChVector<>(0); }
    virtual double GetRadius() const override { return 0.2; }
    virtual double GetWidth() const override { return 0.1; }
};

// =============================================================================


int main(int argc, char* argv[]) {
    // Create the mechanical system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<>(0.0, 0.0, -9.8));

    // Create the quarter-vehicle chassis
    auto chassis = chrono_types::make_shared<ChBody>();
    system.AddBody(chassis);
    chassis->SetIdentifier(1);
    chassis->SetName("chassis");
    chassis->SetBodyFixed(false);
    chassis->SetCollide(false);
    chassis->SetMass(500);
    chassis->SetInertiaXX(ChVector<>(1, 1, 1));
    chassis->SetPos(ChVector<>(0, 0, 0));
    chassis->SetRot(ChQuaternion<>(1, 0, 0, 0));

    {
        auto boxH = chrono_types::make_shared<ChBoxShape>();
        boxH->GetBoxGeometry().SetLengths(ChVector<>(2, 0.02, 0.02));
        chassis->AddAsset(boxH);
        auto boxV = chrono_types::make_shared<ChBoxShape>();
        boxV->GetBoxGeometry().SetLengths(ChVector<>(0.02, 0.02, 2));
        chassis->AddAsset(boxV);
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().rad = 0.05;
        cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0.25, 0);
        cyl->GetCylinderGeometry().p2 = ChVector<>(0, -0.25, 0);
        chassis->AddAsset(cyl);
        auto color = chrono_types::make_shared<ChColorAsset>(0.4f, 0.5f, 0.6f);
        chassis->AddAsset(color);
    }

    // Create the spindle body
    auto spindle = chrono_types::make_shared<ChBody>();
    system.AddBody(spindle);
    spindle->SetIdentifier(2);
    spindle->SetName("wheel");
    spindle->SetBodyFixed(false);
    spindle->SetCollide(false);
    spindle->SetMass(40);
    spindle->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));
    spindle->SetPos(ChVector<>(0, 0, 0));
    spindle->SetRot(ChQuaternion<>(1, 0, 0, 0));

    // Create the wheel
    auto wheel = chrono_types::make_shared<MyWheel>();
    wheel->Initialize(spindle, LEFT);
    wheel->SetVisualizationType(VisualizationType::PRIMITIVES);

    // Create the tire
    std::shared_ptr<ChDeformableTire> tire;
    switch (fea_type) {
        case FEAType::ANCF: {
            auto tire_ancf = chrono_types::make_shared<ANCFToroidalTire>("ANCF_Tire");
            tire_ancf->SetDivCircumference(30);
            tire_ancf->SetDivWidth(6);
            tire = tire_ancf;
            break;
        }
        case FEAType::REISSNER: {
            auto tire_reissner = chrono_types::make_shared<ReissnerToroidalTire>("Reissner_Tire");
            tire_reissner->SetDivCircumference(30);
            tire_reissner->SetDivWidth(6);
            tire = tire_reissner;
            break;
        }
    }

    tire->EnablePressure(true);
    tire->EnableContact(true);
    tire->EnableRimConnection(true);
    //tire->SetContactSurfaceType(ChDeformableTire::TRIANGLE_MESH);

    std::static_pointer_cast<ChTire>(tire)->Initialize(wheel);
    tire->SetVisualizationType(VisualizationType::MESH);
    wheel->GetTire() = tire;

    // Create the terrain
    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    double terrain_height = -tire->GetRadius() - 0.01;
    auto terrain = chrono_types::make_shared<RigidTerrain>(&system);
    auto patch = terrain->AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 100, 2);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 4);
    terrain->Initialize();

    // Connect chassis to ground through a plane-plane joint.
    // The normal to the common plane is along the y global axis.
    auto plane_plane = chrono_types::make_shared<ChLinkLockPlanePlane>();
    system.AddLink(plane_plane);
    plane_plane->SetName("plane_plane");
    plane_plane->Initialize(patch->GetGroundBody(), chassis, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)));

    // Connect wheel to chassis through a revolute joint.
    // The axis of rotation is along the y global axis.
    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    system.AddLink(revolute);
    revolute->SetName("revolute");
    revolute->Initialize(chassis, spindle, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)));

    // Solver and integrator settings
    switch (solver_type) {
        case ChSolver::Type::MINRES: {
            GetLog() << "Using MINRES solver\n";
            auto minres_solver = chrono_types::make_shared<ChSolverMINRES>();
            minres_solver->SetMaxIterations(1000);
            minres_solver->EnableDiagonalPreconditioner(true);
            minres_solver->SetTolerance(1e-12);
            minres_solver->SetVerbose(true);
            system.SetSolver(minres_solver);
            break;
        }
        case ChSolver::Type::PARDISO: {
            GetLog() << "Using MKL solver\n";
            auto mkl_solver = chrono_types::make_shared<ChSolverMKL>();
            mkl_solver->UseSparsityPatternLearner(true);
            mkl_solver->LockSparsityPattern(true);
            system.SetSolver(mkl_solver);
            break;
        }
    }

    system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(20);
    integrator->SetAbsTolerances(5e-05, 5e-03);
    integrator->SetMode(ChTimestepperHHT::ACCELERATION);
    integrator->SetScaling(true);
    integrator->SetVerbose(true);

    // Create the Irrlicht app
    ChIrrApp app(&system, L"Toroidal Tire Test", core::dimension2d<u32>(800, 600), false, true);
    app.AddTypicalLogo();
    app.AddTypicalSky();
    app.AddTypicalLights(irr::core::vector3df(-130.f, -130.f, 50.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.AddTypicalCamera(core::vector3df(0, -1.5, 0.2f), core::vector3df(0, 0, 0));

    app.AssetBindAll();
    app.AssetUpdateAll();

    // Perform the simulation
    auto reporter = chrono_types::make_shared<MyContactReporter>(patch->GetGroundBody());

    app.SetTimestep(step_size);

    while (app.GetDevice()->run()) {
        app.BeginScene();
        app.DrawAll();
        app.EndScene();

        app.DoStep();

        std::cout << "Time: " << system.GetChTime() << "  Wheel center height: " << wheel->GetPos().z() << std::endl;
        // Report tire-terrain contacts
        system.GetContactContainer()->ReportAllContacts(reporter);
    }
}
