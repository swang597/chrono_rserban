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
// Simple example demonstrating the use of a spring with force defined through
// a functor class.
//
// The spring also carries additional internal dynamics.  In this demo, these
// are completely coupled from the connected bodies.
//
// =============================================================================

#include <cstdio>

#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/core/ChLog.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/solver/ChSolverMINRES.h"

#include "chrono_mkl/ChSolverMKL.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

// =============================================================================

enum class SolverType {
    MINRES,  // MINRES iterative solver
    PARDISO  // Pardiso sparse direct solver
};
SolverType solver_type = SolverType::PARDISO;

enum class IntegratorType {
    HHT,  // HHT
    EI,   // Euler implicit
    EIL   // euler implicit linearized
};
IntegratorType integrator_type = IntegratorType::HHT;

bool use_jacobians = true;

double step_size = 1e-4;

// =============================================================================

double mass = 1;
double rest_length = 1.5;
double spring_coef = 1e3;
double critical_damping = 2 * std::sqrt(mass * spring_coef);
double damping_coef = 0.5 * critical_damping;

// =============================================================================

// Functor class implementing the force for a ChLinkSpringCB.
class MySpringForce : public ChLinkSpringCB::ForceFunctor {
    virtual double operator()(double time,          // current time
                              double rest_length,   // undeformed length
                              double length,        // current length
                              double vel,           // current velocity (positive when extending)
                              ChLinkSpringCB* link  // back-pointer to associated link
                              ) override {
        // Access current states.
        ////ChVectorDynamic<> states = link->GetStates();
        ////std::cout << "t = " << time << "  " << states(0) << " " << states(1) << std::endl;

        double force = -spring_coef * (length - rest_length) - damping_coef * vel;
        return force;
    }
};

// Functor class implementing the ODE right-hand side for a ChLinkSpringCB.
class MySpringRHS : public ChLinkSpringCB::ODE {
    virtual int GetNumStates() const override { return 2; }
    virtual void SetInitialConditions(ChVectorDynamic<>& states,  ///< output vector containig initial conditions
                                      ChLinkSpringCB* link        ///< back-pointer to associated link
                                      ) override {
        states(0) = 1;
        states(1) = 0;
    }
    virtual void CalculateRHS(double time,
                              const ChVectorDynamic<>& states,  ///< current states
                              ChVectorDynamic<>& rhs,           ///< output vector containing the ODE right-hand side
                              ChLinkSpringCB* link              ///< back-pointer to associated link
                              ) override {
        rhs(0) = states(0);
        rhs(1) = std::cos(time);
    }
};

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, 0, 0));

    // Create the ground body with two visualization spheres
    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);

    auto sph = chrono_types::make_shared<ChSphereShape>();
    sph->GetSphereGeometry().rad = 0.1;
    sph->Pos = ChVector<>(0, 0, 0);
    ground->AddAsset(sph);

    // Create a body suspended through a ChLinkSpring
    auto body = chrono_types::make_shared<ChBody>();
    system.AddBody(body);
    body->SetPos(ChVector<>(0, -3, 0));
    body->SetBodyFixed(false);
    body->SetCollide(false);
    body->SetMass(mass);
    body->SetInertiaXX(ChVector<>(1, 1, 1));

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().SetLengths(ChVector<>(1, 1, 1));
    body->AddAsset(box);
    body->AddAsset(chrono_types::make_shared<ChColorAsset>(0.6f, 0, 0));

    // Create the spring between body and ground. The spring end points are specified in the body relative frames.
    MySpringForce force;
    MySpringRHS rhs;

    auto spring = chrono_types::make_shared<ChLinkSpringCB>();
    spring->Initialize(body, ground, true, ChVector<>(0, 0, 0), ChVector<>(0, 0, 0), false, rest_length);
    ////spring->IsStiff(use_jacobians);
    spring->RegisterForceFunctor(&force);
    spring->RegisterODE(&rhs);
    system.AddLink(spring);
    spring->AddAsset(chrono_types::make_shared<ChColorAsset>(0.5f, 0.5f, 0.5f));
    spring->AddAsset(chrono_types::make_shared<ChPointPointSpring>(0.05, 80, 15));

    // Create the Irrlicht application
    ChIrrApp application(&system, L"Active spring test", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 0, 6));
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Create output directory and log file
    const std::string out_dir = GetChronoOutputPath() + "DEMO_ACTIVE_SPRING";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    std::string logfile = out_dir + "/log.dat";
    ChStreamOutAsciiFile log(logfile.c_str());

    std::string btitle("Body - ");

    // Modify integrator
    switch (integrator_type) {
        case IntegratorType::HHT: {
            btitle += "HHT - ";
            system.SetTimestepperType(ChTimestepper::Type::HHT);
            auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
            integrator->SetAlpha(-0.2);
            integrator->SetMaxiters(20);
            integrator->SetAbsTolerances(1e-6);
            break;
        }
        case IntegratorType::EI: {
            btitle += "EI - ";
            system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
            auto integrator = std::static_pointer_cast<ChTimestepperEulerImplicit>(system.GetTimestepper());
            integrator->SetMaxiters(1);
            break;
        }
        case IntegratorType::EIL: {
            btitle += "EIL - ";
            break;
        }
    }

    // Modify solver
    switch (solver_type) {
        case SolverType::MINRES: {
            btitle += "MINRES - ";
            system.SetSolverType(ChSolver::Type::MINRES);
            system.SetSolverWarmStarting(true);
            system.SetMaxItersSolverSpeed(200);
            system.SetMaxItersSolverStab(200);
            system.SetTolForce(1e-14);
            auto msolver = std::static_pointer_cast<ChSolverMINRES>(system.GetSolver());
            msolver->SetDiagonalPreconditioning(true);
            msolver->SetVerbose(false);
            break;
        }
        case SolverType::PARDISO: {
            btitle += "PARDISO - ";
            auto mkl_solver = chrono_types::make_shared<ChSolverMKL>();
            mkl_solver->LockSparsityPattern(false);
            system.SetSolver(mkl_solver);
            break;
        }
    }

    btitle += use_jacobians ? " Jac YES" : "Jac NO";

    // Simulation loop
    application.SetTimestep(step_size);
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();

        ChVectorDynamic<> state = spring->GetStates();
        log << system.GetChTime() << ", " << state(0) << ", " << state(1) << ", ";
        log << body->GetPos().y() << ", " << body->GetPos_dt().y();
        log << "\n";

        if (system.GetChTime() >= 0.4)
            break;
    }

#ifdef CHRONO_POSTPROCESS
    // Plot results
    std::string gplfile = out_dir + "/tmp.gpl";
    postprocess::ChGnuPlot mplot(gplfile.c_str());

    mplot.OutputWindow(0);
    mplot.SetGrid();
    mplot.SetTitle("ODE solution");
    mplot.SetLabelX("t");
    mplot.SetLabelY("y");
    mplot.SetCommand("set ytics nomirror");
    mplot.SetCommand("set y2range [-1:1]");
    mplot.SetCommand("set y2tics -1, 0.25");
    mplot.Plot(logfile.c_str(), 1, 2, "state 1", " with lines lw 2 axis x1y1");
    mplot.Plot(logfile.c_str(), 1, 3, "state 2", " with lines lw 2 axis x1y2");

    mplot.OutputWindow(1);
    mplot.SetGrid();
    mplot.SetTitle(btitle.c_str());
    mplot.SetLabelX("t");
    mplot.SetLabelY("y");
    mplot.SetCommand("set ytics nomirror");
    mplot.SetCommand("set y2range [-20:20]");
    mplot.SetCommand("set y2tics -20, 5");
    mplot.Plot(logfile.c_str(), 1, 4, "y", " with lines lw 2 axis x1y1");
    mplot.Plot(logfile.c_str(), 1, 5, "y'", " with lines lw 2 axis x1y2");

#endif

    return 0;
}
