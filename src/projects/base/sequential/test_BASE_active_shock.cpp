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
// Simple example demonstrating the use of a ChLinkSpringCB with states.
//
//
// =============================================================================

#include <cstdio>

#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/core/ChLog.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"

#include "chrono_irrlicht/ChIrrApp.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

// =============================================================================

class ShockForce : public ChLinkSpringCB::ForceFunctor {
  public:
    virtual double operator()(double time,          // current time
                              double rest_length,   // undeformed length
                              double length,        // current length
                              double vel,           // current velocity (positive when extending)
                              ChLinkSpringCB* link  // back-pointer to associated link
                              ) override {
        // Damper force is the value oif the 2nd state.
        // Flip the sign to make the force acting against velocity
        return -link->GetStates()(1);
    }
};

class ShockODE : public ChLinkSpringCB::ODE {
  public:
    ShockODE() {
        // Setup damper tables for high frequency signals
        m_hf_damper_table.AddPoint(-5, -11936.925);
        m_hf_damper_table.AddPoint(-3, -11368.5);
        m_hf_damper_table.AddPoint(-0.33, -10335);
        m_hf_damper_table.AddPoint(-0.22, -8376);
        m_hf_damper_table.AddPoint(-0.13, -5789);
        m_hf_damper_table.AddPoint(-0.05, -2672);
        m_hf_damper_table.AddPoint(-0.02, -654);
        m_hf_damper_table.AddPoint(0, 0);
        m_hf_damper_table.AddPoint(0.02, 652);
        m_hf_damper_table.AddPoint(0.05, 1649);
        m_hf_damper_table.AddPoint(0.13, 2975);
        m_hf_damper_table.AddPoint(0.22, 4718);
        m_hf_damper_table.AddPoint(0.33, 7496);
        m_hf_damper_table.AddPoint(3, 8245.6);
        m_hf_damper_table.AddPoint(5, 8657.88);

        // Setup damper tables for low frequency signals
        m_lf_damper_table.AddPoint(-5, -12183.06833);
        m_lf_damper_table.AddPoint(-3, -11602.92222);
        m_lf_damper_table.AddPoint(-0.33, -10548);
        m_lf_damper_table.AddPoint(-0.22, -8620);
        m_lf_damper_table.AddPoint(-0.13, -6669);
        m_lf_damper_table.AddPoint(-0.05, -2935);
        m_lf_damper_table.AddPoint(-0.02, -986);
        m_lf_damper_table.AddPoint(0, 0);
        m_lf_damper_table.AddPoint(0.02, 766);
        m_lf_damper_table.AddPoint(0.05, 3621);
        m_lf_damper_table.AddPoint(0.13, 12628);
        m_lf_damper_table.AddPoint(0.22, 14045);
        m_lf_damper_table.AddPoint(0.33, 15444);
        m_lf_damper_table.AddPoint(3, 16988.27778);
        m_lf_damper_table.AddPoint(5, 17837.69167);
    }

    virtual int GetNumStates() const override { return 2; }
    virtual void SetInitialConditions(ChVectorDynamic<>& states,  // output vector containig initial conditions
                                      ChLinkSpringCB* link        // back-pointer to associated link
                                      ) override {
        // we start with zero velocity and zero force
        states(0) = 0;
        states(1) = 0;
    }

    virtual void CalculateRHS(double time,
                              const ChVectorDynamic<>& states,  // current states
                              ChVectorDynamic<>& rhs,           // output vector containing the ODE right-hand side
                              ChLinkSpringCB* link              // back-pointer to associated link
                              ) override {
        // ODE1
        // y_dot0 = (u0 - y0)/T0;
        // u0 is damper velocity vel = input
        // y0 = delayed damper velocity v_del = stage(0) = output
        const double T0 = 0.04;
        const double T1 = 1.02e-3;
        double vel = link->GetDist_dt();
        rhs(0) = (vel - states(0)) / T0;
        double vel_delayed = states(0);
        double vel_min = std::min(vel, vel_delayed);

        double force_hf = m_hf_damper_table.Get_y(vel);
        double force_lf = m_lf_damper_table.Get_y(vel_min);
        double force1 = 0.0;
        if (vel > 0.0) {
            force1 = force_hf + force_lf;
        } else {
            force1 = force_hf;
        }

        // ODE2
        // y_dot1 = (u1 - y1)/T1;
        // u1 = damper force1 = input
        // y1 = delayed damper force = stage(1) = output
        rhs(1) = (force1 - states(1)) / T1;
    }

  private:
    ChFunction_Recorder m_hf_damper_table;
    ChFunction_Recorder m_lf_damper_table;
};

// =============================================================================

class Actuation : public ChFunction {
  public:
    Actuation() : m_period(0.75), m_width(0.01), m_start(0.5) {}
    virtual Actuation* Clone() const override { return new Actuation(*this); }

    virtual void Update(double x) override {
        if (x > m_start + m_period) {
            m_start += m_period;
        }
    }

    virtual double Get_y(double x) const override {
        if (x > m_start && x < m_start + m_width)
            return 2;
        return 0;
    }

  private:
    double m_period;
    double m_width;
    double m_start;
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

    // Create a body suspended through a ChLinkSpringCB
    auto body = chrono_types::make_shared<ChBody>();
    system.AddBody(body);
    body->SetPos(ChVector<>(0, -3, 0));
    body->SetBodyFixed(false);
    body->SetCollide(false);
    body->SetMass(500);
    body->SetInertiaXX(ChVector<>(1, 1, 1));

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().SetLengths(ChVector<>(1, 1, 1));
    body->AddAsset(box);
    body->AddAsset(chrono_types::make_shared<ChColorAsset>(0.6f, 0, 0));

    // Create an actuator to control body speed (along Y direction)
    auto x2y = Q_from_AngZ(-CH_C_PI_2);
    auto speed_fun = chrono_types::make_shared<Actuation>();
    auto motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
    system.AddLink(motor);
    motor->Initialize(ground, body, ChFrame<>(body->GetPos(), x2y));
    motor->SetSpeedFunction(speed_fun);

    // Create the shock between body and ground. The end points are specified in the body relative frames.
    ShockForce force;
    ShockODE rhs;

    auto spring = chrono_types::make_shared<ChLinkSpringCB>();
    spring->Initialize(body, ground, true, ChVector<>(0, 0, 0), ChVector<>(0, 0, 0));
    spring->RegisterForceFunctor(&force);
    spring->RegisterODE(&rhs);
    system.AddLink(spring);
    spring->AddAsset(chrono_types::make_shared<ChColorAsset>(0.5f, 0.5f, 0.5f));
	spring->AddAsset(chrono_types::make_shared<ChPointPointSegment>());

    // Create the Irrlicht application
    ChIrrApp application(&system, L"Active spring test", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 0, 6));
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Create output directory and log file
    const std::string out_dir = GetChronoOutputPath() + "DEMO_ACTIVE_SHOCK";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    std::string logfile = out_dir + "/log.dat";
    ChStreamOutAsciiFile log(logfile.c_str());

    // Modify integrator
    ////system.SetTimestepperType(ChTimestepper::Type::HHT);
    ////auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
    ////integrator->SetAlpha(0);
    ////integrator->SetMaxiters(20);
    ////integrator->SetAbsTolerances(1e-6);

    // Simulation loop
    application.SetTimestep(1e-4);
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();

        double vel = body->GetPos_dt().y();
        ChVectorDynamic<> state = spring->GetStates();
        log << system.GetChTime() << ", " << vel << ", " << state(0) << ", " << state(1) << "\n";
    }

#ifdef CHRONO_POSTPROCESS
    // Plot results
    std::string gplfile = out_dir + "/tmp.gpl";
    postprocess::ChGnuPlot mplot(gplfile.c_str());

    mplot.SetTitle("ODE solution");
    mplot.SetLabelX("t");
    mplot.SetLabelY("y");
    //mplot.SetCommand("set ytics nomirror");
    //mplot.SetCommand("set y2range [-1:1]");
    //mplot.SetCommand("set y2tics -1, 0.25");
    mplot.Plot(logfile.c_str(), 1, 2, "velocity", " with lines lw 2 axis x1y1");
    mplot.Plot(logfile.c_str(), 1, 3, "state 1", " with lines lw 2 axis x1y1");
    mplot.Plot(logfile.c_str(), 1, 4, "state 2", " with lines lw 2 axis x1y2");
#endif

    return 0;
}
