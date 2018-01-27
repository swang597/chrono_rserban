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
// Test program for obtaining drawbar pull at specified longitudinal slip.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
// All units SI.
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChFileutils.h"
#include "chrono/motion_functions/ChFunction.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/wvp/WVP.h"

#define USE_IRRLICHT

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::wvp;

// =============================================================================

// Initial vehicle location and orientation
double terrainLength = 30;  /// 400;
ChVector<> initLoc(-terrainLength / 2 + 6, 0, 0.7);

// Simulation step size
double step_size = 1e-4;

// Simulation end time
double tend = 8;

// Time interval between two render frames
double output_step_size = 1.0 / 400;

// output directory
const std::string out_dir = "../WVP_DRAWBAR_PULL";
bool output = true;

// =============================================================================

int main(int argc, char* argv[]) {
    // Longitudinal slip value
    double slip = 0.0;
    if (argc > 1) {
        slip = atof(argv[1]);
    }

    // Imposed vehicle forward speed
    double vel = 1;

    // --------------
    // Create vehicle
    // --------------

    WVP wvp;
    wvp.SetChassisFixed(false);
    wvp.SetInitPosition(ChCoordsys<>(initLoc, QUNIT));
    wvp.SetInitFwdVel(vel);
    wvp.SetTireType(TireModelType::RIGID);
    wvp.SetTireStepSize(step_size);
    wvp.DisconnectPowertrain();
    wvp.Initialize();

    wvp.GetSystem()->Set_G_acc(ChVector<>(0, 0, -9.81));

    wvp.SetChassisVisualizationType(VisualizationType::NONE);
    wvp.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    wvp.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    wvp.SetWheelVisualizationType(VisualizationType::NONE);
    wvp.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    std::cout << "Vehicle mass:               " << wvp.GetVehicle().GetVehicleMass() << std::endl;
    std::cout << "Vehicle mass (with tires):  " << wvp.GetTotalMass() << std::endl;
    std::cout << "Tire radius:                " << wvp.GetTire(FRONT_LEFT)->GetRadius() << std::endl;

    // ---------------
    // Create test rig
    // ---------------

    auto ground = std::shared_ptr<ChBody>(wvp.GetSystem()->NewBody());
    ground->SetBodyFixed(true);
    ground->SetCollide(false);
    wvp.GetSystem()->AddBody(ground);
    {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().rad = 0.02;
        cyl->GetCylinderGeometry().p1 = ChVector<>(-terrainLength / 2, 0, initLoc.z());
        cyl->GetCylinderGeometry().p2 = ChVector<>(+terrainLength / 2, 0, initLoc.z());
        ground->AddAsset(cyl);
        auto col = std::make_shared<ChColorAsset>(0.2f, 0.3f, 0.6f);
        ground->AddAsset(col);
    }

    auto rig = std::shared_ptr<ChBody>(wvp.GetSystem()->NewBody());
    rig->SetPos(initLoc);
    rig->SetRot(QUNIT);
    rig->SetMass(100);
    rig->SetBodyFixed(false);
    rig->SetCollide(false);
    wvp.GetSystem()->AddBody(rig);
    {
        auto box = std::make_shared<ChBoxShape>();
        box->GetBoxGeometry().SetLengths(ChVector<>(1, 0.4, 0.4));
        rig->AddAsset(box);
    }

    auto prismatic1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(ground, rig, ChCoordsys<>(initLoc, Q_from_AngY(CH_C_PI_2)));
    wvp.GetSystem()->AddLink(prismatic1);

    auto prismatic2 = std::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(rig, wvp.GetChassisBody(), ChCoordsys<>(initLoc, QUNIT));
    wvp.GetSystem()->AddLink(prismatic2);

    // ----------------
    // Create actuators
    // ----------------

    // Calculate wheel angular velocity for given slip
    double radius = wvp.GetTire(FRONT_LEFT)->GetRadius();
    double omega = vel / (radius * (1 - slip));

    std::cout << "Slip: " << slip << "  Forward vel: " << vel << "  Wheel omg: " << omega << std::endl;

    // Impose forward vehicle velocity
    ////auto fun_vel = std::make_shared<ChFunction_Const>(vel);
    ////auto actuator = std::make_shared<ChLinkMotorLinearSpeed>();
    ////actuator->Initialize(rig, ground, ChFrame<>(initLoc, QUNIT));
    ////actuator->SetSpeedFunction(fun_vel);
    ////wvp.GetSystem()->AddLink(actuator);

    auto fun_vel = std::make_shared<ChFunction_Ramp>(0, vel);
    auto actuator = std::make_shared<ChLinkLinActuator>();
    actuator->Set_lin_offset(1);
    actuator->Set_dist_funct(fun_vel);
    actuator->Initialize(ground, rig, false, ChCoordsys<>(initLoc, QUNIT), ChCoordsys<>(initLoc + ChVector<>(1, 0, 0), QUNIT));
    wvp.GetSystem()->AddLink(actuator);

    // Impose wheel angular velocity
    std::vector<std::shared_ptr<ChLinkMotorRotationSpeed>> motors;
    auto fun_omg = std::make_shared<ChFunction_Const>(omega);
    for (int i = 0; i < 4; i++) {
        auto motor = std::make_shared<ChLinkMotorRotationSpeed>();
        auto wheel = wvp.GetVehicle().GetWheelBody(i);
        auto pos = wheel->GetPos();
        motor->Initialize(wvp.GetChassisBody(), wheel, ChFrame<>(pos, Q_from_AngX(CH_C_PI_2)));
        motor->SetSpeedFunction(fun_omg);
        wvp.GetSystem()->AddLink(motor);
        motors.push_back(motor);
    }

    // ------------------
    // Create the terrain
    // ------------------

    // Deformable terrain properties (LETE sand)
    double Kphi = 5301e3;
    double Kc = 102e3;
    double n = 0.793;
    double c = 1.3e3;
    double phi = 31.1;
    double K = 1.2e-2;
    double E_elastic = 2e8;
    double damping = 3e4;

    float mu = 0.8f;
    float restitution = 0.01f;
    float E = 2e7f;
    float nu = 0.3f;
    double depth = 10;
    double factor = 2;

    auto terrain = new SCMDeformableTerrain(wvp.GetSystem());
    terrain->SetPlane(ChCoordsys<>(VNULL, Q_from_AngX(CH_C_PI_2)));
    terrain->SetSoilParametersSCM(Kphi, Kc, n, c, phi, K, E_elastic, damping);
    terrain->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.15);
    terrain->SetAutomaticRefinement(true);
    terrain->SetAutomaticRefinementResolution(0.1);
    terrain->EnableMovingPatch(wvp.GetChassisBody(), ChVector<>(-2, 0, 0), 5, 3);
    terrain->Initialize(0.0, terrainLength, 5.0, terrainLength * factor, 5 * factor);

#ifdef USE_IRRLICHT
    // -------------------------------------
    // Create the vehicle Irrlicht interface
    // -------------------------------------

    ChWheeledVehicleIrrApp app(&wvp.GetVehicle(), &wvp.GetPowertrain(), L"WVP sequential test");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(-2.0, 0.0, 1.75), 6.0, 0.5);
    app.AssetBindAll();
    app.AssetUpdateAll();
#endif

    // -------------
    // Prepare output
    // -------------

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // CSV stream
    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil(output_step_size / step_size);

    // ---------------
    // Simulation loop
    // ---------------

    int step_number = 0;
    double time = 0;
    double total_timing = 0;

    while (wvp.GetSystem()->GetChTime() < tend) {
        if (wvp.GetVehicle().GetVehiclePos().x() > terrainLength / 2 - 1)
            break;

#ifdef USE_IRRLICHT
        if (!app.GetDevice()->run())
            break;

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();
#endif

        time = wvp.GetSystem()->GetChTime();

        // Collect output data from modules (for inter-module communication)
        double throttle_input = 0;
        double steering_input = 0;
        double braking_input = 0;

        // Update modules (process inputs from other modules)
        terrain->Synchronize(time);
        wvp.Synchronize(time, steering_input, braking_input, throttle_input, *terrain);
#ifdef USE_IRRLICHT
        app.Synchronize("", steering_input, throttle_input, braking_input);
#endif

        // Advance simulation for one timestep for all modules
        terrain->Advance(step_size);
        wvp.Advance(step_size);
#ifdef USE_IRRLICHT
        app.Advance(step_size);
#endif

        if (output && step_number % output_steps == 0) {
            csv << time;
            csv << wvp.GetVehicle().GetVehicleSpeed();
            for (int i = 0; i < 4; i++) {
                csv << wvp.GetVehicle().GetWheelOmega(i);
                csv << wvp.GetTire(i)->GetLongitudinalSlip();
            }
            csv << actuator->Get_react_force();
            csv << std::endl;
        }

        // Increment frame number
        step_number++;

        total_timing += wvp.GetSystem()->GetTimerStep();
    }

    std::cout << "Total execution time: " << total_timing << std::endl;

    if (output) {
        csv.write_to_file(out_dir + "/output_" + std::to_string(slip) + ".dat");
    }

    return 0;
}
