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
// Benchmark test for HMMWV acceleration test.
//
// =============================================================================

#include "chrono/utils/ChBenchmark.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

class HmmwvAccTest : public utils::ChBenchmarkTest {
public:
    HmmwvAccTest();
    ~HmmwvAccTest();

    ChSystem* GetSystem() override { return m_hmmwv->GetSystem(); }
    void ExecuteStep() override;

    void SimulateVis();

private:
    HMMWV_Full* m_hmmwv;
    RigidTerrain* m_terrain;
    ChPathFollowerDriver* m_driver;

    double m_step_veh;
    double m_step_tire;
};

HmmwvAccTest::HmmwvAccTest() : m_step_veh(1e-3), m_step_tire(1e-3) {
    PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;
    TireModelType tire_model = TireModelType::FIALA;
    DrivelineType drive_type = DrivelineType::AWD;
    double terrainLength = 500.0;

    // Create the HMMWV vehicle, set parameters, and initialize.
    m_hmmwv = new HMMWV_Full();
    m_hmmwv->SetContactMethod(ChContactMethod::SMC);
    m_hmmwv->SetChassisFixed(false);
    m_hmmwv->SetInitPosition(ChCoordsys<>(ChVector<>(-terrainLength / 2 + 5, 0, 0.7), ChQuaternion<>(1, 0, 0, 0)));
    m_hmmwv->SetPowertrainType(powertrain_model);
    m_hmmwv->SetDriveType(drive_type);
    m_hmmwv->SetTireType(tire_model);
    m_hmmwv->SetTireStepSize(m_step_tire);
    m_hmmwv->SetAerodynamicDrag(0.5, 5.0, 1.2);
    m_hmmwv->Initialize();

    m_hmmwv->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetWheelVisualizationType(VisualizationType::NONE);
    m_hmmwv->SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the terrain
    m_terrain = new RigidTerrain(m_hmmwv->GetSystem());
    auto patch_material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch_material->SetFriction(0.9f);
    patch_material->SetRestitution(0.01f);
    patch_material->SetYoungModulus(2e7f);
    auto patch = m_terrain->AddPatch(patch_material, ChCoordsys<>(ChVector<>(0, 0, -5), QUNIT), ChVector<>(terrainLength, 5, 10));
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    m_terrain->Initialize();

    // Create the straight path and the driver system
    auto path = StraightLinePath(ChVector<>(-terrainLength / 2, 0, 0.5), ChVector<>(terrainLength / 2, 0, 0.5), 1);
    m_driver = new ChPathFollowerDriver(m_hmmwv->GetVehicle(), path, "my_path", 1000.0);
    m_driver->GetSteeringController().SetLookAheadDistance(5.0);
    m_driver->GetSteeringController().SetGains(0.5, 0, 0);
    m_driver->GetSpeedController().SetGains(0.4, 0, 0);
    m_driver->Initialize();
}

HmmwvAccTest::~HmmwvAccTest() {
    delete m_hmmwv;
    delete m_terrain;
    delete m_driver;
}

void HmmwvAccTest::ExecuteStep() {
    double time = m_hmmwv->GetSystem()->GetChTime();

    // Driver inputs
    ChDriver::Inputs driver_inputs = m_driver->GetInputs();

    // Update modules (process inputs from other modules)
    m_driver->Synchronize(time);
    m_terrain->Synchronize(time);
    m_hmmwv->Synchronize(time, driver_inputs, *m_terrain);

    // Advance simulation for one timestep for all modules
    m_driver->Advance(m_step_veh);
    m_terrain->Advance(m_step_veh);
    m_hmmwv->Advance(m_step_veh);
}

void HmmwvAccTest::SimulateVis() {
    ChWheeledVehicleIrrApp app(&m_hmmwv->GetVehicle(), L"HMMWV acceleration test");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);

    app.AssetBindAll();
    app.AssetUpdateAll();

    while (app.GetDevice()->run()) {
        ChDriver::Inputs driver_inputs = m_driver->GetInputs();
        app.BeginScene();
        app.DrawAll();
        ExecuteStep();
        app.Synchronize("Acceleration test", driver_inputs);
        app.Advance(m_step_veh);
        app.EndScene();
    }
}

// =============================================================================

#define NUM_SKIP_STEPS 2000  // number of steps for hot start
#define NUM_SIM_STEPS 1000   // number of simulation steps for each benchmark

CH_BM_SIMULATION_LOOP(HmmwvAcc, HmmwvAccTest, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        HmmwvAccTest test;
        test.SimulateVis();
        return 0;
    }

    ::benchmark::RunSpecifiedBenchmarks();
}
