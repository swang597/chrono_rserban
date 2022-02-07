// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.

// Chrono::Vehicle + Chrono::Multicore demo program for simulating a HMMWV vehicle
// over rigid or granular material.
//
// Contact uses the SMC (penalty) formulation.
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/fea/ChContactSurfaceMesh.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/FEADeformableTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::fea;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// -----------------------------------------------------------------------------
// Vehicle parameters
// -----------------------------------------------------------------------------

// Type of tire (controls both contact and visualization)
enum class TireType { CYLINDRICAL, LUGGED };
TireType tire_type = TireType::CYLINDRICAL;

// Tire contact material properties
float Y_t = 1.0e6f;
float cr_t = 0.1f;
float mu_t = 0.8f;

// Initial vehicle position and orientation
ChVector<> initLoc(-5, -2, 1.6);
ChQuaternion<> initRot(1, 0, 0, 0);

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-4;

// Time interval between two render frames (1/FPS)
double render_step_size = 1.0 / 100;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Output directories
const std::string out_dir = GetChronoOutputPath() + "HMMWV_DEF_SOIL";
const std::string img_dir = out_dir + "/IMG";

// Visualization output
bool img_output = false;

// =============================================================================

class MyDriver : public ChDriver {
  public:
    MyDriver(ChVehicle& vehicle, double delay) : ChDriver(vehicle), m_delay(delay) {}
    ~MyDriver() {}

    virtual void Synchronize(double time) override {
        m_throttle = 0;
        m_steering = 0;
        m_braking = 0;

        double eff_time = time - m_delay;

        // Do not generate any driver inputs for a duration equal to m_delay.
        if (eff_time < 0)
            return;

        if (eff_time > 0.2)
            m_throttle = 0.7;
        else
            m_throttle = 3.5 * eff_time;

        if (eff_time < 2)
            m_steering = 0;
        else
            m_steering = 0.6 * std::sin(CH_C_2PI * (eff_time - 2) / 6);
    }

  private:
    double m_delay;
};

// =============================================================================

void CreateLuggedGeometry(std::shared_ptr<ChBody>wheel_body, std::shared_ptr<ChMaterialSurface>wheel_material) {
    std::string lugged_file("hmmwv/lugged_wheel_section.obj");
    geometry::ChTriangleMeshConnected lugged_mesh;
    ChConvexDecompositionHACDv2 lugged_convex;
    chrono::utils::LoadConvexMesh(vehicle::GetDataFile(lugged_file), lugged_mesh, lugged_convex);
    int num_hulls = lugged_convex.GetHullCount();

    auto coll_model = wheel_body->GetCollisionModel();
    coll_model->ClearModel();

    // Assemble the tire contact from 15 segments, properly offset.
    // Each segment is further decomposed in convex hulls.
    for (int iseg = 0; iseg < 15; iseg++) {
        ChQuaternion<> rot = Q_from_AngAxis(iseg * 24 * CH_C_DEG_TO_RAD, VECT_Y);
        for (int ihull = 0; ihull < num_hulls; ihull++) {
            std::vector<ChVector<> > convexhull;
            lugged_convex.GetConvexHullResult(ihull, convexhull);
            coll_model->AddConvexHull(wheel_material, convexhull, VNULL, rot);
        }
    }

    // Add a cylinder to represent the wheel hub.
    coll_model->AddCylinder(wheel_material, 0.223, 0.223, 0.126);
    coll_model->BuildModel();

    // Visualization
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(vehicle::GetDataFile("hmmwv/lugged_wheel.obj"), false, false);

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName("lugged_wheel");
    wheel_body->AddAsset(trimesh_shape);

    auto mcolor = chrono_types::make_shared<ChColorAsset>(0.3f, 0.3f, 0.3f);
    wheel_body->AddAsset(mcolor);
}

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------
    // Create HMMWV vehicle
    // --------------------
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChContactMethod::SMC);
    my_hmmwv.SetChassisFixed(true);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
    my_hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    switch (tire_type) {
        case TireType::CYLINDRICAL:
            my_hmmwv.SetTireType(TireModelType::RIGID_MESH);
            break;
        case TireType::LUGGED:
            my_hmmwv.SetTireType(TireModelType::RIGID);
            break;
    }
    my_hmmwv.Initialize();
    my_hmmwv.SetChassisVisualizationType(VisualizationType::NONE);

    // -----------------------------------------------------------
    // Set tire contact material, contact model, and visualization
    // -----------------------------------------------------------
    auto wheel_material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    wheel_material->SetFriction(mu_t);
    wheel_material->SetYoungModulus(Y_t);
    wheel_material->SetRestitution(cr_t);

    switch (tire_type) {
        case TireType::CYLINDRICAL:
            my_hmmwv.SetTireVisualizationType(VisualizationType::MESH);
            break;
        case TireType::LUGGED:
            my_hmmwv.SetTireVisualizationType(VisualizationType::NONE);
            for (auto& axle : my_hmmwv.GetVehicle().GetAxles()) {
                CreateLuggedGeometry(axle->m_wheels[0]->GetSpindle(), wheel_material);
                CreateLuggedGeometry(axle->m_wheels[1]->GetSpindle(), wheel_material);
            }
    }

    // --------------------
    // Create driver system
    // --------------------
    MyDriver driver(my_hmmwv.GetVehicle(), 0.5);
    driver.Initialize();

    // ------------------
    // Create the terrain
    // ------------------
    ChSystem* system = my_hmmwv.GetSystem();
    system->SetNumThreads(std::min(8, ChOMP::GetNumProcs()));
    system->SetNumThreads(12);

    // FEA Terrain Generation:
    FEADeformableTerrain terrain(system);
    // Use either a regular grid:
    ChVector<> start_point(-7.4, -3.8, -0.5);
    ChVector<> terrain_dimension(6.0, 3.5, 0.5);
    ChVector<> terrain_discretization(12.0, 7.0, 4.0);
    /// ChVector<> terrain_discretization(60.0, 35.0, 5.0);
    /// ChVector<> terrain_discretization(30.0, 18.0, 3.0);
    terrain.Initialize(start_point, terrain_dimension, terrain_discretization);
    terrain.SetSoilParametersFEA(200.0,    ///< Soil density
                                 1.379e7,  ///< Soil modulus of elasticity
                                 0.3,      ///< Soil Poisson ratio
                                 10000.0,  ///< [in] Soil yield stress, for plasticity
                                 5000,     ///< [in] Soil hardening slope, for plasticity
                                 0.00001,  ///< Soil internal friction angle
                                 0.00001   ///< Soil dilatancy angle
    );

    // Add contact surface mesh.
    ////auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    ////mysurfmaterial->SetYoungModulus(6e4);
    ////mysurfmaterial->SetFriction(0.3f);
    ////mysurfmaterial->SetRestitution(0.2f);
    ////mysurfmaterial->SetAdhesion(0);
    ////mysurfmaterial->SetKt(4e4);
    ////mysurfmaterial->SetKn(1e5);

    ////std::shared_ptr<ChContactSurfaceMesh> my_contactsurface(new ChContactSurfaceMesh(mysurfmaterial));
    ////terrain.GetMesh()->AddContactSurface(my_contactsurface);
    ////my_contactsurface->AddFacesFromBoundary(1.0e-2);  // Sphere swept


    auto mvisualizemeshref = chrono_types::make_shared<ChVisualizationFEAmesh>(*(terrain.GetMesh()));
    mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    terrain.GetMesh()->AddAsset(mvisualizemeshref);

    auto mvisualizemeshcoll = chrono_types::make_shared<ChVisualizationFEAmesh>(*(terrain.GetMesh()));
    mvisualizemeshcoll->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_CONTACTSURFACES);
    mvisualizemeshcoll->SetWireframe(true);
    mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1, 0.5, 0));
    terrain.GetMesh()->AddAsset(mvisualizemeshcoll);

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------
    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), L"HMMWV Deformable Soil Demo");
    app.AddTypicalLights();
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // -----------------
    // Initialize output
    // -----------------
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (img_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------
    std::cout << "Total vehicle mass: " << my_hmmwv.GetTotalMass() << std::endl;

    // Solver settings.
    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    system->SetSolver(solver);
    solver->SetVerbose(false);

    // Setup timestepper
    system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto stepper = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    stepper->SetAlpha(-0.2);
    stepper->SetMaxiters(200);
    stepper->SetAbsTolerances(1e-06);
    stepper->SetMode(ChTimestepperHHT::POSITION);
    stepper->SetScaling(true);
    stepper->SetVerbose(true);

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    ChTimer<> timer;

    while (app.GetDevice()->run()) {
        double time = system->GetChTime();
        std::cout << "\ntime = " << time << std::endl;

        if (step_number == 800) {
            std::cout << "\nstart timer at t = " << time << std::endl;
            timer.start();
        }
        if (step_number == 1400) {
            timer.stop();
            std::cout << "stop timer at t = " << time << std::endl;
            std::cout << "elapsed: " << timer() << std::endl;
            std::cout << "\nSCM stats for last step:" << std::endl;
            /// terrain.PrintStepStatistics(std::cout);
        }

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        tools::drawColorbar(0, 0.1, "Sinkage", app.GetDevice(), 30);
        app.EndScene();

        if (img_output && step_number % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
            app.WriteImageToFile(filename);
            render_frame++;
        }

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        app.Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;
    }

    return 0;
}