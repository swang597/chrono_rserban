// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jay Taves
// =============================================================================
//
// Demo code illustrating synchronization of the SCM semi-empirical model for
// deformable soil
//
// See also in chrono_vehicle:
// - demo_VEH_DeformableSoil
// - demo_VEH_DeformableSoilAndTire
// - demo_VEH_HMMWV_DefSoil
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/agent/SynSCMTerrainAgent.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#endif

#undef CHRONO_SENSOR

#ifdef CHRONO_SENSOR
    #include "chrono_sensor/ChSensorManager.h"
    #include "chrono_sensor/ChCameraSensor.h"
    #include "chrono_sensor/filters/ChFilterAccess.h"
    #include "chrono_sensor/filters/ChFilterSave.h"
    #include "chrono_sensor/filters/ChFilterVisualize.h"
using namespace chrono::sensor;
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::synchrono;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::endl;
using std::cin;

// =============================================================================

double terrainLength = 50;  // size in X direction
double terrainWidth = 50;   // size in Y direction
double delta = 0.05;        // SCM grid spacing

double pathLength = 200;

// Simulation run time
double end_time = 20;

// Simulation step size
double step_size = 2e-3;

// Number of threads
int nthreads = 8;

// Better conserve mass by displacing soil to the sides of a rut
const bool bulldozing = false;

// How often SynChrono state messages are interchanged
double heartbeat = 1e-2;  // 100[Hz]

// Forward declares for straight forward helper functions
void AddCommandLineOptions(ChCLI& cli);

// =============================================================================

int main(int argc, char* argv[]) {
    // -----------------------
    // Create SynChronoManager
    // -----------------------
    auto communicator = chrono_types::make_shared<SynMPICommunicator>(argc, argv);
    int node_id = communicator->GetRank();
    int num_nodes = communicator->GetNumRanks();
    SynChronoManager syn_manager(node_id, num_nodes, communicator);

    // Copyright
    if (node_id == 0) {
        SynLog() << "Copyright (c) 2020 projectchrono.org\n";
        SynLog() << "Chrono version: " << CHRONO_VERSION << "\n\n";
    }

#ifdef _DEBUG
    if (node_id == 0) {
        int foo;
        cout << "Enter something to continue..." << endl;
        cin >> foo;
    }
    MPI_Barrier(MPI_COMM_WORLD);
#endif

    // -----------------------------------------------------
    // CLI SETUP - Get most parameters from the command line
    // -----------------------------------------------------

    ChCLI cli(argv[0]);

    AddCommandLineOptions(cli);
    if (!cli.Parse(argc, argv, node_id == 0))
        return 0;

    // Normal simulation options
    step_size = cli.GetAsType<double>("step_size");
    end_time = cli.GetAsType<double>("end_time");
    heartbeat = cli.GetAsType<double>("heartbeat");
    nthreads = cli.GetAsType<int>("nthreads");

    const double size_x = cli.GetAsType<double>("sizeX");
    const double size_y = cli.GetAsType<double>("sizeY");
    const double cam_x = cli.GetAsType<std::vector<double>>("c_pos")[0];
    const double cam_y = cli.GetAsType<std::vector<double>>("c_pos")[1];
    const double dpu = cli.GetAsType<double>("dpu");
    const int cam_res_width = cli.GetAsType<std::vector<int>>("res")[0];
    const int cam_res_height = cli.GetAsType<std::vector<int>>("res")[1];
    bool visualize = cli.HasValueInVector<int>("irr", node_id);

    // Change SynChronoManager settings
    syn_manager.SetHeartbeat(heartbeat);

    // ----------------------
    // Vehicle Initialization
    // ----------------------
    // Calculate initial position and paths for each vehicle
    // Use up more of the mesh by not placing vehicles in the middle
    ChVector<> offset(-size_x / 2 + 5, 0, 0);

    ChVector<> init_loc;
    ChQuaternion<> init_rot;
    std::shared_ptr<ChBezierCurve> path;
    if (node_id % 2 == 0) {
        // Start even vehicles in a row on the south side, driving north
        init_loc = offset + ChVector<>(0, 2.0 * (node_id + 1), 0.5);
        init_rot = Q_from_AngZ(0);
        path = StraightLinePath(init_loc, init_loc + ChVector<>(pathLength, 0, 0));
    } else {
        // Start odd vehicles staggered going up the west edge, driving east
        init_loc = offset + ChVector<>(2.0 * (node_id - 1), -5.0 - 2.0 * (node_id - 1), 0.5);
        init_rot = Q_from_AngZ(CH_C_PI / 2);
        path = StraightLinePath(init_loc, init_loc + ChVector<>(0, pathLength, 0));
    }

    // Create the HMMWV
    HMMWV_Full hmmwv;
    hmmwv.SetContactMethod(ChContactMethod::SMC);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(ChCoordsys<>(init_loc, init_rot));
    hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
    hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    hmmwv.SetTireType(TireModelType::RIGID);
    hmmwv.SetTireStepSize(step_size);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::MESH);
    hmmwv.SetSteeringVisualizationType(VisualizationType::NONE);
    hmmwv.SetWheelVisualizationType(VisualizationType::MESH);
    hmmwv.SetTireVisualizationType(VisualizationType::MESH);

    // -----------------------------------------------------------
    // Set tire contact material, contact model, and visualization
    // -----------------------------------------------------------
    auto wheel_material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    wheel_material->SetFriction(0.8f);
    wheel_material->SetYoungModulus(1.0e6f);
    wheel_material->SetRestitution(0.1f);

    hmmwv.SetTireVisualizationType(VisualizationType::MESH);

    // What we defined earlier, a straight line
    ChPathFollowerDriver driver(hmmwv.GetVehicle(), path, "Box path", 10);
    driver.Initialize();

    // Reasonable defaults for the underlying PID
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.GetSteeringController().SetGains(0.4, 0.1, 0.2);
    driver.GetSteeringController().SetLookAheadDistance(2);

    // Add vehicle as an agent
    auto vehicle_agent = chrono_types::make_shared<SynWheeledVehicleAgent>(&hmmwv.GetVehicle());
    vehicle_agent->SetZombieVisualizationFiles("hmmwv/hmmwv_chassis.obj", "hmmwv/hmmwv_rim.obj",
                                               "hmmwv/hmmwv_tire_left.obj");
    vehicle_agent->SetNumWheels(4);
    syn_manager.AddAgent(vehicle_agent);

    // ----------------------
    // Terrain specific setup
    // ----------------------
    ChSystem* system = hmmwv.GetSystem();
    system->SetNumThreads(nthreads);
    std::cout << "Num threads: " << nthreads << std::endl;

    SCMDeformableTerrain terrain(system, visualize);
    terrain.SetSoilParameters(2e6,   // Bekker Kphi
                              0,     // Bekker Kc
                              1.1,   // Bekker n exponent
                              0,     // Mohr cohesive limit (Pa)
                              30,    // Mohr friction limit (degrees)
                              0.01,  // Janosi shear coefficient (m)
                              2e8,   // Elastic stiffness (Pa/m), before plastic yield
                              3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    // Configure the SCM terrain
    if (bulldozing) {
        terrain.EnableBulldozing(bulldozing);
        terrain.SetBulldozingParameters(
            55,   // angle of friction for erosion of displaced material at the border of the rut
            0.8,  // displaced material vs downward pressed material.
            5,    // number of erosion refinements per timestep
            10);  // number of concentric vertex selections subject to erosion
    }

    // The physics do not change when you add a moving patch, you just make it much easier for the SCM
    // implementation to do its job by restricting where it has to look for contacts
    terrain.AddMovingPatch(hmmwv.GetVehicle().GetChassisBody(), ChVector<>(0, 0, 0), ChVector<>(5, 3, 1));

    // Optionally, enable moving patch feature (multiple patches around each wheel)
    ////for (auto& axle : my_hmmwv.GetVehicle().GetAxles()) {
    ////    terrain.AddMovingPatch(axle->m_wheels[0]->GetSpindle(), ChVector<>(0, 0, 0), ChVector<>(1, 0.5, 1));
    ////    terrain.AddMovingPatch(axle->m_wheels[1]->GetSpindle(), ChVector<>(0, 0, 0), ChVector<>(1, 0.5, 1));
    ////}

    terrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.1);

    terrain.Initialize(terrainLength, terrainWidth, delta);

    // Create an SCMTerrainAgent and add it to the SynChrono manager
    auto scm = chrono_types::make_shared<SCMDeformableTerrain>(terrain);
    auto terrain_agent = chrono_types::make_shared<SynSCMTerrainAgent>(scm);
    syn_manager.AddAgent(terrain_agent);

    // Choice of soft parameters is arbitrary
    SCMParameters params;
    params.InitializeParametersAsSoft();
    terrain_agent->SetSoilParametersFromStruct(&params);

    // Initialzie the SynChrono manager
    syn_manager.Initialize(system);

    // -------------
    // Visualization
    // -------------
#ifdef CHRONO_IRRLICHT
    // Create the vehicle Irrlicht interface
    std::shared_ptr<ChWheeledVehicleIrrApp> app;
    if (visualize) {
        app = chrono_types::make_shared<ChWheeledVehicleIrrApp>(&hmmwv.GetVehicle(), L"SynChrono SCM Demo");
        app->SetSkyBox();
        app->AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250,
                              130);
        app->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
        app->SetTimestep(step_size);
        app->AssetBindAll();
        app->AssetUpdateAll();
    }
#endif

#ifdef CHRONO_SENSOR
    ChSensorManager sensor_manager(system);
    if (cli.HasValueInVector<int>("sens", node_id)) {
        // Give the camera a fixed place to live
        auto origin = chrono_types::make_shared<ChBody>();
        origin->SetBodyFixed(true);
        system->AddBody(origin);

        // Happens to be a reasonable-looking height
        ChVector<> camera_loc(cam_x, cam_y, 25);

        // Rotations to get a nice angle
        ChQuaternion<> rotation = QUNIT;
        const bool USE_ISO_VIEW = true;
        if (USE_ISO_VIEW) {
            ChQuaternion<> qA = Q_from_AngAxis(35 * CH_C_DEG_TO_RAD, VECT_Y);
            ChQuaternion<> qB = Q_from_AngAxis(135 * CH_C_DEG_TO_RAD, VECT_Z);
            rotation = rotation >> qA >> qB;
        } else {
            // Top down view
            ChQuaternion<> qA = Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_Y);
            ChQuaternion<> qB = Q_from_AngAxis(180 * CH_C_DEG_TO_RAD, VECT_Z);
            rotation = rotation >> qA >> qB;
        }

        auto overhead_camera = chrono_types::make_shared<chrono::sensor::ChCameraSensor>(
            origin,                                         // body camera is attached to
            30.0f,                                          // update rate in Hz
            chrono::ChFrame<double>(camera_loc, rotation),  // offset pose
            cam_res_width,                                  // image width
            cam_res_height,                                 // image height
            (float)CH_C_PI / 3                              // FOV
        );

        overhead_camera->SetName("Overhead Cam");
        overhead_camera->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

        // Do we draw a window on the screen?
        if (cli.GetAsType<bool>("sens_vis"))
            overhead_camera->PushFilter(chrono_types::make_shared<ChFilterVisualize>(cam_res_width, cam_res_height));

        // Do we save images to disc?
        std::string file_path = std::string("SENSOR_OUTPUT/scm") + std::to_string(node_id) + std::string("/");
        if (cli.GetAsType<bool>("sens_save"))
            overhead_camera->PushFilter(chrono_types::make_shared<ChFilterSave>(file_path));

        sensor_manager.AddSensor(overhead_camera);
    }
#endif

    // ---------------
    // Simulation loop
    // ---------------
    // Number of simulation steps between miscellaneous events
    double render_step_size = 1.0 / 100;
    int render_steps = (int)std::ceil(render_step_size / step_size);

    system->SetSolverMaxIterations(50);

    // Initialize simulation frame counters
    int step_number = 0;
    ChRealtimeStepTimer realtime_timer;

    ChTimer<> timer;
    timer.start();

    while (true) {
        double time = system->GetChTime();

        // End simulation
        if (time >= end_time         // ran out of time
            || !syn_manager.IsOk())  // SynChronoManager has shutdown
            break;

#ifdef CHRONO_IRRLICHT
        if (app && !app->GetDevice()->run())  //  Irrlicht visualization has stopped
            break;

        // Render scene
        if (step_number % render_steps == 0 && app) {
            app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app->DrawAll();
            app->EndScene();
        }
#endif

        // Get driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        syn_manager.Synchronize(time);  // Synchronize between nodes
        driver.Synchronize(time);
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);
#ifdef CHRONO_IRRLICHT
        if (app)
            app->Synchronize("", driver_inputs);
#endif

        // Advance simulation for one timestep for all modules
        ////system->DoStepDynamics(step_size);

        driver.Advance(step_size);
        terrain.Advance(step_size);
        hmmwv.Advance(step_size);
#ifdef CHRONO_IRRLICHT
        if (app)
            app->Advance(step_size);
#endif

#ifdef CHRONO_SENSOR
        sensor_manager.Update();
#endif

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        ////realtime_timer.Spin(step_size);
    }
    timer.stop();

    for (int i = 0; i < num_nodes; i++) {
        if (node_id == i) {
            std::cout << "Stats for rank: " << node_id << std::endl;
            std::cout << "stop timer at t = " << end_time << std::endl;
            std::cout << "elapsed: " << timer() << std::endl;
            std::cout << "RTF: " << (timer() / end_time) << std::endl;
            std::cout << "\nSCM stats for last step:" << std::endl;
            terrain.PrintStepStatistics(std::cout);
            std::cout << std::endl;
        }
    }

    return 0;
}

void AddCommandLineOptions(ChCLI& cli) {
    // Standard demo options
    cli.AddOption<double>("Simulation", "s,step_size", "Step size", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "e,end_time", "End time", std::to_string(end_time));
    cli.AddOption<double>("Simulation", "b,heartbeat", "Heartbeat", std::to_string(heartbeat));
    cli.AddOption<int>("Simulation", "n,nthreads", "Number threads", std::to_string(nthreads));

    // SCM specific options
    cli.AddOption<double>("Demo", "d,dpu", "Divisions per unit", "20");
    cli.AddOption<std::string>("Demo", "t,terrain_type", "Terrain Type", "SCM", "Rigid,SCM");

    // Visualization is the only reason you should be shy about terrain size. The implementation can easily handle a
    // practically infinite terrain (provided you don't need to visualize it)
    cli.AddOption<double>("Demo", "x,sizeX", "Size in the X", "100");
    cli.AddOption<double>("Demo", "y,sizeY", "Size in the Y", "50");

    // Irrlicht options
    cli.AddOption<std::vector<int>>("Irrlicht", "i,irr", "Ranks for irrlicht usage", "-1");

    // Sensor options
    cli.AddOption<std::vector<int>>("Sensor", "sens", "Ranks for sensor usage", "-1");
    cli.AddOption<bool>("Sensor", "sens_save", "Toggle sensor saving ON", "false");
    cli.AddOption<bool>("Sensor", "sens_vis", "Toggle sensor visualization ON", "false");

    cli.AddOption<std::vector<int>>("Demo", "r,res", "Camera resolution", "1280,720", "width,height");
    cli.AddOption<std::vector<double>>("Demo", "c_pos", "Camera Position", "-15,-25", "X,Y");
}
