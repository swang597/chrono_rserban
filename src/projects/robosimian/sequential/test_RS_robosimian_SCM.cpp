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
// RoboSimian on rigid terrain
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_models/robosimian/robosimian.h"

#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "irrlicht_app.h"

using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;

// Drop the robot on SCM terrain
bool drop = true;

// Phase durations
double duration_pose = 1.0;          // Interval to assume initial pose
double duration_settle_robot = 0.5;  // Interval to allow robot settling on terrain
double duration_sim = 10;            // Duration of actual locomotion simulation

// Output frequencies
double output_fps = 100;
double render_fps = 60;

// Output directories
const std::string out_dir = "../ROBOSIMIAN_SCM";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string img_dir = out_dir + "/IMG";

// =============================================================================

void ShowUsage(const std::string& name);
bool GetProblemSpecs(int argc,
                     char** argv,
                     double& time_step,
                     robosimian::LocomotionMode& mode,
                     int& num_cycles,
                     double& dpb_incr,
                     double& terrain_length,
                     bool& render,
                     bool& data_output,
                     bool& image_output,
                     bool& povray_output);

// =============================================================================

class DBPcontroller : public chrono::robosimian::Driver::PhaseChangeCallback {
  public:
    DBPcontroller(robosimian::RoboSimian* robot);
    ~DBPcontroller() { delete m_csv; }

    virtual void OnPhaseChange(robosimian::Driver::Phase old_phase, robosimian::Driver::Phase new_phase) override;

    void SetCycleFreq(int freq) { m_cycle_freq = freq; }
    void SetIncrement(double incr) { m_dbp_incr = incr; }
    bool Stop() const { return m_avg_speed < 0; }
    void WriteOutput(const std::string& filename) { m_csv->write_to_file(filename); }

    double GetDistance() const { return m_robot->GetChassisPos().x() - m_start_x; }
    double GetDuration() const { return m_robot->GetSystem()->GetChTime() - m_start_time; }
    double GetAvgSpeed() const { return GetDistance() / GetDuration(); }

  private:
    robosimian::RoboSimian* m_robot;
    std::shared_ptr<ChLoadBodyForce> m_load;

    double m_weight;    // total robot weight
    double m_dbp;       // DBP factor (ratio of robot weight)
    double m_dbp_incr;  // DBP factor increment

    int m_cycle_number;  // current cycle
    int m_cycle_freq;    // number of cycles between DBP increases

    double m_start_x;     // cached robot location
    double m_start_time;  // cached time at last location
    double m_avg_speed;   // average speed over last segment

    utils::CSV_writer* m_csv;
    ChTimer<> m_timer;
};

DBPcontroller::DBPcontroller(robosimian::RoboSimian* robot)
    : m_robot(robot),
      m_dbp(0),
      m_dbp_incr(0.04),
      m_cycle_freq(2),
      m_cycle_number(0),
      m_start_x(0),
      m_start_time(0),
      m_avg_speed(0) {
    // Cache robot weight
    double mass = robot->GetMass();
    double g = std::abs(robot->GetSystem()->Get_G_acc().z());
    m_weight = mass * g;

    // Create a body force load on the robot chassis.
    // This is a horizontal force applied at the chassis center.
    auto force_container = chrono_types::make_shared<ChLoadContainer>();
    robot->GetSystem()->Add(force_container);
    m_load = chrono_types::make_shared<ChLoadBodyForce>(robot->GetChassisBody(), VNULL, false, VNULL, true);
    force_container->Add(m_load);

    m_timer.reset();
    m_timer.start();

    // Prepare CSV output
    m_csv = new utils::CSV_writer(",");
    *m_csv << "DBP_factor"
           << "DBP_force"
           << "Avg_speed" << endl;
}

void DBPcontroller::OnPhaseChange(robosimian::Driver::Phase old_phase, robosimian::Driver::Phase new_phase) {
    m_timer.stop();
    cout << "Elapsed time: " << m_timer() << endl;
    m_timer.reset();
    m_timer.start();

    if (new_phase == robosimian::Driver::CYCLE) {
        if (m_cycle_number == 0) {
            m_start_x = m_robot->GetChassisPos().x();
            m_start_time = m_robot->GetSystem()->GetChTime();
        } else if (m_cycle_number % m_cycle_freq == 0) {
            // Save average speed at current DBP factor
            m_avg_speed = GetAvgSpeed();
            cout << "*** DBP: " << m_dbp << " force: " << m_dbp * m_weight << " avg. speed: " << m_avg_speed << endl;
            *m_csv << m_dbp << m_dbp * m_weight << m_avg_speed << endl;
            // Cache new start time and location
            m_start_x = m_robot->GetChassisPos().x();
            m_start_time = m_robot->GetSystem()->GetChTime();
            // Increment DBP force
            m_dbp += m_dbp_incr;
            m_load->SetForce(ChVector<>(-m_dbp * m_weight, 0, 0), false);
        }

        cout << "  FL wheel location:  " << m_robot->GetWheelPos(robosimian::LimbID::FL).x() << endl;
        cout << "  FR wheel location:  " << m_robot->GetWheelPos(robosimian::LimbID::FR).x() << endl;
        cout << "  Chassis location:   " << m_robot->GetChassisPos().x() << endl;
        cout << "  Distance travelled: " << GetDistance() << endl;

        m_cycle_number++;
    }
}

// =============================================================================

std::shared_ptr<vehicle::SCMDeformableTerrain> CreateTerrain(robosimian::RoboSimian* robot,
                                                             double length,
                                                             double width,
                                                             double height,
                                                             double offset) {
    // Deformable terrain properties (LETE sand)
    ////double Kphi = 5301e3;    // Bekker Kphi
    ////double Kc = 102e3;       // Bekker Kc
    ////double n = 0.793;        // Bekker n exponent
    ////double coh = 1.3e3;      // Mohr cohesive limit (Pa)
    ////double phi = 31.1;       // Mohr friction limit (degrees)
    ////double K = 1.2e-2;       // Janosi shear coefficient (m)
    ////double E_elastic = 2e8;  // Elastic stiffness (Pa/m), before plastic yeld
    ////double damping = 3e4;    // Damping coefficient (Pa*s/m)

    // Deformable terrain properties (CDT FGS dry - 6/29/2018)
    double Kphi = 6259.1e3;  // Bekker Kphi
    double Kc = 5085.6e3;    // Bekker Kc
    double n = 1.42;         // Bekker n exponent
    double coh = 1.58e3;     // Mohr cohesive limit (Pa)
    double phi = 34.1;       // Mohr friction limit (degrees)
    double K = 22.17e-3;     // Janosi shear coefficient (m)
    double E_elastic = 2e8;  // Elastic stiffness (Pa/m), before plastic yeld
    double damping = 3e4;    // Damping coefficient (Pa*s/m)

    // Initial number of divisions per unit (m)
    double factor = 8;

    // Mesh divisions
    int ndivX = (int)std::ceil(length * factor);
    int ndivY = (int)std::ceil(width * factor);

    auto terrain = chrono_types::make_shared<vehicle::SCMDeformableTerrain>(robot->GetSystem());
    terrain->SetPlane(ChCoordsys<>(ChVector<>(length / 2 - offset, 0, 0), Q_from_AngX(CH_C_PI_2)));
    terrain->SetSoilParameters(Kphi, Kc, n, coh, phi, K, E_elastic, damping);
    terrain->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.15);
    terrain->Initialize(height, length, width, ndivX, ndivY);
    terrain->SetAutomaticRefinement(true);
    terrain->SetAutomaticRefinementResolution(1.0 / 64);

    // Enable moving patch feature
    terrain->EnableMovingPatch(robot->GetChassisBody(), ChVector<>(0, 0, 0), 3.0, 2.0);

    return terrain;
}

void SetContactProperties(robosimian::RoboSimian* robot) {
    assert(robot->GetSystem()->GetContactMethod() == ChMaterialSurface::SMC);

    float friction = 0.4f;
    float Y = 1e7f;
    float cr = 0.0f;

    robot->GetSledBody()->GetMaterialSurfaceSMC()->SetFriction(friction);
    robot->GetWheelBody(robosimian::LimbID::FR)->GetMaterialSurfaceSMC()->SetFriction(friction);
    robot->GetWheelBody(robosimian::LimbID::FL)->GetMaterialSurfaceSMC()->SetFriction(friction);
    robot->GetWheelBody(robosimian::LimbID::RR)->GetMaterialSurfaceSMC()->SetFriction(friction);
    robot->GetWheelBody(robosimian::LimbID::RL)->GetMaterialSurfaceSMC()->SetFriction(friction);

    robot->GetSledBody()->GetMaterialSurfaceSMC()->SetYoungModulus(Y);
    robot->GetWheelBody(robosimian::LimbID::FR)->GetMaterialSurfaceSMC()->SetYoungModulus(Y);
    robot->GetWheelBody(robosimian::LimbID::FL)->GetMaterialSurfaceSMC()->SetYoungModulus(Y);
    robot->GetWheelBody(robosimian::LimbID::RR)->GetMaterialSurfaceSMC()->SetYoungModulus(Y);
    robot->GetWheelBody(robosimian::LimbID::RL)->GetMaterialSurfaceSMC()->SetYoungModulus(Y);

    robot->GetSledBody()->GetMaterialSurfaceSMC()->SetRestitution(cr);
    robot->GetWheelBody(robosimian::LimbID::FR)->GetMaterialSurfaceSMC()->SetRestitution(cr);
    robot->GetWheelBody(robosimian::LimbID::FL)->GetMaterialSurfaceSMC()->SetRestitution(cr);
    robot->GetWheelBody(robosimian::LimbID::RR)->GetMaterialSurfaceSMC()->SetRestitution(cr);
    robot->GetWheelBody(robosimian::LimbID::RL)->GetMaterialSurfaceSMC()->SetRestitution(cr);
}

// =============================================================================

int main(int argc, char* argv[]) {
    // ----------------------------
    // Parse command line arguments
    // ----------------------------
    robosimian::LocomotionMode mode;
    double time_step;
    int num_cycles;
    double dbp_incr;
    double terrain_length;
    bool render;
    bool data_output;
    bool povray_output;
    bool image_output;

    // Extract arguments
    if (!GetProblemSpecs(argc, argv, time_step, mode, num_cycles, dbp_incr, terrain_length, render, data_output,
                         image_output, povray_output)) {
        cout << "-------------" << endl;
        return 1;
    }

    double terrain_width = 2.0;
    double location_offset = 2.0;

    // ------------
    // Timed events
    // ------------

    double time_create_terrain = duration_pose;                       // create terrain after robot assumes initial pose
    double time_start = time_create_terrain + duration_settle_robot;  // start actual simulation after robot settling
    double time_end = time_start + duration_sim;                      // end simulation after specified duration

    // -------------
    // Create system
    // -------------

    ChSystemSMC my_sys;
    my_sys.SetSolverMaxIterations(200);
    my_sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));
    ////my_sys.Set_G_acc(ChVector<double>(0, 0, 0));

    // -----------------------
    // Create RoboSimian robot
    // -----------------------

    std::string mode_name;
    switch (mode) {
        case robosimian::LocomotionMode::WALK:
            mode_name = "walk";
            break;
        case robosimian::LocomotionMode::SCULL:
            mode_name = "scull";
            break;
        case robosimian::LocomotionMode::INCHWORM:
            mode_name = "inchworm";
            break;
        case robosimian::LocomotionMode::DRIVE:
            mode_name = "drive";
            break;
    }

    robosimian::RoboSimian robot(&my_sys, true, true);

    // Set output directory

    robot.SetOutputDirectory(out_dir, mode_name);

    // Set actuation mode for wheel motors

    ////robot.SetMotorActuationMode(robosimian::ActuationMode::ANGLE);

    // Control collisions (default: true for sled and wheels only)

    ////robot.SetCollide(robosimian::CollisionFlags::NONE);
    ////robot.SetCollide(robosimian::CollisionFlags::ALL);
    ////robot.SetCollide(robosimian::CollisionFlags::LIMBS);
    ////robot.SetCollide(robosimian::CollisionFlags::CHASSIS | robosimian::CollisionFlags::WHEELS);

    // Set visualization modes (default: all COLLISION)

    ////robot.SetVisualizationTypeChassis(robosimian::VisualizationType::MESH);
    ////robot.SetVisualizationTypeLimb(robosimian::FL, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimb(robosimian::FR, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimb(robosimian::RL, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimb(robosimian::RR, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimbs(robosimian::VisualizationType::NONE);
    robot.SetVisualizationTypeChassis(robosimian::VisualizationType::MESH);
    robot.SetVisualizationTypeSled(robosimian::VisualizationType::MESH);
    robot.SetVisualizationTypeLimbs(robosimian::VisualizationType::MESH);

    // Initialize Robosimian robot

    ////robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI)));

    // -----------------------------------
    // Create a driver and attach to robot
    // -----------------------------------

    std::shared_ptr<robosimian::Driver> driver;
    switch (mode) {
        case robosimian::LocomotionMode::WALK:
            driver = chrono_types::make_shared<robosimian::Driver>(
                "",                                                           // start input file
                GetChronoDataFile("robosimian/actuation/walking_cycle.txt"),  // cycle input file
                "",                                                           // stop input file
                true);
            break;
        case robosimian::LocomotionMode::SCULL:
            driver = chrono_types::make_shared<robosimian::Driver>(
                GetChronoDataFile("robosimian/actuation/sculling_start.txt"),   // start input file
                GetChronoDataFile("robosimian/actuation/sculling_cycle2.txt"),  // cycle input file
                GetChronoDataFile("robosimian/actuation/sculling_stop.txt"),    // stop input file
                true);
            break;
        case robosimian::LocomotionMode::INCHWORM:
            driver = chrono_types::make_shared<robosimian::Driver>(
                GetChronoDataFile("robosimian/actuation/inchworming_start.txt"),  // start input file
                GetChronoDataFile("robosimian/actuation/inchworming_cycle.txt"),  // cycle input file
                GetChronoDataFile("robosimian/actuation/inchworming_stop.txt"),   // stop input file
                true);
            break;
        case robosimian::LocomotionMode::DRIVE:
            driver = chrono_types::make_shared<robosimian::Driver>(
                GetChronoDataFile("robosimian/actuation/driving_start.txt"),  // start input file
                GetChronoDataFile("robosimian/actuation/driving_cycle.txt"),  // cycle input file
                GetChronoDataFile("robosimian/actuation/driving_stop.txt"),   // stop input file
                true);
            break;
    }

    driver->SetTimeOffsets(duration_pose, duration_settle_robot);
    robot.SetDriver(driver);

    // -----------------------------------------------------
    // Drawback pull setup (as phase-change callback object)
    // -----------------------------------------------------

    cout << "Problem parameters" << endl;
    cout << "  Locomotion mode:       " << mode_name << endl;
    cout << "  Integration step size: " << time_step << endl;
    cout << "  Number cycles:         " << num_cycles << endl;
    cout << "  DBP increment:         " << dbp_incr << endl;
    cout << "  Terrain length:        " << terrain_length << endl;
    cout << "  Render?        " << (render ? "YES" : "NO") << endl;
    cout << "  Data output?   " << (data_output ? "YES" : "NO") << endl;
    cout << "  Image output?  " << (image_output ? "YES" : "NO") << endl;
    cout << "  PovRay output? " << (povray_output ? "YES" : "NO") << endl;
    cout << "RoboSimian total mass: " << robot.GetMass() << endl;

    DBPcontroller DBP_controller(&robot);
    DBP_controller.SetCycleFreq(num_cycles);
    DBP_controller.SetIncrement(dbp_incr);
    driver->RegisterPhaseChangeCallback(&DBP_controller);

    // -------------------------------
    // Create the visualization window
    // -------------------------------

    RobotIrrApp* application = nullptr;
    if (render) {
        application = new RobotIrrApp(&robot, driver.get(), L"RoboSimian - SCM terrain",
                                      irr::core::dimension2d<irr::u32>(800, 600));
        application->AddTypicalLogo();
        application->AddTypicalSky();
        application->AddTypicalCamera(irr::core::vector3df(1, -2.75f, 0.2f), irr::core::vector3df(1, 0, 0));
        application->AddTypicalLights(irr::core::vector3df(100.f, 100.f, 100.f),
                                      irr::core::vector3df(100.f, -100.f, 80.f));
        application->AddLightWithShadow(irr::core::vector3df(10.0f, -6.0f, 3.0f), irr::core::vector3df(0, 0, 0), 3, -3,
                                        7, 40, 512);

        application->AssetBindAll();
        application->AssetUpdateAll();
    }

    // -----------------------------
    // Initialize output directories
    // -----------------------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            cout << "Error creating directory " << pov_dir << endl;
            return 1;
        }
    }
    if (image_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            cout << "Error creating directory " << img_dir << endl;
            return 1;
        }
    }

    // ---------------------------------
    // Run simulation for specified time
    // ---------------------------------

    int output_steps = (int)std::ceil((1.0 / output_fps) / time_step);
    int render_steps = (int)std::ceil((1.0 / render_fps) / time_step);
    int sim_frame = 0;
    int output_frame = 0;
    int render_frame = 0;

    std::shared_ptr<vehicle::SCMDeformableTerrain> terrain;
    bool terrain_created = false;

    while (true) {
        if (render && !application->GetDevice()->run()) {
            break;
        }

        if (DBP_controller.Stop()) {
            break;
        }

        if (robot.GetChassisPos().x() > terrain_length - 2 * location_offset) {
            cout << "Reached end of terrain patch!" << endl;
            cout << "  time = " << my_sys.GetChTime() << endl;
            cout << "  Chassis location: " << robot.GetChassisPos().x() << endl;
            break;
        }

        if (drop && !terrain_created && my_sys.GetChTime() > time_create_terrain) {
            // Set terrain height
            double z = robot.GetWheelPos(robosimian::FR).z() - 0.15;

            // Create terrain
            terrain = CreateTerrain(&robot, terrain_length, terrain_width, z, location_offset);
            SetContactProperties(&robot);

            if (render) {
                application->AssetBindAll();
                application->AssetUpdateAll();
            }

            // Release robot
            robot.GetChassisBody()->SetBodyFixed(false);

            terrain_created = true;
        }

        if (render) {
            application->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            application->DrawAll();
        }

        if (data_output && sim_frame % output_steps == 0) {
            robot.Output();
        }

        // Output POV-Ray date and/or snapshot images
        if (sim_frame % render_steps == 0) {
            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%04d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(&my_sys, filename);
            }
            if (render && image_output) {
                char filename[100];
                sprintf(filename, "%s/img_%04d.jpg", img_dir.c_str(), render_frame + 1);
                irr::video::IImage* image = application->GetVideoDriver()->createScreenShot();
                if (image) {
                    application->GetVideoDriver()->writeImageToFile(image, filename);
                    image->drop();
                }
            }

            render_frame++;
        }

        robot.DoStepDynamics(time_step);

        sim_frame++;

        if (render) {
            application->EndScene();
        }
    }

    DBP_controller.WriteOutput(out_dir + "/DBP_" + mode_name + ".csv");

    delete application;
    return 0;
}

// =============================================================================
// ID values to identify command line arguments
enum {
    OPT_HELP,
    OPT_STEP,
    OPT_MODE,
    OPT_CYCLES,
    OPT_INCREMENT,
    OPT_LENGTH,
    OPT_NO_RENDER,
    OPT_DATA_OUT,
    OPT_IMG_OUT,
    OPT_POVRAY_OUT
};

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_STEP, "-s", SO_REQ_CMB},
                                    {OPT_MODE, "-m", SO_REQ_CMB},
                                    {OPT_CYCLES, "-c", SO_REQ_CMB},
                                    {OPT_INCREMENT, "-i", SO_REQ_CMB},
                                    {OPT_LENGTH, "-l", SO_REQ_CMB},
                                    {OPT_NO_RENDER, "--no-render", SO_NONE},
                                    {OPT_DATA_OUT, "--data-output", SO_NONE},
                                    {OPT_IMG_OUT, "--image-output", SO_NONE},
                                    {OPT_POVRAY_OUT, "--povray-output", SO_NONE},
                                    {OPT_HELP, "-?", SO_NONE},
                                    {OPT_HELP, "-h", SO_NONE},
                                    {OPT_HELP, "--help", SO_NONE},
                                    SO_END_OF_OPTIONS};

void ShowUsage(const std::string& name) {
    cout << "Usage: " << name << " -m=MODE -c=NUM_CYCLES -i=INCREMENT [OPTIONS]" << endl;
    cout << " -s=STEP" << endl;
    cout << "        Integration step size (default: 1e-3)" << endl;
    cout << " -m=MODE" << endl;
    cout << "        Locomotion mode (default: DRIVE)" << endl;
    cout << "        0: WALK, 1: SCULL, 2: INCHWORM, 3: DRIVE" << endl;
    cout << " -c=NUM_CYCLES" << endl;
    cout << "        Number of cycles for constant DBP force (default: 2)" << endl;
    cout << " -i=INCREMENT" << endl;
    cout << "        DBP factor increment (default: 0.04)" << endl;
    cout << " -l=LENGTH" << endl;
    cout << "        Length of terrain patch (default: 8.0)" << endl;
    cout << " --no-render" << endl;
    cout << "        Disable run-time rendering" << endl;
    cout << " --data-output" << endl;
    cout << "        Enable data output to file (one file per limb)" << endl;
    cout << " --image-output" << endl;
    cout << "        Enable capture for Irrlicht (ignored if no rendering)" << endl;
    cout << " --povray-output" << endl;
    cout << "        Enable generation of Pov-Ray postprocessing files" << endl;
    cout << " -? -h --help" << endl;
    cout << "        Print this message and exit." << endl;
    cout << endl;
}

bool GetProblemSpecs(int argc,
                     char** argv,
                     double& time_step,
                     robosimian::LocomotionMode& mode,
                     int& num_cycles,
                     double& dpb_incr,
                     double& terrain_length,
                     bool& render,
                     bool& data_output,
                     bool& image_output,
                     bool& povray_output) {
    // Default values
    time_step = 1e-3;
    render = true;
    mode = robosimian::LocomotionMode::DRIVE;
    num_cycles = 2;
    dpb_incr = 0.04;
    terrain_length = 8.0;
    data_output = false;
    povray_output = false;
    image_output = false;

    // Create the option parser and pass it the program arguments and the array of valid options.
    CSimpleOptA args(argc, argv, g_options);

    // Then loop for as long as there are arguments to be processed.
    while (args.Next()) {
        // Exit immediately if we encounter an invalid argument.
        if (args.LastError() != SO_SUCCESS) {
            cout << "Invalid argument: " << args.OptionText() << endl;
            ShowUsage(argv[0]);
            return false;
        }

        // Process the current argument.
        switch (args.OptionId()) {
            case OPT_HELP:
                ShowUsage(argv[0]);
                return false;
            case OPT_MODE: {
                auto mode_in = std::stoi(args.OptionArg());
                switch (mode_in) {
                    case 0:
                        mode = robosimian::LocomotionMode::WALK;
                        break;
                    case 1:
                        mode = robosimian::LocomotionMode::SCULL;
                        break;
                    case 2:
                        mode = robosimian::LocomotionMode::INCHWORM;
                        break;
                    case 3:
                        mode = robosimian::LocomotionMode::DRIVE;
                        break;
                    default:
                        cout << "Invalid locomotion mode" << endl;
                        ShowUsage(argv[0]);
                        return false;
                }
                break;
            }
            case OPT_STEP:
                time_step = std::stod(args.OptionArg());
                break;
            case OPT_CYCLES:
                num_cycles = std::stoi(args.OptionArg());
                break;
            case OPT_INCREMENT:
                dpb_incr = std::stod(args.OptionArg());
                break;
            case OPT_LENGTH:
                terrain_length = std::stod(args.OptionArg());
                break;
            case OPT_NO_RENDER:
                render = false;
                break;
            case OPT_DATA_OUT:
                data_output = true;
                break;
            case OPT_IMG_OUT:
                image_output = true;
                break;
            case OPT_POVRAY_OUT:
                povray_output = true;
                break;
        }
    }

    image_output = image_output && render;

    return true;
}
