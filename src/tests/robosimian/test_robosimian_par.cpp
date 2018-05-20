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
// RoboSimian on granular terrain
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <vector>

#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono_vehicle/terrain/GranularTerrain.h"

#include "chrono_opengl/ChOpenGLWindow.h"

#include "robosimian.h"

using namespace chrono;
using namespace chrono::collision;

// Integration step size
double step_size = 1e-4;

// OpenGL rendering?
bool render = true;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Drop the robot on terrain
bool drop = true;

// Timed events
double time_create_terrain = 0.5;
double duration_settle_terrain = 1.0;
double time_release = time_create_terrain + duration_settle_terrain;
double duration_settle_robot = 0.5;
double time_offset = time_release + duration_settle_robot;
double duration_sim = 10;
double time_end = time_offset + duration_sim;

// Output directories
const std::string out_dir = GetChronoOutputPath() + "ROBOSIMIAN_PAR";
const std::string pov_dir = out_dir + "/POVRAY";

// POV-Ray output
bool povray_output = false;

// =============================================================================

class RobotDriverCallback : public robosimian::Driver::PhaseChangeCallback {
  public:
    RobotDriverCallback(robosimian::RoboSimian* robot) : m_robot(robot), m_start_x(0), m_start_time(0) {}
    virtual void OnPhaseChange(robosimian::Driver::Phase old_phase, robosimian::Driver::Phase new_phase) override;

    double GetDistance() const { return m_robot->GetChassisPos().x() - m_start_x; }
    double GetDuration() const { return m_robot->GetSystem()->GetChTime() - m_start_time; }
    double GetAvgSpeed() const { return GetDistance() / GetDuration(); }

    double m_start_x;
    double m_start_time;

  private:
    robosimian::RoboSimian* m_robot;
};

void RobotDriverCallback::OnPhaseChange(robosimian::Driver::Phase old_phase, robosimian::Driver::Phase new_phase) {
    if (new_phase == robosimian::Driver::CYCLE && old_phase != robosimian::Driver::CYCLE) {
        m_start_x = m_robot->GetChassisPos().x();
        m_start_time = m_robot->GetSystem()->GetChTime();
    }
}

// =============================================================================

double CreateTerrain(ChSystem& sys, double x, double z) {
    double r_g = 0.0075;
    double rho_g = 2000;
    double coh = 400e3;
    double area = CH_C_PI * r_g * r_g;
    double coh_force = area * coh;
    double coh_g = coh_force * step_size;

    std::cout << "x = " << x << "  z = " << z << std::endl;

    double length = 6;
    double width = 3;
    unsigned int num_layers = 5;
    ChVector<> center(length / 2 + x - 1.5, 0, z - num_layers * 2.1 * r_g);

    vehicle::GranularTerrain terrain(&sys);
    terrain.SetContactFrictionCoefficient(0.7f);
    terrain.SetContactCohesion((float)coh_g);
    terrain.SetCollisionEnvelope(0.1 * r_g / 5);

    terrain.EnableVisualization(true);
    terrain.Initialize(center, length, width, num_layers, r_g, rho_g);

    std::cout << "Generated " << terrain.GetNumParticles() << " particles";

    return (length + x - 2 * 1.5);
}

// =============================================================================

int main(int argc, char* argv[]) {
    // -------------
    // Create system
    // -------------

    ////ChSystemParallelSMC my_sys;
    ChSystemParallelNSC my_sys;
    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));
    ////my_sys.Set_G_acc(ChVector<double>(0, 0, 0));

    int threads = 2;
    int max_threads = CHOMPfunctions::GetNumProcs();
    if (threads > max_threads)
        threads = max_threads;
    my_sys.SetParallelThreadNumber(threads);
    CHOMPfunctions::SetNumThreads(threads);

    my_sys.GetSettings()->solver.tolerance = 1e-3;
    my_sys.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    my_sys.GetSettings()->solver.max_iteration_normal = 0;
    my_sys.GetSettings()->solver.max_iteration_sliding = 100;
    my_sys.GetSettings()->solver.max_iteration_spinning = 0;
    my_sys.GetSettings()->solver.max_iteration_bilateral = 100;
    my_sys.GetSettings()->solver.compute_N = false;
    my_sys.GetSettings()->solver.alpha = 0;
    my_sys.GetSettings()->solver.cache_step_length = true;
    my_sys.GetSettings()->solver.use_full_inertia_tensor = false;
    my_sys.GetSettings()->solver.contact_recovery_speed = 1000;
    my_sys.GetSettings()->solver.bilateral_clamp_speed = 1e8;
    my_sys.GetSettings()->min_threads = threads;

    my_sys.GetSettings()->collision.collision_envelope = 0.01;
    my_sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    my_sys.ChangeSolverType(SolverType::BB);

    // -----------------------
    // Create RoboSimian robot
    // -----------------------

    robosimian::RoboSimian robot(&my_sys, true, true);

    // Ensure wheels are actuated in ANGLE mode (required for Chrono::Parallel)
    robot.SetMotorActuationMode(robosimian::ActuationMode::ANGLE);

    ////robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI)));

    robot.SetVisualizationTypeChassis(robosimian::VisualizationType::MESH);
    robot.SetVisualizationTypeSled(robosimian::VisualizationType::MESH);
    robot.SetVisualizationTypeLimbs(robosimian::VisualizationType::MESH);

    // -----------------------------------
    // Create a driver and attach to robot
    // -----------------------------------

    ////auto driver = std::make_shared<robosimian::Driver>(
    ////    "",                                                           // start input file
    ////    GetChronoDataFile("robosimian/actuation/walking_cycle.txt"),  // cycle input file
    ////    "",                                                           // stop input file
    ////    true);
    ////auto driver = std::make_shared<robosimian::Driver>(
    ////    GetChronoDataFile("robosimian/actuation/sculling_start.txt"),  // start input file
    ////    GetChronoDataFile("robosimian/actuation/sculling_cycle.txt"),  // cycle input file
    ////    GetChronoDataFile("robosimian/actuation/sculling_stop.txt"),   // stop input file
    ////    true);
    ////auto driver = std::make_shared<robosimian::Driver>(
    ////    GetChronoDataFile("robosimian/actuation/inchworming_start.txt"),  // start input file
    ////    GetChronoDataFile("robosimian/actuation/inchworming_cycle.txt"),  // cycle input file
    ////    GetChronoDataFile("robosimian/actuation/inchworming_stop.txt"),   // stop input file
    ////    true);
    auto driver = std::make_shared<robosimian::Driver>(
        GetChronoDataFile("robosimian/actuation/driving_start.txt"),  // start input file
        GetChronoDataFile("robosimian/actuation/driving_cycle.txt"),  // cycle input file
        GetChronoDataFile("robosimian/actuation/driving_stop.txt"),   // stop input file
        true);

    RobotDriverCallback cbk(&robot);
    driver->RegisterPhaseChangeCallback(&cbk);

    driver->SetOffset(time_offset);
    robot.SetDriver(driver);

    // -----------------
    // Initialize OpenGL
    // -----------------

    if (render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(1280, 720, "RoboSimian", &my_sys);
        gl_window.SetCamera(ChVector<>(2, -2, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
        gl_window.SetRenderMode(opengl::WIREFRAME);
    }

    // -----------------------------
    // Initialize output directories
    // -----------------------------

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }

    // ---------------------------------
    // Run simulation for specified time
    // ---------------------------------

    int render_steps = (int)std::ceil(render_step_size / step_size);
    int sim_frame = 0;
    int render_frame = 0;

    bool terrain_created = false;
    bool robot_released = false;
    double x_max = 0;

    while (true) {
        double time = my_sys.GetChTime();
        double x = robot.GetChassisPos().x();

        if (time >= time_end) {
            std::cout << "Reached final time: " << time << std::endl;
            break;
        }

        if (drop) {
            if (!terrain_created && time > time_create_terrain) {
                // Robot position and bottom point
                double z = robot.GetWheelPos(robosimian::FR).z() - 0.15;
                // Create terrain
                std::cout << "Time: " << time << "  CREATE TERRAIN" << std::endl;
                x_max = CreateTerrain(my_sys, x, z);
                terrain_created = true;
            }
            if (!robot_released && time > time_release) {
                std::cout << "Time: " << time << "  RELEASE ROBOT" << std::endl;
                robot.GetChassis()->GetBody()->SetBodyFixed(false);
                robot_released = true;
            }

            if (robot_released && x > x_max) {
                std::cout << "Time: " << time << " Reached maximum distance" << std::endl;
                break;
            }
        }

        // Output POV-Ray data
        if (povray_output && sim_frame % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteShapesPovray(&my_sys, filename);
            render_frame++;
        }

        ////double time = my_sys.GetChTime();
        ////double A = CH_C_PI / 6;
        ////double freq = 2;
        ////double val = 0.5 * A * (1 - std::cos(CH_C_2PI * freq * time));
        ////robot.Activate(robosimian::FR, "joint2", time, val);
        ////robot.Activate(robosimian::RL, "joint5", time, val);
        ////robot.Activate(robosimian::FL, "joint8", time, -0.4 * time);

        robot.DoStepDynamics(step_size);

        ////if (my_sys.GetNcontacts() > 0) {
        ////    robot.ReportContacts();
        ////}

        if (render) {
            opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
            if (gl_window.Active()) {
                gl_window.Render();
            } else {
                break;
            }
        }

        sim_frame++;
    }

    return 0;
}
