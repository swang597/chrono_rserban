#include <cmath>
#include <cstdio>
#include <vector>

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono_opengl/ChOpenGLWindow.h"

#include "robosimian.h"

using namespace chrono;
using namespace chrono::collision;

double time_step = 1e-3;

int main(int argc, char* argv[]) {
    // Create system
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
    my_sys.GetSettings()->solver.max_iteration_sliding = 200;
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

    // Create the robot
    robosimian::RoboSimian robot(&my_sys, true, true);

    ////robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI)));

    robot.SetVisualizationTypeChassis(robosimian::VisualizationType::MESH);
    robot.SetVisualizationTypeSled(robosimian::VisualizationType::MESH);
    robot.SetVisualizationTypeLimbs(robosimian::VisualizationType::MESH);

    // Initialize OpenGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "RoboSimian", &my_sys);
    gl_window.SetCamera(ChVector<>(2, 2, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    // Run simulation for specified time
    int sim_frame = 0;

    while (gl_window.Active()) {
        gl_window.Render();

        double time = my_sys.GetChTime();
        double A = CH_C_PI / 6;
        double freq = 2;
        double val = 0.5 * A * (1 - std::cos(CH_C_2PI * freq * time));
        robot.Activate(robosimian::FR, "joint2", time, val);
        robot.Activate(robosimian::RL, "joint5", time, val);

        my_sys.DoStepDynamics(time_step);
        
        if (my_sys.GetNcontacts() > 0) {
            robot.ReportContacts();
        }

        sim_frame++;
    }

    return 0;
}
