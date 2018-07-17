#include <omp.h>
#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono_opengl/ChOpenGLWindow.h"

using namespace chrono;
using namespace chrono::collision;


int main(int argc, char* argv[]) {
    // BASIC SETUP
    double time_step = 1e-3;
    double time_end = 25;
    double time_start = 1;

    double out_fps = 50;

    double tolerance = 1e-3;
    unsigned int max_iteration = 100;

    enum ForceType { BODY_FORCE, ACCUMULATOR_FORCE };
    ForceType force_type = BODY_FORCE;

    bool floating = false;

    float friction = 0.5f;

    // Create system
    ////ChSystemParallelSMC my_sys;
    ChSystemParallelNSC my_sys;

    if (floating)
        my_sys.Set_G_acc(ChVector<double>(0, 0, 0));
    else
        my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));

    int num_threads = 1;
    my_sys.SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

    // Set solver parameters
    my_sys.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    my_sys.GetSettings()->solver.tolerance = tolerance;

    my_sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_R;
    my_sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    my_sys.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    my_sys.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    // Create the plate
    auto plate = std::shared_ptr<ChBody>(my_sys.NewBody());

    plate->SetIdentifier(0);
    plate->SetPos(ChVector<>(0, 0, 0));
    plate->SetBodyFixed(true);
    plate->SetCollide(true);

    plate->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(plate.get(), ChVector<>(4, 0.5, 0.1));
    plate->GetCollisionModel()->BuildModel();

    my_sys.AddBody(plate);

    // Create the ball
    double mass = 1;
    double radius = 0.15;
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

    auto ball = std::shared_ptr<ChBody>(my_sys.NewBody());

    ball->SetIdentifier(2);
    ball->SetMass(mass);
    ball->SetInertiaXX(inertia);
    ball->SetPos(ChVector<>(-3, 0, 1));
    ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ball->SetBodyFixed(false);
    ball->SetCollide(true);
  
    ball->GetCollisionModel()->ClearModel();
    utils::AddSphereGeometry(ball.get(), radius);
    ball->GetCollisionModel()->BuildModel();

    my_sys.AddBody(ball);

    switch (ball->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            ball->GetMaterialSurfaceNSC()->SetFriction(friction);
            ball->GetMaterialSurfaceNSC()->SetRestitution(0);
            break;
        case ChMaterialSurface::SMC:
            ball->GetMaterialSurfaceSMC()->SetFriction(friction);
            ball->GetMaterialSurfaceSMC()->SetRestitution(0);
            break;
    }

    // Force (ramp to maximum and then back to 0)
    double fullForce = 1;
    ChFunction_Recorder forceFunct;
    forceFunct.AddPoint(0, 0);
    forceFunct.AddPoint(time_start, 0);
    forceFunct.AddPoint(time_start + 1, fullForce);
    forceFunct.AddPoint(time_start + 2, fullForce);
    forceFunct.AddPoint(time_start + 3, 0);
    forceFunct.AddPoint(time_end, 0);

    if (force_type == BODY_FORCE) {
        auto drawForce = std::make_shared<ChForce>();
        ball->AddForce(drawForce);
        drawForce->SetMode(ChForce::ForceType::FORCE); // force or torque
        drawForce->SetFrame(ChForce::ReferenceFrame::BODY);
        drawForce->SetAlign(ChForce::AlignmentFrame::WORLD_DIR);
        drawForce->SetVrelpoint(ChVector<>(0, 0, 0));
        drawForce->SetF_x(std::make_shared<ChFunction_Recorder>(forceFunct));
        drawForce->SetF_y(std::make_shared<ChFunction_Const>(0));
        drawForce->SetF_z(std::make_shared<ChFunction_Const>(0));
    }

    // Create the visualization window
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Collide flag", &my_sys);
    gl_window.SetCamera(ChVector<>(0, -4, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
    gl_window.SetRenderMode(opengl::WIREFRAME);

    // Run simulation for specified time
    int out_steps = static_cast<int>(std::ceil((1 / time_step) / out_fps));
    int sim_frame = 0;

    while (my_sys.GetChTime() < time_end) {
        if (force_type == ACCUMULATOR_FORCE) {
            double Fx = forceFunct.Get_y(my_sys.GetChTime());
            ball->Empty_forces_accumulators();
            ball->Accumulate_force(ChVector<>(Fx, 0, 0), ball->GetPos(), false);
        }
        my_sys.DoStepDynamics(time_step);
        sim_frame++;

        std::cout << "Vx: " << ball->GetPos_dt().x() << std::endl;

        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        if (gl_window.Active()) {
            gl_window.Render();
        } else {
            return 1;
        }
    }

    return 0;
}
