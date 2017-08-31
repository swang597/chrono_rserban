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
// Author: Radu Serban
// =============================================================================
//
// Chrono::Vehicle + Chrono::Parallel program for simulating a WVP vehicle
// on granular terrain.
//
// Contact uses non-smooth (DVI) formulation.
//
// Vehicle dimensions:
// - vehicle reference frame at front axle
// - wheel base: 4.039
// - wheel track: 2.066
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <cstdlib>
#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "chrono/ChConfig.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChSteeringController.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/terrain/GranularTerrain.h"

#include "chrono_models/vehicle/wvp/WVP.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::vehicle;
using namespace chrono::vehicle::wvp;

using std::cout;
using std::endl;

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

// Container
double hdimX = 3.5;
double hdimY = 1.5;
double hdimZ = 0.5;

// Minimum number of particles
unsigned int num_particles = 45000;

// Fixed base layer?
bool rough = false;

// Enable moving patch?
bool moving_patch = true;

// Particle radius (mm)
double radius = 20;

// Granular material density (kg/m3)
double rho = 2000;

// Coefficient of friction
double mu = 0.9;

// Cohesion pressure (kPa)
double coh = 400;

// Moving patch parameters
double buffer_distance = 1.5;
double shift_distance = 0.25;

// -----------------------------------------------------------------------------
// Specification of the vehicle model
// -----------------------------------------------------------------------------

// Vehicle horizontal offset
double horizontal_offset = 5;
double horizontal_pos = hdimX - horizontal_offset;

// Initial vehicle position, orientation, and forward velocity
ChVector<> initLoc(-horizontal_pos, 0, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);
double initSpeed = 0;

// Contact material properties for tires
float mu_t = 1.0f;
float cr_t = 0;

// Target speed
double mph_to_ms = 0.44704;
double target_speed = 5 * mph_to_ms;

// Drawbar pull force
double full_force = 1000;

// -----------------------------------------------------------------------------
// Timed events
// -----------------------------------------------------------------------------

// Total simulation duration.
double time_end = 7;

// Time when the vehicle is created (allows for granular material settling)
double time_create_vehicle = 0.05;//// 0.25;

// Delay before starting the engine (before setting the target speed)
double delay_start_engine = 0.25;
double time_start_engine = time_create_vehicle + delay_start_engine;

// Delay before start applying drawbar force
double delay_start_drawbar = 4;
double time_start_drawbar = time_start_engine + delay_start_drawbar;

// Time to ramp to full drawbar pull force
double delay_max_drawbar = 10;
double time_max_drawbar = time_start_drawbar + delay_max_drawbar;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Enable thread tuning?
bool thread_tuning = false;

// Number of threads
int threads = 8;

// Integration step size
double time_step = 1e-3;

// Solver settings
int max_iteration_bilateral = 100;
int max_iteration_normal = 0;
int max_iteration_sliding = 50;
int max_iteration_spinning = 0;

double tolerance = 1e-3;
double alpha = 0;
float contact_recovery_speed = 1000;

// Output
bool render = true;
bool output = true;
bool povray = false;
double povray_frequency = 50.0;
double output_frequency = 100.0;
std::string out_dir = GetChronoOutputPath() + "WVP_GRANULAR_TRACTIVE";
const std::string pov_dir = out_dir + "/POVRAY";

// =============================================================================

// Custom material composition law.
// Use the maximum coefficient of friction.
class CustomCompositionStrategy : public ChMaterialCompositionStrategy<real> {
  public:
    virtual real CombineFriction(real a1, real a2) const override { return std::max<real>(a1, a2); }
};

// =============================================================================

WVP* CreateVehicle(ChSystem* system, double vertical_offset, std::shared_ptr<ChFunction_Recorder> forceFunct);
ChPathFollowerDriver* CreateDriver(WVP* hmmwv);

void progressbar(unsigned int x, unsigned int n, unsigned int w = 50);
void TimingOutput(chrono::ChSystem* mSys, chrono::ChStreamOutAsciiFile* ofile = NULL);

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ------------------------
    // Convert input parameters
    // ------------------------

    double r_g = radius / 1000;
    double rho_g = rho;
    double mu_g = mu;

    double area = CH_C_PI * r_g * r_g;
    double coh_force = area * (coh * 1e3);
    double coh_g = coh_force * time_step;

    double envelope = 0.1 * r_g;

    // ---------------------------------
    // Create output directory and files
    // ---------------------------------

    std::ofstream ofile_sim;
    std::ofstream ofile_frc;
    std::string del("  ");

    if (output || povray) {
        if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
            cout << "Error creating directory " << out_dir << endl;
            return 1;
        }

        if (povray) {
            if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
                std::cout << "Error creating directory " << pov_dir << std::endl;
                return 1;
            }
        }

        // Open the output file stream
        if (output) {
            ofile_sim.open(out_dir + "/simulation.out", std::ios::out);
            ofile_frc.open(out_dir + "/tractive.out", std::ios::out);
        }
    }

    // -------------
    // Create system
    // -------------

    ChSystemParallelNSC* system = new ChSystemParallelNSC();
    system->Set_G_acc(ChVector<>(0, 0, -9.81));

    // Use a custom material property composition strategy.
    // This ensures that tire-terrain interaction always uses the same coefficient of friction.
    std::unique_ptr<CustomCompositionStrategy> strategy(new CustomCompositionStrategy);
    system->SetMaterialCompositionStrategy(std::move(strategy));

    int max_threads = CHOMPfunctions::GetNumProcs();
    if (threads > max_threads)
        threads = max_threads;
    system->SetParallelThreadNumber(threads);
    CHOMPfunctions::SetNumThreads(threads);
#pragma omp parallel
#pragma omp master
    cout << "Using " << CHOMPfunctions::GetNumThreads() << " threads" << endl;

    system->GetSettings()->perform_thread_tuning = thread_tuning;

    // --------------------
    // Edit system settings
    // --------------------

    system->GetSettings()->solver.tolerance = tolerance;
    system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    system->GetSettings()->solver.compute_N = false;
    system->GetSettings()->solver.alpha = alpha;
    system->GetSettings()->solver.cache_step_length = true;
    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    system->GetSettings()->solver.bilateral_clamp_speed = 1e8;
    system->GetSettings()->min_threads = threads;
    system->ChangeSolverType(SolverType::BB);

    system->GetSettings()->collision.collision_envelope = envelope;
    system->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    system->GetSettings()->collision.bins_per_axis = vec3(100, 30, 2);
    system->GetSettings()->collision.fixed_bins = true;

    // Specify active box.
    system->GetSettings()->collision.use_aabb_active = false;
    system->GetSettings()->collision.aabb_min = real3(-1.1 * hdimX, -1.1 * hdimY, 0);
    system->GetSettings()->collision.aabb_max = real3(+1.1 * hdimX, +1.1 * hdimY, 10 * hdimZ);

    // ------------------
    // Create the terrain
    // ------------------

    GranularTerrain terrain(system);
    terrain.SetContactFrictionCoefficient((float)mu_g);
    terrain.SetContactCohesion((float)coh_g);
    terrain.SetCollisionEnvelope(envelope / 5);
    if (rough) {
        int nx = (int)std::round((2 * hdimX) / (4 * r_g));
        int ny = (int)std::round((2 * hdimY) / (4 * r_g));
        terrain.EnableRoughSurface(nx, ny);
    }
    terrain.EnableVisualization(true);
    terrain.EnableVerbose(true);

    terrain.Initialize(ChVector<>(0, 0, 0), 2 * hdimX, 2 * hdimY, num_particles, r_g, rho_g);
    uint actual_num_particles = terrain.GetNumParticles();

    std::cout << "Number of particles: " << actual_num_particles << std::endl;

    // Save parameters and problem setup to output file
    if (output) {
        ofile_sim << "# Radius (mm):     " << radius << endl;
        ofile_sim << "# Density (kg/m3): " << rho << endl;
        ofile_sim << "# Friction:        " << mu << endl;
        ofile_sim << "# Cohesion (kPa):  " << coh << endl;
        ofile_sim << "# " << endl;
        ofile_sim << "# Num threads:     " << threads << endl;
        ofile_sim << "# Num particles:   " << actual_num_particles << endl;
        ofile_sim << "# " << endl;

        ofile_sim.precision(7);
        ofile_sim << std::scientific;

        ofile_frc << "time throttle MotorSpeed VehicleSpeed EngineTorque ";
        ofile_frc << "WheelAngVelX_1 WheelAngVelY_1 WheeleAngVelZ_1 ";
        ofile_frc << "WheelAngVelX_2 WheelAngVelY_1 WheeleAngVelZ_2 ";
        ofile_frc << "WheelAngVelX_3 WheelAngVelY_1 WheeleAngVelZ_3 ";
        ofile_frc << "WheelAngVelX_4 WheelAngVelY_1 WheeleAngVelZ_4 ";
        ofile_frc << "WheelLongSlip_1 WheelLongSlip_2 WheelLongSlip_3 WheelLongSlip_4 ";
        ofile_frc << "DrawBarForce" << endl;

        ofile_frc.precision(6);
        ofile_frc << std::scientific;
    }

#ifdef CHRONO_OPENGL
    // Initialize OpenGL
    if (render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(1280, 720, "HMMWV acceleration", system);
        gl_window.SetCamera(ChVector<>(-horizontal_pos, -5, 0), ChVector<>(-horizontal_pos, 0, 0), ChVector<>(0, 0, 1));
        gl_window.SetRenderMode(opengl::WIREFRAME);
    }
#endif

    // -----------------
    // Recorder function
    // -----------------

    auto forceFunct = std::make_shared<ChFunction_Recorder>();
    forceFunct->AddPoint(0, 0);
    forceFunct->AddPoint(time_start_drawbar, 0);
    forceFunct->AddPoint(time_max_drawbar, -full_force);
    forceFunct->AddPoint(time_end, -full_force);

    // ---------------
    // Simulation loop
    // ---------------

    WVP* wvp = nullptr;
    ChPathFollowerDriver* driver = nullptr;

    // Number of simulation steps between render/output frames
    int povray_steps = (int)std::ceil((1 / povray_frequency) / time_step);
    int output_steps = (int)std::ceil((1 / output_frequency) / time_step);

    double time = 0;
    int sim_frame = 0;
    int povray_frame = 0;
    int out_frame = 0;
    int next_povray_frame = 0;
    int next_out_frame = 0;
    bool engine_on = false;
    double exec_time = 0;

    while (time < time_end) {
        // POV-Ray output
        if (povray && sim_frame == next_povray_frame) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), povray_frame + 1);
            utils::WriteShapesPovray(system, filename);

            std::cout << "Povray output at time " << time << "  " << filename << std::endl;
            povray_frame++;
            next_povray_frame += povray_steps;
        }

        // Create the vehicle
        if (!wvp && time > time_create_vehicle) {
            cout << time << "    Create vehicle" << endl;

            double max_height = terrain.GetHeight(0, 0);
            wvp = CreateVehicle(system, max_height, forceFunct);
            driver = CreateDriver(wvp);

            // Enable moving patch, based on vehicle location
            if (moving_patch)
                terrain.EnableMovingPatch(wvp->GetChassisBody(), buffer_distance, shift_distance);

            next_out_frame = sim_frame;
        }

        // Synchronize terrain system
        terrain.Synchronize(time);

        if (wvp) {
            // Set target speed
            if (!engine_on && time > time_start_engine) {
                cout << "    Start engine" << endl;
                driver->SetDesiredSpeed(target_speed);
                engine_on = true;
            }

            // Extract current driver inputs
            double steering_input = driver->GetSteering();
            double braking_input = driver->GetBraking();
            double throttle_input = driver->GetThrottle();

            // Synchronize vehicle systems
            driver->Synchronize(time);
            wvp->Synchronize(time, steering_input, braking_input, throttle_input, terrain);

            // Save output
            if (output && sim_frame == next_out_frame) {
                ChVector<> pv = wvp->GetChassisBody()->GetFrame_REF_to_abs().GetPos();
                ChVector<> vv = wvp->GetChassisBody()->GetFrame_REF_to_abs().GetPos_dt();
                ChVector<> av = wvp->GetChassisBody()->GetFrame_REF_to_abs().GetPos_dtdt();

                ofile_sim << system->GetChTime() << del;
                ofile_sim << throttle_input << del << steering_input << del;

                ofile_sim << pv.x() << del << pv.y() << del << pv.z() << del;
                ofile_sim << vv.x() << del << vv.y() << del << vv.z() << del;
                ofile_sim << av.x() << del << av.y() << del << av.z() << del;

                ofile_sim << endl;

                ofile_frc << system->GetChTime() << del;
                ofile_frc << wvp->GetPowertrain().GetMotorSpeed() << del;
                ofile_frc << wvp->GetVehicle().GetVehicleSpeed() << del;
                ofile_frc << wvp->GetPowertrain().GetMotorTorque() << del;

                for (int i = 0; i < 4; i++) {
                    ChVector<> avel = wvp->GetVehicle().GetWheelAngVel(i);
                    ofile_frc << avel.x() << del << avel.y() << del << avel.z() << del;
                }

                for (int i = 0; i < 4; i++) {
                    ofile_frc << wvp->GetTire(i)->GetLongitudinalSlip() << del;
                }

                ofile_frc << forceFunct->Get_y(system->GetChTime()) << del;

                ofile_frc << endl;

                out_frame++;
                next_out_frame += output_steps;
            }

            // Advance vehicle systems
            driver->Advance(time_step);
            wvp->Advance(time_step);
        } else {
            // Advance system state (no vehicle created yet)
            system->DoStepDynamics(time_step);
        }

#ifdef CHRONO_OPENGL
        if (render) {
            opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
            if (gl_window.Active())
                gl_window.Render();
            else
                break;
        }
#endif

        // Display performance metrics
        TimingOutput(system);

        // Update counters.
        time += time_step;
        sim_frame++;
        exec_time += system->GetTimerStep();
    }

    // Final stats
    cout << "==================================" << endl;
    cout << "Simulation time:   " << exec_time << endl;

    if (output) {
        ofile_sim << "# " << endl;
        ofile_sim << "# Simulation time (s): " << exec_time << endl;
        ofile_sim.close();
    }

    delete wvp;
    delete driver;

    return 0;
}

// =============================================================================

WVP* CreateVehicle(ChSystem* system, double vertical_offset, std::shared_ptr<ChFunction_Recorder> forceFunct) {
    auto wvp = new WVP(system);

    wvp->SetContactMethod(ChMaterialSurface::NSC);
    wvp->SetChassisFixed(false);
    wvp->SetInitPosition(ChCoordsys<>(initLoc + ChVector<>(0, 0, vertical_offset), initRot));
    wvp->SetInitFwdVel(initSpeed);
    wvp->SetTireType(TireModelType::RIGID);
    wvp->SetTireStepSize(time_step);

    wvp->Initialize();

    wvp->SetChassisVisualizationType(VisualizationType::NONE);
    wvp->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    wvp->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    wvp->SetWheelVisualizationType(VisualizationType::MESH);
    wvp->SetTireVisualizationType(VisualizationType::NONE);

    wvp->GetVehicle().SetStepsize(time_step);

    auto drawForce = std::make_shared<ChForce>();
    wvp->GetChassisBody()->AddForce(drawForce);
    drawForce->SetMode(ChForce::ForceType::FORCE);
    drawForce->SetFrame(ChForce::ReferenceFrame::BODY);
    drawForce->SetAlign(ChForce::AlignmentFrame::WORLD_DIR);
    drawForce->SetVrelpoint(wvp->GetChassis()->GetLocalPosCOM());
    drawForce->SetF_x(forceFunct);
    drawForce->SetF_y(std::make_shared<ChFunction_Const>(0));
    drawForce->SetF_z(std::make_shared<ChFunction_Const>(0));

    return wvp;
}

ChPathFollowerDriver* CreateDriver(WVP* wvp) {
    double height = initLoc.z();
    auto path = StraightLinePath(ChVector<>(-2 * hdimX, 0, height), ChVector<>(20 * hdimX, 0, height));

    auto driver = new ChPathFollowerDriver(wvp->GetVehicle(), path, "straight_line", 0.0);
    driver->GetSteeringController().SetGains(0.5, 0, 0);
    driver->GetSteeringController().SetLookAheadDistance(5);
    driver->GetSpeedController().SetGains(1.4, 0.1, 0.0);

    driver->Initialize();

    return driver;
}

// =============================================================================

// Utility function for displaying an ASCII progress bar for the quantity x
// which must be a value between 0 and n. The width 'w' represents the number
// of '=' characters corresponding to 100%.
void progressbar(unsigned int x, unsigned int n, unsigned int w) {
    if ((x != n) && (x % (n / 100 + 1) != 0))
        return;

    float ratio = x / (float)n;
    unsigned int c = (unsigned int)(ratio * w);

    std::cout << std::setw(3) << (int)(ratio * 100) << "% [";
    for (unsigned int x = 0; x < c; x++)
        std::cout << "=";
    for (unsigned int x = c; x < w; x++)
        std::cout << " ";
    std::cout << "]\r" << std::flush;
}

// Utility function to print to console a few important step statistics
void TimingOutput(chrono::ChSystem* mSys, chrono::ChStreamOutAsciiFile* ofile) {
    double TIME = mSys->GetChTime();
    double STEP = mSys->GetTimerStep();
    double BROD = mSys->GetTimerCollisionBroad();
    double NARR = mSys->GetTimerCollisionNarrow();
    double SOLVER = mSys->GetTimerSolver();
    double UPDT = mSys->GetTimerUpdate();
    double RESID = 0;
    int REQ_ITS = 0;
    int BODS = mSys->GetNbodies();
    int CNTC = mSys->GetNcontacts();
    if (chrono::ChSystemParallel* parallel_sys = dynamic_cast<chrono::ChSystemParallel*>(mSys)) {
        RESID = std::static_pointer_cast<chrono::ChIterativeSolverParallel>(mSys->GetSolver())->GetResidual();
        REQ_ITS = std::static_pointer_cast<chrono::ChIterativeSolverParallel>(mSys->GetSolver())->GetTotalIterations();
        BODS = parallel_sys->GetNbodies();
        CNTC = parallel_sys->GetNcontacts();
    }

    if (ofile) {
        char buf[200];
        sprintf(buf, "%8.5f  %7.4f  %7.4f  %7.4f  %7.4f  %7.4f  %7d  %7d  %7d  %7.4f\n", TIME, STEP, BROD, NARR, SOLVER,
                UPDT, BODS, CNTC, REQ_ITS, RESID);
        *ofile << buf;
    }

    printf("   %8.5f | %7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.4f\n", TIME, STEP, BROD, NARR,
           SOLVER, UPDT, BODS, CNTC, REQ_ITS, RESID);
}
