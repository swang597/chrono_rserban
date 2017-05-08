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
// Chrono::Vehicle + Chrono::Parallel program for simulating a HMMWV vehicle
// on granular terrain.
//
// Contact uses non-smooth (DVI) formulation.
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChSystemDescriptorParallel.h"
#include "chrono_parallel/collision/ChNarrowphaseRUtils.h"

//#undef CHRONO_OPENGL
#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

#include "driver_model.h"
#include "monitoring.h"
#include "event_queue.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::endl;

// -----------------------------------------------------------------------------
// Input parameters
// -----------------------------------------------------------------------------

// Terrain slope (in degrees)
std::vector<double> slope_values = { 0, 10, 20, 30 };

// Particle radius (in m)
std::vector<double> radius_values = {0.005, 0.010, 0.020, 0.030};

// Particle density (in Kg/m3)
std::vector<double> density_values = {1000, 2000, 3000};

// Inter-particle coefficient of friction
std::vector<float> friction_values = {0.2f, 0.4f, 0.6f, 0.8f, 1.0f};

// Cohesion force
std::vector<float> cohesion_values = { 50, 100, 150, 200 };

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

// Container
double hdimX = 6;
double hdimY = 1.75;
double hdimZ = 0.5;
double hthick = 0.05;

// Number of layers of granular material
int num_layers = 4;

// -----------------------------------------------------------------------------
// Specification of the vehicle model
// -----------------------------------------------------------------------------

// Type of wheel/tire (controls both contact and visualization)
enum WheelType { CYLINDRICAL, LUGGED };
WheelType wheel_type = CYLINDRICAL;

// Vehicle horizontal offset
double horizontal_offset = 2.5;
double horizontal_pos = hdimX - horizontal_offset;

// Initial vehicle position, orientation, and forward velocity
ChVector<> initLoc(-horizontal_pos, 0, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);
double initSpeed = 0;

// Contact material properties for tires
float mu_t = 0.8f;
float cr_t = 0;

// -----------------------------------------------------------------------------
// Collision families
// -----------------------------------------------------------------------------

int coll_fam_c = 5;
int coll_fam_g = 2;

// -----------------------------------------------------------------------------
// Timed events
// -----------------------------------------------------------------------------

// Total simulation duration.
double time_end = 7;

// Time when the vehicle is created (allows for granular material settling)
double time_create_vehicle = 0.25;

// Duration before starting to apply throttle (allows for vehicle settling)
double delay_start_engine = 0.25;
double time_start_engine = time_create_vehicle + delay_start_engine;

// Delay before throttle reaches maximum (linear ramp)
double delay_max_throttle = 0.5;
double time_max_throttle = time_start_engine + delay_max_throttle;

// Time when terrain is pitched (rotate gravity)
double time_pitch = time_start_engine;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 20;
bool thread_tuning = false;

// Integration step size
double time_step = 1e-3;

// Solver settings
int max_iteration_bilateral = 100;
int max_iteration_normal = 0;
int max_iteration_sliding = 50;
int max_iteration_spinning = 0;

double tolerance = 1e-3;
float contact_recovery_speed = 1000;

// Output
bool output = true;
double output_frequency = 100.0;
std::string out_dir = "../HMMWV_GO_NOGO";

// =============================================================================

/*

// Callback class for specifying rigid tire contact model.
// This version uses cylindrical contact shapes.
class MyCylindricalTire : public ChTireContactCallback {
  public:
    virtual void onCallback(std::shared_ptr<ChBody> wheelBody) {
        wheelBody->SetCollisionModel(std::make_shared<collision::ChCollisionModelParallel>());

        wheelBody->GetCollisionModel()->ClearModel();
        wheelBody->GetCollisionModel()->AddCylinder(0.46, 0.46, 0.127);
        wheelBody->GetCollisionModel()->BuildModel();

        wheelBody->GetCollisionModel()->SetFamily(coll_fam_t);

        wheelBody->GetMaterialSurfaceDEM()->SetFriction(mu_t);
        wheelBody->GetMaterialSurfaceDEM()->SetRestitution(cr_t);

        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0.127, 0);
        cyl->GetCylinderGeometry().p2 = ChVector<>(0, -0.127, 0);
        cyl->GetCylinderGeometry().rad = 0.46;
        wheelBody->AddAsset(cyl);
    }
};

// Callback class for specifying rigid tire contact model.
// This version uses a collection of convex contact shapes (meshes).
class MyLuggedTire : public ChTireContactCallback {
  public:
    MyLuggedTire() {
        std::string lugged_file("hmmwv/lugged_wheel_section.obj");
        geometry::ChTriangleMeshConnected lugged_mesh;
        utils::LoadConvexMesh(vehicle::GetDataFile(lugged_file), lugged_mesh, lugged_convex);
        num_hulls = lugged_convex.GetHullCount();
    }

    virtual void onCallback(std::shared_ptr<ChBody> wheelBody) {
        auto coll_model = std::make_shared<collision::ChCollisionModelParallel>();
        wheelBody->SetCollisionModel(coll_model);

        coll_model->ClearModel();

        // Assemble the tire contact from 15 segments, properly offset.
        // Each segment is further decomposed in convex hulls.
        for (int iseg = 0; iseg < 15; iseg++) {
            ChQuaternion<> rot = Q_from_AngAxis(iseg * 24 * CH_C_DEG_TO_RAD, VECT_Y);
            for (int ihull = 0; ihull < num_hulls; ihull++) {
                std::vector<ChVector<> > convexhull;
                lugged_convex.GetConvexHullResult(ihull, convexhull);
                coll_model->AddConvexHull(convexhull, VNULL, rot);
            }
        }

        // Add a cylinder to represent the wheel hub.
        coll_model->AddCylinder(0.223, 0.223, 0.126);

        coll_model->BuildModel();

        coll_model->SetFamily(coll_fam_t);

        wheelBody->GetMaterialSurfaceDEM()->SetFriction(mu_t);
        wheelBody->GetMaterialSurfaceDEM()->SetRestitution(cr_t);
    }

  private:
    ChConvexDecompositionHACDv2 lugged_convex;
    int num_hulls;
};

*/

// =============================================================================

void CreateContainer(ChSystem* system, float mu_g, float coh_g) {
    bool visible_walls = false;

    auto material = std::make_shared<ChMaterialSurfaceNSC>();
    material->SetFriction(mu_g);
    material->SetCompliance(1e-9f);
    material->SetCohesion(coh_g);

    auto ground = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
    ground->SetIdentifier(-1);
    ground->SetMass(1000);
    ground->SetBodyFixed(true);
    ground->SetCollide(true);

    ground->SetMaterialSurface(material);

    ground->GetCollisionModel()->ClearModel();

    ground->GetCollisionModel()->SetFamily(coll_fam_c);
    ground->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(WheeledCollisionFamily::TIRES);

    // Bottom box
    utils::AddBoxGeometry(ground.get(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -hthick),
                          ChQuaternion<>(1, 0, 0, 0), true);
    // Left box
    utils::AddBoxGeometry(ground.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
                          ChVector<>(0, hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);
    // Right box
    utils::AddBoxGeometry(ground.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
                          ChVector<>(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);

    // Front box
    utils::AddBoxGeometry(ground.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
                          ChVector<>(hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);
    // Rear box
    utils::AddBoxGeometry(ground.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
                          ChVector<>(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);

    ground->GetCollisionModel()->BuildModel();

    system->AddBody(ground);
}

// =============================================================================

int CreateParticles(ChSystem* system, double r_g, double rho_g, float mu_g, float coh_g) {
    // Create a material
    auto mat_g = std::make_shared<ChMaterialSurfaceNSC>();
    mat_g->SetFriction(mu_g);
    mat_g->SetRestitution(0);
    mat_g->SetCohesion(coh_g);

    // Create a particle generator and a mixture entirely made out of spheres
    utils::Generator gen(system);
    std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
    m1->setDefaultMaterial(mat_g);
    m1->setDefaultDensity(rho_g);
    m1->setDefaultSize(r_g);

    // Set starting value for body identifiers
    gen.setBodyIdentifier(1);

    // Create particles in layers until reaching the desired number of particles
    double r = 1.01 * r_g;
    ChVector<> hdims(hdimX - r, hdimY - r, 0);
    ChVector<> center(0, 0, 2 * r);

    for (int il = 0; il < num_layers; il++) {
        gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
        center.z() += 2 * r;
    }

    return gen.getTotalNumBodies();
}

double FindHighestParticle(ChSystem* system) {
    double highest = 0;
    for (size_t i = 0; i < system->Get_bodylist()->size(); ++i) {
        auto body = (*system->Get_bodylist())[i];
        if (body->GetIdentifier() > 0 && body->GetPos().z() > highest)
            highest = body->GetPos().z();
    }
    return highest;
}

// =============================================================================

HMMWV_Full* CreateVehicle(ChSystem* system, double vertical_offset) {
    auto hmmwv = new HMMWV_Full(system);

    hmmwv->SetContactMethod(ChMaterialSurface::NSC);
    hmmwv->SetChassisFixed(false);
    hmmwv->SetInitPosition(ChCoordsys<>(initLoc + ChVector<>(0, 0, vertical_offset), initRot));
    hmmwv->SetInitFwdVel(initSpeed);
    hmmwv->SetPowertrainType(PowertrainModelType::SHAFTS);
    hmmwv->SetDriveType(DrivelineType::AWD);
    hmmwv->SetTireType(TireModelType::RIGID);

    hmmwv->Initialize();

    hmmwv->SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetWheelVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetTireVisualizationType(VisualizationType::PRIMITIVES);

    return hmmwv;
}

HMMWV_Driver* CreateDriver(HMMWV_Full* hmmwv) {
    // Create the straigh-line path
    double height = initLoc.z();

    std::vector<ChVector<>> points;
    std::vector<ChVector<>> inCV;
    std::vector<ChVector<>> outCV;

    points.push_back(ChVector<>(-10 * hdimX, 0, height));
    inCV.push_back(ChVector<>(-10 * hdimX, 0, height));
    outCV.push_back(ChVector<>(-9 * hdimX, 0, height));

    points.push_back(ChVector<>(10 * hdimX, 0, height));
    inCV.push_back(ChVector<>(9 * hdimX, 0, height));
    outCV.push_back(ChVector<>(10 * hdimX, 0, height));

    auto path = std::make_shared<ChBezierCurve>(points, inCV, outCV);

    double look_ahead_dist = 5;
    double Kp_steering = 0.5;
    double Ki_steering = 0;
    double Kd_steering = 0;
    auto driver = new HMMWV_Driver(hmmwv->GetVehicle(), path, time_start_engine, time_max_throttle);
    driver->SetLookAheadDistance(look_ahead_dist);
    driver->SetGains(Kp_steering, Ki_steering, Kd_steering);

    driver->Initialize();

    return driver;
}

// =============================================================================

int main(int argc, char* argv[]) {
    // --------------------------
    // Set up simulation scenario
    // --------------------------

    int slope_index = 0;
    int radius_index = 2;
    int density_index = 1;
    int friction_index = 4;
    int cohesion_index = 3;

    double slope = slope_values[slope_index] * (CH_C_PI / 180);
    double r_g = radius_values[radius_index];
    double rho_g = density_values[density_index];
    float coh_g = cohesion_values[cohesion_index];
    float mu_g = friction_values[friction_index];

    cout << "Set up" << endl;
    cout << "  Slope:    " << slope_index << "  " << slope << endl;
    cout << "  Radius:   " << radius_index << "  " << r_g << endl;
    cout << "  Density:  " << density_index << "  " << rho_g << endl;
    cout << "  Friction: " << friction_index << "  " << mu_g << endl;
    cout << "  Cohesion: " << cohesion_index << "  " << coh_g << endl;

    // --------------------------
    // Create output directories
    // --------------------------

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    out_dir += "/" + std::to_string(slope_index) + "_" + std::to_string(radius_index) + "_" +
               std::to_string(density_index) + "_" + std::to_string(friction_index) + "_" +
               std::to_string(cohesion_index);

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    // Open the output file stream
    std::ofstream outf;
    outf.open(out_dir + "/results.dat", std::ios::out);
    outf.precision(7);
    outf << std::scientific;

    // Delimiter in output file
    std::string del("  ");

    // -------------------------
    // Create system and vehicle
    // -------------------------
    ChVector<> gravity(0, 0, -9.81);
    ChVector<> gravityR = ChMatrix33<>(slope, ChVector<>(0, 1, 0)) * gravity;

    ChSystemParallelNSC* system = new ChSystemParallelNSC();
    system->Set_G_acc(gravity);

    // ----------------------
    // Set number of threads.
    // ----------------------
    int max_threads = omp_get_num_procs();
    if (threads > max_threads)
        threads = max_threads;
    system->SetParallelThreadNumber(threads);
    omp_set_num_threads(threads);
    cout << "Using " << threads << " threads" << endl;

    system->GetSettings()->perform_thread_tuning = thread_tuning;

    // ---------------------
    // Edit system settings
    // ---------------------

    system->GetSettings()->solver.tolerance = tolerance;
    system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    system->GetSettings()->solver.compute_N = false;
    system->GetSettings()->solver.alpha = 0;
    system->GetSettings()->solver.cache_step_length = true;
    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    system->GetSettings()->solver.bilateral_clamp_speed = 1e8;
    system->GetSettings()->min_threads = threads;
    system->ChangeSolverType(SolverType::BB);
    ////system->SetLoggingLevel(LoggingLevel::LOG_INFO);
    ////system->SetLoggingLevel(LoggingLevel::LOG_TRACE);

    system->GetSettings()->collision.collision_envelope = 0.1 * r_g;
    system->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    system->GetSettings()->collision.bins_per_axis = vec3(100, 30, 2);
    system->GetSettings()->collision.fixed_bins = true;

    // Specify active box.
    system->GetSettings()->collision.use_aabb_active = false;
    system->GetSettings()->collision.aabb_min = real3(-1.1 * hdimX, -1.1 * hdimY, 0);
    system->GetSettings()->collision.aabb_max = real3(+1.1 * hdimX, +1.1 * hdimY, 10 * hdimZ);

    // -------------------
    // Create the terrain.
    // -------------------

    CreateContainer(system, mu_g, coh_g);
    int actual_num_particles = CreateParticles(system, r_g, rho_g, mu_g, coh_g);
    cout << "Created " << actual_num_particles << " particles." << endl;

#ifdef CHRONO_OPENGL
    // Initialize OpenGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "HMMWV go/no-go", system);
    gl_window.SetCamera(ChVector<>(0, -10, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);
#endif

    // -------------------
    // Simulation loop
    // -------------------
    HMMWV_Full* hmmwv = nullptr;
    HMMWV_Driver* driver = nullptr;
    FlatTerrain terrain(0);

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil((1 / output_frequency) / time_step);

    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int next_out_frame = 0;
    double exec_time = 0;

    bool is_pitched = false;
    double x_pos = -horizontal_pos;

    while (time < time_end && x_pos < horizontal_pos) {
        // Create the vehicle
        if (!hmmwv && time > time_create_vehicle) {
            cout << time << "    Create vehicle" << endl;

            double max_height = FindHighestParticle(system);
            hmmwv = CreateVehicle(system, max_height);
            driver = CreateDriver(hmmwv);

            next_out_frame = sim_frame;
        }

        // Rotate gravity vector
        if (!is_pitched && time > time_pitch) {
            cout << time << "    Pitch: " << gravityR.x() << " " << gravityR.y() << " " << gravityR.z() << endl;
            system->Set_G_acc(gravityR);
            is_pitched = true;
        }

        if (hmmwv) {
            // Extract current driver inputs
            double steering_input = driver->GetSteering();
            double braking_input = driver->GetBraking();
            double throttle_input = driver->GetThrottle();

            // Synchronize vehicle systems
            driver->Synchronize(time);
            hmmwv->Synchronize(time, steering_input, braking_input, throttle_input, terrain);

            // Update vehicle x position
            x_pos = hmmwv->GetChassis()->GetPos().x();

            // Save output
            if (output && sim_frame == next_out_frame) {
                ChVector<> pv = hmmwv->GetChassisBody()->GetFrame_REF_to_abs().GetPos();
                ChVector<> vv = hmmwv->GetChassisBody()->GetFrame_REF_to_abs().GetPos_dt();
                ChVector<> av = hmmwv->GetChassisBody()->GetFrame_REF_to_abs().GetPos_dtdt();

                ChVector<> v0 = hmmwv->GetVehicle().GetWheelLinVel(0);
                ChVector<> v1 = hmmwv->GetVehicle().GetWheelLinVel(1);
                ChVector<> v2 = hmmwv->GetVehicle().GetWheelLinVel(2);
                ChVector<> v3 = hmmwv->GetVehicle().GetWheelLinVel(3);

                outf << system->GetChTime() << del;
                outf << throttle_input << del << steering_input << del;

                outf << pv.x() << del << pv.y() << del << pv.z() << del;
                outf << vv.x() << del << vv.y() << del << vv.z() << del;
                outf << av.x() << del << av.y() << del << av.z() << del;

                outf << v0.x() << del << v0.y() << del << v0.z() << del;
                outf << v1.x() << del << v1.y() << del << v1.z() << del;
                outf << v2.x() << del << v2.y() << del << v2.z() << del;
                outf << v3.x() << del << v3.y() << del << v3.z() << del;

                outf << endl;

                out_frame++;
                next_out_frame += output_steps;
            }

            // Advance vehicle systems
            driver->Advance(time_step);
            hmmwv->Advance(time_step);
        } else {
            // Advance system state (no vehicle created yet)
            system->DoStepDynamics(time_step);
        }

#ifdef CHRONO_OPENGL
        if (gl_window.Active()) {
            gl_window.Render();
        } else
            break;
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
    cout << "Number of threads: " << threads << endl;

    delete hmmwv;
    delete driver;

    return 0;
}
