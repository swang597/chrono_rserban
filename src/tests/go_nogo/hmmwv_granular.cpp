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

#include <iostream>
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
#include "chrono_parallel/physics/Ch3DOFContainer.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

#include "driver_model.h"
#include "monitoring.h"
#include "options.h"

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
std::vector<double> slope_values = {0, 10, 20, 30};

// Particle radius (in m)
std::vector<double> radius_values = {0.005, 0.010, 0.020, 0.030};

// Particle density (in Kg/m3)
std::vector<double> density_values = {1000, 2000, 3000};

// Inter-particle coefficient of friction
std::vector<double> friction_values = {0.2, 0.4, 0.6, 0.8, 1.0};

// Cohesion pressure (in kPa)
std::vector<double> cohesion_values = {1, 5, 10, 20};

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

// Use 3DOF particles?
//#define USE_PARTICLES

// Container
double hdimX = 6;
double hdimY = 1.75;
double hdimZ = 0.5;
double hthick = 0.05;

bool rough = true;

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
double alpha = 0;
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

double CreateContainer(ChSystem* system,  // containing system
                       double mu,         // coefficient of friction
                       double coh,        // cohesion (constant impulse)
                       double radius      // radius of roughness spheres (if positive)
                       ) {
    bool visible_walls = false;

    auto material = std::make_shared<ChMaterialSurfaceNSC>();
    material->SetFriction((float)mu);
    material->SetCohesion((float)coh);
    material->SetCompliance(1e-9f);

    auto ground = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
    ground->SetIdentifier(-1);
    ground->SetMass(1000);
    ground->SetBodyFixed(true);
    ground->SetCollide(true);

    ground->SetMaterialSurface(material);

    ground->GetCollisionModel()->ClearModel();

    // Attention: collision family for ground should be >= 5 (first values used in Chrono::Vehicle)
    ground->GetCollisionModel()->SetFamily(5);
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

    // If a positive radius was provided, create a "rough" surface
    if (radius > 0) {
        double d = 4 * radius;
        int nx = (int)std::floor(hdimX / d);
        int ny = (int)std::floor(hdimY / d);
        for (int ix = -nx; ix <= nx; ix++) {
            for (int iy = -ny; iy <= ny; iy++) {
                utils::AddSphereGeometry(ground.get(), radius, ChVector<>(ix * d, iy * d, radius));
            }
        }
    }

    ground->GetCollisionModel()->BuildModel();

    system->AddBody(ground);

    return (radius > 0) ? 2 * radius : 0;
}

// =============================================================================

int CreateParticles(ChSystemParallelNSC* system,  // containing system
                    double radius,                // particle radius
                    double rho,                   // particle density
                    double mu,                    // coefficient of friction
                    double coh,                   // cohesion (constant impulse)
                    double top_height             // top height of rigid container
                    ) {
#ifdef USE_PARTICLES
    auto particle_container = std::make_shared<ChParticleContainer>();
    system->Add3DOFContainer(particle_container);

    particle_container->kernel_radius = 2 * radius;
    particle_container->mass = rho * (4 * CH_C_PI / 3) * std::pow(radius, 3);

    particle_container->contact_mu = mu;
    particle_container->contact_cohesion = coh;
    particle_container->contact_compliance = 1e-9;

    particle_container->mu = mu;
    particle_container->cohesion = coh;
    particle_container->compliance = 1e-9;

    particle_container->alpha = alpha;

    particle_container->contact_recovery_speed = contact_recovery_speed;
    particle_container->collision_envelope = 0.1 * radius;

    std::vector<real3> pos;
    std::vector<real3> vel;

    double r = 1.01 * radius;
    ChVector<> hdims(hdimX - r, hdimY - r, 0);
    ChVector<> center(0, 0, top_height + 2 * r);

    utils::PDSampler<> sampler(2 * r);
    for (int il = 0; il < num_layers; il++) {
        utils::Generator::PointVector points = sampler.SampleBox(center, hdims);
        center.z() += 2 * r;
        for (int i = 0; i < points.size(); i++) {
            pos.push_back(real3(points[i].x(), points[i].y(), points[i].z()));
            vel.push_back(real3(0, 0, 0));
        }
    }

    particle_container->UpdatePosition(0);
    particle_container->AddBodies(pos, vel);

    return (int)pos.size();
#else
    // Create a material
    auto mat_g = std::make_shared<ChMaterialSurfaceNSC>();
    mat_g->SetFriction((float)mu);
    mat_g->SetCohesion((float)coh);
    mat_g->SetCompliance(1e-9f);

    // Create a particle generator and a mixture entirely made out of spheres
    utils::Generator gen(system);
    std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
    m1->setDefaultMaterial(mat_g);
    m1->setDefaultDensity(rho);
    m1->setDefaultSize(radius);

    // Set starting value for body identifiers
    gen.setBodyIdentifier(1);

    // Create particles in layers until reaching the desired number of particles
    double r = 1.01 * radius;
    ChVector<> hdims(hdimX - r, hdimY - r, 0);
    ChVector<> center(0, 0, top_height + 2 * r);

    for (int il = 0; il < num_layers; il++) {
        gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
        center.z() += 2 * r;
    }

    return gen.getTotalNumBodies();
#endif
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
    // ----------------------------
    // Parse command line arguments
    // ----------------------------

    int slope_idx = 0;
    int radius_idx = 2;
    int density_idx = 1;
    int friction_idx = 4;
    int cohesion_idx = 3;
    bool render = true;

    if (!GetProblemSpecs(argc, argv, slope_idx, radius_idx, density_idx, friction_idx, cohesion_idx, render)) {
        cout << "HUH?" << endl;
        return 1;
    }

    if (slope_idx < 0 || slope_idx >= slope_values.size()) {
        cout << "Slope index out of permissible range [0, " << slope_values.size() - 1 << "]" << endl;
        return 1;
    }
    if (radius_idx < 0 || radius_idx >= radius_values.size()) {
        cout << "Radius index out of permissible range [0, " << radius_values.size() - 1 << "]" << endl;
        return 1;
    }
    if (density_idx < 0 || density_idx >= density_values.size()) {
        cout << "Density index out of permissible range [0, " << density_values.size() - 1 << "]" << endl;
        return 1;
    }
    if (friction_idx < 0 || friction_idx >= friction_values.size()) {
        cout << "Friction index out of permissible range [0, " << friction_values.size() - 1 << "]" << endl;
        return 1;
    }
    if (cohesion_idx < 0 || cohesion_idx >= cohesion_values.size()) {
        cout << "Cohesion index out of permissible range [0, " << cohesion_values.size() - 1 << "]" << endl;
        return 1;
    }

    double slope = slope_values[slope_idx] * (CH_C_PI / 180);
    double r_g = radius_values[radius_idx];
    double rho_g = density_values[density_idx];
    double mu_g = friction_values[friction_idx];
    
    double coh_pressure = cohesion_values[cohesion_idx];
    double area = CH_C_PI * r_g * r_g;
    double coh_force = area * (coh_pressure * 1e3);
    double coh_g = coh_force * time_step;

    cout << "Set up" << endl;
    cout << "  Slope:    " << slope_idx << "  " << slope << endl;
    cout << "  Radius:   " << radius_idx << "  " << r_g << endl;
    cout << "  Density:  " << density_idx << "  " << rho_g << endl;
    cout << "  Friction: " << friction_idx << "  " << mu_g << endl;
    cout << "  Cohesion: " << cohesion_idx << "  " << coh_g << endl;

    // --------------------------
    // Create output directories
    // --------------------------
    std::ofstream outf;
    std::ofstream statsf;

    if (output) {
        if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
            cout << "Error creating directory " << out_dir << endl;
            return 1;
        }

        out_dir += "/" + std::to_string(slope_idx) + "_" + std::to_string(radius_idx) + "_" + std::to_string(density_idx) +
            "_" + std::to_string(friction_idx) + "_" + std::to_string(cohesion_idx);

        if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
            cout << "Error creating directory " << out_dir << endl;
            return 1;
        }

        // Open the output file stream
        outf.open(out_dir + "/results.dat", std::ios::out);
        outf.precision(7);
        outf << std::scientific;
    }

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
    system->GetSettings()->solver.alpha = alpha;
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

    double top_height = CreateContainer(system, mu_g, coh_g, (rough ? r_g : -1));
    int actual_num_particles = CreateParticles(system, r_g, rho_g, mu_g, coh_g, top_height);
    cout << "Created " << actual_num_particles << " particles." << endl;

    // Save current settings
    if (output) {
        statsf.open(out_dir + "/settings.dat", std::ios::out);
        statsf << "  Slope:    " << slope_idx << "  " << slope << " degrees" << endl;
        statsf << "  Radius:   " << radius_idx << "  " << r_g << " m" << endl;
        statsf << "  Density:  " << density_idx << "  " << rho_g << " kg/m3" << endl;
        statsf << "  Friction: " << friction_idx << "  " << mu_g << endl;
        statsf << "  Cohesion: " << cohesion_idx << "  " << coh_pressure << " kPa  " << coh_g << " N.s" << endl;
        statsf << endl;
        statsf << "  Num threads:   " << threads << endl;
        statsf << "  Num layers:    " << num_layers << endl;
        statsf << "  Num particles: " << actual_num_particles << endl;
        statsf << endl;
    }

#ifdef CHRONO_OPENGL
    // Initialize OpenGL
    if (render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(1280, 720, "HMMWV go/no-go", system);
        gl_window.SetCamera(ChVector<>(0, -10, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
        gl_window.SetRenderMode(opengl::WIREFRAME);
    }
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
        statsf << " Simulation time: " << exec_time << endl;

        statsf.close();
        outf.close();
    }

    delete hmmwv;
    delete driver;

    return 0;
}
