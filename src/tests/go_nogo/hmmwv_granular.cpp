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
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChSystemDescriptorParallel.h"
#include "chrono_parallel/collision/ChNarrowphaseRUtils.h"

//#undef CHRONO_OPENGL
#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

#include "monitoring.h"
#include "event_queue.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::endl;

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

// Container
double hdimX = 4;
double hdimY = 1.75;
double hdimZ = 0.5;
double hthick = 0.25;

// Granular material
int Id_g = 1;
double r_g = 0.02;
double rho_g = 2500;
double vol_g = (4.0 / 3) * CH_C_PI * r_g * r_g * r_g;
double mass_g = rho_g * vol_g;
ChVector<> inertia_g = 0.4 * mass_g * r_g * r_g * ChVector<>(1, 1, 1);

float mu_g = 1;
float cohesion_g = 200;
float cr_g = 0;

unsigned int num_particles = 10000;

// -----------------------------------------------------------------------------
// Specification of the vehicle model
// -----------------------------------------------------------------------------

// Type of wheel/tire (controls both contact and visualization)
enum WheelType { CYLINDRICAL, LUGGED };
WheelType wheel_type = CYLINDRICAL;

// Initial vehicle position and orientation
ChVector<> initLoc(-hdimX + 2.5, 0, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);

// Contact material properties for tires
float mu_t = 0.8f;
float cohesion_t = 0;
float cr_t = 0;

// -----------------------------------------------------------------------------
// Collision families
// -----------------------------------------------------------------------------

int coll_fam_c = 5;
int coll_fam_g = 2;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 20;
bool thread_tuning = false;

// Total simulation duration.
double time_end = 7;

// Duration of the "hold time" (vehicle chassis fixed and no driver inputs).
// This can be used to allow the granular material to settle.
double time_hold = 0.05;

// Integration step size
double time_step = 1e-3;

// Solver settings
int max_iteration_bilateral = 100;
int max_iteration_normal = 0;
int max_iteration_sliding = 50;
int max_iteration_spinning = 0;

double tolerance = 1e-3;
float contact_recovery_speed = 40;

// Periodically monitor maximum bilateral constraint violation
bool monitor_bilaterals = false;
int bilateral_frame_interval = 100;

// Output
const std::string out_dir = "../HMMWV_GO_NOGO";

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

void CreateContainer(ChSystem* system) {
    bool visible_walls = false;

    auto material = std::make_shared<ChMaterialSurface>();
    material->SetFriction(mu_g);
    material->SetCompliance(1e-9);
    material->SetCohesion(cohesion_g);

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

int CreateParticles(ChSystem* system) {
    // Create a material
    auto mat_g = std::make_shared<ChMaterialSurface>();
    mat_g->SetFriction(mu_g);
    mat_g->SetRestitution(cr_g);
    mat_g->SetCohesion(cohesion_g);

    // Create a particle generator and a mixture entirely made out of spheres
    utils::Generator gen(system);
    std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
    m1->setDefaultMaterial(mat_g);
    m1->setDefaultDensity(rho_g);
    m1->setDefaultSize(r_g);

    // Set starting value for body identifiers
    gen.setBodyIdentifier(Id_g);

    // Create particles in layers until reaching the desired number of particles
    double r = 1.01 * r_g;
    ChVector<> hdims(hdimX - r, hdimY - r, 0);
    ChVector<> center(0, 0, 2 * r);

    while (gen.getTotalNumBodies() < num_particles) {
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

    hmmwv->SetContactMethod(ChMaterialSurfaceBase::DVI);
    hmmwv->SetChassisFixed(false);
    hmmwv->SetInitPosition(ChCoordsys<>(initLoc + ChVector<>(0, 0, vertical_offset), initRot));
    ////hmmwv->SetInitFwdVel(10);
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

ChBezierCurve* CreatePath() {
    std::vector<ChVector<>> points;
    std::vector<ChVector<>> inCV;
    std::vector<ChVector<>> outCV;

    points.push_back(ChVector<>(-10 * hdimX, 0, 1));
    inCV.push_back(ChVector<>(-10 * hdimX, 0, 1));
    outCV.push_back(ChVector<>(-9 * hdimX, 0, 1));

    points.push_back(ChVector<>(10 * hdimX, 0, 1));
    inCV.push_back(ChVector<>(9 * hdimX, 0, 1));
    outCV.push_back(ChVector<>(10 * hdimX, 0, 1));

    auto path = new ChBezierCurve(points, inCV, outCV);

    return path;
}

ChPathFollowerDriver* CreateDriver(HMMWV_Full* hmmwv, ChBezierCurve* path) {
    double look_ahead_dist = 5;
    double Kp_steering = 0.5;
    double Ki_steering = 0;
    double Kd_steering = 0;
    auto driver = new ChPathFollowerDriver(hmmwv->GetVehicle(), path, "straight_line", 0);
    driver->GetSteeringController().SetLookAheadDistance(look_ahead_dist);
    driver->GetSteeringController().SetGains(Kp_steering, Ki_steering, Kd_steering);

    driver->Initialize();

    return driver;
}

/*
void CreateVehicle(ChSystem* system, double vertical_offset, HMMWV_Full** hmmwv, ChBezierCurve** path, ChPathFollowerDriver** driver) {
    // Create and initialize HMMWV vehicle
    *hmmwv = new HMMWV_Full(system);

    (*hmmwv)->SetContactMethod(ChMaterialSurfaceBase::DVI);
    (*hmmwv)->SetChassisFixed(false);
    (*hmmwv)->SetInitPosition(ChCoordsys<>(initLoc + ChVector<>(0, 0, vertical_offset), initRot));
    (*hmmwv)->SetPowertrainType(PowertrainModelType::SHAFTS);
    (*hmmwv)->SetDriveType(DrivelineType::AWD);
    (*hmmwv)->SetTireType(TireModelType::RIGID);

    (*hmmwv)->Initialize();

    (*hmmwv)->SetChassisVisualizationType(VisualizationType::NONE);
    (*hmmwv)->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    (*hmmwv)->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    (*hmmwv)->SetWheelVisualizationType(VisualizationType::PRIMITIVES);
    (*hmmwv)->SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Straight-line Bezier path
    std::vector<ChVector<>> points;
    std::vector<ChVector<>> inCV;
    std::vector<ChVector<>> outCV;

    points.push_back(ChVector<>(-10 * hdimX, 0, 1));
    inCV.push_back(ChVector<>(-10 * hdimX, 0, 1));
    outCV.push_back(ChVector<>(-9 * hdimX, 0, 1));

    points.push_back(ChVector<>(10 * hdimX, 0, 1));
    inCV.push_back(ChVector<>(9 * hdimX, 0, 1));
    outCV.push_back(ChVector<>(10 * hdimX, 0, 1));

    *path = new ChBezierCurve(points, inCV, outCV);

    // Create and initialize path-following driver
    double look_ahead_dist = 5;
    double Kp_steering = 0.5;
    double Ki_steering = 0;
    double Kd_steering = 0;
    *driver = new ChPathFollowerDriver((*hmmwv)->GetVehicle(), *path, "straight_line", 0);
    (*driver)->GetSteeringController().SetLookAheadDistance(look_ahead_dist);
    (*driver)->GetSteeringController().SetGains(Kp_steering, Ki_steering, Kd_steering);
    (*driver)->Initialize();
}
*/

// =============================================================================

void fun1(int i, double d) {
    cout << i << "  " << d << endl;
}

void fun2(const std::string& s) {
    cout << s << endl;
}

class Foo {
public:
    void operator()() { cout << "Functor " << endl; }
};

int main(int argc, char* argv[]) {
    /*
    // Set path to Chrono and Chrono::Vehicle data directories
    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath(CHRONO_VEHICLE_DATA_DIR);
    */

    // --------------------------
    // Create output directories
    // --------------------------

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    // -------------------------
    // Create system and vehicle
    // -------------------------

    ChSystemParallelDVI* system = new ChSystemParallelDVI();

    system->Set_G_acc(ChVector<>(0, 0, -9.81));

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
    system->GetSettings()->collision.bins_per_axis = vec3(20, 20, 10);
    system->GetSettings()->collision.fixed_bins = true;

    // Specify active box.
    system->GetSettings()->collision.use_aabb_active = true;
    system->GetSettings()->collision.aabb_min = real3(-1.1 * hdimX, -1.1 * hdimY, 0);
    system->GetSettings()->collision.aabb_max = real3(+1.1 * hdimX, +1.1 * hdimY, 10 * hdimZ);

    // -------------------
    // Create the terrain.
    // -------------------

    CreateContainer(system);
    int actual_num_particles = CreateParticles(system);
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
    ChBezierCurve* path = nullptr;
    ChPathFollowerDriver* driver = nullptr;
    FlatTerrain terrain(0);

    double time = 0;
    int sim_frame = 0;
    int next_out_frame = 0;
    double exec_time = 0;
    int num_contacts = 0;

    while (time < time_end) {
        // At the end of the hold time, create the vehicle.
        if (!hmmwv && time > time_hold) {
            cout << endl << "Create vehicle at t = " << time << endl;
            double max_height = FindHighestParticle(system);
            hmmwv = CreateVehicle(system, max_height);
            path = CreatePath();
            driver = CreateDriver(hmmwv, path);
        }

        if (hmmwv) {
            double steering_input = driver->GetSteering();
            double braking_input = 0;
            double throttle_input = 1;

            driver->Synchronize(time);
            hmmwv->Synchronize(time, steering_input, braking_input, throttle_input, terrain);

            ChVector<> pos = hmmwv->GetChassis()->GetCOMPos();
            cout << pos.z() << endl;

            driver->Advance(time_step);
            hmmwv->Advance(time_step);
        }
        else {
            system->DoStepDynamics(time_step);
        }

#ifdef CHRONO_OPENGL
        if (gl_window.Active()) {
            gl_window.Render();
        } else
            break;
#endif

        ////progressbar(out_steps + sim_frame - next_out_frame + 1, out_steps);
        TimingOutput(system);

        // Periodically display maximum constraint violation
        if (monitor_bilaterals && sim_frame % bilateral_frame_interval == 0) {
            std::vector<double> cvec;
            cout << "  Max. violation = " << system->CalculateConstraintViolation(cvec) << endl;
        }

        // Update counters.
        time += time_step;
        sim_frame++;
        exec_time += system->GetTimerStep();
        num_contacts += system->GetNcontacts();
    }

    // Final stats
    cout << "==================================" << endl;
    cout << "Simulation time:   " << exec_time << endl;
    cout << "Number of threads: " << threads << endl;

    delete hmmwv;
    delete path;
    delete driver;

    return 0;
}
