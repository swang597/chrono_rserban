#include <omp.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono_parallel/physics/ChSystemParallel.h>

#include <chrono/utils/ChUtilsCreators.h>
#include <chrono/utils/ChUtilsSamplers.h>

using std::cout;
using std::endl;

using namespace chrono;
using namespace chrono::collision;

std::string file_name;
std::string file_name_prefix("shear_results");
std::string csv_header("t,x,f");

// UNIT SYSTEM: SI (kg, m, s)
// Conversion factors for specifications
double ft2in = 12;
double in2m = 0.0254;
double min2s = 60;
double g2kg = 1.0 / 1000.0;

double grav = 9.81;  // Magnitude of gravity in the downward direction

// NOTE true radius in 2-100 um
double sphere_inflation = 20;                      // Multiplier on particle radius
double sphere_radius = sphere_inflation * 100e-6;  // Particle radius 100um = 100e-6m
double sphere_density = 400;                       // Particle density 0.4 g/cm^3 = 400 kg/m^3
double sphere_mass = 4 * CH_C_PI * sphere_radius * sphere_radius * sphere_radius * sphere_density / 3;

// Particle material: Parameters to tune
double sphere_mu = 0.18;  // Coefficient of friction
double sphere_cr = 0.87;  // Coefficient of restitution

double uncompressed_volume =
    0.003873 * ft2in * ft2in * ft2in * in2m * in2m * in2m;  // 0.003873 ft^3 sample before compression

// Interior dimensions of shear chamber
double box_dim_X = 2.416 * in2m;  // 2.416in shear box diameter
double box_dim_Y = 2.416 * in2m;  // 2.416in shear box diameter
double box_dim_Z = 2 * 2.416 * in2m;
// double box_dim_Z = uncompressed_volume / (box_dim_X * box_dim_Y); // TODO

double box_mass = 100;

// Box material
double box_mu = sphere_mu;
double box_cr = sphere_cr;

double dt = 1e-4;  // Simulation timestep
double tolerance = 0.1;
int max_iteration_normal = 10;
int max_iteration_sliding = 10;
int max_iteration_spinning = 10;
int max_iteration_bilateral = 10;
double contact_recovery_speed = 10e30;

bool clamp_bilaterals = false;
double bilateral_clamp_speed = 0.1;

double out_interval = 1e-2;  // Prints a status at this interval
unsigned int out_steps = out_interval / dt;

double plate_area = box_dim_X * box_dim_Y;
// Confining pressures expressed as mass per area kg/m2
// NOTE confining mass selected at runtime to allow batch runs
double confining_masses[] = {25, 100, 250, 500, 1000, 2000, 2500};
double plate_mass;  // confining_mass * plate_area;  // Plate mass necessary to generate the confining pressure

double settling_time = 0.25;

double stop_velocity = 1;            // Stop the compression phase when the plate reaches this vertical velocity
double max_compression_time = 0.35;  // Alternative stop condition for ending the compression phase

double shear_velocity_inflation = 1000;  // Multiplier on shear_velocity speed for shorter simulation
double shear_velocity = shear_velocity_inflation * 0.01 * in2m / min2s;  // X velocity of top shear section 0.01in/min
double shear_displacement = 0.25 * in2m;                                 // X displacement at which the test ends
// TODO shear_displacement

double box_thick = shear_displacement / 2;  // Thickness of walls so that no material spills during shearing

void AddBox(ChSystemParallel& m_sys, std::shared_ptr<ChBody>& plate, std::shared_ptr<ChBody>& top) {
    auto box_mat = std::make_shared<ChMaterialSurfaceNSC>();
    box_mat->SetFriction(box_mu);
    box_mat->SetRestitution(box_cr);

    double hx = box_dim_X / 2;
    double hy = box_dim_Y / 2;
    double hz = box_dim_Z / 2;

    double hthick = box_thick / 2;

    // Plate for applying confining pressure
    plate = std::shared_ptr<ChBody>(m_sys.NewBody());
    plate->SetPos(ChVector<>(0, 0, hz + hthick));
    plate->SetMass(plate_mass);
    plate->SetMaterialSurface(box_mat);
    plate->SetBodyFixed(false);
    plate->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(plate.get(), ChVector<>(hx, hy, hthick));
    plate->GetCollisionModel()->BuildModel();
    plate->SetCollide(true);
    plate->GetCollisionModel()->SetFamily(1);
    plate->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
    m_sys.AddBody(plate);

    // Top half of shear box
    ChVector<> pos(0, 0, 0);
    top = std::shared_ptr<ChBody>(m_sys.NewBody());
    top->SetPos(pos);
    top->SetMass(box_mass / 2);
    top->SetMaterialSurface(box_mat);
    top->SetBodyFixed(true);
    top->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(top.get(), ChVector<>(hx, hy, hthick), ChVector<>(0, 0, hz + hthick));              // Top
    utils::AddBoxGeometry(top.get(), ChVector<>(hthick, hy, hz / 2), ChVector<>(-(hx + hthick), 0, hz / 2));  // Low X
    utils::AddBoxGeometry(top.get(), ChVector<>(hthick, hy, hz / 2), ChVector<>(hx + hthick, 0, hz / 2));     // High X
    utils::AddBoxGeometry(top.get(), ChVector<>(hx, hthick, hz / 2), ChVector<>(0, -(hy + hthick), hz / 2));  // Low Y
    utils::AddBoxGeometry(top.get(), ChVector<>(hx, hthick, hz / 2), ChVector<>(0, hy + hthick, hz / 2));     // High Y
    top->GetCollisionModel()->BuildModel();
    top->SetCollide(true);
    top->GetCollisionModel()->SetFamily(1);
    top->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
    m_sys.AddBody(top);

    // Enforce a slider relationship between the load plate and the top of the shear box
    auto link = std::make_shared<ChLinkLockPrismatic>();
    link->Initialize(plate, top, ChCoordsys<>(ChVector<>(0, 0, 0)));
    m_sys.AddLink(link);

    // Bottom half of shear box
    auto bot = std::shared_ptr<ChBody>(m_sys.NewBody());
    bot->SetPos(pos);
    bot->SetMass(box_mass / 2);
    bot->SetMaterialSurface(box_mat);
    bot->SetBodyFixed(true);
    bot->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bot.get(), ChVector<>(hx, hy, hthick), ChVector<>(0, 0, -(hz + hthick)));            // Bottom
    utils::AddBoxGeometry(bot.get(), ChVector<>(hthick, hy, hz / 2), ChVector<>(-(hx + hthick), 0, -hz / 2));  // Low X
    utils::AddBoxGeometry(bot.get(), ChVector<>(hthick, hy, hz / 2), ChVector<>(hx + hthick, 0, -hz / 2));     // High X
    utils::AddBoxGeometry(bot.get(), ChVector<>(hx, hthick, hz / 2), ChVector<>(0, -(hy + hthick), -hz / 2));  // Low Y
    utils::AddBoxGeometry(bot.get(), ChVector<>(hx, hthick, hz / 2), ChVector<>(0, hy + hthick, -hz / 2));     // High Y
    bot->GetCollisionModel()->BuildModel();
    bot->SetCollide(true);
    bot->GetCollisionModel()->SetFamily(1);
    bot->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
    m_sys.AddBody(bot);
}

unsigned int AddParticles(ChSystemParallelNSC& m_sys, ChVector<> box_center, ChVector<> hdims) {
    auto sphere_mat = std::make_shared<ChMaterialSurfaceNSC>();
    sphere_mat->SetFriction(sphere_mu);
    sphere_mat->SetRestitution(sphere_cr);

    utils::PDSampler<> sampler(2.01 * sphere_radius);
    auto points = sampler.SampleBox(box_center, ChVector<>(hdims.x(), hdims.y(), hdims.z()));
    for (unsigned int i = 0; i < points.size(); i++) {
        auto sphere = std::shared_ptr<ChBody>(m_sys.NewBody());

        sphere->SetMaterialSurface(sphere_mat);
        sphere->SetMass(sphere_mass);
        // sphere->SetInertiaXX(inertia);  // TODO
        sphere->SetPos(points[i]);
        sphere->SetRot(ChQuaternion<>(1, 0, 0, 0));
        sphere->SetBodyFixed(false);
        sphere->SetCollide(true);

        sphere->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(sphere.get(), sphere_radius);
        sphere->GetCollisionModel()->BuildModel();

        m_sys.AddBody(sphere);
    }

    return points.size();
}

void WriteParticles(ChSystemParallelNSC& m_sys, std::string file_name) {
    std::ofstream ostream;
    ostream.open(file_name);
    ostream << "x,y,z,U" << endl;
    for (auto body : m_sys.Get_bodylist()) {
        ChVector<> pos = body->GetPos();
        ostream << pos.x() << "," << pos.y() << "," << pos.z() << "," << body->GetPos_dt().Length() << endl;
    }

    ostream.close();
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        cout << "usage: " << argv[0] << " <pressure_index: 0-6>" << endl;
        return 1;
    }
    unsigned int approx_particles = (box_dim_X - 2 * sphere_radius) * (box_dim_Y - 2 * sphere_radius) *
                                    (box_dim_Z - 2 * sphere_radius) /
                                    (8 * sphere_radius * sphere_radius * sphere_radius);
    plate_mass = confining_masses[std::stoi(argv[1])] * plate_area;
    file_name = file_name_prefix + argv[1] + ".csv";

    cout << "Unit system: SI (mks)" << endl;
    cout << "Pressure index: " << std::stoi(argv[1]) << endl;
    cout << "Plate mass: " << plate_mass << endl;
    cout << "Data file: " << file_name << endl;
    cout << "Particle radius: " << sphere_radius << endl;
    cout << "Box dimensions: " << box_dim_X << " X " << box_dim_Y << " X " << box_dim_Z << endl;
    cout << "Approximate number of particles: " << approx_particles << endl;
    cout << "Shear velocity: " << shear_velocity << endl;
    cout << "Shear displacement: " << shear_displacement << endl;

    ChSystemParallelNSC m_sys;

    m_sys.Set_G_acc(ChVector<>(0, 0, -grav));

    unsigned int num_threads = omp_get_num_procs();
    m_sys.SetParallelThreadNumber(num_threads);
    omp_set_num_threads(num_threads);
    m_sys.GetSettings()->max_threads = num_threads;
    cout << "OpenMP threads: " << num_threads << endl;

    m_sys.GetSettings()->solver.use_full_inertia_tensor = false;
    m_sys.GetSettings()->solver.tolerance = tolerance;
    m_sys.GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    m_sys.GetSettings()->solver.clamp_bilaterals = clamp_bilaterals;
    m_sys.GetSettings()->solver.bilateral_clamp_speed = bilateral_clamp_speed;

    m_sys.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    m_sys.GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    m_sys.GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    m_sys.GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    m_sys.GetSettings()->solver.alpha = 0;
    m_sys.GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    m_sys.ChangeSolverType(SolverType::APGD);

    m_sys.GetSettings()->collision.collision_envelope = 0.01 * sphere_radius;

    m_sys.GetSettings()->collision.bins_per_axis = vec3(20, 20, 20);  // TODO heuristic
    m_sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    // Add containing box and compression plate
    std::shared_ptr<ChBody> plate;
    std::shared_ptr<ChBody> top;
    AddBox(m_sys, plate, top);

    // Add spherical particles
    unsigned int num_particles = AddParticles(
        m_sys, ChVector<>(0, 0, 0),
        ChVector<>(box_dim_X / 2 - sphere_radius, box_dim_Y / 2 - sphere_radius, box_dim_Z / 2 - sphere_radius));

    cout << "Running settling..." << endl;
    plate->SetBodyFixed(true);
    double m_time = 0;
    unsigned int step = 0;
    while (m_time < settling_time) {
        m_sys.DoStepDynamics(dt);
        if (step % out_steps == 0) {
            cout << std::setprecision(4) << "Time: " << m_time << endl;
            WriteParticles(m_sys, std::string("points_compression") + std::to_string(step) + std::string(".csv"));
        }
        step++;
        m_time += dt;
    }

    double highest = -1000000;
    for (unsigned int i = 3; i < 3 + num_particles; i++) {
        highest = std::max(m_sys.Get_bodylist()[i]->GetPos().z(), highest);
    }

    num_particles += AddParticles(m_sys, ChVector<>(0, 0, (highest + box_dim_Z / 2) / 2),
                                  ChVector<>(box_dim_X / 2 - sphere_radius, box_dim_Y / 2,
                                             (box_dim_Z / 2 - sphere_radius - (highest + sphere_radius)) / 2));

    cout << "Actual number of particles: " << num_particles << endl;

    // Run compression phase until plate reaches a small enough velocity, or until
    cout << "Running compression..." << endl;
    plate->SetBodyFixed(false);
    m_time = 0;
    do {
        m_sys.DoStepDynamics(dt);
        if (step % out_steps == 0) {
            cout << std::setprecision(4) << "Time: " << m_time << endl;
            cout << std::setprecision(4) << "\tPlate height: " << plate->GetPos().z() << endl;
            WriteParticles(m_sys, std::string("points_compression") + std::to_string(step) + std::string(".csv"));
        }
        m_time += dt;
        step++;
    } while (plate->GetPos_dt().z() < stop_velocity && m_time < max_compression_time);

    ChVector<> top_compress_pos = top->GetPos();
    ChVector<> plate_compress_pos = plate->GetPos();

    // Run shearing for specified displacement
    cout << endl << "Running shear test..." << endl;

    std::stringstream ss;
    ss << csv_header << endl;

    step = 0;
    m_time = 0;
    while (top->GetPos().x() < shear_displacement) {
        top->SetPos(top_compress_pos + ChVector<>(step * dt * shear_velocity, 0, 0));
        top->SetPos_dt(ChVector<>(shear_velocity, 0, 0));

        plate->SetPos(plate_compress_pos + ChVector<>(step * dt * shear_velocity, 0, 0));
        plate->SetPos_dt(ChVector<>(shear_velocity, 0, 0));

        m_sys.DoStepDynamics(dt);

        // Output displacement and force
        if (step % out_steps == 0) {
            cout << std::setprecision(4) << "Time: " << m_time << endl;
            cout << std::setprecision(4) << "\tShear displacement: " << top->GetPos().x() << endl;
            WriteParticles(m_sys, std::string("points_shear") + std::to_string(step) + std::string(".csv"));
        }
        m_time += dt;
        step++;

        double shear_force = std::abs(top->GetContactForce().x());
        ss << m_time << "," << top->GetPos().x() << "," << shear_force << endl;
    }

    std::ofstream ostream;
    ostream.open(file_name);
    ostream << ss.str();
    ostream.close();

    return 0;
}