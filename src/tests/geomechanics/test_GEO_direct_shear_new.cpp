#include <omp.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

#include "chrono/ChConfig.h"
#include <chrono/motion_functions/ChFunction_Setpoint.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChLinkMotorLinearPosition.h>
#include <chrono_parallel/physics/ChSystemParallel.h>

#include <chrono/utils/ChUtilsCreators.h>
#include <chrono/utils/ChUtilsSamplers.h>

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using std::cout;
using std::endl;
using std::string;

using namespace chrono;
using namespace chrono::collision;

string file_name;
string file_name_prefix("shear_results");
string csv_header("t,x,fm,fc");

// UNIT SYSTEM: SI (kg, m, s)
// Conversion factors for specifications
double ft2in = 12;
double in2m = 0.0254;
double min2s = 60;
double g2kg = 1.0 / 1000.0;

double grav = 9.81;  // Magnitude of gravity in the -z direction

double sphere_inflation = 10;                     // Multiplier on particle radius
double sphere_radius = sphere_inflation * 50e-6;  // Particle radius 50um = 50e-6m
double sphere_density = 400;                      // Particle density 0.4 g/cm^3 = 400 kg/m^3
double sphere_mass = 4 * CH_C_PI * sphere_radius * sphere_radius * sphere_radius * sphere_density / 3;
ChVector<> sphere_inertia = 2 * sphere_mass * sphere_radius * sphere_radius / 3 * ChVector<>(1, 1, 1);

// Particle material: Parameters to tune
double sphere_mu = 0.18;  // Coefficient of friction
double sphere_cr = 0.87;  // Coefficient of restitution

double uncompressed_volume =
    0.003873 * ft2in * ft2in * ft2in * in2m * in2m * in2m;  // 0.003873 ft^3 sample before compression

// Interior dimensions of shear chamber
double box_dim_X = 2.416 * in2m;  // 2.416in shear box diameter
double box_dim_Y = 2.416 * in2m;  // 2.416in shear box diameter
double box_dim_Z = uncompressed_volume / (box_dim_X * box_dim_Y);

double sampling_to_settled_ratio = 1.85;  // TODO tune

double sampling_dim_Z = box_dim_Z * sampling_to_settled_ratio;

double box_mass = 100;

// Box material
double box_mu = sphere_mu;
double box_cr = sphere_cr;

double dt = 1e-4;  // Simulation timestep
double tolerance = 0.1;
int max_iteration_normal = 0;
int max_iteration_sliding = 100;
int max_iteration_spinning = 0;
int max_iteration_bilateral = 100;
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

double settling_time = 0.11;

double max_compression_time = 0.35;  // Alternative stop condition for ending the compression phase

double shear_velocity_inflation = 1000;  // Multiplier on shear_velocity speed for shorter simulation
double shear_velocity = shear_velocity_inflation * 0.01 * in2m / min2s;  // X velocity of top shear section 0.01in/min
double shear_displacement = 0.25 * in2m;                                 // X displacement at which the test ends 0.25in
double shear_time = shear_displacement / shear_velocity;

double box_thick = 1.5 * shear_displacement;  // Thickness of walls so that no material spills during shearing

void AddBox(ChSystemParallel& m_sys, std::shared_ptr<ChBody>& top) {
    auto box_mat = std::make_shared<ChMaterialSurfaceNSC>();
    box_mat->SetFriction(box_mu);
    box_mat->SetRestitution(box_cr);

    double hx = box_dim_X / 2;
    double hy = box_dim_Y / 2;
    double hz = box_dim_Z / 2;

    double hthick = box_thick / 2;

    // Top half of shear box
    bool top_vis = false;
    ChVector<> pos(0, 0, 0);
    top = std::shared_ptr<ChBody>(m_sys.NewBody());
    top->SetPos(pos);
    top->SetMass(box_mass / 2);
    top->SetMaterialSurface(box_mat);
    top->SetBodyFixed(true);
    top->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(top.get(), ChVector<>(hthick, hy, hz / 2), ChVector<>(-(hx + hthick), 0, hz / 2), QUNIT, top_vis);  // Low X
    utils::AddBoxGeometry(top.get(), ChVector<>(hthick, hy, hz / 2), ChVector<>(hx + hthick, 0, hz / 2), QUNIT, top_vis);     // High X
    utils::AddBoxGeometry(top.get(), ChVector<>(hx, hthick, hz / 2), ChVector<>(0, -(hy + hthick), hz / 2), QUNIT, top_vis);  // Low Y
    utils::AddBoxGeometry(top.get(), ChVector<>(hx, hthick, hz / 2), ChVector<>(0, hy + hthick, hz / 2), QUNIT, top_vis);     // High Y
    top->GetCollisionModel()->BuildModel();
    top->SetCollide(true);
    top->GetCollisionModel()->SetFamily(1);
    top->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
    m_sys.AddBody(top);

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

void AddPlate(ChSystemParallelNSC& m_sys,
              std::shared_ptr<ChBody>& plate,
              std::shared_ptr<ChBody>& top,
              double plate_bottom) {
    auto box_mat = std::make_shared<ChMaterialSurfaceNSC>();
    box_mat->SetFriction(box_mu);
    box_mat->SetRestitution(box_cr);

    double hx = box_dim_X / 2;
    double hy = box_dim_Y / 2;
    double hz = box_dim_Z / 2;

    double hthick = box_thick / 2;

    // Plate for applying confining pressure
    plate = std::shared_ptr<ChBody>(m_sys.NewBody());
    plate->SetPos(ChVector<>(0, 0, plate_bottom + hthick));
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

    auto prismatic_plate_box = std::make_shared<ChLinkLockPrismatic>();
    prismatic_plate_box->Initialize(plate, top, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    m_sys.AddLink(prismatic_plate_box);
}

void AddMotor(ChSystemParallelNSC& m_sys,
              std::shared_ptr<ChBody>& top,
              std::shared_ptr<ChLinkMotorLinearPosition>& motor) {
    double hz = box_dim_Z / 2;

    auto ground = std::shared_ptr<ChBody>(m_sys.NewBody());
    ground->SetBodyFixed(true);
    ground->SetPos(ChVector<>(0, 0, 0));
    m_sys.AddBody(ground);

    auto pos_func = std::make_shared<ChFunction_Setpoint>();

    motor = std::make_shared<ChLinkMotorLinearPosition>();
    motor->Initialize(top, ground, ChFrame<>(ChVector<>(0, 0, hz / 2), QUNIT));
    motor->SetMotionFunction(pos_func);
    m_sys.AddLink(motor);
}

void FixPlate(ChSystemParallelNSC& m_sys, std::shared_ptr<ChBody>& plate, std::shared_ptr<ChBody>& top) {
    auto lock = std::make_shared<ChLinkLockLock>();
    lock->Initialize(plate, top, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    m_sys.AddLink(lock);
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
        sphere->SetInertiaXX(sphere_inertia);
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

void WriteParticles(ChSystemParallelNSC& m_sys, string file_name) {
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
    bool render = false;

    if (argc < 2) {
        cout << "usage: " << argv[0] << " <pressure_index: 0-6> [true]" << endl;
        return 1;
    } else if (argc > 2) {
        render = true;
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
    cout << "Settling time: " << settling_time << endl;
    cout << "Compression time: " << max_compression_time << endl;
    cout << "Shear velocity: " << shear_velocity << endl;
    cout << "Shear displacement: " << shear_displacement << endl;
    cout << "Shear time: " << shear_time << endl;

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

    m_sys.GetSettings()->collision.collision_envelope = 0.1 * sphere_radius;

    double factor = 2 * 2 * sphere_radius;  // Length of each dimension of a bin
    double total_x = box_dim_X + 2 * box_thick + shear_displacement;
    double total_y = box_dim_Y + 2 * box_thick;
    double total_z = box_dim_Z + box_thick;

    int bins_x = std::ceil(total_x / factor);
    int bins_y = std::ceil(total_y / factor);
    int bins_z = std::ceil(total_z / factor);

    cout << "Bins: " << bins_x << " X " << bins_y << " X " << bins_z << endl;

    m_sys.GetSettings()->collision.bins_per_axis = vec3(bins_x, bins_y, bins_z);
    m_sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    // Add containing box and compression plate
    std::shared_ptr<ChBody> plate;  // Weighted plate - top face of shear box
    std::shared_ptr<ChBody> top;    // Top section of the shear box
    AddBox(m_sys, top);

    // Add spherical particles
    unsigned int num_particles = AddParticles(
        m_sys, ChVector<>(0, 0, -box_dim_Z / 2 + sampling_dim_Z / 2),
        ChVector<>(box_dim_X / 2 - sphere_radius, box_dim_Y / 2 - sphere_radius, sampling_dim_Z / 2 - sphere_radius));

    cout << "Actual number of particles: " << num_particles << endl;

#ifdef CHRONO_OPENGL
    if (render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(800, 600, "Direct shear", &m_sys);
        gl_window.SetCamera(ChVector<>(3 * box_dim_Y, 0, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), (float)sphere_radius,
            (float)sphere_radius);
        gl_window.SetRenderMode(opengl::WIREFRAME);
    }
#endif

    // Settle material under its own weight for a fixed amount of time
    cout << endl << "Running settling..." << endl;
    double m_time = 0;
    unsigned int step = 0;
    while (m_time < settling_time) {
        m_sys.DoStepDynamics(dt);
#ifdef CHRONO_OPENGL
        if (render) {
            opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
            if (gl_window.Active())
                gl_window.Render();
            else
                break;
        }
#endif
        if (step % out_steps == 0) {
            cout << std::setprecision(4) << "Time: " << m_time << endl;
            WriteParticles(m_sys, string("points_compression") + std::to_string(step) + string(".csv"));
        }
        step++;
        m_time += dt;
    }

    // Add a weighted top plate
    double highest = -10e30;
    for (unsigned int i = 2; i < m_sys.Get_bodylist().size(); i++) {
        highest = std::max(highest, m_sys.Get_bodylist()[i]->GetPos().z());
    }

    highest += 2 * sphere_radius;

    AddPlate(m_sys, plate, top, highest);

    // Compress the material under the weight of the plate
    cout << endl << "Running compression..." << endl;
    m_time = 0;

    do {
        m_sys.DoStepDynamics(dt);
#ifdef CHRONO_OPENGL
        if (render) {
            opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
            if (gl_window.Active())
                gl_window.Render();
            else
                break;
        }
#endif
        if (step % out_steps == 0) {
            cout << std::setprecision(4) << "Time: " << m_time << endl;
            cout << std::setprecision(4) << "\tPlate height: " << plate->GetPos().z() << endl;
            WriteParticles(m_sys, string("points_compression") + std::to_string(step) + string(".csv"));
        }
        m_time += dt;
        step++;
    } while (m_time < max_compression_time);

    // Create a motor to slide the top of the box in the +x direction
    std::shared_ptr<ChLinkMotorLinearPosition> motor;
    top->SetBodyFixed(false);
    AddMotor(m_sys, top, motor);

    // Fix the plate to the top of the box
    // FixPlate(m_sys, plate, top); // BUG redundant constraints break the simulation

    // Run shearing for specified displacement
    cout << endl << "Running shear test..." << endl;

    std::ofstream ostream;
    ostream.open(file_name);
    ostream << csv_header << endl;

    step = 0;
    m_time = 0;
    while (m_time < shear_time) {
        auto pos_func = std::static_pointer_cast<ChFunction_Setpoint>(motor->GetMotorFunction());
        double pos = step * dt * shear_velocity;
        double pos_dt = shear_velocity;
        pos_func->SetSetpointAndDerivatives(pos, pos_dt, 0);

        m_sys.DoStepDynamics(dt);
#ifdef CHRONO_OPENGL
        if (render) {
            opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
            if (gl_window.Active())
                gl_window.Render();
            else
                break;
        }
#endif

        // Output displacement and force
        if (step % out_steps == 0) {
            cout << std::setprecision(4) << "Time: " << m_time << endl;
            cout << std::setprecision(4) << "\tShear displacement: " << top->GetPos().x() << endl;
            cout << std::setprecision(4) << "\tShear force (motor): " << motor->GetMotorForce() << endl;

            m_sys.CalculateContactForces();
            double shear_force_contact = m_sys.GetBodyContactForce(top).x;
            cout << std::setprecision(4) << "\tShear force (contact): " << shear_force_contact << endl;

            WriteParticles(m_sys, string("points_shear") + std::to_string(step) + string(".csv"));
        }
        m_time += dt;
        step++;

        double shear_force_motor = motor->GetMotorForce();
        m_sys.CalculateContactForces();
        double shear_force_contact = m_sys.GetBodyContactForce(top).x;
        ostream << m_time << "," << top->GetPos().x() << "," << shear_force_motor << "," << shear_force_contact << endl;
    }

    ostream.close();
    return 0;
}