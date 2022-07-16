// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Polaris wheeled vehicle on SPH terrain
//
// =============================================================================

#include <cstdio>
#include <string>
#include <fstream>
#include <array>
#include <stdexcept>
#include <iomanip>

#include <thrust/copy.h>
#include <thrust/gather.h>
#include <thrust/for_each.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/functional.h>
#include <thrust/execution_policy.h>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/ChVisualizationFsi.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::vehicle;

using std::cout;
using std::cin;
using std::endl;

// -----------------------------------------------------------------------------

enum class MRZR_MODEL {ORIGINAL, MODIFIED};

MRZR_MODEL model = MRZR_MODEL::MODIFIED;

// Speed controller target speed (in m/s)
double target_speed = 7;

const ChVector<> gravity(0, 0, -9.81);

// -----------------------------------------------------------------------------

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& terrain_dir,
                     double& tend,
                     double& step_size,
                     double& active_box_dim,
                     double& output_major_fps,
                     double& output_minor_fps,
                     int& output_frames,
                     bool& output_velocities,
                     double& filter_window,
                     double& vis_output_fps,
                     bool& run_time_vis,
                     bool& run_time_vis_particles,
                     bool& run_time_vis_bce,
                     double& run_time_vis_fps,
                     bool& verbose) {
    ChCLI cli(argv[0], "Polaris SPH terrain simulation");

    cli.AddOption<std::string>("Simulation", "terrain_dir", "Directory with terrain specification data");
    cli.AddOption<double>("Simulation", "tend", "Simulation end time [s]", std::to_string(tend));
    cli.AddOption<double>("Simulation", "step_size", "Integration step size [s]", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "active_box_dim", "Half-dimension of active box [m]",
                          std::to_string(active_box_dim));

    cli.AddOption<double>("Simulation output", "output_major_fps", "Simulation output major frequency [fps]",
                          std::to_string(output_major_fps));
    cli.AddOption<double>("Simulation output", "output_minor_fps", "Simulation output major frequency [fps]",
                          std::to_string(output_minor_fps));
    cli.AddOption<int>("Simulation output", "output_frames", "Number of successive output frames", std::to_string(output_frames));
    cli.AddOption<bool>("Simulation output", "no_particle_vel", "Do not output particle velocities");
    cli.AddOption<double>("Simulation output", "filter_window", "Running average filter window [s]",
                          std::to_string(filter_window));

    cli.AddOption<bool>("", "quiet", "Disable all messages during simulation");

    cli.AddOption<double>("Visualization", "vis_output_fps", "Visualization output frequency [fps]",
                          std::to_string(vis_output_fps));
    cli.AddOption<bool>("Visualization", "run_time_vis", "Enable run-time visualization");
    cli.AddOption<bool>("Visualization", "run_time_vis_particles", "Enable run-time particle visualization");
    cli.AddOption<bool>("Visualization", "run_time_vis_bce", "Enabale run-time BCE markjer visualization");
    cli.AddOption<double>("Visualization", "run_time_vis_fps", "Run-time visualization frequency [fps]",
                          std::to_string(run_time_vis_fps));


    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    try {
        terrain_dir = cli.Get("terrain_dir").as<std::string>();
    } catch (std::domain_error&) {
        cout << "\nERROR: Missing terrain specification directory!\n\n" << endl;
        cli.Help();
        return false;
    }

    output_major_fps = cli.GetAsType<double>("output_major_fps");
    output_minor_fps = cli.GetAsType<double>("output_minor_fps");
    output_frames = cli.GetAsType<int>("output_frames");
    output_velocities = !cli.GetAsType<bool>("no_particle_vel");

    filter_window = cli.GetAsType<double>("filter_window");

    vis_output_fps = cli.GetAsType<double>("vis_output_fps");
    run_time_vis = cli.GetAsType<bool>("run_time_vis");
    run_time_vis_particles = cli.GetAsType<bool>("run_time_vis_particles");
    run_time_vis_bce = cli.GetAsType<bool>("run_time_vis_bce");
    run_time_vis_fps = cli.GetAsType<double>("run_time_vis_fps");

    tend = cli.GetAsType<double>("tend");
    step_size = cli.GetAsType<double>("step_size");
    active_box_dim = cli.GetAsType<double>("active_box_dim");

    verbose = !cli.GetAsType<bool>("quiet");

    return true;
}

// -----------------------------------------------------------------------------

class DataWriter {
  public:
    /// Construct an output data writer for the specified FSI system and vehicle.
    DataWriter(ChSystemFsi& sysFSI, std::shared_ptr<WheeledVehicle> vehicle)
        : m_sysFSI(sysFSI),
          m_vehicle(vehicle),
          m_out_vel(false),
          m_filter(true),
          m_filter_window(0.05),
          m_verbose(true) {
        m_wheels[0] = vehicle->GetWheel(0, LEFT);
        m_wheels[1] = vehicle->GetWheel(0, RIGHT);
        m_wheels[2] = vehicle->GetWheel(1, LEFT);
        m_wheels[3] = vehicle->GetWheel(1, RIGHT);

        // Set default size of sampling box
        double tire_radius = m_wheels[0]->GetTire()->GetRadius();
        double tire_width = m_wheels[0]->GetTire()->GetWidth();
        m_box_size.x() = 2.0 * std::sqrt(3.0) * tire_radius;
        m_box_size.y() = 1.5 * tire_width;
        m_box_size.z() = 0.2;

        // Set default offset of sampling box
        m_box_offset = ChVector<>(0.15, 0.0, 0.0);
    }

    ~DataWriter() { m_veh_stream.close(); }

    /// Enable/disable output data running-average filtering (default: true). 
    void UseFilteredData(bool val, double window) {
        m_filter = val;
        m_filter_window = window;
    }

    /// Enable/disable terminal messages (default: true).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Enable/disable output of SPH particle velocities (default: true).
    void SaveVelocities(bool val) { m_out_vel = val; }

    /// Set the location (relative to tire bottom point) and dimension (x and y) of the soil sampling domain.
    void SetSamplingVolume(const ChVector<>& offset, const ChVector2<>& size) {
        m_box_size.x() = size.x();
        m_box_size.y() = size.y();
        m_box_offset = offset;
    }

    /// Initialize the data writer, specifying the output directory and output frequency parameters.
    ///
    /// The data writer is reset with a frequency 'major_FPS'.  At each reset time, new sampling volumes are created
    /// under each wheel and the particle indices in each sampling volume are cached.  Following a reset, the states of
    /// all tagged particles are saved at 'num_minor' subsequent frames, separated by 'minor_FPS'.
    ///
    /// For example, assuming a step_size of 1 ms, a major and minor frequencies of 20 and 200, respectively, and
    /// num_minor=5, output will be generated at the following simulation frames: 0,5,10,15,20, 50,55,60,65,70,
    /// 100,105,110,115,120, etc. The sampling boxes and tracked particles are re-evaluated at frames: 0, 50, 100, etc.
    ///
    /// Each output frame is uniquely identified with a index of the "major" frame M and an index of the "minor" frame
    /// m. At each output frame, the following files are generated:
    /// - soil_M_m_w0.csv, soil_M_m_w1.csv, soil_M_m_w2.csv, soil_M_m_w3.csv
    ///   which contain the state (position and optionally velocity) of a particle in the sampling volume below the
    ///   corresponding wheel.  Wheels are counted front-to-back and left-to-right; i.e., in the order FL, FR, RL, RR.
    /// - vehicle_M_m.csv
    ///   each containing 9 lines as follows:
    ///   line 1: vehicle position, orientation, linear velocity, angular velocity (3+4+3+3 values)
    ///   line 2-5: wheel position, orientation, linear velocity, angular velocity (3+4+3+3 values per line)
    ///   line 6-9: tire force and moment (3+3 calues per line)
    ///
    /// Positions and orientations are relative to the global frame. 
    /// Rotations are provided as a quaternion (w,x,y,z).
    /// Linear and angular velocities are relative to and expressed in the global frame.
    /// Tire forces and moments are assumed applied at the wheel/tire center and are given in the global frame.
    void Initialize(const std::string& dir, double major_FPS, double minor_FPS, int num_minor, double step_size) {
        m_dir = dir;
        m_out_frames = num_minor;
        m_major_skip = (int)std::round((1.0 / major_FPS) / step_size);
        m_minor_skip = (int)std::round((1.0 / minor_FPS) / step_size);
        m_major_frame = -1;
        m_minor_frame = 0;

        // Sanity check
        if (m_minor_skip * m_out_frames > m_major_skip) {
            cout << "Error: Incompatible output frequencies!" << endl;
            throw std::runtime_error("Incompatible output frequencies");
        }

        // Create filters
        if (m_filter) {
            int steps = (int)std::round(m_filter_window / step_size);
            for (int i = 0; i < m_num_veh_outputs; i++) {
                m_veh_filters[i] = chrono_types::make_shared<chrono::utils::ChRunningAverage>(steps);
            }
        }

        std::string filename = m_dir + "/vehicle.csv";
        m_veh_stream.open(filename, std::ios_base::trunc);

        cout << "Wheel sampling box size:   " << m_box_size << endl;
        cout << "Wheel sampling box offset: " << m_box_offset << endl;
    }

    /// Run the data writer at the current simulation frame.
    /// This function must be called at each simulation frame; output occurs only at those frames that are consistent
    /// with the given output frequencies.
    void Process(int sim_frame, const DriverInputs& driver_inputs) {
        m_drv_inp = driver_inputs;

        CollectVehicleData();

        if (sim_frame % m_major_skip == 0) {
            m_last_major = sim_frame;
            m_major_frame++;
            m_minor_frame = 0;
            if (m_verbose)
                cout << "Start collection " << m_major_frame << endl;
            Reset();
        }
        if ((sim_frame - m_last_major) % m_minor_skip == 0 && m_minor_frame < m_out_frames) {
            if (m_verbose)
                cout << "    Output data " << m_major_frame << "/" << m_minor_frame << endl;
            Write();
            m_minor_frame++;
        }
        if (m_verbose)
            cout << std::flush;
    }

  private:
    void Reset() {
        for (int i = 0; i < 4; i++) {
            auto wheel_pos = m_wheels[i]->GetPos();
            auto wheel_normal = m_wheels[i]->GetSpindle()->GetRot().GetYaxis();
            auto tire_radius = m_wheels[i]->GetTire()->GetRadius();

            ChVector<> Z_dir(0, 0, 1);
            ChVector<> X_dir = Vcross(wheel_normal, ChVector<>(0, 0, 1)).GetNormalized();
            ChVector<> Y_dir = Vcross(Z_dir, X_dir);
            ChMatrix33<> box_rot(X_dir, Y_dir, Z_dir);
            ChVector<> box_pos = wheel_pos + box_rot * (m_box_offset - ChVector<>(0, 0, tire_radius));

            m_indices[i] = m_sysFSI.FindParticlesInBox(ChFrame<>(box_pos, box_rot), m_box_size);
        }
    }

    void Write() {
        for (int i = 0; i < 4; i++) {
            std::string filename = m_dir + "/soil_" + std::to_string(m_major_frame) + "_" +
                                   std::to_string(m_minor_frame) + "_w" + std::to_string(i) + ".csv ";
            if (m_out_vel)
                WriteParticlePosVel(m_sysFSI, m_indices[i], filename);
            else
                WriteParticlePos(m_sysFSI, m_indices[i], filename);
        }

        {
            std::string filename = m_dir + "/vehicle_" + std::to_string(m_major_frame) + "_" +
                                   std::to_string(m_minor_frame) + ".csv ";
            WriteVehicleData(filename);
        }
    }

    void CollectVehicleData() { 
        size_t start = 0;
        
        auto v_pos = m_vehicle->GetPos();
        m_veh_outputs[start + 0] = v_pos.x();
        m_veh_outputs[start + 1] = v_pos.y();
        m_veh_outputs[start + 2] = v_pos.z();
        start += 3;

        auto v_rot = m_vehicle->GetRot();
        m_veh_outputs[start + 0] = v_rot.e0();
        m_veh_outputs[start + 1] = v_rot.e1();
        m_veh_outputs[start + 2] = v_rot.e2();
        m_veh_outputs[start + 3] = v_rot.e3();
        start += 4;

        auto v_vel = m_vehicle->GetPointVelocity(ChVector<>(0, 0, 0));
        m_veh_outputs[start + 0] = v_vel.x();
        m_veh_outputs[start + 1] = v_vel.y();
        m_veh_outputs[start + 2] = v_vel.z();
        start += 3;

        auto v_omg = m_vehicle->GetChassisBody()->GetWvel_par();
        m_veh_outputs[start + 0] = v_omg.x();
        m_veh_outputs[start + 1] = v_omg.y();
        m_veh_outputs[start + 2] = v_omg.z();
        start += 3;

        for (int i = 0; i < 4; i++) {
            auto w_state = m_wheels[i]->GetState();

            m_veh_outputs[start + 0] = w_state.pos.x();
            m_veh_outputs[start + 1] = w_state.pos.y();
            m_veh_outputs[start + 2] = w_state.pos.z();
            start += 3;

            m_veh_outputs[start + 0] = w_state.rot.e0();
            m_veh_outputs[start + 1] = w_state.rot.e1();
            m_veh_outputs[start + 2] = w_state.rot.e2();
            m_veh_outputs[start + 3] = w_state.rot.e3();
            start += 4;

            m_veh_outputs[start + 0] = w_state.lin_vel.x();
            m_veh_outputs[start + 1] = w_state.lin_vel.y();
            m_veh_outputs[start + 2] = w_state.lin_vel.z();
            start += 3;

            m_veh_outputs[start + 0] = w_state.ang_vel.x();
            m_veh_outputs[start + 1] = w_state.ang_vel.y();
            m_veh_outputs[start + 2] = w_state.ang_vel.z();
            start += 3;
        }

        for (int i = 0; i < 4; i++) {
            const auto& t_force = m_wheels[i]->GetSpindle()->Get_accumulated_force();
            m_veh_outputs[start + 0] = t_force.x();
            m_veh_outputs[start + 1] = t_force.y();
            m_veh_outputs[start + 2] = t_force.z();
            start += 3;

            const auto& t_torque = m_wheels[i]->GetSpindle()->Get_accumulated_torque();
            m_veh_outputs[start + 0] = t_torque.x();
            m_veh_outputs[start + 1] = t_torque.y();
            m_veh_outputs[start + 2] = t_torque.z();
            start += 3;
        }

        if (m_filter) {
            for (int i = 0; i < m_num_veh_outputs; i++) {
                m_veh_outputs[i] = m_veh_filters[i]->Add(m_veh_outputs[i]);
            }
        }
    }

    void WriteVehicleData(const std::string& filename) {
        const auto& o = m_veh_outputs;
        std::ofstream stream;
        stream.open(filename, std::ios_base::trunc);

        size_t start = 0;

        // Driver inputs
        stream << m_drv_inp.m_steering << ", " << m_drv_inp.m_throttle << ", " << m_drv_inp.m_braking << "\n";

        // Vehicle position, orientation, linear and angular velocities
        for (int j = 0; j < 13; j++)
            stream << o[start + j] << ", ";
        stream << "\n";
        start += 13;

        // Wheel position, orientation, linear and angular velocities
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 13; j++)
                stream << o[start + j] << ", ";
            stream << "\n";
            start += 13;
        }

        // Tire force and moment
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 6; j++)
                stream << o[start + j] << ", ";
            stream << "\n";
            start += 6;
        }

        stream.close();

        // Write line to global vehicle output file
        m_veh_stream << m_vehicle->GetChTime() << "    ";
        m_veh_stream << m_drv_inp.m_steering << " " << m_drv_inp.m_throttle << " " << m_drv_inp.m_braking << "    ";
        for (int i = 0; i < m_num_veh_outputs; i++)
            m_veh_stream << o[i] << "  ";
        m_veh_stream << "\n";
    }

    struct print_particle_pos {
        print_particle_pos(std::ofstream* stream) : m_stream(stream) {}
        __host__ void operator()(const Real4 p) { (*m_stream) << p.x << ", " << p.y << ", " << p.z << "\n"; }
        std::ofstream* m_stream;
    };

    struct print_particle_pos_vel {
        print_particle_pos_vel(std::ofstream* stream) : m_stream(stream) {}
        template <typename T>
        __host__ void operator()(const T pv) {
            auto p = thrust::get<0>(pv);
            auto v = thrust::get<1>(pv);
            (*m_stream) << p.x << ", " << p.y << ", " << p.z << ", " << v.x << ", " << v.y << ", " << v.z << "\n";
        }
        std::ofstream* m_stream;
    };

    void WriteParticlePos(ChSystemFsi& sysFSI,
                          const thrust::device_vector<int>& indices_D,
                          const std::string& filename) {
        // Get particle positions on device
        auto pos_D = sysFSI.GetParticlePositions(indices_D);

        // Copy vector to host
        thrust::host_vector<Real4> pos_H = pos_D;

        // Write output file
        std::ofstream stream;
        stream.open(filename, std::ios_base::trunc);
        thrust::for_each(thrust::host, pos_H.begin(), pos_H.end(), print_particle_pos(&stream));
        stream.close();
    }

    void WriteParticlePosVel(ChSystemFsi& sysFSI,
                             const thrust::device_vector<int>& indices_D,
                             const std::string& filename) {
        // Get particle positions and velocities on device
        auto pos_D = sysFSI.GetParticlePositions(indices_D);
        auto vel_D = sysFSI.GetParticleVelocities(indices_D);

        // Copy vectors to host
        thrust::host_vector<Real4> pos_H = pos_D;
        thrust::host_vector<Real3> vel_H = vel_D;

        // Write output file
        std::ofstream stream;
        stream.open(filename, std::ios_base::trunc);
        thrust::for_each(thrust::host,                                                                 //
                         thrust::make_zip_iterator(thrust::make_tuple(pos_H.begin(), vel_H.begin())),  //
                         thrust::make_zip_iterator(thrust::make_tuple(pos_H.end(), vel_H.end())),      //
                         print_particle_pos_vel(&stream)                                               //
        );
        stream.close();
    }

    ChSystemFsi& m_sysFSI;
    std::shared_ptr<WheeledVehicle> m_vehicle;
    std::array<std::shared_ptr<ChWheel>, 4> m_wheels;
    std::array<thrust::device_vector<int>, 4> m_indices;

    DriverInputs m_drv_inp;

    std::string m_dir;
    int m_major_skip;
    int m_minor_skip;
    int m_out_frames;
    int m_major_frame;
    int m_minor_frame;
    int m_last_major;

    bool m_out_vel;
    ChVector<> m_box_size;
    ChVector<> m_box_offset;
    double m_box_x;
    double m_box_z;

    bool m_filter;
    double m_filter_window;
    static const int m_num_veh_outputs = (7 + 6) + 4 * (7 + 6 + 3 + 3);
    std::array<double, m_num_veh_outputs> m_veh_outputs;
    std::array<std::shared_ptr<chrono::utils::ChRunningAverage>, m_num_veh_outputs> m_veh_filters;
    std::ofstream m_veh_stream;

    bool m_verbose;
};

// -----------------------------------------------------------------------------

void CreateMeshMarkers(const geometry::ChTriangleMeshConnected& mesh,
                       double delta,
                       std::vector<ChVector<>>& point_cloud) {
    ChVector<> minV = mesh.m_vertices[0];
    ChVector<> maxV = mesh.m_vertices[0];
    ChVector<> currV = mesh.m_vertices[0];
    for (unsigned int i = 1; i < mesh.m_vertices.size(); ++i) {
        currV = mesh.m_vertices[i];
        if (minV.x() > currV.x())
            minV.x() = currV.x();
        if (minV.y() > currV.y())
            minV.y() = currV.y();
        if (minV.z() > currV.z())
            minV.z() = currV.z();
        if (maxV.x() < currV.x())
            maxV.x() = currV.x();
        if (maxV.y() < currV.y())
            maxV.y() = currV.y();
        if (maxV.z() < currV.z())
            maxV.z() = currV.z();
    }
    ////printf("start coords: %f, %f, %f\n", minV.x(), minV.y(), minV.z());
    ////printf("end coords: %f, %f, %f\n", maxV.x(), maxV.y(), maxV.z());

    const double EPSI = 1e-6;

    ChVector<> ray_origin;
    for (double x = minV.x(); x < maxV.x(); x += delta) {
        ray_origin.x() = x + 1e-9;
        for (double y = minV.y(); y < maxV.y(); y += delta) {
            ray_origin.y() = y + 1e-9;
            for (double z = minV.z(); z < maxV.z(); z += delta) {
                ray_origin.z() = z + 1e-9;

                ChVector<> ray_dir[2] = {ChVector<>(5, 0.5, 0.25), ChVector<>(-3, 0.7, 10)};
                int intersectCounter[2] = {0, 0};

                for (unsigned int i = 0; i < mesh.m_face_v_indices.size(); ++i) {
                    auto& t_face = mesh.m_face_v_indices[i];
                    auto& v1 = mesh.m_vertices[t_face.x()];
                    auto& v2 = mesh.m_vertices[t_face.y()];
                    auto& v3 = mesh.m_vertices[t_face.z()];

                    // Find vectors for two edges sharing V1
                    auto edge1 = v2 - v1;
                    auto edge2 = v3 - v1;

                    bool t_inter[2] = {false, false};

                    for (unsigned int j = 0; j < 2; j++) {
                        // Begin calculating determinant - also used to calculate uu parameter
                        auto pvec = Vcross(ray_dir[j], edge2);
                        // if determinant is near zero, ray is parallel to plane of triangle
                        double det = Vdot(edge1, pvec);
                        // NOT CULLING
                        if (det > -EPSI && det < EPSI) {
                            t_inter[j] = false;
                            continue;
                        }
                        double inv_det = 1.0 / det;

                        // calculate distance from V1 to ray origin
                        auto tvec = ray_origin - v1;

                        // Calculate uu parameter and test bound
                        double uu = Vdot(tvec, pvec) * inv_det;
                        // The intersection lies outside of the triangle
                        if (uu < 0.0 || uu > 1.0) {
                            t_inter[j] = false;
                            continue;
                        }

                        // Prepare to test vv parameter
                        auto qvec = Vcross(tvec, edge1);

                        // Calculate vv parameter and test bound
                        double vv = Vdot(ray_dir[j], qvec) * inv_det;
                        // The intersection lies outside of the triangle
                        if (vv < 0.0 || ((uu + vv) > 1.0)) {
                            t_inter[j] = false;
                            continue;
                        }

                        double tt = Vdot(edge2, qvec) * inv_det;
                        if (tt > EPSI) {  // ray intersection
                            t_inter[j] = true;
                            continue;
                        }

                        // No hit, no win
                        t_inter[j] = false;
                    }

                    intersectCounter[0] += t_inter[0] ? 1 : 0;
                    intersectCounter[1] += t_inter[1] ? 1 : 0;
                }

                if (((intersectCounter[0] % 2) == 1) && ((intersectCounter[1] % 2) == 1))  // inside mesh
                    point_cloud.push_back(ChVector<>(x, y, z));
            }
        }
    }
}

void CreateTerrain(ChSystem& sys,
                   ChSystemFsi& sysFSI,
                   const std::string& terrain_dir,
                   bool terrain_mesh_vis,
                   bool terrain_mesh_contact) {
    // Create SPH markers with initial locations from file
    int num_particles = 0;
    ChVector<> aabb_min(std::numeric_limits<double>::max());
    ChVector<> aabb_max(-std::numeric_limits<double>::max());

    ChVector<> marker;
    std::string line;
    std::string cell;

    std::ifstream is(vehicle::GetDataFile(terrain_dir + "/particles_20mm.txt"));
    getline(is, line);  // Comment line
    while (getline(is, line)) {
        std::stringstream ls(line);
        for (int i = 0; i < 3; i++) {
            getline(ls, cell, ',');
            marker[i] = stod(cell);
            aabb_min[i] = std::min(aabb_min[i], marker[i]);
            aabb_max[i] = std::max(aabb_max[i], marker[i]);
        }
        ////ChVector<> tau(-sysFSI.GetSensity() * std::abs(gravity.z) * (-marker.z() + fzDim));
        ChVector<> tau(0);
        sysFSI.AddSPHParticle(marker, sysFSI.GetDensity(), 0, sysFSI.GetViscosity(), sysFSI.GetKernelLength(), VNULL,
                              tau, VNULL);
        num_particles++;
    }
    is.close();

    // Set computational domain
    ChVector<> aabb_dim = aabb_max - aabb_min;
    aabb_dim.z() *= 50;
    sysFSI.SetBoundaries(aabb_min - 0.1 * aabb_dim, aabb_max + 0.1 * aabb_dim);

    // Create ground body and attach BCE markers
    // Note: BCE markers must be created after SPH markers!
    auto body = std::shared_ptr<ChBody>(sys.NewBody());
    body->SetBodyFixed(true);
    sys.AddBody(body);

    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(vehicle::GetDataFile(terrain_dir + "/mesh.obj"), true, false);

    if (terrain_mesh_vis) {
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetMutable(false);
        body->AddVisualShape(trimesh_shape);
    }

    if (terrain_mesh_contact) {
        MaterialInfo mat_info;
        mat_info.mu = 0.9f;
        auto mat = mat_info.CreateMaterial(sys.GetContactMethod());
        body->GetCollisionModel()->ClearModel();
        body->GetCollisionModel()->AddTriangleMesh(mat, trimesh, true, false, VNULL, ChMatrix33<>(1), 0.01);
        body->GetCollisionModel()->BuildModel();
        body->SetCollide(true);
    }

    sysFSI.AddFileBCE(body, vehicle::GetDataFile(terrain_dir + "/bce_20mm.txt"), VNULL, QUNIT, 1.0, false);
}

std::shared_ptr<WheeledVehicle> CreateVehicle(ChSystem& sys,
                                              const ChCoordsys<>& init_pos,
                                              ChSystemFsi& sysFSI) {
    std::string model_dir = (model == MRZR_MODEL::ORIGINAL) ? "mrzr/JSON_orig/" : "mrzr/JSON_new/";

    std::string vehicle_json = model_dir + "vehicle/MRZR.json";
    ////std::string powertrain_json = model_dir + "powertrain/MRZR_SimplePowertrain.json";
    std::string powertrain_json = model_dir + "powertrain/MRZR_SimpleMapPowertrain.json";
    std::string tire_json = model_dir + "tire/MRZR_RigidTire.json";

    std::string tire_coll_obj = "mrzr/meshes_new/Polaris_tire_collision.obj";

    // Create and initialize the vehicle
    auto vehicle = chrono_types::make_shared<WheeledVehicle>(&sys, vehicle::GetDataFile(vehicle_json));
    vehicle->Initialize(init_pos);
    vehicle->GetChassis()->SetFixed(false);
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto powertrain = ReadPowertrainJSON(vehicle::GetDataFile(powertrain_json));
    vehicle->InitializePowertrain(powertrain);

    // Create BCE markers for a tire
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(vehicle::GetDataFile(tire_coll_obj));
    std::vector<ChVector<>> point_cloud;
    CreateMeshMarkers(trimesh, sysFSI.GetInitialSpacing(), point_cloud);

    // Create and initialize the tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));
            vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
            sysFSI.AddFsiBody(wheel->GetSpindle());
            sysFSI.AddPointsBCE(wheel->GetSpindle(), point_cloud, VNULL, QUNIT);
        }
    }

    return vehicle;
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string terrain_dir;
    double tend = 30;
    double step_size = 5e-4;
    double active_box_dim = 0.5;
    double output_major_fps = 20;
    double output_minor_fps = 1000;
    int output_frames = 5;
    bool output_velocities = true;        // output particle velocities
    double filter_window = 0;             // do not filter data
    double vis_output_fps = 0;            // no post-processing visualization output
    bool run_time_vis = false;            // no run-time visualization
    double run_time_vis_fps = 0;          // render every simulation frame
    bool run_time_vis_particles = false;  // render only terrain surface mesh
    bool run_time_vis_bce = false;        // render vehicle meshes
    bool verbose = true;

    if (!GetProblemSpecs(argc, argv, terrain_dir, tend, step_size, active_box_dim, output_major_fps, output_minor_fps,
                         output_frames, output_velocities, filter_window, vis_output_fps, run_time_vis,
                         run_time_vis_particles, run_time_vis_bce, run_time_vis_fps, verbose)) {
        return 1;
    }

    bool sim_output = (output_major_fps > 0);
    bool use_filter = (filter_window > 0);
    bool vis_output = (vis_output_fps > 0);

    // Check input files exist
    if (!filesystem::path(vehicle::GetDataFile(terrain_dir + "/sph_params.json")).exists()) {
        std::cout << "Input file sph_params.json not found in directory " << terrain_dir << std::endl;
        return 1;
    }
    if (!filesystem::path(vehicle::GetDataFile(terrain_dir + "/path.txt")).exists()) {
        std::cout << "Input file path.txt not found in directory " << terrain_dir << std::endl;
        return 1;
    }
    if (!filesystem::path(vehicle::GetDataFile(terrain_dir + "/particles_20mm.txt")).exists()) {
        std::cout << "Input file particles_20mm.txt not found in directory " << terrain_dir << std::endl;
        return 1;
    }
    if (!filesystem::path(vehicle::GetDataFile(terrain_dir + "/bce_20mm.txt")).exists()) {
        std::cout << "Input file bce_20mm.txt not found in directory " << terrain_dir << std::endl;
        return 1;
    }

    // Create the Chrono systems
    ChSystemNSC sys;
    ChSystemFsi sysFSI(sys);

    // Load SPH parameter file
    cout << "Load SPH parameter file..." << endl;
    sysFSI.ReadParametersFromFile(vehicle::GetDataFile(terrain_dir + "/sph_params.json"));

    sysFSI.SetActiveDomain(ChVector<>(active_box_dim, active_box_dim, 1));
    sysFSI.SetDiscreType(false, false);
    sysFSI.SetWallBC(BceVersion::ORIGINAL);
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);
    sysFSI.SetStepSize(step_size);
    sysFSI.SetVerbose(false);

    // Set simulation data output and FSI information output
    std::string out_dir = GetChronoOutputPath() + "POLARIS_SPH/";
    std::string vis_dir = out_dir + "Visualization/";
    sysFSI.SetOutputLength(0);
    ////sysFSI.SetOutputDirectory(out_dir);

    sys.Set_G_acc(gravity);

    // Create terrain
    cout << "Create terrain..." << endl;
    bool terrain_mesh_contact = false;
    CreateTerrain(sys, sysFSI, terrain_dir, !run_time_vis_particles, terrain_mesh_contact);

    // Create vehicle
    cout << "Create vehicle..." << endl;
    double slope = 0;
    double banking = 0;
    if (filesystem::path(vehicle::GetDataFile(terrain_dir + "/slope.txt")).exists()) {
        std::ifstream is(vehicle::GetDataFile(terrain_dir + "/slope.txt"));
        is >> slope >> banking;
        is.close();
    }
    ChCoordsys<> init_pos(ChVector<>(4, 0, 0.25 + 4 * std::sin(slope)), Q_from_AngX(banking) * Q_from_AngY(-slope));
    auto vehicle = CreateVehicle(sys, init_pos, sysFSI);

    // Create driver
    auto path = ChBezierCurve::read(vehicle::GetDataFile(terrain_dir + "/path.txt"));
    double x_max = path->getPoint(path->getNumPoints() - 1).x() - 3.0;
    ChPathFollowerDriver driver(*vehicle, path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(2.0);
    driver.GetSteeringController().SetGains(1.0, 0, 0);
    driver.GetSpeedController().SetGains(0.6, 0.05, 0);
    driver.Initialize();

    // Complete construction of FSI system
    sysFSI.Initialize();

    // Create run-time visualization
    ChVisualizationFsi fsi_vis(&sysFSI);
    if (run_time_vis) {
        fsi_vis.SetTitle("Chrono::FSI single wheel demo");
        fsi_vis.SetSize(1280, 720);
        fsi_vis.SetCameraPosition(ChVector<>(-3, 0, 6), ChVector<>(5, 0, 0.5));
        fsi_vis.SetCameraMoveScale(1.0f);
        fsi_vis.EnableFluidMarkers(run_time_vis_particles);
        fsi_vis.EnableRigidBodyMarkers(run_time_vis_bce);
        fsi_vis.EnableBoundaryMarkers(false);
        if (!run_time_vis_bce) {
            fsi_vis.AttachSystem(&sys);
        }
        fsi_vis.SetRenderMode(ChVisualizationFsi::RenderMode::SOLID);
        fsi_vis.EnableInfoOverlay(false);
        fsi_vis.Initialize();
    }

    // Enable data output
    cout << "===============================================================================" << endl;
    auto sim_dir = out_dir + filesystem::path(terrain_dir).filename() + "/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(sim_dir))) {
        cout << "Error creating directory " << sim_dir << endl;
        return 1;
    }
    DataWriter data_writer(sysFSI, vehicle);
    data_writer.SetVerbose(verbose);
    data_writer.SaveVelocities(output_velocities);
    data_writer.UseFilteredData(use_filter, filter_window);
    data_writer.Initialize(sim_dir, output_major_fps, output_minor_fps, output_frames, step_size);
    cout << "Simulation output data saved in: " << sim_dir << endl;
    cout << "===============================================================================" << endl;

    // Simulation loop
    DriverInputs driver_inputs = {0, 0, 0};
    ChTerrain terrain;

    int vis_output_steps = (int)std::round((1.0 / vis_output_fps) / step_size);
    int render_steps = (run_time_vis_fps > 0) ? (int)std::round((1.0 / run_time_vis_fps) / step_size) : 1;
    int vis_output_frame = 0;

    double t = 0;
    int frame = 0;
    while (t < tend) {
        // Stop before end of patch
        if (vehicle->GetPos().x() > x_max)
            break;

        // Simulation data output
        if (sim_output)
            data_writer.Process(frame, driver_inputs);

        // Visualization data output
        if (vis_output && frame % vis_output_steps == 0) {
            if (verbose)
                cout << "Visualization output frame = " << vis_output_frame << endl;
            sysFSI.PrintParticleToFile(vis_dir);
            std::string vehicle_file = vis_dir + "/vehicle_" + std::to_string(vis_output_frame) + ".csv";
            chrono::utils::WriteVisualizationAssets(&sys, vehicle_file);

            vis_output_frame++;
        }

        // Run-time visualization
        if (run_time_vis && frame % render_steps == 0) {
            if (!fsi_vis.Render())
                break;
        }

        // Set current driver inputs
        driver_inputs = driver.GetInputs();

        if (t < 1) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        } else {
            ChClampValue(driver_inputs.m_throttle, driver_inputs.m_throttle, (t - 1) / 0.5);
        }

        if (verbose)
            cout << std::fixed << std::setprecision(3) << "t = " << t << "  STB = " << driver_inputs.m_steering << " "
                 << driver_inputs.m_throttle << " " << driver_inputs.m_braking << "  spd = " << vehicle->GetSpeed()
                 << endl;

        driver.Synchronize(t);
        vehicle->Synchronize(t, driver_inputs, terrain);

        // Advance system state
        driver.Advance(step_size);
        sysFSI.DoStepDynamics_FSI();
        t += step_size;

        frame++;
    }

    return 0;
}
