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

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChOpenGLWindow.h"
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "FindParticles.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::vehicle;

using std::cout;
using std::cin;
using std::endl;

// -----------------------------------------------------------------------------

enum class MRZR_MODEL {ORIGINAL, MODIFIED};

MRZR_MODEL model = MRZR_MODEL::ORIGINAL;

// -----------------------------------------------------------------------------

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& terrain_dir,
                     double& output_major_fps,
                     double& output_minor_fps,
                     int& output_frames,
                     bool& output_velocities,
                     double& filter_window,
                     double& vis_output_fps,
                     bool& run_time_vis,
                     bool& verbose) {
    ChCLI cli(argv[0], "Polaris SPH terrain simulation");

    cli.AddOption<std::string>("Required", "terrain_dir", "Directory with terrain specification data");

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

    verbose = !cli.GetAsType<bool>("quiet");

    return true;
}

// -----------------------------------------------------------------------------

class DataWriter {
  public:

    /// Construct an output data writer for the specified FSI system and vehicle.
    DataWriter(std::shared_ptr<ChSystemFsi_impl> sysFSI, std::shared_ptr<WheeledVehicle> vehicle)
        : m_sysFSI(sysFSI), m_vehicle(vehicle), m_out_vel(false), m_filter(true), m_filter_window(0.05), m_verbose(true) {
        m_wheels[0] = vehicle->GetWheel(0, LEFT);
        m_wheels[1] = vehicle->GetWheel(0, RIGHT);
        m_wheels[2] = vehicle->GetWheel(1, LEFT);
        m_wheels[3] = vehicle->GetWheel(1, RIGHT);

        // Set default size of sampling box
        double tire_radius = m_wheels[0]->GetTire()->GetRadius();
        double tire_width = m_wheels[0]->GetTire()->GetWidth();
        m_box_size.x() = std::sqrt(3.0) * tire_radius;
        m_box_size.y() = 1.2 * tire_width;
        m_box_size.z() = 0.3;

        // Set default x location of sampling box
        m_box_x = 0;
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

    /// Set the location (relative to wheel center) and dimension (x and y) of the soil sampling domain.
    void SetSamplingVolume(double x, const ChVector2<>& size) {
        m_box_size.x() = size.x();
        m_box_size.y() = size.y();
        m_box_x = x;
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
        m_major_skip = (int)std::ceil((1.0 / major_FPS) / step_size);
        m_minor_skip = (int)std::ceil((1.0 / minor_FPS) / step_size);
        m_major_frame = -1;
        m_minor_frame = 0;

        // Sanity check
        if (m_minor_skip * m_out_frames > m_major_skip) {
            cout << "Error: Incompatible output frequencies!" << endl;
            throw std::runtime_error("Incompatible output frequencies");
        }

        // Create filters
        if (m_filter) {
            int steps = (int)std::ceil(m_filter_window / step_size);
            for (int i = 0; i < m_num_veh_outputs; i++) {
                m_veh_filters[i] = chrono_types::make_shared<chrono::utils::ChRunningAverage>(steps);
            }
        }

        std::string filename = m_dir + "/vehicle.csv";
        m_veh_stream.open(filename, std::ios_base::trunc);

        cout << "Wheel sampling box size:       " << m_box_size << endl;
        cout << "Wheel sampling box x location: " << m_box_x << endl;
    }

    /// Run the data writer at the current simulation frame.
    /// This function must be called at each simulation frame; output occurs only at those frames that are consistent
    /// with the given output frequencies.
    void Process(int sim_frame, const ChDriver::Inputs& driver_inputs) {
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
            ChVector<> box_pos(wheel_pos + ChVector<>(m_box_x, 0, -tire_radius));
            ChMatrix33<> box_rot(X_dir, Y_dir, Z_dir);

            m_indices[i] = FindParticlesInBox(m_sysFSI, ConvertOBB(ChFrame<>(box_pos, box_rot), m_box_size));
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

        m_veh_stream << m_vehicle->GetChTime() << "    ";
        for (int i = 0; i < m_num_veh_outputs; i++)
            m_veh_stream << o[i] << "  ";
        m_veh_stream << "\n";
    }

    static OBBspec ConvertOBB(const ChFrame<>& box_frame, const ChVector<>& box_size) {
        // Extract OBB position and rotation
        const ChVector<>& box_pos = box_frame.GetPos();
        ChVector<> Ax = box_frame.GetA().Get_A_Xaxis();
        ChVector<> Ay = box_frame.GetA().Get_A_Yaxis();
        ChVector<> Az = box_frame.GetA().Get_A_Zaxis();

        // Save as OBB spec
        OBBspec obb;
        obb.h = 0.5 * mR3(box_size.x(), box_size.y(), box_size.z());
        obb.p = mR3(box_pos.x(), box_pos.y(), box_pos.z());
        obb.ax = mR3(Ax.x(), Ax.y(), Ax.z());
        obb.ay = mR3(Ay.x(), Ay.y(), Ay.z());
        obb.az = mR3(Az.x(), Az.y(), Az.z());

        return obb;
    }

    std::shared_ptr<ChSystemFsi_impl> m_sysFSI;
    std::shared_ptr<WheeledVehicle> m_vehicle;
    std::array<std::shared_ptr<ChWheel>, 4> m_wheels;
    std::array<thrust::device_vector<int>, 4> m_indices;

    ChDriver::Inputs m_drv_inp;

    std::string m_dir;
    int m_major_skip;
    int m_minor_skip;
    int m_out_frames;
    int m_major_frame;
    int m_minor_frame;
    int m_last_major;

    bool m_out_vel;
    ChVector<> m_box_size;
    double m_box_x;

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

void CreateTerrain(ChSystem& sys, ChSystemFsi& sysFSI, std::shared_ptr<fsi::SimParams> params, const std::string& terrain_dir, bool enable_terrain_mesh) {
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
        ////ChVector<> tau(-params->rho0 * std::abs(params->gravity.z) * (-marker.z() + fzDim));
        ChVector<> tau(0);
        sysFSI.AddSphMarker(marker, params->rho0, 0, params->mu0, params->HSML, -1, VNULL, tau, VNULL);
        num_particles++;
    }
    is.close();

    // Set computational domain
    ChVector<> aabb_dim = aabb_max - aabb_min;
    aabb_dim.z() *= 50;
    sysFSI.SetBoundaries(aabb_min - 0.1 * aabb_dim, aabb_max + 0.1 * aabb_dim, params);

    // Setup sub domains for a faster neighbor particle searching
    sysFSI.SetSubDomain(params);

    // Set start/end particle array indices
    sysFSI.AddRefArray(0, num_particles, -1, -1);

    // Create ground body and attach BCE markers
    // Note: BCE markers must be created after SPH markers!
    auto body = std::shared_ptr<ChBody>(sys.NewBody());
    body->SetBodyFixed(true);
    sys.AddBody(body);

    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(vehicle::GetDataFile(terrain_dir + "/mesh.obj"), true, false);
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetMutable(false);
    body->AddVisualShape(trimesh_shape);

    if (enable_terrain_mesh) {
        MaterialInfo mat_info;
        mat_info.mu = 0.9f;
        auto mat = mat_info.CreateMaterial(sys.GetContactMethod());
        body->GetCollisionModel()->ClearModel();
        body->GetCollisionModel()->AddTriangleMesh(mat, trimesh, true, false, VNULL, ChMatrix33<>(1), 0.01);
        body->GetCollisionModel()->BuildModel();
        body->SetCollide(true);
    }

    sysFSI.AddBceFile(params, body, vehicle::GetDataFile(terrain_dir + "/bce_20mm.txt"), VNULL, QUNIT, 1.0, false);
}

std::shared_ptr<WheeledVehicle> CreateVehicle(ChSystem& sys,
                                              const ChCoordsys<>& init_pos,
                                              ChSystemFsi& sysFSI,
                                              std::shared_ptr<fsi::SimParams> params) {
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
    auto delta = params->MULT_INITSPACE * params->HSML;
    std::vector<ChVector<>> point_cloud;
    CreateMeshMarkers(trimesh, (double)delta, point_cloud);

    // Create and initialize the tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));
            vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
            sysFSI.AddFsiBody(wheel->GetSpindle());
            sysFSI.AddBceFromPoints(params, wheel->GetSpindle(), point_cloud, VNULL, QUNIT);
        }
    }

    return vehicle;
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string terrain_dir;
    double output_major_fps = 20;
    double output_minor_fps = 1000;
    int output_frames = 5;
    bool output_velocities = true;  // output particle velocities
    double filter_window = 0;       // do not filter data
    double vis_output_fps = 0;      // no post-processing visualization output
    bool run_time_vis = false;      // no run-time visualization
    bool verbose = true;

    if (!GetProblemSpecs(argc, argv, terrain_dir, output_major_fps, output_minor_fps, output_frames, output_velocities,
                         filter_window, vis_output_fps, run_time_vis, verbose)) {
        return 1;
    }

    bool sim_output = (output_major_fps > 0);
    bool use_filter = (filter_window > 0);
    bool vis_output = (vis_output_fps > 0);

    bool enable_terrain_mesh = false;

    // Create the Chrono systems
    ChSystemNSC sys;
    ChSystemFsi sysFSI(sys);

    // Load SPH parameter file
    cout << "Load SPH parameter file..." << endl;
    std::shared_ptr<fsi::SimParams> params = sysFSI.GetSimParams();
    sysFSI.SetSimParameter(vehicle::GetDataFile(terrain_dir + "/sph_params.json"), params, ChVector<>(1));
    sysFSI.SetDiscreType(false, false);
    sysFSI.SetWallBC(BceVersion::ORIGINAL);
    sysFSI.SetFluidDynamics(params->fluid_dynamic_type);

    // Set simulation data output and FSI information output
    std::string out_dir = GetChronoOutputPath() + "FSI_POLARIS/";
    std::string vis_dir = params->demo_dir;
    sysFSI.SetFsiInfoOutput(false);
    sysFSI.SetFsiOutputDir(params, vis_dir, out_dir, "");
    sysFSI.SetOutputLength(0);

    sys.Set_G_acc(ChVector<>(params->gravity.x, params->gravity.y, params->gravity.z));

    // Create terrain
    cout << "Create terrain..." << endl;
    CreateTerrain(sys, sysFSI, params, terrain_dir, enable_terrain_mesh);

    // Create vehicle
    cout << "Create vehicle..." << endl;
    ChCoordsys<> init_pos(ChVector<>(4, 0, 0.25), QUNIT);  //// TODO
    auto vehicle = CreateVehicle(sys, init_pos, sysFSI, params);

    // Create driver
    auto path = ChBezierCurve::read(vehicle::GetDataFile(terrain_dir + "/path.txt"));
    ChPathFollowerDriver driver(*vehicle, path, "my_path", 0);
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(0.8, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Finalize construction of FSI system
    sysFSI.Finalize();

#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    if (run_time_vis) {
        gl_window.Initialize(1280, 720, "JSON visualization", &sys);
        gl_window.SetCamera(ChVector<>(0, -4, 2), ChVector<>(5, 0, 0.5), ChVector<>(0, 0, 1));
        gl_window.SetRenderMode(opengl::SOLID);
        gl_window.EnableHUD(false);
    }
#endif

    // Enable data output
    cout << "===============================================================================" << endl;
    auto sim_dir = out_dir + filesystem::path(terrain_dir).filename() + "/";
    if (!filesystem::create_directory(filesystem::path(sim_dir))) {
        cout << "Error creating directory " << sim_dir << endl;
        return 1;
    }
    DataWriter data_writer(sysFSI.GetFsiData(), vehicle);
    data_writer.SetVerbose(verbose);
    data_writer.SaveVelocities(output_velocities);
    data_writer.UseFilteredData(use_filter, filter_window);
    data_writer.Initialize(sim_dir, output_major_fps, output_minor_fps, output_frames, params->dT);
    cout << "Simulation output data saved in: " << sim_dir << endl;
    cout << "===============================================================================" << endl;

    // Simulation loop
    double t = 0;
    double tend = 30;

    ChDriver::Inputs driver_inputs = {0, 0, 0};
    ChTerrain terrain;
    double x_max = path->getPoint(path->getNumPoints() - 1).x() - 4;

    int vis_output_steps = (int)std::ceil((1.0 / vis_output_fps) / params->dT);
    int vis_output_frame = 0;

    int frame = 0;
    while (t < tend) {
#ifdef CHRONO_OPENGL
        if (run_time_vis) {
            if (!gl_window.Active())
                break;
            gl_window.Render();
        }
#endif

        //// TODO
        if (vehicle->GetPos().x() > x_max)
            break;

        if (verbose)
            cout << "t = " << t << "  pos = " << vehicle->GetPos() << "  spd = " << vehicle->GetSpeed() << endl;

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

        // Set current driver inputs
        driver_inputs = driver.GetInputs();
        driver_inputs.m_braking = 0;
        if (t < 1)
            driver_inputs.m_throttle = 0;
        else if (t < 1.5)
            driver_inputs.m_throttle = (t - 1) / 0.5;
        else
            driver_inputs.m_throttle = 1;

        driver.Synchronize(t);
        vehicle->Synchronize(t, driver_inputs, terrain);

        // Advance system state
        driver.Advance(params->dT);
        sysFSI.DoStepDynamics_FSI();
        t += params->dT;

        frame++;
    }

    return 0;
}
