// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_opengl/ChOpenGLWindow.h"

#include "H5Cpp.h"


using namespace chrono;
using namespace chrono::collision;

void create_objects(ChSystemParallelNSC& sys);
void output_frame_ascii(ChSystemParallelNSC& sys, int frame, std::ofstream& file);
void output_frame_hdf5(ChSystemParallelNSC& sys, int frame, H5::H5File& file);

bool visualization = true;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create system
    int threads = 8;
    double time_step = 5e-3;
    uint max_iteration = 30;
    real tolerance = 1e-3;

    ChSystemParallelNSC sys;

    // Set number of threads.
    int max_threads = CHOMPfunctions::GetNumProcs();
    if (threads > max_threads)
        threads = max_threads;
    sys.SetParallelThreadNumber(threads);
    CHOMPfunctions::SetNumThreads(threads);

    // Set gravitational acceleration
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Create objects
    create_objects(sys);

    // Set solver parameters
    sys.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    sys.GetSettings()->solver.max_iteration_normal = 0;
    sys.GetSettings()->solver.max_iteration_sliding = max_iteration;
    sys.GetSettings()->solver.max_iteration_spinning = 0;
    sys.GetSettings()->solver.max_iteration_bilateral = 0;
    sys.GetSettings()->solver.tolerance = tolerance;
    sys.GetSettings()->solver.alpha = 0;
    sys.GetSettings()->solver.contact_recovery_speed = 10000;
    sys.ChangeSolverType(SolverType::APGD);
    sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    sys.GetSettings()->collision.collision_envelope = 0.01;
    sys.GetSettings()->collision.bins_per_axis = vec3(20, 20, 10);

    // Open the ASCII file
    std::string fileASCII_name = "testfile_par.txt";
    std::ofstream fileASCII;
    fileASCII.open(fileASCII_name);

    // Create output HDF5 file (overwrite existing file)
    // and create a group for the simulation frames
    H5std_string fileHDF5_name("testfile_par.h5");
    H5::H5File fileHDF5(fileHDF5_name, H5F_ACC_TRUNC);
    H5::Group frames_group(fileHDF5.createGroup("/Frames"));

    // Perform the simulation
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "HDF5 parallel", &sys);
    gl_window.SetCamera(ChVector<>(0, -30, 10), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    // Hack: take one step of length 0 to initialize data arrays
    sys.DoStepDynamics(0);

    int frame = 0;
    int out_frame = 0;
    while (gl_window.Active()) {
        if (out_frame < 200 && frame % 100 == 0) {
            std::cout << "Frame: " << out_frame << "  Time: " << sys.GetChTime() << std::endl;
            output_frame_ascii(sys, out_frame, fileASCII);
            output_frame_hdf5(sys, out_frame, fileHDF5);
            out_frame++;
        }

        gl_window.DoStepDynamics(time_step);
        gl_window.Render();

        frame++;
    }

    // Close ASCII and HDF5 files
    fileASCII.close();
    fileHDF5.close();

    return 0;
}

std::string format_number(int num, int precision) {
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(precision) << num;
    return out.str();
}

void output_frame_hdf5(ChSystemParallelNSC& sys, int frame, H5::H5File& file) {
    double time = sys.GetChTime();
    auto frame_name = std::string("Frame_") + format_number(frame, 6);

    // Open the frames group and create a group for this frame
    H5::Group frames_group = file.openGroup("/Frames");
    H5::Group f_group(frames_group.createGroup(frame_name));

    // Create an attribute with timestamp
    {
        H5::DataSpace dataspace(H5S_SCALAR);
        H5::Attribute att = f_group.createAttribute("Timestamp", H5::PredType::NATIVE_DOUBLE, dataspace);
        att.write(H5::PredType::NATIVE_DOUBLE, &time);
    }

    // HDF5 compund datatypes
    H5::CompType vec_type(sizeof(real3));
    vec_type.insertMember("x", HOFFSET(real3, x), H5::PredType::NATIVE_DOUBLE);
    vec_type.insertMember("y", HOFFSET(real3, y), H5::PredType::NATIVE_DOUBLE);
    vec_type.insertMember("z", HOFFSET(real3, z), H5::PredType::NATIVE_DOUBLE);

    H5::CompType quat_type(sizeof(quaternion));
    quat_type.insertMember("e0", HOFFSET(quaternion, w), H5::PredType::NATIVE_DOUBLE);
    quat_type.insertMember("e1", HOFFSET(quaternion, x), H5::PredType::NATIVE_DOUBLE);
    quat_type.insertMember("e2", HOFFSET(quaternion, y), H5::PredType::NATIVE_DOUBLE);
    quat_type.insertMember("e3", HOFFSET(quaternion, z), H5::PredType::NATIVE_DOUBLE);
  
    auto dm = sys.data_manager;
    auto nbodies = dm->num_rigid_bodies;

    // Populate body group
    if (nbodies > 0) {
        H5::Group b_group(f_group.createGroup("Bodies"));
        hsize_t dim[] = { nbodies };
        H5::DataSpace dataspace(1, dim);

        // Write mass information
        {
            auto blist = *sys.Get_bodylist();
            std::vector<double> mass(nbodies);
            for (uint i = 0; i < nbodies; i++) {
                mass[i] = blist[i]->GetMass();
            }
            std::cout << "    mass size: " << mass.size() << std::endl;
            H5::DataSet set_mass = b_group.createDataSet("Mass", H5::PredType::NATIVE_DOUBLE, dataspace);
            set_mass.write(mass.data(), H5::PredType::NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT);
        }

        // Write positions and orientations
        {
            auto& pos = dm->host_data.pos_rigid;
            auto& rot = dm->host_data.rot_rigid;
            H5::DataSet set_pos = b_group.createDataSet("Positions", vec_type, dataspace);
            H5::DataSet set_rot = b_group.createDataSet("Rotations", quat_type, dataspace);
            set_pos.write(pos.data(), vec_type);
            set_rot.write(rot.data(), quat_type);
        }

        // Write velocities
        {
            auto& vel = dm->host_data.v;
            custom_vector<real3> lin_vel(nbodies);
            custom_vector<real3> ang_vel(nbodies);
            for (uint i = 0; i < nbodies; i++) {
                lin_vel[i] = real3(vel[i * 6 + 0], vel[i * 6 + 1], vel[i * 6 + 2]);
                ang_vel[i] = real3(vel[i * 6 + 3], vel[i * 6 + 4], vel[i * 6 + 5]);
            }
            H5::DataSet set_lin = b_group.createDataSet("Linear Velocities", vec_type, dataspace);
            H5::DataSet set_ang = b_group.createDataSet("Angular Velocities", vec_type, dataspace);
            set_lin.write(lin_vel.data(), vec_type);
            set_ang.write(ang_vel.data(), vec_type);

            ////hsize_t dims[2] = {nbodies, 6};
            ////H5::DataSpace dataspace(2, dims);
            ////H5::DataSet dataset = b_group.createDataSet("Velocities", H5::PredType::NATIVE_DOUBLE, dataspace);
            ////dataset.write(vel.data(), H5::PredType::NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT);
        }

    }
}

void output_frame_ascii(ChSystemParallelNSC& sys, int frame, std::ofstream& file) {
    file << frame << " " << sys.GetChTime() << std::endl;

    auto dm = sys.data_manager;
    auto nbodies = dm->num_rigid_bodies;
    if (nbodies > 0) {
        // Write positions and velocities
        auto& pos = dm->host_data.pos_rigid;
        auto& rot = dm->host_data.rot_rigid;
        auto& vel = dm->host_data.v;
    }
}

// Create mechanical system
void create_objects(ChSystemParallelNSC& sys) {
    // Bin half-dimensions
    ChVector<> hdim(6, 6, 6);
    double hthick = 0.1;

    // Initial height of granular material
    double height = 10;

    // Ball dimensions
    double radius = 0.2;
    double density = 200;
    double volume = (4.0 / 3) * CH_C_PI * radius * radius * radius;
    double mass = density * volume;
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

    // Common material
    auto mat = std::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);

    // Create the containing bin
    auto bin = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
    bin->SetMaterialSurface(mat);
    bin->SetIdentifier(-100);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hdim.y(), hthick), ChVector<>(0, 0, -hthick));
    utils::AddBoxGeometry(bin.get(), ChVector<>(hthick, hdim.y(), hdim.z()),
        ChVector<>(-hdim.x() - hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), ChVector<>(hthick, hdim.y(), hdim.z()),
        ChVector<>(hdim.x() + hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hthick, hdim.z()),
        ChVector<>(0, -hdim.y() - hthick, hdim.z()));
    utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hthick, hdim.z()),
        ChVector<>(0, hdim.y() + hthick, hdim.z()));
    bin->GetCollisionModel()->SetFamily(1);
    bin->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);
    bin->GetCollisionModel()->BuildModel();

    sys.AddBody(bin);

    // Create the falling balls
    utils::PDSampler<> sampler(0.5);
    auto points =
        sampler.SampleBox(ChVector<>(0, 0, height / 2 + 1), ChVector<>(hdim.x() - 1, hdim.y() - 1, height / 2));

    int ballId = 0;
    for (auto point : points) {
        auto ball = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
        ball->SetMaterialSurface(mat);

        ball->SetIdentifier(ballId++);
        ball->SetMass(mass);
        ball->SetInertiaXX(inertia);
        ball->SetPos(point);
        ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
        ball->SetBodyFixed(false);
        ball->SetCollide(true);

        ball->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(ball.get(), radius);
        ball->GetCollisionModel()->BuildModel();

        sys.AddBody(ball);
    }
}
