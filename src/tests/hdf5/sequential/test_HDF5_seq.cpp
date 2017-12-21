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
// Saving simulation output in HDF5 file
//
// =============================================================================

#include <fstream>
#include <sstream>

#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "H5Cpp.h"

using namespace chrono;
using namespace chrono::irrlicht;

void create_objects(ChSystemNSC& sys);
void output_frame_ascii(ChSystemNSC& sys, int frame, std::ofstream& file);
void output_frame_hdf5(ChSystemNSC& sys, int frame, H5::H5File& file);

bool visualization = true;
int nballs = 1000;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;

    ChIrrApp application(&sys, L"Collisions between objects", irr::core::dimension2d<irr::u32>(800, 600), false);
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(0, 30, -10));

    create_objects(sys);

    application.AssetBindAll();
    application.AssetUpdateAll();

    sys.SetSolverType(ChSolver::Type::SOR);
    sys.SetMaxItersSolverSpeed(20);
    sys.SetMaxItersSolverStab(5);

    // Open the ASCII file
    std::string fileASCII_name = "testfile_seq.txt";
    std::ofstream fileASCII;
    fileASCII.open(fileASCII_name);

    // Create output HDF5 file (overwrite existing file)
    // and create a group for the simulation frames
    H5std_string fileHDF5_name("testfile_seq.h5");
    H5::H5File fileHDF5(fileHDF5_name, H5F_ACC_TRUNC);
    H5::Group frames_group(fileHDF5.createGroup("/Frames"));

    double time_step = 0.02;
    int frame = 0;
    int out_frame = 0;
    while (application.GetDevice()->run()) {
        if (out_frame < 200 && frame % 100 == 0) {
            std::cout << "Frame: " << out_frame << "  Time: " << sys.GetChTime() << std::endl;
            output_frame_ascii(sys, out_frame, fileASCII);
            output_frame_hdf5(sys, out_frame, fileHDF5);
            out_frame++;
        }

        if (visualization) {
            application.BeginScene();
            application.DrawAll();
        }

        sys.DoStepDynamics(time_step);
        
        if (visualization) {
            application.EndScene();
        }

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

struct vec {
    double x, y, z;
};
struct quat {
    double w, x, y, z;
};

void output_frame_hdf5(ChSystemNSC& sys, int frame, H5::H5File& file) {
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
    H5::CompType vec_type(sizeof(vec));
    vec_type.insertMember("x", HOFFSET(vec, x), H5::PredType::NATIVE_DOUBLE);
    vec_type.insertMember("y", HOFFSET(vec, y), H5::PredType::NATIVE_DOUBLE);
    vec_type.insertMember("z", HOFFSET(vec, z), H5::PredType::NATIVE_DOUBLE);

    H5::CompType quat_type(sizeof(quat));
    quat_type.insertMember("e0", HOFFSET(quat, w), H5::PredType::NATIVE_DOUBLE);
    quat_type.insertMember("e1", HOFFSET(quat, x), H5::PredType::NATIVE_DOUBLE);
    quat_type.insertMember("e2", HOFFSET(quat, y), H5::PredType::NATIVE_DOUBLE);
    quat_type.insertMember("e3", HOFFSET(quat, z), H5::PredType::NATIVE_DOUBLE);

    // Populate body group
    auto nbodies = sys.Get_bodylist()->size();
    if (nbodies > 0) {
        H5::Group b_group(f_group.createGroup("Bodies"));
        hsize_t dim[] = { nbodies };
        H5::DataSpace dataspace(1, dim);

        // Write positions and velocities
        auto bodies = *sys.Get_bodylist();
        std::vector<vec> pos(nbodies);
        std::vector<quat> rot(nbodies);
        std::vector<vec> lin_vel(nbodies);
        std::vector<vec> ang_vel(nbodies);
        for (auto i = 0; i < nbodies; i++) {
            const ChVector<>& p = bodies[i]->GetPos();
            const ChQuaternion<>& q = bodies[i]->GetRot();
            const ChVector<>& lv = bodies[i]->GetPos_dt();
            const ChVector<>& av = bodies[i]->GetWvel_loc();
            pos[i] = { p.x(), p.y(), p.z() };
            rot[i] = { q.e0(), q.e1(), q.e2(), q.e3() };
            lin_vel[i] = { lv.x(), lv.x(), lv.x() };
            ang_vel[i] = { av.x(), av.y(), av.z() };
        }

        {
            H5::DataSet set_pos = b_group.createDataSet("Positions", vec_type, dataspace);
            H5::DataSet set_rot = b_group.createDataSet("Rotations", quat_type, dataspace);
            set_pos.write(pos.data(), vec_type);
            set_rot.write(rot.data(), quat_type);
        }
        {
            H5::DataSet set_lin = b_group.createDataSet("Linear Velocities", vec_type, dataspace);
            H5::DataSet set_ang = b_group.createDataSet("Angular Velocities", vec_type, dataspace);
            set_lin.write(lin_vel.data(), vec_type);
            set_ang.write(ang_vel.data(), vec_type);
        }

    }

    // Populate link group
    auto nlinks = sys.Get_linklist()->size();
    if (nlinks > 0) {
        H5::Group l_group(f_group.createGroup("Links"));
    }
}

std::ostream& operator<<(std::ostream& out, const vec& a) {
    return out << a.x << " " << a.y << " " << a.z;
}

std::ostream& operator<<(std::ostream& out, const quat& a) {
    return out << a.w << " " << a.x << " " << a.y << " " << a.z;
}

void output_frame_ascii(ChSystemNSC& sys, int frame, std::ofstream& file) {
    file << frame << " " << sys.GetChTime() << std::endl;

    auto nbodies = sys.Get_bodylist()->size();
    if (nbodies > 0) {
        // Write positions and velocities
        auto bodies = *sys.Get_bodylist();
        std::vector<vec> pos(nbodies);
        std::vector<quat> rot(nbodies);
        std::vector<vec> lin_vel(nbodies);
        std::vector<vec> ang_vel(nbodies);
        for (auto i = 0; i < nbodies; i++) {
            const ChVector<>& p = bodies[i]->GetPos();
            const ChQuaternion<>& q = bodies[i]->GetRot();
            const ChVector<>& lv = bodies[i]->GetPos_dt();
            const ChVector<>& av = bodies[i]->GetWvel_loc();
            pos[i] = { p.x(), p.y(), p.z() };
            rot[i] = { q.e0(), q.e1(), q.e2(), q.e3() };
            lin_vel[i] = { lv.x(), lv.x(), lv.x() };
            ang_vel[i] = { av.x(), av.y(), av.z() };
        }
        std::copy(pos.begin(), pos.end(), std::ostream_iterator<vec>(file, "\t"));
        file << std::endl;
        std::copy(rot.begin(), rot.end(), std::ostream_iterator<quat>(file, "\t"));
        file << std::endl;
        std::copy(lin_vel.begin(), lin_vel.end(), std::ostream_iterator<vec>(file, "\t"));
        file << std::endl;
        std::copy(ang_vel.begin(), ang_vel.end(), std::ostream_iterator<vec>(file, "\t"));
        file << std::endl;
    }
}

void create_objects(ChSystemNSC& sys) {
    // Falling objects
    for (int bi = 0; bi < nballs; bi++) {
        auto sphere = std::make_shared<ChBodyEasySphere>(0.5, 1000, true, true);
        sphere->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        sphere->GetMaterialSurfaceNSC()->SetFriction(0.2f);

        sys.Add(sphere);

        auto texture = std::make_shared<ChTexture>();
        texture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
        sphere->AddAsset(texture);
    }

    // Container
    auto material = std::make_shared<ChMaterialSurfaceNSC>();
    auto ground = utils::CreateBoxContainer(&sys, -1, material, ChVector<>(10, 10, 10), 0.4, ChVector<>(0, -4, 0), QUNIT,
                                            true, true, true, false);
    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    ground->AddAsset(texture);

    // Rotating mixer
    auto mixer = std::make_shared<ChBodyEasyBox>(10, 5, 1, 4000, true, true);
    mixer->SetPos(ChVector<>(0, -1.6, 0));
    mixer->GetMaterialSurfaceNSC()->SetFriction(0.4f);
    sys.Add(mixer);

    // Motor
    auto motor = std::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(mixer, ground, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
    auto fun = std::make_shared<ChFunction_Const>(0.5);
    motor->SetSpeedFunction(fun);
    sys.AddLink(motor);
}
