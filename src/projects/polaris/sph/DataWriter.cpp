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
// Output data writer for Polaris on SPH terrain system
//
// =============================================================================

#include <array>

#include <thrust/copy.h>
#include <thrust/gather.h>
#include <thrust/for_each.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/functional.h>
#include <thrust/execution_policy.h>

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_fsi/ChSystemFsi.h"

#include "DataWriter.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fsi;

DataWriter::DataWriter(ChSystemFsi& sysFSI, std::shared_ptr<WheeledVehicle> vehicle)
    : m_sysFSI(sysFSI), m_vehicle(vehicle), m_out_vel(false), m_filter(true), m_filter_window(0.05), m_verbose(true) {
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

DataWriter::~DataWriter() {
    m_veh_stream.close();
}

void DataWriter::UseFilteredData(bool val, double window) {
    m_filter = val;
    m_filter_window = window;
}

void DataWriter::SetSamplingVolume(const ChVector<>& offset, const ChVector2<>& size) {
    m_box_size.x() = size.x();
    m_box_size.y() = size.y();
    m_box_offset = offset;
}

void DataWriter::Initialize(const std::string& dir,
                            double major_FPS,
                            double minor_FPS,
                            int num_minor,
                            double step_size) {
    m_dir = dir;
    m_out_frames = num_minor;
    m_major_skip = (int)std::round((1.0 / major_FPS) / step_size);
    m_minor_skip = (int)std::round((1.0 / minor_FPS) / step_size);
    m_major_frame = -1;
    m_last_major = -1;
    m_minor_frame = 0;

    // Sanity check
    if (m_minor_skip * m_out_frames > m_major_skip) {
        std::cout << "Error: Incompatible output frequencies!" << std::endl;
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

    std::cout << "Wheel sampling box size:   " << m_box_size << std::endl;
    std::cout << "Wheel sampling box offset: " << m_box_offset << std::endl;
}

void DataWriter::Process(int sim_frame, const DriverInputs& driver_inputs) {
    m_drv_inp = driver_inputs;

    CollectVehicleData();

    if (sim_frame % m_major_skip == 0) {
        m_last_major = sim_frame;
        m_major_frame++;
        m_minor_frame = 0;
        if (m_verbose)
            std::cout << "Start collection " << m_major_frame << std::endl;
        Reset();
    }
    if (m_last_major > 0 && (sim_frame - m_last_major) % m_minor_skip == 0 && m_minor_frame < m_out_frames) {
        if (m_verbose)
            std::cout << "    Output data " << m_major_frame << "/" << m_minor_frame << std::endl;
        Write();
        m_minor_frame++;
    }
    if (m_verbose)
        std::cout << std::flush;
}

void DataWriter::Reset() {
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

void DataWriter::Write() {
    for (int i = 0; i < 4; i++) {
        std::string filename = m_dir + "/soil_" + std::to_string(m_major_frame) + "_" + std::to_string(m_minor_frame) +
                               "_w" + std::to_string(i) + ".csv ";
        if (m_out_vel)
            WriteParticlePosVel(m_sysFSI, m_indices[i], filename);
        else
            WriteParticlePos(m_sysFSI, m_indices[i], filename);
    }

    {
        std::string filename =
            m_dir + "/vehicle_" + std::to_string(m_major_frame) + "_" + std::to_string(m_minor_frame) + ".csv ";
        WriteVehicleData(filename);
    }
}

void DataWriter::CollectVehicleData() {
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

void DataWriter::WriteVehicleData(const std::string& filename) {
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

void DataWriter::WriteParticlePos(ChSystemFsi& sysFSI,
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

void DataWriter::WriteParticlePosVel(ChSystemFsi& sysFSI,
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
