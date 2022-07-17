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

#pragma once

#include <array>
#include <fstream>

#include <thrust/copy.h>
#include <thrust/gather.h>
#include <thrust/for_each.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/functional.h>
#include <thrust/execution_policy.h>

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono/utils/ChFilters.h"
#include "chrono_fsi/ChSystemFsi.h"

class DataWriter {
  public:
    /// Construct an output data writer for the specified FSI system and vehicle.
    DataWriter(chrono::fsi::ChSystemFsi& sysFSI, std::shared_ptr<chrono::vehicle::WheeledVehicle> vehicle);

    ~DataWriter();

    /// Enable/disable output data running-average filtering (default: true).
    void UseFilteredData(bool val, double window);

    /// Enable/disable terminal messages (default: true).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Enable/disable output of SPH particle velocities (default: true).
    void SaveVelocities(bool val) { m_out_vel = val; }

    /// Set the location (relative to tire bottom point) and dimension (x and y) of the soil sampling domain.
    void SetSamplingVolume(const chrono::ChVector<>& offset, const chrono::ChVector2<>& size);

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
    void Initialize(const std::string& dir, double major_FPS, double minor_FPS, int num_minor, double step_size);

    /// Run the data writer at the current simulation frame.
    /// This function must be called at each simulation frame; output occurs only at those frames that are consistent
    /// with the given output frequencies.
    void Process(int sim_frame, const chrono::vehicle::DriverInputs& driver_inputs);

  private:
    void Reset();

    void Write();

    void CollectVehicleData();

    void WriteVehicleData(const std::string& filename);

    void WriteParticlePos(chrono::fsi::ChSystemFsi& sysFSI,
                          const thrust::device_vector<int>& indices_D,
                          const std::string& filename);

    void WriteParticlePosVel(chrono::fsi::ChSystemFsi& sysFSI,
                             const thrust::device_vector<int>& indices_D,
                             const std::string& filename);

    chrono::fsi::ChSystemFsi& m_sysFSI;
    std::shared_ptr<chrono::vehicle::WheeledVehicle> m_vehicle;
    std::array<std::shared_ptr<chrono::vehicle::ChWheel>, 4> m_wheels;
    std::array<thrust::device_vector<int>, 4> m_indices;

    chrono::vehicle::DriverInputs m_drv_inp;

    std::string m_dir;
    int m_major_skip;
    int m_minor_skip;
    int m_out_frames;
    int m_major_frame;
    int m_minor_frame;
    int m_last_major;

    bool m_out_vel;
    chrono::ChVector<> m_box_size;
    chrono::ChVector<> m_box_offset;
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
