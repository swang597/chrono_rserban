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
// Author: Deniz Tanyildiz, Radu Serban
// =============================================================================
//
// Object drop on SPH terrain
//
// =============================================================================

#include <cstdio>
#include <string>
#include <stdexcept>
#include <iomanip>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/ChVisualizationFsi.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::vehicle;

using std::cout;
using std::cin;
using std::endl;

// ===================================================================================================================

// Dimension of the container
double bxDim = 1.0;
double byDim = 1.0;
double bzDim = 0.5;

// ===================================================================================================================

std::shared_ptr<chrono::ChBody> CreateSolids(chrono::ChSystemNSC& sys,
                                                 chrono::fsi::ChSystemFsi& sysFSI) {
    // Set common material Properties
    auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mat->SetYoungModulus(1e8);
    mat->SetFriction(0.2f);
    mat->SetRestitution(0.05f);
    mat->SetAdhesion(0);

    // Get particle spacing in the simulation
    auto init_spacing = sysFSI.GetInitialSpacing();

    // Create bottom plate
    double factor = 3.0;
    double gxDim = factor * bxDim;
    double gyDim = factor * byDim;
    double gzDim = 0.1;
    double top_z = -3 * init_spacing;
    ChVector<> size_XY(bxDim / 2 + 3 * init_spacing, byDim / 2 + 0 * init_spacing, 2 * init_spacing);

    auto ground = chrono_types::make_shared<ChBodyEasyBox>(gxDim, gyDim, gzDim, 1000, false, true, mat);
    ground->SetPos(ChVector<>(0.0, 0.0, top_z - gzDim / 2));
    ground->SetCollide(true);
    ground->SetBodyFixed(true);
    sys.AddBody(ground);

    // Add BCE markers
    sysFSI.AddBoxBCE(ground, top_z, QUNIT, size_XY, 12);

    // Create a falling sphere
    double sphere_radius = 0.25;
    double volume = chrono::utils::CalcSphereVolume(sphere_radius);
    double density = 7840;
    double mass = density * volume;
    ChVector<> sphere_pos = ChVector<>(0.0, 0, bzDim + sphere_radius);
    ChVector<> sphere_vel = ChVector<>(0.0, 0.0, 0.0);
    ChVector<> gyration = chrono::utils::CalcSphereGyration(sphere_radius).diagonal();

    auto sphere = chrono_types::make_shared<ChBody>();
    sphere->SetPos(sphere_pos);
    sphere->SetPos_dt(sphere_vel);
    sphere->SetMass(mass);
    sphere->SetInertiaXX(mass * gyration);
    sphere->SetCollide(true);
    sphere->SetBodyFixed(false);
    sphere->GetCollisionModel()->ClearModel();
    chrono::utils::AddSphereGeometry(sphere.get(), mat, sphere_radius, VNULL, QUNIT);
    sphere->GetCollisionModel()->BuildModel();
    sys.AddBody(sphere);

    // Add this body to the FSI system and create BCEs
    sysFSI.AddFsiBody(sphere);
    sysFSI.AddSphereBCE(sphere, ChVector<>(0), ChQuaternion<>(1, 0, 0, 0), sphere_radius);

    return sphere;
}

int main(int argc, char* argv[]) {
    double tend = 30;
    double step_size = 5e-4;
    double output_fps = 120;
    double render_fps = 120;

    bool render = true;
    bool output = true;
    bool verbose = true;

    // Create the Chrono systems
    ChSystemNSC sys;
    ChSystemFsi sysFSI(sys);

    const ChVector<> gravity(0, 0, -9.81);
    sysFSI.Set_G_acc(gravity);
    sys.Set_G_acc(gravity);

    ChSystemFsi::ElasticMaterialProperties mat_props;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.stress = 0;  // default
    mat_props.viscosity_alpha = 0.5;
    mat_props.viscosity_beta = 0.0;
    mat_props.mu_I0 = 0.03;
    mat_props.mu_fric_s = 0.7;
    mat_props.mu_fric_2 = 0.7;
    mat_props.average_diam = 0.005;
    mat_props.friction_angle = CH_C_PI / 10;  // default
    mat_props.dilation_angle = CH_C_PI / 10;  // default
    mat_props.cohesion_coeff = 0;             // default
    mat_props.kernel_threshold = 0.8;
    sysFSI.SetElasticSPH(mat_props);

    double rho = 1700.0;
    double cohesion = 1.0e2;
    sysFSI.SetDensity(rho);
    sysFSI.SetCohesionForce(cohesion);

    double init_spacing = 0.01;
    double kernel_length = 0.01;
    sysFSI.SetInitialSpacing(init_spacing);
    sysFSI.SetKernelLength(kernel_length);

    sysFSI.SetContainerDim(ChVector<>(bxDim, byDim, bzDim));
    sysFSI.SetDiscreType(false, false);
    sysFSI.SetWallBC(BceVersion::ORIGINAL);
    sysFSI.SetRigidBodyBC(BceVersion::ORIGINAL);
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);
    sysFSI.SetOutputLength(0);
    sysFSI.SetVerbose(false);

    sysFSI.SetStepSize(step_size);
    sysFSI.SetMaxStepSize(step_size);

    // Set up the periodic boundary condition (if not, set relative larger values)
    ChVector<> cMin(-bxDim / 2 * 10, -byDim / 2 - 0.5 * init_spacing, -bzDim * 10);
    ChVector<> cMax(bxDim / 2 * 10, byDim / 2 + 0.5 * init_spacing, bzDim * 10);
    sysFSI.SetBoundaries(cMin, cMax);

    // Initialize the SPH particles
    ChVector<> boxCenter(0.0, 0.0, bzDim / 2);
    ChVector<> boxHalfDim(bxDim / 2, byDim / 2, bzDim / 2);
    sysFSI.AddBoxSPH(init_spacing, kernel_length, boxCenter, boxHalfDim);

    // Create bottom plate and dropping sphere with BCE markers
    cout << "Create sphere..." << endl;
    auto sphere = CreateSolids(sys, sysFSI);

#ifdef CHRONO_OPENGL
    // Create run-time visualization
    opengl::ChVisualSystemOpenGL vis;
    ChVisualizationFsi visFSI(&sysFSI, &vis);
    visFSI.SetTitle("Object drop test");
    visFSI.SetSize(1280, 720);
    visFSI.SetCameraPosition(ChVector<>(-7, 0, 6), ChVector<>(1, 0, 0.5));
    visFSI.SetCameraMoveScale(1.0f);
    visFSI.EnableFluidMarkers(true);
    visFSI.EnableRigidBodyMarkers(false);
    visFSI.EnableBoundaryMarkers(false);
    visFSI.SetRenderMode(ChVisualizationFsi::RenderMode::SOLID);

    vis.AttachSystem(&sys);
    vis.Initialize();
#endif

    // Output directory
    std::string out_dir = GetChronoOutputPath() + "COHESION_SPH/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    // Simulation loop
    int output_steps = (int)std::round((1.0 / output_fps) / step_size);
    int render_steps = (int)std::round((1.0 / render_fps) / step_size);
    int vis_output_frame = 0;

    double t = 0;
    int frame = 0;
    sysFSI.Initialize();
#ifdef CHRONO_OPENGL
    visFSI.Initialize();
#endif

    while (t < tend) {
        // Visualization data output
        if (output && frame % output_steps == 0) {
            sysFSI.PrintParticleToFile(out_dir);
            vis_output_frame++;
        }

#ifdef CHRONO_OPENGL
        // Run-time visualization
        if (render && frame % render_steps == 0) {
            if (!visFSI.Render())
                break;
        }
#endif

        // Advance both FSI and embedded MBD systems
        sysFSI.DoStepDynamics_FSI();
        t += step_size;

        frame++;
    }

    return 0;
}

