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

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------

std::string ground_obj = "terrain/sph_markers/rms4_20x4.obj";
std::string ground_bce = "terrain/sph_markers/rms4_20x4_20mm_bce.txt";
std::string sph_particles = "terrain/sph_markers/rms4_20x4_20mm_particles.txt";

std::string vehicle_json = "mrzr/vehicle/MRZR.json";
std::string powertrain_json = "mrzr/powertrain/MRZR_SimplePowertrain.json";
std::string tire_json = "mrzr/tire/MRZR_TMeasyTire.json";
std::string tire_coll_obj = "mrzr/meshes_new/Polaris_tire_collision.obj";

std::string sph_params = "fsi/input_json/test_VEH_SPH_Polaris.json";

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

void CreateTerrain(ChSystem& sys, ChSystemFsi& sysFSI, std::shared_ptr<fsi::SimParams> params) {
    // Create SPH markers with initial locations from file
    int num_particles = 0;
    ChVector<> aabb_min(std::numeric_limits<double>::max());
    ChVector<> aabb_max(-std::numeric_limits<double>::max());

    ChVector<> marker;
    std::string line;
    std::string cell;

    std::ifstream is(vehicle::GetDataFile(sph_particles));
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
    aabb_dim.z() *= 10;
    sysFSI.SetBoundaries(aabb_min - 1.5 * aabb_dim, aabb_max + 1.5 * aabb_dim, params);

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
    trimesh->LoadWavefrontMesh(vehicle::GetDataFile(ground_obj), true, false);
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetStatic(true);
    body->AddAsset(trimesh_shape);

    sysFSI.AddBceFile(params, body, vehicle::GetDataFile(ground_bce), VNULL, QUNIT, 1.0, false);
}

std::shared_ptr<WheeledVehicle> CreateVehicle(ChSystem& sys,
                                              const ChCoordsys<>& init_pos,
                                              ChSystemFsi& sysFSI,
                                              std::shared_ptr<fsi::SimParams> params) {
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
    // Create the Chrono systems
    ChSystemNSC sys;
    ChSystemFsi sysFSI(sys);

    // Load SPH parameter file
    std::cout << "Load SPH parameter file..." << std::endl;
    std::shared_ptr<fsi::SimParams> params = sysFSI.GetSimParams();
    sysFSI.SetSimParameter(GetChronoDataFile(sph_params), params, ChVector<>(1));
    sysFSI.SetDiscreType(false, false);
    sysFSI.SetFluidDynamics(params->fluid_dynamic_type);

    sys.Set_G_acc(ChVector<>(params->gravity.x, params->gravity.y, params->gravity.z));

    // Create terrain
    std::cout << "Create terrain..." << std::endl;
    CreateTerrain(sys, sysFSI, params);

    // Set simulation data output and FSI information output
    std::string out_dir = GetChronoOutputPath() + "FSI_M113/";
    std::string demo_dir;
    sysFSI.SetFsiInfoOutput(false);
    sysFSI.SetFsiOutputDir(params, demo_dir, out_dir, "");
    sysFSI.SetOutputLength(0);

    // Create vehicle
    std::cout << "Create vehicle..." << std::endl;
    ChCoordsys<> init_pos(ChVector<>(4, 0, 0.5), QUNIT);  //// TODO
    auto vehicle = CreateVehicle(sys, init_pos, sysFSI, params);

    // Finalize construction of FSI system
    sysFSI.Finalize();

#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "JSON visualization", &sys);
    gl_window.SetCamera(ChVector<>(0, -4, 2), ChVector<>(5, 0, 0.5), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::SOLID);
    gl_window.EnableHUD(false);
#endif

    // Simulation loop
    double t = 0;
    double tend = 10;
    ChTerrain terrain;
    while (t < tend) {
#ifdef CHRONO_OPENGL
        if (!gl_window.Active())
            break;
        gl_window.Render();
#endif
        std::cout << "t = " << t << "  pos = " << vehicle->GetVehiclePos() << std::endl;

        // Set current driver inputs
        ChDriver::Inputs driver_inputs = {0, 0.8, 0};  //// TODO
        vehicle->Synchronize(t, driver_inputs, terrain);

        // Advance system state
        sysFSI.DoStepDynamics_FSI();
        t += params->dT;
    }

    return 0;
}
