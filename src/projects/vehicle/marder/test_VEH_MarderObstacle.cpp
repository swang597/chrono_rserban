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
// Authors: Radu Serban / Rainer Gericke
// =============================================================================
//
// Demonstration program for Marder vehicle on rigid NRMM trapezoidal obstacle
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/core/ChTimer.h"
#include "chrono/solver/ChSolverBB.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"

#include "chrono_vehicle/terrain/ObsModTerrain.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/marder/Marder.h"

#ifdef CHRONO_PARDISO_MKL
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;
using namespace chrono::vehicle::marder;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================
// Initial vehicle position
ChVector<> initLoc(0, 0, 0.9);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Simulation step size
double step_size = 5e-4;

// Use HHT + MKL
bool use_mkl = false;

// Time interval between two render frames
double render_step_size = 1.0 / 120;  // FPS = 120

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 2.0, 0.0);

// Driver input files
std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("Marder/driver/SteeringController.json");
std::string speed_controller_file("Marder/driver/SpeedController.json");

// Output directories
const std::string out_top_dir = GetChronoOutputPath() + "Marder";
const std::string out_dir = out_top_dir + "/OBSTACLE";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string img_dir = out_dir + "/IMG";

// Output
bool povray_output = false;
bool img_output = false;
bool dbg_output = false;

// =============================================================================

// =============================================================================
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChFunction_Recorder accTravel;
    accTravel.AddPoint(1.0, 1.0);
    accTravel.AddPoint(5.0, 10.0);
    accTravel.AddPoint(10.0, 25.0);
    accTravel.AddPoint(15.0, 80.0);
    accTravel.AddPoint(20.0, 300.0);

    double target_speed = 2.0;
    double xpos_max = 100.0;
    initLoc.x() = -accTravel.Get_y(target_speed);

    // --------------------------
    // Construct the Marder vehicle
    // --------------------------

    ChContactMethod contact_method = ChContactMethod::SMC;
    CollisionType chassis_collision_type = CollisionType::NONE;
    ////TrackShoeType shoe_type = TrackShoeType::SINGLE_PIN;
    ////DrivelineTypeTV driveline_type = DrivelineTypeTV::SIMPLE;
    BrakeType brake_type = BrakeType::SIMPLE;
    EngineModelType engine_type = EngineModelType::SIMPLE_MAP;
    TransmissionModelType transmission_type = TransmissionModelType::SIMPLE_MAP;

    Marder marder;
    marder.SetContactMethod(contact_method);
    ////marder.SetTrackShoeType(shoe_type);
    ////marder.SetDrivelineType(driveline_type);
    marder.SetBrakeType(brake_type);
    marder.SetEngineType(engine_type);
    marder.SetTransmissionType(transmission_type);
    marder.SetChassisCollisionType(chassis_collision_type);

    ////marder.SetChassisFixed(true);
    ////marder.CreateTrack(false);

    // Disable gravity in this simulation
    ////marder.GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));

    // Control steering type (enable crossdrive capability)
    ////marder.GetDriveline()->SetGyrationMode(true);

    // ------------------------------------------------
    // Initialize the vehicle at the specified position
    // ------------------------------------------------
    marder.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    marder.Initialize();

    // Set visualization type for vehicle components.
    VisualizationType track_vis = VisualizationType::MESH;
    marder.SetChassisVisualizationType(VisualizationType::MESH);
    marder.SetSprocketVisualizationType(track_vis);
    marder.SetIdlerVisualizationType(track_vis);
    marder.SetRollerVisualizationType(track_vis);
    marder.SetSuspensionVisualizationType(track_vis);
    marder.SetRoadWheelVisualizationType(track_vis);
    marder.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // --------------------------------------------------
    // Control internal collisions and contact monitoring
    // --------------------------------------------------

    // Enable contact on all tracked vehicle parts, except the left sprocket
    ////marder.GetVehicle().SetCollide(TrackedCollisionFlag::ALL & (~TrackedCollisionFlag::SPROCKET_LEFT));

    // Disable contact for all tracked vehicle parts
    ////marder.GetVehicle().SetCollide(TrackedCollisionFlag::NONE);

    // Disable all contacts for vehicle chassis (if chassis collision was defined)
    ////marder.GetVehicle().SetChassisCollide(false);

    // Disable only contact between chassis and track shoes (if chassis collision was defined)
    ////marder.GetVehicle().SetChassisVehicleCollide(false);

    // Monitor internal contacts for the chassis, left sprocket, left idler, and first shoe on the left track.
    ////marder.GetVehicle().MonitorContacts(TrackedCollisionFlag::CHASSIS | TrackedCollisionFlag::SPROCKET_LEFT |
    ////                        TrackedCollisionFlag::SHOES_LEFT | TrackedCollisionFlag::IDLER_LEFT);

    // Monitor only contacts involving the chassis.
    marder.GetVehicle().MonitorContacts(TrackedCollisionFlag::CHASSIS);

    // Collect contact information.
    // If enabled, number of contacts and local contact point locations are collected for all
    // monitored parts.  Data can be written to a file by invoking ChTrackedVehicle::WriteContacts().
    ////marder.GetVehicle().SetContactCollection(true);

    // under belly points to estimate vehicle/ground interference
    std::vector<ChVector<double> > bellyPts;
    bellyPts.push_back(ChVector<>(0.2332, 0, 0));
    bellyPts.push_back(ChVector<double>(-0.1043, 0, -0.3759));
    bellyPts.push_back(ChVector<double>(-1.45045, 0, -0.3759));
    bellyPts.push_back(ChVector<double>(-2.7966, 0, -0.3759));
    bellyPts.push_back(ChVector<double>(-4.14275, 0, -0.3759));
    bellyPts.push_back(ChVector<double>(-5.4889, 0, -0.3759));
    bellyPts.push_back(ChVector<double>(-5.9805, 0, 0.3655));
    std::vector<ChFunction_Recorder> clearance;
    clearance.resize(bellyPts.size());

    // ------------------
    // Create the terrain
    // ------------------

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.7f;
    minfo.Y = 2e7f;

    // Create the ground
    double base_height = 0.0;
    float friction_coef = 0.8f;
    double aa = 170.0;
    double obl = 5.0;
    double obh = 1.0;
    ObsModTerrain terrain(marder.GetSystem(), base_height, friction_coef, aa, obl, obh);
    auto terrain_mat = minfo.CreateMaterial(contact_method);
    terrain.EnableCollisionMesh(terrain_mat, std::abs(initLoc.x()) + 5, 0.03);
    terrain.Initialize(ObsModTerrain::VisualisationType::MESH);
    xpos_max = terrain.GetXObstacleEnd() + 7.0;

    // Create the driver
    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(marder.GetVehicle(), vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", 0.0);
    driver.Initialize();

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    auto vis = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Marder Vehicle Ride");
    vis->SetChaseCamera(trackPoint, 10.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&marder.GetVehicle());

    // -----------------
    // Initialize output
    // -----------------

    std::vector<std::string> dirs_to_create = {out_top_dir, out_dir};
    if (img_output) {
      dirs_to_create.push_back(img_dir);
    }

    for (const auto& dir : dirs_to_create) {
      if (!filesystem::create_directory(filesystem::path(dir))) {
          std::cout << "Error creating directory " << dir << std::endl;
          return 1;
      }
    }

    // Set up vehicle output
    marder.GetVehicle().SetChassisOutput(true);
    marder.GetVehicle().SetTrackAssemblyOutput(VehicleSide::LEFT, true);
    marder.GetVehicle().SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    marder.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

    // Cannot use HHT + MKL with NSC contact
    if (contact_method == ChContactMethod::NSC) {
        use_mkl = false;
    }

#ifndef CHRONO_PARDISO_MKL
    // Cannot use HHT + PardisoMKL if Chrono::PardisoMKL not available
    use_mkl = false;
#endif

    if (use_mkl) {
#ifdef CHRONO_PARDISO_MKL
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        marder.GetSystem()->SetSolver(mkl_solver);

        marder.GetSystem()->SetTimestepperType(ChTimestepper::Type::HHT);
        auto integrator = std::static_pointer_cast<ChTimestepperHHT>(marder.GetSystem()->GetTimestepper());
        integrator->SetAlpha(-0.2);
        integrator->SetMaxiters(50);
        integrator->SetAbsTolerances(1e-4, 1e2);
        integrator->SetMode(ChTimestepperHHT::ACCELERATION);
        integrator->SetStepControl(false);
        integrator->SetModifiedNewton(false);
        integrator->SetScaling(true);
        integrator->SetVerbose(true);
#endif
    } else {
        auto solver = chrono_types::make_shared<ChSolverBB>();
        solver->SetMaxIterations(120);
        solver->SetOmega(0.8);
        solver->SetSharpnessLambda(1.0);
        marder.GetSystem()->SetSolver(solver);

        marder.GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
        marder.GetSystem()->SetMinBounceSpeed(2.0);
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(marder.GetVehicle().GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(marder.GetVehicle().GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(marder.GetVehicle().GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(marder.GetVehicle().GetNumTrackShoes(RIGHT));

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    std::ofstream kurs(out_dir + "/path.txt");

    ChTimer timer;
    timer.reset();
    timer.start();
    double sim_time = 0;
    double bail_out_time = 30.0;

    ChRunningAverage avg(100);                    // filter angine torque
    std::vector<double> engineForce;              // store obstacle related tractive force
    double effRadius = 0.328414781 + 0.06 / 2.0;  // sprocket pitch radius + track shoe thickness / 2
    double gear_ratio = 0.05;

    while (vis->Run()) {
        // Debugging output
        if (dbg_output) {
            auto track_L = marder.GetVehicle().GetTrackAssembly(LEFT);
            auto track_R = marder.GetVehicle().GetTrackAssembly(RIGHT);
            cout << "Time: " << marder.GetSystem()->GetChTime() << endl;
            cout << "      Num. contacts: " << marder.GetSystem()->GetNcontacts() << endl;
            const ChFrameMoving<>& c_ref = marder.GetChassisBody()->GetFrame_REF_to_abs();
            const ChVector<>& c_pos = marder.GetVehicle().GetPos();
            cout << "      chassis:    " << c_pos.x() << "  " << c_pos.y() << "  " << c_pos.z() << endl;
            {
                const ChVector<>& i_pos_abs = track_L->GetIdler()->GetWheelBody()->GetPos();
                const ChVector<>& s_pos_abs = track_L->GetSprocket()->GetGearBody()->GetPos();
                ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      L idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      L sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            {
                const ChVector<>& i_pos_abs = track_R->GetIdler()->GetWheelBody()->GetPos();
                const ChVector<>& s_pos_abs = track_R->GetSprocket()->GetGearBody()->GetPos();
                ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      R idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      R sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            cout << "      L suspensions (arm angles):";
            for (size_t i = 0; i < track_L->GetNumTrackSuspensions(); i++) {
                cout << " " << track_L->GetTrackSuspension(i)->GetCarrierAngle();
            }
            cout << endl;
            cout << "      R suspensions (arm angles):";
            for (size_t i = 0; i < track_R->GetNumTrackSuspensions(); i++) {
                cout << " " << track_R->GetTrackSuspension(i)->GetCarrierAngle();
            }
            cout << endl;
        }

        if (step_number % render_steps == 0) {
            // Render scene
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(marder.GetSystem(), filename);
            }
            if (img_output && step_number > 200) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
                vis->WriteImageToFile(filename);
            }
            render_frame++;
        }

        double time = marder.GetVehicle().GetChTime();
        sim_time = time;
        double speed = marder.GetVehicle().GetSpeed();
        double xpos = marder.GetVehicle().GetPos().x();
        double yerr = marder.GetVehicle().GetPos().y();
        kurs << time << "\t" << xpos << "\t" << yerr << "\t" << speed << "\t" << std::endl;
        if (xpos >= -1.0 && xpos <= xpos_max) {
            double eTorque = avg.Add(std::static_pointer_cast<ChEngineSimpleMap>(marder.GetVehicle().GetEngine())
                                         ->GetOutputMotorshaftTorque());
            engineForce.push_back(eTorque * effRadius / gear_ratio);
            for (size_t i = 0; i < bellyPts.size(); i++) {
                ChVector<> p = marder.GetVehicle().GetPointLocation(bellyPts[i]);
                double t = terrain.GetHeight(ChVector<>(p.x(), p.y(), 0));
                clearance[i].AddPoint(xpos, p.z() - t);
            }
        }
        if (xpos > xpos_max) {
            break;
        }
        if (time > bail_out_time) {
            break;
        }
        driver.SetDesiredSpeed(ChSineStep(time, 1.0, 0.0, 2.0, target_speed));
        // Collect output data from modules
        DriverInputs driver_inputs = driver.GetInputs();
        marder.GetVehicle().GetTrackShoeStates(LEFT, shoe_states_left);
        marder.GetVehicle().GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)

        driver.Synchronize(time);
        terrain.Synchronize(time);
        marder.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        marder.Advance(step_size);
        vis->Advance(step_size);

        // Report if the chassis experienced a collision
        if (marder.GetVehicle().IsPartInContact(TrackedCollisionFlag::CHASSIS)) {
            std::cout << time << "  chassis contact" << std::endl;
        }

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        // realtime_timer.Spin(step_size);
    }

    timer.stop();
    kurs.close();
    marder.GetVehicle().WriteContacts("Marder_contacts.out");

    double clearMin = 99.0;
    for (size_t i = 0; i < clearance.size(); i++) {
        double x1, x2;
        double vmin, vmax;
        clearance[i].Estimate_x_range(x1, x2);
        clearance[i].Estimate_y_range(x1, x2, vmin, vmax, 0);
        GetLog() << "Clearance#" << i << " = " << vmin << "\n";
        if (vmin < clearMin) {
            clearMin = vmin;
        }
    }

    double wallclock_time = timer.GetTimeSeconds();
    GetLog() << "Model time      = " << sim_time << " s\n";
    GetLog() << "Wall clock time = " << wallclock_time << " s\n";

    double fMax = 0.0;
    double fMean = 0.0;
    for (size_t i = 0; i < engineForce.size(); i++) {
        if (engineForce[i] > fMax)
            fMax = engineForce[i];
        fMean += engineForce[i];
    }
    fMean /= double(engineForce.size());
    GetLog() << "Average Tractive Force = " << fMean << " N\n";
    GetLog() << "Max Tractive Force     = " << fMax << " N\n";
    GetLog() << "Min. Clearance         = " << clearMin << " m\n";

    return 0;
}
