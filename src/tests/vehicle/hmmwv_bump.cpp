// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demonstration of a steering path-follower PID controller.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================
// Problem parameters

// Contact method type
ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DEM;

// Type of tire model (RIGID, LUGRE, FIALA, or PACEJKA)
TireModelType tire_model = TireModelType::RIGID;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

// Initial vehicle location and orientation
ChVector<> initLoc(5, 5, 0.5);
ChQuaternion<> initRot = Q_from_AngZ(0);

// Desired vehicle speed (m/s)
double target_speed = 12;

// Bump parameters
double dist_to_bump = 40;
double bump_radius = 2;
double bump_height = 0.2;

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 200.0;  // size in X direction
double terrainWidth = 50.0;   // size in Y direction

// Simulation step size, end time
double step_size = 1e-3;

// Render FPS
double fps = 60;

// Output
bool state_output = true;
int filter_window_size = 40;
const std::string out_dir = "../HMMWV_BUMP";

// =============================================================================

void CreateBezierPath(const ChVector<>& start, const ChVector<>& dir, double length, ChBezierCurve& path) {
    std::vector<ChVector<>> points;
    std::vector<ChVector<>> inCV;
    std::vector<ChVector<>> outCV;

    // Start point
    ChVector<> P1 = start;
    points.push_back(P1);
    inCV.push_back(P1);
    outCV.push_back(P1 + 0.1 * length * dir);

    // End point
    ChVector<> P2 = start + length * dir;
    points.push_back(P2);
    inCV.push_back(P2 - 0.1 * length * dir);
    outCV.push_back(P2);

    // Generate Bezier path
    path.setPoints(points, inCV, outCV);
}

// =============================================================================

int main(int argc, char* argv[]) {
    // ------------------------------
    // Create the vehicle and terrain
    // ------------------------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(contact_method);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
    my_hmmwv.SetDriveType(DrivelineType::RWD);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(chassis_vis_type);
    my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    my_hmmwv.SetSteeringVisualizationType(steering_vis_type);
    my_hmmwv.SetWheelVisualizationType(wheel_vis_type);
    my_hmmwv.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());
    terrain.SetContactFrictionCoefficient(0.9f);
    terrain.SetContactRestitutionCoefficient(0.01f);
    terrain.SetContactMaterialProperties(2e7f, 0.3f);
    terrain.SetColor(ChColor(1, 1, 1));
    terrain.SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), (float)terrainLength, (float)terrainWidth);
    terrain.Initialize(terrainHeight, terrainLength, terrainWidth);

    // ---------------
    // Add bump
    // ---------------

    ChVector<> fwd_dir = initRot.GetXaxis();
    ChVector<> loc = ChVector<>(initLoc.x, initLoc.y, bump_height - bump_radius) + fwd_dir * dist_to_bump;

    std::cout << loc.x << "  " << loc.y << "  " << loc.z << std::endl;

    auto bump = std::shared_ptr<ChBody>(my_hmmwv.GetSystem()->NewBody());
    bump->SetPos(loc);
    bump->SetRot(initRot);
    bump->SetBodyFixed(true);
    bump->SetCollide(true);

    bump->GetCollisionModel()->ClearModel();
    utils::AddCylinderGeometry(bump.get(), bump_radius, 4.0);
    bump->GetCollisionModel()->BuildModel();

    auto color = std::make_shared<ChColorAsset>();
    color->SetColor(ChColor(0.2f, 0.3f, 0.4f));
    bump->AddAsset(color);

    my_hmmwv.GetSystem()->AddBody(bump);

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"Steering Controller Demo",
                        irr::core::dimension2d<irr::u32>(800, 640));

    app.SetHUDLocation(500, 20);
    app.SetSkyBox();
    app.AddTypicalLogo();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 150.f), irr::core::vector3df(30.f, 50.f, 150.f), 250, 250);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    app.SetTimestep(step_size);

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // ------------------------
    // Create the driver system
    // ------------------------

    ChBezierCurve path;
    CreateBezierPath(initLoc, fwd_dir, 2 * dist_to_bump, path);

    ChPathFollowerDriver driver_follower(my_hmmwv.GetVehicle(), &path, "my_path", target_speed);
    driver_follower.GetSteeringController().SetLookAheadDistance(5);
    driver_follower.GetSteeringController().SetGains(0.5, 0, 0);
    driver_follower.GetSpeedController().SetGains(0.4, 0, 0);
    driver_follower.Initialize();

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();

    // -----------------
    // Initialize output
    // -----------------

    if (state_output) {
        if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    utils::ChRunningAverage filter_drv_acc(filter_window_size);
    utils::ChRunningAverage filter_com_acc(filter_window_size);
    utils::ChRunningAverage filter_flw_acc(filter_window_size);

    // ---------------
    // Simulation loop
    // ---------------

    auto chassis = my_hmmwv.GetVehicle().GetChassis();
    auto chassis_body = my_hmmwv.GetVehicle().GetChassisBody();
    auto wheel_body = my_hmmwv.GetVehicle().GetWheelBody(FRONT_LEFT);

    // Driver location in vehicle local frame
    ChVector<> drv_pos_loc = my_hmmwv.GetChassis()->GetLocalDriverCoordsys().pos;

    // Number of simulation steps between miscellaneous events
    double render_step_size = 1 / fps;
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;
    int sim_frame = 0;
    int render_frame = 0;

    while (app.GetDevice()->run()) {
        // Extract system state
        double time = my_hmmwv.GetSystem()->GetChTime();

        ChVector<> drv_pos_abs = my_hmmwv.GetVehicle().GetDriverPos();
        ChVector<> veh_pos_abs = my_hmmwv.GetVehicle().GetVehiclePos();
        ChVector<> com_pos_abs = chassis_body->GetPos();
        ChVector<> flw_pos_abs = wheel_body->GetPos();

        ChVector<> drv_acc_loc = my_hmmwv.GetVehicle().GetVehicleAcceleration(drv_pos_loc);
        ChVector<> drv_acc_abs = chassis_body->GetFrame_REF_to_abs().PointAccelerationLocalToParent(drv_pos_loc);
        ChVector<> com_acc_abs = chassis_body->GetPos_dtdt();
        ChVector<> flw_acc_abs = wheel_body->GetPos_dtdt();

        double vert_drv_acc_abs = filter_drv_acc.Add(drv_acc_abs.z);
        double vert_com_acc_abs = filter_com_acc.Add(com_acc_abs.z);
        double vert_flw_acc_abs = filter_flw_acc.Add(flw_acc_abs.z);

        // Collect output
        if (state_output) {
            csv << time << my_hmmwv.GetVehicle().GetVehicleSpeed();
            csv << drv_pos_loc << drv_pos_abs << com_pos_abs << flw_pos_abs;
            csv << drv_acc_loc << drv_acc_abs << com_acc_abs << flw_acc_abs;
            csv << vert_drv_acc_abs << vert_com_acc_abs << vert_flw_acc_abs;
            csv << std::endl;
        }

        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver_follower.GetThrottle();
        double steering_input = driver_follower.GetSteering();
        double braking_input = driver_follower.GetBraking();

        /*
        // Hack for acceleration-braking maneuver
        static bool braking = false;
        if (my_hmmwv.GetVehicle().GetVehicleSpeed() > target_speed)
            braking = true;
        if (braking) {
            throttle_input = 0;
            braking_input = 1;
        } else {
            throttle_input = 1;
            braking_input = 0;
        }
        */

        // Update sentinel and target location markers for the path-follower controller.
        const ChVector<>& pS = driver_follower.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver_follower.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x, (irr::f32)pS.y, (irr::f32)pS.z));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x, (irr::f32)pT.y, (irr::f32)pT.z));

        // Render scene and output POV-Ray data
        if (sim_frame % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();


            render_frame++;
        }

        // Update modules (process inputs from other modules)
        driver_follower.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize("Test", steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver_follower.Advance(step);
        terrain.Advance(step);
        my_hmmwv.Advance(step);
        app.Advance(step);

        // Increment simulation frame number
        sim_frame++;
    }

    if (state_output)
        csv.write_to_file(out_dir + "/state.out");

    return 0;
}
