// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// Demo program for U401 simulation.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
// All units SI.
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_models/vehicle/g-wagon/gd250.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_postprocess/ChGnuPlot.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::gwagon;

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::NONE;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Type of tire model (RIGID, TMEASY, TMSIMPLE, FIALA)
TireModelType tire_model = TireModelType::TMEASY;

// Brake model (SHAFTS only!)
BrakeType brake_model = BrakeType::SIMPLE;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 0.75);

bool use_realtime = true;

// Simulation step sizes
double step_size = 1e-5;
double tire_step_size = step_size;

// Simulation end time
double tend = 15;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "GD250";
const std::string pov_dir = out_dir + "/POVRAY";
bool povray_output = false;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the vehicle, set parameters, and initialize
    GD250 gd250;
    gd250.SetContactMethod(ChContactMethod::NSC);
    gd250.SetChassisFixed(false);
    gd250.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    gd250.SetTireType(tire_model);
    gd250.SetTireStepSize(tire_step_size);
    gd250.SetBrakeType(brake_model);
    gd250.SetKinematicMode(true);
    gd250.SetInitFwdVel(0.0);
    gd250.Initialize();

    gd250.SetChassisVisualizationType(chassis_vis_type);
    gd250.SetSuspensionVisualizationType(suspension_vis_type);
    gd250.SetSteeringVisualizationType(steering_vis_type);
    gd250.SetWheelVisualizationType(wheel_vis_type);
    gd250.SetTireVisualizationType(tire_vis_type);

    std::string tireTypeS;
    switch (tire_model) {
        case TireModelType::TMSIMPLE:
            tireTypeS = "TMsimple";
            break;
        case TireModelType::TMEASY:
            tireTypeS = "TMeasy";
            break;
        case TireModelType::FIALA:
            tireTypeS = "Fiala";
            break;
    }

    std::cout << "Vehicle mass: " << gd250.GetVehicle().GetMass() << std::endl;

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(gd250.GetSystem());

    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);

    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 300, 300);
    patch->SetColor(ChColor(0.8f, 0.8f, 1.0f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 1200, 1200);
    terrain.Initialize();

    // ------------------------------------------------------------------------------
    // Create the vehicle run-time visualization interface and the interactive driver
    // ------------------------------------------------------------------------------

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1

    std::shared_ptr<ChVehicleVisualSystem> vis;
    std::shared_ptr<ChDriver> driver;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("GD250 Demo");
            vis_irr->SetChaseCamera(trackPoint, 6.0, 0.5);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&gd250.GetVehicle());

            // Create the interactive Irrlicht driver system
            auto driver_irr = chrono_types::make_shared<ChInteractiveDriverIRR>(*vis_irr);
            driver_irr->SetSteeringDelta(render_step_size / steering_time);
            driver_irr->SetThrottleDelta(render_step_size / throttle_time);
            driver_irr->SetBrakingDelta(render_step_size / braking_time);
            driver_irr->Initialize();

            vis = vis_irr;
            driver = driver_irr;
#endif
            break;
        }
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("GD250 Demo");
            vis_vsg->AttachVehicle(&gd250.GetVehicle());
            vis_vsg->SetChaseCamera(trackPoint, 6.0, 0.5);
            vis_vsg->SetWindowSize(ChVector2<int>(800, 600));
            vis_vsg->SetWindowPosition(ChVector2<int>(100, 300));
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->Initialize();

            // Create the interactive VSG driver system
            auto driver_vsg = chrono_types::make_shared<ChInteractiveDriverVSG>(*vis_vsg);
            driver_vsg->SetSteeringDelta(render_step_size / steering_time);
            driver_vsg->SetThrottleDelta(render_step_size / throttle_time);
            driver_vsg->SetBrakingDelta(render_step_size / braking_time);
            driver_vsg->Initialize();

            vis = vis_vsg;
            driver = driver_vsg;
#endif
            break;
        }
    }

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    gd250.GetVehicle().LogSubsystemTypes();

    int render_steps = (int)std::ceil(render_step_size / step_size);
    int step_number = 0;
    int render_frame = 0;

    double maxKingpinAngle = 0.0;

    std::string vehModel("U401");
    gd250.GetVehicle().EnableRealtime(use_realtime);
    if (use_realtime) {
        vehModel.append("#RT");
    }
    utils::ChRunningAverage RTF_filter(50);
    ChFunction_Recorder mfunTireOmega, mfunWheelOmega;

    while (vis->Run()) {
        double time = gd250.GetSystem()->GetChTime();
        switch (tire_model) {
            case TireModelType::TMEASY: {
                auto tire = std::static_pointer_cast<ChTMeasyTire>(gd250.GetVehicle().GetTire(1, LEFT));
                mfunTireOmega.AddPoint(time, tire->GetTireOmega());
            } break;
        }
        // Render scene
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (povray_output && step_number % render_steps == 0) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(gd250.GetSystem(), filename);
            }

            render_frame++;
        }

        // Collect output data from modules (for inter-module communication)
        DriverInputs driver_inputs = driver->GetInputs();

        // Update modules (process inputs from other modules)
        driver->Synchronize(time);
        terrain.Synchronize(time);
        gd250.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        terrain.Advance(step_size);
        gd250.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;
    }

    chrono::postprocess::ChGnuPlot mplot1(out_dir + "/tmp_gnuplot_1.gpl");
    vehModel.append("-");
    vehModel.append(tireTypeS);
    // mplot1.OutputPNG(out_dir + "/tire_omega.png", 800, 600);
    mplot1.OutputPNG(out_dir + "/" + vehModel + ".png", 800, 600);
    mplot1.SetGrid();
    mplot1.SetTitle(vehModel);
    mplot1.SetLabelX("Time (s)");
    mplot1.SetLabelY("Tire Omega (rad/s)");
    mplot1.Plot(mfunTireOmega, "from ChFunction_Recorder", " with lines lt -1 lc rgb'#00AAEE' ");

    std::cout << "Maximum Kingpin Angle = " << maxKingpinAngle << " deg" << std::endl;
    return 0;
}
