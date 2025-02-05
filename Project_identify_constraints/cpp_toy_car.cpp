#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
// #include "chrono/physics/ChLinkLockRevolute.h"
// #include "chrono/physics/ChLinkLockPrismatic.h"
// #include "chrono/irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_vehicle/terrain/SCMTerrain.h" //Shu added
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include <fstream>
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Chrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono physical system
    ChSystemNSC sys;

    // // Create the ground
    // auto ground = chrono_types::make_shared<ChBodyEasyBox>(10, 1, 10, 1000, true, false);
    // ground->SetPos(ChVector<>(0, -1, 0));
    // ground->SetBodyFixed(true);
    // sys.AddBody(ground);

    // Create the SCM terrain
    auto terrain = chrono_types::make_shared<vehicle::SCMTerrain>(&sys);
    // SCMDeformableTerrain terrain(&sys);
    // terrain->SetPlane(ChCoordsys<>(ChVector<>(0, 0, 0)));
    // terrain->SetPlane(ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    terrain->SetPlane(ChCoordsys<>(ChVector<>(0, -5, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
    
    terrain->SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.05);
    terrain->Initialize(10, 10, 0.1); // Length, width, mesh resolution
    // terrain->AddMovingPatch(m_chassis_body, ChVector<>(0, 0, 0),
    //                         ChVector<>(2 * m_tire->GetRadius(), 1.0, 2 * m_tire->GetRadius()));

    // Set soil parameters
    terrain->SetSoilParameters(2e6,  // Bekker Kphi
                              0,    // Bekker Kc
                              1.1,  // Bekker n exponent
                              0,    // Mohr cohesive limit (Pa)
                              30,   // Mohr friction limit (degrees)
                              0.01, // Janosi shear coefficient (m)
                              4e7,  // Elastic stiffness (Pa/m), before plastic yield
                              3e4   // Damping (Pa s/m)
    );
    // terrain.SetBulldozingFlow(true);   // Enable bulldozing
    // terrain.SetBulldozingParameters(55,  // Angle of friction for erosion of displaced material at the border of the rut
    //                                 1,    // For bulldozing model, the number of erosion refinements per timestep
    //                                 5,    // Amount of displaced material left along the border: 0 = none, 1 = all
    //                                 0.3   // Fraction in [0,1] of displaced material that goes downward, rest goes to sides
    // );
    
    // Create the car body
    auto car_body = chrono_types::make_shared<ChBodyEasyBox>(2, 1, 0.5, 1000, true, false);
    car_body->SetPos(ChVector<>(0, 0, 0.5));
    sys.AddBody(car_body);


    // // Function to create a wheel
    // auto create_wheel = [&](ChVector<> position) {
    //     auto wheel = chrono_types::make_shared<ChBodyEasyCylinder>(0.3, 0.1, 1000, true, false);
    //     wheel->SetPos(position);
    //     wheel->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_X));
    //     sys.AddBody(wheel);
    //     return wheel;
    // };
    
    // Function to create a wheel
    auto create_wheel = [&](ChVector<> position) {
        auto wheel = chrono_types::make_shared<ChBodyEasyCylinder>(
            geometry::ChAxis::Y, // Cylinder along Y axis
            0.3, // radius
            0.1, // height
            1000, // density
            true, // visualize
            true // collide
        );
        wheel->SetPos(position);
        sys.AddBody(wheel);
        return wheel;
    };
    
    // Create the four wheels
    auto front_left_wheel = create_wheel(ChVector<>(0.8, 0.5, 0.2));
    auto front_right_wheel = create_wheel(ChVector<>(0.8, -0.5, 0.2));
    auto rear_left_wheel = create_wheel(ChVector<>(-0.8, 0.5, 0.2));
    auto rear_right_wheel = create_wheel(ChVector<>(-0.8, -0.5, 0.2));


    // Function to create a revolute joint
    auto create_revolute_joint = [&](std::shared_ptr<ChBody> body, std::shared_ptr<ChBody> wheel, ChVector<> position) {
        auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
        revolute->Initialize(body, wheel, ChCoordsys<>(position, Q_from_AngAxis(CH_C_PI_2, VECT_X)));
        sys.AddLink(revolute);
    };

    // Attach the wheels to the car body
    create_revolute_joint(car_body, front_left_wheel, ChVector<>(0.8, 0.5, 0.2));
    create_revolute_joint(car_body, front_right_wheel, ChVector<>(0.8, -0.5, 0.2));
    create_revolute_joint(car_body, rear_left_wheel, ChVector<>(-0.8, 0.5, 0.2));
    create_revolute_joint(car_body, rear_right_wheel, ChVector<>(-0.8, -0.5, 0.2));

    // Create a motor to drive the rear wheels
    auto motor_left = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor_left->Initialize(car_body, rear_left_wheel, ChFrame<>(ChVector<>(-0.8, 0.5, 0.2), QUNIT));
    sys.AddLink(motor_left);
    auto speed_function_left = chrono_types::make_shared<ChFunction_Const>(CH_C_PI); // Speed w=3.145 rad/sec
    motor_left->SetSpeedFunction(speed_function_left);

    auto motor_right = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor_right->Initialize(car_body, rear_right_wheel, ChFrame<>(ChVector<>(-0.8, -0.5, 0.2), QUNIT));
    sys.AddLink(motor_right);
    auto speed_function_right = chrono_types::make_shared<ChFunction_Const>(CH_C_PI); // Speed w=3.145 rad/sec
    motor_right->SetSpeedFunction(speed_function_right);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Toy Car Simulation");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 0, 5));
    vis->AddTypicalLights();

    // Simulation loop
    ChRealtimeStepTimer realtime_timer;
    double time_step = 0.01;

    while (vis->Run()) {
        // Perform a simulation step
        sys.DoStepDynamics(time_step);

        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->RenderCOGFrames(1.0);
        vis->RenderFrame(ChFrame<>(), 10);
        
        vis->EndScene();

        // Enforce soft real-time
        realtime_timer.Spin(time_step);
    }

    return 0;
}
