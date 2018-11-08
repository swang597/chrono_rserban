#include <omp.h>
#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrTools.h"

using namespace chrono;
using namespace chrono::collision;

int main(int argc, char* argv[]) {
    // BASIC SETUP
    double time_step = 1e-3;
    double time_end = 25;
    double time_start = 1;

    double out_fps = 50;

    double tolerance = 1e-3;
    unsigned int max_iteration = 100;

    enum ForceType { BODY_FORCE, ACCUMULATOR_FORCE };
    ForceType force_type = BODY_FORCE;

    bool floating = false;

    float friction = 0.5f;

    // Create system
    ////ChSystemSMC my_sys;
    ChSystemNSC my_sys;

    if (floating)
        my_sys.Set_G_acc(ChVector<double>(0, 0, 0));
    else
        my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));

    // Create the plate
    auto plate = std::shared_ptr<ChBody>(my_sys.NewBody());

    plate->SetIdentifier(0);
    plate->SetPos(ChVector<>(0, 0, 0));
    plate->SetBodyFixed(true);
    plate->SetCollide(true);

    plate->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(plate.get(), ChVector<>(4, 0.5, 0.1));
    plate->GetCollisionModel()->BuildModel();

    my_sys.AddBody(plate);

    // Create the ball
    double mass = 1;
    double radius = 0.15;
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

    auto ball = std::shared_ptr<ChBody>(my_sys.NewBody());

    ball->SetIdentifier(2);
    ball->SetMass(mass);
    ball->SetInertiaXX(inertia);
    ball->SetPos(ChVector<>(-3, 0, 1));
    ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ball->SetBodyFixed(false);
    ball->SetCollide(true);

    ball->GetCollisionModel()->ClearModel();
    utils::AddSphereGeometry(ball.get(), radius);
    ball->GetCollisionModel()->BuildModel();

    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("greenwhite.png"));
    ball->AddAsset(texture);

    my_sys.AddBody(ball);

    switch (ball->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            ball->GetMaterialSurfaceNSC()->SetFriction(friction);
            ball->GetMaterialSurfaceNSC()->SetRestitution(0);
            break;
        case ChMaterialSurface::SMC:
            ball->GetMaterialSurfaceSMC()->SetFriction(friction);
            ball->GetMaterialSurfaceSMC()->SetRestitution(0);
            break;
    }

    // Force (ramp to maximum and then back to 0)
    double fullForce = 1;
    ChFunction_Recorder forceFunct;
    forceFunct.AddPoint(0, 0);
    forceFunct.AddPoint(time_start, 0);
    forceFunct.AddPoint(time_start + 1, fullForce);
    forceFunct.AddPoint(time_start + 2, fullForce);
    forceFunct.AddPoint(time_start + 3, 0);
    forceFunct.AddPoint(time_end, 0);

    if (force_type == BODY_FORCE) {
        auto drawForce = std::make_shared<ChForce>();
        ball->AddForce(drawForce);
        drawForce->SetMode(ChForce::ForceType::FORCE);  // force or torque
        drawForce->SetFrame(ChForce::ReferenceFrame::BODY);
        drawForce->SetAlign(ChForce::AlignmentFrame::WORLD_DIR);
        drawForce->SetVrelpoint(ChVector<>(0, 0, 0));
        drawForce->SetF_x(std::make_shared<ChFunction_Recorder>(forceFunct));
        drawForce->SetF_y(std::make_shared<ChFunction_Const>(0));
        drawForce->SetF_z(std::make_shared<ChFunction_Const>(0));
    }

    // Create the visualization window
    irrlicht::ChIrrApp application(&my_sys, L"Body force test", irr::core::dimension2d<irr::u32>(800, 600), false,
                                   true);
    irrlicht::ChIrrWizard::add_typical_Logo(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Lights(application.GetDevice(), irr::core::vector3df(30.f, -100.f, 30.f),
                                              irr::core::vector3df(30.f, -80.f, -30.f));
    irrlicht::ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(0, -4, 2));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Run simulation for specified time
    int out_steps = static_cast<int>(std::ceil((1 / time_step) / out_fps));
    int sim_frame = 0;

    while (my_sys.GetChTime() < time_end) {
        if (force_type == ACCUMULATOR_FORCE) {
            double Fx = forceFunct.Get_y(my_sys.GetChTime());
            ball->Empty_forces_accumulators();
            ball->Accumulate_force(ChVector<>(Fx, 0, 0), ball->GetPos(), false);
        }
        my_sys.DoStepDynamics(time_step);
        sim_frame++;

        std::cout << "Vx: " << ball->GetPos_dt().x() << std::endl;

        if (application.GetDevice()->run()) {
            application.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            application.DrawAll();
            irrlicht::ChIrrTools::drawAllCOGs(my_sys, application.GetVideoDriver(), 1);
            application.EndScene();
        } else {
            return 1;
        }
    }

    return 0;
}
