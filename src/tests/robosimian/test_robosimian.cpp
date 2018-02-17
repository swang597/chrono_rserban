#include <omp.h>
#include <cmath>
#include <cstdio>
#include <vector>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrTools.h"

#include "robosimian.h"

using namespace chrono;
using namespace chrono::collision;

double time_step = 1e-3;

int main(int argc, char* argv[]) {
    // Create system
    ////ChSystemSMC my_sys;
    ChSystemNSC my_sys;
    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));

    robosimian::RoboSimian robot(&my_sys, true);
    robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI)));
    robot.SetVisualizationTypeChassis(robosimian::VisualizationType::PRIMITIVES);

    // Create the visualization window
    irrlicht::ChIrrApp application(&my_sys, L"Robosimian", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    irrlicht::ChIrrWizard::add_typical_Logo(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Lights(application.GetDevice(), irr::core::vector3df(30.f, -100.f, 30.f),
                                              irr::core::vector3df(30.f, -80.f, -30.f));
    irrlicht::ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(0, -4, 2));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Run simulation for specified time
    int sim_frame = 0;

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        application.DrawAll();
        irrlicht::ChIrrTools::drawAllCOGs(my_sys, application.GetVideoDriver(), 1);
        application.EndScene();

        my_sys.DoStepDynamics(time_step);
        sim_frame++;
    }

    return 0;
}
