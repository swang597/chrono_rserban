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

class EventReceiver : public irr::IEventReceiver {
  public:
    EventReceiver(robosimian::RoboSimian& robot, irrlicht::ChIrrApp& app)
        : m_robot(robot), m_app(app), m_vis(robosimian::VisualizationType::PRIMITIVES) {}

    virtual bool OnEvent(const irr::SEvent& event) {
        if (event.EventType != irr::EET_KEY_INPUT_EVENT)
            return false;
        if (!event.KeyInput.PressedDown) {
            switch (event.KeyInput.Key) {
                case irr::KEY_KEY_C:
                    m_vis = (m_vis == robosimian::VisualizationType::MESH ? robosimian::VisualizationType::PRIMITIVES
                                                                          : robosimian::VisualizationType::MESH);

                    m_robot.SetVisualizationTypeChassis(m_vis);
                    m_robot.SetVisualizationTypeLimbs(m_vis);
                    m_robot.SetVisualizationTypeWheels(m_vis);

                    m_app.AssetBindAll();
                    m_app.AssetUpdateAll();

                    return true;
            }
        }
        return false;
    }

  private:
    robosimian::RoboSimian& m_robot;
    robosimian::VisualizationType m_vis;
    irrlicht::ChIrrApp m_app;
};

int main(int argc, char* argv[]) {
    // Create system
    ////ChSystemSMC my_sys;
    ChSystemNSC my_sys;
    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));
    ////my_sys.Set_G_acc(ChVector<double>(0, 0, 0));

    my_sys.SetMaxItersSolverSpeed(200);
    my_sys.SetMaxItersSolverStab(200);
    my_sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    robosimian::RoboSimian robot(&my_sys, true);
    ////robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI)));

    ////robot.SetVisualizationTypeChassis(robosimian::VisualizationType::PRIMITIVES);
    ////robot.SetVisualizationTypeLimb(robosimian::FL, robosimian::VisualizationType::PRIMITIVES);
    ////robot.SetVisualizationTypeLimb(robosimian::FR, robosimian::VisualizationType::PRIMITIVES);
    ////robot.SetVisualizationTypeLimb(robosimian::RL, robosimian::VisualizationType::PRIMITIVES);
    ////robot.SetVisualizationTypeLimb(robosimian::RR, robosimian::VisualizationType::PRIMITIVES);
    ////robot.SetVisualizationTypeLimbs(robosimian::VisualizationType::NONE);
    ////robot.SetVisualizationTypeWheels(robosimian::VisualizationType::PRIMITIVES);

    // Create the visualization window
    irrlicht::ChIrrApp application(&my_sys, L"RoboSimian", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    irrlicht::ChIrrWizard::add_typical_Logo(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Lights(application.GetDevice(), irr::core::vector3df(100.f, 100.f, 100.f),
                                              irr::core::vector3df(100.f, -100.f, 80.f));
    irrlicht::ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(0, -2, 2));

    application.SetUserEventReceiver(new EventReceiver(robot, application));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Run simulation for specified time
    int sim_frame = 0;

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        application.DrawAll();
        ////irrlicht::ChIrrTools::drawAllCOGs(my_sys, application.GetVideoDriver(), 1);
        application.EndScene();

        double time = my_sys.GetChTime();
        double A = CH_C_PI / 6;
        double freq = 2;
        double val = 0.5 * A * (1 - std::cos(CH_C_2PI * freq * time));
        robot.Activate(robosimian::FR, "joint2", time, val);
        robot.Activate(robosimian::RL, "joint5", time, val);

        my_sys.DoStepDynamics(time_step);
        
        if (my_sys.GetNcontacts() > 0) {
            robot.ReportContacts();
        }

        sim_frame++;
    }

    return 0;
}
