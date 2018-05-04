#include <cmath>
#include <cstdio>
#include <vector>

#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrTools.h"

#include "robosimian.h"

using namespace chrono;
using namespace chrono::collision;

////double time_step = 1e-3;
double time_step = 5e-4;

// Time interval between two render frames
double render_step_size = 1.0 / 30;  // FPS = 50

// Time interval for assuming initial pose
double time_offset = 3;

// Output directories
const std::string out_dir = GetChronoOutputPath() + "ROBOSIMIAN";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string img_dir = out_dir + "/IMG";

// POV-Ray amd/or IMG output
bool povray_output = false;
bool image_output = false;

// =============================================================================

class EventReceiver : public irr::IEventReceiver {
  public:
    EventReceiver(robosimian::RoboSimian& robot, irrlicht::ChIrrApp& app)
        : m_robot(robot), m_app(app), m_vis(robosimian::VisualizationType::COLLISION) {}

    virtual bool OnEvent(const irr::SEvent& event) {
        if (event.EventType != irr::EET_KEY_INPUT_EVENT)
            return false;
        if (!event.KeyInput.PressedDown) {
            switch (event.KeyInput.Key) {
                case irr::KEY_KEY_C:
                    m_vis = (m_vis == robosimian::VisualizationType::MESH ? robosimian::VisualizationType::COLLISION
                                                                          : robosimian::VisualizationType::MESH);

                    m_robot.SetVisualizationTypeChassis(m_vis);
                    m_robot.SetVisualizationTypeSled(m_vis);
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

// =============================================================================

class RayCaster {
  public:
    RayCaster(ChSystem* sys, const ChFrame<>& origin, const ChVector2<>& dims, double spacing);

    const std::vector<ChVector<>>& GetPoints() const { return m_points; }

    void Update();

  private:
    ChSystem* m_sys;
    ChFrame<> m_origin;
    ChVector2<> m_dims;
    double m_spacing;
    std::shared_ptr<ChBody> m_body;
    std::shared_ptr<ChGlyphs> m_glyphs;
    std::vector<ChVector<>> m_points;
};

RayCaster::RayCaster(ChSystem* sys, const ChFrame<>& origin, const ChVector2<>& dims, double spacing)
    : m_sys(sys), m_origin(origin), m_dims(dims), m_spacing(spacing) {
    m_body = std::shared_ptr<ChBody>(sys->NewBody());
    m_body->SetBodyFixed(true);
    m_body->SetCollide(false);
    sys->AddBody(m_body);

    m_glyphs = std::make_shared<ChGlyphs>();
    m_glyphs->SetGlyphsSize(0.004);
    m_glyphs->SetZbufferHide(true);
    m_glyphs->SetDrawMode(ChGlyphs::GLYPH_POINT);
    m_body->AddAsset(m_glyphs);
}

void RayCaster::Update() {
    m_points.clear();

    ChVector<> dir = m_origin.GetA().Get_A_Zaxis();
    int nx = static_cast<int>(std::round(m_dims.x() / m_spacing));
    int ny = static_cast<int>(std::round(m_dims.y() / m_spacing));
    for (int ix = 0; ix < nx; ix++) {
        for (int iy = 0; iy < ny; iy++) {
            double x_local = -0.5 * m_dims.x() + ix * m_spacing;
            double y_local = -0.5 * m_dims.y() + iy * m_spacing;
            ChVector<> from = m_origin.TransformPointLocalToParent(ChVector<>(x_local, y_local, 0.0));
            ChVector<> to = from + dir * 100;
            collision::ChCollisionSystem::ChRayhitResult result;
            m_sys->GetCollisionSystem()->RayHit(from, to, result);
            if (result.hit)
                m_points.push_back(result.abs_hitPoint);
        }
    }

    m_glyphs->Reserve(0);
    for (unsigned int id = 0; id < m_points.size(); id++) {
        m_glyphs->SetGlyphPoint(id, m_points[id], ChColor(1, 1, 0));
    }
}

// =============================================================================

void CreateCamera(irrlicht::ChIrrApp& application,
                  const irr::core::vector3df& position,
                  const irr::core::vector3df& target) {
    irrlicht::RTSCamera* camera =
        new irrlicht::RTSCamera(application.GetDevice(), application.GetSceneManager()->getRootSceneNode(),
                                application.GetSceneManager(), -1, -160.0f, 1.0f, 0.003f);

    camera->setPosition(position);
    camera->setTarget(target);
    camera->setUpVector(irr::core::vector3df(0, 0, 1));
    camera->setNearValue(0.1f);
    camera->setMinZoom(0.6f);
}

// =============================================================================

std::shared_ptr<ChBody> CreateTerrain(ChSystem& sys, const ChVector<>& hdim, const ChVector<>& loc) {
    auto ground = std::shared_ptr<ChBody>(sys.NewBody());
    ground->SetBodyFixed(true);
    ground->SetCollide(true);

    ground->GetCollisionModel()->ClearModel();
    ground->GetCollisionModel()->AddBox(hdim.x(), hdim.y(), hdim.z(), loc);
    ground->GetCollisionModel()->BuildModel();

    auto box = std::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = hdim;
    box->GetBoxGeometry().Pos = loc;
    ground->AddAsset(box);

    sys.AddBody(ground);

    return ground;
}

// =============================================================================

int main(int argc, char* argv[]) {
    // Create system

    ////ChSystemSMC my_sys;
    ChSystemNSC my_sys;

    my_sys.SetMaxItersSolverSpeed(200);
    if (my_sys.GetContactMethod() == ChMaterialSurface::NSC)
        my_sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));
    ////my_sys.Set_G_acc(ChVector<double>(0, 0, 0));

    // Create RoboSimian robot

    robosimian::RoboSimian robot(&my_sys, true, true);

    // Control collisions (default: true for sled and wheels only)

    ////robot.SetCollide(robosimian::CollisionFlags::NONE);
    ////robot.SetCollide(robosimian::CollisionFlags::ALL);
    ////robot.SetCollide(robosimian::CollisionFlags::LIMBS);
    ////robot.SetCollide(robosimian::CollisionFlags::CHASSIS | robosimian::CollisionFlags::WHEELS);

    // Set visualization modes (default: all COLLISION)

    ////robot.SetVisualizationTypeChassis(robosimian::VisualizationType::MESH);
    ////robot.SetVisualizationTypeLimb(robosimian::FL, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimb(robosimian::FR, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimb(robosimian::RL, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimb(robosimian::RR, robosimian::VisualizationType::COLLISION);
    ////robot.SetVisualizationTypeLimbs(robosimian::VisualizationType::NONE);
    ////robot.SetVisualizationTypeChassis(robosimian::VisualizationType::MESH);
    ////robot.SetVisualizationTypeSled(robosimian::VisualizationType::MESH);
    ////robot.SetVisualizationTypeLimbs(robosimian::VisualizationType::MESH);

    // Initialize Robosimian robot

    ////robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI)));

    // Create a driver and attach to robot

    ////auto driver = std::make_shared<robosimian::DriverFiles>(
    ////    "",                                                 // start input file
    ////    GetChronoDataFile("robosimian/actuation/walking_cycle.txt"),  // cycle input file
    ////    "",                                                 // stop input file
    ////    true);
    ////auto driver = std::make_shared<robosimian::DriverFiles>(
    ////    GetChronoDataFile("robosimian/actuation/sculling_start.txt"),  // start input file
    ////    GetChronoDataFile("robosimian/actuation/sculling_cycle.txt"),  // cycle input file
    ////    GetChronoDataFile("robosimian/actuation/sculling_stop.txt"),   // stop input file
    ////    true);
    auto driver = std::make_shared<robosimian::DriverFiles>(
        GetChronoDataFile("robosimian/actuation/inchworming_start.txt"),  // start input file
        GetChronoDataFile("robosimian/actuation/inchworming_cycle.txt"),  // cycle input file
        GetChronoDataFile("robosimian/actuation/inchworming_stop.txt"),   // stop input file
        true);

    driver->SetOffset(time_offset);
    robot.SetDriver(driver);

    // Cast rays into collision models

    ////RayCaster caster(&my_sys, ChFrame<>(ChVector<>(2, 0, -1), Q_from_AngY(-CH_C_PI_2)), ChVector2<>(2.5, 2.5), 0.02);
    RayCaster caster(&my_sys, ChFrame<>(ChVector<>(0, -2, -1), Q_from_AngX(-CH_C_PI_2)), ChVector2<>(2.5, 2.5), 0.02);

    // Create the visualization window

    irrlicht::ChIrrApp application(&my_sys, L"RoboSimian", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    irrlicht::ChIrrWizard::add_typical_Logo(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Lights(application.GetDevice(), irr::core::vector3df(100.f, 100.f, 100.f),
                                              irr::core::vector3df(100.f, -100.f, 80.f));
    irrlicht::ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(1, -2.5f, 0.2f),
                                              irr::core::vector3df(1, 0, 0));
    ////CreateCamera(application, irr::core::vector3df(0, 2.5f, 0), irr::core::vector3df(0, 0, 0));

    application.SetUserEventReceiver(new EventReceiver(robot, application));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Initialize output directories

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }
    if (image_output) {
        if (ChFileutils::MakeDirectory(img_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // Rigid terrain parameters
    double length = 8;
    double width = 2;
    ChCoordsys<> gridCsys;
    int gridNu;
    int gridNv;
    bool renderGrid = false;

    // Run simulation for specified time
    int render_steps = (int)std::ceil(render_step_size / time_step);
    int sim_frame = 0;
    int render_frame = 0;

    bool released = false;

    while (application.GetDevice()->run()) {
        ////caster.Update();

        if (!released && my_sys.GetChTime() > time_offset / 2) {
            // Set terrain height
            double z = robot.GetWheelPos(robosimian::FR).z() - 0.15;

            // Create terrain
            ChVector<> hdim(length / 2, width / 2, 0.1);
            ChVector<> loc(length / 4, 0, z - 0.1);
            auto ground = CreateTerrain(my_sys, hdim, loc);
            application.AssetBind(ground);
            application.AssetUpdate(ground);

            // Coordinate system for grid
            gridCsys = ChCoordsys<>(ChVector<>(length / 4, 0, z + 0.01), chrono::Q_from_AngAxis(-CH_C_PI_2, VECT_Z));
            gridNu = static_cast<int>(width / 0.1);
            gridNv = static_cast<int>(length / 0.1);

            // Release robot
            robot.GetChassis()->GetBody()->SetBodyFixed(false);
            released = true;
            renderGrid = true;
        }

        application.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        application.DrawAll();

        if (renderGrid) {
            irrlicht::ChIrrTools::drawGrid(application.GetVideoDriver(), 0.1, 0.1, gridNu, gridNv, gridCsys, irr::video::SColor(255, 255, 130, 80), true);
        }

        // Output POV-Ray date and/or snapshot images
        if (sim_frame % render_steps == 0) {
            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(&my_sys, filename);
            }
            if (image_output) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.png", img_dir.c_str(), render_frame + 1);
                irr::video::IImage* image = application.GetVideoDriver()->createScreenShot();
                if (image) {
                    application.GetVideoDriver()->writeImageToFile(image, filename);
                    image->drop();
                }
            }

            render_frame++;
        }

        ////double time = my_sys.GetChTime();
        ////double A = CH_C_PI / 6;
        ////double freq = 2;
        ////double val = 0.5 * A * (1 - std::cos(CH_C_2PI * freq * time));
        ////robot.Activate(robosimian::FR, "joint2", time, val);
        ////robot.Activate(robosimian::RL, "joint5", time, val);

        robot.DoStepDynamics(time_step);

        ////if (my_sys.GetNcontacts() > 0) {
        ////    robot.ReportContacts();
        ////}

        sim_frame++;

        application.EndScene();
    }

    return 0;
}
