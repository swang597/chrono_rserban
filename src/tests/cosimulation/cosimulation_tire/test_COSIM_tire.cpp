#include <omp.h>
#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"

#include "chrono_fea/ChLoadContactSurfaceMesh.h"

#include "chrono_mkl/ChSolverMKL.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrTools.h"

#include "chrono_opengl/ChOpenGLWindow.h"

using namespace chrono;

using std::cout;
using std::endl;

class TireNode {
public:
    TireNode();
    ~TireNode();
    bool Advance(double step);
    const ChVector<>& GetRimPos() const { return m_rim->GetPos(); }
    double GetTireMass() const { return m_tire->GetTireMass(); }
    double GetWheelMass() const { return m_rim->GetMass() + m_tire->GetTireMass(); }
private:
    ChSystemSMC * m_sys;
    irrlicht::ChIrrApp* m_app;
    std::shared_ptr<ChBody> m_rim;
    std::shared_ptr<vehicle::ChANCFTire> m_tire;
    std::shared_ptr<fea::ChLoadContactSurfaceMesh> m_contact_load;

};

TireNode::TireNode() {
    // Create system
    m_sys = new ChSystemSMC();
    m_sys->Set_G_acc(ChVector<double>(0, 0, -9.8));


    // Solver settings
    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    mkl_solver->SetSparsityPatternLock(true);
    m_sys->SetSolver(mkl_solver);

    // Integrator settings
    m_sys->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(m_sys->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(50);
    integrator->SetAbsTolerances(5e-05, 1.8e00);
    integrator->SetMode(ChTimestepperHHT::POSITION);
    integrator->SetScaling(true);
    integrator->SetVerbose(true);
    integrator->SetMaxItersSuccess(5);

    // Create the rim body
    m_rim = std::shared_ptr<ChBody>(m_sys->NewBody());
    m_rim->SetMass(45);
    m_rim->SetInertiaXX(ChVector<>(0.113, 0.113, 0.113));
    m_rim->SetBodyFixed(false);
    m_sys->AddBody(m_rim);

    m_rim->SetPos(ChVector<>(0, 0, 0.6));
    m_rim->SetRot(QUNIT);
    m_rim->SetPos_dt(ChVector<>(0, 0, 0));
    m_rim->SetWvel_loc(ChVector<>(0, 0, 0));

    auto sphere = std::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = 0.1;
    m_rim->AddAsset(sphere);

    // Create the tire
    m_tire = std::make_shared<vehicle::ANCFTire>(vehicle::GetDataFile("hmmwv/tire/HMMWV_ANCFTire.json"));
    m_tire->EnablePressure(true);
    m_tire->EnableContact(true);
    m_tire->EnableRimConnection(true);
    m_tire->SetContactSurfaceType(vehicle::ChDeformableTire::TRIANGLE_MESH);

    m_tire->Initialize(m_rim, vehicle::VehicleSide::LEFT);

    m_tire->SetVisualizationType(vehicle::VisualizationType::PRIMITIVES);

    // Create a mesh load for contact forces and add it to the tire's load container
    auto contact_surface = std::static_pointer_cast<fea::ChContactSurfaceMesh>(m_tire->GetContactSurface());
    m_contact_load = std::make_shared<fea::ChLoadContactSurfaceMesh>(contact_surface);
    m_tire->GetLoadContainer()->Add(m_contact_load);

    // Create Irrlicht visualization app
    m_app = new irrlicht::ChIrrApp(m_sys, L"Tire", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    irrlicht::ChIrrWizard::add_typical_Logo(m_app->GetDevice());
    irrlicht::ChIrrWizard::add_typical_Sky(m_app->GetDevice());
    irrlicht::ChIrrWizard::add_typical_Lights(m_app->GetDevice(), irr::core::vector3df(30.0f, -100.0f, 30.0f),
                                              irr::core::vector3df(30.0f, -80.0f, -30.0f));
    irrlicht::ChIrrWizard::add_typical_Camera(m_app->GetDevice(), irr::core::vector3df(0, -4, 0.6f));

    m_app->AssetBindAll();
    m_app->AssetUpdateAll();

    // Mark completion of system construction
    m_sys->SetupInitial();
}

bool TireNode::Advance(double step) {
    m_sys->DoStepDynamics(step);
    m_app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
    m_app->DrawAll();
    m_app->EndScene();

    return m_app->GetDevice()->run();
}

TireNode::~TireNode() {
    delete m_sys;
}

class TerrainNode {
public:
    TerrainNode();
    ~TerrainNode();

    bool Advance(double step);
private:
    ChSystemSMC * m_sys;
};

TerrainNode::TerrainNode() {
    m_sys = new ChSystemSMC();
    m_sys->Set_G_acc(ChVector<double>(0, 0, -9.8));

    // Create the plate
    auto plate = std::shared_ptr<ChBody>(m_sys->NewBody());

    plate->SetIdentifier(0);
    plate->SetPos(ChVector<>(0, 0, 0));
    plate->SetBodyFixed(true);
    plate->SetCollide(true);

    plate->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(plate.get(), ChVector<>(4, 0.5, 0.1));
    plate->GetCollisionModel()->BuildModel();

    m_sys->AddBody(plate);


    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Terrain", m_sys);
    gl_window.SetCamera(ChVector<>(0, -1, 0.5), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
    gl_window.SetRenderMode(opengl::SOLID);

}

TerrainNode::~TerrainNode() {
    delete m_sys;
}

bool TerrainNode::Advance(double step) {
    m_sys->DoStepDynamics(step);
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    if (gl_window.Active()) {
        gl_window.Render();
        return true;
    }
    else {
        return false;
    }
}

int main(int argc, char* argv[]) {
    double time_step = 1e-4; // 4e-5;
    double time_end = 25;
    double time_start = 1;

    TireNode tire;
    TerrainNode terrain;

    //cout << "Wheel mass: " << tire.GetWheelMass() << endl;

    // Run simulation for specified time
    double time = 0;
    int sim_frame = 0;

    while (time < time_end) {
        auto& rim_pos = tire.GetRimPos();
        cout << "time: " << time << "    " << rim_pos.x() << " " << rim_pos.y() << " " << rim_pos.z() << endl;
        bool tire_run = tire.Advance(time_step);
        bool terrain_run = terrain.Advance(time_step);
       
        if (!tire_run || !terrain_run) {
            break;
        }

        time += time_step;
        sim_frame++;
    }

    return 0;
}
