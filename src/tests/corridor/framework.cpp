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
// =============================================================================

#include "chrono/geometry/ChLineBezier.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "framework.h"
#include "truckAV.h"
#include "vanAV.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace av {

Framework::Framework(const Scene& scene, bool render_coll)
    : m_scene(scene), m_render_coll(render_coll), m_step(1e-3), m_app(nullptr), m_initialized(false) {
    m_system = new ChSystemNSC();
    m_system->Set_G_acc(ChVector<>(0, 0, -9.81));

    m_system->SetMaxItersSolverSpeed(150);
    m_system->SetMaxItersSolverStab(150);
    m_system->SetMaxPenetrationRecoverySpeed(4.0);

    CreateTerrain();
}

Framework::~Framework() {
    delete m_app;
    delete m_terrain;
    delete m_system;
}

ChVector<> Framework::GetLocation(const GPScoord& gps) const {
    auto loc = m_scene.FromGPS(gps);
    auto h = m_terrain->GetHeight(loc.x(), loc.y());
    return ChVector<>(loc.x(), loc.y(), h);
}

Area Framework::GetArea(const GPScoord& gps_min, const GPScoord& gps_max) const {
    auto loc_min = m_scene.FromGPS(gps_min);
    auto h_min = m_terrain->GetHeight(loc_min.x(), loc_min.y());

    auto loc_max = m_scene.FromGPS(gps_max);
    auto h_max = m_terrain->GetHeight(loc_max.x(), loc_max.y());

    return Area(ChVector<>(loc_min.x(), loc_min.y(), h_min), ChVector<>(loc_max.x(), loc_max.y(), h_max));
}

// Note: cannot use std::make_shared because the various constructors are not public.

unsigned int Framework::AddPath(std::shared_ptr<chrono::ChBezierCurve> curve, bool closed) {
    auto id = Agent::GenerateID();
    auto path = std::shared_ptr<Path>(new Path(this, curve, closed));
    path->m_id = id;
    Path::m_paths.insert(std::make_pair(id, path));

    return id;
}

unsigned int Framework::AddPath(const std::vector<GPScoord>& gps_points, double v_offset, bool closed) {
    auto id = Agent::GenerateID();
    auto path = std::shared_ptr<Path>(new Path(this, gps_points, v_offset, closed));
    path->m_id = id;
    Path::m_paths.insert(std::make_pair(id, path));

    return id;
}

unsigned int Framework::AddPath(const std::vector<chrono::ChVector<>>& points, double v_offset, bool closed) {
    auto id = Agent::GenerateID();
    auto path = std::shared_ptr<Path>(new Path(this, points, v_offset, closed));
    path->m_id = id;
    Path::m_paths.insert(std::make_pair(id, path));

    return id;
}

unsigned int Framework::AddVehicle(Vehicle::Type type, const ChCoordsys<>& pos) {
    auto id = Agent::GenerateID();
    std::shared_ptr<Vehicle> vehicle;

    switch (type) {
        case Vehicle::Type::TRUCK:
            vehicle = std::shared_ptr<TruckAV>(new TruckAV(this, pos));
            break;
        case Vehicle::Type::VAN:
            vehicle = std::shared_ptr<VanAV>(new VanAV(this, pos));
            break;
        default:
            std::cout << "Unknown vehicle type" << std::endl;
            return -1;
    }

    vehicle->m_id = id;
    vehicle->m_framework = this;
    Vehicle::m_vehicles.insert(std::make_pair(id, vehicle));

    return id;
}

// Create a vehicle associated with given path
unsigned int Framework::AddVehicle(Vehicle::Type type, unsigned int path_id, double target_speed) {
    auto path = Path::Find(path_id);
    auto p1 = path->m_curve->getPoint(0);
    auto p2 = path->m_curve->getPoint(1);
    auto u = (p2 - p1).GetNormalized();
    auto v = Vcross(ChVector<>(0, 0, 1), u);
    auto w = Vcross(u, v);
    ChMatrix33<> A;
    A.Set_A_axis(u, v, w);

    std::cout << "Add vehicle" << std::endl;
    std::cout << "  " << p1.x() << "  " << p1.y() << "  " << p1.z() << std::endl;

    auto id = AddVehicle(type, ChCoordsys<>(p1 + ChVector<>(0, 0, 0.5), A.Get_A_quaternion()));
    if (id == -1)
        return id;

    Vehicle::Find(id)->SetupDriver(path->m_curve, path->m_closed, target_speed);

    return id;
}

unsigned int Framework::AddTrafficLight(const ChVector<>& pos) {
    auto id = Agent::GenerateID();
    auto light = std::shared_ptr<TrafficLight>(new TrafficLight(this, pos));
    light->m_id = id;
    TrafficLight::m_traffic_lights.insert(std::make_pair(id, light));

    return id;
}

unsigned int Framework::AddTrafficLight(const chrono::ChVector2<>& gps_point) {
    return AddTrafficLight(GetLocation(gps_point));
}

void Framework::SetPathColor(unsigned int id, const ChColor& color) {
    auto path = Path::Find(id);
    if (path)
        path->m_color = color;
}

void Framework::SetEgoVehicle(unsigned int id) {
    m_ego_vehicle = Vehicle::Find(id);
}

void Framework::Run(double time_end, int fps, bool real_time) {
    Initialize();

    double time = 0;
    double next_draw = 0;
    double delta = 1.0 / fps;
    ChTimer<double> timer;

    timer.start();
    while (m_app->GetDevice()->run() && time <= time_end) {
        if (m_system->GetChTime() > next_draw) {
            m_app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            m_app->DrawAll();
            m_app->EndScene();
            next_draw += delta;
        }

        Advance();
        time = m_system->GetChTime();

        if (real_time)
            while (timer.GetTimeSecondsIntermediate() < time) {}
    }
    timer.stop();

    std::cout << "Simulation time: " << time << std::endl;
    std::cout << "Run time:        " << timer() << std::endl;
}

void Framework::CreateTerrain() {
    m_terrain = new RigidTerrain(m_system);

    auto patch = m_terrain->AddPatch(ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT), GetChronoDataFile(m_scene.m_coll_file),
                                     "scene", 0.01, m_render_coll);
    patch->SetContactFrictionCoefficient(0.9f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);

    ChVector<> bbmin, bbmax;
    patch->GetGroundBody()->GetCollisionModel()->GetAABB(bbmin, bbmax);
    std::cout << "Scene Bounding Box" << std::endl;
    std::cout << "  " << bbmin.x() << "  " << bbmin.y() << "  " << bbmin.z() << std::endl;
    std::cout << "  " << bbmax.x() << "  " << bbmax.y() << "  " << bbmax.z() << std::endl;

    if (m_render_coll) {
        auto texture = std::make_shared<ChTexture>();
        texture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
        texture->SetTextureScale(1, 1);
        patch->GetGroundBody()->AddAsset(texture);
    }

    m_terrain->Initialize();
}

void Framework::Initialize() {
    if (m_initialized)
        return;

    // Create visualization assets for all paths
    auto road = std::shared_ptr<ChBody>(m_system->NewBody());
    road->SetBodyFixed(true);
    m_system->AddBody(road);
    for (auto p : Path::GetList()) {
        auto n = static_cast<unsigned int>(p.second->m_curve->getNumPoints());
        auto path_asset = std::make_shared<ChLineShape>();
        path_asset->SetLineGeometry(std::make_shared<geometry::ChLineBezier>(p.second->m_curve));
        path_asset->SetColor(p.second->m_color);
        path_asset->SetName("path_" + std::to_string(p.first));
        path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * n, 400));
        road->AddAsset(path_asset);
    }

    // Create Irrlicht visualization app
    if (!m_ego_vehicle)
        m_ego_vehicle = Vehicle::GetList().begin()->second;

    auto pos = m_ego_vehicle->GetPosition();
    auto pos1 = pos + ChVector<>(30, -30, 100);
    auto pos2 = pos + ChVector<>(30, +30, 100);

    m_app = new ChWheeledVehicleIrrApp(&m_ego_vehicle->GetVehicle(), &m_ego_vehicle->GetPowertrain(), L"Corridor Demo");
    m_app->SetSkyBox();
    m_app->AddTypicalLogo();
    m_app->AddTypicalLights(irr::core::vector3df((irr::f32)pos1.x(), (irr::f32)pos1.y(), (irr::f32)pos1.z()),
                            irr::core::vector3df((irr::f32)pos2.x(), (irr::f32)pos2.y(), (irr::f32)pos2.z()), 250, 130);
    m_app->SetChaseCamera(ChVector<>(0.0, 0.0, .75), 6.0, 0.5);
    m_app->SetTimestep(m_step);

    // Create scene visualization mesh
    if (!m_render_coll) {
        auto smanager = m_app->GetSceneManager();
        auto imesh = smanager->getMesh(GetChronoDataFile(m_scene.m_vis_file).c_str());
        auto node = smanager->addMeshSceneNode(imesh, 0, -1, irr::core::vector3df(0, 0, 0), irr::core::vector3df(0, 0, 0),
            irr::core::vector3df(1, 1, 1));
    }

    // Complete Irrlicht asset construction
    m_app->AssetBindAll();
    m_app->AssetUpdateAll();

    m_initialized = true;
}

void Framework::Advance() {
    double time = m_system->GetChTime();

    m_terrain->Synchronize(time);
    for (auto v : Vehicle::GetList()) {
        v.second->Synchronize(time);
    }
    m_app->Synchronize("", m_ego_vehicle->m_steering, m_ego_vehicle->m_throttle, m_ego_vehicle->m_braking);

    m_terrain->Advance(m_step);
    for (auto v : Vehicle::GetList()) {
        v.second->Advance(m_step);
    }
    m_app->Advance(m_step);

    m_system->DoStepDynamics(m_step);
}

void Framework::ListAgents() {
    std::cout << "\nList of traffic agents" << std::endl;
    std::cout << "  Vehicles" << std::endl;
    for (auto v : Vehicle::GetList()) {
        auto pos = v.second->GetPosition();
        std::cout << "    " << v.first << "  " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
    }

    std::cout << "  Lights" << std::endl;
    for (auto l : TrafficLight::GetList()) {
        auto pos = l.second->GetPosition();
        std::cout << "    " << l.first << "  " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
    }
}

}  // end namespace av
