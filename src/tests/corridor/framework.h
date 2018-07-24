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
// Author: Radu Serban
// =============================================================================
//
// =============================================================================

#ifndef AV_FRAMEWORK_H
#define AV_FRAMEWORK_H

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "scene.h"
#include "path.h"
#include "vehicle.h"
#include "traffic_light.h"

namespace av {

class Framework {
  public:
    Framework(const Scene& scene, bool render_coll = false);
    ~Framework();

    unsigned int AddPath(std::shared_ptr<chrono::ChBezierCurve> curve, bool closed);
    unsigned int AddPath(const std::vector<GPScoord>& gps_points, double v_offset, bool closed);
    unsigned int AddPath(const std::vector<chrono::ChVector<>>& points, double v_offset, bool closed);

    void SetPathColor(unsigned int id, const chrono::ChColor& color);

    unsigned int AddVehicle(Vehicle::Type type, const chrono::ChCoordsys<>& pos);
    unsigned int AddVehicle(Vehicle::Type type, unsigned int path_id, double target_speed);

    unsigned int AddTrafficLight(const chrono::ChVector<>& pos);
    unsigned int AddTrafficLight(const chrono::ChVector2<>& gps_point);

    void SetEgoVehicle(unsigned int vehicle_id);
    void SetIntegrationStep(double step) { m_step = step; }

    void Run(double time_end, int fps = 30, bool real_time = true);

    chrono::ChSystem* GetSystem() { return m_system; }
    chrono::ChVector<> GetLocation(const GPScoord& gps) const;
    Area GetArea(const GPScoord& gps_min, const GPScoord& gps_max) const;

    void ListAgents();

  private:
    void CreateTerrain();
    void Initialize();
    void Advance();

    Scene m_scene;
    chrono::ChSystem* m_system;
    chrono::vehicle::ChWheeledVehicleIrrApp* m_app;
    chrono::vehicle::RigidTerrain* m_terrain;

    double m_step;
    bool m_initialized;
    bool m_render_coll;

    std::shared_ptr<Vehicle> m_ego_vehicle;

    friend class TruckAV;
    friend class VanAV;
    friend class TrafficLight;
};

}  // end namespace av

#endif
