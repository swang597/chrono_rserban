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

#ifndef ROBO_SIMIAN_H
#define ROBO_SIMIAN_H

#include <string>
#include <vector>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChSystem.h"

namespace robosimian {

enum LimbID { FRONT_LEFT = 0, FRONT_RIGHT = 1, REAR_LEFT = 2, REAR_RIGHT = 3 };

enum class VisualizationType {
    NONE,        ///< no visualization
    PRIMITIVES,  ///< use primitve shapes
    MESH         ///< use meshes
};

class Part {
  public:
    Part(const std::string& name);
    virtual ~Part() {}

    const std::string& GetName() const { return m_name; }
    void SetName(const std::string& name) { m_name = name; }
    void SetVisualizationType(VisualizationType vis);

    virtual void AddVisualizationAssets(VisualizationType vis) {}
    virtual void RemoveVisualizationAssets() {}

  protected:
    struct BoxShape {
        BoxShape(const chrono::ChVector<>& pos, const chrono::ChQuaternion<>& rot, const chrono::ChVector<>& dims)
            : m_pos(pos), m_rot(rot), m_dims(dims) {}
        chrono::ChVector<> m_pos;
        chrono::ChQuaternion<> m_rot;
        chrono::ChVector<> m_dims;
    };

    struct SphereShape {
        SphereShape(const chrono::ChVector<>& pos, double radius) : m_pos(pos), m_radius(radius) {}
        chrono::ChVector<> m_pos;
        double m_radius;
    };

    struct CylinderShape {
        CylinderShape(const chrono::ChVector<>& pos, const chrono::ChQuaternion<>& rot, double radius, double length)
            : m_pos(pos), m_rot(rot), m_radius(radius), m_length(length) {}
        chrono::ChVector<> m_pos;
        chrono::ChQuaternion<> m_rot;
        double m_radius;
        double m_length;
    };

    std::string m_name;                      ///< subsystem name
    std::vector<BoxShape> m_boxes;           ///< set of primitive boxes (collision and visualization)
    std::vector<SphereShape> m_spheres;      ///< set of primitive spheres (collision and visualization)
    std::vector<CylinderShape> m_cylinders;  ///< set of primitive cylinders (collision and visualization)
};

class Chassis : public Part {
  public:
    Chassis(const std::string& name, chrono::ChSystem* system, bool fixed);
    ~Chassis() {}

    void Initialize(const chrono::ChCoordsys<>& pos);

    void SetCollide(bool val);

  private:
    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    std::shared_ptr<chrono::ChBodyAuxRef> m_body;

    static const double m_mass;                    ///< chassis mass
    static const chrono::ChVector<> m_COM;         ///< location of the center of mass in the body frame.
    static const chrono::ChVector<> m_inertia_xx;  ///< moments of inertia (w.r.t. centroidal frame)
    static const chrono::ChVector<> m_inertia_xy;  ///< products of inertia (w.r.t. centroidal frame)

    static const std::string m_vis_mesh_name;  ///< chassis visualization mesh name
};

////class Link : public Part {
////public:
////    Link(const std::string& name, chrono::ChSystem* system);
////private:
////};

class Limb : public Part {
  public:
    Limb(const std::string& name, chrono::ChSystem* system);
    ~Limb() {}

    void Initialize();

  private:
    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    chrono::ChColor GetColor(size_t index);

    static const std::string m_vis_mesh_names[11];  ///< link visualization mesh names
};

class RoboSimian {
  public:
    RoboSimian(chrono::ChMaterialSurface::ContactMethod contact_method, bool fixed = false);
    RoboSimian(chrono::ChSystem* system, bool fixed = false);
    ~RoboSimian();

    void Initialize(const chrono::ChCoordsys<>& pos);

    void SetVisualizationTypeChassis(VisualizationType vis);
    void SetVisualizationTypeLimb(LimbID which, VisualizationType vis);

  private:
    void Create(bool fixed);

    chrono::ChSystem* m_system;  ///< pointer to the Chrono system
    bool m_owns_system;          ///< true if system created at construction

    std::shared_ptr<Chassis> m_chassis;          ///< robot chassis
    std::vector<std::shared_ptr<Limb>> m_limbs;  ///< robot limbs
};

}  // end namespace robosimian

#endif
