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
#include <unordered_map>
#include <vector>
#include <array>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

namespace robosimian {

enum LimbID {
    FR = 0,  ///< front right
    RR = 1,  ///< rear right
    RL = 2,  ///< rear left
    FL = 3   ///< front left
};

enum class VisualizationType {
    NONE,       ///< no visualization
    COLLISION,  ///< render primitive collision shapes
    MESH        ///< render meshes
};

namespace CollisionFamily {
enum Enum {
    LIMB_FR = 1,  ///< front-right limb
    LIMB_RR = 2,  ///< rear-right limb
    LIMB_RL = 3,  ///< rear-left limb
    LIMB_FL = 4,  ///< front-left limb
    CHASSIS = 5,  ///< chassis (torso)
    SLED = 6,     ///< sled
    WHEEL_DD = 7  ///< direct-drive wheels
};
}

namespace CollisionFlags {
enum Enum {
    NONE = 0,          ///< no collision shapes on any body
    CHASSIS = 1 << 0,  ///< chassis (torso)
    SLED = 1 << 1,     ///< sled
    LIMBS = 1 << 2,    ///< all limb bodies (Excluding final wheels)
    WHEELS = 1 << 3,   ///< all wheels
    ALL = 0xFFFF       ///< collision enabled on all bodies
};
}

enum class ActuationMode {
    ANGLE,  ///< prescribe time-series for joint angle
    SPEED,  ///< prescribe time-series for joint angular speed
    FIXED   ///< weld joint
};

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

struct MeshShape {
    MeshShape(const chrono::ChVector<>& pos, const chrono::ChQuaternion<>& rot, const std::string& name, bool convex)
        : m_pos(pos), m_rot(rot), m_name(name), m_convex(convex) {}
    chrono::ChVector<> m_pos;
    chrono::ChQuaternion<> m_rot;
    std::string m_name;
    bool m_convex;
};

class Part {
  public:
    Part(const std::string& name, chrono::ChSystem* system);
    virtual ~Part() {}

    const std::string& GetName() const { return m_name; }
    void SetName(const std::string& name) { m_name = name; }
    void SetVisualizationType(VisualizationType vis);

  protected:
    void AddVisualizationAssets(VisualizationType vis);
    void AddCollisionShapes();

    std::string m_name;                            ///< subsystem name
    std::shared_ptr<chrono::ChBodyAuxRef> m_body;  ///< rigid body
    std::vector<BoxShape> m_boxes;                 ///< set of collision boxes
    std::vector<SphereShape> m_spheres;            ///< set of collision spheres
    std::vector<CylinderShape> m_cylinders;        ///< set of collision cylinders
    std::vector<MeshShape> m_meshes;               ///< set of collision meshes
    std::string m_mesh_name;                       ///< visualization mesh name
    chrono::ChVector<> m_offset;                   ///< offset for visualization mesh
    chrono::ChColor m_color;                       ///< visualization asset color

    friend class RoboSimian;
    friend class Limb;
};

class Chassis : public Part {
  public:
    Chassis(const std::string& name, chrono::ChSystem* system, bool fixed);
    ~Chassis() {}

    /// Initialize the chassis at the specified (absolute) position.
    void Initialize(const chrono::ChCoordsys<>& pos);

    /// Enable/disable collision for the sled (Default: false).
    void SetCollide(bool state);

  private:
    bool m_collide;
};

class Sled : public Part {
  public:
    Sled(const std::string& name, chrono::ChSystem* system);
    ~Sled() {}

    /// Initialize the sled at the specified position (relative to the chassis).
    void Initialize(std::shared_ptr<chrono::ChBodyAuxRef> chassis,  ///< chassis body
                    const chrono::ChVector<>& xyz,                  ///< location (relative to chassis)
                    const chrono::ChVector<>& rpy                   ///< roll-pitch-yaw (relative to chassis)
    );

    /// Enable/disable collision for the sled (default: true).
    void SetCollide(bool state);

  private:
    bool m_collide;
};

class WheelDD : public Part {
  public:
    WheelDD(const std::string& name, int id, chrono::ChSystem* system);
    ~WheelDD() {}

    void Initialize(std::shared_ptr<chrono::ChBodyAuxRef> chassis,  ///< chassis body
                    const chrono::ChVector<>& xyz,                  ///< location (relative to chassis)
                    const chrono::ChVector<>& rpy                   ///< roll-pitch-yaw (relative to chassis)
    );
};

class Link {
  public:
    Link(const std::string& mesh_name,
         const chrono::ChVector<>& offset,
         const chrono::ChColor& color,
         double mass,
         const chrono::ChVector<>& com,
         const chrono::ChVector<>& inertia_xx,
         const chrono::ChVector<>& inertia_xy,
         const std::vector<CylinderShape>& shapes)
        : m_mesh_name(mesh_name),
          m_offset(offset),
          m_color(color),
          m_mass(mass),
          m_com(com),
          m_inertia_xx(inertia_xx),
          m_inertia_xy(inertia_xy),
          m_shapes(shapes) {}

  private:
    std::string m_mesh_name;
    chrono::ChVector<> m_offset;
    chrono::ChColor m_color;
    double m_mass;
    chrono::ChVector<> m_com;
    chrono::ChVector<> m_inertia_xx;
    chrono::ChVector<> m_inertia_xy;
    std::vector<CylinderShape> m_shapes;

    friend class Limb;
};

struct LinkData {
    std::string name;
    Link link;
    bool include;
};

struct JointData {
    std::string name;
    std::string linkA;
    std::string linkB;
    ActuationMode mode;
    chrono::ChVector<> xyz;
    chrono::ChVector<> rpy;
    chrono::ChVector<> axis;
};

class Limb {
  public:
    Limb(const std::string& name, LimbID id, const LinkData data[], chrono::ChSystem* system);
    ~Limb() {}

    /// Initialize the link at the specified position (relative to the chassis).
    void Initialize(std::shared_ptr<chrono::ChBodyAuxRef> chassis,  ///< chassis body
                    const chrono::ChVector<>& xyz,                  ///< location (relative to chassis)
                    const chrono::ChVector<>& rpy,                  ///< roll-pitch-yaw (relative to chassis)
                    CollisionFamily::Enum collision_family          ///< collision family
    );

    /// Set visualization type for all limb links.
    void SetVisualizationType(VisualizationType vis);

    /// Enable/disable collision on all links, except final wheel (default: false).
    void SetCollideLinks(bool state);

    /// Enable/disable collision for final wheel (default: true).
    void SetCollideWheel(bool state);

    /// Set activation for given motor at current time.
    void Activate(const std::string& motor_name, double time, double val);

    /// Set activations for all motors at current time.
    void Activate(double time, const std::array<double, 8>& vals);

  private:
    LimbID m_id;
    std::string m_name;
    std::unordered_map<std::string, std::shared_ptr<Part>> m_links;
    std::unordered_map<std::string, std::shared_ptr<chrono::ChLink>> m_joints;
    std::unordered_map<std::string, std::shared_ptr<chrono::ChLinkMotorRotation>> m_motors;
    std::shared_ptr<Part> m_wheel;

    bool m_collide_links;  ///< collide flag for all links (except final wheel)
    bool m_collide_wheel;  ///< collide flag for the final wheel
};

class ContactManager;

class RoboSimian {
  public:
    RoboSimian(chrono::ChMaterialSurface::ContactMethod contact_method, bool has_sled = false, bool fixed = false);
    RoboSimian(chrono::ChSystem* system, bool has_sled = false, bool fixed = false);
    ~RoboSimian();

    chrono::ChSystem* GetSystem() { return m_system; }

    /// Set collision flags for the various subsystems.
    /// By default, collision is enabled for the sled and wheels only.
    /// The 'flags' argument can be any of the CollisionFlag enums, or a combination thereof (using bit-wise operators).
    void SetCollide(int flags);

    void SetActuationData(const std::string& filename);

    void SetVisualizationTypeChassis(VisualizationType vis);
    void SetVisualizationTypeSled(VisualizationType vis);
    void SetVisualizationTypeLimbs(VisualizationType vis);
    void SetVisualizationTypeLimb(LimbID id, VisualizationType vis);
    void SetVisualizationTypeWheels(VisualizationType vis);

    void Initialize(const chrono::ChCoordsys<>& pos);

    void Activate(LimbID id, const std::string& motor_name, double time, double val);

    void Activate(double time);

    void DoStepDynamics(double step);

    void ReportContacts();

  private:
    void Create(bool has_sled, bool fixed);

    void LoadDataLine();

    chrono::ChSystem* m_system;  ///< pointer to the Chrono system
    bool m_owns_system;          ///< true if system created at construction

    bool m_has_data;                                       ///< true if an input actuation file was provided
    std::ifstream m_ifstream;                              ///< input file stream
    double m_time_1;                                       ///< time for cached activations
    double m_time_2;                                       ///< time for cached activations
    std::array<std::array<double, 8>, 4> m_activations_1;  ///< cached activations
    std::array<std::array<double, 8>, 4> m_activations_2;  ///< cached activations

    std::shared_ptr<Chassis> m_chassis;          ///< robot chassis
    std::shared_ptr<Sled> m_sled;                ///< optional sled attached to chassis
    std::vector<std::shared_ptr<Limb>> m_limbs;  ///< robot limbs
    ////std::shared_ptr<WheelDD> m_wheel_left;       ///< left DD wheel
    ////std::shared_ptr<WheelDD> m_wheel_right;      ///< right DD wheel

    ContactManager* m_contacts;
};

}  // end namespace robosimian

#endif
