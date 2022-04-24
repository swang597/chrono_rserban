// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// WVP Pacejka 2002 tire subsystem
//
// =============================================================================

#ifndef WVP_PACEJKA_TIRE_H
#define WVP_PACEJKA_TIRE_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChPacejkaTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace wvp {

/// Pacejka 2002 tire model for the WVP vehicle.
class CH_MODELS_API WVP_PacejkaTire : public ChPacejkaTire {
  public:
    WVP_PacejkaTire(const std::string& name);
    ~WVP_PacejkaTire() {}

    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    static const double m_mass;
    static const ChVector<> m_inertia;

    static const std::string m_pacTireFile;

    static const std::string m_meshFile_left;
    static const std::string m_meshFile_right;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono

#endif
