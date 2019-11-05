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
// Authors: Radu Serban, Michael Taylor, Rainer Gericke
// =============================================================================
//
// FEDA PAC02 tire subsystem
//
// Coefficents were pulled from the Adams/Tire help - Adams 2017.1.
// https://simcompanion.mscsoftware.com/infocenter/index?page=content&id=DOC11293&cat=2017.1_ADAMS_DOCS&actp=LIST
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/feda/FEDA_Pac02Tire.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double FEDA_Pac02Tire::m_mass = 55.4;
const ChVector<> FEDA_Pac02Tire::m_inertia(6.39, 11.31, 6.39);

const std::string FEDA_Pac02Tire::m_meshName = "FEDA_tire_POV_geom";
const std::string FEDA_Pac02Tire::m_meshFile = "feda/tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FEDA_Pac02Tire::FEDA_Pac02Tire(const std::string& name) : ChPac02Tire(name), m_use_vert_map(false) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void FEDA_Pac02Tire::SetPac02Params() {  // begin of variables set up
    m_use_mode = 3;
    m_allow_mirroring = false;
    m_PacCoeff.R0 = 0.4987;
    m_PacCoeff.width = 0.335;
    m_PacCoeff.aspect_ratio = 0.65;
    m_PacCoeff.rim_radius = 0.2858;
    m_PacCoeff.rim_width = 0.2286;
    m_PacCoeff.Cz = 565190;
    m_PacCoeff.Kz = 751.791;
    m_PacCoeff.FzNomin = 21674;
    // begin scaling factors set up
    m_PacScal.lfz0 = 1;
    m_PacScal.lcx = 1;
    m_PacScal.lmux = 1;
    m_PacScal.lex = 1;
    m_PacScal.lkx = 1;
    m_PacScal.lhx = 1;
    m_PacScal.lvx = 1;
    m_PacScal.lcy = 1;
    m_PacScal.lmuy = 1;
    m_PacScal.ley = 1;
    m_PacScal.lky = 1;
    m_PacScal.lhy = 1;
    m_PacScal.lvy = 1;
    m_PacScal.lgay = 1;
    m_PacScal.ltr = 1;
    m_PacScal.lres = 1;
    m_PacScal.lgaz = 1;
    m_PacScal.lxal = 1;
    m_PacScal.lyka = 1;
    m_PacScal.lvyka = 1;
    m_PacScal.ls = 1;
    m_PacScal.lsgkp = 1;
    m_PacScal.lsgal = 1;
    m_PacScal.lgyr = 1;
    m_PacScal.lmx = 1;
    m_PacScal.lmy = 1;
    // setting longitidunal coefficients
    m_PacCoeff.pcx1 = 1.4;
    m_PacCoeff.pdx1 = 0.93385;
    m_PacCoeff.pdx2 = -0.043779;
    m_PacCoeff.pex1 = -5.8966;
    m_PacCoeff.pex2 = -7.0247;
    m_PacCoeff.pex3 = -0.21695;
    m_PacCoeff.pex4 = 0;
    m_PacCoeff.pkx1 = 7.5991;
    m_PacCoeff.pkx2 = 2.0158e-05;
    m_PacCoeff.pkx3 = -0.11869;
    m_PacCoeff.phx1 = 0;
    m_PacCoeff.phx2 = 0;
    m_PacCoeff.pvx1 = -0;
    m_PacCoeff.pvx2 = 0;
    m_PacCoeff.rbx1 = 10;
    m_PacCoeff.rbx2 = 6;
    m_PacCoeff.rcx1 = 1;
    m_PacCoeff.rhx1 = 0;
    // setting lateral coefficients
    m_PacCoeff.pcy1 = 1.2742;
    m_PacCoeff.pdy1 = -0.73151;
    m_PacCoeff.pdy2 = 0.10076;
    m_PacCoeff.pdy3 = -1.6121;
    m_PacCoeff.pey1 = 0.069355;
    m_PacCoeff.pey2 = -0.045834;
    m_PacCoeff.pey3 = 0.23519;
    m_PacCoeff.pey4 = 89.965;
    m_PacCoeff.pky1 = -12.265;
    m_PacCoeff.pky2 = 2.3291;
    m_PacCoeff.pky3 = 0.39846;
    m_PacCoeff.phy1 = 0.0041814;
    m_PacCoeff.phy2 = 0.0019571;
    m_PacCoeff.phy3 = -0.038875;
    m_PacCoeff.pvy1 = 0.0078979;
    m_PacCoeff.pvy2 = -0.0033858;
    m_PacCoeff.pvy3 = -0.21044;
    m_PacCoeff.pvy4 = -0.13928;
    m_PacCoeff.rby1 = 0;
    m_PacCoeff.rby2 = 0;
    m_PacCoeff.rby3 = 0;
    m_PacCoeff.rby1 = 0;
    m_PacCoeff.rhy1 = 0;
    m_PacCoeff.rvy1 = 0;
    m_PacCoeff.rvy2 = 0;
    m_PacCoeff.rvy3 = 0;
    m_PacCoeff.rvy4 = 0;
    m_PacCoeff.rvy5 = 0;
    m_PacCoeff.rvy6 = 0;
    // setting alignment coefficients
    m_PacCoeff.qbz1 = 10.231;
    m_PacCoeff.qbz2 = -2.8746;
    m_PacCoeff.qbz3 = -9.9609;
    m_PacCoeff.qbz4 = 0.58091;
    m_PacCoeff.qbz5 = -0.52975;
    m_PacCoeff.qbz9 = 0.5;
    m_PacCoeff.qcz1 = 1.4;
    m_PacCoeff.qdz1 = 0.079179;
    m_PacCoeff.qdz2 = -0.024616;
    m_PacCoeff.qdz3 = -0.031977;
    m_PacCoeff.qdz4 = 0.1399;
    m_PacCoeff.qdz6 = -0.0022134;
    m_PacCoeff.qdz7 = -0.0010696;
    m_PacCoeff.qdz8 = -0.017916;
    m_PacCoeff.qdz9 = 0.023003;
    m_PacCoeff.qez1 = -0.20626;
    m_PacCoeff.qez2 = -0.58411;
    m_PacCoeff.qez3 = -3.2451;
    m_PacCoeff.qez4 = 0.45327;
    m_PacCoeff.qez5 = 7.8689;
    m_PacCoeff.qhz1 = 0.0012666;
    m_PacCoeff.qhz2 = -0.0069367;
    m_PacCoeff.qhz3 = 0.090016;
    m_PacCoeff.qhz4 = 0.1671;
    m_PacCoeff.ssz1 = 0;
    m_PacCoeff.ssz2 = 0;
    m_PacCoeff.ssz3 = 0;
    m_PacCoeff.ssz4 = 0;
    // setting overturning coefficients
    m_PacCoeff.qsx1 = 0;
    m_PacCoeff.qsx2 = 0;
    m_PacCoeff.qsx3 = 0;
    // setting rolling coefficients
    m_PacCoeff.qsy1 = 0;
    m_PacCoeff.qsy2 = 0;

    m_use_vert_map = true;
    m_vert_map.AddPoint(0, 0);
    m_vert_map.AddPoint(0.005, 2004.06);
    m_vert_map.AddPoint(0.01, 4242.26);
    m_vert_map.AddPoint(0.015, 6688.46);
    m_vert_map.AddPoint(0.02, 9316.51);
    m_vert_map.AddPoint(0.025, 12100.2);
    m_vert_map.AddPoint(0.03, 15013.5);
    m_vert_map.AddPoint(0.035, 18030.2);
    m_vert_map.AddPoint(0.04, 21124.2);
    m_vert_map.AddPoint(0.045, 24269.2);
    m_vert_map.AddPoint(0.05, 27439.2);
    m_vert_map.AddPoint(0.055, 30607.9);
    m_vert_map.AddPoint(0.06, 33749.4);
    m_vert_map.AddPoint(0.065, 36837.3);
    m_vert_map.AddPoint(0.07, 39845.5);
    m_vert_map.AddPoint(0.075, 42748);
    m_vert_map.AddPoint(0.08, 45518.5);
    m_vert_map.AddPoint(0.085, 48130.9);
    m_vert_map.AddPoint(0.09, 50559.1);
    m_vert_map.AddPoint(0.095, 52776.8);
    m_vert_map.AddPoint(0.1, 54758);

    m_use_bott_map = true;
    m_bott_map.AddPoint(0, 0);
    m_bott_map.AddPoint(0.09, 0);
    m_bott_map.AddPoint(0.1, 100000);
    m_bott_map.AddPoint(0.2, 200000);
    m_bott_map.AddPoint(0.3, 300000);
    m_bott_map.AddPoint(0.4, 400000);
    m_bott_map.AddPoint(0.5, 500000);
    m_bott_map.AddPoint(0.6, 600000);
    m_bott_map.AddPoint(6, 6e+06);
}

double FEDA_Pac02Tire::GetNormalStiffnessForce(double depth) const {
    if (m_use_vert_map)
        if (m_use_bott_map) {
            return m_vert_map.Get_y(depth) + m_bott_map.Get_y(depth);
        } else {
            return m_vert_map.Get_y(depth);
        }
    else if (m_use_bott_map) {
        return depth * m_PacCoeff.Cz + m_bott_map.Get_y(depth);
    } else {
        return depth * m_PacCoeff.Cz;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void FEDA_Pac02Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        trimesh->Transform(ChVector<>(0, GetOffset(), 0), ChMatrix33<>(1));
        m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->GetSpindle()->AddAsset(m_trimesh_shape);
    } else {
        ChPac02Tire::AddVisualizationAssets(vis);
    }
}

void FEDA_Pac02Tire::RemoveVisualizationAssets() {
    ChPac02Tire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by FEDA_Pac02Tire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto& assets = m_wheel->GetSpindle()->GetAssets();
    auto it = std::find(assets.begin(), assets.end(), m_trimesh_shape);
    if (it != assets.end())
        assets.erase(it);
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
