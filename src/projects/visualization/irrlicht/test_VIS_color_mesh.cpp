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
// Test program for coloring a mesh visualization asset
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"

#include "chrono/assets/ChAssetLevel.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChObjShapeFile.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

unsigned int num_line_points = 500;
unsigned int num_render_points = 1000;

int main(int argc, char* argv[]) {
    // Create the system (Y up)
    ChSystemNSC msystem;

    // Create a ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetBodyFixed(true);
    ground->SetCollide(false);
    msystem.AddBody(ground);

    // Visualization
    auto vis_level = chrono_types::make_shared<ChAssetLevel>();
    ground->AddAsset(vis_level);

    auto vis_mesh = chrono_types::make_shared<ChObjShapeFile>();
    vis_mesh->SetFilename(GetChronoDataFile("models/tractor_wheel/tractor_wheel.obj"));
    vis_level->AddAsset(vis_mesh);

    vis_level->AddAsset(chrono_types::make_shared<ChColorAsset>(0.5f, 0.0f, 0.0f));

    // Create the Irrlicht visualization
    ChIrrApp application(&msystem, L"Mesh color", irr::core::dimension2d<irr::u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(2, 0, 0));

    // Complete asset construction
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.EndScene();
        msystem.DoStepDynamics(0.1);
    }

    return 0;
}
