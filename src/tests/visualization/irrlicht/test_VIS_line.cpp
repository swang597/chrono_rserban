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
// Test program for rendering polygonal line
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"

#include "chrono/core/ChBezierCurve.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/geometry/ChLineSegment.h"

#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

unsigned int num_line_points = 500;
unsigned int num_render_points = 1000;

int main(int argc, char* argv[]) {
    // Create the system (Y up)
    ChSystemNSC msystem;

    // Create a ground body
    auto ground = std::make_shared<ChBody>();
    ground->SetBodyFixed(true);
    ground->SetCollide(false);
    msystem.AddBody(ground);

    // Create a set of points
    double angle = CH_C_PI / 12;
    std::vector<ChVector<>> points;
    for (unsigned int i = 0; i < num_line_points; i++) {
        double r = i * 0.02;
        double x = 10 + r * std::cos(i * angle);
        double y = r * std::sin(i * angle);
        points.push_back(ChVector<>(x, y, 0));
    }

    // Create a path asset and add segments to it
    auto path_asset = std::make_shared<ChPathShape>();
    for (unsigned int i = 1; i < num_line_points; i++) {
        double len = (points[i] - points[i - 1]).Length();
        geometry::ChLineSegment segment(points[i - 1], points[i]);
        path_asset->GetPathGeometry()->AddSubLine(segment, len);
    }
    path_asset->SetNumRenderPoints(num_render_points);
    ground->AddAsset(path_asset);

    // Shift all points in X direction
    for (auto& p : points)
        p.x() -= 20;

    // Create a Bezier curve asset, reusing the points
    auto bezier_curve = std::make_shared<ChBezierCurve>(points);
    auto bezier_line = std::make_shared<geometry::ChLineBezier>(bezier_curve);
    auto bezier_asset = std::make_shared<ChLineShape>();
    bezier_asset->SetLineGeometry(bezier_line);
    bezier_asset->SetNumRenderPoints(num_render_points);
    ground->AddAsset(bezier_asset);

    // Create the Irrlicht visualization
    ChIrrApp application(&msystem, L"Line render test", irr::core::dimension2d<irr::u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(0, 0, -20));

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
