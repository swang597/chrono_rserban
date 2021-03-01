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
// Test for SCM terrain patch generated from height-map
// =============================================================================

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Global parameter for tire:
    double tire_rad = 0.8;
    ChVector<> tire_center(-4.0, 0, 0.02 + tire_rad);

    ChSystemSMC my_system;
    my_system.Set_G_acc(ChVector<>(0., 0, -9.8));

    // Create the Irrlicht visualization
    ChIrrApp application(&my_system, L"SCM test", core::dimension2d<u32>(1280, 720), VerticalDir::Z);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(2.0f, 2.0f, 2.5f));

    std::shared_ptr<ChBody> mtruss(new ChBody);
    mtruss->SetBodyFixed(true);
    my_system.Add(mtruss);

    // Create a rigid body with a cylinder collision shape
    std::shared_ptr<ChBody> mrigidbody(new ChBody);
    my_system.Add(mrigidbody);
    mrigidbody->SetMass(500);
    mrigidbody->SetInertiaXX(ChVector<>(20, 20, 20));
    mrigidbody->SetPos(tire_center + ChVector<>(0, 0, 0.3));

    double radius = 0.5;
    double width = 0.4;

    auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mrigidbody->GetCollisionModel()->ClearModel();
    mrigidbody->GetCollisionModel()->AddCylinder(material, radius, radius, width / 2);
    mrigidbody->GetCollisionModel()->BuildModel();
    mrigidbody->SetCollide(true);

    auto cyl_shape = chrono_types::make_shared<ChCylinderShape>();
    cyl_shape->GetCylinderGeometry().rad = radius;
    cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, +width / 2, 0);
    cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
    mrigidbody->AddAsset(cyl_shape);

    std::shared_ptr<ChColorAsset> mcol(new ChColorAsset);
    mcol->SetColor(ChColor(0.3f, 0.3f, 0.3f));
    mrigidbody->AddAsset(mcol);

    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->SetSpindleConstraint(ChLinkMotorRotation::SpindleConstraint::OLDHAM);
    motor->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, CH_C_PI / 4.0));
    motor->Initialize(mrigidbody, mtruss, ChFrame<>(tire_center, Q_from_AngX(-CH_C_PI_2)));
    my_system.Add(motor);

    // Create the 'deformable terrain' object
    vehicle::SCMDeformableTerrain mterrain(&my_system);

    // Use either a regular grid:
    ////mterrain.Initialize(6, 2, 0.04);

    // Or use a height map:
    mterrain.Initialize(vehicle::GetDataFile("terrain/height_maps/slope.bmp"), 12, 2, 0.0, 0.5, 0.04);

    // Set the soil terramechanical parameters
    mterrain.SetSoilParameters(0.2e6,  // Bekker Kphi
                               0,      // Bekker Kc
                               1.1,    // Bekker n exponent
                               0,      // Mohr cohesive limit (Pa)
                               30,     // Mohr friction limit (degrees)
                               0.01,   // Janosi shear coefficient (m)
                               4e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                               3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE, 0, 30000.2);
    ////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.15);
    mterrain.GetMesh()->SetWireframe(true);

    application.AssetBindAll();
    application.AssetUpdateAll();
    application.SetTimestep(0.002);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.GetActiveCamera()->setTarget(core::vector3dfCH(mrigidbody->GetPos()));
        application.GetActiveCamera()->setPosition(core::vector3dfCH(mrigidbody->GetPos() + ChVector<>(0, 2, 0)));
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
