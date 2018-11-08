#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_opengl/ChOpenGLWindow.h"

using namespace chrono;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, 0, 0));

    ////auto body0 = std::make_shared<ChBodyEasyBox>(1.0, 2.0, 3.0, 100);
    ////system.AddBody(body0);

    auto body1 = std::make_shared< ChBodyEasyMesh>(GetChronoDataFile("ob_chess_table.obj"), 1000, true, true);
    body1->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(-3.0, 0.0, 0.0), QUNIT));

    auto body2 = std::make_shared<ChBodyEasyMesh>(GetChronoDataFile("pallet.obj"), 1000, true, true);
    body2->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(+3.0, 0.0, 0.0), QUNIT));

    system.AddBody(body1);
    system.AddBody(body2);

    ////auto body1 = std::make_shared<ChBody>();
    ////body1->SetPos(ChVector<>(-3.0, 0.0, 0.0));
    ////system.AddBody(body1);
    ////auto trimesh1 = std::make_shared< geometry::ChTriangleMeshConnected>();
    ////trimesh1->LoadWavefrontMesh(GetChronoDataFile("ob_chess_table.obj"), true, false);
    ////auto vshape1 = std::make_shared<ChTriangleMeshShape>();
    ////vshape1->SetMesh(trimesh1);
    ////vshape1->SetStatic(true);
    ////vshape1->SetName("mesh1");
    ////body1->AddAsset(vshape1);

    ////auto body2 = std::make_shared<ChBody>();
    ////body2->SetPos(ChVector<>(3.0, 0.0, 0.0));
    ////system.AddBody(body2);
    ////auto trimesh2 = std::make_shared< geometry::ChTriangleMeshConnected>();
    ////trimesh2->LoadWavefrontMesh(GetChronoDataFile("pallet.obj"), true, false);
    ////auto vshape2 = std::make_shared<ChTriangleMeshShape>();
    ////vshape2->SetMesh(trimesh2);
    ////vshape2->SetStatic(true);
    ////vshape2->SetName("mesh2");
    ////body2->AddAsset(vshape2);

    auto& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1600, 900, "OpenGL meshes", &system);
    gl_window.SetCamera(ChVector<>(0.0, 10.0, 0.0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::SOLID);

    double step = 0.01;
    while (gl_window.Active()) {
        if (gl_window.Running()) {
            system.DoStepDynamics(step);
        }
        gl_window.Render();
    }
    return 0;
}