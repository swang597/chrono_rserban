/*
Name: Jack Collins
Date: 25/1/17
Description:
A parallel model of the simulation, currently won't run as the function utils::AddTriangularMesh creates an error when running.
*/

#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChSystemDescriptorParallel.h"
#include "chrono_parallel/collision/ChNarrowphaseRUtils.h"
#include "chrono_parallel/collision/ChBroadphaseUtils.h"

#include "chrono_opengl/ChOpenGLWindow.h"
#include "chrono_opengl/shapes/ChOpenGLOBJ.h"
#include "chrono_opengl/core/ChOpenGLMaterial.h"

#include "chrono_vehicle/terrain/DeformableTerrain.h"

////#include "chrono_postprocess/ChPovRay.h"
////#include "chrono_postprocess/ChPovRayAssetCustom.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::opengl;
////using namespace postprocess; 

using std::cout;
using std::endl;


std::shared_ptr<ChBody> CreateBracketA(ChSystemParallel* system) {

    auto BracketA = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::NSC);

    BracketA->SetMass(0.256);
    BracketA->SetPos(ChVector<>(0, 0, 0));
    BracketA->SetRot(ChQuaternion<>(1, 0, 0, 0));
    BracketA->SetCollide(true);

    BracketA->GetCollisionModel()->ClearModel();
    utils::AddTriangleMeshGeometry(BracketA.get(),GetChronoDataFile("BracketA.obj"), "BracketA", ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), true);
    BracketA->GetCollisionModel()->BuildModel();


    system->AddBody(BracketA);

    auto cm = std::static_pointer_cast<ChCollisionModelParallel>(BracketA->GetCollisionModel())->mData;

    custom_vector<real3> aabb_min;
    custom_vector<real3> aabb_max;
    int count = 0;

    for (size_t i = 0; i < cm.size(); i++) {
        real3 A = cm[i].A;
        real3 B = cm[i].B;
        real3 C = cm[i].C;

        real3 minp, maxp;

        minp.x = Min(A.x, Min(B.x, C.x));
        minp.y = Min(A.y, Min(B.y, C.y));
        minp.z = Min(A.z, Min(B.z, C.z));
        maxp.x = Max(A.x, Max(B.x, C.x));
        maxp.y = Max(A.y, Max(B.y, C.y));
        maxp.z = Max(A.z, Max(B.z, C.z));

        aabb_min.push_back(minp);
        aabb_max.push_back(maxp);

        real vol = (maxp.x - minp.x) * (maxp.y - minp.y) * (maxp.z - minp.z);

        real AB = Length(B - A);
        real BC = Length(C - B);
        real CA = Length(A - C);

        if (AB <= 0 || BC <= 0 || CA <= 0) {
            cout << i << endl;
            cout << A.x << " " << A.y << " " << A.z << endl;
            cout << B.x << " " << B.y << " " << B.z << endl;
            cout << C.x << " " << C.y << " " << C.z << endl;
            count++;
        }
    }

    cout << endl << count << endl;
    cout << "Num. faces: " << cm.size() << endl;

    // determine the bounds on the total space and subdivide based on the bins per axis
    bbox res(aabb_min[0], aabb_min[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    res = thrust::transform_reduce(aabb_min.begin(), aabb_min.end(), unary_op, res, binary_op);
    res = thrust::transform_reduce(aabb_max.begin(), aabb_max.end(), unary_op, res, binary_op);

    real3 minp = res.first;
    real3 maxp = res.second;

    real3 size = maxp - minp;

    cout << "Domain" << endl;
    cout << minp.x << " " << minp.y << " " << minp.z << endl;
    cout << maxp.x << " " << maxp.y << " " << maxp.z << endl;
    cout << size.x << " " << size.y << " " << size.z << endl;


    return BracketA;
}


// ========================================================================

int main(int argc, char* argv[]) {

    ////SetChronoDataPath(CHRONO_DATA_DIR);


    // Create system
    ChSystemParallelNSC* msystem = new ChSystemParallelNSC();

    // Set number of threads.
    int threads = omp_get_num_procs();
    msystem->SetParallelThreadNumber(threads);


    double time_step = 0.01;
    omp_set_num_threads(threads);


    msystem->SetStep(time_step);

    msystem->Set_G_acc(ChVector<>(0, -9.81, 0));

    msystem->GetSettings()->solver.tolerance = 1e-3;

    msystem->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    msystem->GetSettings()->solver.max_iteration_normal = 30;
    msystem->GetSettings()->solver.max_iteration_sliding = 30;
    msystem->GetSettings()->solver.max_iteration_spinning = 0;
    msystem->GetSettings()->solver.alpha = 0;
    msystem->GetSettings()->solver.contact_recovery_speed = 0.1;
    msystem->ChangeSolverType(SolverType::APGDREF);

    msystem->GetSettings()->collision.collision_envelope = 0.05;

    msystem->GetSettings()->collision.bins_per_axis = vec3(1, 1, 1);




//------------------------------------------------------------------ Solid Bodies -----------------------------------------------------------

    std::shared_ptr<ChBody> BracketA;

    BracketA = CreateBracketA(msystem);

//------------------------------------------------------------------ Markers -----------------------------------------------------------


//------------------------------------------------------------------ Links-----------------------------------------------------------

    ////Set the motor as a fixed body, this will be changed later so that it moves in the "Y" direction with a lower limit.
    BracketA->SetBodyFixed(true);

//------------------------------------------------------------------ POVRay-----------------------------------------------------------

    ////ChPovRay pov_exporter = ChPovRay(msystem);
    ////// Sets some file names for in-out processes.
    ////pov_exporter.SetTemplateFile(GetChronoDataFile("_template_POV.pov"));
    ////pov_exporter.SetOutputScriptFile("rendering_frames.pov");
    ////pov_exporter.SetOutputDataFilebase("my_state");
    ////pov_exporter.SetPictureFilebase("picture");


    ////ChFileutils::MakeDirectory("output");
    ////ChFileutils::MakeDirectory("anim");
    ////pov_exporter.SetOutputDataFilebase("output/my_state");
    ////pov_exporter.SetPictureFilebase("anim/picture");
  
    ////pov_exporter.AddAll();

    ////pov_exporter.ExportScript();




//------------------------------------------------------------------ THE VISUALIZATION-----------------------------------------------------------
    ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "M113", msystem);
    gl_window.SetCamera(ChVector<>(0, -1, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
    gl_window.SetRenderMode(opengl::WIREFRAME);

    while (1) {

        gl_window.Render();
        msystem->DoStepDynamics(time_step);
        ////pov_exporter.ExportData();

    }


    return 0;
}