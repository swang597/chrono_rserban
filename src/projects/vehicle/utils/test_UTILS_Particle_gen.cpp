#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <unordered_map>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChGlobal.h"
#include "chrono_opengl/ChOpenGLWindow.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

using namespace chrono;

// =============================================================================

bool GetProblemSpecs(int argc, char** argv, std::string& obj_file, double& delta, double& min_depth) {
    delta = 0.1;
    min_depth = 0.2;

    ChCLI cli(argv[0], "BCE generator");

    cli.AddOption<std::string>("", "obj_file", "Input OBJ file (assumed relative to data/ directory)");
    cli.AddOption<double>("", "delta", "Particle spacing", std::to_string(delta));
    cli.AddOption<double>("", "min_depth", "Minimum soil depth", std::to_string(min_depth));

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    try {
        obj_file = cli.Get("obj_file").as<std::string>();
    } catch (std::domain_error&) {
        std::cout << "\nERROR: Missing input OBJ file!\n\n" << std::endl;
        cli.Help();
        return false;
    }
    delta = cli.GetAsType<double>("delta");
    min_depth = cli.GetAsType<double>("min_depth");

    return true;
}

// =============================================================================
// Hash function for a pair of integer grid coordinates
struct CoordHash {
  public:
    // 31 is just a decently-sized prime number to reduce bucket collisions
    std::size_t operator()(const ChVector2<int>& p) const { return p.x() * 31 + p.y(); }
};

// Calculate barycentric coordinates of point'v'
bool calcBarycentricCoordinates(const ChVector<>& v1,
                                const ChVector<>& v2,
                                const ChVector<>& v3,
                                const ChVector<>& v,
                                double& a1,
                                double& a2,
                                double& a3) {
    double denom = (v2.y() - v3.y()) * (v1.x() - v3.x()) + (v3.x() - v2.x()) * (v1.y() - v3.y());
    a1 = ((v2.y() - v3.y()) * (v.x() - v3.x()) + (v3.x() - v2.x()) * (v.y() - v3.y())) / denom;
    a2 = ((v3.y() - v1.y()) * (v.x() - v3.x()) + (v1.x() - v3.x()) * (v.y() - v3.y())) / denom;
    a3 = 1 - a1 - a2;

    return (0 <= a1) && (a1 <= 1) && (0 <= a2) && (a2 <= 1) && (0 <= a3) && (a3 <= 1);
}

int main(int argc, char* argv[]) {
    std::string obj_file;
    double delta;
    double min_depth;

    if (!GetProblemSpecs(argc, argv, obj_file, delta, min_depth)) {
        return 1;
    }

    // Load triangular mesh
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(GetChronoDataFile(obj_file), false, false);

    const auto& vertices = trimesh.getCoordsVertices();
    const auto& faces = trimesh.getIndicesVertexes();

    // Find x, y, and z ranges of vertex data
    auto minmaxX = std::minmax_element(begin(vertices), end(vertices),
                                       [](const ChVector<>& v1, const ChVector<>& v2) { return v1.x() < v2.x(); });
    auto minmaxY = std::minmax_element(begin(vertices), end(vertices),
                                       [](const ChVector<>& v1, const ChVector<>& v2) { return v1.y() < v2.y(); });
    auto minmaxZ = std::minmax_element(begin(vertices), end(vertices),
                                       [](const ChVector<>& v1, const ChVector<>& v2) { return v1.z() < v2.z(); });
    auto minX = minmaxX.first->x() + delta;
    auto maxX = minmaxX.second->x() - delta;
    auto minY = minmaxY.first->y() + delta;
    auto maxY = minmaxY.second->y() - delta;
    auto minZ = minmaxZ.first->z();
    auto maxZ = minmaxZ.second->z();

    auto sizeX = (maxX - minX);
    auto sizeY = (maxY - minY);
    ChVector<> center((maxX + minX) / 2, (maxY + minY) / 2, 0);

    int nx = static_cast<int>(std::ceil((sizeX / 2) / delta));  // half number of divisions in X direction
    int ny = static_cast<int>(std::ceil((sizeY / 2) / delta));  // number of divisions in Y direction
    delta = sizeX / (2.0 * nx);                                 // actual grid spacing

    std::cout << nx << "  " << ny << std::endl;

    std::unordered_map<ChVector2<int>, double, CoordHash> h_map;
    double a1, a2, a3;
    for (const auto& f : faces) {
        // Find bounds of (shifted) face projection
        const auto& v1 = vertices[f[0]] - center;
        const auto& v2 = vertices[f[1]] - center;
        const auto& v3 = vertices[f[2]] - center;
        auto x_min = ChMin(ChMin(v1.x(), v2.x()), v3.x());
        auto x_max = ChMax(ChMax(v1.x(), v2.x()), v3.x());
        auto y_min = ChMin(ChMin(v1.y(), v2.y()), v3.y());
        auto y_max = ChMax(ChMax(v1.y(), v2.y()), v3.y());
        int i_min = static_cast<int>(std::floor(x_min / delta));
        int j_min = static_cast<int>(std::floor(y_min / delta));
        int i_max = static_cast<int>(std::ceil(x_max / delta));
        int j_max = static_cast<int>(std::ceil(y_max / delta));
        ChClampValue(i_min, -nx, +nx);
        ChClampValue(i_max, -nx, +nx);
        ChClampValue(j_min, -ny, +ny);
        ChClampValue(j_max, -ny, +ny);
        // Loop over all grid nodes within bounds
        for (int i = i_min; i <= i_max; i++) {
            for (int j = j_min; j <= j_max; j++) {
                ChVector<> v(i * delta, j * delta, 0);
                if (calcBarycentricCoordinates(v1, v2, v3, v, a1, a2, a3)) {
                    double h = minZ + a1 * v1.z() + a2 * v2.z() + a3 * v3.z();
                    h_map.insert({ChVector2<int>(i, j), h});
                }
            }
        }
    }

    std::cout << h_map.size() << std::endl;

    // Create a Chrono system
    ChSystemSMC sys;
    sys.Set_G_acc(VNULL);
    double radius = delta;

    // Parse the hash map and create points in the corresponding vertical segment [-min_depth, h]
    std::vector<ChVector<>> points;
    for (const auto& p : h_map) {
        const auto& ij = p.first;
        auto v = ChVector<>(ij.x() * delta, ij.y() * delta, -min_depth) + center;
        while (v.z() <= p.second) {
            // Cache the point
            points.push_back(v);
            // Create and add a body for each point
            auto body = chrono_types::make_shared<ChBodyEasySphere>(radius, 1, true, false);
            body->SetPos(v);
            sys.AddBody(body);
            // Move up
            v.z() += delta;
        }
    }

    sys.Setup();
    std::cout << sys.GetNbodies() << std::endl;

    // Initialize OpenGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "JSON visualization", &sys);
    gl_window.SetCamera(ChVector<>(maxX, maxY, maxZ), ChVector<>(0, 0, 0.5), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::SOLID);

    while (gl_window.Active()) {
        gl_window.Render();
    }

    // Create output directory
    const std::string out_dir = "../TEST_PARTICLE_GEN";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Write particles to file
    std::ofstream myFile(out_dir + "/" + filesystem::path(obj_file).stem() + ".txt", std::ios::trunc);
    myFile << "x, y, z\n";
    for (const auto& p : points) {
        myFile << p.x() << ", " << p.y() << ", " << p.z() << ",\n";
    }
    myFile.close();

    return 0;
}
