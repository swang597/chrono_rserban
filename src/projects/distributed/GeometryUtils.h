#include "chrono/physics/ChBody.h"
#include "chrono/utils/ChUtilsSamplers.h"

using namespace chrono;

// Adds two spheres tangent to each other at the body center
void AddTangentBiSphere(ChBody* body, std::shared_ptr<ChMaterialSurface> mat, double radius) {
    utils::AddBiSphereGeometry(body, mat, radius / 2.0, radius);
}

// Adds two spheres whose boundaries coincide with each other's centers
void AddBiSphere(ChBody* body, std::shared_ptr<ChMaterialSurface> mat, double radius) {
    utils::AddBiSphereGeometry(body, mat, 2.0 * radius / 3.0, 2.0 * radius / 3.0);
}

// Adds a central large sphere flanked by two smaller spheres
void AddTrisphere(ChBody* body, std::shared_ptr<ChMaterialSurface> mat, double radius) {
    utils::AddSphereGeometry(body, mat, 2.0 * radius / 3.0);
    utils::AddSphereGeometry(body, mat, radius / 3.0, ChVector<>(2.0 * radius / 3.0, 0, 0));
    utils::AddSphereGeometry(body, mat, radius / 3.0, ChVector<>(-2.0 * radius / 3.0, 0, 0));
}

// Adds a large sphere on one side of the model and a small sphere on the other
void AddAsymmetricBisphere(ChBody* body, std::shared_ptr<ChMaterialSurface> mat, double radius) {
    utils::AddSphereGeometry(body, mat, 4.0 * radius / 5, ChVector<>(-radius / 5.0, 0, 0));
    utils::AddSphereGeometry(body, mat, 2.0 * radius / 5.0, ChVector<>(3.0 * radius / 5.0, 0, 0));
}

// Adds three colinear spheres in order decreading order of size
void AddSnowman(ChBody* body, std::shared_ptr<ChMaterialSurface> mat, double radius) {
    utils::AddSphereGeometry(body, mat, 4.0 * radius / 7.0, ChVector<>(-3.0 * radius / 7.0, 0, 0));
    utils::AddSphereGeometry(body, mat, 2.0 * radius / 7.0, ChVector<>(3.0 * radius / 7.0, 0, 0));
    utils::AddSphereGeometry(body, mat, radius / 7.0, ChVector<>(6.0 * radius / 7.0, 0, 0));
}
