#include <string>

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include "chrono_fsi/ChSystemFsi_impl.cuh"
#include "chrono_fsi/physics/ChSphGeneral.cuh"

struct OBBspec {
    chrono::fsi::Real3 h;           // OBB half-size
    chrono::fsi::Real3 p;           // OBB center
    chrono::fsi::Real3 ax, ay, az;  // OBB rotation matrix columns
};

thrust::device_vector<int> FindParticlesInBox(std::shared_ptr<chrono::fsi::ChSystemFsi_impl> sysFSI,
                                              const OBBspec& obb);
void WriteParticlePos(std::shared_ptr<chrono::fsi::ChSystemFsi_impl> sysFSI,
                      const thrust::device_vector<int>& indices_D,
                      const std::string& filename);
void WriteParticlePosVel(std::shared_ptr<chrono::fsi::ChSystemFsi_impl> sysFSI,
                         const thrust::device_vector<int>& indices_D,
                         const std::string& filename);
