#include <fstream>

#include <thrust/copy.h>
#include <thrust/gather.h>
#include <thrust/for_each.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/functional.h>
#include <thrust/execution_policy.h>

#include "FindParticles.h"

using namespace chrono;
using namespace chrono::fsi;

struct in_box {
    in_box(const OBBspec& obb) : m_obb(obb) {}

    __device__ bool operator()(const Real4 v) {
        // Convert location in box frame
        auto d = mR3(v) - m_obb.p;
        auto w = mR3(                                                //
            m_obb.ax.x * d.x + m_obb.ax.y * d.y + m_obb.ax.z * d.z,  //
            m_obb.ay.x * d.x + m_obb.ay.y * d.y + m_obb.ay.z * d.z,  //
            m_obb.az.x * d.x + m_obb.az.y * d.y + m_obb.az.z * d.z   //
        );
        // Check w between all box limits
        const Real3& h = m_obb.h;
        return (w.x >= -h.x && w.x <= +h.x) && (w.y >= -h.y && w.y <= +h.y) && (w.z >= -h.z && w.z <= +h.z);
    }

    OBBspec m_obb;
};

struct print_particle_pos {
    print_particle_pos(std::ofstream* stream) : m_stream(stream) {}
    __host__ void operator()(const Real4 p) { (*m_stream) << p.x << ", " << p.y << ", " << p.z << "\n"; }
    std::ofstream* m_stream;
};

struct print_particle_pos_vel {
    print_particle_pos_vel(std::ofstream* stream) : m_stream(stream) {}
    template <typename T>
    __host__ void operator()(const T pv) {
        auto p = thrust::get<0>(pv);
        auto v = thrust::get<1>(pv);
        (*m_stream) << p.x << ", " << p.y << ", " << p.z << ", " << v.x << ", " << v.y << ", " << v.z << "\n";
    }
    std::ofstream* m_stream;
};

thrust::device_vector<int> FindParticlesInBox(std::shared_ptr<ChSystemFsi_impl> sysFSI, const OBBspec& obb) {
    // Extract indices of SPH particles contained in the OBB
    auto ref = sysFSI->fsiGeneralData->referenceArray;
    auto pos_D = sysFSI->sphMarkersD2->posRadD;

    // Find start and end locations for SPH particles (exclude ghost and BCE markers)
    bool haveHelper = (ref[0].z == -3) ? true : false;
    bool haveGhost = (ref[0].z == -2 || ref[1].z == -2) ? true : false;
    auto sph_start = ref[haveHelper + haveGhost].x;
    auto sph_end = ref[haveHelper + haveGhost].y;
    auto num_sph = sph_end - sph_start;

    // Preallocate output vector of indices
    thrust::device_vector<int> indices_D(num_sph);

    // Extract indices of SPH particles inside OBB
    thrust::counting_iterator<int> first(0);
    thrust::counting_iterator<int> last(num_sph);
    auto end = thrust::copy_if(thrust::device,     // execution policy
                               first, last,        // range of all particle indices
                               pos_D.begin(),      // stencil vector
                               indices_D.begin(),  // beginning of destination
                               in_box(obb)         // predicate for stencil elements
    );

    // Trim the output vector of indices
    size_t num_active = (size_t)(end - indices_D.begin());
    indices_D.resize(num_active);

    return indices_D;
}

void WriteParticlePos(std::shared_ptr<ChSystemFsi_impl> sysFSI,
                      const thrust::device_vector<int>& indices_D,
                      const std::string& filename) {
    // Gather positions from particles with specified indices
    const auto& allpos_D = sysFSI->sphMarkersD2->posRadD;

    thrust::device_vector<Real4> pos_D(allpos_D.size());

    auto end = thrust::gather(thrust::device,                      // execution policy
                              indices_D.begin(), indices_D.end(),  // range of gather locations
                              allpos_D.begin(),                    // beginning of source
                              pos_D.begin()                        // beginning of destination
    );

    // Trim the output vector of particle positions
    size_t num_active = (size_t)(end - pos_D.begin());
    assert(num_active == indices_D.size());
    pos_D.resize(num_active);

    // Copy vector to host
    thrust::host_vector<Real4> pos_H = pos_D;

    // Write output file
    std::ofstream stream;
    stream.open(filename, std::ios_base::trunc);
    thrust::for_each(thrust::host, pos_H.begin(), pos_H.end(), print_particle_pos(&stream));
    stream.close();
}

void WriteParticlePosVel(std::shared_ptr<ChSystemFsi_impl> sysFSI,
                         const thrust::device_vector<int>& indices_D,
                         const std::string& filename) {
    // Gather positions and velocities from particles with specified indices
    auto allpos_D = sysFSI->sphMarkersD2->posRadD;
    auto allvel_D = sysFSI->sphMarkersD2->velMasD;

    thrust::device_vector<Real4> pos_D(allpos_D.size());
    thrust::device_vector<Real3> vel_D(allpos_D.size());

    /*
    //// RADU TODO: - this gives errors! Bug in thrust?
    auto end = thrust::gather(
        thrust::device,                                                                     // execution policy
        indices_D.begin(), indices_D.end(),                                                 // range of gather locations
        thrust::make_zip_iterator(thrust::make_tuple(allpos_D.begin(), allvel_D.begin())),  // beginning of source
        thrust::make_zip_iterator(thrust::make_tuple(pos_D.begin(), vel_D.begin()))         // beginning of destination
    );
    */

    auto end = thrust::gather(thrust::device,                      // execution policy
                              indices_D.begin(), indices_D.end(),  // range of gather locations
                              allpos_D.begin(),                    // beginning of source
                              pos_D.begin()                        // beginning of destination
    );

    thrust::gather(thrust::device,                      // execution policy
                   indices_D.begin(), indices_D.end(),  // range of gather locations
                   allvel_D.begin(),                    // beginning of source
                   vel_D.begin()                        // beginning of destination
    );

    // Trim the output vectors of particle positions and velocities
    size_t num_active = (size_t)(end - pos_D.begin());
    assert(num_active == indices_D.size());
    pos_D.resize(num_active);
    vel_D.resize(num_active);

    // Copy vectors to host
    thrust::host_vector<Real4> pos_H = pos_D;
    thrust::host_vector<Real3> vel_H = vel_D;

    // Write output file
    std::ofstream stream;
    stream.open(filename, std::ios_base::trunc);
    thrust::for_each(thrust::host,                                                                 //
                     thrust::make_zip_iterator(thrust::make_tuple(pos_H.begin(), vel_H.begin())),  //
                     thrust::make_zip_iterator(thrust::make_tuple(pos_H.end(), vel_H.end())),      //
                     print_particle_pos_vel(&stream)                                               //
    );
    stream.close();
}
