// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Tests for various methods of calculating a global AABB (which encompasses the
// AABBs of a collection of collision shapes).
//
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>
#include <algorithm>
#include <climits>
#include <random>

#include <thrust/transform.h>
#include <thrust/transform_reduce.h>

#include "chrono/core/ChTimer.h"
#include "chrono/parallel/ChOpenMP.h"

#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/collision/ChCollision.h"
#include "chrono_parallel/collision/ChBroadphaseUtils.h"

#if defined(CHRONO_OPENMP_ENABLED)
#include <thrust/system/omp/execution_policy.h>
#elif defined(CHRONO_TBB_ENABLED)
#include <thrust/system/tbb/execution_policy.h>
#endif

#define EXEC_POLICY thrust::omp::par
////#define EXEC_POLICY

using namespace chrono;
using namespace chrono::collision;

// Original implementation in Chrono::Parallel (ChBroadphase.cpp)
void RigidBoundingBox(const custom_vector<real3>& aabb_min,
                      const custom_vector<real3>& aabb_max,
                      real3& min_point,
                      real3& max_point) {
    if (aabb_min.size() == 0)
        return;

    bbox res(aabb_min[0], aabb_min[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    res = thrust::transform_reduce(EXEC_POLICY, aabb_min.begin(), aabb_min.end(), unary_op, res, binary_op);
    res = thrust::transform_reduce(EXEC_POLICY, aabb_max.begin(), aabb_max.end(), unary_op, res, binary_op);

    min_point = res.first;
    max_point = res.second;
}

// A sequential implementation of the conditional algorithm
void RigidBoundingBox_seq(const custom_vector<real3>& aabb_min,
                          const custom_vector<real3>& aabb_max,
                          const custom_vector<uint>& id_rigid,
                          const custom_vector<char>& collide_rigid,
                          real3& min_point,
                          real3& max_point) {
    uint num_rigid_shapes = aabb_min.size();

    if (aabb_min.size() == 0)
        return;

    min_point = real3(+C_LARGE_REAL, +C_LARGE_REAL, +C_LARGE_REAL);
    max_point = real3(-C_LARGE_REAL, -C_LARGE_REAL, -C_LARGE_REAL);

    for (int i = 0; i < num_rigid_shapes; i++) {
        uint id = id_rigid[i];

        if (id == UINT_MAX || !collide_rigid[id]) {
            continue;
        }

        for (int j = 0; j < 3; j++) {
            if (aabb_min[i][j] < min_point[j])
                min_point[j] = aabb_min[i][j];
            if (aabb_max[i][j] > max_point[j])
                max_point[j] = aabb_max[i][j];
        }
    }
}

// A parallel implementation of the conditional algorithm
auto inverted = thrust::make_tuple(real3(+C_LARGE_REAL, +C_LARGE_REAL, +C_LARGE_REAL),
                                   real3(-C_LARGE_REAL, -C_LARGE_REAL, -C_LARGE_REAL),
                                   0);

struct BoxInvert {
    BoxInvert(const custom_vector<char>* collide) : m_collide(collide) {}
    thrust::tuple<real3, real3, uint> operator()(const thrust::tuple<real3, real3, uint>& lhs) {
        uint lhs_id = thrust::get<2>(lhs);
        if (lhs_id == UINT_MAX || (*m_collide)[lhs_id] == 0)
            return inverted;
        else
            return lhs;
    }
    const custom_vector<char>* m_collide;
};

struct BoxReduce {
    thrust::tuple<real3, real3, uint> operator()(const thrust::tuple<real3, real3, uint>& lhs,
                                                 const thrust::tuple<real3, real3, uint>& rhs) {
        real3 lhs_ll = thrust::get<0>(lhs);
        real3 lhs_ur = thrust::get<1>(lhs);

        real3 rhs_ll = thrust::get<0>(rhs);
        real3 rhs_ur = thrust::get<1>(rhs);

        real3 ll = real3(Min(lhs_ll.x, rhs_ll.x), Min(lhs_ll.y, rhs_ll.y), Min(lhs_ll.z, rhs_ll.z));
        real3 ur = real3(Max(lhs_ur.x, rhs_ur.x), Max(lhs_ur.y, rhs_ur.y), Max(lhs_ur.z, rhs_ur.z));

        return thrust::tuple<real3, real3, uint>(ll, ur, 0);
    }
};

void RigidBoundingBox_par(const custom_vector<real3>& aabb_min,
                          const custom_vector<real3>& aabb_max,
                          const custom_vector<uint>& id_rigid,
                          const custom_vector<char>& collide_rigid,
                          real3& min_point,
                          real3& max_point) {
    auto begin = thrust::make_zip_iterator(thrust::make_tuple(aabb_min.begin(), aabb_max.begin(), id_rigid.begin()));
    auto end = thrust::make_zip_iterator(thrust::make_tuple(aabb_min.end(), aabb_max.end(), id_rigid.end()));

    auto result = thrust::transform_reduce(EXEC_POLICY, begin, end, BoxInvert(&collide_rigid), inverted, BoxReduce());

    min_point = thrust::get<0>(result);
    max_point = thrust::get<1>(result);
}

// =================================================================================

int main(int argc, char* argv[]) {
    // Set number of threads
    int threads = 10;
    int max_threads = CHOMPfunctions::GetNumProcs();
    if (threads > max_threads)
        threads = max_threads;
    CHOMPfunctions::SetNumThreads(threads);

#pragma omp parallel
#pragma omp master
    std::cout << "Using " << CHOMPfunctions::GetNumThreads() << " threads.\n" << std::endl;

    // Set an arbitrary number of bodies
    uint nbodies = 100;

    // Miscellaneous uniform distributions
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist_id(0, nbodies - 1);
    std::uniform_int_distribution<> dist_coll(0, 1);
    std::uniform_real_distribution<> dist_loc(-1.0, +1.0);
    std::uniform_real_distribution<> dist_size(0.0, +1.0);

    // Number of shapes
    // - use a small number (e.g. 10) to check correctness
    // - use a lrage number (1 million +) to check performance
    uint nshapes = 1000000;

    // Set bounding boxes for the "shapes"
    custom_vector<real3> aabb_min(nshapes);
    custom_vector<real3> aabb_max(nshapes);
    for (uint i = 0; i < nshapes; i++) {
        aabb_min[i].x = 10 * dist_loc(gen);
        aabb_min[i].y = 20 * dist_loc(gen);
        aabb_min[i].z = 30 * dist_loc(gen);

        aabb_max[i].x = aabb_min[i].x + 1 * dist_size(gen);
        aabb_max[i].y = aabb_min[i].y + 2 * dist_size(gen);
        aabb_max[i].z = aabb_min[i].z + 3 * dist_size(gen);

        ////std::cout << id_rigid[i] << std::endl;
        ////std::cout << "  " << aabb_min[i].x << "  " << aabb_min[i].y << "  " << aabb_min[i].z << std::endl;
        ////std::cout << "  " << aabb_max[i].x << "  " << aabb_max[i].y << "  " << aabb_max[i].z << std::endl;
    }

    // Set "body" identifiers for all "shapes"
    // id_1:  all shapes active (valid body IDs)
    // id_2:  some shapes inactive (some IDs = UINT_MAX)
    custom_vector<uint> id_1(nshapes);
    custom_vector<uint> id_2(nshapes);
    for (uint i = 0; i < nshapes; i++) {
        id_1[i] = dist_id(gen);
        bool flip = dist_coll(gen);
        id_2[i] = flip ? UINT_MAX : dist_id(gen);
    }

    // Set collide flags for all "bodies"
    // collide_1:  all bodies collide
    // collide_2:  some bodies do not collide
    custom_vector<char> collide_1(nbodies);
    custom_vector<char> collide_2(nbodies);
    for (uint i = 0; i < nbodies; i++) {
        collide_1[i] = true;
        collide_2[i] = dist_coll(gen);
    }

    // ---------------------------------------------------------
    real3 minB, maxB;  // result for base algorithm
    real3 minS, maxS;  // result for conditional sequential algorithm
    real3 minP, maxP;  // result for conditional parallel algorithm

    ChTimer<> timer;
    // ---------------------------------------------------------

    std::cout << "\nTEST 1" << std::endl;
    std::cout << "   All shapes active.   All bodies colliding." << std::endl;

    timer.reset();
    timer.start();
    RigidBoundingBox(aabb_min, aabb_max, minB, maxB);
    timer.stop();
    std::cout << "Base Algorithm" << std::endl;
    std::cout << "  " << minB.x << "  " << minB.y << "  " << minB.z << std::endl;
    std::cout << "  " << maxB.x << "  " << maxB.y << "  " << maxB.z << std::endl;
    std::cout << "  Time: " << timer() << std::endl;

    timer.reset();
    timer.start();
    RigidBoundingBox_seq(aabb_min, aabb_max, id_1, collide_1, minS, maxS);
    timer.stop();
    std::cout << "Conditional Algorithm (sequential)" << std::endl;
    std::cout << "  " << minS.x << "  " << minS.y << "  " << minS.z << std::endl;
    std::cout << "  " << maxS.x << "  " << maxS.y << "  " << maxS.z << std::endl;
    std::cout << "  Time: " << timer() << std::endl;

    timer.reset();
    timer.start();
    RigidBoundingBox_par(aabb_min, aabb_max, id_1, collide_1, minS, maxS);
    timer.stop();
    std::cout << "Conditional Algorithm (parallel)" << std::endl;
    std::cout << "  " << minS.x << "  " << minS.y << "  " << minS.z << std::endl;
    std::cout << "  " << maxS.x << "  " << maxS.y << "  " << maxS.z << std::endl;
    std::cout << "  Time: " << timer() << std::endl;

    // ---------------------------------------------------------

    std::cout << "\nTEST 2" << std::endl;
    std::cout << "   All shapes active.  Some bodies not colliding." << std::endl;

    RigidBoundingBox(aabb_min, aabb_max, minB, maxB);
    std::cout << "Base Algorithm" << std::endl;
    std::cout << "  " << minB.x << "  " << minB.y << "  " << minB.z << std::endl;
    std::cout << "  " << maxB.x << "  " << maxB.y << "  " << maxB.z << std::endl;

    RigidBoundingBox_seq(aabb_min, aabb_max, id_1, collide_2, minS, maxS);
    std::cout << "Conditional Algorithm (sequential)" << std::endl;
    std::cout << "  " << minS.x << "  " << minS.y << "  " << minS.z << std::endl;
    std::cout << "  " << maxS.x << "  " << maxS.y << "  " << maxS.z << std::endl;

    RigidBoundingBox_par(aabb_min, aabb_max, id_1, collide_2, minS, maxS);
    std::cout << "Conditional Algorithm (parallel)" << std::endl;
    std::cout << "  " << minS.x << "  " << minS.y << "  " << minS.z << std::endl;
    std::cout << "  " << maxS.x << "  " << maxS.y << "  " << maxS.z << std::endl;

    // ---------------------------------------------------------

    std::cout << "\nTEST 3" << std::endl;
    std::cout << "   Some shapes inactivated.  All bodies collide." << std::endl;

    RigidBoundingBox(aabb_min, aabb_max, minB, maxB);
    std::cout << "Base Algorithm" << std::endl;
    std::cout << "  " << minB.x << "  " << minB.y << "  " << minB.z << std::endl;
    std::cout << "  " << maxB.x << "  " << maxB.y << "  " << maxB.z << std::endl;

    RigidBoundingBox_seq(aabb_min, aabb_max, id_2, collide_1, minS, maxS);
    std::cout << "Conditional Algorithm (sequential)" << std::endl;
    std::cout << "  " << minS.x << "  " << minS.y << "  " << minS.z << std::endl;
    std::cout << "  " << maxS.x << "  " << maxS.y << "  " << maxS.z << std::endl;

    RigidBoundingBox_par(aabb_min, aabb_max, id_2, collide_1, minS, maxS);
    std::cout << "Conditional Algorithm (parallel)" << std::endl;
    std::cout << "  " << minS.x << "  " << minS.y << "  " << minS.z << std::endl;
    std::cout << "  " << maxS.x << "  " << maxS.y << "  " << maxS.z << std::endl;

    // ---------------------------------------------------------

    std::cout << "\nTEST 4" << std::endl;
    std::cout << "   Some shapes inactivated.  Some bodies not colliding." << std::endl;

    RigidBoundingBox(aabb_min, aabb_max, minB, maxB);
    std::cout << "Base Algorithm" << std::endl;
    std::cout << "  " << minB.x << "  " << minB.y << "  " << minB.z << std::endl;
    std::cout << "  " << maxB.x << "  " << maxB.y << "  " << maxB.z << std::endl;

    RigidBoundingBox_seq(aabb_min, aabb_max, id_2, collide_2, minS, maxS);
    std::cout << "Conditional Algorithm (sequential)" << std::endl;
    std::cout << "  " << minS.x << "  " << minS.y << "  " << minS.z << std::endl;
    std::cout << "  " << maxS.x << "  " << maxS.y << "  " << maxS.z << std::endl;

    RigidBoundingBox_par(aabb_min, aabb_max, id_2, collide_2, minS, maxS);
    std::cout << "Conditional Algorithm (parallel)" << std::endl;
    std::cout << "  " << minS.x << "  " << minS.y << "  " << minS.z << std::endl;
    std::cout << "  " << maxS.x << "  " << maxS.y << "  " << maxS.z << std::endl;

    return 0;
}
