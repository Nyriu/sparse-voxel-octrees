/*
Copyright (c) 2013 Benedikt Bitterli

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

   1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.

   2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.

   3. This notice may not be removed or altered from any source
   distribution.
*/

#ifndef VOXELOCTREE_HPP_
#define VOXELOCTREE_HPP_

#include "math/Vec3.hpp"

#include "ChunkedAllocator.hpp"
#include "IntTypes.hpp"

#include <memory>
#include <vector>

class VoxelData;

class VoxelOctree {
    static const int32 MaxScale = 23;

    uint64 _octreeSize;
    std::unique_ptr<uint32[]> _octree;

    VoxelData *_voxels;
    Vec3 _center;

    uint64 buildOctree(ChunkedAllocator<uint32> &allocator, int x, int y, int z, int size, uint64 descriptorIndex);

public:
    VoxelOctree(const char *path);
    VoxelOctree(VoxelData *voxels);

    Vec3 center() const {
        return _center;
    }

    void save(const char *path);
    bool raymarch(const Vec3 &o, const Vec3 &d, float rayScale, uint32 &normal, float &t, const unsigned int rmode=0);
private:
    // only voxels
    bool raymarchV(const Vec3 &o, const Vec3 &d, float rayScale, uint32 &normal, float &t);
    // voxels and SDF
    bool raymarchSDF(const Vec3 &o, const Vec3 &d, float rayScale, uint32 &normal, float &t);
    // only SDF
    bool pureSphereTracing(const Vec3 &orig, const Vec3 &dir, float rayScale, uint32 &normal, float &t,
        //float max_distance=1e10
        float max_distance=1e2
        );

};

#endif /* VOXELOCTREE_HPP_ */
