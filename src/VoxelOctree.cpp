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

#include "VoxelOctree.hpp"
#include "VoxelData.hpp"
#include "Debug.hpp"
#include "Util.hpp"

#include "third-party/lz4.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <ios>
#include <iostream>
#include <stdio.h>
#include <cmath>

#include <bitset>
#include <inttypes.h>

#include <limits>
constexpr float infinity = std::numeric_limits<float>::max();


static const uint32 BitCount[] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};

static const size_t CompressionBlockSize = 64*1024*1024;

VoxelOctree::VoxelOctree(const char *path) : _voxels(0) {
    FILE *fp = fopen(path, "rb");

    if (fp) {

        fread(_center.a, sizeof(float), 3, fp);
        fread(&_octreeSize, sizeof(uint64), 1, fp);

        _octree.reset(new uint32[_octreeSize]);

        std::unique_ptr<char[]> buffer(new char[LZ4_compressBound(CompressionBlockSize)]);
        char *dst = reinterpret_cast<char *>(_octree.get());

        LZ4_streamDecode_t *stream = LZ4_createStreamDecode();
        LZ4_setStreamDecode(stream, dst, 0);

        uint64 compressedSize = 0;
        for (uint64 offset = 0; offset < _octreeSize*sizeof(uint32); offset += CompressionBlockSize) {
            uint64 compSize;
            fread(&compSize, sizeof(uint64), 1, fp);
            fread(buffer.get(), sizeof(char), size_t(compSize), fp);

            int outSize = std::min<int>(_octreeSize*sizeof(uint32) - offset, CompressionBlockSize);
            LZ4_decompress_fast_continue(stream, buffer.get(), dst + offset, outSize);
            compressedSize += compSize + 8;
        }
        LZ4_freeStreamDecode(stream);

        fclose(fp);

        std::cout << "Octree size: " << prettyPrintMemory(_octreeSize*sizeof(uint32))
                  << " Compressed size: " << prettyPrintMemory(compressedSize) << std::endl;
    }
}

void VoxelOctree::save(const char *path) {
    FILE *fp = fopen(path, "wb");

    if (fp) {
        fwrite(_center.a, sizeof(float), 3, fp);
        fwrite(&_octreeSize, sizeof(uint64), 1, fp);

        LZ4_stream_t *stream = LZ4_createStream();
        LZ4_resetStream(stream);

        std::unique_ptr<char[]> buffer(new char[LZ4_compressBound(CompressionBlockSize)]);
        const char *src = reinterpret_cast<char *>(_octree.get());

        uint64 compressedSize = 0;
        for (uint64 offset = 0; offset < _octreeSize*sizeof(uint32); offset += CompressionBlockSize) {
            int outSize = int(std::min(_octreeSize*sizeof(uint32) - offset, uint64(CompressionBlockSize)));
            uint64 compSize = LZ4_compress_continue(stream, src + offset, buffer.get(), outSize);

            fwrite(&compSize, sizeof(uint64), 1, fp);
            fwrite(buffer.get(), sizeof(char), size_t(compSize), fp);

            compressedSize += compSize + 8;
        }

        LZ4_freeStream(stream);

        fclose(fp);

        std::cout << "Octree size: " << prettyPrintMemory(_octreeSize*sizeof(uint32))
                  << " Compressed size: " << prettyPrintMemory(compressedSize) << std::endl;
    }
}

VoxelOctree::VoxelOctree(VoxelData *voxels)
: _voxels(voxels)
{
    std::unique_ptr<ChunkedAllocator<uint32>> octreeAllocator(new ChunkedAllocator<uint32>());
    octreeAllocator->pushBack(0); // reserve root's childdesc

    buildOctree(*octreeAllocator, 0, 0, 0, _voxels->sideLength(), 0);
    (*octreeAllocator)[0] |= 1 << 18; // set 19th bit of root's childdesc // means offset = 1

    _octreeSize = octreeAllocator->size() + octreeAllocator->insertionCount();
    _octree = octreeAllocator->finalize();
    _center = _voxels->getCenter();
}

uint64 VoxelOctree::buildOctree(ChunkedAllocator<uint32> &allocator, int x, int y, int z, int size, uint64 descriptorIndex) {
    _voxels->prepareDataAccess(x, y, z, size);

    int halfSize = size >> 1;

    int posX[] = {x + halfSize, x, x + halfSize, x, x + halfSize, x, x + halfSize, x};
    int posY[] = {y + halfSize, y + halfSize, y, y, y + halfSize, y + halfSize, y, y};
    int posZ[] = {z + halfSize, z + halfSize, z + halfSize, z + halfSize, z, z, z, z};

    uint64 childOffset = uint64(allocator.size()) - descriptorIndex; // cells to skip before putting first child

    int childCount = 0;
    int childIndices[8];
    uint32 childMask = 0;
    for (int i = 0; i < 8; i++) {
        if (_voxels->cubeContainsVoxelsDestructive(posX[i], posY[i], posZ[i], halfSize)) {
            childMask |= 128 >> i;
            childIndices[childCount++] = i;
        }
    }

    bool hasLargeChildren = false;
    uint32 leafMask;
    if (halfSize == 1) {
        leafMask = 0;

        for (int i = 0; i < childCount; i++) {
            int idx = childIndices[childCount - i - 1];
            allocator.pushBack(_voxels->getVoxelDestructive(posX[idx], posY[idx], posZ[idx]));
        }
    } else {
        leafMask = childMask;
        for (int i = 0; i < childCount; i++)
            allocator.pushBack(0); // reserve slot for child's childdesc

        uint64 grandChildOffsets[8];
        uint64 delta = 0;
        uint64 insertionCount = allocator.insertionCount();
        for (int i = 0; i < childCount; i++) {
            int idx = childIndices[childCount - i - 1];
            grandChildOffsets[i] = delta + buildOctree(allocator, posX[idx], posY[idx], posZ[idx],
                halfSize, descriptorIndex + childOffset + i);
            delta += allocator.insertionCount() - insertionCount; // add insertions number added in child subtree
            insertionCount = allocator.insertionCount();
            if (grandChildOffsets[i] > 0x3FFF) // offset cannot be contained in 14bits
                hasLargeChildren = true;
        }

        for (int i = 0; i < childCount; i++) {
            uint64 childIndex = descriptorIndex + childOffset + i;
            uint64 offset = grandChildOffsets[i];
            if (hasLargeChildren) { // if at least one childoffset is big, all are treated differently
                offset += childCount - i;
                allocator.insert(childIndex + 1, uint32(offset));
                allocator[childIndex] |= 0x20000; // set 18th bit on child's childescriptor // means parent handled the insertion "from above"
                offset >>= 32;
            }
            allocator[childIndex] |= uint32(offset << 18); // save the offset in the first 14 bits?
        }
    }

    allocator[descriptorIndex] = (childMask << 8) | leafMask; // save masks in current childdesc
    if (hasLargeChildren)
        allocator[descriptorIndex] |= 0x10000; // set 17th bit in current childdesc if large children

    return childOffset;
}

bool VoxelOctree::raymarch(const Vec3 &o, const Vec3 &d, float rayScale, uint32 &normal, float &t, const unsigned int rmode) {
  float rayScale_factor = 1.f/10.f;
  if (rmode == 1) {
    rayScale *= rayScale_factor;
    return raymarchSDF(o, d, rayScale, normal, t);
  }
  if (rmode == 2) {
    rayScale *= rayScale_factor;
    return pureSphereTracing(o, d, rayScale, normal, t);
  }
  return raymarchV(o, d, rayScale, normal, t);
}

bool VoxelOctree::raymarchV(const Vec3 &o, const Vec3 &d, float rayScale, uint32 &normal, float &t) {
    struct StackEntry {
        uint64 offset;
        float maxT;
    };
    StackEntry rayStack[MaxScale + 1];

    float ox = o.x, oy = o.y, oz = o.z;
    float dx = d.x, dy = d.y, dz = d.z;

    if (std::fabs(dx) < 1e-4f) dx = 1e-4f;
    if (std::fabs(dy) < 1e-4f) dy = 1e-4f;
    if (std::fabs(dz) < 1e-4f) dz = 1e-4f;

    float dTx = 1.0f/-std::fabs(dx);
    float dTy = 1.0f/-std::fabs(dy);
    float dTz = 1.0f/-std::fabs(dz);

    float bTx = dTx*ox;
    float bTy = dTy*oy;
    float bTz = dTz*oz;

    uint8 octantMask = 7;
    if (dx > 0.0f) octantMask ^= 1, bTx = 3.0f*dTx - bTx;
    if (dy > 0.0f) octantMask ^= 2, bTy = 3.0f*dTy - bTy;
    if (dz > 0.0f) octantMask ^= 4, bTz = 3.0f*dTz - bTz;

    float minT = std::max(2.0f*dTx - bTx, std::max(2.0f*dTy - bTy, 2.0f*dTz - bTz));
    float maxT = std::min(     dTx - bTx, std::min(     dTy - bTy,      dTz - bTz));
    minT = std::max(minT, 0.0f);

    uint32 current = 0;
    uint64 parent  = 0;
    int idx     = 0;
    float posX  = 1.0f;
    float posY  = 1.0f;
    float posZ  = 1.0f;
    int scale   = MaxScale - 1;

    float scaleExp2 = 0.5f;

    if (1.5f*dTx - bTx > minT) idx ^= 1, posX = 1.5f;
    if (1.5f*dTy - bTy > minT) idx ^= 2, posY = 1.5f;
    if (1.5f*dTz - bTz > minT) idx ^= 4, posZ = 1.5f;

    while (scale < MaxScale) {
        if (current == 0)
            current = _octree[parent];

        float cornerTX = posX*dTx - bTx;
        float cornerTY = posY*dTy - bTy;
        float cornerTZ = posZ*dTz - bTz;
        float maxTC = std::min(cornerTX, std::min(cornerTY, cornerTZ));

        int childShift = idx ^ octantMask;
        uint32 childMasks = current << childShift;

        if ((childMasks & 0x8000) && minT <= maxT) { // if childExists and ray not arrived end
            if (maxTC*rayScale >= scaleExp2) {
                t = maxTC;
                return true;
            }

            float maxTV = std::min(maxT, maxTC);
            float half = scaleExp2*0.5f;
            float centerTX = half*dTx + cornerTX;
            float centerTY = half*dTy + cornerTY;
            float centerTZ = half*dTz + cornerTZ;

            if (minT <= maxTV) {
                uint64 childOffset = current >> 18; // ignore first 18 bits = 8+8 masks + hasLargeChildren + parent hasLargeChildren (so I've large sibling)
                                                    // this is used as "pointer" as index of an array of childdescriptors
                if (current & 0x20000) // 18th // means parent hasLargeChildren // so current has large siblings
                    childOffset = (childOffset << 32) | uint64(_octree[parent + 1]); // recreate old offset  

                if (!(childMasks & 0x80)) { // if leaf
                    normal = _octree[childOffset + parent + BitCount[((childMasks >> (8 + childShift)) << childShift) & 127]]; // parent+childOffset make "pointer" to first childDescriptor, then BitCount[...] gives another offset for the right childDescriptor, wiht &127 we look only the last 7 bits
                    break;
                }

                rayStack[scale].offset = parent;
                rayStack[scale].maxT = maxT;

                uint32 siblingCount = BitCount[childMasks & 127]; // use non-leaf mask
                parent += childOffset + siblingCount;
                if (current & 0x10000) // 17th // hasLargeChildren so child occupy x2 space because of insertions
                    parent += siblingCount;

                idx = 0;
                scale--;
                scaleExp2 = half;

                // select child and "pull near the voxel opposite corner" because ray travels positive->negative and the corner is the smallest
                if (centerTX > minT) idx ^= 1, posX += scaleExp2;
                if (centerTY > minT) idx ^= 2, posY += scaleExp2;
                if (centerTZ > minT) idx ^= 4, posZ += scaleExp2;

                maxT = maxTV;
                current = 0; // reset current to force fetch on next loop

                continue;
            }
        }
        // ADVANCE
        int stepMask = 0;
        if (cornerTX <= maxTC) stepMask ^= 1, posX -= scaleExp2; // "push away the voxel opposite corner" because ray travels positive->negative
        if (cornerTY <= maxTC) stepMask ^= 2, posY -= scaleExp2; // and idx will become smaller and smaller
        if (cornerTZ <= maxTC) stepMask ^= 4, posZ -= scaleExp2;

        minT = maxTC; // move "ray head"
        idx ^= stepMask;
        // POP
        if ((idx & stepMask) != 0) {
            int differingBits = 0;
            if (stepMask & 1) differingBits |= floatBitsToUint(posX) ^ floatBitsToUint(posX + scaleExp2);
            if (stepMask & 2) differingBits |= floatBitsToUint(posY) ^ floatBitsToUint(posY + scaleExp2);
            if (stepMask & 4) differingBits |= floatBitsToUint(posZ) ^ floatBitsToUint(posZ + scaleExp2);
            scale = (floatBitsToUint((float)differingBits) >> 23) - 127; // highest differing bit
            scaleExp2 = uintBitsToFloat((scale - MaxScale + 127) << 23);

            parent = rayStack[scale].offset;
            maxT   = rayStack[scale].maxT;

            int shX = floatBitsToUint(posX) >> scale;
            int shY = floatBitsToUint(posY) >> scale;
            int shZ = floatBitsToUint(posZ) >> scale;
            posX = uintBitsToFloat(shX << scale);
            posY = uintBitsToFloat(shY << scale);
            posZ = uintBitsToFloat(shZ << scale);
            idx = (shX & 1) | ((shY & 1) << 1) | ((shZ & 1) << 2);

            current = 0;
        }
    }

    if (scale >= MaxScale)
        return false;

    t = minT; // we arrive here with the while break after reaching a leaf
    return true;
}


float sdf(const Vec3 &p);
inline Vec3 normalAt(const Vec3& p);
static inline Vec3 unorm_coord(Vec3 nc, Vec3 p0, Vec3 region_dims);
static constexpr float hit_threshold = 10e-6; // min distance to signal a ray-surface hit

bool VoxelOctree::raymarchSDF(const Vec3 &orig, const Vec3 &dir, float rayScale, uint32 &normal, float &t) {
    struct StackEntry {
        uint64 offset;
        float maxT;
    };
    StackEntry rayStack[MaxScale + 1];

    //if (
    //    //rayScale != 0.f
    //    //true
    //    //false
    //    ) {
    //  printf("\n");
    //  printf("orig {%f %f %f}\n",
    //      orig.x,
    //      orig.y,
    //      orig.z
    //      );
    //  printf("dir {%f %f %f}\n",
    //      dir.x,
    //      dir.y,
    //      dir.z
    //      );
    //}


    float ox = orig.x, oy = orig.y, oz = orig.z;
    float dx = dir.x, dy = dir.y, dz = dir.z;

    if (std::fabs(dx) < 1e-4f) dx = 1e-4f;
    if (std::fabs(dy) < 1e-4f) dy = 1e-4f;
    if (std::fabs(dz) < 1e-4f) dz = 1e-4f;

    float dTx = 1.0f/-std::fabs(dx);
    float dTy = 1.0f/-std::fabs(dy);
    float dTz = 1.0f/-std::fabs(dz);

    float bTx = dTx*ox;
    float bTy = dTy*oy;
    float bTz = dTz*oz;

    uint8 octantMask = 7;
    if (dx > 0.0f) octantMask ^= 1, bTx = 3.0f*dTx - bTx;
    if (dy > 0.0f) octantMask ^= 2, bTy = 3.0f*dTy - bTy;
    if (dz > 0.0f) octantMask ^= 4, bTz = 3.0f*dTz - bTz;

    float minT = std::max(2.0f*dTx - bTx, std::max(2.0f*dTy - bTy, 2.0f*dTz - bTz));
    float maxT = std::min(     dTx - bTx, std::min(     dTy - bTy,      dTz - bTz));
    minT = std::max(minT, 0.0f);

    uint32 current = 0;
    uint64 parent  = 0;
    int idx     = 0;
    float posX  = 1.0f;
    float posY  = 1.0f;
    float posZ  = 1.0f;
    int scale   = MaxScale - 1;

    float scaleExp2 = 0.5f;

    if (1.5f*dTx - bTx > minT) idx ^= 1, posX = 1.5f;
    if (1.5f*dTy - bTy > minT) idx ^= 2, posY = 1.5f;
    if (1.5f*dTz - bTz > minT) idx ^= 4, posZ = 1.5f;

    while (scale < MaxScale) {
        if (current == 0)
            current = _octree[parent];

        float cornerTX = posX*dTx - bTx;
        float cornerTY = posY*dTy - bTy;
        float cornerTZ = posZ*dTz - bTz;
        float maxTC = std::min(cornerTX, std::min(cornerTY, cornerTZ));

        int childShift = idx ^ octantMask;
        uint32 childMasks = current << childShift;

        if ((childMasks & 0x8000) && minT <= maxT) { // if childExists and ray not arrived end
            if (maxTC*rayScale >= scaleExp2) {
                t = maxTC;
                return true;
            }

            float maxTV = std::min(maxT, maxTC);
            float half = scaleExp2*0.5f;
            float centerTX = half*dTx + cornerTX;
            float centerTY = half*dTy + cornerTY;
            float centerTZ = half*dTz + cornerTZ;

            if (minT <= maxTV) {
                uint64 childOffset = current >> 18; // ignore first 18 bits = 8+8 masks + hasLargeChildren + parent hasLargeChildren (so I've large sibling)
                                                    // this is used as "pointer" as index of an array of childdescriptors
                if (current & 0x20000) // 18th // means parent hasLargeChildren // so current has large siblings
                    childOffset = (childOffset << 32) | uint64(_octree[parent + 1]); // recreate old offset  

                if (!(childMasks & 0x80)) { // if leaf
                  // TODO work here
                  // TODO CRITICAL
                  // Need to retrieve *world position* to check world points against SDF
                  //
                  // launch sphere tracing and calc t and normal
                  // not necessarily break
                  // maybe local slow down leads to a miss

                  bool hit = pureSphereTracing(
                      orig + minT*dir,
                      dir,
                      rayScale,
                      normal,t // out
                      //,maxTV
                      );

                  if (hit) {
                    break;
                  } else {
                    //break; // TODO to show gross depth
                    //return false; // TODO tmp solution

                    //if miss force ADVANCE
                    minT = maxT + 0.000001f;
                    continue;
                  }
                }

                rayStack[scale].offset = parent;
                rayStack[scale].maxT = maxT;

                uint32 siblingCount = BitCount[childMasks & 127]; // use non-leaf mask
                parent += childOffset + siblingCount;
                if (current & 0x10000) // 17th // hasLargeChildren so child occupy x2 space because of insertions
                    parent += siblingCount;

                idx = 0;
                scale--;
                scaleExp2 = half;

                // select child and "pull near the voxel opposite corner" because ray travels positive->negative and the corner is the smallest
                if (centerTX > minT) idx ^= 1, posX += scaleExp2;
                if (centerTY > minT) idx ^= 2, posY += scaleExp2;
                if (centerTZ > minT) idx ^= 4, posZ += scaleExp2;

                maxT = maxTV;
                current = 0; // reset current to force fetch on next loop

                continue;
            }
        }
        // ADVANCE
        int stepMask = 0;
        if (cornerTX <= maxTC) stepMask ^= 1, posX -= scaleExp2; // "push away the voxel opposite corner" because ray travels positive->negative
        if (cornerTY <= maxTC) stepMask ^= 2, posY -= scaleExp2; // and idx will become smaller and smaller
        if (cornerTZ <= maxTC) stepMask ^= 4, posZ -= scaleExp2;

        minT = maxTC; // move "ray head"
        idx ^= stepMask;
        // POP
        if ((idx & stepMask) != 0) {
            int differingBits = 0;
            if (stepMask & 1) differingBits |= floatBitsToUint(posX) ^ floatBitsToUint(posX + scaleExp2);
            if (stepMask & 2) differingBits |= floatBitsToUint(posY) ^ floatBitsToUint(posY + scaleExp2);
            if (stepMask & 4) differingBits |= floatBitsToUint(posZ) ^ floatBitsToUint(posZ + scaleExp2);
            scale = (floatBitsToUint((float)differingBits) >> 23) - 127; // highest differing bit
            scaleExp2 = uintBitsToFloat((scale - MaxScale + 127) << 23);

            parent = rayStack[scale].offset;
            maxT   = rayStack[scale].maxT;

            int shX = floatBitsToUint(posX) >> scale;
            int shY = floatBitsToUint(posY) >> scale;
            int shZ = floatBitsToUint(posZ) >> scale;
            posX = uintBitsToFloat(shX << scale);
            posY = uintBitsToFloat(shY << scale);
            posZ = uintBitsToFloat(shZ << scale);
            idx = (shX & 1) | ((shY & 1) << 1) | ((shZ & 1) << 2);

            current = 0;
        }
    }

    if (scale >= MaxScale)
        return false;

    t = minT; // we arrive here with the while break after reaching a leaf
    return true;
}

bool VoxelOctree::pureSphereTracing(const Vec3 &orig, const Vec3 &dir, float rayScale, uint32 &normal, float &t, float max_distance) {
  Vec3 worig = unorm_coord(
      orig,
      Vec3(-2.f,-2.f,-2.f), // TODO parametrize
      Vec3(4.f,4.f,4.f)     // TODO parametrize
      );

  float ti=0.0f;
  float minDistance = infinity;
  float dist = infinity;
  bool hit = false;
  Vec3 p = worig;
  while (ti < max_distance) {
    dist = sdf(p);

    if (dist < minDistance) {
      minDistance = dist;
    }

    // did we intersect the shape?
    if (minDistance <= hit_threshold + ti * rayScale) {
      hit = true;
      break;
    }
    ti += dist;
    p = worig + ti*dir;
  }

  if (hit) {
    normal = compressMaterial(
        normalAt(p),
        1.0
        );
  } else {
    normal = compressMaterial(
        Vec3(1,1,1).normalize(),
        1.0
        );
  }


  if (rayScale != 0) {
    //t = ti - ti*rayScale;
    t = ti - 5;
  } else {
    t = ti;
  }
  return hit;
}



float sphere(Vec3 p, Vec3 c, float r) {
  return (p - c).length() - r;
}
//float sdf(const Vec3 &p) {
//  float r = .3;
//  return sphere(p, Vec3(1,0,0), r);
//}
float sdf(const Vec3 &p) {
  // example for flat index compression
  float r = .4;
  return sphere(p, Vec3(.5,0,.5), r);
}
static constexpr float gradient_delta = 10e-5; // delta used to compute gradient (normal)
inline Vec3 normalAt(const Vec3& p) {
  return (Vec3(
        sdf(p + Vec3(gradient_delta,0,0)) - sdf(p + Vec3(-gradient_delta,0,0)),
        sdf(p + Vec3(0,gradient_delta,0)) - sdf(p + Vec3(0,-gradient_delta,0)),
        sdf(p + Vec3(0,0,gradient_delta)) - sdf(p + Vec3(0,0,-gradient_delta)))
      ).normalize();
}

static inline Vec3 unorm_coord(Vec3 nc, Vec3 p0, Vec3 region_dims) {
  return (nc - Vec3(1.0)) * region_dims + p0;
}


