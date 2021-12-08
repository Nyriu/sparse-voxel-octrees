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


#include "ThreadBarrier.hpp"
#include "VoxelOctree.hpp"
#include "PlyLoader.hpp"
#include "VoxelData.hpp"
#include "Events.hpp"
#include "Timer.hpp"
#include "Util.hpp"

#include "thread/ThreadUtils.hpp"
#include "thread/ThreadPool.hpp"

#include "math/MatrixStack.hpp"
#include "math/Vec3.hpp"
#include "math/Mat4.hpp"

#include <algorithm>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <cstring>
#include <atomic>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <SDL.h>

#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream

#ifndef M_PI
#define M_PI        3.14159265358979323846
#endif

/* Number of threads to use - adapt this to your platform for optimal results */
static const int NumThreads = 1; //16; // TODO used 1 to debug
/* Screen resolution */
static const int GWidth  = 1280;
static const int GHeight = 720;

static const float AspectRatio = GHeight/(float)GWidth;
static const int TileSize = 8;

static SDL_Surface *backBuffer;
static ThreadBarrier *barrier;

static std::atomic<bool> doTerminate;
static std::atomic<bool> renderHalfSize;

struct BatchData {
    unsigned int rmode;
    float coarseScale_factor;

    int id;
    int x0, y0;
    int x1, y1;

    int tilesX, tilesY;
    float *depthBuffer;
    VoxelOctree *tree;

    bool verbose_mouse;
    std::vector<std::pair<std::string, std::vector<int>>> *mouse_moves;
};


std::vector<std::pair<std::string, std::vector<int>>> read_csv(std::string filename){
    // Reads a CSV file into a vector of <string, vector<int>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, int vector> pairs to store the result
    std::vector<std::pair<std::string, std::vector<int>>> result;

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw std::runtime_error("Could not open file");

    // Helper vars
    std::string line, colname;
    int val;

    // Read the column names
    if(myFile.good())
    {
        // Extract the first line in the file
        std::getline(myFile, line);

        // Create a stringstream from line
        std::stringstream ss(line);

        // Extract each column name
        while(std::getline(ss, colname, ',')){
            
            // Initialize and add <colname, int vector> pairs to result
            result.push_back({colname, std::vector<int> {}});
        }
    }

    // Read data, line by line
    while(std::getline(myFile, line))
    {
        // Create a stringstream of the current line
        std::stringstream ss(line);
        
        // Keep track of the current column index
        int colIdx = 0;
        
        // Extract each integer
        while(ss >> val){
            
            // Add the current integer to the 'colIdx' column's values vector
            result.at(colIdx).second.push_back(val);
            
            // If the next token is a comma, ignore it and move on
            if(ss.peek() == ',') ss.ignore();
            
            // Increment the column index
            colIdx++;
        }
    }

    // Close file
    myFile.close();

    return result;
}


Vec3 shade(int intNormal, const Vec3 &ray, const Vec3 &light) {
    Vec3 n;
    float c;
    decompressMaterial(intNormal, n, c);

    float d = std::max(light.dot(ray.reflect(n)), 0.0f);
    float specular = d*d;

    return c*0.9f*std::abs(light.dot(n)) + specular*0.2f;
}

void renderTile(int x0, int y0, int x1, int y1, int stride, float scale, float zx, float zy, float zz,
        const Mat4 &tform, const Vec3 &light, VoxelOctree *tree, const Vec3 &pos, float minT,
        const unsigned int rmode,
        // TODO REMOVE BELOW ARGS
        float *depthBuffer,
        int idx, int tilesX
        ) {
    uint32 *buffer  = (uint32 *)backBuffer->pixels;
    int pitch       = backBuffer->pitch;

    float dy = AspectRatio - y0*scale;
    for (int y = y0; y < y1; ++y, dy -= scale) {
        float dx = -1.0f + x0*scale;
        for (int x = x0; x < x1; ++x, dx += scale) {
            int cornerX = x - ((x - x0) % stride);
            int cornerY = y - ((y - y0) % stride);
            if (cornerX != x || cornerY != y) {
                buffer[x + y*pitch/4] = buffer[cornerX + cornerY*pitch/4];
                continue;
            }

            Vec3 dir = Vec3(
                dx*tform.a11 + dy*tform.a12 + zx,
                dx*tform.a21 + dy*tform.a22 + zy,
                dx*tform.a31 + dy*tform.a32 + zz
            );
            dir *= invSqrt(dir.x*dir.x + dir.y*dir.y + dir.z*dir.z);


            Vec3 col;

            bool show_depthBuffer = true; // TODO REMOVE
            uint32 intNormal;
            float t;
            if (tree->raymarch(pos + dir*minT, dir, 0.0f, intNormal, t, rmode)) {
              col = shade(intNormal, dir, light);
            } else if (show_depthBuffer) {
              col = Vec3(
                  std::min(std::min(depthBuffer[idx], depthBuffer[idx - 1]),
                    std::min(depthBuffer[idx - tilesX],
                      depthBuffer[idx - tilesX - 1]))
                  ,
                  0,0);
            }

#ifdef __APPLE__
            uint32 color =
                 uint32(std::min(col.x, 1.0f)*255.0) <<  8   |
                (uint32(std::min(col.y, 1.0f)*255.0) <<  16) |
                (uint32(std::min(col.z, 1.0f)*255.0) <<  24) |
                0x000000FFu;
#else
            uint32 color =
                 uint32(std::min(col.x, 1.0f)*255.0)        |
                (uint32(std::min(col.y, 1.0f)*255.0) <<  8) |
                (uint32(std::min(col.z, 1.0f)*255.0) << 16) |
                0xFF000000u;
#endif
            buffer[x + y*pitch/4] = color;
        }
    }
}

void renderBatch(BatchData *data) {
    const float TreeMiss = 1e10;

    int x0 = data->x0, y0 = data->y0;
    int x1 = data->x1, y1 = data->y1;
    int tilesX = data->tilesX;
    int tilesY = data->tilesY;
    float *depthBuffer = data->depthBuffer;
    VoxelOctree *tree = data->tree;

    Mat4 tform;
    MatrixStack::get(INV_MODELVIEW_STACK, tform);

    Vec3 pos = tform*Vec3() + tree->center() + Vec3(1.0); // capire il tform

    tform.a14 = tform.a24 = tform.a34 = 0.0f;

    float scale = 2.0f/GWidth;
    float tileScale = TileSize*scale;
    float planeDist = 1.0f/std::tan(float(M_PI)/6.0f);
    float zx = planeDist*tform.a13, zy = planeDist*tform.a23, zz = planeDist*tform.a33;
    float coarseScale = 2.0f*TileSize/(planeDist*GHeight);
    coarseScale *= data->coarseScale_factor;

    int stride = renderHalfSize ? 3 : 1;

    Vec3 light = (tform*Vec3(-1.0, 1.0, -1.0)).normalize();

    std::memset((uint8 *)backBuffer->pixels + y0*backBuffer->pitch, 0, (y1 - y0)*backBuffer->pitch);

    float dy = AspectRatio - y0*scale;
    for (int y = 0, idx = 0; y < tilesY; y++, dy -= tileScale) {
        float dx = -1.0f + x0*scale;
        for (int x = 0; x < tilesX; x++, dx += tileScale, idx++) {
            Vec3 dir = Vec3(
                dx*tform.a11 + dy*tform.a12 + zx,
                dx*tform.a21 + dy*tform.a22 + zy,
                dx*tform.a31 + dy*tform.a32 + zz
            );
            dir *= invSqrt(dir.x*dir.x + dir.y*dir.y + dir.z*dir.z);

            uint32 intNormal;
            float t;
            Vec3 col;
            if (tree->raymarch(pos, dir, coarseScale, intNormal, t, data->rmode))
                depthBuffer[idx] = t;
            else
                depthBuffer[idx] = TreeMiss;

            if (x > 0 && y > 0) {
                float minT = std::min(std::min(depthBuffer[idx], depthBuffer[idx - 1]),
                    std::min(depthBuffer[idx - tilesX],
                    depthBuffer[idx - tilesX - 1]));

                if (minT != TreeMiss) {
                    int tx0 = (x - 1)*TileSize + x0;
                    int ty0 = (y - 1)*TileSize + y0;
                    int tx1 = std::min(tx0 + TileSize, x1);
                    int ty1 = std::min(ty0 + TileSize, y1);
                    renderTile(tx0, ty0, tx1, ty1, stride, scale, zx, zy, zz, tform, light, tree, pos,
                            std::max(minT - 0.03f, 0.0f),
                            data->rmode,
                            // TODO REMOVE BELOW ARGS
                            depthBuffer,
                            idx, tilesX
                            );
                }
            }
        }
    }
}

int renderLoop_interactive(void *threadData) {
    BatchData *data = (BatchData *)threadData;

    float cS_factor = 1.f;
    switch (data->rmode) {
      case 0: // Voxel
        break;
      case 1: // Voxel + SphereTracing
        cS_factor = 0.1f;
        break;
      case 2: // SphereTracing
        cS_factor = 0.1f;
        break;
    }
    float planeDist = 1.0f/std::tan(float(M_PI)/6.0f);
    float coarseScale = 2.0f*TileSize/(planeDist*GHeight);
    coarseScale *= cS_factor;
    data->coarseScale_factor = cS_factor;

    std::cout <<
      "coarseScale " << coarseScale <<
      "\ncoarseScale_factor " << cS_factor <<
      std::endl;

    float radius = 1.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    if (data->id == 0) {
        MatrixStack::set(VIEW_STACK, Mat4::translate(Vec3(0.0f, 0.0f, -radius)));
        MatrixStack::set(MODEL_STACK, Mat4());
    }

    Timer timer;
    while (!doTerminate) {

        timer.start();
        barrier->waitPre();
        renderBatch(data);
        barrier->waitPost();
        //timer.bench("\nrenderBatch");
        timer.bench("renderBatch");

        if (data->id == 0) {
            if (SDL_MUSTLOCK(backBuffer))
                SDL_UnlockSurface(backBuffer);

            SDL_UpdateRect(backBuffer, 0, 0, 0, 0);

            int event;
            while ((event = waitEvent()) && (event == SDL_MOUSEMOTION && !getMouseDown(0) && !getMouseDown(1)));


            if (getKeyDown(SDLK_ESCAPE)) {
                doTerminate = true;
                barrier->releaseAll();
            }

            float mx = float(getMouseXSpeed());
            float my = float(getMouseYSpeed());

            if (data->verbose_mouse) {
              std::cout <<
                "SDL_event " << event <<
                "\nmx " << mx <<
                "\nmy " << my <<
                "\ngetMouseDown(0) " << getMouseDown(0) <<
                "\ngetMouseDown(1) " << getMouseDown(1) <<
                std::endl;
            }

            if (getMouseDown(0) && (mx != 0 || my != 0)) {
              pitch = std::fmod(pitch - my, 360.0f);
              yaw = std::fmod(yaw + (std::fabs(pitch) > 90.0f ? mx : -mx), 360.0f);

              if (pitch >  180.0f) pitch -= 360.0f;
              else if (pitch < -180.0f) pitch += 360.0f;

              MatrixStack::set(MODEL_STACK, Mat4::rotXYZ(Vec3(pitch, 0.0f, 0.0f))*
                  Mat4::rotXYZ(Vec3(0.0f, yaw, 0.0f)));
              renderHalfSize = true;
            } else if (getMouseDown(1) && my != 0) {
              radius *= std::min(std::max(1.0f - my*0.01f, 0.5f), 1.5f);
              radius = std::min(radius, 25.0f);
              MatrixStack::set(VIEW_STACK, Mat4::translate(Vec3(0.0f, 0.0f, -radius)));
              renderHalfSize = true;
            } else {
              renderHalfSize = false;
            }

            if (SDL_MUSTLOCK(backBuffer))
                SDL_LockSurface(backBuffer);
        }
    }

    return 0;
}


int renderLoop_recorded(void *threadData) {
    BatchData *data = (BatchData *)threadData;

    float cS_factor = 1.f;
    switch (data->rmode) {
      case 0: // Voxel
        break;
      case 1: // Voxel + SphereTracing
        cS_factor = 0.1f;
        break;
      case 2: // SphereTracing
        cS_factor = 0.1f;
        break;
    }
    float planeDist = 1.0f/std::tan(float(M_PI)/6.0f);
    float coarseScale = 2.0f*TileSize/(planeDist*GHeight);
    coarseScale *= cS_factor;
    data->coarseScale_factor = cS_factor;

    std::cout <<
      "coarseScale " << coarseScale <<
      "\ncoarseScale_factor " << cS_factor <<
      std::endl;

    float radius = 1.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    if (data->id == 0) {
        MatrixStack::set(VIEW_STACK, Mat4::translate(Vec3(0.0f, 0.0f, -radius)));
        MatrixStack::set(MODEL_STACK, Mat4());
    }
    
    unsigned int current_mouse_move = 0;

    Timer timer;
    while (!doTerminate) {

        timer.start();
        barrier->waitPre();
        renderBatch(data);
        barrier->waitPost();
        //timer.bench("\nrenderBatch");
        timer.bench("renderBatch");

        if (data->id == 0) {
          if (SDL_MUSTLOCK(backBuffer)) {
            SDL_UnlockSurface(backBuffer);
          }
          SDL_UpdateRect(backBuffer, 0, 0, 0, 0);


          current_mouse_move++;
          if (current_mouse_move >= (*data->mouse_moves)[0].second.size()) {
            doTerminate = true;
            barrier->releaseAll();
          }

          int   event          =   int((*data->mouse_moves)[0].second[current_mouse_move]);
          float mx             = float((*data->mouse_moves)[1].second[current_mouse_move]);
          float my             = float((*data->mouse_moves)[2].second[current_mouse_move]);
          int   getMouseDown_0 =   int((*data->mouse_moves)[2].second[current_mouse_move]);
          int   getMouseDown_1 =   int((*data->mouse_moves)[2].second[current_mouse_move]);

          if (getMouseDown_0 && (mx != 0 || my != 0)) {
            pitch = std::fmod(pitch - my, 360.0f);
            yaw = std::fmod(yaw + (std::fabs(pitch) > 90.0f ? mx : -mx), 360.0f);

            if (pitch >  180.0f) pitch -= 360.0f;
            else if (pitch < -180.0f) pitch += 360.0f;

            MatrixStack::set(MODEL_STACK, Mat4::rotXYZ(Vec3(pitch, 0.0f, 0.0f))*
                Mat4::rotXYZ(Vec3(0.0f, yaw, 0.0f)));
            renderHalfSize = true;
          } else if (getMouseDown_1 && my != 0) {
            radius *= std::min(std::max(1.0f - my*0.01f, 0.5f), 1.5f);
            radius = std::min(radius, 25.0f);
            MatrixStack::set(VIEW_STACK, Mat4::translate(Vec3(0.0f, 0.0f, -radius)));
            renderHalfSize = true;
          } else {
            renderHalfSize = false;
          }

          if (SDL_MUSTLOCK(backBuffer))
              SDL_LockSurface(backBuffer);
        }
    }

    return 0;
}

/* Maximum allowed memory allocation sizes for lookup table and cache blocks.
 * Larger => faster conversion usually, but adapt this to your own RAM size.
 * The conversion will still succeed with memory sizes much, much smaller than
 * the size of the voxel data, only slower.
 */
static const size_t dataMemory = int64_t(1024)*1024*1024;

void printHelp() {
    std::cout << "Usage: benchmark [options] filename ..." << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "-viewer               set program to SVO rendering mode." << std::endl;
    std::cout << "  --mode <m>          set tracing algorithm, m equals 0 or 1, where 0 indicates RENDER VOXELS, 1 indicates RENDER VOXEL+SDF, and 2 RENDER ONLY SDF." << std::endl;
    std::cout << "-mouse-log filename   use a recorded mouse input." << std::endl;
    std::cout << "-verbose-mouse        print mouse infos." << std::endl;
    //std::cout << "-record-mouse filename            record mouse into file." << std::endl; // TODO
    //std::cout << "-headless             do not open any window." << std::endl; // TODO
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  benchmark -viewer --mode 0 ../models/XYZRGB-Dragon.oct" << std::endl;
    std::cout << "  benchmark -viewer --mode 1 ../models/XYZRGB-Dragon.oct" << std::endl;
    std::cout << "  benchmark -viewer --mode 2 ../models/XYZRGB-Dragon.oct" << std::endl;
    std::cout << "  benchmark -viewer --mode 2 -mouse-log ../benchmark/mouse_log/20211209_112129_mouse.csv ../models/XYZRGB-Dragon.oct" << std::endl;
    std::cout << "  benchmark -viewer --mode 2 -verbose-mouse ../models/XYZRGB-Dragon.oct" << std::endl;
    std::cout << std::endl << std::endl;
}


int main(int argc, char *argv[]) {
    unsigned int rmode = 0;         //render mode // default voxel only
    std::string mouseInputFile = "";
    bool verboseMouse = false;
    std::string inputFile = "";
    std::string outputFile = "";
    
    /* parse arguments */
    if ((argc == 5) && (std::string(argv[1]) == "-viewer")) {
      rmode = atoi(argv[3]);
      inputFile = argv[4];
    } else if ((argc == 6) &&
        (std::string(argv[1]) == "-viewer") &&
        (std::string(argv[4]) == "-verbose-mouse")
        ) {
      rmode = atoi(argv[3]);
      verboseMouse = true;
      inputFile = argv[5];
    } else if ((argc == 7) &&
        (std::string(argv[1]) == "-viewer") &&
        (std::string(argv[4]) == "-mouse-log")
        ) {
      rmode = atoi(argv[3]);
      mouseInputFile = argv[5];
      inputFile = argv[6];
    }

    else {
      std::cout << "Invalid arguments! Please refer to the help info!" << std::endl;
      printHelp();
      return 0;
    }

    if (rmode != 0 && rmode != 1 && rmode != 2) {
      std::cout << "Invalid arguments! Please refer to the help info!" << std::endl;
      printHelp();
      return 0;
    }



    Timer timer;
    
    if (std::string(argv[1]) == "-viewer")  {
        
        std::unique_ptr<VoxelOctree> tree(new VoxelOctree(inputFile.c_str()));

        timer.bench("octree_init");
        std::cout <<
          "view_mode " << rmode // rendering mode
          << std::endl;


        SDL_Init(SDL_INIT_VIDEO);

        SDL_WM_SetCaption("Sparse Voxel Octrees", "Sparse Voxel Octrees");
        backBuffer = SDL_SetVideoMode(GWidth, GHeight, 32, SDL_SWSURFACE);

        SDL_Thread *threads[NumThreads - 1];
        BatchData threadData[NumThreads];

        barrier = new ThreadBarrier(NumThreads);
        doTerminate = false;

        if (SDL_MUSTLOCK(backBuffer))
            SDL_LockSurface(backBuffer);

        int stride = (GHeight - 1) / NumThreads + 1;
        for (int i = 0; i < NumThreads; i++) {
            threadData[i].rmode = rmode;
            threadData[i].id = i;
            threadData[i].tree = tree.get();
            threadData[i].x0 = 0;
            threadData[i].x1 = GWidth;
            threadData[i].y0 = i*stride;
            threadData[i].y1 = std::min((i + 1)*stride, GHeight);
            threadData[i].tilesX = (threadData[i].x1 - threadData[i].x0 - 1) / TileSize + 2;
            threadData[i].tilesY = (threadData[i].y1 - threadData[i].y0 - 1) / TileSize + 2;
            threadData[i].depthBuffer = new float[threadData[i].tilesX*threadData[i].tilesY];

            threadData[i].mouse_moves = nullptr;
            threadData[i].verbose_mouse = verboseMouse;
        }


        if (mouseInputFile == "") {
          for (int i = 1; i < NumThreads; i++)
            threads[i - 1] = SDL_CreateThread(&renderLoop_interactive, (void *)&threadData[i]);

          renderLoop_interactive((void *)&threadData[0]);

        } else {
          std::vector<std::pair<std::string, std::vector<int>>> mouse_moves;
          mouse_moves = read_csv(mouseInputFile);
          threadData[0].mouse_moves = &mouse_moves;

          for (int i = 1; i < NumThreads; i++)
            threads[i - 1] = SDL_CreateThread(&renderLoop_recorded, (void *)&threadData[i]);

          renderLoop_recorded((void *)&threadData[0]);
        }

        for (int i = 1; i < NumThreads; i++)
            SDL_WaitThread(threads[i - 1], 0);

        if (SDL_MUSTLOCK(backBuffer))
            SDL_UnlockSurface(backBuffer);

        SDL_Quit();
    }

    return 0;
}
