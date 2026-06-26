/*
====================================================================================================
  Presence Audio SDK
  High-Performance Real-time Audio Path Tracing & EAX Simulation Library
====================================================================================================

  Copyright (c) 2026 Presence Collaboratory, NSDeathman & Gemini 3

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  1. The above copyright notice and this permission notice shall be included in all
     copies or substantial portions of the Software.

  2. Any project (commercial, free, open-source, or closed-source) using this Software
     must include attribution to "Presence Audio SDK by Presence Collaboratory" in its
     documentation, credits, or about screen.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

====================================================================================================
  Developed by: NSDeathman (Architecture & Core), Gemini 3 (Optimization & Math)
  Organization: Presence Collaboratory
====================================================================================================
*/
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <memory>

#include "TestUtils.h"
#include "../PresenceAudioSDK/Include/PresenceSystem.h"

using namespace Presence;

// ------------------------------------------------------------------
// Geometry providers for different scenarios
// ------------------------------------------------------------------

// 1. Closed box room (original scenario, kept for comparison)
class BoxRoomProvider : public IGeometryProvider {
public:
    BoxRoomProvider() {
        // Room 10x10x4 m, center at (0,0,0)
        walls.push_back({ {0, 1, 0},  0.0f, MaterialType::Stone }); // floor y=0
        walls.push_back({ {0, -1, 0}, 4.0f, MaterialType::Wood });  // ceiling y=4
        walls.push_back({ {0, 0, -1}, 5.0f, MaterialType::Stone }); // z=5
        walls.push_back({ {0, 0, 1},  5.0f, MaterialType::Stone }); // z=-5
        walls.push_back({ {-1, 0, 0}, 5.0f, MaterialType::Metal }); // x=5
        walls.push_back({ {1, 0, 0},  5.0f, MaterialType::Metal }); // x=-5
    }

    RayHit CastRay(const float3& start, const float3& dir, float maxDist) override {
        RayHit closestHit;
        closestHit.isHit = false;
        closestHit.distance = maxDist;

        for (const auto& plane : walls) {
            float denom = plane.normal.dot(dir);
            if (denom < -1e-6f) {
                float t = -(plane.normal.dot(start) + plane.distance) / denom;
                if (t > 0.001f && t < closestHit.distance) {
                    closestHit.distance = t;
                    closestHit.isHit = true;
                    closestHit.normal = plane.normal;
                    closestHit.materialID = static_cast<int>(plane.mat);
                }
            }
        }
        return closestHit;
    }

private:
    struct Plane {
        float3 normal;
        float  distance;
        MaterialType mat;
    };
    std::vector<Plane> walls;
};

// 2. Open space – only a ground plane (large outdoor area)
class OpenSpaceProvider : public IGeometryProvider {
public:
    OpenSpaceProvider() {
        // Ground at y=0, material Grass
        walls.push_back({ {0, 1, 0}, 0.0f, MaterialType::Soft });
    }

    RayHit CastRay(const float3& start, const float3& dir, float maxDist) override {
        RayHit closestHit;
        closestHit.isHit = false;
        closestHit.distance = maxDist;

        for (const auto& plane : walls) {
            float denom = plane.normal.dot(dir);
            if (denom < -1e-6f) {
                float t = -(plane.normal.dot(start) + plane.distance) / denom;
                if (t > 0.001f && t < closestHit.distance) {
                    closestHit.distance = t;
                    closestHit.isHit = true;
                    closestHit.normal = plane.normal;
                    closestHit.materialID = static_cast<int>(plane.mat);
                }
            }
        }
        return closestHit;
    }

private:
    struct Plane {
        float3 normal;
        float  distance;
        MaterialType mat;
    };
    std::vector<Plane> walls;
};

// 3. Long corridor – narrow and extended along the Z axis
class CorridorProvider : public IGeometryProvider {
public:
    CorridorProvider() {
        // Corridor: 2m wide (X: -1..1), 3m high (Y: 0..3), length 40m (Z: -20..20)
        walls.push_back({ {0, 1, 0},  0.0f, MaterialType::Stone }); // floor
        walls.push_back({ {0, -1, 0}, 3.0f, MaterialType::Wood });  // ceiling
        walls.push_back({ {0, 0, -1}, 20.0f, MaterialType::Stone }); // z = 20  (far end)
        walls.push_back({ {0, 0, 1},  20.0f, MaterialType::Stone }); // z = -20 (near end)
        walls.push_back({ {-1, 0, 0}, 1.0f, MaterialType::Metal });  // x = 1  (right wall)
        walls.push_back({ {1, 0, 0},  1.0f, MaterialType::Metal });  // x = -1 (left wall)
    }

    RayHit CastRay(const float3& start, const float3& dir, float maxDist) override {
        RayHit closestHit;
        closestHit.isHit = false;
        closestHit.distance = maxDist;

        for (const auto& plane : walls) {
            float denom = plane.normal.dot(dir);
            if (denom < -1e-6f) {
                float t = -(plane.normal.dot(start) + plane.distance) / denom;
                if (t > 0.001f && t < closestHit.distance) {
                    closestHit.distance = t;
                    closestHit.isHit = true;
                    closestHit.normal = plane.normal;
                    closestHit.materialID = static_cast<int>(plane.mat);
                }
            }
        }
        return closestHit;
    }

private:
    struct Plane {
        float3 normal;
        float  distance;
        MaterialType mat;
    };
    std::vector<Plane> walls;
};

// 4. Complex geometry – a convex pentagonal room
class ComplexRoomProvider : public IGeometryProvider {
public:
    ComplexRoomProvider() {
        // Floor and ceiling
        walls.push_back({ {0, 1, 0},  0.0f, MaterialType::Stone }); // floor y=0
        walls.push_back({ {0, -1, 0}, 4.0f, MaterialType::Wood });  // ceiling y=4

        // Pentagonal vertical walls (convex, vertices in counter‑clockwise order)
        // Vertices (x,z): (5,0), (2,4), (-2,4), (-5,0), (-2,-4)
        AddVerticalWall({ 5,0 }, { 2,4 });   // wall A
        AddVerticalWall({ 2,4 }, { -2,4 });  // wall B
        AddVerticalWall({ -2,4 }, { -5,0 }); // wall C
        AddVerticalWall({ -5,0 }, { -2,-4 });// wall D
        AddVerticalWall({ -2,-4 }, { 5,0 }); // wall E
    }

    RayHit CastRay(const float3& start, const float3& dir, float maxDist) override {
        RayHit closestHit;
        closestHit.isHit = false;
        closestHit.distance = maxDist;

        for (const auto& plane : walls) {
            float denom = plane.normal.dot(dir);
            if (denom < -1e-6f) {
                float t = -(plane.normal.dot(start) + plane.distance) / denom;
                if (t > 0.001f && t < closestHit.distance) {
                    closestHit.distance = t;
                    closestHit.isHit = true;
                    closestHit.normal = plane.normal;
                    closestHit.materialID = static_cast<int>(plane.mat);
                }
            }
        }
        return closestHit;
    }

private:
    struct Plane {
        float3 normal;
        float  distance;
        MaterialType mat;
    };

    void AddVerticalWall(const float2& p1, const float2& p2) {
        // Edge vector
        float2 edge = { p2.x - p1.x, p2.y - p1.y }; // using y as second coordinate (z in 3D)
        // Inward normal (rotate -90° for CCW polygon)
        float2 inNormal = { -edge.y, edge.x };
        float len = std::sqrt(inNormal.x * inNormal.x + inNormal.y * inNormal.y);
        inNormal.x /= len;
        inNormal.y /= len;

        float3 normal = { inNormal.x, 0.0f, inNormal.y };
        // Plane equation: dot(normal, P) - dot(normal, p1) = 0 → distance = -dot(normal, p1)
        float d = -(normal.x * p1.x + normal.z * p1.y); // p1.y -> z
        walls.push_back({ normal, d, MaterialType::Stone });
    }

    std::vector<Plane> walls;
};


// ------------------------------------------------------------------
// Test execution helper
// ------------------------------------------------------------------
void RunScenario(const std::string& name,
    IGeometryProvider* geometry,
    const float3& startPos,
    const float3& movePerFrame,
    int totalFrames,
    float dt)
{
    Logger::Log("=== Starting scenario: " + name + " ===");

    AudioSystem audioSystem;

    Settings settings;
    settings.maxBounces = 16;
    settings.useMultithreading = true;
    settings.maxRayDistance = 100.0f;
    settings.updateInterval = dt;

    Logger::Log("Initializing AudioSystem (" + std::string(audioSystem.GetVersionString()) + ")...");
    audioSystem.Initialize(geometry, settings);

    float3 listenerPos = startPos;
    bool dataValidOnce = false;

    for (int i = 0; i < totalFrames; ++i) {
        listenerPos.x += movePerFrame.x;
        listenerPos.y += movePerFrame.y;
        listenerPos.z += movePerFrame.z;

        audioSystem.Update(listenerPos, dt);
        EAXResult res = audioSystem.GetEAXResult();

        if (res.isValid) dataValidOnce = true;

        if (i % 10 == 0) {
            std::string status = res.isValid ? "[VALID]" : "[WAITING]";
            Logger::Log("Frame " + std::to_string(i) + " " + status +
                " | Pos: " + listenerPos.to_string());

            if (res.isValid) {
                Logger::LogParam("Room Level (mB)", (float)res.lRoom);
                Logger::LogParam("Decay Time (s)", res.flDecayTime);
                Logger::LogParam("Reflections (mB)", (float)res.lReflections);
                Logger::LogParam("Reverb (mB)", (float)res.lReverb);
                Logger::LogParam("Env Size (m)", res.flEnvironmentSize);
                Logger::LogParam("Debug Enclosed", res.debugEnclosedness);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }

    if (!dataValidOnce)
        Logger::Log("SCENARIO " + name + " FAILED: No valid EAX data!", true);
    else
        Logger::Log("SCENARIO " + name + " PASSED: Valid data stream received.");

    audioSystem.Shutdown();
    Logger::Log("=== Scenario " + name + " finished ===\n");
}


// ------------------------------------------------------------------
// Main
// ------------------------------------------------------------------
int main() {
    Logger::Init("PresenceAutotest_Log.txt");
    Logger::Log("PresenceAutotest v1.0 (multi‑scenario) started");

    try {
        // Scenario 1: Closed box room
        {
            BoxRoomProvider room;
            // Listener in center, walking toward the metal wall (+X)
            RunScenario("ClosedRoom",
                &room,
                float3(0.0f, 2.0f, 0.0f),   // start
                float3(0.05f, 0.0f, 0.0f), // movement per frame
                60,                          // frames
                0.033f);
        }

        // Scenario 2: Open space (ground only)
        {
            OpenSpaceProvider open;
            // Listener at (0, 1.7, 0), moving horizontally
            RunScenario("OpenSpace",
                &open,
                float3(0.0f, 1.7f, 0.0f),
                float3(0.1f, 0.0f, 0.05f),
                60,
                0.033f);
        }

        // Scenario 3: Long corridor
        {
            CorridorProvider corridor;
            // Listener at the beginning of the corridor, walking along Z+
            RunScenario("LongCorridor",
                &corridor,
                float3(0.0f, 1.5f, -18.0f),
                float3(0.0f, 0.0f, 0.3f),
                60,
                0.033f);
        }

        // Scenario 4: Complex pentagonal room
        {
            ComplexRoomProvider complex;
            // Listener near center, circling inside
            RunScenario("ComplexRoom",
                &complex,
                float3(0.0f, 2.0f, 0.0f),
                float3(0.0f, 0.0f, 0.0f), // we’ll update manually inside the loop?
                60,
                0.033f);
            // Note: If you want a more interesting path, you can create a custom loop,
            // but for consistency we keep the same helper. A static movement vector
            // can be set to something like (0.05, 0, 0.05) to walk diagonally.
        }

    }
    catch (const std::exception& e) {
        Logger::Log(std::string("EXCEPTION: ") + e.what(), true);
        return -1;
    }
    catch (...) {
        Logger::Log("UNKNOWN EXCEPTION OCCURRED", true);
        return -1;
    }

    Logger::Log("All scenarios completed. Press Enter to exit...");
    std::cin.get();
    return 0;
}
