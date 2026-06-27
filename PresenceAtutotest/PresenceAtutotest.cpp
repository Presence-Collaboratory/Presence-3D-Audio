/*
====================================================================================================
  Presence Audio SDK - Automated Test Suite
  High-Performance Real-time Audio Path Tracing & EAX Simulation Library
====================================================================================================

  Copyright (c) 2026 Presence Collaboratory, NSDeathman & Gemini 3 & DeepSeek

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
#include <string>

#include "TestUtils.h"
#include "../PresenceAudioSDK/Include/PresenceSystem.h"

using namespace Presence;

// =================================================================================================
// SECTION 1: TEST SCENE GEOMETRY PROVIDERS
// -------------------------------------------------------------------------------------------------
// Each class implements IGeometryProvider and supplies a simple plane-based geometry
// that the audio system traces against. Plane normals point inward.
// =================================================================================================

/**
 * @brief Closed rectangular room 10x10x4 m.
 *
 * Materials: floor Stone, ceiling Wood, walls Metal/Stone.
 * Listener position moves from center toward +X wall (metal).
 * Expected: high enclosedness, long RT60, strong reflections.
 */
class BoxRoomProvider : public IGeometryProvider
{
public:
    BoxRoomProvider()
    {
        walls.push_back({ {0, 1, 0},  0.0f, MaterialType::Stone });   // floor   y = 0
        walls.push_back({ {0, -1, 0}, 4.0f, MaterialType::Wood });    // ceiling y = 4
        walls.push_back({ {0, 0, -1}, 5.0f, MaterialType::Stone });   // z = 5
        walls.push_back({ {0, 0, 1},  5.0f, MaterialType::Stone });   // z = -5
        walls.push_back({ {-1, 0, 0}, 5.0f, MaterialType::Metal });   // x = 5
        walls.push_back({ {1, 0, 0},  5.0f, MaterialType::Metal });   // x = -5
    }

    RayHit CastRay(const float3& start, const float3& dir, float maxDist) override
    {
        return IntersectPlanes(start, dir, maxDist);
    }

private:
    struct Plane
    {
        float3 normal;
        float  distance;   // plane equation: dot(n, p) + distance = 0
        MaterialType mat;
    };
    std::vector<Plane> walls;

    RayHit IntersectPlanes(const float3& start, const float3& dir, float maxDist) const
    {
        RayHit closestHit;
        closestHit.isHit = false;
        closestHit.distance = maxDist;

        for (const auto& plane : walls)
        {
            float denom = plane.normal.dot(dir);
            if (denom < -1e-6f)   // ray front‑facing the plane
            {
                float t = -(plane.normal.dot(start) + plane.distance) / denom;
                if (t > 0.001f && t < closestHit.distance)
                {
                    closestHit.distance = t;
                    closestHit.isHit = true;
                    closestHit.normal = plane.normal;
                    closestHit.materialID = static_cast<int>(plane.mat);
                }
            }
        }
        return closestHit;
    }
};

/**
 * @brief Open outdoor space with only a soft ground plane at y = 0.
 *
 * Expected: openness ~0.5, Env Size large (limited to 50000 m³), RT60 short.
 */
class OpenSpaceProvider : public IGeometryProvider
{
public:
    OpenSpaceProvider()
    {
        walls.push_back({ {0, 1, 0}, 0.0f, MaterialType::Soft });   // ground
    }

    RayHit CastRay(const float3& start, const float3& dir, float maxDist) override
    {
        return IntersectPlanes(start, dir, maxDist);
    }

private:
    struct Plane { float3 normal; float distance; MaterialType mat; };
    std::vector<Plane> walls;

    RayHit IntersectPlanes(const float3& start, const float3& dir, float maxDist) const
    {
        RayHit closestHit{ false, maxDist, {}, 0 };
        for (const auto& plane : walls)
        {
            float denom = plane.normal.dot(dir);
            if (denom < -1e-6f)
            {
                float t = -(plane.normal.dot(start) + plane.distance) / denom;
                if (t > 0.001f && t < closestHit.distance)
                {
                    closestHit.distance = t;
                    closestHit.isHit = true;
                    closestHit.normal = plane.normal;
                    closestHit.materialID = static_cast<int>(plane.mat);
                }
            }
        }
        return closestHit;
    }
};

/**
 * @brief Long narrow corridor (2x3x40 m) extending along the Z axis.
 *
 * Expected: fully enclosed, MFP ~2 m, Volume ~70 m³, RT60 decreases as listener moves.
 */
class CorridorProvider : public IGeometryProvider
{
public:
    CorridorProvider()
    {
        // Floor and ceiling
        walls.push_back({ {0, 1, 0},  0.0f,  MaterialType::Stone });   // y = 0
        walls.push_back({ {0, -1, 0}, 3.0f,  MaterialType::Wood });    // y = 3
        // End walls
        walls.push_back({ {0, 0, -1}, 20.0f, MaterialType::Stone });   // z = 20
        walls.push_back({ {0, 0, 1},  20.0f, MaterialType::Stone });   // z = -20
        // Side walls
        walls.push_back({ {-1, 0, 0}, 1.0f,  MaterialType::Metal });   // x = 1
        walls.push_back({ {1, 0, 0},  1.0f,  MaterialType::Metal });   // x = -1
    }

    RayHit CastRay(const float3& start, const float3& dir, float maxDist) override
    {
        return IntersectPlanes(start, dir, maxDist);
    }

private:
    struct Plane { float3 normal; float distance; MaterialType mat; };
    std::vector<Plane> walls;

    RayHit IntersectPlanes(const float3& start, const float3& dir, float maxDist) const
    {
        RayHit closestHit{ false, maxDist, {}, 0 };
        for (const auto& plane : walls)
        {
            float denom = plane.normal.dot(dir);
            if (denom < -1e-6f)
            {
                float t = -(plane.normal.dot(start) + plane.distance) / denom;
                if (t > 0.001f && t < closestHit.distance)
                {
                    closestHit.distance = t;
                    closestHit.isHit = true;
                    closestHit.normal = plane.normal;
                    closestHit.materialID = static_cast<int>(plane.mat);
                }
            }
        }
        return closestHit;
    }
};

/**
 * @brief Convex pentagonal room (height 4 m).
 *
 * Floor Stone, ceiling Wood, five vertical Stone walls.
 * Listener stationary at the center.
 * Expected: consistent parameters, no drift.
 */
class ComplexRoomProvider : public IGeometryProvider
{
public:
    ComplexRoomProvider()
    {
        // Floor and ceiling
        walls.push_back({ {0, 1, 0},  0.0f, MaterialType::Stone });   // y = 0
        walls.push_back({ {0, -1, 0}, 4.0f, MaterialType::Wood });    // y = 4

        // Pentagonal vertical walls in counter‑clockwise order
        AddVerticalWall({ 5, 0 }, { 2, 4 });   // A
        AddVerticalWall({ 2, 4 }, { -2, 4 });  // B
        AddVerticalWall({ -2, 4 }, { -5, 0 });  // C
        AddVerticalWall({ -5, 0 }, { -2, -4 }); // D
        AddVerticalWall({ -2, -4 }, { 5, 0 });  // E
    }

    RayHit CastRay(const float3& start, const float3& dir, float maxDist) override
    {
        return IntersectPlanes(start, dir, maxDist);
    }

private:
    struct Plane { float3 normal; float distance; MaterialType mat; };
    std::vector<Plane> walls;

    void AddVerticalWall(const float2& p1, const float2& p2)
    {
        float2 edge(p2.x - p1.x, p2.y - p1.y);        // (p2.y is z coordinate)
        float2 inward(-edge.y, edge.x);                // rotate -90° for CCW polygon
        float len = std::sqrt(inward.x * inward.x + inward.y * inward.y);
        inward.x /= len;
        inward.y /= len;

        float3 normal(inward.x, 0.0f, inward.y);
        float d = -(normal.x * p1.x + normal.z * p1.y);
        walls.push_back({ normal, d, MaterialType::Stone });
    }

    RayHit IntersectPlanes(const float3& start, const float3& dir, float maxDist) const
    {
        RayHit closestHit{ false, maxDist, {}, 0 };
        for (const auto& plane : walls)
        {
            float denom = plane.normal.dot(dir);
            if (denom < -1e-6f)
            {
                float t = -(plane.normal.dot(start) + plane.distance) / denom;
                if (t > 0.001f && t < closestHit.distance)
                {
                    closestHit.distance = t;
                    closestHit.isHit = true;
                    closestHit.normal = plane.normal;
                    closestHit.materialID = static_cast<int>(plane.mat);
                }
            }
        }
        return closestHit;
    }
};

// =================================================================================================
// SECTION 2: SCENARIO EXECUTION HELPER
// -------------------------------------------------------------------------------------------------
// Sets up the audio system, runs the simulation frame by frame,
// and logs the EAX state every 10 frames.
// =================================================================================================

static void RunScenario(const std::string& name,
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

    Logger::Log("Initializing AudioSystem (" + std::string(AudioSystem::GetVersionString()) + ")...");
    audioSystem.Initialize(geometry, settings);

    float3 listenerPos = startPos;
    bool dataValidOnce = false;

    for (int frame = 0; frame < totalFrames; ++frame)
    {
        listenerPos.x += movePerFrame.x;
        listenerPos.y += movePerFrame.y;
        listenerPos.z += movePerFrame.z;

        audioSystem.Update(listenerPos, dt);
        EAXResult res = audioSystem.GetEAXResult();

        if (res.isValid)
            dataValidOnce = true;

        // Log every 10th frame to keep output manageable
        if (frame % 10 == 0)
        {
            std::string status = res.isValid ? "[VALID]" : "[WAITING]";
            Logger::Log("Frame " + std::to_string(frame) + " " + status +
                " | Pos: " + listenerPos.to_string());

            if (res.isValid)
            {
                Logger::LogParam("Room Level (mB)", static_cast<float>(res.lRoom));
                Logger::LogParam("Room HF (mB)", static_cast<float>(res.lRoomHF));
                Logger::LogParam("Room Rolloff", res.flRoomRolloffFactor);
                Logger::LogParam("Decay Time (s)", res.flDecayTime);
                Logger::LogParam("Decay HF Ratio", res.flDecayHFRatio);
                Logger::LogParam("Reflections (mB)", static_cast<float>(res.lReflections));
                Logger::LogParam("Refl Delay (s)", res.flReflectionsDelay);
                Logger::LogParam("Reverb (mB)", static_cast<float>(res.lReverb));
                Logger::LogParam("Reverb Delay (s)", res.flReverbDelay);
                Logger::LogParam("Env Size (m)", res.flEnvironmentSize);
                Logger::LogParam("Env Diffusion", res.flEnvironmentDiffusion);
                Logger::LogParam("Air Absorb HF", res.flAirAbsorptionHF);
                Logger::LogParam("Debug Enclosed", res.debugEnclosedness);
                Logger::LogParam("Debug Openness", res.debugOpenness);
                Logger::LogParam("Debug MFP (m)", res.debugMeanFreePath);
                Logger::LogParam("Debug Volume (m^3)", res.debugPhysicalVolume);
            }
        }

        // Simulate real‑time pacing (30 fps)
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }

    if (!dataValidOnce)
        Logger::Log("SCENARIO " + name + " FAILED: No valid EAX data received!", true);
    else
        Logger::Log("SCENARIO " + name + " PASSED: Valid data stream received.");

    audioSystem.Shutdown();
    Logger::Log("=== Scenario " + name + " finished ===\n");
}

// =================================================================================================
// SECTION 3: MAIN ENTRY POINT
// -------------------------------------------------------------------------------------------------
// Instantiates test geometry, runs each scenario, and reports overall success/failure.
// =================================================================================================

int main()
{
    Logger::Init("PresenceAutotest_Log.txt");
    Logger::Log("PresenceAutotest v1.0 (multi‑scenario) started");

    try
    {
        // ---------- Scenario 1: ClosedBoxRoom ----------
        {
            BoxRoomProvider room;
            RunScenario("ClosedRoom",
                &room,
                float3(0.0f, 2.0f, 0.0f),      // start at centre
                float3(0.05f, 0.0f, 0.0f),      // walk slowly toward +X (metal wall)
                60,                              // 2 seconds total
                0.033f);
        }

        // ---------- Scenario 2: OpenSpace ----------
        {
            OpenSpaceProvider open;
            RunScenario("OpenSpace",
                &open,
                float3(0.1f, 1.7f, 0.05f),      // slightly above ground
                float3(0.1f, 0.0f, 0.05f),      // move diagonally
                60,
                0.033f);
        }

        // ---------- Scenario 3: LongCorridor ----------
        {
            CorridorProvider corridor;
            RunScenario("LongCorridor",
                &corridor,
                float3(0.0f, 1.5f, -18.0f),     // near the far end
                float3(0.0f, 0.0f, 0.3f),       // walk along Z+
                60,
                0.033f);
        }

        // ---------- Scenario 4: ComplexPentagonalRoom ----------
        {
            ComplexRoomProvider complex;
            RunScenario("ComplexRoom",
                &complex,
                float3(0.0f, 2.0f, 0.0f),       // centre
                float3(0.0f, 0.0f, 0.0f),       // stationary listener
                60,
                0.033f);
        }
    }
    catch (const std::exception& e)
    {
        Logger::Log(std::string("EXCEPTION: ") + e.what(), true);
        return -1;
    }
    catch (...)
    {
        Logger::Log("UNKNOWN EXCEPTION OCCURRED", true);
        return -1;
    }

    Logger::Log("All scenarios completed. Press Enter to exit...");
    std::cin.get();
    return 0;
}
