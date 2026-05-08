/*
====================================================================================================
  Presence Audio SDK - Main System Interface
  High-Performance Real-time Audio Path Tracing & EAX Simulation Library
====================================================================================================

  Copyright (c) 2025 Presence Collaboratory, NSDeathman & Gemini 3

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
#include "PresenceSystem.h"

#include <vector>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <cmath>
#include <chrono>
#include <cstring>
#include <random>
#include <array>
#include <algorithm>
#include <condition_variable>
#include <unordered_map>
#include <functional>
#include <map>
#include <string>
#include <memory>
#include <stdexcept>

// Parallel Patterns Library (Windows)
#ifdef _WIN32
#include <ppl.h>
#endif

namespace Presence
{
    // =================================================================================================
    // UTILS & CONSTANTS
    // =================================================================================================
    static const float PI = 3.1415926535f;
    static const int MAX_TRACKED_MATERIALS = 64;

    // Сфера направлений (46 векторов)
    static const float3 SphereDirections[] = {
        {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
        {0.577f,0.577f,0.577f}, {0.577f,0.577f,-0.577f}, {0.577f,-0.577f,0.577f}, {0.577f,-0.577f,-0.577f},
        {-0.577f,0.577f,0.577f}, {-0.577f,0.577f,-0.577f}, {-0.577f,-0.577f,0.577f}, {-0.577f,-0.577f,-0.577f},
        {0,0.525f,0.850f}, {0,0.525f,-0.850f}, {0,-0.525f,0.850f}, {0,-0.525f,-0.850f},
        {0.850f,0,0.525f}, {0.850f,0,-0.525f}, {-0.850f,0,0.525f}, {-0.850f,0,-0.525f},
        {0.525f,0.850f,0}, {0.525f,-0.850f,0}, {-0.525f,0.850f,0}, {-0.525f,-0.850f,0}
    };
    static const int DIRECTIONS_COUNT = sizeof(SphereDirections) / sizeof(float3);

    // =================================================================================================
    // VERSION INFO
    // =================================================================================================
    float AudioSystem::GetVersion() { return 0.22f; }
    const char* AudioSystem::GetVersionString() { return "Presence Audio ver. 0.22 (Size Accuracy Fix)"; }

    // =================================================================================================
    // THREAD-SAFE RANDOM GENERATOR
    // =================================================================================================
    class ThreadSafeRandom {
    private:
        static std::mutex s_SeedMutex;
        static uint32_t s_NextSeed;
        static thread_local std::unique_ptr<std::mt19937> tls_RNG;

    public:
        static std::mt19937& Get() {
            if (!tls_RNG) {
                uint32_t seed;
                {
                    std::lock_guard<std::mutex> lock(s_SeedMutex);
                    seed = s_NextSeed++ + (uint32_t)std::chrono::high_resolution_clock::now().time_since_epoch().count();
                }
                tls_RNG = std::make_unique<std::mt19937>(seed);
            }
            return *tls_RNG;
        }

        static float3 Direction() {
            auto& rng = Get();
            std::uniform_real_distribution<float> d(-1.0f, 1.0f);
            float3 v;
            do { v = float3(d(rng), d(rng), d(rng)); } while (v.length_sq() > 1.0f || v.length_sq() < 0.001f);
            return v.normalize();
        }

        static float3 HemisphereDir(const float3& normal) {
            float3 d = Direction();
            if (d.dot(normal) < 0) d = d * -1.0f;
            return d;
        }
    };

    std::mutex ThreadSafeRandom::s_SeedMutex;
    uint32_t ThreadSafeRandom::s_NextSeed = 12345;
    thread_local std::unique_ptr<std::mt19937> ThreadSafeRandom::tls_RNG;

    // =================================================================================================
    // PROFILER
    // =================================================================================================
    class PerformanceProfiler {
        struct ThreadLocalData { uint64_t totalCalls = 0; uint64_t totalTimeNS = 0; uint64_t maxTimeNS = 0; };
        using ThreadId = std::thread::id;
        std::unordered_map<std::string, std::unordered_map<ThreadId, ThreadLocalData>> m_Data;
        std::mutex m_Mutex;
        bool m_Enabled = false;
    public:
        void SetEnabled(bool enabled) { m_Enabled = enabled; }
        bool IsEnabled() const { return m_Enabled; }
        class ScopedTimer {
            PerformanceProfiler& m_Profiler; const char* m_Name;
            std::chrono::high_resolution_clock::time_point m_Start; bool m_Active;
        public:
            ScopedTimer(const char* name, PerformanceProfiler& profiler) : m_Profiler(profiler), m_Name(name), m_Active(profiler.IsEnabled()) {
                if (m_Active) m_Start = std::chrono::high_resolution_clock::now();
            }
            ~ScopedTimer() {
                if (m_Active) {
                    auto end = std::chrono::high_resolution_clock::now();
                    m_Profiler.Record(m_Name, std::chrono::duration_cast<std::chrono::nanoseconds>(end - m_Start).count());
                }
            }
        };
        void Record(const char* name, uint64_t ns) {
            std::lock_guard<std::mutex> lock(m_Mutex);
            auto& threadData = m_Data[name][std::this_thread::get_id()];
            threadData.totalCalls++; threadData.totalTimeNS += ns;
            threadData.maxTimeNS = std::max(threadData.maxTimeNS, ns);
        }
    };
#define PROFILE_SCOPE(name) Presence::PerformanceProfiler::ScopedTimer __timer(name, m_Profiler)

    // =================================================================================================
    // MATERIAL SYSTEM
    // =================================================================================================
    class MaterialSystem {
        std::vector<MaterialParams> m_Materials;
        mutable std::shared_mutex m_Mutex;
    public:
        MaterialSystem() {
            m_Materials.resize((int)MaterialType::Count);
            Set(0, { 1.00f, 0.00f, 0.00f, 0.00f });
            Set(1, { 0.15f, 0.70f, 0.15f, 0.70f });
            Set(2, { 0.10f, 0.95f, 0.05f, 0.80f });
            Set(3, { 0.40f, 0.30f, 0.40f, 0.30f });
            Set(4, { 0.80f, 0.10f, 0.80f, 0.10f });
            Set(5, { 0.30f, 0.85f, 0.05f, 0.60f });
            Set(6, { 0.00f, 0.01f, 0.99f, 0.01f });
        }
        bool Set(int id, const MaterialParams& p) {
            std::unique_lock<std::shared_mutex> lock(m_Mutex);
            if (id >= 0) {
                if (id >= (int)m_Materials.size()) m_Materials.resize(id + 1);
                m_Materials[id] = p; return true;
            }
            return false;
        }
        int Add(const MaterialParams& p) {
            std::unique_lock<std::shared_mutex> lock(m_Mutex);
            m_Materials.push_back(p);
            return (int)m_Materials.size() - 1;
        }
        MaterialParams Get(int id) const {
            std::shared_lock<std::shared_mutex> lock(m_Mutex);
            if (id < 0 || id >= (int)m_Materials.size()) return m_Materials[0];
            return m_Materials[id];
        }
        float GetStepSize(int id) const { return (Get(id).transmission > 0.5f) ? 0.2f : 0.4f; }
    };

    // =================================================================================================
    // OCCLUSION CALCULATOR
    // =================================================================================================
    class OcclusionCalculator {
    public:
        struct Config {
            float MaxDistance = 80.0f; 
        };
    private:
        IGeometryProvider* m_Provider;
        const MaterialSystem& m_Materials;
        PerformanceProfiler& m_Profiler;
        Config m_Config;
        struct CacheEntry { float occlusion; uint64_t lastFrame; float3 lPos, sPos; };
        mutable std::unordered_map<uint64_t, CacheEntry> m_Cache;
        mutable std::unordered_map<uint64_t, float> m_History;
        mutable std::mutex m_CacheMutex;
        uint64_t m_CurrentFrame = 0;

    public:
        OcclusionCalculator(IGeometryProvider* p, const MaterialSystem& m, PerformanceProfiler& prof)
            : m_Provider(p), m_Materials(m), m_Profiler(prof) {}
        void SetConfig(const Config& c) { m_Config = c; }
        void Tick() {
            std::lock_guard<std::mutex> lock(m_CacheMutex); m_CurrentFrame++;
            if (m_CurrentFrame % 1000 == 0) {
                for (auto it = m_Cache.begin(); it != m_Cache.end(); ) { if (m_CurrentFrame - it->second.lastFrame > 30) it = m_Cache.erase(it); else ++it; }
                if (m_History.size() > 2000) m_History.clear();
            }
        }
        float Calculate(const float3& lPos, const float3& sPos) const {
            float3 dir = sPos - lPos; float dist = dir.magnitude();
            if (dist < 0.5f) return 1.0f; if (dist > m_Config.MaxDistance) return 0.0f;
            dir = dir.normalize();

            // Simple Raycast Check
            RayHit hit = m_Provider->CastRay(lPos, dir, dist);
            if (!hit.isHit) return 1.0f;

            // Transmission logic
            float energy = 1.0f;
            float trans = m_Materials.Get(hit.materialID).transmission;
            energy *= trans;

            // Second wall check?
            if (energy > 0.1f) {
                float3 p = lPos; p.mad(dir, hit.distance + 0.5f);
                float remDist = dist - hit.distance - 0.5f;
                if (remDist > 0) {
                    RayHit h2 = m_Provider->CastRay(p, dir, remDist);
                    if (h2.isHit) energy *= m_Materials.Get(h2.materialID).transmission;
                }
            }
            return std::max(energy, 0.0f);
        }
    };

    // =================================================================================================
    // MAIN SYSTEM IMPL
    // =================================================================================================
    class AudioSystem::Impl
    {
    public:
        IGeometryProvider* m_Provider = nullptr;
        Settings m_Settings;
        PerformanceProfiler m_Profiler;
        MaterialSystem m_Materials;

        std::unique_ptr<OcclusionCalculator> m_Occlusion;
        std::unique_ptr<std::thread> m_WorkerThread;

        std::mutex m_Mutex;
        std::condition_variable m_CV;
        std::atomic<bool> m_StopThread{ false };
        std::atomic<bool> m_WorkPending{ false };
        std::atomic<bool> m_Initialized{ false };

        float3 m_TargetPos, m_LastPos;
        float m_Fog = 0.0f;

        struct AnalysisContext {
            float3 CameraPosition;
            int FrameCount = 0;

            // --- NEW METRICS FOR ACCURACY ---
            float RawTotalDist = 0.0f;   // Unweighted geometric distance sum
            int TotalSegments = 0;       // Total number of path segments traced

            // Old energy-weighted metrics (for reverb tail calc)
            float EnergyWeightedDist = 0.0f;

            // Openness metric
            int FirstHitCount = 0;       // How many rays hit something immediately

            int MatHits[MAX_TRACKED_MATERIALS] = { 0 };

            // Results
            float PhysicalVolume = 100.0f;
            float MeanFreePath = 5.0f;
            float PhysicalReflectivity = 0.3f;
            float GeometricOpenness = 0.5f;
            float GeometricEnclosedness = 0.5f;
            float ReverbTime = 1.0f;
            float PerceivedReflectivity = 0.0f;

            void Reset() {
                FrameCount = 0;
                RawTotalDist = 0; TotalSegments = 0;
                EnergyWeightedDist = 0; FirstHitCount = 0;
                std::memset(MatHits, 0, sizeof(MatHits));
            }

            static float Lerp(float a, float b, float t) { return a + (b - a) * t; }
            static float Clamp(float v, float min, float max) { return (v < min) ? min : (v > max) ? max : v; }
        } m_Context;

        EAXResult m_TargetEAX, m_CurrentEAX;

        ~Impl() { Shutdown(); }

        void Init(IGeometryProvider* p, const Settings& s) {
            m_Provider = p; m_Settings = s;
            m_Occlusion = std::make_unique<OcclusionCalculator>(p, m_Materials, m_Profiler);
            m_Initialized = true; m_Context.Reset();
            if (s.useMultithreading) m_WorkerThread = std::make_unique<std::thread>(&Impl::PhysicsLoop, this);
        }

        void Shutdown() {
            m_Initialized = false;
            if (m_WorkerThread) {
                { std::lock_guard<std::mutex> lock(m_Mutex); m_StopThread = true; }
                m_CV.notify_all();
                if (m_WorkerThread->joinable()) m_WorkerThread->join();
                m_WorkerThread.reset();
            }
        }

        // ---------------------------------------------------------------------------------------------
        // MATH: CalculatePhysics (Improved MFP)
        // ---------------------------------------------------------------------------------------------
        void CalculatePhysics()
        {
            float numRays = (float)DIRECTIONS_COUNT;
            float totalFrames = std::max((float)m_Context.FrameCount, 1.0f);

            // 1. Mean Free Path (MFP)
            // FIX: Use Average Segment Length, not Total Path Length
            float totalSegments = std::max((float)m_Context.TotalSegments, 1.0f);
            float avgSegment = m_Context.RawTotalDist / totalSegments;

            m_Context.MeanFreePath = std::max(avgSegment, 0.5f);

            // 2. Geometric Enclosedness (Ratio of rays that hit something vs Sky)
            float totalFirstHits = (float)m_Context.FirstHitCount / totalFrames;
            float hitRatio = totalFirstHits / numRays;
            m_Context.GeometricEnclosedness = AnalysisContext::Clamp(hitRatio, 0.0f, 1.0f);
            m_Context.GeometricOpenness = 1.0f - m_Context.GeometricEnclosedness;

            // 3. Physical Volume
            // V = 4/3 * PI * R^3. Using MFP as Radius gives a good approximation for convex rooms.
            // For a 10x10x4 room, MFP ~ 4.5m. V ~ 380m^3. Size ~ 7.2m.
            float estVol = (4.0f / 3.0f) * PI * std::pow(m_Context.MeanFreePath, 3.0f);

            if (m_Context.GeometricOpenness > 0.6f) estVol = std::min(estVol, 50000.0f);
            m_Context.PhysicalVolume = AnalysisContext::Clamp(estVol, 10.0f, 500000.0f);

            // 4. Physical Reflectivity
            float totalMatHits = 0;
            float accumRefl = 0;
            for (int i = 0; i < MAX_TRACKED_MATERIALS; ++i) {
                if (m_Context.MatHits[i] > 0) {
                    float hits = (float)m_Context.MatHits[i] / totalFrames;
                    totalMatHits += hits;
                    accumRefl += hits * m_Materials.Get(i).rt60_weight;
                }
            }
            float baseReflectivity = (totalMatHits > 0) ? (accumRefl / totalMatHits) : 0.1f;
            float effectiveReflectivity = baseReflectivity * (1.0f - m_Context.GeometricOpenness * 0.8f);

            m_Context.PhysicalReflectivity = AnalysisContext::Clamp(effectiveReflectivity, 0.05f, 0.95f);
            m_Context.PerceivedReflectivity = std::pow(m_Context.PhysicalReflectivity, 0.5f);

            // 5. Reverb Time (Eyring)
            float surfArea = 6.0f * std::pow(m_Context.PhysicalVolume, 2.0f / 3.0f);
            float absCoeff = AnalysisContext::Clamp(1.0f - m_Context.PhysicalReflectivity, 0.01f, 0.99f);
            float rt60 = 0.161f * m_Context.PhysicalVolume / (-surfArea * std::log(1.0f - absCoeff));

            // Tweaks
            if (m_Context.PhysicalVolume < 150.0f) rt60 *= std::max(0.3f, m_Context.PhysicalVolume / 150.0f);
            if (m_Context.GeometricOpenness > 0.4f) rt60 *= AnalysisContext::Clamp(1.0f - (m_Context.GeometricOpenness - 0.4f) * 1.8f, 0.1f, 1.0f);

            m_Context.ReverbTime = AnalysisContext::Clamp(rt60, 0.1f, 8.0f);
        }

        void ComputeEAX()
        {
            CalculatePhysics();

            EAXResult eax;
            eax.isValid = true;
            eax.debugEnclosedness = m_Context.GeometricEnclosedness;
            eax.debugOpenness = m_Context.GeometricOpenness;

            float fog = AnalysisContext::Clamp(m_Fog, 0.0f, 1.0f);
            float open = m_Context.GeometricOpenness;

            // Room
            float roomInt = 0.3f + (0.7f * m_Context.GeometricEnclosedness);
            roomInt *= std::sqrt(m_Context.PerceivedReflectivity);
            if (open > 0.4f) {
                float t = AnalysisContext::Clamp((open - 0.4f) / 0.6f, 0.0f, 1.0f);
                roomInt *= AnalysisContext::Clamp(1.0f - (t * t), 0.1f, 1.0f);
            }
            float val = AnalysisContext::Lerp(-4000.0f, 400.0f, AnalysisContext::Clamp(roomInt - (fog * 0.4f), 0.0f, 1.1f));
            eax.lRoom = static_cast<int32_t>(val);
            if (m_Context.PhysicalVolume < 70.0f && m_Context.GeometricEnclosedness > 0.8f) eax.lRoom += 500;
            eax.lRoom = static_cast<int32_t>(AnalysisContext::Clamp((float)eax.lRoom, -10000.0f, 600.0f));

            // Decay
            eax.flDecayTime = m_Context.ReverbTime;
            if (fog > 0.5f) eax.flDecayTime *= 0.8f;

            // Reflections
            float reflScale = (open > 0.6f) ? 0.8f : 1.0f;
            float reflectionsVal = AnalysisContext::Lerp(-2500.0f, 200.0f, m_Context.PhysicalReflectivity * reflScale);
            eax.lReflections = static_cast<int32_t>(AnalysisContext::Clamp(reflectionsVal, -10000.0f, 500.0f));

            float baseDelay = m_Context.MeanFreePath / 340.0f; // MFP is now accurate (~5m -> 0.015s)
            if (open > 0.6f) baseDelay *= 1.5f;
            eax.flReflectionsDelay = AnalysisContext::Clamp(baseDelay, 0.0f, 0.3f);

            // Reverb
            float reverbVal = AnalysisContext::Lerp(-3000.0f, 0.0f, roomInt);
            if (open > 0.7f) reverbVal -= 600.0f;
            eax.lReverb = static_cast<int32_t>(AnalysisContext::Clamp(reverbVal, -10000.0f, 200.0f));
            eax.flReverbDelay = AnalysisContext::Clamp(eax.flReflectionsDelay + 0.02f, 0.0f, 0.1f);

            // Tone
            float hfLoss = -100.0f;
            float outdoorCut = open * -1200.0f;
            float roomHFVal = (float)eax.lRoom + hfLoss + outdoorCut;
            eax.lRoomHF = static_cast<int32_t>(AnalysisContext::Clamp(roomHFVal, -10000.0f, -100.0f));

            eax.flDecayHFRatio = (open > 0.6f) ? 0.5f : 0.83f;
            eax.flEnvironmentDiffusion = (open > 0.7f) ? 0.6f : 1.0f;
            eax.flAirAbsorptionHF = -5.0f + (open * -8.0f) - (fog * 5.0f);

            // Env Size: Size of the acoustic space side (Volume^(1/3))
            eax.flEnvironmentSize = std::pow(m_Context.PhysicalVolume, 1.0f / 3.0f);

            { std::lock_guard<std::mutex> lock(m_Mutex); m_TargetEAX = eax; }
        }

        void PhysicsLoop() {
            while (!m_StopThread) {
                float3 curPos;
                {
                    std::unique_lock<std::mutex> lock(m_Mutex);
                    m_CV.wait(lock, [this] { return m_WorkPending || m_StopThread; });
                    if (m_StopThread) break;
                    curPos = m_TargetPos;
                    m_WorkPending = false;
                }

                if (m_LastPos.distance_to(curPos) > 0.5f) { m_Context.Reset(); m_LastPos = curPos; }
                m_Context.CameraPosition = curPos;

                {
                    PROFILE_SCOPE("EAX::Trace");
                    for (int i = 0; i < DIRECTIONS_COUNT; ++i) {
                        float energy = 1.0f;
                        float3 pos = curPos; float3 dir = SphereDirections[i];

                        for (int b = 0; b < m_Settings.maxBounces; ++b) {
                            RayHit hit = m_Provider->CastRay(pos, dir, m_Settings.maxRayDistance);

                            // Hit nothing (Sky)
                            if (!hit.isHit) {
                                // Treat sky as a segment of max length
                                m_Context.RawTotalDist += m_Settings.maxRayDistance;
                                m_Context.TotalSegments++;
                                m_Context.EnergyWeightedDist += m_Settings.maxRayDistance * energy;
                                break;
                            }

                            // Hit Wall
                            // 1. Geometric data (Unweighted)
                            m_Context.RawTotalDist += hit.distance;
                            m_Context.TotalSegments++;

                            // 2. Acoustic data (Weighted)
                            m_Context.EnergyWeightedDist += hit.distance * energy;

                            // 3. Openness Check
                            if (b == 0) m_Context.FirstHitCount++;

                            // 4. Material
                            if (hit.materialID >= 0 && hit.materialID < MAX_TRACKED_MATERIALS)
                                m_Context.MatHits[hit.materialID]++;

                            energy *= (1.0f - m_Materials.Get(hit.materialID).absorption);
                            if (energy < 0.05f) break;

                            pos = pos; pos.mad(dir, hit.distance - 0.05f);
                            dir = ThreadSafeRandom::HemisphereDir(hit.normal);
                        }
                    }
                }

                m_Context.FrameCount++;
                ComputeEAX();
                if (m_Occlusion) m_Occlusion->Tick();
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }
        }
    };

    // =================================================================================================
    // PUBLIC API
    // =================================================================================================

    AudioSystem::AudioSystem() : m_Impl(new Impl()) {}
    AudioSystem::~AudioSystem() { Shutdown(); delete m_Impl; }

    void AudioSystem::Initialize(IGeometryProvider* p, const Settings& s) { if (m_Impl) m_Impl->Init(p, s); }
    void AudioSystem::Shutdown() { if (m_Impl) m_Impl->Shutdown(); }

    template <typename T> T LerpEAX(T current, T target, float step) {
        float diff = static_cast<float>(target - current);
        if (std::abs(diff) < 0.1f) return target;
        return static_cast<T>(static_cast<float>(current) + diff * step);
    }

    void AudioSystem::Update(const float3& pos, float dt, float fog) {
        if (!m_Impl || !m_Impl->m_Initialized) return;
        m_Impl->m_Fog = fog;

        if (m_Impl->m_Settings.useMultithreading) {
            { std::lock_guard<std::mutex> lock(m_Impl->m_Mutex); m_Impl->m_TargetPos = pos; m_Impl->m_WorkPending = true; }
            m_Impl->m_CV.notify_one();
        }

        EAXResult target;
        { std::lock_guard<std::mutex> lock(m_Impl->m_Mutex); target = m_Impl->m_TargetEAX; }

        if (target.isValid) {
            float speed = 2.0f * dt;
            if (std::abs(target.lRoom - m_Impl->m_CurrentEAX.lRoom) > 1000) speed = 5.0f * dt;

            auto& curr = m_Impl->m_CurrentEAX;
            curr.lRoom = LerpEAX(curr.lRoom, target.lRoom, speed);
            curr.lRoomHF = LerpEAX(curr.lRoomHF, target.lRoomHF, speed);
            curr.lReflections = LerpEAX(curr.lReflections, target.lReflections, speed);
            curr.lReverb = LerpEAX(curr.lReverb, target.lReverb, speed);
            curr.flDecayTime = LerpEAX(curr.flDecayTime, target.flDecayTime, speed);
            curr.flEnvironmentSize = LerpEAX(curr.flEnvironmentSize, target.flEnvironmentSize, speed);

            curr.flDecayHFRatio = target.flDecayHFRatio;
            curr.flReflectionsDelay = target.flReflectionsDelay;
            curr.flReverbDelay = target.flReverbDelay;
            curr.flAirAbsorptionHF = target.flAirAbsorptionHF;
            curr.flEnvironmentDiffusion = target.flEnvironmentDiffusion;
            curr.isValid = true;
        }
    }

    EAXResult AudioSystem::GetEAXResult() const { return m_Impl ? m_Impl->m_CurrentEAX : EAXResult(); }
    bool AudioSystem::SetMaterialProperties(int id, const MaterialParams& p) { return m_Impl && m_Impl->m_Materials.Set(id, p); }
    int AudioSystem::CreateCustomMaterial(const MaterialParams& p) { return m_Impl ? m_Impl->m_Materials.Add(p) : -1; }

    float AudioSystem::CalculateOcclusion(const float3& lPos, const float3& sPos) {
        if (!m_Impl || !m_Impl->m_Occlusion) return 1.0f;
        return m_Impl->m_Occlusion->Calculate(lPos, sPos);
    }
}
