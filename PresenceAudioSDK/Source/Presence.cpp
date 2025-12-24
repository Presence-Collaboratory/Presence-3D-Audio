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
    static const float SPEED_OF_SOUND = 340.0f;

    // Оптимизированное распределение точек на сфере (16 направлений для анализа EAX)
    static const float3 SphereDirections[] = {
        {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
        {0.707f,0,0.707f}, {-0.707f,0,0.707f}, {0.707f,0,-0.707f}, {-0.707f,0,-0.707f},
        {0.577f,0.577f,0.577f}, {-0.577f,-0.577f,-0.577f},
        {0,0.707f,0.707f}, {0,-0.707f,-0.707f}, {0.707f,0.707f,0}, {-0.707f,-0.707f,0}
    };
    static const int DIRECTIONS_COUNT = sizeof(SphereDirections) / sizeof(float3);

    // =================================================================================================
    // VERSION INFO
    // =================================================================================================
    float AudioSystem::GetVersion() { return 0.2f; }
    const char* AudioSystem::GetVersionString() { return "Presence Audio ver. 0.2 InDev"; }

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
        struct ThreadLocalData {
            uint64_t totalCalls = 0;
            uint64_t totalTimeNS = 0;
            uint64_t maxTimeNS = 0;
        };

        using ThreadId = std::thread::id;
        std::unordered_map<std::string, std::unordered_map<ThreadId, ThreadLocalData>> m_Data;
        std::mutex m_Mutex;
        bool m_Enabled = false;

    public:
        void SetEnabled(bool enabled) { m_Enabled = enabled; }
        bool IsEnabled() const { return m_Enabled; }

        class ScopedTimer {
            PerformanceProfiler& m_Profiler;
            const char* m_Name;
            std::chrono::high_resolution_clock::time_point m_Start;
            bool m_Active;
        public:
            ScopedTimer(const char* name, PerformanceProfiler& profiler)
                : m_Profiler(profiler), m_Name(name), m_Active(profiler.IsEnabled()) {
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
            threadData.totalCalls++;
            threadData.totalTimeNS += ns;
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
            Set(0, { 1.00f, 0.00f, 0.00f, 0.00f }); // Air
            Set(1, { 0.15f, 0.70f, 0.15f, 0.70f }); // Stone
            Set(2, { 0.10f, 0.95f, 0.05f, 0.80f }); // Metal
            Set(3, { 0.40f, 0.30f, 0.40f, 0.30f }); // Wood
            Set(4, { 0.80f, 0.10f, 0.80f, 0.10f }); // Soft
            Set(5, { 0.30f, 0.85f, 0.05f, 0.60f }); // Glass
            Set(6, { 0.00f, 0.01f, 0.99f, 0.01f }); // Absorber
        }

        bool Set(int id, const MaterialParams& p) {
            std::unique_lock<std::shared_mutex> lock(m_Mutex);
            if (id >= 0 && id < (int)m_Materials.size()) { m_Materials[id] = p; return true; }
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

        float GetStepSize(int id) const {
            return (Get(id).transmission > 0.5f) ? 0.2f : 0.4f;
        }
    };

    // =================================================================================================
    // OCCLUSION CALCULATOR
    // =================================================================================================
    class OcclusionCalculator {
    public:
        struct Config {
            int MaxDiffractionDepth = 3;
            float MaxDistance = 80.0f;
            float MinDirectThreshold = 0.02f;
            bool EnableVertical = true;
            bool EnablePortals = true;
        };

    private:
        IGeometryProvider* m_Provider;
        const MaterialSystem& m_Materials;
        PerformanceProfiler& m_Profiler;
        Config m_Config;

        struct CacheEntry {
            float occlusion;
            uint64_t lastFrame;
            float3 lPos, sPos;
        };
        mutable std::unordered_map<uint64_t, CacheEntry> m_Cache;
        mutable std::unordered_map<uint64_t, float> m_History;
        mutable std::mutex m_CacheMutex;
        uint64_t m_CurrentFrame = 0;

        struct PathNode { float3 pos; float energy; int depth; float3 prevDir; };

    public:
        OcclusionCalculator(IGeometryProvider* p, const MaterialSystem& m, PerformanceProfiler& prof)
            : m_Provider(p), m_Materials(m), m_Profiler(prof) {}

        void SetConfig(const Config& c) { m_Config = c; }

        void Tick() {
            std::lock_guard<std::mutex> lock(m_CacheMutex);
            m_CurrentFrame++;
            if (m_CurrentFrame % 1000 == 0) {
                const uint64_t MAX_AGE = 30;
                for (auto it = m_Cache.begin(); it != m_Cache.end(); ) {
                    if (m_CurrentFrame - it->second.lastFrame > MAX_AGE) it = m_Cache.erase(it); else ++it;
                }
                if (m_History.size() > 2000) m_History.clear();
            }
        }

        float Calculate(const float3& lPos, const float3& sPos) const {
            PROFILE_SCOPE("Occlusion::Calculate");

            float3 dir = sPos - lPos;
            float dist = dir.magnitude();

            if (dist < 0.5f) return 1.0f;
            if (dist > m_Config.MaxDistance) return 0.0f;
            dir = dir.normalize();

            uint64_t key = Hash(lPos, sPos);
            float cached = 0.0f;
            if (TryGetCache(key, lPos, sPos, cached)) return cached;

            float direct = ComputeDirect(lPos, sPos, dist, dir);
            if (direct > 0.9f) {
                UpdateCache(key, direct, lPos, sPos);
                return direct;
            }

            float diff = ComputeDiffraction(lPos, sPos, dist, direct);

            float special = 0.0f;
            if (diff < 0.2f && direct < 0.2f) {
                special = ComputeSpecial(lPos, sPos);
            }

            float bass = ComputeBass(direct, dist);

            float result = 0.0f;
            if (direct > 0.2f) {
                result = direct + (diff * 0.2f);
            }
            else {
                float indirect = std::max(diff, special);
                indirect *= (1.0f / (1.0f + dist * 0.02f));
                result = std::min(indirect, 0.75f);
            }

            result = std::clamp(result + bass, 0.05f, 1.0f);
            UpdateCache(key, result, lPos, sPos);
            return result;
        }

    private:
        float ComputeDirect(const float3& start, const float3& end, float dist, const float3& dir) const {
            float energy = 1.0f;
            float3 cur = start; cur.mad(dir, 0.2f);
            float travelled = 0.2f;
            int steps = 0;

            while (travelled < dist && steps < 6) {
                float scan = dist - travelled;
                RayHit hit = m_Provider->CastRay(cur, dir, scan);

                if (!hit.isHit) break;
                float3 hitPos = cur; hitPos.mad(dir, hit.distance);
                if ((end - hitPos).magnitude() < 0.5f) break;

                float trans = m_Materials.Get(hit.materialID).transmission;
                energy *= trans;
                if (energy < m_Config.MinDirectThreshold) return 0.0f;

                float step = m_Materials.GetStepSize(hit.materialID);
                cur = hitPos; cur.mad(dir, step);
                travelled += (hit.distance + step);
                steps++;
            }
            return energy;
        }

        float ComputeDiffraction(const float3& start, const float3& target, float dist, float directE) const {
            if (dist > 40.0f || directE > 0.5f) return ComputeSimpleDiffraction(start, target);

            std::vector<PathNode> stack;
            stack.reserve(16);
            stack.push_back({ start, 1.0f, 0, float3(0,0,0) });

            float bestE = 0.0f;
            int ops = 0;

            while (!stack.empty() && ops < 40) {
                PathNode cur = stack.back(); stack.pop_back(); ops++;

                float3 toT = target - cur.pos;
                float d = toT.magnitude();
                toT = toT.normalize();

                RayHit hit = m_Provider->CastRay(cur.pos, toT, d);
                if (!hit.isHit || (d - hit.distance < 0.5f)) {
                    float e = cur.energy * std::powf(0.6f, (float)cur.depth) * (1.0f / (1.0f + d * 0.03f));
                    if (e > bestE) bestE = e;
                    continue;
                }

                if (cur.depth >= m_Config.MaxDiffractionDepth || cur.energy < 0.05f) continue;

                int rays = (cur.depth == 0) ? 6 : 3;
                for (int i = 0; i < rays; ++i) {
                    float3 dir;
                    if (cur.depth == 0 && i < 3) {
                        float3 r = ThreadSafeRandom::Direction();
                        dir = (toT + r * 0.7f).normalize();
                    }
                    else {
                        dir = ThreadSafeRandom::Direction();
                        if (dir.dot(cur.prevDir) < -0.5f) dir = dir * -1.0f;
                    }

                    const float STEP = 1.8f;
                    if (!m_Provider->CastRay(cur.pos, dir, STEP).isHit) {
                        float3 next = cur.pos; next.mad(dir, STEP);
                        stack.push_back({ next, cur.energy * 0.75f, cur.depth + 1, dir });
                    }
                }
            }
            return bestE;
        }

        float ComputeSimpleDiffraction(const float3& lPos, const float3& sPos) const {
            for (int i = 0; i < 4; ++i) {
                float3 dir = ThreadSafeRandom::Direction();
                if (!m_Provider->CastRay(lPos, dir, 1.5f).isHit) {
                    float3 p = lPos; p.mad(dir, 1.5f);
                    float3 toS = sPos - p;
                    if (!m_Provider->CastRay(p, toS.normalize(), toS.magnitude()).isHit) return 0.25f;
                }
            }
            return 0.0f;
        }

        float ComputeSpecial(const float3& lPos, const float3& sPos) const {
            float maxE = 0.0f;
            if (m_Config.EnableVertical) {
                float3 offsets[] = { {0,2.5f,0}, {0,-2.5f,0} };
                for (const auto& off : offsets) {
                    if (!m_Provider->CastRay(lPos, off.normalize(), 2.5f).isHit) {
                        float3 p = lPos + off;
                        float3 toS = sPos - p;
                        if (!m_Provider->CastRay(p, toS.normalize(), toS.magnitude()).isHit) maxE = std::max(maxE, 0.35f);
                    }
                }
            }
            if (m_Config.EnablePortals && maxE < 0.2f) {
                int hits = 0; const int RAYS = 5;
                for (int i = 0; i < RAYS; ++i) {
                    float3 dir = ThreadSafeRandom::Direction();
                    RayHit hit = m_Provider->CastRay(lPos, dir, 4.0f);
                    if (hit.isHit) {
                        float3 p = lPos; p.mad(dir, hit.distance);
                        float3 hole = ThreadSafeRandom::HemisphereDir(hit.normal);
                        float3 check = p; check.mad(hole, 0.6f); check = check + hit.normal * -0.4f;
                        float3 toS = sPos - check;
                        if (!m_Provider->CastRay(p, toS.normalize(), toS.magnitude()).isHit) hits++;
                    }
                }
                if (hits > 0) maxE = std::max(maxE, (float)hits / RAYS * 0.5f);
            }
            return maxE;
        }

        float ComputeBass(float direct, float dist) const {
            if (direct > 0.6f) return 0.0f;
            return (1.0f - direct) * (1.0f / (1.0f + dist * 0.05f)) * 0.25f;
        }

        uint64_t Hash(const float3& a, const float3& b) const noexcept {
            auto q = [](float f) {return (int)(f * 10.0f); };
            uint64_t h = 0;
            auto c = [&h](int k) { h ^= std::hash<int>{}(k)+0x9e3779b9 + (h << 6) + (h >> 2); };
            c(q(a.x)); c(q(a.y)); c(q(a.z)); c(q(b.x)); c(q(b.y)); c(q(b.z));
            return h;
        }

        bool TryGetCache(uint64_t key, const float3& lPos, const float3& sPos, float& out) const {
            std::lock_guard<std::mutex> lock(m_CacheMutex);
            auto it = m_Cache.find(key);
            if (it != m_Cache.end()) {
                if ((lPos - it->second.lPos).length_sq() < 0.05f &&
                    (sPos - it->second.sPos).length_sq() < 0.05f &&
                    (m_CurrentFrame - it->second.lastFrame) <= 5) {
                    out = it->second.occlusion;
                    return true;
                }
            }
            return false;
        }

        void UpdateCache(uint64_t key, float val, const float3& lPos, const float3& sPos) const {
            std::lock_guard<std::mutex> lock(m_CacheMutex);
            float& hist = m_History[key];
            if (hist == 0.0f) hist = val;
            hist = hist * 0.7f + val * 0.3f;
            m_Cache[key] = { hist, m_CurrentFrame, lPos, sPos };
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

        // State
        float3 m_TargetPos, m_LastPos;
        float m_Fog = 0.0f;

        struct AnalysisContext {
            float3 CameraPosition;
            int FrameCount = 0;
            float TotalDist = 0, TotalHits = 0;
            int MatHits[MAX_TRACKED_MATERIALS] = { 0 };
            void Reset() { FrameCount = 0; TotalDist = 0; TotalHits = 0; std::memset(MatHits, 0, sizeof(MatHits)); }
        } m_Context;

        EAXResult m_TargetEAX, m_CurrentEAX;

        ~Impl() { Shutdown(); }

        void Init(IGeometryProvider* p, const Settings& s) {
            if (!p) throw std::invalid_argument("GeometryProvider null");
            m_Provider = p;
            m_Settings = s;
            // m_Profiler.SetEnabled(true); // Uncomment to debug profile

            m_Occlusion = std::make_unique<OcclusionCalculator>(p, m_Materials, m_Profiler);
            if (m_Occlusion) {
                OcclusionCalculator::Config c;
                // Map settings here if needed
                m_Occlusion->SetConfig(c);
            }

            m_Initialized = true;
            m_Context.Reset();

            if (s.useMultithreading) {
                m_WorkerThread = std::make_unique<std::thread>(&Impl::PhysicsLoop, this);
            }
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
                            if (!hit.isHit) { m_Context.TotalDist += m_Settings.maxRayDistance * energy; break; }
                            m_Context.TotalDist += hit.distance * energy;
                            if (b == 0) m_Context.TotalHits++;
                            if (hit.materialID >= 0 && hit.materialID < MAX_TRACKED_MATERIALS) m_Context.MatHits[hit.materialID]++;

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

        void ComputeEAX() {
            float frames = (float)std::max(m_Context.FrameCount, 1);
            float rays = (float)DIRECTIONS_COUNT;
            float mfp = std::max(m_Context.TotalDist / (frames * rays), 0.5f);
            float enclosed = (float)m_Context.TotalHits / (frames * rays);

            EAXResult res; res.isValid = true;
            res.lRoom = (int)((enclosed - 1.0f) * 2000.0f);
            res.flDecayTime = mfp * 0.15f;
            res.flEnvironmentSize = mfp;

            { std::lock_guard<std::mutex> lock(m_Mutex); m_TargetEAX = res; }
        }
    };

    // =================================================================================================
    // PUBLIC API
    // =================================================================================================

    AudioSystem::AudioSystem() : m_Impl(new Impl()) {}
    AudioSystem::~AudioSystem() { Shutdown(); delete m_Impl; }

    void AudioSystem::Initialize(IGeometryProvider* p, const Settings& s) {
        if (m_Impl) m_Impl->Init(p, s);
    }

    void AudioSystem::Shutdown() {
        if (m_Impl) m_Impl->Shutdown();
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
        if (target.isValid) m_Impl->m_CurrentEAX = target;
    }

    EAXResult AudioSystem::GetEAXResult() const { return m_Impl ? m_Impl->m_CurrentEAX : EAXResult(); }
    bool AudioSystem::SetMaterialProperties(int id, const MaterialParams& p) { return m_Impl && m_Impl->m_Materials.Set(id, p); }
    int AudioSystem::CreateCustomMaterial(const MaterialParams& p) { return m_Impl ? m_Impl->m_Materials.Add(p) : -1; }

    float AudioSystem::CalculateOcclusion(const float3& lPos, const float3& sPos) {
        if (!m_Impl || !m_Impl->m_Occlusion) return 1.0f;
        return m_Impl->m_Occlusion->Calculate(lPos, sPos);
    }
}
