/*
====================================================================================================
  Presence Audio SDK
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
#include "TestUtils.h"
#include "MockGeometry.h"
#include "../PresenceAudioSDK/Include/PresenceSystem.h"
#include <thread>
#include <chrono>
// Линковка с библиотекой (если используется MSVC)
// #pragma comment(lib, "PresenceAudioSDK.lib")
using namespace Presence;
void RunSimulationTest() {
    Logger::Log("=== Starting Simulation Test ===");

        // 1. Создаем систему
        AudioSystem audioSystem;

    // Создаем геометрию на стеке (или через new, если объект большой)
    BoxRoomProvider geometry;

    Logger::Log("Geometry created: Box 10x10x4m");

    Settings settings;
    settings.maxBounces = 3;            // Достаточно для теста
    settings.useMultithreading = true;  // Тестируем многопоточность
    settings.maxRayDistance = 100.0f;
    settings.updateInterval = 0.033f;

    Logger::Log("Initializing AudioSystem (" + std::string(audioSystem.GetVersionString()) + ")...");
    audioSystem.Initialize(&geometry, settings);

    // 2. Симуляция игрового цикла (60 кадров ~ 2 секунды)
    // Слушатель стоит в центре комнаты (2 метра над полом)
    float3 listenerPos(0.0f, 2.0f, 0.0f);
    float dt = 0.033f; // ~30 FPS

    Logger::Log("Running simulation loop (60 frames)...");
    Logger::Log("Listener moving from Center (0,2,0) towards Metal Wall (+X)");

    bool dataValidOnce = false;

    for (int i = 0; i < 60; ++i) {
        // Эмуляция движения: слушатель идет к стене X+
        listenerPos.x += 0.05f;

        // Обновляем аудио движок
        audioSystem.Update(listenerPos, dt);

        // Получаем результат
        EAXResult res = audioSystem.GetEAXResult();

        if (res.isValid) dataValidOnce = true;

        if (i % 10 == 0) { // Логируем каждый 10-й кадр
            std::string status = res.isValid ? "[VALID]" : "[WAITING]";
            Logger::Log("Frame " + std::to_string(i) + " " + status + " | Pos: " + listenerPos.to_string());

            if (res.isValid) {
                Logger::LogParam("Room Level (mB)", (float)res.lRoom);
                Logger::LogParam("Decay Time (s)", res.flDecayTime);
                Logger::LogParam("Reflections (mB)", (float)res.lReflections);
                Logger::LogParam("Reverb (mB)", (float)res.lReverb);
                Logger::LogParam("Env Size (m)", res.flEnvironmentSize);
                Logger::LogParam("Debug Enclosed", res.debugEnclosedness);
            }
        }

        // Эмуляция времени кадра
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    if (!dataValidOnce) {
        Logger::Log("TEST FAILED: No valid EAX data received!", true);
    }
    else {
        Logger::Log("TEST PASSED: Valid data stream received.");
    }

    // 3. Завершение
    Logger::Log("Shutting down system...");
    audioSystem.Shutdown();
}
int main() {
    // Инициализация логгера
    Logger::Init("PresenceAutotest_Log.txt");
    Logger::Log("PresenceAutotest v1.0 started");
        try {
        RunSimulationTest();
    }
    catch (const std::exception& e) {
        Logger::Log(std::string("EXCEPTION: ") + e.what(), true);
        return -1;
    }
    catch (...) {
        Logger::Log("UNKNOWN EXCEPTION OCCURRED", true);
        return -1;
    }

    Logger::Log("Done. Press Enter to exit...");
    std::cin.get();
    return 0;
}
