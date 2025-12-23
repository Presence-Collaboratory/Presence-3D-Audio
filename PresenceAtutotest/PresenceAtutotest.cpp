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
#include <PresenceSystem.h>
#include <thread>
#include <chrono>

#pragma comment(lib, "PresenceAudioSDK.lib")

using namespace Presence;

void RunSimulationTest() {
    Logger::Log("=== Starting Simulation Test ===");

    // 1. Создаем систему
    AudioSystem audioSystem;
    BoxRoomProvider geometry; // Наша фейковая комната

    Settings settings;
    settings.maxBounces = 2; // Для теста хватит 2 отскоков
    settings.useMultithreading = true;
    settings.maxRayDistance = 100.0f;

    Logger::Log("Initializing AudioSystem...");
    audioSystem.Initialize(&geometry, settings);

    // 2. Симуляция игрового цикла (например, 60 кадров)
    // Слушатель стоит в центре комнаты (2 метра над полом)
    float3 listenerPos(0.0f, 2.0f, 0.0f);
    float dt = 0.033f; // ~30 FPS

    Logger::Log("Running simulation loop (60 frames)...");

    for (int i = 0; i < 60; ++i) {
        // Эмуляция движения: слушатель медленно идет к стене
        listenerPos.x += 0.05f;

        // Обновляем аудио движок
        audioSystem.Update(listenerPos, dt);

        // Получаем результат
        EAXResult res = audioSystem.GetEAXResult();

        if (i % 10 == 0) { // Логируем каждый 10-й кадр
            std::string status = res.isValid ? "[VALID]" : "[WAITING]";
            Logger::Log("Frame " + std::to_string(i) + " " + status +
                " | Pos: " + listenerPos.to_string());

            if (res.isValid) {
                Logger::LogParam("  > Room Level", (float)res.lRoom);
                Logger::LogParam("  > Decay Time", res.flDecayTime);
                Logger::LogParam("  > Reflections", (float)res.lReflections);
                Logger::LogParam("  > Env Size", res.flEnvironmentSize);
                Logger::LogParam("  > Enclosedness", res.debugEnclosedness);
            }
        }

        // Маленькая пауза, чтобы не заспамить консоль (эмуляция времени кадра)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 3. Завершение
    Logger::Log("Shutting down system...");
    audioSystem.Shutdown();
    Logger::Log("Test Finished Successfully.");
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

    Logger::Log("Press Enter to exit...");
    std::cin.get();
    return 0;
}
