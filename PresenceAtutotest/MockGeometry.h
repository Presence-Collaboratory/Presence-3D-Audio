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
#pragma once
#include "../PresenceAudioSDK/Include/PresenceSystem.h"
#include <vector>
#include <cmath>
using namespace Presence;
struct Plane {
    float3 normal;
    float distance; // Смещение плоскости (D в уравнении dot(N, P) + D = 0)
    MaterialType mat;
};
class BoxRoomProvider : public IGeometryProvider {
private:
    std::vector<Plane> walls;
public:
    BoxRoomProvider() {
        // Создаем комнату 10x10x4 метра (центр в 0,0,0)
        // Уравнение плоскости: dot(Normal, Point) + Distance = 0

            // Пол (y = 0). Нормаль вверх (0,1,0). 
                // 0*x + 1*y + 0*z + D = 0 => 0 + D = 0 => D = 0.
        walls.push_back({ {0, 1, 0}, 0.0f, MaterialType::Stone });

        // Потолок (y = 4). Нормаль вниз (0,-1,0). 
        // 0*x + (-1)*4 + 0*z + D = 0 => -4 + D = 0 => D = 4.
        walls.push_back({ {0, -1, 0}, 4.0f, MaterialType::Wood });

        // Стена Z+ (z = 5). Нормаль внутрь (0,0,-1).
        // (-1)*5 + D = 0 => D = 5.
        walls.push_back({ {0, 0, -1}, 5.0f, MaterialType::Stone });

        // Стена Z- (z = -5). Нормаль внутрь (0,0,1).
        // 1*(-5) + D = 0 => D = 5.
        walls.push_back({ {0, 0, 1}, 5.0f, MaterialType::Stone });

        // Стена X+ (x = 5). Нормаль внутрь (-1,0,0).
        // (-1)*5 + D = 0 => D = 5.
        walls.push_back({ {-1, 0, 0}, 5.0f, MaterialType::Metal });

        // Стена X- (x = -5). Нормаль внутрь (1,0,0).
        // 1*(-5) + D = 0 => D = 5.
        walls.push_back({ {1, 0, 0}, 5.0f, MaterialType::Metal });
    }

    // Реализация трассировки луча
    virtual RayHit CastRay(const float3& start, const float3& dir, float maxDist) override {
        RayHit closestHit;
        closestHit.isHit = false;
        closestHit.distance = maxDist;

        for (const auto& plane : walls) {
            float denom = plane.normal.dot(dir);

            // Если denom < 0, значит луч летит НАВСТРЕЧУ плоскости (нормаль смотрит на нас)
            // Иначе мы стреляем в "спину" стене или параллельно.
            if (denom < -1e-6f) {
                // Уравнение пересечения луча и плоскости:
                // t = -(dot(N, S) + D) / dot(N, V)
                float t = -(plane.normal.dot(start) + plane.distance) / denom;

                if (t > 0.001f && t < closestHit.distance) {
                    closestHit.distance = t;
                    closestHit.isHit = true;
                    closestHit.normal = plane.normal;
                    closestHit.materialID = (int)plane.mat;
                }
            }
        }

        return closestHit;
    }
};
