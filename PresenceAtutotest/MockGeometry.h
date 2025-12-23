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
#include <PresenceSystem.h>
#include <limits>
#include <vector>

using namespace Presence;

// Простая структура плоскости для пересечений
struct Plane {
    float3 normal;
    float distance; // Расстояние от начала координат (d в уравнении ax+by+cz+d=0)
    MaterialType mat;
};

class BoxRoomProvider : public IGeometryProvider {
private:
    std::vector<Plane> walls;

public:
    BoxRoomProvider() {
        // Создаем комнату 10x10x4 метра (центр в 0,0,0)
        // Пол (y = 0)
        walls.push_back({ {0, 1, 0}, 0.0f, MaterialType::Stone });
        // Потолок (y = 4)
        walls.push_back({ {0, -1, 0}, 4.0f, MaterialType::Wood }); // Потолок выше на 4м
        // Стена Z+ (z = 5)
        walls.push_back({ {0, 0, -1}, 5.0f, MaterialType::Stone });
        // Стена Z- (z = -5)
        walls.push_back({ {0, 0, 1}, 5.0f, MaterialType::Stone });
        // Стена X+ (x = 5)
        walls.push_back({ {-1, 0, 0}, 5.0f, MaterialType::Metal });
        // Стена X- (x = -5)
        walls.push_back({ {1, 0, 0}, 5.0f, MaterialType::Metal });
    }

    // Реализация трассировки
    virtual RayHit CastRay(const float3& start, const float3& dir, float maxDist) override {
        RayHit closestHit;
        closestHit.distance = maxDist;
        closestHit.isHit = false;

        for (const auto& plane : walls) {
            // Пересечение луча и плоскости: t = -(dot(n, p0) + d) / dot(n, dir)
            // Но у нас уравнение Plane: dot(n, p) + distance = 0 -> distance это смещение
            // Обычно плоскость задается как dot(n, p) = d.
            // Для упрощения используем логику смещения плоскости от начала координат.

            float denom = plane.normal.dot(dir);

            // Если denom близок к 0, луч параллелен плоскости
            if (std::abs(denom) > 1e-6f) {
                // Вектор от точки на плоскости до старта
                // Точка на плоскости: center = normal * distance (для box)
                // Или просто используем стандартную формулу для Axis Aligned Planes

                // Перепишем для box offset logic:
                // Плоскость определяется: (P - P0) * n = 0
                // P0 = plane.normal * (-plane.distance) - если distance это offset
                // Давайте проще: t = (offset - dot(n, start)) / dot(n, dir)

                // В данном сетапе distance - это смещение плоскости вдоль нормали.
                // Например, пол: N(0,1,0), D=0. Точка на полу (0,0,0).
                // Потолок: N(0,-1,0), D=4. Точка (0,4,0).

                // Корректная формула для нашего определения Box:
                // Точка на плоскости P_plane = plane.normal * (-plane.distance) - НЕТ.
                // Давайте просто считать distance как расстояние от центра комнаты (0,0,0) до стены.
                // Стена X=5. Нормаль (-1, 0, 0) (смотрит внутрь). 
                // Уравнение плоскости: dot(N, P) + D = 0.
                // dot((-1,0,0), (5,0,0)) + D = 0 => -5 + D = 0 => D = 5.

                float t = (plane.distance - plane.normal.dot(start)) / denom;

                if (t > 0.001f && t < closestHit.distance) {
                    closestHit.distance = t;
                    closestHit.isHit = true;
                    closestHit.normal = plane.normal;
                    closestHit.material = plane.mat;
                }
            }
        }

        return closestHit;
    }
};
