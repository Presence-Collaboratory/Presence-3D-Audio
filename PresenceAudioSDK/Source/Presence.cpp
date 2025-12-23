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
#include <PresenceSystem.h> 

#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <chrono>
#include <cstring>
#include <random>
#include <array>
#include <ppl.h>

namespace Presence
{
    // =================================================================================================
    // БАЗА ДАННЫХ МАТЕРИАЛОВ (Central Material Registry)
    // =================================================================================================
    // Централизованное хранилище физических свойств материалов.
    // Используется паттерн Data-Driven Design для легкого добавления новых типов поверхностей.
    // =================================================================================================

    struct MaterialParams
    {
        float transmission;  // [0.0-1.0] Проницаемость: сколько звука проходит сквозь препятствие (для Occlusion).
        float reflectivity;  // [0.0-1.0] Отражающая способность: сила эха и вторичных отражений.
        float absorption;    // [0.0-1.0] Поглощение: сколько энергии теряется при ударе луча (для Path Tracing).
        float rt60_weight;   // [0.0-1.0] Вес материала при расчете общего времени реверберации комнаты (Tone).
    };

    // Таблица свойств. Индекс в массиве должен строго соответствовать enum MaterialType.
    static const MaterialParams kMaterialRegistry[] =
    {
        // Transm | Refl  | Absorp | Weight(RT60)
        // Воздух
        { 1.00f,   0.00f,   0.00f,   0.00f },

        // Stone/Concrete: Ставим 0.15 (15%). Это достаточно тихо, но не 0.
        // Звук будет приглушен на -16dB, что ощутимо, но слышно.
        { 0.15f,   0.70f,   0.15f,   0.70f },

        // Metal: Металл звенит, но плохо пропускает. Ставим 0.1 (10%).
        { 0.10f,   0.95f,   0.05f,   0.80f },

        // Wood: Дерево хорошо пропускает НЧ. Ставим 0.4 (40%).
        { 0.40f,   0.30f,   0.40f,   0.30f },

        // Soft: Ткань/Кусты почти не глушат звук. 80%.
        { 0.80f,   0.10f,   0.80f,   0.10f },

        // Glass: Стекло тонкое. 30%.
        { 0.30f,   0.85f,   0.05f,   0.60f },

        // Absorber
        { 0.00f,   0.01f,   0.99f,   0.01f },
    };

    // Безопасный доступ к свойствам материала с проверкой границ массива.
    static const MaterialParams& GetMatInfo(MaterialType t)
    {
        int idx = (int)t;
        // Fallback на воздух, если индекс поврежден или материал не определен
        if (idx < 0 || idx >= (int)MaterialType::Count) return kMaterialRegistry[0];
        return kMaterialRegistry[idx];
    }

    // =================================================================================================
    // КОНСТАНТЫ И МАТЕМАТИКА
    // =================================================================================================
    static const float PI = 3.1415926535f;

    // =================================================================================================
    // НАПРАВЛЕНИЯ ТРАССИРОВКИ (Uniform Sphere Distribution)
    // =================================================================================================
    // Набор из 46 векторов, равномерно распределенных по сфере.
    // Основан на вершинах геометрических тел (Куб, Октаэдр, Икосаэдр) для исключения "слепых зон"
    // при статическом положении слушателя. В отличие от Random Raycasting, это дает стабильный результат.
    // =================================================================================================
    static const float3 SphereDirections[] = {
        // --- Группа 1: Основные оси (Cardinals) [6 шт] ---
        {1.000000f, 0.000000f, 0.000000f}, {-1.000000f, 0.000000f, 0.000000f},
        {0.000000f, 1.000000f, 0.000000f}, {0.000000f, -1.000000f, 0.000000f},
        {0.000000f, 0.000000f, 1.000000f}, {0.000000f, 0.000000f, -1.000000f},

        // --- Группа 2: Углы Куба (Corners) [8 шт] ---
        // Нормализованные векторы углов (1.0 / sqrt(3) ≈ 0.577350)
        {0.577350f, 0.577350f, 0.577350f},  {0.577350f, 0.577350f, -0.577350f},
        {0.577350f, -0.577350f, 0.577350f}, {0.577350f, -0.577350f, -0.577350f},
        {-0.577350f, 0.577350f, 0.577350f}, {-0.577350f, 0.577350f, -0.577350f},
        {-0.577350f, -0.577350f, 0.577350f},{-0.577350f, -0.577350f, -0.577350f},

        // --- Группа 3: Вершины Икосаэдра (Icosahedron) [12 шт] ---
        // Обеспечивают покрытие "сферических" углов. (0, ±A, ±B)
        {0.000000f, 0.525731f, 0.850651f},  {0.000000f, 0.525731f, -0.850651f},
        {0.000000f, -0.525731f, 0.850651f}, {0.000000f, -0.525731f, -0.850651f},

        {0.850651f, 0.000000f, 0.525731f},  {0.850651f, 0.000000f, -0.525731f},
        {-0.850651f, 0.000000f, 0.525731f}, {-0.850651f, 0.000000f, -0.525731f},

        {0.525731f, 0.850651f, 0.000000f},  {0.525731f, -0.850651f, 0.000000f},
        {-0.525731f, 0.850651f, 0.000000f}, {-0.525731f, -0.850651f, 0.000000f},

        // --- Группа 4: Дополнительные плоскости (Dodecahedron/Edges) [12 шт] ---
        // Заполняют промежутки между основными осями.
        {0.000000f, 0.309017f, 0.951057f},  {0.000000f, 0.309017f, -0.951057f},
        {0.000000f, -0.309017f, 0.951057f}, {0.000000f, -0.309017f, -0.951057f},

        {0.951057f, 0.000000f, 0.309017f},  {0.951057f, 0.000000f, -0.309017f},
        {-0.951057f, 0.000000f, 0.309017f}, {-0.951057f, 0.000000f, -0.309017f},

        {0.309017f, 0.951057f, 0.000000f},  {0.309017f, -0.951057f, 0.000000f},
        {-0.309017f, 0.951057f, 0.000000f}, {-0.309017f, -0.951057f, 0.000000f},

        // --- Группа 5: Промежуточные углы (Mid-points) [8 шт] ---
        // Финальное уплотнение сетки лучей.
        {0.809017f, 0.500000f, 0.309017f},   {0.809017f, 0.500000f, -0.309017f},
        {0.809017f, -0.500000f, 0.309017f},  {0.809017f, -0.500000f, -0.309017f},
        {-0.809017f, 0.500000f, 0.309017f},  {-0.809017f, 0.500000f, -0.309017f},
        {-0.809017f, -0.500000f, 0.309017f}, {-0.809017f, -0.500000f, -0.309017f}
    };

    static const int DIRECTIONS_COUNT = sizeof(SphereDirections) / sizeof(float3);

    // =================================================================================================
    // КОНТЕКСТ АНАЛИЗА (Analysis Context)
    // =================================================================================================
    // Структура для хранения накопленных данных трассировки.
    // Данные аккумулируются на протяжении нескольких кадров для сглаживания результата.
    // =================================================================================================
    struct AnalysisContext
    {
        float3 CameraPosition;

        // --- Накопительные данные (Raw Data) ---
        int Accum_FrameCount = 0;           // Сколько кадров накоплено
        float Accum_TotalDistance = 0.0f;   // Суммарная длина всех лучей
        int Accum_TotalHits = 0;            // Количество лучей, ударившихся о препятствие (для Enclosedness)

        // Статистика попаданий по типам материалов
        int Accum_MaterialHits[(int)MaterialType::Count] = { 0 };

        // --- Рассчитанные параметры (Processed Data) ---
        float PhysicalVolume = 100.0f;      // Оценочный объем помещения (м^3)
        float MeanFreePath = 5.0f;          // Средняя длина свободного пробега звуковой волны
        float PhysicalReflectivity = 0.3f;  // Средний коэффициент отражения стен
        float GeometricOpenness = 0.5f;     // Коэффициент открытости (0 - бункер, 1 - поле)
        float GeometricEnclosedness = 0.5f; // Обратный коэффициент (1 - бункер)
        float ReverbTime = 1.0f;            // RT60: Время затухания звука на 60дБ
        float PerceivedReflectivity = 0.0f; // Психоакустическая отражаемость

        void ResetAccumulation() {
            Accum_FrameCount = 0;
            Accum_TotalDistance = 0.0f;
            Accum_TotalHits = 0;
            std::memset(Accum_MaterialHits, 0, sizeof(Accum_MaterialHits));
        }

        // Вспомогательные математические функции
        static float Lerp(float a, float b, float t) { return a + (b - a) * t; }
        static float Clamp(float v, float min, float max) { return (v < min) ? min : (v > max) ? max : v; }
    };

    // =================================================================================================
    // РЕАЛИЗАЦИЯ СИСТЕМЫ (PIMPL)
    // =================================================================================================
    // Скрытая реализация логики.
    // 
    // ИЗМЕНЕНИЯ ДЛЯ МНОГОПОТОЧНОСТИ (PPL):
    // В отличие от последовательной версии, здесь используется Microsoft Parallel Patterns Library.
    // Трассировка лучей выполняется параллельно на всех доступных ядрах CPU.
    // Для предотвращения гонки данных (Race Conditions) используется паттерн Map-Reduce:
    // 1. Каждый поток имеет свой локальный аккумулятор данных (thread-local AnalysisContext).
    // 2. Генераторы случайных чисел (RNG) создаются уникальными для каждого потока.
    // 3. После завершения всех лучей результаты объединяются (Reduce) в общий контекст.
    // =================================================================================================
    class AudioSystem::Impl
    {
    public:
        // --- Внешние зависимости ---
        IGeometryProvider* m_Provider = nullptr; // Интерфейс к физическому движку игры.
        Settings m_Settings;                     // Глобальные настройки качества/производительности.

        // --- Управление потоком ---
        std::thread* m_WorkerThread = nullptr;   // Основной фоновый поток, управляющий циклом обновлений.
        std::mutex m_Mutex;                      // Защита общих данных (m_TargetPos, m_TargetEAX) от гонки.
        std::condition_variable m_CV;            // Сигнализация потоку о появлении новых данных.
        std::atomic<bool> m_StopThread{ false }; // Флаг для корректной остановки потока при выходе.
        std::atomic<bool> m_WorkPending{ false };// Флаг "есть новая позиция для обработки".

        // --- Состояние симуляции ---
        float3 m_TargetPos;             // Целевая позиция слушателя (получена из Update).
        float3 m_LastProcessedPos;      // Позиция, где был произведен последний сброс аккумуляции.
        float m_CurrentFogDensity = 0.0f; // Текущая плотность тумана (влияет на затухание ВЧ).

        // --- Данные анализа ---
        AnalysisContext m_Context;      // Главный контекст, хранящий объединенные результаты всех кадров.
        EAXResult m_TargetEAX;          // Рассчитанный результат (сырой), готовый к отправке.
        EAXResult m_CurrentEAX;         // Текущий результат (сглаженный), выдаваемый наружу.

        // Конструктор по умолчанию
        Impl() {
            // Инициализация RNG здесь больше не нужна, так как генераторы создаются 
            // локально внутри параллельных потоков для обеспечения потокобезопасности.
        }

        // ---------------------------------------------------------------------------------------------
        // ГЕНЕРАЦИЯ СЛУЧАЙНОГО НАПРАВЛЕНИЯ (Thread-Safe)
        // ---------------------------------------------------------------------------------------------
        // Генерирует вектор в полусфере, ориентированной по нормали поверхности.
        // Используется для расчета диффузных отражений (рассеивания звука).
        //
        // @param normal: Нормаль поверхности, от которой происходит отскок.
        // @param rng: Ссылка на локальный генератор случайных чисел потока.
        // ---------------------------------------------------------------------------------------------
        float3 RandomHemisphereDir(float3 normal, std::mt19937& rng) {
            std::uniform_real_distribution<float> dist(0.0f, 1.0f);
            float3 d;
            // Rejection Sampling: генерируем точки в кубе [-1..1], пока не попадем внутрь сферы.
            // Это дает равномерное распределение, в отличие от простой полярной генерации.
            do {
                d = float3(dist(rng) * 2.0f - 1.0f, dist(rng) * 2.0f - 1.0f, dist(rng) * 2.0f - 1.0f);
            } while (d.magnitude() > 1.0f);

            d = d.normalize();

            // Если вектор смотрит "внутрь" стены (против нормали), разворачиваем его.
            if (d.dot(normal) < 0) d = d * -1.0f;
            return d;
        }

        // ---------------------------------------------------------------------------------------------
        // РЕКУРСИВНАЯ ТРАССИРОВКА (Path Tracing Core)
        // ---------------------------------------------------------------------------------------------
        // Основная функция расчета пути звука. Выполняется параллельно.
        // ВАЖНО: Функция не пишет в this->m_Context во избежание блокировок и гонок данных.
        // Все результаты записываются в localCtx (локальный аккумулятор потока).
        //
        // @param start: Точка начала луча.
        // @param dir: Направление луча.
        // @param depth: Текущая глубина рекурсии (номер отскока).
        // @param energy: Оставшаяся энергия звуковой волны (0.0 - 1.0).
        // @param localCtx: Ссылка на локальный контекст данных текущего потока PPL.
        // @param localRNG: Ссылка на локальный генератор случайных чисел.
        // ---------------------------------------------------------------------------------------------
        void TraceRecursive(const float3& start, const float3& dir, int depth, float energy,
            AnalysisContext& localCtx, std::mt19937& localRNG)
        {
            // Условие выхода: лимит отскоков или энергия затухла ниже порога слышимости (5%)
            if (depth >= m_Settings.maxBounces || energy < 0.05f) return;

            // Запрос к физическому движку (самая тяжелая операция)
            RayHit hit = m_Provider->CastRay(start, dir, m_Settings.maxRayDistance);
            float dist = hit.isHit ? hit.distance : m_Settings.maxRayDistance;

            // [THREAD LOCAL WRITE] Накапливаем дистанцию в локальный контекст потока
            localCtx.Accum_TotalDistance += dist * energy;

            // Enclosedness считаем только по первичным лучам (насколько открыто небо над головой)
            if (depth == 0 && hit.isHit) {
                localCtx.Accum_TotalHits++;
            }

            // Если луч ушел в небо (Sky), рекурсия прерывается
            if (!hit.isHit) return;

            // [THREAD LOCAL WRITE] Статистика материалов
            int matIdx = (int)hit.material;
            if (matIdx >= 0 && matIdx < (int)MaterialType::Count)
                localCtx.Accum_MaterialHits[matIdx]++;

            // Расчет физики отражения
            float absorption = GetMatInfo(hit.material).absorption;
            float newEnergy = energy * (1.0f - absorption);

            // Сдвигаем точку старта чуть назад по нормали (Bias), чтобы избежать самопересечения
            float3 newPos = start;
            newPos.mad(dir, dist - 0.05f);

            // Генерируем случайное направление отскока и уходим в рекурсию
            float3 bounceDir = RandomHemisphereDir(hit.normal, localRNG);
            TraceRecursive(newPos, bounceDir, depth + 1, newEnergy, localCtx, localRNG);
        }

        // ---------------------------------------------------------------------------------------------
        // ФИЗИЧЕСКИЕ РАСЧЕТЫ (Acoustic Parameter Estimation)
        // ---------------------------------------------------------------------------------------------
        // Преобразование накопленной статистики лучей в акустические параметры помещения.
        // Выполняется один раз за кадр после объединения (Reduce) всех данных.
        // ---------------------------------------------------------------------------------------------
        void CalculatePhysics()
        {
            float numRays = (float)DIRECTIONS_COUNT;
            float totalFrames = std::max((float)m_Context.Accum_FrameCount, 1.0f);

            // 1. Mean Free Path (MFP) - Средняя длина свободного пробега
            float totalDist = std::max(m_Context.Accum_TotalDistance, 1.0f);
            float avgDist = totalDist / (totalFrames * numRays);
            m_Context.MeanFreePath = std::max(avgDist, 0.5f);

            // 2. Geometric Enclosedness - Коэффициент замкнутости
            float totalHits = (float)m_Context.Accum_TotalHits / totalFrames;
            float hitRatio = totalHits / numRays;
            m_Context.GeometricEnclosedness = AnalysisContext::Clamp(hitRatio, 0.0f, 1.0f);
            m_Context.GeometricOpenness = 1.0f - m_Context.GeometricEnclosedness;

            // 3. Physical Volume (Sabine approximation)
            // Оценка объема V = 4/3 * PI * R^3, где R = MFP.
            float estVol = (4.0f / 3.0f) * PI * std::pow(m_Context.MeanFreePath, 3.0f);

            // Если мы на улице, ограничиваем виртуальный объем, чтобы эхо не было бесконечным
            if (m_Context.GeometricOpenness > 0.6f)
                estVol = std::min(estVol, 50000.0f);

            m_Context.PhysicalVolume = AnalysisContext::Clamp(estVol, 10.0f, 500000.0f);

            // 4. Physical Reflectivity - Средняя отражаемость стен
            float totalMatHits = 0;
            float accumRefl = 0;

            for (int i = 1; i < (int)MaterialType::Count; ++i) {
                float hits = (float)m_Context.Accum_MaterialHits[i] / totalFrames;
                totalMatHits += hits;
                accumRefl += hits * GetMatInfo((MaterialType)i).rt60_weight;
            }

            float baseReflectivity = (totalMatHits > 0) ? (accumRefl / totalMatHits) : 0.1f;
            // Коррекция: если стен нет (Openness), то и отражений нет
            float effectiveReflectivity = baseReflectivity * (1.0f - m_Context.GeometricOpenness * 0.8f);

            m_Context.PhysicalReflectivity = AnalysisContext::Clamp(effectiveReflectivity, 0.05f, 0.95f);
            m_Context.PerceivedReflectivity = std::pow(m_Context.PhysicalReflectivity, 0.5f);

            // 5. Reverb Time (RT60) - Формула Эйринга (Eyring Equation)
            float surfArea = 6.0f * std::pow(m_Context.PhysicalVolume, 2.0f / 3.0f);
            float absCoeff = AnalysisContext::Clamp(1.0f - m_Context.PhysicalReflectivity, 0.01f, 0.99f);
            float rt60 = 0.161f * m_Context.PhysicalVolume / (-surfArea * std::log(1.0f - absCoeff));

            // --- Эвристические коррекции ---

            // Small Room Fix: В тесных помещениях эхо гаснет быстрее из-за дифракции
            if (m_Context.PhysicalVolume < 150.0f) {
                float smallRoomDamp = std::max(0.3f, m_Context.PhysicalVolume / 150.0f);
                rt60 *= smallRoomDamp;
            }

            // Outdoor Fix: На улице звук уходит вверх
            if (m_Context.GeometricOpenness > 0.4f) {
                float outdoorDamp = 1.0f - (m_Context.GeometricOpenness - 0.4f) * 1.8f;
                rt60 *= AnalysisContext::Clamp(outdoorDamp, 0.1f, 1.0f);
            }

            // Safety Limit
            float maxRT60 = 1.0f + (m_Context.PhysicalVolume / 500.0f);
            rt60 = std::min(rt60, maxRT60);

            m_Context.ReverbTime = AnalysisContext::Clamp(rt60, 0.1f, 8.0f);
        }

        // ---------------------------------------------------------------------------------------------
        // EAX / OpenAL PARAMETERS MAPPING
        // ---------------------------------------------------------------------------------------------
        // Конвертация физических параметров в формат EAX (mB, seconds).
        // Логика идентична последовательной версии, вызывается в конце такта симуляции.
        // ---------------------------------------------------------------------------------------------
        EAXResult CalculateEAXParams()
        {
            EAXResult eax;
            eax.isValid = true;
            eax.debugEnclosedness = m_Context.GeometricEnclosedness;
            eax.debugOpenness = m_Context.GeometricOpenness;

            float fog = AnalysisContext::Clamp(m_CurrentFogDensity, 0.0f, 1.0f);
            float open = m_Context.GeometricOpenness;

            // === Room (Master Volume) ===
            float roomInt = 0.3f + (0.7f * m_Context.GeometricEnclosedness);
            roomInt *= std::sqrt(m_Context.PerceivedReflectivity);

            if (open > 0.4f) {
                float t = AnalysisContext::Clamp((open - 0.4f) / 0.6f, 0.0f, 1.0f);
                roomInt *= AnalysisContext::Clamp(1.0f - (t * t), 0.1f, 1.0f);
            }

            float val = AnalysisContext::Lerp(-4000.0f, 400.0f, AnalysisContext::Clamp(roomInt - (fog * 0.4f), 0.0f, 1.1f));
            eax.lRoom = static_cast<int32_t>(val);

            // Boost для бункеров
            if (m_Context.PhysicalVolume < 70.0f && m_Context.GeometricEnclosedness > 0.8f)
                eax.lRoom += 500;
            eax.lRoom = static_cast<int32_t>(AnalysisContext::Clamp((float)eax.lRoom, -10000.0f, 600.0f));

            // === Decay Time ===
            eax.flDecayTime = m_Context.ReverbTime;
            if (fog > 0.5f) eax.flDecayTime *= 0.8f;

            // === Reflections ===
            float reflScale = (open > 0.6f) ? 0.8f : 1.0f;
            float reflectionsVal = AnalysisContext::Lerp(-2500.0f, 200.0f, m_Context.PhysicalReflectivity * reflScale);
            eax.lReflections = static_cast<int32_t>(AnalysisContext::Clamp(reflectionsVal, -10000.0f, 500.0f));

            float baseDelay = m_Context.MeanFreePath / 340.0f;
            if (open > 0.6f) baseDelay *= 1.5f;
            eax.flReflectionsDelay = AnalysisContext::Clamp(baseDelay, 0.0f, 0.3f);

            // === Reverb ===
            float reverbVal = AnalysisContext::Lerp(-3000.0f, 0.0f, roomInt);
            if (open > 0.7f) reverbVal -= 600.0f;
            eax.lReverb = static_cast<int32_t>(AnalysisContext::Clamp(reverbVal, -10000.0f, 200.0f));
            eax.flReverbDelay = AnalysisContext::Clamp(eax.flReflectionsDelay + 0.04f, 0.0f, 0.1f);

            // === Tone ===
            float hfLoss = -100.0f;
            float outdoorCut = open * -1200.0f;
            float roomHFVal = (float)eax.lRoom + hfLoss + outdoorCut;
            eax.lRoomHF = static_cast<int32_t>(AnalysisContext::Clamp(roomHFVal, -10000.0f, -100.0f));

            eax.flDecayHFRatio = (open > 0.6f) ? 0.5f : 0.83f;
            eax.flEnvironmentDiffusion = (open > 0.7f) ? 0.6f : 1.0f;
            eax.flAirAbsorptionHF = -5.0f + (open * -8.0f) - (fog * 5.0f);
            eax.flEnvironmentSize = std::pow(m_Context.PhysicalVolume, 1.0f / 3.0f);

            return eax;
        }

        // ---------------------------------------------------------------------------------------------
        // РАБОЧИЙ ЦИКЛ ПОТОКА (Worker Thread Loop)
        // ---------------------------------------------------------------------------------------------
        // Основной цикл фонового потока. 
        // 1. Ожидает появления задания (m_WorkPending).
        // 2. Распределяет лучи между ядрами процессора (parallel_for).
        // 3. Собирает результаты и обновляет глобальное состояние.
        // ---------------------------------------------------------------------------------------------
        void ThreadLoop()
        {
            while (!m_StopThread)
            {
                float3 currentPos;

                // --- Блок ожидания (Wait) ---
                {
                    std::unique_lock<std::mutex> lock(m_Mutex);
                    // Спим, пока не придет сигнал Update() или Shutdown()
                    m_CV.wait(lock, [this] { return m_WorkPending || m_StopThread; });

                    if (m_StopThread) break;

                    currentPos = m_TargetPos;
                    m_WorkPending = false;
                }

                // Сброс аккумуляции при перемещении игрока более чем на 0.5 метра
                if (m_LastProcessedPos.distance_to(currentPos) > 0.5f) {
                    m_Context.ResetAccumulation();
                    m_LastProcessedPos = currentPos;
                }
                m_Context.CameraPosition = currentPos;

                // --- ИНИЦИАЛИЗАЦИЯ ЛОКАЛЬНЫХ КОНТЕКСТОВ (Thread Local Storage) ---
                // concurrency::combinable создает коллекцию объектов AnalysisContext,
                // где каждый поток получает свою собственную копию.
                concurrency::combinable<AnalysisContext> threadContexts([]() {
                    AnalysisContext ctx;
                    ctx.ResetAccumulation();
                    return ctx;
                    });

                // --- ПАРАЛЛЕЛЬНАЯ ТРАССИРОВКА (Parallel For) ---
                // Библиотека PPL автоматически разбивает цикл на блоки и раздает их потокам.
                concurrency::parallel_for(0, DIRECTIONS_COUNT, [&](int i) {

                    // 1. Получаем доступ к "личному" аккумулятору текущего потока
                    AnalysisContext& localCtx = threadContexts.local();

                    // 2. Создаем легкий RNG для этого потока.
                    // Seed (зерно) зависит от ID потока и номера итерации, 
                    // чтобы случайные числа были уникальными в каждом потоке.
                    std::hash<std::thread::id> hasher;
                    unsigned int seed = (unsigned int)(hasher(std::this_thread::get_id()) + i + clock());
                    std::mt19937 localRNG(seed);

                    // 3. Запускаем рекурсивную трассировку с локальными данными
                    TraceRecursive(currentPos, SphereDirections[i], 0, 1.0f, localCtx, localRNG);
                    });

                // --- РЕДУКЦИЯ (Reduction) ---
                // Объединяем данные из всех потоков в главный контекст (m_Context).
                threadContexts.combine_each([&](const AnalysisContext& local) {
                    m_Context.Accum_TotalDistance += local.Accum_TotalDistance;
                    m_Context.Accum_TotalHits += local.Accum_TotalHits;

                    // Суммируем хиты по каждому материалу
                    for (int m = 0; m < (int)MaterialType::Count; ++m) {
                        m_Context.Accum_MaterialHits[m] += local.Accum_MaterialHits[m];
                    }
                    });

                m_Context.Accum_FrameCount++; // Увеличиваем счетчик накопленных кадров

                // --- ФИНАЛИЗАЦИЯ ---
                CalculatePhysics();
                EAXResult result = CalculateEAXParams();

                // Публикация результата (под защитой мьютекса)
                {
                    std::lock_guard<std::mutex> lock(m_Mutex);
                    m_TargetEAX = result;
                }

                // Искусственная задержка (~30 Hz), чтобы не грузить CPU на 100%
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
        }
    };

    // =================================================================================================
    // PUBLIC API IMPLEMENTATION
    // =================================================================================================

    AudioSystem::AudioSystem() : m_Impl(new Impl()) {}
    AudioSystem::~AudioSystem() { Shutdown(); delete m_Impl; }

    void AudioSystem::Initialize(IGeometryProvider* p, const Settings& s) {
        if (!p) return; m_Impl->m_Provider = p; m_Impl->m_Settings = s;
        if (s.useMultithreading) m_Impl->m_WorkerThread = new std::thread(&AudioSystem::Impl::ThreadLoop, m_Impl);
    }

    void AudioSystem::Shutdown() {
        if (m_Impl->m_WorkerThread) {
            { std::lock_guard<std::mutex> lock(m_Impl->m_Mutex); m_Impl->m_StopThread = true; }
            m_Impl->m_CV.notify_all();
            if (m_Impl->m_WorkerThread->joinable()) m_Impl->m_WorkerThread->join();
            delete m_Impl->m_WorkerThread; m_Impl->m_WorkerThread = nullptr;
        }
    }

    template <typename T> T LerpEAX(T current, T target, float step) {
        float diff = static_cast<float>(target - current);
        if (std::abs(diff) < 0.1f) return target;
        return static_cast<T>(static_cast<float>(current) + diff * step);
    }

    void AudioSystem::Update(const float3& position, float dt, float envFogDensity) {
        if (!m_Impl->m_WorkerThread) return;
        m_Impl->m_CurrentFogDensity = envFogDensity;

        // Отправка задания в поток
        { std::lock_guard<std::mutex> lock(m_Impl->m_Mutex); m_Impl->m_TargetPos = position; m_Impl->m_WorkPending = true; }
        m_Impl->m_CV.notify_one();

        // Получение результата из потока
        EAXResult target;
        { std::lock_guard<std::mutex> lock(m_Impl->m_Mutex); target = m_Impl->m_TargetEAX; }
        if (!target.isValid) return;

        // Плавная интерполяция значений (Smoothing) во избежание резких скачков звука
        float speed = 2.0f * dt;
        // Если значения изменились радикально (телепорт/вход в бункер), ускоряем интерполяцию
        if (std::abs(target.lRoom - m_Impl->m_CurrentEAX.lRoom) > 1000) speed = 5.0f * dt;

        auto& curr = m_Impl->m_CurrentEAX;
        curr.lRoom = LerpEAX(curr.lRoom, target.lRoom, speed);
        curr.lRoomHF = LerpEAX(curr.lRoomHF, target.lRoomHF, speed);
        curr.lReflections = LerpEAX(curr.lReflections, target.lReflections, speed);
        curr.lReverb = LerpEAX(curr.lReverb, target.lReverb, speed);
        curr.flDecayTime = LerpEAX(curr.flDecayTime, target.flDecayTime, speed);
        curr.flEnvironmentSize = LerpEAX(curr.flEnvironmentSize, target.flEnvironmentSize, speed);

        // Параметры без интерполяции
        curr.flDecayHFRatio = target.flDecayHFRatio;
        curr.flReflectionsDelay = target.flReflectionsDelay;
        curr.flReverbDelay = target.flReverbDelay;
        curr.flAirAbsorptionHF = target.flAirAbsorptionHF;
        curr.flEnvironmentDiffusion = target.flEnvironmentDiffusion;
        curr.isValid = true;
    }

    EAXResult AudioSystem::GetEAXResult() const { return m_Impl->m_CurrentEAX; }

    // =================================================================================================
    // РАСЧЕТ ОККЛЮЗИИ (OCCLUSION CALCULATION)
    // =================================================================================================
    // Определяет, насколько громко слышен источник звука с учетом препятствий.
    // Использует гибридный метод: прямая трассировка (проницаемость) + отражения (дифракция).
    // =================================================================================================
    float AudioSystem::CalculateOcclusion(const float3& listenerPos, const float3& sourcePos)
    {
        if (!m_Impl || !m_Impl->m_Provider) return 1.0f;

        float3 dirToSource = sourcePos - listenerPos;
        float totalDist = dirToSource.magnitude();

        // Если очень близко - считаем, что препятствий нет (борьба с клиппингом камеры)
        if (totalDist < 0.5f) return 1.0f;

        dirToSource = dirToSource.normalize();

        // =============================================================================================
        // ШАГ 1: Multi-Layer Penetration (Пробитие стен)
        // =============================================================================================

        float directEnergy = 1.0f;
        float3 currentPos = listenerPos;

        // Сдвигаем точку старта, чтобы не попасть в коллизию ГГ (голову)
        currentPos.mad(dirToSource, 0.2f);

        float distTravelled = 0.2f; // Сколько уже прошли
        int safeGuard = 0;
        const int MAX_LAYERS = 4; // Максимум 4 стены, дальше тишина

        while (distTravelled < totalDist && safeGuard < MAX_LAYERS)
        {
            float scanDist = totalDist - distTravelled;

            // Пускаем луч
            RayHit hit = m_Impl->m_Provider->CastRay(currentPos, dirToSource, scanDist);

            if (!hit.isHit)
            {
                // Путь чист до самого источника
                break;
            }

            // Мы во что-то попали. Проверяем расстояние от точки удара до ИСТОЧНИКА звука.
            // Вычисляем точку удара
            float3 hitPoint = currentPos;
            hitPoint.mad(dirToSource, hit.distance);

            float distHitToSource = (sourcePos - hitPoint).magnitude();

            // [ВАЖНО] Игнорирование геометрии источника.
            // Если мы попали во что-то, что ближе 40см к источнику звука, 
            // мы считаем, что попали в "тело" монстра или "модель" оружия. Это не стена.
            if (distHitToSource < 0.4f)
            {
                break; // Считаем, что дошли до цели
            }

            // Это реальная стена. Применяем штраф.
            float trans = GetMatInfo(hit.material).transmission;
            directEnergy *= trans;

            // Если энергия стала слишком мала - выходим
            if (directEnergy < 0.05f)
            {
                directEnergy = 0.0f;
                break;
            }

            // ПРОБИВАНИЕ:
            // Перемещаем точку трассировки ЗА стену.
            // 0.3f - это шаг 30см. Это позволяет перепрыгнуть толщину большинства стен в Сталкере.
            float stepSize = 0.3f;

            currentPos = hitPoint;
            currentPos.mad(dirToSource, stepSize);

            distTravelled += (hit.distance + stepSize);
            safeGuard++;
        }

        // Если прямой путь заблокирован полностью, оставляем шанс для отражений
        // Но не позволяем отражениям быть громче 30%, если прямой видимости нет вообще.
        float maxRefl = (directEnergy > 0.1f) ? 0.5f : 0.2f;

        // =============================================================================================
        // ШАГ 2: Отражения (Diffraction approximation)
        // =============================================================================================
        float reflectedEnergy = 0.0f;

        // Считаем отражения, только если прямой звук приглушен.
        if (directEnergy < 0.9f)
        {
            // Упрощенная схема: пускаем меньше лучей для производительности
            const int NUM_RAYS = 4;
            const float CONE_ANGLE = 0.6f;

            static thread_local std::mt19937 rng(std::random_device{}());
            std::uniform_real_distribution<float> dist01(0.0f, 1.0f);

            // Базис
            float3 right = float3(0, 1, 0).cross(dirToSource).normalize();
            if (right.magnitude() < 0.01f) right = float3(1, 0, 0);
            float3 up = dirToSource.cross(right).normalize();

            for (int i = 0; i < NUM_RAYS; ++i)
            {
                float r = dist01(rng) * CONE_ANGLE;
                float theta = dist01(rng) * 2.0f * PI;
                float3 offset = (right * std::cos(theta) + up * std::sin(theta)) * r;
                float3 rayDir = (dirToSource + offset).normalize();

                // Ищем отражающую поверхность рядом
                RayHit hit = m_Impl->m_Provider->CastRay(listenerPos, rayDir, totalDist * 1.2f);

                if (hit.isHit)
                {
                    // Проверяем, видит ли эта стена источник
                    float3 hitPoint = listenerPos; hitPoint.mad(rayDir, hit.distance - 0.1f);
                    float3 toSource = sourcePos - hitPoint;
                    float d = toSource.magnitude();
                    toSource = toSource.normalize();

                    // Check Shadow
                    RayHit shadow = m_Impl->m_Provider->CastRay(hitPoint, toSource, d);

                    // Также игнорируем самопересечение с источником в shadow ray
                    bool clearPath = !shadow.isHit;
                    if (shadow.isHit) {
                        // Если теневой луч попал в сам источник (дистанция от хита до источника мала)
                        float distShadowHitToSource = d - shadow.distance; // Приблизительно
                        if (distShadowHitToSource < 0.5f) clearPath = true;
                    }

                    if (clearPath)
                    {
                        // Отражение найдено
                        float wallRefl = GetMatInfo(hit.material).reflectivity;
                        reflectedEnergy += wallRefl * (1.0f / NUM_RAYS);
                    }
                }
            }
        }

        // Смешиваем. Отражения не могут быть громче maxRefl.
        reflectedEnergy = std::min(reflectedEnergy, maxRefl);

        // Финальная формула: Direct + Reflections
        float total = directEnergy + reflectedEnergy;

        return std::clamp(total, 0.0f, 1.0f);
    }
} // namespace Presence
