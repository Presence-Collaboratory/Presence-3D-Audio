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
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <mutex>
#include <iomanip>

class Logger {
public:
    static void Init(const std::string& filename) {
        GetInstance().fileStream.open(filename, std::ios::out | std::ios::trunc);
    }

    static void Log(const std::string& msg, bool isError = false) {
        std::lock_guard<std::mutex> lock(GetInstance().logMutex);

        // Вывод в консоль с цветом (упрощенно для Windows)
        if (isError) std::cerr << "[ERROR] " << msg << std::endl;
        else         std::cout << "[INFO]  " << msg << std::endl;

        // Вывод в файл
        if (GetInstance().fileStream.is_open()) {
            GetInstance().fileStream << (isError ? "[ERROR] " : "[INFO]  ") << msg << std::endl;
            GetInstance().fileStream.flush(); // Чтобы данные не потерялись при краше
        }
    }

    static void LogParam(const std::string& name, float value) {
        std::lock_guard<std::mutex> lock(GetInstance().logMutex);
        std::stringstream ss;
        ss << std::left << std::setw(25) << name << ": " << std::fixed << std::setprecision(4) << value;

        std::cout << ss.str() << std::endl;
        if (GetInstance().fileStream.is_open()) GetInstance().fileStream << ss.str() << std::endl;
    }

private:
    std::ofstream fileStream;
    std::mutex logMutex;

    // Singleton pattern
    static Logger& GetInstance() {
        static Logger instance;
        return instance;
    }

    Logger() {}
    ~Logger() { if (fileStream.is_open()) fileStream.close(); }
    Logger(const Logger&) = delete;
    void operator=(const Logger&) = delete;
};
