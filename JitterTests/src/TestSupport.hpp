#pragma once

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Jitter2/Precision.hpp>

#if JITTER_USE_CATCH2
#include <catch2/catch_test_macros.hpp>
#endif

namespace JitterTests
{

inline void Require(bool condition, const char* message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

inline void RequireClose(Jitter2::Real actual, Jitter2::Real expected, Jitter2::Real epsilon, const char* message)
{
    if (std::abs(actual - expected) > epsilon)
    {
        throw std::runtime_error(std::string(message) + ": actual=" + std::to_string(actual)
            + " expected=" + std::to_string(expected));
    }
}

#if !JITTER_USE_CATCH2

using TestFunction = void (*)();

inline std::vector<std::pair<std::string, TestFunction>>& Registry()
{
    static std::vector<std::pair<std::string, TestFunction>> registry;
    return registry;
}

struct Registrar
{
    Registrar(const char* name, TestFunction function)
    {
        Registry().emplace_back(name, function);
    }
};

inline int RunAll()
{
    int failures = 0;
    for (const auto& [name, function] : Registry())
    {
        try
        {
            function();
            std::cout << "[pass] " << name << '\n';
        }
        catch (const std::exception& ex)
        {
            ++failures;
            std::cerr << "[fail] " << name << ": " << ex.what() << '\n';
        }
    }

    return failures == 0 ? 0 : 1;
}

#endif

} // namespace JitterTests

#if JITTER_USE_CATCH2
#define JITTER_TEST_CASE(name) TEST_CASE(name)
#else
#define JITTER_DETAIL_CONCAT_IMPL(left, right) left##right
#define JITTER_DETAIL_CONCAT(left, right) JITTER_DETAIL_CONCAT_IMPL(left, right)
#define JITTER_TEST_CASE_IMPL(name, line) \
    static void JITTER_DETAIL_CONCAT(jitter_test_, line)(); \
    static ::JitterTests::Registrar JITTER_DETAIL_CONCAT(jitter_registrar_, line)(name, &JITTER_DETAIL_CONCAT(jitter_test_, line)); \
    static void JITTER_DETAIL_CONCAT(jitter_test_, line)()
#define JITTER_TEST_CASE(name) JITTER_TEST_CASE_IMPL(name, __LINE__)
#endif
