#pragma once

#include <array>
#include <functional>
#include <sstream>
#include <string>
#include <string_view>

namespace Jitter2
{

// Provides logging functionality.
class Logger
{
public:
    enum class LogLevel
    {
        Information,
        Warning,
        Error
    };

    using ListenerFunction = std::function<void(LogLevel, const std::string&)>;

    static inline ListenerFunction Listener {};

    static void Information(std::string_view format)
    {

    // Internal logging method that invokes all registered listeners with the given message.
    // level: The log level of the message.
    // format: The message to log.
        Log(LogLevel::Information, format);
    }

    template<typename... TArgs>
    static void Information(std::string_view format, const TArgs&... args)
    {

        LogFormat(LogLevel::Information, format, args...);
    }

    static void Warning(std::string_view format)
    {
        Log(LogLevel::Warning, format);
    }

    template<typename... TArgs>
    static void Warning(std::string_view format, const TArgs&... args)
    {

        LogFormat(LogLevel::Warning, format, args...);
    }

    static void Error(std::string_view format)
    {
        Log(LogLevel::Error, format);
    }

    template<typename... TArgs>
    static void Error(std::string_view format, const TArgs&... args)
    {

        LogFormat(LogLevel::Error, format, args...);
    }

private:
    static void Log(LogLevel level, std::string_view format)
    {
        if (Listener)
        {
            Listener(level, std::string(format));
        }
    }

    template<typename T>
    static std::string ToString(const T& value)
    {
        std::ostringstream stream;
        stream << value;
        return stream.str();
    }

    static void ReplaceAll(std::string& text, const std::string& token, const std::string& value)
    {
        std::size_t position = 0;
        while ((position = text.find(token, position)) != std::string::npos)
        {
            text.replace(position, token.size(), value);
            position += value.size();
        }
    }

    // Formats a log message with one argument and invokes the listeners.
    template<typename... TArgs>
    static void LogFormat(LogLevel level, std::string_view format, const TArgs&... args)
    {
        if (!Listener)
        {
            return;
        }

        std::string message(format);
        const std::array<std::string, sizeof...(TArgs)> values {ToString(args)...};

        for (std::size_t index = 0; index < values.size(); ++index)
        {
            ReplaceAll(message, "{" + std::to_string(index) + "}", values[index]);
        }

        Listener(level, message);
    }
};

} // namespace Jitter2
