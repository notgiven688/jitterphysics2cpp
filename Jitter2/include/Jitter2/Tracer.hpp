#pragma once

#include <chrono>
#include <cstdint>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#ifndef JITTER_ENABLE_PROFILING
#define JITTER_ENABLE_PROFILING 0
#endif

namespace Jitter2
{

enum class TraceCategory : std::uint16_t
{
    General,
    Solver,
    Runtime,
    Invoke,
};

enum class TraceName : std::int64_t
{
    Scope,
    PruneInvalidPairs,
    UpdateBoundingBoxes,
    ScanMoved,
    UpdateProxies,
    ScanOverlaps,
    Gc,
    Step,
    NarrowPhase,
    AddArbiter,
    ReorderContacts,
    CheckDeactivation,
    Solve,
    RemoveArbiter,
    UpdateContacts,
    UpdateBodies,
    BroadPhase,
    Queue,
    PreStep,
    PostStep,
};

enum class TracePhase : std::uint8_t
{
    Begin,
    End,
    Instant,
    Complete,
};

// Provides access to performance tracing features.
// When tracing is enabled, performance data can be recorded during simulation
// and exported for analysis with external tools such as Chrome's tracing viewer.
class Tracer
{
public:
    static constexpr std::string_view DefaultPath = "trace.json";

    static void ProfileBegin(TraceName name, TraceCategory category = TraceCategory::Solver)
    {
#if JITTER_ENABLE_PROFILING
        Record(TraceEvent {
            NowMicroseconds(),
            0.0,
            ThreadId(),
            category,
            TracePhase::Begin,
            name,
        });
#else
        (void)name;
        (void)category;
#endif
    }

    static void ProfileEnd(TraceName name, TraceCategory category = TraceCategory::Solver)
    {
#if JITTER_ENABLE_PROFILING
        Record(TraceEvent {
            NowMicroseconds(),
            0.0,
            ThreadId(),
            category,
            TracePhase::End,
            name,
        });
#else
        (void)name;
        (void)category;
#endif
    }

    static void ProfileEvent(TraceName name, TraceCategory category = TraceCategory::Solver)
    {
#if JITTER_ENABLE_PROFILING
        Record(TraceEvent {
            NowMicroseconds(),
            0.0,
            ThreadId(),
            category,
            TracePhase::Instant,
            name,
        });
#else
        (void)name;
        (void)category;
#endif
    }

    // Starts measuring the duration of a profiling scope on the current thread.
    // Only one scope measurement can be active per thread at a time. Nested calls to
    // ProfileScopeBegin will overwrite the previous start timestamp,
    // resulting in incorrect duration measurements.
    static void ProfileScopeBegin()
    {
#if JITTER_ENABLE_PROFILING
        scopeStartMicro_ = NowMicroseconds();
#endif
    }

    // Ends the current profiling scope and records its duration if it exceeds the given threshold.
    static void ProfileScopeEnd(
        TraceName name = TraceName::Scope,
        TraceCategory category = TraceCategory::Solver,
        double thresholdMicroseconds = 100.0)
    {
#if JITTER_ENABLE_PROFILING
        const double end = NowMicroseconds();
        const double duration = end - scopeStartMicro_;
        if (duration < thresholdMicroseconds)
        {
            return;
        }

        Record(TraceEvent {
            scopeStartMicro_,
            duration,
            ThreadId(),
            category,
            TracePhase::Complete,
            name,
        });
#else
        (void)name;
        (void)category;
        (void)thresholdMicroseconds;
#endif
    }

    // Writes all recorded trace events to a JSON file.
    // filename: The file path to write to. Defaults to trace.json.
    // clear: If true, all recorded data is cleared after writing.
    // If false, recorded data remains in memory.
    // The output file uses the Chrome Trace Event format and can be opened in
    // chrome://tracing or compatible viewers.
    static void WriteToFile(std::string_view filename = DefaultPath, bool clear = true)
    {
        std::ofstream writer(std::string(filename), std::ios::out | std::ios::trunc);
        writer << '[';

#if JITTER_ENABLE_PROFILING
        bool first = true;
        std::lock_guard lock(AllBuffersMutex());
        for (PerThreadBuffer* buffer : AllBuffers())
        {
            for (const TraceEvent& event : buffer->Events)
            {
                if (!first)
                {
                    writer << ',';
                }
                first = false;

                writer << "{\"name\":\"" << ToString(event.Name)
                    << "\",\"cat\":\"" << ToString(event.Category)
                    << "\",\"ph\":\"" << PhaseToChar(event.Phase)
                    << "\",\"ts\":" << event.TimestampMicro;

                if (event.Phase == TracePhase::Complete)
                {
                    writer << ",\"dur\":" << event.DurationMicro;
                }

                writer << ",\"pid\":1,\"tid\":" << event.ThreadId << '}';
            }

            if (clear)
            {
                buffer->Events.clear();
            }
        }
#else
        (void)clear;
#endif

        writer << ']';
    }

private:
    struct TraceEvent
    {
        double TimestampMicro = 0.0;
        double DurationMicro = 0.0;
        int ThreadId = 0;
        TraceCategory Category = TraceCategory::Solver;
        TracePhase Phase = TracePhase::Instant;
        TraceName Name = TraceName::Scope;
    };

    struct PerThreadBuffer
    {
        std::vector<TraceEvent> Events;
    };

    static double NowMicroseconds()
    {
        using Clock = std::chrono::steady_clock;
        return std::chrono::duration<double, std::micro>(Clock::now().time_since_epoch()).count();
    }

    static int ThreadId()
    {
        const auto value = std::hash<std::thread::id> {}(std::this_thread::get_id());
        return static_cast<int>(value & 0x7fffffffU);
    }

    static PerThreadBuffer& EnsureThreadBuffer()
    {
        if (perThreadBuffer_ == nullptr)
        {
            perThreadBuffer_ = new PerThreadBuffer();
            perThreadBuffer_->Events.reserve(InitialCapacity);
            std::lock_guard lock(AllBuffersMutex());
            AllBuffers().push_back(perThreadBuffer_);
        }

        return *perThreadBuffer_;
    }

    static void Record(const TraceEvent& event)
    {
        EnsureThreadBuffer().Events.push_back(event);
    }

    static std::vector<PerThreadBuffer*>& AllBuffers()
    {
        static std::vector<PerThreadBuffer*> buffers;
        return buffers;
    }

    static std::mutex& AllBuffersMutex()
    {
        static std::mutex mutex;
        return mutex;
    }

    static const char* ToString(TraceCategory category)
    {
        switch (category)
        {
        case TraceCategory::General: return "General";
        case TraceCategory::Solver: return "Solver";
        case TraceCategory::Runtime: return "Runtime";
        case TraceCategory::Invoke: return "Invoke";
        }
        return "Unknown";
    }

    static const char* ToString(TraceName name)
    {
        switch (name)
        {
        case TraceName::Scope: return "Scope";
        case TraceName::PruneInvalidPairs: return "PruneInvalidPairs";
        case TraceName::UpdateBoundingBoxes: return "UpdateBoundingBoxes";
        case TraceName::ScanMoved: return "ScanMoved";
        case TraceName::UpdateProxies: return "UpdateProxies";
        case TraceName::ScanOverlaps: return "ScanOverlaps";
        case TraceName::Gc: return "Gc";
        case TraceName::Step: return "Step";
        case TraceName::NarrowPhase: return "NarrowPhase";
        case TraceName::AddArbiter: return "AddArbiter";
        case TraceName::ReorderContacts: return "ReorderContacts";
        case TraceName::CheckDeactivation: return "CheckDeactivation";
        case TraceName::Solve: return "Solve";
        case TraceName::RemoveArbiter: return "RemoveArbiter";
        case TraceName::UpdateContacts: return "UpdateContacts";
        case TraceName::UpdateBodies: return "UpdateBodies";
        case TraceName::BroadPhase: return "BroadPhase";
        case TraceName::Queue: return "Queue";
        case TraceName::PreStep: return "PreStep";
        case TraceName::PostStep: return "PostStep";
        }
        return "Unknown";
    }

    static char PhaseToChar(TracePhase phase)
    {
        switch (phase)
        {
        case TracePhase::Begin: return 'B';
        case TracePhase::End: return 'E';
        case TracePhase::Instant: return 'i';
        case TracePhase::Complete: return 'X';
        }
        return '?';
    }

    static constexpr std::size_t InitialCapacity = 16'384;
    static inline thread_local PerThreadBuffer* perThreadBuffer_ = nullptr;
    static inline thread_local double scopeStartMicro_ = 0.0;
};

} // namespace Jitter2
