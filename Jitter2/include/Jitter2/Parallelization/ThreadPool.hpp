#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <Jitter2/Parallelization/Parallel.hpp>

namespace Jitter2::Parallelization
{

class ThreadPool
{
public:
    static constexpr float ThreadsPerProcessor = 0.9f;

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;
    ThreadPool& operator=(ThreadPool&&) = delete;
    ~ThreadPool();

    [[nodiscard]] static ThreadPool& Instance();
    [[nodiscard]] static bool InstanceInitialized();
    [[nodiscard]] static int ThreadCountSuggestion();

    [[nodiscard]] int ThreadCount() const;
    [[nodiscard]] bool IsPaused() const;

    // Changes the number of worker threads.
    // numThreads: The new thread count.
    // Existing worker threads are stopped and new ones are created.
    // This operation blocks until all previous threads have terminated.
    void ChangeThreadCount(int numThreads);
    void AddTask(const std::function<void(Batch)>& action, Batch parameter);

    // Executes all queued tasks and blocks until completion.
    // Tasks are distributed to per-thread queues in round-robin order. The main thread
    // participates as worker 0 and performs work stealing from other queues.
    // This method automatically calls ResumeWorkers at the start.
    void Execute();

    // Resumes all worker threads so they can process queued tasks.
    // Called automatically by Execute. Manual calls are typically only needed
    // when using World.ThreadModelType.Persistent.
    void ResumeWorkers();

    // Pauses all worker threads after they finish their current tasks.
    // Workers will block on a wait handle until ResumeWorkers is called.
    // This reduces CPU usage between simulation steps.
    void PauseWorkers();

private:
    struct Task
    {
        std::function<void(Batch)> Action;
        Batch Parameter;
    };

    ThreadPool();

    void ThreadProc(int index);
    bool TryDequeue(int index, Task& task);
    void Enqueue(int index, Task task);
    void DrainQueue(int index, int& performedTasks);
    void Steal(int index);
    void PerformTask(Task& task);
    void StopThreads();
    void StartThreads(int numThreads);

    struct WorkQueue
    {
        std::mutex Mutex;
        std::deque<Task> Tasks;
    };

    mutable std::mutex mutex_;
    std::condition_variable workAvailable_;
    std::vector<std::thread> threads_;
    std::vector<Task> taskList_;
    std::vector<std::unique_ptr<WorkQueue>> queues_;
    std::exception_ptr capturedException_;
    bool running_ = true;
    bool workersPaused_ = false;
    std::atomic<int> tasksLeft_ {0};
    int threadCount_ = 0;
};

} // namespace Jitter2::Parallelization
