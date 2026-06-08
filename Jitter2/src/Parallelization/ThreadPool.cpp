#include <Jitter2/Parallelization/ThreadPool.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <thread>

#if defined(__i386__) || defined(__x86_64__) || defined(_M_IX86) || defined(_M_X64)
#include <immintrin.h>
#endif

namespace Jitter2::Parallelization
{

namespace
{

std::mutex InstanceMutex;
std::unique_ptr<ThreadPool> InstanceStorage;

void SpinWaitOnce()
{
#if defined(__i386__) || defined(__x86_64__) || defined(_M_IX86) || defined(_M_X64)
    _mm_pause();
#else
    std::this_thread::yield();
#endif
}

} // namespace

void GetBounds(int numElements, int numDivisions, int part, int& start, int& end)
{
    if (numDivisions <= 0)
    {
        throw std::invalid_argument("Number of divisions must be positive.");
    }

    if (part < 0 || part >= numDivisions)
    {
        throw std::out_of_range("Part index is outside the number of divisions.");
    }

    const int div = numElements / numDivisions;
    const int mod = numElements % numDivisions;

    start = div * part + std::min(part, mod);
    end = start + div + (part < mod ? 1 : 0);
}

void ForBatch(
    int lower,
    int upper,
    int numTasks,
    const std::function<void(Batch)>& action,
    bool execute)
{
    if (upper < lower)
    {
        throw std::invalid_argument("Upper bound must be greater than or equal to lower bound.");
    }

    if (numTasks <= 0)
    {
        return;
    }

#if JITTER_ENABLE_MULTITHREADING
    ThreadPool& threadPool = ThreadPool::Instance();
    for (int i = 0; i < numTasks; ++i)
    {
        int start = 0;
        int end = 0;
        GetBounds(upper - lower, numTasks, i, start, end);
        threadPool.AddTask(action, Batch {start + lower, end + lower});
    }

    if (execute)
    {
        threadPool.Execute();
    }
#else
    for (int i = 0; i < numTasks; ++i)
    {
        int start = 0;
        int end = 0;
        GetBounds(upper - lower, numTasks, i, start, end);
        action(Batch {start + lower, end + lower});
    }

    (void)execute;
#endif
}

ThreadPool& ThreadPool::Instance()
{
    std::lock_guard lock(InstanceMutex);
    if (!InstanceStorage)
    {
        InstanceStorage.reset(new ThreadPool());
    }

    return *InstanceStorage;
}

bool ThreadPool::InstanceInitialized()
{
    std::lock_guard lock(InstanceMutex);
    return InstanceStorage != nullptr;
}

int ThreadPool::ThreadCountSuggestion()
{
    const unsigned int hardware = std::thread::hardware_concurrency();
    if (hardware == 0)
    {
        return 1;
    }

    return std::max(1, static_cast<int>(std::floor(static_cast<float>(hardware) * ThreadsPerProcessor)));
}

ThreadPool::ThreadPool()
{
#if JITTER_ENABLE_MULTITHREADING
    ChangeThreadCount(ThreadCountSuggestion());
#else
    ChangeThreadCount(1);
#endif
}

ThreadPool::~ThreadPool()
{
    StopThreads();
}

int ThreadPool::ThreadCount() const
{
    std::lock_guard lock(mutex_);
    return threadCount_;
}

bool ThreadPool::IsPaused() const
{
    std::lock_guard lock(mutex_);
    return workersPaused_;
}

void ThreadPool::ChangeThreadCount(int numThreads)
{
#if !JITTER_ENABLE_MULTITHREADING
    numThreads = 1;
#endif

    numThreads = std::max(1, numThreads);

    {
        std::lock_guard lock(mutex_);
        if (numThreads == threadCount_)
        {
            return;
        }
    }

    StopThreads();
    StartThreads(numThreads);
    PauseWorkers();
}

void ThreadPool::AddTask(const std::function<void(Batch)>& action, Batch parameter)
{
    taskList_.push_back(Task {action, parameter});
}

void ThreadPool::Execute()
{
    ResumeWorkers();

    int executeThreadCount = 1;
    int totalTasks = 0;
    {
        std::lock_guard lock(mutex_);
        capturedException_ = nullptr;
        totalTasks = static_cast<int>(taskList_.size());
        tasksLeft_.store(totalTasks, std::memory_order_release);
        executeThreadCount = threadCount_;
    }

    for (int i = 0; i < totalTasks; ++i)
    {
        Enqueue(i % executeThreadCount, std::move(taskList_[static_cast<std::size_t>(i)]));
    }

    taskList_.clear();

    // The main thread owns queue 0.
    int performedTasks = 0;
    DrainQueue(0, performedTasks);

    // Only after finishing local work do we try to steal from the other queues.
    Steal(0);

    while (tasksLeft_.load(std::memory_order_acquire) > 0)
    {
        SpinWaitOnce();
    }

    std::exception_ptr exception;
    {
        std::lock_guard lock(mutex_);
        exception = capturedException_;
    }

    if (exception)
    {
        PauseWorkers();
        std::rethrow_exception(exception);
    }
}

void ThreadPool::ResumeWorkers()
{
    {
        std::lock_guard lock(mutex_);
        workersPaused_ = false;
    }

    workAvailable_.notify_all();
}

void ThreadPool::PauseWorkers()
{
    std::lock_guard lock(mutex_);
    workersPaused_ = true;
}

void ThreadPool::ThreadProc(int index)
{
    while (true)
    {
        int performedTasks = 0;
        DrainQueue(index, performedTasks);

        if (performedTasks > 0)
        {
      // Finish local work before stealing from the other queues.
            Steal(index);
        }

        std::this_thread::yield();

        {
            std::unique_lock lock(mutex_);
            workAvailable_.wait(lock, [this]
            {
                return !running_ || !workersPaused_;
            });

            if (!running_)
            {
                return;
            }
        }
    }
}

bool ThreadPool::TryDequeue(int index, Task& task)
{
    WorkQueue& queue = *queues_[static_cast<std::size_t>(index)];
    std::lock_guard lock(queue.Mutex);
    if (queue.Tasks.empty())
    {
        return false;
    }

    task = std::move(queue.Tasks.front());
    queue.Tasks.pop_front();
    return true;
}

void ThreadPool::Enqueue(int index, Task task)
{
    WorkQueue& queue = *queues_[static_cast<std::size_t>(index)];
    std::lock_guard lock(queue.Mutex);
    queue.Tasks.push_back(std::move(task));
}

void ThreadPool::DrainQueue(int index, int& performedTasks)
{
    Task task;
    while (TryDequeue(index, task))
    {
        PerformTask(task);
        ++performedTasks;
    }
}

void ThreadPool::Steal(int index)
{
    const int queueCount = static_cast<int>(queues_.size());
    for (int i = 1; i < queueCount; ++i)
    {
        int performedTasks = 0;
        const int queueIndex = (i + index) % queueCount;
        DrainQueue(queueIndex, performedTasks);
    }
}

void ThreadPool::PerformTask(Task& task)
{
    try
    {
        task.Action(task.Parameter);
    }
    catch (...)
    {
        std::lock_guard lock(mutex_);
        if (!capturedException_)
        {
            capturedException_ = std::current_exception();
        }
    }

    tasksLeft_.fetch_sub(1, std::memory_order_acq_rel);
}

void ThreadPool::StopThreads()
{
    std::vector<std::thread> threads;
    {
        std::lock_guard lock(mutex_);
        running_ = false;
        workersPaused_ = false;
        threads = std::move(threads_);
        taskList_.clear();
        tasksLeft_.store(0, std::memory_order_release);
    }

    workAvailable_.notify_all();
    for (std::thread& thread : threads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }

    {
        std::lock_guard lock(mutex_);
        queues_.clear();
        threadCount_ = 0;
        running_ = true;
    }
}

void ThreadPool::StartThreads(int numThreads)
{
    std::lock_guard lock(mutex_);
    threadCount_ = numThreads;
    threads_.clear();
    threads_.reserve(static_cast<std::size_t>(std::max(0, threadCount_ - 1)));
    queues_.clear();
    queues_.reserve(static_cast<std::size_t>(threadCount_));
    for (int i = 0; i < threadCount_; ++i)
    {
        queues_.push_back(std::make_unique<WorkQueue>());
    }

    running_ = true;
    workersPaused_ = false;

#if JITTER_ENABLE_MULTITHREADING
    for (int i = 1; i < threadCount_; ++i)
    {
        threads_.emplace_back([this, i]
        {
            ThreadProc(i);
        });
    }
#endif
}

} // namespace Jitter2::Parallelization
