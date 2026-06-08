#pragma once

#include <atomic>
#include <thread>

namespace Jitter2::Parallelization
{

class ReaderWriterLock
{
public:

    // Acquires the read lock. Blocks while a writer holds the lock.
    // Multiple threads can hold the read lock simultaneously.
    // Call ExitReadLock to release.
    void EnterReadLock()
    {
        while (true)
        {
            while (writer_.load(std::memory_order_acquire) == 1)
            {
                std::this_thread::yield();
            }

            reader_.fetch_add(1, std::memory_order_acquire);
            if (writer_.load(std::memory_order_acquire) == 0)
            {
                break;
            }

            reader_.fetch_sub(1, std::memory_order_release);
        }
    }

    // Exits the read section.
    void ExitReadLock()
    {
        reader_.fetch_sub(1, std::memory_order_release);
    }

    // Acquires the write lock with exclusive access. Blocks until all readers and writers release.
    // Only one thread can hold the write lock at a time.
    // Call ExitWriteLock to release.
    void EnterWriteLock()
    {
        int expected = 0;
        while (!writer_.compare_exchange_weak(
            expected,
            1,
            std::memory_order_acquire,
            std::memory_order_relaxed))
        {
            expected = 0;
            std::this_thread::yield();
        }

        while (reader_.load(std::memory_order_acquire) != 0)
        {
            std::this_thread::yield();
        }
    }

    // Exits the write section.
    void ExitWriteLock()
    {
        writer_.store(0, std::memory_order_release);
    }

private:
    std::atomic<int> writer_ {0};
    std::atomic<int> reader_ {0};
};

} // namespace Jitter2::Parallelization
