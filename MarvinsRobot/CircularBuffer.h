#pragma once
#include <array>
#include <mutex>
#include <stdexcept>
#include <vector>
#include <utility>

template <typename T, std::size_t size>
class CircularBuffer
{
    mutable std::mutex mutex;
    std::size_t index = 0; // index of next add
    std::array<T, size> buffer;
public:
    // Filled
    CircularBuffer(T fill_value = T{}) { buffer.fill(fill_value); }

    void add(T item)
    {
        std::scoped_lock lock(mutex);
        buffer[index] = std::move(item);
        index = (index + 1) % size;
    }


    // returns the last n values in reverse order of arrival.
    std::vector<T> get_snapshot(std::size_t n = size) const
    {
        if (n > size)
            throw std::runtime_error("Request exceeds buffer capacity.");

        std::vector<T> ret;
        ret.reserve(n);
        std::scoped_lock lock(mutex);
        for (std::size_t i = 0; i < n; i++)
            ret.push_back(buffer[(index - n + i + size) % size]);
        return ret;
    }
};