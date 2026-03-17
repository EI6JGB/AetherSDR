#pragma once

#include <atomic>
#include <cstddef>
#include <cstring>
#include <algorithm>

// Lock-free single-producer single-consumer ring buffer.
// Thread-safe when exactly one thread writes and one thread reads.
// Uses power-of-two size for efficient masking.
class SpscRingBuffer {
public:
    explicit SpscRingBuffer(size_t minSize = 131072)
    {
        // Round up to next power of two
        m_capacity = 1;
        while (m_capacity < minSize) m_capacity <<= 1;
        m_data = new char[m_capacity];
        m_mask = m_capacity - 1;
    }

    ~SpscRingBuffer() { delete[] m_data; }

    // Non-copyable
    SpscRingBuffer(const SpscRingBuffer&) = delete;
    SpscRingBuffer& operator=(const SpscRingBuffer&) = delete;

    size_t capacity() const { return m_capacity; }

    size_t writeAvailable() const
    {
        const size_t w = m_writePos.load(std::memory_order_relaxed);
        const size_t r = m_readPos.load(std::memory_order_acquire);
        return m_capacity - (w - r);
    }

    size_t readAvailable() const
    {
        const size_t w = m_writePos.load(std::memory_order_acquire);
        const size_t r = m_readPos.load(std::memory_order_relaxed);
        return w - r;
    }

    // Write up to `bytes` into the buffer. Returns bytes actually written.
    size_t write(const void* src, size_t bytes)
    {
        const size_t avail = writeAvailable();
        bytes = std::min(bytes, avail);
        if (bytes == 0) return 0;

        const size_t w = m_writePos.load(std::memory_order_relaxed);
        const size_t pos = w & m_mask;
        const size_t firstChunk = std::min(bytes, m_capacity - pos);

        std::memcpy(m_data + pos, src, firstChunk);
        if (firstChunk < bytes)
            std::memcpy(m_data, static_cast<const char*>(src) + firstChunk, bytes - firstChunk);

        m_writePos.store(w + bytes, std::memory_order_release);
        return bytes;
    }

    // Read up to `bytes` from the buffer. Returns bytes actually read.
    size_t read(void* dst, size_t bytes)
    {
        const size_t avail = readAvailable();
        bytes = std::min(bytes, avail);
        if (bytes == 0) return 0;

        const size_t r = m_readPos.load(std::memory_order_relaxed);
        const size_t pos = r & m_mask;
        const size_t firstChunk = std::min(bytes, m_capacity - pos);

        std::memcpy(dst, m_data + pos, firstChunk);
        if (firstChunk < bytes)
            std::memcpy(static_cast<char*>(dst) + firstChunk, m_data, bytes - firstChunk);

        m_readPos.store(r + bytes, std::memory_order_release);
        return bytes;
    }

    void reset()
    {
        m_readPos.store(0, std::memory_order_relaxed);
        m_writePos.store(0, std::memory_order_relaxed);
    }

private:
    char*  m_data{nullptr};
    size_t m_capacity{0};
    size_t m_mask{0};
    alignas(64) std::atomic<size_t> m_writePos{0};
    alignas(64) std::atomic<size_t> m_readPos{0};
};
