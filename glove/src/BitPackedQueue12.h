#ifndef BIT_PACKED_QUEUE12_H
#define BIT_PACKED_QUEUE12_H

#include <stdint.h>   // for uint8_t, uint16_t
#include <string.h>   // for memset (optional)

// A ring buffer queue holding up to CAPACITY 12-bit samples,
// all stored in a static 900-byte array (no dynamic memory).
class BitPackedQueue12 {
public:
    // For 30 seconds at 20 Hz => 600 samples.
    static const size_t CAPACITY = 600;
    static const size_t BITS_PER_SAMPLE = 12;

    // Total bits = 600 * 12 = 7200. => 7200 / 8 = 900 bytes
    static const size_t BYTES_NEEDED = (CAPACITY * BITS_PER_SAMPLE + 7) / 8;

    // Constructor: clear by default
    BitPackedQueue12() {
        clear();
    }

    // Remove all samples (make empty).
    void clear() {
        readIndex_  = 0;
        writeIndex_ = 0;
        count_      = 0;
        // If desired, zero out data_ for debugging:
        // memset(data_, 0, BYTES_NEEDED);
    }

    // Push a 12-bit value into the queue.
    // Returns false if already full.
    bool push(uint16_t sample12) {
        if (count_ >= CAPACITY) {
            // Queue is full
            return false;
        }
        store12bits(writeIndex_, sample12);
        writeIndex_ = (writeIndex_ + 1) % CAPACITY;
        count_++;
        return true;
    }

    // Pop (FIFO) the oldest 12-bit value into outSample.
    // Returns false if empty.
    bool pop(uint16_t &outSample) {
        if (count_ == 0) {
            // Queue is empty
            return false;
        }
        outSample = load12bits(readIndex_);
        readIndex_ = (readIndex_ + 1) % CAPACITY;
        count_--;
        return true;
    }

    // Check if empty
    bool empty() const {
        return (count_ == 0);
    }

    // Check if full
    bool full() const {
        return (count_ >= CAPACITY);
    }

    // Current number of 12-bit samples stored
    size_t size() const {
        return count_;
    }

    // Maximum number of samples storable
    size_t capacity() const {
        return CAPACITY;
    }

private:
    // Static array for all samples
    uint8_t data_[BYTES_NEEDED];

    // Ring buffer indices + count
    size_t readIndex_;
    size_t writeIndex_;
    size_t count_;

    // Store 12 bits at ring position "index"
    void store12bits(size_t index, uint16_t value) {
        const size_t bitOffset = index * BITS_PER_SAMPLE; // 12 * index
        const size_t bytePos   = bitOffset / 8;
        const size_t shift     = bitOffset % 8;

        // Gather up to 3 bytes from data_
        uint32_t b0 = (bytePos     < BYTES_NEEDED) ? data_[bytePos]     : 0;
        uint32_t b1 = (bytePos + 1 < BYTES_NEEDED) ? data_[bytePos + 1] : 0;
        uint32_t b2 = (bytePos + 2 < BYTES_NEEDED) ? data_[bytePos + 2] : 0;

        // Combine into 24 bits
        uint32_t combined = (b2 << 16) | (b1 << 8) | b0;

        // Clear out the 12-bit region
        // We'll overlay (value & 0xFFF) << shift
        const uint32_t mask12 = 0xFFFu << shift;
        combined &= ~mask12; // zero that region
        combined |= (static_cast<uint32_t>(value & 0xFFFu) << shift);

        // Write back
        if (bytePos     < BYTES_NEEDED) data_[bytePos]     =  (combined      ) & 0xFF;
        if (bytePos + 1 < BYTES_NEEDED) data_[bytePos + 1] =  (combined >> 8 ) & 0xFF;
        if (bytePos + 2 < BYTES_NEEDED) data_[bytePos + 2] =  (combined >> 16) & 0xFF;
    }

    // Load 12 bits from ring position "index"
    uint16_t load12bits(size_t index) const {
        const size_t bitOffset = index * BITS_PER_SAMPLE;
        const size_t bytePos   = bitOffset / 8;
        const size_t shift     = bitOffset % 8;

        // Read up to 3 bytes
        uint32_t b0 = (bytePos     < BYTES_NEEDED) ? data_[bytePos]     : 0;
        uint32_t b1 = (bytePos + 1 < BYTES_NEEDED) ? data_[bytePos + 1] : 0;
        uint32_t b2 = (bytePos + 2 < BYTES_NEEDED) ? data_[bytePos + 2] : 0;

        uint32_t combined = (b2 << 16) | (b1 << 8) | b0;
        uint16_t result = static_cast<uint16_t>((combined >> shift) & 0xFFF);
        return result;
    }
};

#endif // BIT_PACKED_QUEUE12_H
