#pragma once

#include "HelperFuncs.h"
#include "BitPackedQueue12.h"

static int testTimestep = 0;
static int testForwardPin = 25;
static int testBackwardPin = 26;
static int testPotiPin = 36;

inline void testMotor() {
    testTimestep++;
    float sine = sin(testTimestep * 0.001);
    float mappedValue = mapFloat(sine, -1.0, 1.0, -1023, 1023);
    Serial.print(">mappedValue:");
    Serial.println(mappedValue);
    if (mappedValue > 0) {
        analogWrite(testForwardPin, mappedValue);
        analogWrite(testBackwardPin, 0);
    } else {
        analogWrite(testForwardPin, 0);
        analogWrite(testBackwardPin, abs(mappedValue));
    }
}

inline void testPoti() {
    int value = analogRead(testPotiPin);
    Serial.print(">value:");
    Serial.println(value);
}

void testBitPackedQueue12() {
  Serial.println("=== Testing BitPackedQueue12 ===");

  BitPackedQueue12 queue;
  static const size_t cap = BitPackedQueue12::CAPACITY;
  
  // For testing, we will push a total of 700 samples:
  // - Phase 1: Push half capacity (300 samples)
  // - Phase 2: Pop 100 samples
  // - Phase 3: Push additional samples to fill the queue (600 - (300-100) = 400 samples)
  //
  // The FIFO order after these operations should be:
  //   Phase 1 leftover samples (indices 100..299) followed by
  //   Phase 3 samples (indices 300..699).
  //
  // Since we will eventually compare 600 popped samples to a reference array,
  // the reference array must be sized to hold all 700 samples.
  const size_t phase1Count = cap / 2;        // 300
  const size_t phase2Count = phase1Count / 3;  // 100
  const size_t phase3Count = cap - (phase1Count - phase2Count); // 600 - (300-100) = 400
  const size_t totalSamples = phase1Count + phase3Count;         // 300 + 400 = 700
  
  uint16_t reference[totalSamples]; // Holds all samples in logical order

  // --- Phase 1: Push 300 samples ---
  Serial.print("Phase 1: pushing ");
  Serial.print(phase1Count);
  Serial.println(" samples...");
  for (size_t i = 0; i < phase1Count; i++) {
    // Generate a 12-bit sample: (i * 7) mod 4096
    uint16_t sample = static_cast<uint16_t>((i * 7) & 0x0FFF);
    reference[i] = sample;
    if (!queue.push(sample)) {
      Serial.println("ERROR: push failed in Phase 1!");
      return;
    }
  }
  if (queue.size() != phase1Count) {
    Serial.println("ERROR: size mismatch after Phase 1!");
    return;
  }

  // --- Phase 2: Pop 100 samples ---
  Serial.print("Phase 2: popping ");
  Serial.print(phase2Count);
  Serial.println(" samples...");
  for (size_t i = 0; i < phase2Count; i++) {
    uint16_t outVal;
    if (!queue.pop(outVal)) {
      Serial.println("ERROR: pop failed in Phase 2!");
      return;
    }
    // The expected value is the i-th sample that was pushed
    if (outVal != reference[i]) {
      Serial.print("ERROR: Phase 2 mismatch at i=");
      Serial.print(i);
      Serial.print(" expected=");
      Serial.print(reference[i]);
      Serial.print(" got=");
      Serial.println(outVal);
      return;
    }
  }
  size_t remainingAfterPhase2 = phase1Count - phase2Count; // should be 300 - 100 = 200
  if (queue.size() != remainingAfterPhase2) {
    Serial.println("ERROR: size mismatch after Phase 2!");
    return;
  }

  // --- Phase 3: Push additional samples until the queue is full ---
  Serial.print("Phase 3: pushing ");
  Serial.print(phase3Count);
  Serial.println(" samples...");
  // New samples will be stored in reference[phase1Count ... totalSamples-1] (i.e. indices 300 to 699)
  for (size_t i = 0; i < phase3Count; i++) {
    size_t idx = phase1Count + i; // Logical index for this new sample
    uint16_t sample = static_cast<uint16_t>((idx * 7) & 0x0FFF);
    reference[idx] = sample;
    if (!queue.push(sample)) {
      Serial.println("ERROR: push failed in Phase 3!");
      return;
    }
  }
  if (!queue.full() || queue.size() != cap) {
    Serial.println("ERROR: queue not full after Phase 3!");
    return;
  }
  // An extra push must fail because the queue is full.
  if (queue.push(0x123)) {
    Serial.println("ERROR: extra push succeeded even though queue is full!");
    return;
  }

  // --- Phase 4: Pop all 600 samples to verify FIFO order ---
  Serial.print("Phase 4: popping all ");
  Serial.print(cap);
  Serial.println(" samples to verify FIFO order...");
  // The FIFO order is:
  //  - The remaining Phase 1 samples: reference[phase2Count] to reference[phase1Count-1]
  //  - Then the Phase 3 samples: reference[phase1Count] to reference[totalSamples-1]
  // Thus, when popping, the expected sample for the i-th pop is reference[i + phase2Count].
  for (size_t i = 0; i < cap; i++) {
    uint16_t outVal;
    if (!queue.pop(outVal)) {
      Serial.println("ERROR: pop failed in Phase 4!");
      return;
    }
    size_t expectedIdx = i + phase2Count; // First pop should compare to reference[phase2Count]
    uint16_t expected = reference[expectedIdx];
    if (outVal != expected) {
      Serial.print("ERROR: mismatch on final pop at i=");
      Serial.print(i);
      Serial.print(" expected=");
      Serial.print(expected);
      Serial.print(" got=");
      Serial.println(outVal);
      return;
    }
  }
  if (!queue.empty()) {
    Serial.println("ERROR: queue not empty after final pops!");
    return;
  }

  // --- Optional: Test clear() functionality ---
  queue.push(0xAAA); // push a sample
  queue.clear();
  if (!queue.empty() || queue.size() != 0) {
    Serial.println("ERROR: queue did not clear properly!");
    return;
  }

  Serial.println("All tests PASSED for BitPackedQueue12!");
}