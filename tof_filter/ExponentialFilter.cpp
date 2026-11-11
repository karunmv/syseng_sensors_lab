
#include "ExponentialFilter.h"

ExponentialFilter::ExponentialFilter(
    float a, int window
) {
  alpha = constrain(a, 0.001f, 1.0f);
  windowSize = constrain(window, 1, MAX_SAMPLES);
  reset();
}

void ExponentialFilter::reset() {
  count = 0;
  nextIndex = 0;
}

float ExponentialFilter::update(float measurement) {

  // Store new measurement
  samples[nextIndex] = measurement;
  nextIndex = (nextIndex + 1) % windowSize;

  if (count < windowSize) {
    count++;
  }

  float weightedSum = 0;
  float weightSum = 0;
  float weight = 1.0f;

  // Newest sample has highest weight
  for (int i = 0; i < count; i++) {

    int index =
      (nextIndex - 1 - i + windowSize) % windowSize;

    weightedSum += samples[index] * weight;
    weightSum += weight;

    // Exponential decay
    weight *= (1.0f - alpha);
  }

  return weightedSum / weightSum;
}
