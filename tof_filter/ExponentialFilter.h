
#ifndef EXPONENTIAL_FILTER_H
#define EXPONENTIAL_FILTER_H

#include <Arduino.h>

class ExponentialFilter {
public:
  // alpha: weighting strength (0 < alpha <= 1)
  // windowSize: number of samples to retain
  ExponentialFilter(float alpha, int windowSize);

  // Add a measurement and return filtered result
  float update(float measurement);

  // Clear measurement history
  void reset();

private:
  static const int MAX_SAMPLES = 50;

  float alpha;
  int windowSize;

  float samples[MAX_SAMPLES];
  int count;
  int nextIndex;
};

#endif
