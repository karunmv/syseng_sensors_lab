#ifndef EXPONENTIAL_FILTER_H
#define EXPONENTIAL_FILTER_H
#include <Arduino.h>

// From tof_filter/ExponentialFilter.h; algorithm unchanged.
class ExponentialFilter {
public:
  ExponentialFilter(float alpha, int windowSize);
  float update(float measurement);
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
