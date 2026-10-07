#include "filter.h"

#include <cmath>
#include <cstdio>

int main() {
  biquadFilter_t filter{};
  biquadFilterInitLPF(&filter, 20, 100);

  constexpr double pi = 3.14159265358979323846;
  constexpr int warmupSamples = 1000;
  constexpr int measuredSamples = 5000;
  double sumSin = 0.0;
  double sumCos = 0.0;

  for (int sample = 0; sample < warmupSamples + measuredSamples; ++sample) {
    const double phase = 2.0 * pi * 20.0 * sample / 100.0;
    const float input = static_cast<float>(std::sin(phase));
    const float output = biquadFilterApply(&filter, input);

    if (sample >= warmupSamples) {
      sumSin += output * std::sin(phase);
      sumCos += output * std::cos(phase);
    }
  }

  const double inPhase = 2.0 * sumSin / measuredSamples;
  const double quadrature = 2.0 * sumCos / measuredSamples;
  const double gain = std::sqrt(inPhase * inPhase + quadrature * quadrature);
  constexpr double expectedCutoffGain = 0.7071067811865476;
  constexpr double tolerance = 0.03;

  std::printf("20 Hz at 100 Hz: gain %.6f (expected %.6f +/- %.3f)\n",
              gain, expectedCutoffGain, tolerance);
  if (std::abs(gain - expectedCutoffGain) > tolerance) {
    return 1;
  }

  return 0;
}
