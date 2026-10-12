#ifndef __FILTER_H
#define __FILTER_H

#include <Arduino.h>

#include "MathConstants.h"


typedef struct pt1Filter_s {
    float state;
    float k;
} pt1Filter_t;

typedef enum {
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquadFilter_t;


float filterGetNotchQ(float centerFreq, float cutoffFreq);
// 2nd-order filter init
void biquadFilterUpdate(biquadFilter_t *filter, unsigned int cutoffHz, unsigned int samplingRateHz, float Q, biquadFilterType_e filterType);
// 2nd-order notch filter init
void biquadFilterInitNotch(biquadFilter_t *filter, unsigned int samplingRateHz, unsigned int centerFrequencyHz, unsigned int cutoffHz);
// 2nd-order low-pass filter init
void biquadFilterInitLPF(biquadFilter_t *filter, unsigned int cutoffHz, unsigned int samplingRateHz);
// Computes a biquad_t filter on a sample
float biquadFilterApply(biquadFilter_t *filter, float input);
void biquadFilterInit(biquadFilter_t *filter, float cutoffHz, unsigned int samplingRateHz, float Q, biquadFilterType_e filterType);
// pt1 filter gain (cutoff frequency, sample time)
float pt1FilterGain(float f_cut, float dT);
// pt1 low-pass filter init
void pt1FilterInit(pt1Filter_t *filter, float k);
// Update filter gain
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k);
// pt1 low-pass filter apply
float pt1FilterApply(pt1Filter_t *filter, float input);



#endif //__FILTER_H
