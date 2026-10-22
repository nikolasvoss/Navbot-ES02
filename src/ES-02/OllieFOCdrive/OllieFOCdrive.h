#ifndef OllieFOCdrive_h
#define OllieFOCdrive_h

#include <Arduino.h>

#include "filter.h"

#define COMMUNICATION_OBJECT_TWO_WHEEL_BALANCE 0    // Two-wheel balance mode
#define COMMUNICATION_OBJECT_SIMPLEFOC_STUDIO 1     // SimpleFOC Studio host computer
#define COMMUNICATION_OBJECT_CONTROL_DUAL_MOTORS 2  // Control dual motors

#define ROBOT_NOT_TUMBLING 0  // Robot is not tumbling
#define ROBOT_TUMBLING 1      // Robot is tumbling

#define REMOTE_CONTROL_PID_GAINS_MODE_OFF 0
#define REMOTE_CONTROL_PID_GAINS_MODE_ON 1

class MyPIDController {
 public:
  float Kp;  // Proportional coefficient
  float Ki;  // Integral coefficient
  float Kd;  // Derivative coefficient
  float deriv;
  float integral;       // Error integral
  float previousError;  // Previous error
  float iLimit;
  float outputLimit;
  float outP;
  float outI;
  float outD;
  float output;
  float error;
  float enableDFilter;
  biquadFilter_t dFilter;  //
  float cutoffFreq;

  // Constructor, initialize PID parameters
  MyPIDController(float p, float i, float d, float iLimit, float outputLimit, float dt, float EnableDFilter,
                  float CutoffFreq) {
    Kp = p;
    Ki = i;
    Kd = d;
    this->iLimit = iLimit;
    this->outputLimit = outputLimit;
    enableDFilter = EnableDFilter;
    cutoffFreq = CutoffFreq;
    integral = 0;
    previousError = 0;
    deriv = 0;
    outP = outI = outD = output = error = 0;

    if ((int)enableDFilter == 1) {
      biquadFilterInitLPF(&dFilter, (unsigned int)cutoffFreq, (unsigned int)(1.0f / dt));
    }
  }

  // Function to calculate PID output, parameters include error and time interval dt
  float compute(float Error, float dt) {
    error = Error;
    // Calculate error integral
    integral += error * dt;

    // Integral limit
    if (iLimit != 0) {
      if (integral > iLimit) integral = iLimit;
      if (integral < (-iLimit)) integral = -iLimit;
    }

    // Calculate error derivative
    deriv = (error - previousError) / dt;
    if ((int)enableDFilter == 1) {
      deriv = biquadFilterApply(&dFilter, deriv);
    }

    outP = Kp * error;
    outI = Ki * integral;
    outD = Kd * deriv;

    // Calculate PID output
    output = outP + outI + outD;

    // Update previous error
    previousError = error;

    // Output limit
    if (outputLimit != 0) {
      output = constrain(output, -outputLimit, outputLimit);
    }

    return output;
  }
};

#endif
