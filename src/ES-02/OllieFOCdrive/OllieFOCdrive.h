#ifndef OllieFOCdrive_h
#define OllieFOCdrive_h

#include <Arduino.h>
#include "filter.h"
#include "touchscreen.h"

#define ADJUSTMENT_PARAM_BALANCE_SPEED_YAW_ROLL 0
#define ADJUSTMENT_PARAM_BALL_PUSHING 1 // For tuning main balance and movement PID loops

#define COMMUNICATION_OBJECT_TWO_WHEEL_BALANCE 0 // Two-wheel balance mode
#define COMMUNICATION_OBJECT_SIMPLEFOC_STUDIO 1          // SimpleFOC Studio host computer
#define COMMUNICATION_OBJECT_CONTROL_DUAL_MOTORS 2       // Control dual motors

#define ROBOT_NOT_TUMBLING 0  // Robot is not tumbling
#define ROBOT_TUMBLING 1 // Robot is tumbling

#define REMOTE_CONTROL_PID_GAINS_MODE_OFF 0
#define REMOTE_CONTROL_PID_GAINS_MODE_ON_WITHOUT_TOUCH 1
#define REMOTE_CONTROL_PID_GAINS_MODE_ON_WITH_TOUCH 2

#define REMOTE_CONTROL_PM_POSTURE_MODE 0 // Posture control mode
#define REMOTE_CONTROL_PM_MARK_MODE 1    // Mark control mode

#define REMOTE_CONTROL_ROLL_MODE_MANUAL 0
#define REMOTE_CONTROL_ROLL_MODE_AUTO 1

#define REMOTE_CONTROL_ATTITUDE_MODE_DEFAULT 0
#define REMOTE_CONTROL_ATTITUDE_MODE_PITCHING_ADJUST 1 // Pitching adjustment mode
#define REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE 2      // Ball poise mode

typedef union 
{
  struct 
  {
    float x;
    float y;
    float z;
  };
  float axis[3];
} Axis3f;

//Attitude data structure
typedef struct  
{
  Axis3f accf;       //Filtered acceleration (G)
  Axis3f gyrof;      //Filtered gyroscope (deg/s)  
  Axis3f acc;       //Acceleration (G)
  Axis3f gyro;      //Gyroscope (deg/s)  
  float roll;
  float pitch;
  float yaw;
  float temp;
} attitude_t;


typedef struct  
{
  Axis3f acc;       //Acceleration (G)
  Axis3f gyro;      //Gyroscope (deg/s) 
  float roll;
  float pitch;
  float yaw;

  float servo1;
  float servo2;
  float servo3;
  float servo4;

  
} zeroBias_t;

class MyPIDController {
  private:


  public:
  
    float Kp;  // Proportional coefficient
    float Ki;  // Integral coefficient
    float Kd;  // Derivative coefficient
    float deriv;
    float integral;  // Error integral
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
    MyPIDController(float p, float i, float d, float iLimit, float outputLimit,float dt, float EnableDFilter, float CutoffFreq) {
      Kp = p;
      Ki = i;
      Kd = d;
      enableDFilter = EnableDFilter;
      integral = iLimit;
      previousError = outputLimit;//Output limit
      cutoffFreq = CutoffFreq;
      
      if ((int)enableDFilter==1)
      {
        biquadFilterInitLPF(&dFilter, (unsigned int)cutoffFreq, (unsigned int)(1.0f / dt));
      }      
      
    }

    // Function to calculate PID output, parameters include error and time interval dt
    float compute(float Error, float dt) {

      error = Error;
      // Calculate error integral
      integral += error * dt;

      //Integral limit
      if (iLimit != 0)
      {
        if(integral>iLimit)
          integral = iLimit;
        if(integral<(-iLimit))
          integral = -iLimit;
        
      }

      // Calculate error derivative
      deriv = (error - previousError) / dt;
      if (enableDFilter==1)
      {
        deriv = biquadFilterApply(&dFilter, deriv);
      }

      
      outP = Kp * error;
      outI = Ki * integral;
      outD = Kd * deriv;


      // Calculate PID output
      output = outP + outI + outD;

      // Update previous error
      previousError = error;

      //Output limit
      if (outputLimit != 0)
      {
        output = constrain(output, -outputLimit, outputLimit);
      }
  

      return output;
    }

    // Function to set PID coefficients
    void setPID(float p, float i, float d, float iLimit, float outputLimit,float dt, float EnableDFilter, float CutoffFreq) {
      Kp = p;
      Ki = i;
      Kd = d;
      enableDFilter = EnableDFilter;
      integral = iLimit;
      previousError = outputLimit;//Output limit
      cutoffFreq = CutoffFreq;
      
      if ((int)enableDFilter)
      {
        biquadFilterInitLPF(&dFilter, (unsigned int)cutoffFreq, (unsigned int)(1.0f / dt));
      }    
    }
};




#endif
