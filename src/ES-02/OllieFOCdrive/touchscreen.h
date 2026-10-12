#ifndef __touchscreen_H
#define __touchscreen_H

#include <Arduino.h>
#include "filter.h"

typedef struct
{
  uint16_t XPressDat;
  uint16_t YPressDat;
  int16_t XPdat;
  int16_t YPdat;
  int16_t XLdat;
  int16_t YLdat;
  float XPdatF;
  float YPdatF;
  uint8_t state;
  uint8_t rxbuf[10];
  int P_count;
  int L_count;
  int start;
} Touch_t;


void TouchscreenInit(unsigned int samplingRateHz);
void ReadTouchDat(void);
void TouchBiquadFilter(void);

extern Touch_t Touch;

#endif //
