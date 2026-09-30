#include "TuningParameters.h"
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdint.h>

static TuningParameter parameters[TUNING_PARAMETER_COUNT] = {
  {"PP", 0, 20, "Angle proportional gain", nullptr, false},
  {"PI", 0, 500, "Angle integral gain", nullptr, false},
  {"PD", 0, 2, "Angle derivative gain", nullptr, false},
  {"PL", 0, 1, "Angle integral limit; mode and scaling dependent", nullptr, false},
  {"SP", 0, 2, "Speed proportional gain", nullptr, false},
  {"SI", 0, 5, "Speed integral gain", nullptr, false},
  {"SD", 0, 2, "Speed derivative gain", nullptr, false},
  {"SL", 0, 100, "Speed integral limit; mode and scaling dependent", nullptr, false},
  {"YP", 0, 200, "Yaw proportional gain", nullptr, false},
  {"YI", 0, 100, "Yaw integral gain", nullptr, false},
  {"YD", 0, 5, "Yaw derivative gain", nullptr, false},
  {"YL", 0, 100, "Yaw integral limit; mode and scaling dependent", nullptr, false},
  {"RP", 0, 10, "Roll proportional gain", nullptr, false},
  {"RI", 0, 100, "Roll integral gain", nullptr, false},
  {"RD", 0, 5, "Roll derivative gain", nullptr, false},
  {"RL", 0, 100, "Roll integral limit; mode and scaling dependent", nullptr, false},
  {"U", 0, 1, "Manual tuning mode (1) or automatic gains (0)", nullptr, true},
  {"V", 0, 0.4f, "Wheel speed feedback gain", nullptr, false}
};

void bindTuningParameters(float *aP, float *aI, float *aD, float *aL,
                          float *sP, float *sI, float *sD, float *sL,
                          float *yP, float *yI, float *yD, float *yL,
                          float *rP, float *rI, float *rD, float *rL,
                          float *mode, float *wheel) {
  float *targets[TUNING_PARAMETER_COUNT] = {};
  targets[0]=aP; targets[1]=aI; targets[2]=aD; targets[3]=aL;
  targets[4]=sP; targets[5]=sI; targets[6]=sD; targets[7]=sL;
  targets[8]=yP; targets[9]=yI; targets[10]=yD; targets[11]=yL;
  targets[12]=rP; targets[13]=rI; targets[14]=rD; targets[15]=rL;
  targets[16]=mode; targets[17]=wheel;
  for (size_t i=0;i<TUNING_PARAMETER_COUNT;i++) parameters[i].value=targets[i];
}

const TuningParameter *tuningParameterAt(size_t index) {
  return index < TUNING_PARAMETER_COUNT ? &parameters[index] : nullptr;
}
const TuningParameter *findTuningParameter(const char *name) {
  if (!name) return nullptr;
  for (size_t i=0;i<TUNING_PARAMETER_COUNT;i++) if (strcmp(name,parameters[i].name)==0) return &parameters[i];
  return nullptr;
}

bool isSbusFresh(bool hasFrame,unsigned long ageMs,bool failsafeOk) {
  return hasFrame && ageMs <= 250 && failsafeOk;
}

const char *validateTuningWritePolicy(bool supportedMode,bool rcValid,bool ch5Off,const TuningValue *values,size_t count,float currentMode) {
  if(!supportedMode)return "UNSUPPORTED_MODE";
  if(!rcValid)return "RC_UNAVAILABLE";
  if(!ch5Off)return "DRIVE_ACTIVE";
  const char *error=nullptr;
  if(!validateTuningBatch(values,count,currentMode,&error))return error?error:"INVALID_VALUE";
  return nullptr;
}

bool hasDuplicateJsonObjectKeys(const char *body,size_t length) {
  if(!body)return true;
  if(length>0xffffU)return true;
  uint16_t keyOffsets[5][24] = {};
  uint8_t keyLengths[5][24] = {};
  unsigned char counts[5]={0,0,0,0,0};int depth=0;
  for(size_t i=0;i<length;i++){
    char c=body[i];
    if(c=='"'){
      const size_t keyStart=i+1;size_t j=keyStart;bool escaped=false;
      for(;j<length&&body[j]!='"';j++){
        if(body[j]=='\\'){escaped=true;break;}
        if(j-keyStart>=39)return true;
      }
      if(escaped){size_t k=j+1;bool inEscape=false;for(;k<length;k++){if(inEscape){inEscape=false;continue;}if(body[k]=='\\'){inEscape=true;continue;}if(body[k]=='"')break;}if(k>=length)return true;j=k;}
      if(j>=length)return true;
      const size_t keyLength=j-keyStart;
      size_t k=j+1;while(k<length&&isspace((unsigned char)body[k]))k++;
      if(k<length&&body[k]==':'){
        if(escaped||depth<1||depth>4||counts[depth]>=24)return true;
        for(unsigned char q=0;q<counts[depth];q++){
          if(keyLengths[depth][q]==keyLength&&
             memcmp(body+keyOffsets[depth][q],body+keyStart,keyLength)==0)return true;
        }
        keyOffsets[depth][counts[depth]]=(uint16_t)keyStart;
        keyLengths[depth][counts[depth]++]=(uint8_t)keyLength;
      }
      i=j;
    } else if(c=='{'){if(++depth>4)return true;counts[depth]=0;}
    else if(c=='}'){if(depth<1)return true;depth--;}
  }
  return depth!=0;
}

bool parseTuningLine(const char *input, char *nameOut, size_t cap, bool *hasValue, float *valueOut) {
  if (!input || !nameOut || cap < 2 || !hasValue || !valueOut) return false;
  while (isspace((unsigned char)*input)) input++;
  const char *end=input+strlen(input);
  while (end>input && isspace((unsigned char)end[-1])) end--;
  if (end==input || (size_t)(end-input)>48) return false;
  const char *p=input;
  while (p<end && isalpha((unsigned char)*p)) p++;
  size_t nameLen=(size_t)(p-input);
  if (!nameLen || nameLen>=cap) return false;
  memcpy(nameOut,input,nameLen); nameOut[nameLen]='\0';
  const TuningParameter *parameter=findTuningParameter(nameOut);
  if (!parameter) return false;
  if (p==end) { *hasValue=false; return true; }
  if (isspace((unsigned char)*p)) while (p<end && isspace((unsigned char)*p)) p++;
  if (p==end) { *hasValue=false; return true; }
  const char *number=p;
  if (*p=='+' || *p=='-') p++;
  bool digits=false;
  while (p<end && isdigit((unsigned char)*p)) { digits=true; p++; }
  if (p<end && *p=='.') { p++; while (p<end && isdigit((unsigned char)*p)) {digits=true;p++;} }
  if (!digits) return false;
  if (p<end && (*p=='e'||*p=='E')) { p++; if (p<end&&(*p=='+'||*p=='-'))p++; const char *exp=p; while(p<end&&isdigit((unsigned char)*p))p++; if(p==exp)return false; }
  if (p!=end) return false;
  char buf[40]; size_t n=(size_t)(end-number); if(n>=sizeof(buf)) return false;
  memcpy(buf,number,n); buf[n]='\0'; char *parsedEnd=nullptr; double value=strtod(buf,&parsedEnd);
  if (parsedEnd!=buf+n || !isfinite(value) || value>3.402823466e38 || value< -3.402823466e38) return false;
  *valueOut=(float)value; if (!isfinite(*valueOut) || *valueOut<parameter->minimum || *valueOut>parameter->maximum || (parameter->isMode && *valueOut!=0.0f && *valueOut!=1.0f)) return false;
  *hasValue=true; return true;
}

bool validateTuningBatch(const TuningValue *values, size_t count, float currentMode, const char **errorCode) {
  if (errorCode) *errorCode="INVALID_VALUE";
  if (!values || count==0 || count>TUNING_PARAMETER_COUNT) return false;
  bool containsGain=false; float resultMode=currentMode;
  for(size_t i=0;i<count;i++) {
    const TuningParameter *p=findTuningParameter(values[i].name);
    if(!p || !isfinite(values[i].value) || values[i].value<p->minimum || values[i].value>p->maximum || (p->isMode && values[i].value!=0.0f && values[i].value!=1.0f)) return false;
    for(size_t j=0;j<i;j++) if(strcmp(values[j].name,values[i].name)==0) return false;
    if(p->isMode) resultMode=values[i].value; else if(strcmp(p->name,"V")!=0) containsGain=true;
  }
  if(containsGain && resultMode==0.0f) {if(errorCode)*errorCode="AUTO_MODE";return false;}
  if(errorCode)*errorCode=nullptr;
  return true;
}
