#include "../src/ES-02/OllieFOCdrive/TuningParameters.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#include <string>

int main() {
  float values[18] = {5,222,0.11f,0.1f,0.045f,0,0,50,4,0,0,0,0.06f,1.5f,0.0028f,2,1,0.2f};
  bindTuningParameters(&values[0],&values[1],&values[2],&values[3],&values[4],&values[5],&values[6],&values[7],&values[8],&values[9],&values[10],&values[11],&values[12],&values[13],&values[14],&values[15],&values[16],&values[17]);
  const char *names[]={"PP","PI","PD","PL","SP","SI","SD","SL","YP","YI","YD","YL","RP","RI","RD","RL","U","V"};
  for(size_t i=0;i<18;i++) assert(strcmp(tuningParameterAt(i)->name,names[i])==0);
  assert(*findTuningParameter("SP")->value==0.045f);
  char name[4]; bool hasValue=false; float value=0;
  assert(parseTuningLine(" PD0.12 ",name,sizeof(name),&hasValue,&value)); assert(!strcmp(name,"PD")&&hasValue&&fabsf(value-0.12f)<1e-6f);
  assert(parseTuningLine("V",name,sizeof(name),&hasValue,&value)&&!hasValue);
  const char duplicateValues[]="{\"request_id\":\"a\",\"expected_boot_id\":\"b\",\"values\":{\"PP\":5,\"PP\":6}}";
  const char duplicateRoot[]="{\"request_id\":\"a\",\"request_id\":\"b\",\"values\":{\"PP\":5}}";
  const char uniqueKeys[]="{\"request_id\":\"a\",\"expected_boot_id\":\"b\",\"values\":{\"PP\":5,\"PD\":0.1}}";
  assert(hasDuplicateJsonObjectKeys(duplicateValues,sizeof(duplicateValues)-1));
  assert(hasDuplicateJsonObjectKeys(duplicateRoot,sizeof(duplicateRoot)-1));
  assert(!hasDuplicateJsonObjectKeys(uniqueKeys,sizeof(uniqueKeys)-1));
  std::string fullBatch="{\"request_id\":\"0123456789abcdef0123456789abcdef\",\"expected_boot_id\":\"0123456789abcdef\",\"values\":{";
  for(size_t i=0;i<18;i++) {
    if(i) fullBatch+=',';
    fullBatch+='\"'; fullBatch+=names[i]; fullBatch+="\":0";
  }
  fullBatch+="}}";
  assert(!hasDuplicateJsonObjectKeys(fullBatch.c_str(),fullBatch.size()));
  const std::string duplicateBatch=fullBatch.substr(0,fullBatch.size()-2)+",\"PP\":5}}";
  assert(hasDuplicateJsonObjectKeys(duplicateBatch.c_str(),duplicateBatch.size()));
  const std::string longestKey="{\""+std::string(39,'k')+"\":0}";
  const std::string oversizedKey="{\""+std::string(40,'k')+"\":0}";
  assert(!hasDuplicateJsonObjectKeys(longestKey.c_str(),longestKey.size()));
  assert(hasDuplicateJsonObjectKeys(oversizedKey.c_str(),oversizedKey.size()));
  assert(parseTuningLine("SI 4.5e-1",name,sizeof(name),&hasValue,&value)&&fabsf(value-.45f)<1e-6f);
  const char *invalid[]={"PPnan","PP1tail","PP 1 2","T1","U0.5","PP1e999","PP 1.0 junk"};
  for(const char *line:invalid) assert(!parseTuningLine(line,name,sizeof(name),&hasValue,&value));
  TuningValue batch[]={{"U",1},{"PP",6},{"PD",0.12f}}; const char *error=nullptr;
  assert(validateTuningBatch(batch,3,0,&error)&&error==nullptr);
  TuningValue automatic[]={{"U",0},{"PP",5}}; assert(!validateTuningBatch(automatic,2,1,&error)&&!strcmp(error,"AUTO_MODE"));
  TuningValue independent[]={{"V",0.4f}}; assert(validateTuningBatch(independent,1,0,&error));
  TuningValue overflow[]={{"V",0.41f}}; assert(!validateTuningBatch(overflow,1,1,&error));
  TuningValue duplicate[]={{"PP",5},{"PP",6}}; assert(!validateTuningBatch(duplicate,2,1,&error));
  assert(isSbusFresh(true,249,true));assert(isSbusFresh(true,250,true));assert(!isSbusFresh(true,251,true));
  assert(!isSbusFresh(false,0,true));assert(!isSbusFresh(true,1,false));
  assert(!strcmp(validateTuningWritePolicy(false,true,true,batch,3,0),"UNSUPPORTED_MODE"));
  assert(!strcmp(validateTuningWritePolicy(true,false,true,batch,3,0),"RC_UNAVAILABLE"));
  assert(!strcmp(validateTuningWritePolicy(true,true,false,batch,3,0),"DRIVE_ACTIVE"));
  assert(validateTuningWritePolicy(true,true,true,batch,3,0)==nullptr);
}
