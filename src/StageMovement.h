#ifndef STAGE_H
#define STAGE_H

#include "Steering.h"

using namespace std;

enum SteeringDecode{
    movement,
    delaystage,
    unknown
};

class StageMovement{
private:
    vector<float> loadedStages;
    int loadedNumber;
    unsigned long delaystart;
    unsigned long delaytime;
    bool delaytrue;
    bool loopStages;
public:
    StageMovement();    
    void setLoopStages(bool b){loopStages=b;};
    bool getLoopStages() {return loopStages;};
    void setDelay(bool b){delaytrue=b;};
    bool getDelay() {return delaytrue;};
    void setDelayStartTime(){delaystart=millis();};
    unsigned long getDelayStartTime() {return delaystart;};
    void setDelayTime(unsigned long time){delaytime=time;};
    unsigned long getDelayTime() {return delaytime;};
    bool isLoadedNumberOk() {if(loadedStages.size()>0){if(loadedStages.at(loadedNumber)!=TERMINATOR) return true;} return false;};
    void incrementLoadedNumber() {loadedNumber++;};
    void resetLoadedNumber() {loadedNumber=0;};
    void setLoadedNumber(int l) {loadedNumber=l;};
    int getLoadedNumber() {return loadedNumber;};
    SteeringDecode decodeStage(float);
    int decodeMemoryMove(float);
    float decodeSpeed(float);
    float decodeDelayMs(float);
    void setLoadedStages(vector<float> stages){loadedStages=stages;};
    vector<float> getLoadedStages() {return loadedStages;};
    float getLoadedStage(int i) {return loadedStages.at(i);};
};


#endif // STAGE