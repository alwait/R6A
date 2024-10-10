#ifndef STAGE_H
#define STAGE_H

#include "Steering.h"

using namespace std;

class StageMovement{
private:
    vector<float> loadedStages;
    int loadedNumber;
    bool loopStages;
public:
    StageMovement();    
    void setLoopStages(bool b){loopStages=b;};
    bool getLoopStages() {return loopStages;};
    void setLoadedNumber(int l) {loadedNumber=l;};
    int getLoadedNumber() {return loadedNumber;};
    void setLoadedStages(vector<float> stages){loadedStages=stages;};
    vector<float> getLoadedStages() {return loadedStages;};
};


#endif // STAGE