#ifndef OFFSET_H
#define OFFSET_H

#include <AccelStepper.h>
#include <vector>
#include "Joint.h"
#include "RunningState.h"

using namespace std;

class Offset{
private:
    bool offsetSet;
    bool offsetChange; 
    bool offsetChangeBlock;

public:
    Offset();
    bool getOffsetSet() { return offsetSet; }
    bool getOffsetChange() { return offsetChange; }
    bool getOffsetChangeBlock() { return offsetChangeBlock; }
    void setOffsetChange(bool OffsetChange) { offsetChange = OffsetChange; }
    void setOffsetChangeBlock(bool OffsetChangeBlock) { offsetChangeBlock = OffsetChangeBlock; }
    void OffsetChange(const vector<AccelStepper*>&, const vector<Joint*>&);
    RunningState OffsetMove(const vector<AccelStepper*>&);
};

#endif 