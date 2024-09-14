#ifndef OPTIONS_H
#define OPTIONS_H

#include "RunningState.h"

using namespace std;

class Options{
protected:
    bool optionSet;
    bool optionChange; 
    bool optionChangeBlock;
    bool running;
    
public: 
    Options();
    virtual ~Options() {};
    bool getSet() { return optionSet; }
    bool getChange() { return optionChange; }
    bool getChangeBlock() { return optionChangeBlock; }
    void setChange(bool OptionChange) {optionChange = OptionChange;}
    bool isSet() {return optionSet;}
    void changeMain();
    RunningState moveMain();
    virtual void change() = 0;
    virtual bool move() = 0;
};

#endif // OPTIONS