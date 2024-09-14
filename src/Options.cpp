#include "Options.h"

Options::Options(){
    optionSet=false;
    optionChange=false; 
    optionChangeBlock=false;
    running=false;
}

void Options::changeMain(){
if(optionChange && !optionChangeBlock){

        change();

        optionSet=true;
        optionChangeBlock=true;
    }
}

RunningState Options::moveMain(){
    if(optionChange && optionChangeBlock)
    {
        running=false;

        running=move();

        if(!running)
        {  
            optionChange=false;
            optionChangeBlock=false;
            return Stop;
        }
        return Running;
    }
    return NotRunning;
}