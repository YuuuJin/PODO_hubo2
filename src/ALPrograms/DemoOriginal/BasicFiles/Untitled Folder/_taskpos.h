#ifndef TASKPOS_H
#define TASKPOS_H

#include "taskGeneral.h"

class TaskPos
{
public:
    explicit TaskPos();
    ~TaskPos();

public:
    float                   RefCurrent;                 // reference to move at this step
    float                   RefDelta;                    // reference of the past step
    float                   RefToGo;                    // goal position - initial position
    float                   RefInitial;                   // initial position
    unsigned long           GoalTimeCount;        // the time at which the goal is reached
    unsigned long           CurrentTimeCount;   // current time count
    char                    MoveFlag;                 // move flag

public:
    unsigned char   setTaskPos(float _mag, unsigned long _msTime, unsigned char _mode);
    unsigned char   moveTaskPos(void);
};

#endif // TASKPOS_H
