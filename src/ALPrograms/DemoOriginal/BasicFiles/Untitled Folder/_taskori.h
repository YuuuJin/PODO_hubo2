#ifndef TASKORI_H
#define TASKORI_H

#include "taskGeneral.h"

class TaskOri
{
public:
    explicit TaskOri();
    ~TaskOri();

public:
    double                   RefCurrent[5];              // reference pattern to move at this step
    double                   RefDelta[5];                  // reference pattern of the past step
    double                   RefDeltaQT[5];
    double                   UserInput[5];
    double                   RefInitial[5];                  // initial position
    double                   RefInitialInv[5];
    double                   RefdqQT[5];

    double                   RefDeltaRV[4];
    double                   RefdqRV[4];
    double                   TempToGo[4];

    unsigned long	  GoalTimeCount;            // the time at which the goal is reached
    unsigned long	  CurrentTimeCount;       // current time count
    unsigned char	  MoveFlag;                     // move flag

public:
    unsigned char   setTaskOri(double _mag[5], unsigned long _msTime, unsigned char _mode);
    unsigned char   moveTaskOri(void);
    int             QTinv(const double *qt_4x1, double *result_4x1);
};

#endif // TASKORI_H
