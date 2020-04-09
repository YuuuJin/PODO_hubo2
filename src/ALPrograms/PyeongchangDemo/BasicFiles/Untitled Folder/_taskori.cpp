#include "taskori.h"

TaskOri::TaskOri()
{
    for(int i=0; i<5; i++)
    {
        RefCurrent[i] = 0.f;
        RefDelta[i] = 0.f;
        RefDeltaQT[i] = 0.f;
        RefdqQT[i] = 0.f;
        UserInput[i] = 0.f;
        RefInitial[i] = 0.f;
        RefInitialInv[i] = 0.f;
    }
    for(int i=0; i<4; i++)
    {
        RefDeltaRV[i] = 0.f;
        RefdqRV[i] = 0.f;
        TempToGo[i] = 0.f;
    }
    GoalTimeCount = 0;
    CurrentTimeCount = 0;
    MoveFlag = DISABLE;
}

unsigned char TaskOri::setTaskOri(double _mag[5], unsigned long _msTime, unsigned char _mode)
{
    if(_msTime <= 0) { printf(">>> Goal time must be grater than zero..!!(setTaskOri)\n"); return ERR_GOAL_TIME; }

    if(MoveFlag == ENABLE)
    {
        printf(">>> Moving..!!(setTaskOri)\n");
        return ERR_ALREADY_MOVING;
    }
    else
    {
        MoveFlag = DISABLE;

        TempToGo[0] = _mag[1];
        TempToGo[1] = _mag[2];
        TempToGo[2] = _mag[3];
        TempToGo[3] = _mag[4];

        RefInitial[1] = RefCurrent[1];
        RefInitial[2] = RefCurrent[2];
        RefInitial[3] = RefCurrent[3];
        RefInitial[4] = RefCurrent[4];

        switch(_mode)
        {
        case MODE_RELATIVE: //relative mode
            RV2QT(TempToGo, UserInput);

            RefDeltaQT[1] = UserInput[1];
            RefDeltaQT[2] = UserInput[2];
            RefDeltaQT[3] = UserInput[3];
            RefDeltaQT[4] = UserInput[4];
            break;
        case MODE_ABSOLUTE: //absolute mode
            RV2QT(TempToGo, UserInput);
            QTinv(RefInitial, RefInitialInv);
            QTcross(RefInitialInv, UserInput, RefDeltaQT);
            break;
        default:
            printf(">>> Wrong reference mode..!!(setTaskOri)\n");
            return ERR_WRONG_MODE;
            break;
        }

        CurrentTimeCount = 0;
        GoalTimeCount = ((unsigned long)(_msTime))/RT_TIMER_PERIOD_MS;
        MoveFlag = ENABLE;
    }
    return ERR_OK;
}

unsigned char TaskOri::moveTaskOri(void)
{
    float tempTime;

    if(MoveFlag == ENABLE)
    {
        CurrentTimeCount++;
        if(GoalTimeCount <= CurrentTimeCount)
        {
            GoalTimeCount = CurrentTimeCount = 0;
            QTcross(RefInitial, RefDeltaQT, RefCurrent);
            MoveFlag = DISABLE;
            return MOVE_DONE;
        }
        else
        {
            QT2RV(RefDeltaQT, RefDeltaRV);

            tempTime = (float)CurrentTimeCount/(float)GoalTimeCount;
            RefdqRV[0] = RefDeltaRV[0]*0.5f*(1.0f-cosf(PI*tempTime));
            RefdqRV[1] = RefDeltaRV[1];
            RefdqRV[2] = RefDeltaRV[2];
            RefdqRV[3] = RefDeltaRV[3];

            RV2QT(RefdqRV, RefdqQT);
            QTcross(RefInitial, RefdqQT, RefCurrent);
        }
    }
    return STILL_MOVING;
}


int TaskOri::QTinv(const double *qt_4x1, double *result_4x1)
{
    result_4x1[1] = qt_4x1[1]/((qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3])+(qt_4x1[4])*(qt_4x1[4]));
    result_4x1[2] = -qt_4x1[2]/((qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3])+(qt_4x1[4])*(qt_4x1[4]));
    result_4x1[3] = -qt_4x1[3]/((qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3])+(qt_4x1[4])*(qt_4x1[4]));
    result_4x1[4] = -qt_4x1[4]/((qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3])+(qt_4x1[4])*(qt_4x1[4]));

    return 0;
}
